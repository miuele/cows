#include <cstdio>
#include <chrono>
#include <cstdarg>
#include <thread>
#include <charconv>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cogtwa_interfaces/srv/od_read.hpp"
#include "cogtwa_interfaces/srv/od_write.hpp"
#include "cogtwa_interfaces/srv/sdo_read.hpp"
#include "cogtwa_interfaces/srv/sdo_write.hpp"
#include "cogtwa_interfaces/srv/configure_od_notify.hpp"
#include "cogtwa_interfaces/msg/notify_od_write.hpp"

#include <syslog.h>

#include "CANopen.h"
extern "C" {
#include "OD.h"
}
#include "CO_error.h"
#include "CO_epoll_interface.h"
#include "CO_storageLinux.h"

#include "cogtwa/CO_utils.hpp"

#define MAIN_THREAD_INTERVAL_US 100'000
#define NMT_CONTROL (static_cast<CO_NMT_control_t>( \
			CO_NMT_STARTUP_TO_OPERATIONAL \
			| CO_NMT_ERR_ON_ERR_REG \
			| CO_ERR_REG_GENERIC_ERR \
			| CO_ERR_REG_COMMUNICATION))

#define FIRST_HB_TIME 500
#define SDO_SRV_TIMEOUT_TIME 1000
#define SDO_CLI_TIMEOUT_TIME 500

#ifndef CO_ERROR_MSG_MAX_LEN
#define CO_ERROR_MSG_MAX_LEN 128
#endif


using namespace std::chrono_literals;
using namespace std::string_literals;

std::string notify_od_write_topic_name(std::uint16_t index) {
    char a[8]{};
    std::to_chars(std::begin(a), std::end(a), index, 16);
    return "notify_od_write_0x"s + a;
}

struct ODEntryHook {
    std::shared_ptr<rclcpp::Publisher<cogtwa_interfaces::msg::NotifyODWrite>> notify_od_write_publisher;
    OD_extension_t od_extension{};
};

void log_printf(int priority, const char *format, ...) {
    char buffer[CO_ERROR_MSG_MAX_LEN];
	std::va_list ap;

    va_start(ap, format);
	vsnprintf(buffer, std::size(buffer), format, ap);
    va_end(ap);
    rclcpp::Logger logger = rclcpp::get_logger("CANopen");

    if (priority == LOG_DEBUG) {
        RCLCPP_DEBUG(logger, "%s", buffer);
    } else if (priority == LOG_INFO || priority == LOG_NOTICE) {
        RCLCPP_INFO(logger, "%s", buffer);
    } else if (priority == LOG_WARNING) {
        RCLCPP_WARN(logger, "%s", buffer);
    } else if (priority == LOG_ERR || priority == LOG_CRIT) {
        RCLCPP_ERROR(logger, "%s", buffer);
    } else {
        RCLCPP_FATAL(logger, "%s", buffer);
    }
}

class COGtwa
    : public rclcpp::Node
{
public:
    COGtwa(CO_t *CO, OD_t *OD)
        : Node("cogtwa")
    {
        sdo_callback_group_ =
            this->create_callback_group(
                    rclcpp::CallbackGroupType::MutuallyExclusive);

        publisher_ =
            this->create_publisher<std_msgs::msg::String>(
                    "od2110_written", 10);

        service_od_read_ =
            this->create_service<cogtwa_interfaces::srv::ODRead>(
                    "od_read",
                    [CO, OD](
                        const std::shared_ptr<cogtwa_interfaces::srv::ODRead::Request> req,
                        std::shared_ptr<cogtwa_interfaces::srv::ODRead::Response> resp
                      ) {
                        resp->data.resize(req->size);

                        CO_LOCK_OD(CO->CANmodule);
                        resp->ok = OD_get_value(OD_find(OD, req->index), req->sub_index, resp->data.data(), req->size, false)
                            == ODR_OK;
                        CO_UNLOCK_OD(CO->CANmodule);
                    });

        service_od_write_ =
            this->create_service<cogtwa_interfaces::srv::ODWrite>(
                    "od_write",
                    [CO, OD](
                        const std::shared_ptr<cogtwa_interfaces::srv::ODWrite::Request> req,
                        std::shared_ptr<cogtwa_interfaces::srv::ODWrite::Response> resp
                      ) {
                        CO_LOCK_OD(CO->CANmodule);
                        resp->ok = OD_set_value(OD_find(OD, req->index), req->sub_index, req->data.data(), req->data.size(), false)
                            == ODR_OK;
                        CO_UNLOCK_OD(CO->CANmodule);
                    });

        service_sdo_read_ =
            this->create_service<cogtwa_interfaces::srv::SDORead>(
                    "sdo_read",
                    [CO, OD](
                        const std::shared_ptr<cogtwa_interfaces::srv::SDORead::Request> req,
                        std::shared_ptr<cogtwa_interfaces::srv::SDORead::Response> resp
                        ) {
                        resp->data.resize(req->size);

                        CO_SDOclient_t *const sdoc = CO->SDOclient;
                        std::size_t size_read;
                        resp->ok = read_SDO(
                                sdoc,
                                req->node_id,
                                req->index,
                                req->sub_index,
                                resp->data.data(),
                                req->size,
                                &size_read) == CO_SDO_AB_NONE;
                    }, rmw_qos_profile_services_default, sdo_callback_group_);

        service_sdo_write_ =
            this->create_service<cogtwa_interfaces::srv::SDOWrite>(
                    "sdo_write",
                    [CO, OD](
                        const std::shared_ptr<cogtwa_interfaces::srv::SDOWrite::Request> req,
                        std::shared_ptr<cogtwa_interfaces::srv::SDOWrite::Response> resp
                        ) {
                        CO_SDOclient_t *const sdoc = CO->SDOclient;
                        resp->ok = write_SDO(
                                sdoc,
                                req->node_id,
                                req->index,
                                req->sub_index,
                                req->data.data(),
                                req->data.size()) == CO_SDO_AB_NONE;
                    }, rmw_qos_profile_services_default, sdo_callback_group_);

        service_configure_od_notify_ =
            this->create_service<cogtwa_interfaces::srv::ConfigureODNotify>(
                    "configure_od_notify",
                    [this, CO, OD](
                        const std::shared_ptr<cogtwa_interfaces::srv::ConfigureODNotify::Request> req,
                        std::shared_ptr<cogtwa_interfaces::srv::ConfigureODNotify::Response> resp
                        ) {
                        
                        auto topic_name = notify_od_write_topic_name(req->index);
                        auto it = od_hooks_.try_emplace(topic_name, std::make_unique<ODEntryHook>()).first;
                        it->second->notify_od_write_publisher = this->create_publisher<cogtwa_interfaces::msg::NotifyODWrite>(topic_name, 10);
                        it->second->od_extension.object = &*it->second;
                        it->second->od_extension.read = OD_readOriginal;
                        it->second->od_extension.write = [](OD_stream_t *stream, const void *buf, OD_size_t size, OD_size_t *size_written) {
                                cogtwa_interfaces::msg::NotifyODWrite msg;
                                msg.sub_index = stream->subIndex;
                                msg.data.resize(size);
                                std::memcpy(msg.data.data(), buf, size);
                                static_cast<ODEntryHook *>(stream->object)->notify_od_write_publisher->publish(std::move(msg));
                                return OD_writeOriginal(stream, buf, size, size_written);
                            };
                        OD_extension_init(OD_find(OD, req->index), &it->second->od_extension);
                        resp->topic_name = topic_name;
                    });

        timer_ =
            this->create_wall_timer(500ms, [&pub = publisher_]{
                    std_msgs::msg::String msg;
                    msg.data = "Hello, world!";
                    pub->publish(msg);
                });
    }
private:
    std::shared_ptr<rclcpp::CallbackGroup> sdo_callback_group_;

    std::shared_ptr<
        rclcpp::Publisher<std_msgs::msg::String>> publisher_;

    std::shared_ptr<
        rclcpp::Service<cogtwa_interfaces::srv::ODRead>> service_od_read_;

    std::shared_ptr<
        rclcpp::Service<cogtwa_interfaces::srv::ODWrite>> service_od_write_;

    std::shared_ptr<
        rclcpp::Service<cogtwa_interfaces::srv::SDORead>> service_sdo_read_;

    std::shared_ptr<
        rclcpp::Service<cogtwa_interfaces::srv::SDOWrite>> service_sdo_write_;

    std::shared_ptr<
        rclcpp::Service<cogtwa_interfaces::srv::ConfigureODNotify>> service_configure_od_notify_;

    std::unordered_map<std::string, std::unique_ptr<ODEntryHook>> od_hooks_;

    std::shared_ptr<rclcpp::TimerBase> timer_;
};

using namespace std::chrono_literals;

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    CO_CANptrSocketCan_t CANptr{};

    CANptr.can_ifindex = if_nametoindex("vcan0");
    if (CANptr.can_ifindex == 0) {
        log_printf(LOG_CRIT, DBG_NO_CAN_DEVICE, "vcan0");
        std::exit(EXIT_FAILURE);
    }

	std::uint32_t heap_used = 0;
	CO_t *CO = CO_new(nullptr, &heap_used);
	if (CO == nullptr) {
		log_printf(LOG_CRIT, DBG_GENERAL, "CO_new(), heap_used=", heap_used);
		std::exit(EXIT_FAILURE);
	}

	CO_epoll_t ep_main;
    {
        CO_ReturnError_t err = CO_epoll_create(&ep_main, MAIN_THREAD_INTERVAL_US);
        if (err != CO_ERROR_NO) {
            log_printf(LOG_CRIT, DBG_GENERAL, "CO_epoll_create(main), err=", err);
            std::exit(EXIT_FAILURE);
        }
    }

    int app_eventfd = eventfd(0, EFD_NONBLOCK);
    struct epoll_event ev{};
    ev.events = EPOLLIN;
    ev.data.fd = app_eventfd;
    epoll_ctl(ep_main.epoll_fd, EPOLL_CTL_ADD, ev.data.fd, &ev);

	CANptr.epoll_fd = ep_main.epoll_fd;

    CO_epoll_gtw_t ep_gtw;
    {
        static char socket_path[] = "/tmp/CO_command_socket";
        CO_ReturnError_t err = CO_epoll_createGtw(
                &ep_gtw,
                ep_main.epoll_fd,
                CO_COMMAND_IF_LOCAL_SOCKET,
                1000,
                socket_path);
        if (err != CO_ERROR_NO) {
            log_printf(LOG_CRIT, DBG_GENERAL, "CO_epoll_createGtw(), err=", err);
            std::exit(EXIT_FAILURE);
        }
    }

	{
		CO_CANsetConfigurationMode(&CANptr);
		CO_CANmodule_disable(CO->CANmodule);
		CO_ReturnError_t err = CO_CANinit(CO, &CANptr, 0);
		if (err != CO_ERROR_NO) {
			log_printf(LOG_CRIT, DBG_CAN_OPEN, "CO_CANinit()", err);
			std::exit(EXIT_FAILURE);
		}
	}

	{
		std::uint32_t err_info = 0;
		CO_ReturnError_t err = CO_CANopenInit(
				CO, nullptr, nullptr, OD, nullptr, 
				NMT_CONTROL,
				FIRST_HB_TIME,
				SDO_SRV_TIMEOUT_TIME,
				SDO_CLI_TIMEOUT_TIME,
				false,
				3,
				&err_info);
		if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
			if (err == CO_ERROR_OD_PARAMETERS) {
				log_printf(LOG_CRIT, DBG_OD_ENTRY, err_info);
			} else {
				log_printf(LOG_CRIT, DBG_CAN_OPEN, "CO_CANopenInit()", err);
			}
			std::exit(EXIT_FAILURE);
		}
	}

	CO_CANsetNormalMode(CO->CANmodule);

	CO_epoll_initCANopenMain(&ep_main, CO);
    CO_epoll_initCANopenGtw(&ep_gtw, CO);

	{
		std::uint32_t err_info = 0;
		CO_ReturnError_t err = CO_CANopenInitPDO(CO, CO->em, OD, 3, &err_info);
		if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
			if (err == CO_ERROR_OD_PARAMETERS) {
				log_printf(LOG_CRIT, DBG_OD_ENTRY, err_info);
			} else {
				log_printf(LOG_CRIT, DBG_CAN_OPEN, "CO_CANopenInit()", err);
			}
			std::exit(EXIT_FAILURE);
		}
	}

    std::atomic<bool> should_die{false};

    std::thread co_rtml_thread([&] {
            CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
            while (reset == CO_RESET_NOT) {
                CO_epoll_wait(&ep_main);
                if (should_die.load()) {
                    break;
                }
                CO_epoll_processRT(&ep_main, CO, false);
                CO_epoll_processMain(&ep_main, CO, true, &reset);
                CO_epoll_processGtw(&ep_gtw, CO, &ep_main);
                CO_epoll_processLast(&ep_main);
            }
        });

    auto node = std::make_shared<COGtwa>(CO, OD);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node); 

    RCLCPP_INFO(rclcpp::get_logger("cogtwa"), "start spinning...");

    executor.spin();

    should_die.store(true);

    uint64_t u = 1;
    write(ep_main.event_fd, &u, sizeof u);

    co_rtml_thread.join();

	CO_epoll_close(&ep_main);
    CO_epoll_closeGtw(&ep_gtw);

	CO_CANsetConfigurationMode(&CANptr);

	CO_delete(CO);

    rclcpp::shutdown();
}
