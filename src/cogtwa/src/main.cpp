#include <cstdio>
#include <chrono>
#include <cstdarg>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cogtwa_interfaces/srv/od_read_u16.hpp"
#include "cogtwa_interfaces/srv/sdo_read_u16.hpp"

#include <syslog.h>

#include "CANopen.h"
#include "OD.h"
#include "CO_error.h"
#include "CO_epoll_interface.h"
#include "CO_storageLinux.h"

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

        service_od_read_u16_ =
            this->create_service<cogtwa_interfaces::srv::ODReadU16>(
                    "od_read_u16",
                    [CO, OD](
                        const std::shared_ptr<cogtwa_interfaces::srv::ODReadU16::Request> req,
                        std::shared_ptr<cogtwa_interfaces::srv::ODReadU16::Response> resp
                      ) {
                        CO_LOCK_OD(CO->CANmodule);
                        OD_get_u16(OD_find(OD, req->index), req->sub_index, &resp->value, false);
                        CO_UNLOCK_OD(CO->CANmodule);
                    });
        
        service_sdo_read_u16_ =
            this->create_service<cogtwa_interfaces::srv::SDOReadU16>(
                    "sdo_read_u16",
                    [CO, OD](
                        const std::shared_ptr<cogtwa_interfaces::srv::SDOReadU16::Request> req,
                        std::shared_ptr<cogtwa_interfaces::srv::SDOReadU16::Response> resp
                        ) {
                        CO_SDOclient_t *const sdoc = CO->SDOclient;

                        if (CO_SDOclient_setup(
                                    sdoc,
                                    CO_CAN_ID_SDO_CLI + req->node_id,
                                    CO_CAN_ID_SDO_SRV + req->node_id,
                                    req->node_id)
                            != CO_SDO_RT_ok_communicationEnd)
                        {
                            return;
                        }

                        if (CO_SDOclientUploadInitiate(
                                sdoc, req->index, req->sub_index, 1000, false)
                            != CO_SDO_RT_ok_communicationEnd)
                        {
                            return;
                        }

                        CO_SDO_return_t ret;
                        do {
                            CO_SDO_abortCode_t abort_code = CO_SDO_AB_NONE;

                            ret = CO_SDOclientUpload(
                                    sdoc, 10000, false, &abort_code,
                                    nullptr, nullptr, nullptr);
                            if (ret < 0) {
                                return;
                            }
                            std::this_thread::sleep_for(10000us);
                        } while (ret > 0);

                        CO_SDOclientUploadBufRead(sdoc,
                                reinterpret_cast<unsigned char*>(&resp->value), sizeof(resp->value));

                    }, rmw_qos_profile_services_default, sdo_callback_group_);

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
        rclcpp::Service<cogtwa_interfaces::srv::ODReadU16>> service_od_read_u16_;

    std::shared_ptr<
        rclcpp::Service<cogtwa_interfaces::srv::SDOReadU16>> service_sdo_read_u16_;

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

