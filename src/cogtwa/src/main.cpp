#include <cstdio>
#include <chrono>
#include <cstdarg>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cogtwa_interfaces/srv/od_read_u16.hpp"

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
    } else if (priority == LOG_ERR) {
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
        : Node("COGtwa")
    {
        publisher_ =
            this->create_publisher<std_msgs::msg::String>(
                    "od2110_written", 10);

        service_od_read_u16_ =
            this->create_service<cogtwa_interfaces::srv::ODReadU16>(
                    "od_read_u16",
                    [CO, OD](
                        const std::shared_ptr<cogtwa_interfaces::srv::ODReadU16::Request> req,
                        std::shared_ptr<cogtwa_interfaces::srv::ODReadU16::Response> res
                      ) {
                        OD_entry_t *entry = OD_find(OD, req->index);

                        OD_IO_t io;
                        OD_getSub(entry, req->sub_index, &io, false);

                        OD_size_t bytes_read;
                        std::uint16_t value = 0;

                        CO_LOCK_OD(CO->CANmodule);
                        io.read(&io.stream, &value, sizeof value, &bytes_read);
                        CO_UNLOCK_OD(CO->CANmodule);

                        res->value = value;
                    });

        using namespace std::chrono_literals;

        timer_ =
            this->create_wall_timer(500ms, [&pub = publisher_]{
                    std_msgs::msg::String msg;
                    msg.data = "Hello, world!";
                    pub->publish(msg);
                });
    }
private:
    std::shared_ptr<
        rclcpp::Publisher<std_msgs::msg::String>> publisher_;

    std::shared_ptr<
        rclcpp::Service<cogtwa_interfaces::srv::ODReadU16>> service_od_read_u16;

    std::shared_ptr<
        rclcpp::Service<cogtwa_interfaces::srv::SDOReadU16>> service_sdo_read_u16;

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
	CO_ReturnError_t err = CO_epoll_create(&ep_main, MAIN_THREAD_INTERVAL_US);
	if (err != CO_ERROR_NO) {
		log_printf(LOG_CRIT, DBG_GENERAL, "CO_epoll_create(main), err=", err);
		std::exit(EXIT_FAILURE);
	}
    int app_eventfd = eventfd(0, EFD_NONBLOCK);
    struct epoll_event ev{};
    ev.events = EPOLLIN;
    ev.data.fd = app_eventfd;
    epoll_ctl(ep_main.epoll_fd, EPOLL_CTL_ADD, ev.data.fd, &ev);

	CANptr.epoll_fd = ep_main.epoll_fd;

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

	CO_epoll_initCANopenMain(&ep_main, CO);

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

	CO_CANsetNormalMode(CO->CANmodule);

    std::atomic<bool> should_die{false};

    std::future<void> co_fut = std::async(
                std::launch::async,
                [&] {
                    CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
                    while (reset == CO_RESET_NOT) {
                        CO_epoll_wait(&ep_main);
                        if (should_die.load()) {
                            break;
                        }
                        CO_epoll_processRT(&ep_main, CO, false);
                        CO_epoll_processMain(&ep_main, CO, false, &reset);
                        CO_epoll_processLast(&ep_main);
                    }
                });


    rclcpp::spin_until_future_complete(std::make_shared<COGtwa>(CO, OD), co_fut);

    should_die.store(true);

    uint64_t u = 1;
    write(ep_main.event_fd, &u, sizeof u);

    co_fut.wait();

	CO_epoll_close(&ep_main);
	CO_CANsetConfigurationMode(&CANptr);
	CO_delete(CO);

    rclcpp::shutdown();
}

