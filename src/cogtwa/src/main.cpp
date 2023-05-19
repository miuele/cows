#include <cstdio>
#include <chrono>
#include <cstdarg>
#include <thread>
#include <charconv>
#include <optional>
#include <variant>

#include <boost/thread/sync_queue.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cogtwa_interfaces/srv/od_read.hpp"
#include "cogtwa_interfaces/srv/od_write.hpp"
#include "cogtwa_interfaces/srv/sdo_read.hpp"
#include "cogtwa_interfaces/srv/sdo_write.hpp"
#include "cogtwa_interfaces/srv/configure_od_notify.hpp"
#include "cogtwa_interfaces/srv/nmt_state_get_by_node_id.hpp"
#include "cogtwa_interfaces/srv/tpdo_use_unused.hpp"
#include "cogtwa_interfaces/srv/rpdo_use_unused.hpp"
#include "cogtwa_interfaces/srv/hb_consumer_set_by_node_id.hpp"
#include "cogtwa_interfaces/msg/od_write_notify.hpp"
#include "cogtwa_interfaces/msg/nmt_state.hpp"
#include "cogtwa_interfaces/msg/hb_consumer_event.hpp"
#include "cogtwa_interfaces/msg/od_sink.hpp"

#include <syslog.h>

#include "CANopen.h"
extern "C" {
#include "OD.h"
}
#include "CO_error.h"
#include "CO_epoll_interface.h"
#include "CO_storageLinux.h"

#include "cogtwa/CO_utils.hpp"
#include "cogtwa/OD_Extension.hpp"
#include "cogtwa/HB_Consumer.hpp"

#define MAIN_THREAD_INTERVAL_US 100'000
#define NMT_CONTROL (static_cast<CO_NMT_control_t>( \
            CO_NMT_STARTUP_TO_OPERATIONAL \
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

std::string od_write_notify_topic_name(std::uint16_t index) {
    char a[9]{};
    std::to_chars(std::begin(a), std::end(a), index, 16);
    return "od_write_notify_0x"s + a;
}

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


std::optional<std::uint16_t> pdo_use_unused(OD_t *od, std::uint16_t index_from, std::uint16_t index_to, std::uint32_t cob_id) {
    for (OD_entry_t *entry = OD_find(od, index_from)
            ; entry < (od->list + od->size)
            ; ++entry)
    {
        std::uint16_t index = OD_getIndex(entry);
        if (index > index_to) {
            break;
        }
        std::uint32_t value;
        if (OD_get_u32(entry, 1, &value, false) == ODR_OK) {
            if ((value & 0xBFFFFFFFU) == 0x80000000U) {
                if (OD_set_u32(entry, 1, cob_id, false) == ODR_OK) {
                    return index;
                } else {
                    break;
                }
            };
        }
    }
    return std::nullopt;
}

std::optional<std::uint16_t> hb_consumer_set_by_node_id(OD_entry_t *entry, std::uint8_t node_id, std::uint16_t time_ms) {
    std::uint32_t new_value = static_cast<std::uint32_t>(node_id) << 16 | time_ms;
    std::uint8_t empty = 0;
    for (std::uint8_t i = 1; i < entry->subEntriesCount; ++i) {
        std::uint32_t value;
        if (OD_get_u32(entry, i, &value, false) == ODR_OK) {
            std::uint8_t id = (value >> 16) & 0xFFU;
            if (id == node_id) {
                if (OD_set_u32(entry, i, new_value, false) == ODR_OK) {
                    return i;
                } else {
                    return std::nullopt;
                }
            } else if (empty == 0 && id == 0) {
                empty = i;
            }
        }
    }

    // entry corresponding to node_id not found, write to unused entry
    if (empty != 0 && OD_set_u32(entry, empty, new_value, false) == ODR_OK) {
        return empty;
    }

    return std::nullopt;
}

std::mutex co_global_mutex;

class COGtwa
    : public rclcpp::Node
{
private:

    using ODReadService = rclcpp::Service<cogtwa_interfaces::srv::ODRead>;
    using ODWriteService = rclcpp::Service<cogtwa_interfaces::srv::ODWrite>;

    using SDOReadService = rclcpp::Service<cogtwa_interfaces::srv::SDORead>;
    using SDOWriteService = rclcpp::Service<cogtwa_interfaces::srv::SDOWrite>;

    using ConfigureODNotifyService = rclcpp::Service<cogtwa_interfaces::srv::ConfigureODNotify>;

    using NMTStateGetByNodeIdService = rclcpp::Service<cogtwa_interfaces::srv::NMTStateGetByNodeId>;

    using TPDOUseUnusedService = rclcpp::Service<cogtwa_interfaces::srv::TPDOUseUnused>;
    using RPDOUseUnusedService = rclcpp::Service<cogtwa_interfaces::srv::RPDOUseUnused>;

    using HBConsumerSetByNodeId = rclcpp::Service<cogtwa_interfaces::srv::HBConsumerSetByNodeId>;

    using ODWriteNotifyPublisher = rclcpp::Publisher<cogtwa_interfaces::msg::ODWriteNotify>;
    using NMTStatePublisher = rclcpp::Publisher<cogtwa_interfaces::msg::NMTState>;
    using HBConsumerEventPublisher = rclcpp::Publisher<cogtwa_interfaces::msg::HBConsumerEvent>;

    using ODSinkSubscriber = rclcpp::Subscription<cogtwa_interfaces::msg::ODSink>;

    struct ODWriteNotifyData {
        std::uint16_t index;
        std::uint8_t sub_index;
        std::vector<std::uint8_t> data;
    };

    struct NMTStateData {
        std::uint8_t node_id;
        std::uint8_t index;
        CO_NMT_internalState_t state;
    };

    struct HBConsumerEventData {
        std::uint8_t node_id;
        std::uint8_t index;
        HBConsumerEvent event;
    };

    using PublisherDataVariant = std::variant<ODWriteNotifyData, NMTStateData, HBConsumerEventData>;

public:
    COGtwa(CO_t *CO, OD_t *OD)
        : Node("cogtwa"), od_extension_(OD), hb_consumer_(CO->HBcons)
    {

        sdo_callback_group_ =
            this->create_callback_group(
                    rclcpp::CallbackGroupType::MutuallyExclusive);

        service_od_read_ =
            this->create_service<cogtwa_interfaces::srv::ODRead>(
                    "od_read",
                    [can_module = CO->CANmodule, OD](
                        const std::shared_ptr<cogtwa_interfaces::srv::ODRead::Request> req,
                        std::shared_ptr<cogtwa_interfaces::srv::ODRead::Response> resp
                      ) {
                        resp->data.resize(req->size);

                        ScopedODLock lock(can_module);
                        resp->ok = OD_get_value(OD_find(OD, req->index), req->sub_index, resp->data.data(), req->size, false)
                            == ODR_OK;
                    });

        service_od_write_ =
            this->create_service<cogtwa_interfaces::srv::ODWrite>(
                    "od_write",
                    [can_module = CO->CANmodule, OD](
                        const std::shared_ptr<cogtwa_interfaces::srv::ODWrite::Request> req,
                        std::shared_ptr<cogtwa_interfaces::srv::ODWrite::Response> resp
                      ) {
                        OD_entry_t *entry = OD_find(OD, req->index);

                        ScopedODLock lock(can_module);

                        if (req->data.size() > 0) {
                            resp->ok = OD_set_value(entry, req->sub_index, req->data.data(), req->data.size(), req->no_notify)
                                == ODR_OK;
                        }

                        if (req->request_tpdo) {
                            uint8_t *flags_pdo = OD_getFlagsPDO(entry);
                            assert(flags_pdo);
                            OD_requestTPDO(flags_pdo, req->sub_index);
                        }
                    });

        service_tpdo_use_unused_ =
            this->create_service<cogtwa_interfaces::srv::TPDOUseUnused>(
                    "tpdo_use_unused",
                    [can_module = CO->CANmodule, OD](
                        const std::shared_ptr<cogtwa_interfaces::srv::TPDOUseUnused::Request> req,
                        std::shared_ptr<cogtwa_interfaces::srv::TPDOUseUnused::Response> resp
                      ) {

                        ScopedODLock lock(can_module);

                        auto index = pdo_use_unused(OD, 0x1800, 0x19FF, req->cob_id);
                        if (index) {
                            resp->index = index.value();
                            resp->ok = true;
                        } else {
                            resp->index = 0;
                            resp->ok = false;
                        }
                    });

        service_rpdo_use_unused_ =
            this->create_service<cogtwa_interfaces::srv::RPDOUseUnused>(
                    "rpdo_use_unused",
                    [can_module = CO->CANmodule, OD](
                        const std::shared_ptr<cogtwa_interfaces::srv::RPDOUseUnused::Request> req,
                        std::shared_ptr<cogtwa_interfaces::srv::RPDOUseUnused::Response> resp
                      ) {

                        ScopedODLock lock(can_module);

                        auto index = pdo_use_unused(OD, 0x1400, 0x15FF, req->cob_id);
                        if (index) {
                            resp->index = index.value();
                            resp->ok = true;
                        } else {
                            resp->index = 0;
                            resp->ok = false;
                        }
                    });

        service_hb_consumer_set_by_node_id_ =
            this->create_service<cogtwa_interfaces::srv::HBConsumerSetByNodeId>(
                    "hb_consumer_set_by_node_id",
                    [can_module = CO->CANmodule, OD](
                        const std::shared_ptr<cogtwa_interfaces::srv::HBConsumerSetByNodeId::Request> req,
                        std::shared_ptr<cogtwa_interfaces::srv::HBConsumerSetByNodeId::Response> resp
                      ) {

                        ScopedODLock lock(can_module);

                        auto index = hb_consumer_set_by_node_id(OD_find(OD, 0x1016), req->node_id, req->heartbeat_time);
                        if (index) {
                            resp->index = index.value();
                            resp->ok = true;
                        } else {
                            resp->index = 0;
                            resp->ok = false;
                        }
                    });

        od_sink_subscriber_ =
            this->create_subscription<cogtwa_interfaces::msg::ODSink>(
                    "od_sink", 10,
                    [can_module = CO->CANmodule, OD](
                        const std::shared_ptr<cogtwa_interfaces::msg::ODSink> msg
                    ) {
                        OD_entry_t *entry = OD_find(OD, msg->index);

                        ScopedODLock lock(can_module);

                        if (msg->data.size() > 0) {
                            if (OD_set_value(entry, msg->sub_index, msg->data.data(), msg->data.size(), msg->no_notify)
                                    != ODR_OK) {
                                return;
                            }
                        }

                        if (msg->request_tpdo) {
                            uint8_t *flags_pdo = OD_getFlagsPDO(entry);
                            assert(flags_pdo);
                            OD_requestTPDO(flags_pdo, msg->sub_index);
                        }
                    });

        service_sdo_read_ =
            this->create_service<cogtwa_interfaces::srv::SDORead>(
                    "sdo_read",
                    [sdoc = CO->SDOclient, OD](
                        const std::shared_ptr<cogtwa_interfaces::srv::SDORead::Request> req,
                        std::shared_ptr<cogtwa_interfaces::srv::SDORead::Response> resp
                        ) {

                        resp->data.resize(req->size);
                        std::size_t size_read;

                        resp->ok = read_SDO_blocking(
                                sdoc,
                                co_global_mutex,
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
                    [sdoc = CO->SDOclient, OD](
                        const std::shared_ptr<cogtwa_interfaces::srv::SDOWrite::Request> req,
                        std::shared_ptr<cogtwa_interfaces::srv::SDOWrite::Response> resp
                        ) {

                        resp->ok = write_SDO_blocking(
                                sdoc,
                                co_global_mutex,
                                req->node_id,
                                req->index,
                                req->sub_index,
                                req->data.data(),
                                req->data.size()) == CO_SDO_AB_NONE;

                    }, rmw_qos_profile_services_default, sdo_callback_group_);

        service_configure_od_notify_ =
            this->create_service<cogtwa_interfaces::srv::ConfigureODNotify>(
                    "configure_od_notify",
                    [this, OD](
                        const std::shared_ptr<cogtwa_interfaces::srv::ConfigureODNotify::Request> req,
                        std::shared_ptr<cogtwa_interfaces::srv::ConfigureODNotify::Response> resp
                        ) {

                        auto index = req->index;
                        auto topic_name = od_write_notify_topic_name(index);

                        if (req->on_write) {
                            if (od_notify_publishers_.count(index) > 0) {
                                RCLCPP_INFO(this->get_logger(), "ODNotify topic \"%s\" is already activated.", topic_name.c_str());
                            } else {
                                auto publisher = this->create_publisher<cogtwa_interfaces::msg::ODWriteNotify>(topic_name, 10);
                                RCLCPP_INFO(this->get_logger(), "activating ODNotify topic \"%s\".", topic_name.c_str());
                                od_notify_publishers_.emplace(index, publisher);

                                od_extension_.on_write(index, [this, index](OD_stream_t *stream, const void *buf, OD_size_t size, OD_size_t *size_written){

                                    ODWriteNotifyData notify;
                                    notify.index = index;
                                    notify.sub_index = stream->subIndex;
                                    notify.data.resize(size);
                                    std::memcpy(notify.data.data(), buf, size);

                                    this->push_publisher_data(std::move(notify));

                                    return OD_writeOriginal(stream, buf, size, size_written);
                                });
                            }
                        } else {
                            od_notify_publishers_.erase(index);
                            od_extension_.on_write(index, nullptr);
                        }

                        resp->ok = true;
                        resp->topic_name = topic_name;
                    });

        service_nmt_state_get_by_node_id_ =
            this->create_service<cogtwa_interfaces::srv::NMTStateGetByNodeId>(
                    "nmt_state_get_by_node_id",
                    [this, hbcons = CO->HBcons](
                        const std::shared_ptr<cogtwa_interfaces::srv::NMTStateGetByNodeId::Request> req,
                        std::shared_ptr<cogtwa_interfaces::srv::NMTStateGetByNodeId::Response> resp
                        ) {
                        auto node_id = req->node_id;

                        std::lock_guard lock(co_global_mutex);

                        auto index = CO_HBconsumer_getIdxByNodeId(hbcons, node_id);
                        if (index == -1) {
                            resp->ok = false;
                            return;
                        }
                        CO_NMT_internalState_t state;
                        if (CO_HBconsumer_getNmtState(hbcons, index, &state) == -1) {
                            state = CO_NMT_UNKNOWN;
                        }
                        resp->ok = true;
                        resp->nmt_state = state;
                    });

        nmt_state_publisher_ = this->create_publisher<cogtwa_interfaces::msg::NMTState>("nmt_state", 10);
        hb_consumer_event_publisher_ = this->create_publisher<cogtwa_interfaces::msg::HBConsumerEvent>("hb_consumer_event", 10);

        hb_consumer_.on_nmt_state_changed([this](std::uint8_t node_id, std::uint8_t index, CO_NMT_internalState_t state) {
                    RCLCPP_DEBUG(this->get_logger(), "node %u (index: %u) state changed to %d", node_id, index, state);

                    NMTStateData state_data{node_id, index, state};
                    this->push_publisher_data(std::move(state_data));
                });

        hb_consumer_.on_hb_consumer_event([this](std::uint8_t node_id, std::uint8_t index, HBConsumerEvent event) {
                    RCLCPP_DEBUG(this->get_logger(), "node %u (index: %u) event: %d", node_id, index, static_cast<int>(event));

                    HBConsumerEventData event_data{node_id, index, event};
                    this->push_publisher_data(std::move(event_data));
                });
    }

    void push_publisher_data(PublisherDataVariant &&data) {
        publisher_data_queue_.push(std::move(data));
    }

    void close_publisher_data_queue() {
        publisher_data_queue_.close();
    }

    void run_publisher_thread() {
        struct {
            void operator()(const ODWriteNotifyData &notify) {
                node->publish_od_notify(std::move(notify));
            }
            void operator()(const NMTStateData &data) {
                node->publish_nmt_state(data);
            }
            void operator()(const HBConsumerEventData &data) {
                node->publish_hb_consumer_event(data);
            }
            COGtwa *node;
        } publish_data{this};

        while (true) {
            try {
                const auto data = publisher_data_queue_.pull();
                if (std::holds_alternative<ODWriteNotifyData>(data)) {
                    auto index = std::get<ODWriteNotifyData>(data).data.size();
                    RCLCPP_DEBUG(this->get_logger(), "publishing for size: %zu", index);
                }
                std::visit(publish_data, data);
            } catch (const boost::sync_queue_is_closed &) {
                break;
            }
        }
    }

    void publish_od_notify(const ODWriteNotifyData &notify) {
        cogtwa_interfaces::msg::ODWriteNotify msg;
        msg.index = notify.index;
        msg.sub_index = notify.sub_index;
        msg.data = std::move(notify.data);

        od_notify_publishers_.at(notify.index)->publish(msg);
    }

    void publish_nmt_state(const NMTStateData &data) {
        cogtwa_interfaces::msg::NMTState msg;
        msg.node_id = data.node_id;
        msg.index = data.index;
        msg.nmt_state = static_cast<std::int8_t>(data.state);

        nmt_state_publisher_->publish(msg);
    }

    void publish_hb_consumer_event(const HBConsumerEventData &data) {
        cogtwa_interfaces::msg::HBConsumerEvent msg;
        msg.node_id = data.node_id;
        msg.index = data.index;
        msg.hb_consumer_event = static_cast<std::int8_t>(data.event);

        hb_consumer_event_publisher_->publish(msg);
    }

private:
    std::shared_ptr<rclcpp::CallbackGroup> sdo_callback_group_;

    std::shared_ptr<ODReadService> service_od_read_;
    std::shared_ptr<ODWriteService> service_od_write_;
    std::shared_ptr<SDOReadService> service_sdo_read_;
    std::shared_ptr<SDOWriteService> service_sdo_write_;
    std::shared_ptr<ConfigureODNotifyService> service_configure_od_notify_;
    std::shared_ptr<NMTStateGetByNodeIdService> service_nmt_state_get_by_node_id_;
    std::shared_ptr<TPDOUseUnusedService> service_tpdo_use_unused_;
    std::shared_ptr<RPDOUseUnusedService> service_rpdo_use_unused_;
    std::shared_ptr<HBConsumerSetByNodeId> service_hb_consumer_set_by_node_id_;

    boost::sync_queue<PublisherDataVariant> publisher_data_queue_;

    std::unordered_map<std::uint16_t, std::shared_ptr<ODWriteNotifyPublisher>> od_notify_publishers_;

    std::shared_ptr<NMTStatePublisher> nmt_state_publisher_;
    std::shared_ptr<HBConsumerEventPublisher> hb_consumer_event_publisher_;

    std::shared_ptr<ODSinkSubscriber> od_sink_subscriber_;

    ODExtension od_extension_;
    HBConsumer hb_consumer_;
};

using namespace std::chrono_literals;

struct parameters_t {
    std::string can_interface;
    std::string gtwa_interface;
    std::uint16_t node_id;
};

void collect_parameters(parameters_t &params) {
    auto node_init = std::make_shared<rclcpp::Node>("cogtwa_init");
    node_init->declare_parameter("can_interface", "can0");
    node_init->declare_parameter("gtwa_interface", "/tmp/CO_command_socket");
    node_init->declare_parameter("node_id", 3);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_init);

    params.can_interface = node_init->get_parameter("can_interface").as_string();
    params.gtwa_interface = node_init->get_parameter("gtwa_interface").as_string();
    params.node_id = node_init->get_parameter("node_id").as_int();

    executor.spin_some();
}

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    parameters_t params;
    collect_parameters(params);

    CO_CANptrSocketCan_t CANptr{};

    CANptr.can_ifindex = if_nametoindex(params.can_interface.c_str());
    if (CANptr.can_ifindex == 0) {
        log_printf(LOG_CRIT, DBG_NO_CAN_DEVICE, params.can_interface.c_str());
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
        CO_ReturnError_t err = CO_epoll_createGtw(
                &ep_gtw,
                ep_main.epoll_fd,
                CO_COMMAND_IF_LOCAL_SOCKET,
                1000,
                params.gtwa_interface.data());
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
                params.node_id,
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
        CO_ReturnError_t err = CO_CANopenInitPDO(CO, CO->em, OD, params.node_id, &err_info);
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
                std::lock_guard lock(co_global_mutex);
                CO_epoll_processRT(&ep_main, CO, false);
                CO_epoll_processMain(&ep_main, CO, true, &reset);
                CO_epoll_processGtw(&ep_gtw, CO, &ep_main);
                CO_epoll_processLast(&ep_main);
            }
        });

    auto node = std::make_shared<COGtwa>(CO, OD);

    std::thread publisher_thread(std::bind(&COGtwa::run_publisher_thread, node));


    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node); 

    RCLCPP_INFO(rclcpp::get_logger("cogtwa"), "start spinning...");

    executor.spin();

    should_die.store(true);

    uint64_t u = 1;
    write(ep_main.event_fd, &u, sizeof u);

    co_rtml_thread.join();

    node->close_publisher_data_queue();
    publisher_thread.join();

    CO_epoll_close(&ep_main);
    CO_epoll_closeGtw(&ep_gtw);

    CO_CANsetConfigurationMode(&CANptr);

    CO_delete(CO);

    rclcpp::shutdown();
}
