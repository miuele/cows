#ifndef HB_CONSUMER_HPP_INCLUDED
#define HB_CONSUMER_HPP_INCLUDED

#include <functional>
#include <memory>
#include <mutex>

enum struct HBConsumerEvent : std::int8_t {
    HeartbeatStarted = 0,
    TimeOut = 1,
    RemoteReset = 2,
};

class HBConsumer {
private:
    using NMTStateChangedCallbackType = void (std::uint8_t, std::uint8_t, CO_NMT_internalState_t);
    using HBConsumerEventCallbackType = void (std::uint8_t, std::uint8_t, HBConsumerEvent);

    struct Hook {
        std::function<NMTStateChangedCallbackType> callback_nmt_state;
        std::function<NMTStateChangedCallbackType> callback_nmt_state_new;
        std::function<HBConsumerEventCallbackType> callback_cons_event;
        std::function<HBConsumerEventCallbackType> callback_cons_event_new;
        std::unique_ptr<std::mutex> mutex;
    };

public:

    explicit HBConsumer(CO_HBconsumer_t *hbcons)
    {
        const std::uint8_t num_monitored_nodes = hbcons->numberOfMonitoredNodes;

        if (num_monitored_nodes == 0) {
            return;
        }

        hook_.mutex = std::make_unique<std::mutex>();

        for (std::uint8_t index = 0; index < num_monitored_nodes; ++index) {
            CO_HBconsumer_initCallbackNmtChanged(hbcons, index, &hook_, &HBConsumer::hbcons_nmt_state_changed);
            CO_HBconsumer_initCallbackHeartbeatStarted(hbcons, index, &hook_, &HBConsumer::hbcons_heartbeat_started);
            CO_HBconsumer_initCallbackTimeout(hbcons, index, &hook_, &HBConsumer::hbcons_timeout);
            CO_HBconsumer_initCallbackRemoteReset(hbcons, index, &hook_, &HBConsumer::hbcons_remote_reset);
        }

    }

    template <class F>
    void on_nmt_state_changed(F &&func) {
        std::lock_guard<std::mutex> lock(*hook_.mutex);
        hook_.callback_nmt_state_new = std::forward<F>(func);
    }

    template <class F>
    void on_hb_consumer_event(F &&func) {
        std::lock_guard<std::mutex> lock(*hook_.mutex);
        hook_.callback_cons_event_new = std::forward<F>(func);
    }

private:

    static void hbcons_nmt_state_changed(
            std::uint8_t node_id,
            std::uint8_t index,
            CO_NMT_internalState_t nmt_state,
            void *object)
    {
        Hook &hook = *(static_cast<Hook *>(object));
        {
            std::lock_guard<std::mutex> lock(*hook.mutex);
            if (hook.callback_nmt_state_new) {
                hook.callback_nmt_state = std::move(hook.callback_nmt_state_new);
                hook.callback_nmt_state_new = nullptr;
            }
            /*
             * hook.callback_nmt_state is guaranteed to be valid until next call to this function
             */
        }

        if (hook.callback_nmt_state) {
            (hook.callback_nmt_state)(node_id, index, nmt_state);
        }
    }

    static void hbcons_heartbeat_started(
            std::uint8_t node_id,
            std::uint8_t index,
            void *object)
    {
        hbcons_event_ocurred(node_id, index, HBConsumerEvent::HeartbeatStarted, object);
    }

    static void hbcons_timeout(
            std::uint8_t node_id,
            std::uint8_t index,
            void *object)
    {
        hbcons_event_ocurred(node_id, index, HBConsumerEvent::TimeOut, object);
    }

    static void hbcons_remote_reset(
            std::uint8_t node_id,
            std::uint8_t index,
            void *object)
    {
        hbcons_event_ocurred(node_id, index, HBConsumerEvent::RemoteReset, object);
    }

    static void hbcons_event_ocurred(
            std::uint8_t node_id,
            std::uint8_t index,
            HBConsumerEvent event,
            void *object)
    {
        Hook &hook = *static_cast<Hook *>(object);
        {
            std::lock_guard<std::mutex> lock(*hook.mutex);
            if (hook.callback_cons_event_new) {
                hook.callback_cons_event = std::move(hook.callback_cons_event_new);
                hook.callback_cons_event_new = nullptr;
            }
            /*
             * hook.callback_cons_event is guaranteed to be valid until next call to this function
             */
        }

        if (hook.callback_cons_event) {
            (hook.callback_cons_event)(node_id, index, event);
        }
    }

    Hook hook_;
};

#endif
