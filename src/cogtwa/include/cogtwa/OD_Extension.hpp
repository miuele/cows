#ifndef OD_EXTENSION_HPP_INCLUDED
#define OD_EXTENSION_HPP_INCLUDED

#include "cogtwa/assoc_arena.hpp"
#include <variant>
#include <functional>
#include <memory>
#include <mutex>

class ODExtension {
private:
    using WriteCallbackType = ODR_t (OD_stream_t *, const void *, OD_size_t, OD_size_t *);
    using ReadCallbackType = ODR_t (OD_stream_t *, void *, OD_size_t, OD_size_t *);

    struct Hook {
        OD_extension_t od_extension;
        std::function<WriteCallbackType> callback_write;
        std::function<WriteCallbackType> callback_write_new;
        std::function<ReadCallbackType> callback_read;
        std::function<ReadCallbackType> callback_read_new;
        std::unique_ptr<std::mutex> mutex;
    };

    using HooksArena = assoc_arena::AssociativeArena<std::uint16_t, Hook>;

public:

    explicit ODExtension(OD_t *od)
    {
        std::vector<std::uint16_t> user_od_indices;
        for (OD_entry_t *entry = OD_find(od, 0x2000)
                ; entry < (od->list + od->size)
                ; ++entry)
        {
            std::uint16_t index = OD_getIndex(entry);
            if (index > 0x2FFF) {
                break;
            }
            user_od_indices.push_back(index);
        }

        arena_.emplace<HooksArena>(user_od_indices.size());

        for (std::uint16_t index : user_od_indices) {
            Hook *hook = std::get<HooksArena>(arena_).emplace(index);
            hook->od_extension.object = hook;
            hook->od_extension.write = &ODExtension::od_write;
            hook->od_extension.read = &ODExtension::od_read;
            hook->mutex = std::make_unique<std::mutex>();

            OD_extension_init(OD_find(od, index), &hook->od_extension);
        }

    }

    template <class F>
    void on_write(std::uint16_t index, F &&func) {
        Hook *hook = std::get<HooksArena>(arena_).get(index);

        std::lock_guard<std::mutex> lock(*hook->mutex);
        hook->callback_write_new = std::forward<F>(func);
    }

    template <class F>
    void on_read(std::uint16_t index, F &&func) {
        Hook *hook = std::get<HooksArena>(arena_).get(index);

        std::lock_guard<std::mutex> lock(*hook->mutex);
        hook->callback_read_new = std::forward<F>(func);
    }

private:

    static ODR_t od_write(
            OD_stream_t *stream,
            const void *buf,
            OD_size_t size,
            OD_size_t *size_written)
    {
        Hook *hook = static_cast<Hook *>(stream->object);
        {
            std::lock_guard<std::mutex> lock(*hook->mutex);
            if (hook->callback_write_new) {
                hook->callback_write = std::move(hook->callback_write_new);
                hook->callback_write_new = nullptr;
            }
            /*
             * hook->callback_write is guaranteed to be valid until next call to this function
             */
        }

        if (hook->callback_write) {
            return (hook->callback_write)(stream, buf, size, size_written);
        }
        return OD_writeOriginal(stream, buf, size, size_written);
    }

    static ODR_t od_read(
            OD_stream_t *stream,
            void *buf,
            OD_size_t size,
            OD_size_t *size_read)
    {
        Hook *hook = static_cast<Hook *>(stream->object);
        {
            std::lock_guard<std::mutex> lock(*hook->mutex);
            if (hook->callback_read_new) {
                hook->callback_read = std::move(hook->callback_read_new);
                hook->callback_read_new = nullptr;
            }
            /*
             * hook->callback_read is guaranteed to be valid until next call to this function
             */
        }

        if (hook->callback_read) {
            return (hook->callback_read)(stream, buf, size, size_read);
        }
        return OD_readOriginal(stream, buf, size, size_read);
    }

    std::variant<std::monostate, HooksArena> arena_;
};

#endif
