#pragma once

#include "common/epg/epg_types.h"

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>
#include <utility>
#include <vector>

namespace Epg {

struct PortSpec {
    PortId id{};
    std::string type;
};

class IQueue {
  public:
    virtual ~IQueue() = default;
    virtual const std::string &Name() const = 0;
    virtual const std::string &TypeName() const = 0;
    virtual std::type_index TypeIndex() const = 0;
    virtual std::size_t Depth() const = 0;
    virtual std::size_t Size() const = 0;
    virtual bool Empty() const = 0;
    virtual bool PushErased(std::shared_ptr<void> item) = 0;
    virtual std::shared_ptr<void> TryPopErased() = 0;
    virtual std::shared_ptr<void> TryPopLatestErased() = 0;
    virtual QueueDiagnosticsSnapshot Diagnostics() const = 0;
    virtual void SetNotifier(std::function<void()> notifier) = 0;
};

template <class T>
class SpscSharedPtrQueue final : public IQueue {
  public:
    SpscSharedPtrQueue(std::string queueName,
                       std::string queueTypeName,
                       std::size_t queueDepth,
                       OverflowPolicy overflow)
        : m_name(std::move(queueName)),
          m_typeName(std::move(queueTypeName)),
          m_depth(queueDepth),
          m_capacity(queueDepth + 1),
          m_overflow(overflow),
          m_slots(m_capacity)
    {
        if (queueDepth == 0) {
            throw std::invalid_argument(
                "SPSC queue depth must be greater than zero");
        }
    }

    const std::string &Name() const override
    {
        return m_name;
    }
    const std::string &TypeName() const override
    {
        return m_typeName;
    }
    std::type_index TypeIndex() const override
    {
        return std::type_index(typeid(T));
    }
    std::size_t Depth() const override
    {
        return m_depth;
    }

    std::size_t Size() const override
    {
        const auto head = m_head.load(std::memory_order_acquire);
        const auto tail = m_tail.load(std::memory_order_acquire);
        return head >= tail ? head - tail : m_capacity - tail + head;
    }

    bool Empty() const override
    {
        return m_head.load(std::memory_order_acquire) ==
               m_tail.load(std::memory_order_acquire);
    }

    bool Push(std::shared_ptr<T> item)
    {
        auto head = m_head.load(std::memory_order_relaxed);
        auto tail = m_tail.load(std::memory_order_acquire);
        auto next = Increment(head);

        if (next == tail) {
            if (m_overflow == OverflowPolicy::DropNewest) {
                m_diag.droppedNewest.fetch_add(1, std::memory_order_relaxed);
                StoreQueueActivity();
                return false;
            }

            while (next == tail) {
                auto tailNext = Increment(tail);
                if (m_tail.compare_exchange_weak(tail, tailNext,
                                                 std::memory_order_acq_rel,
                                                 std::memory_order_acquire)) {
                    m_diag.overwrittenOldest.fetch_add(
                        1, std::memory_order_relaxed);
                    break;
                }
            }
        }

        m_slots[head] = std::move(item);
        m_head.store(next, std::memory_order_release);
        m_diag.pushed.fetch_add(1, std::memory_order_relaxed);
        StoreQueueActivity();
        UpdateMaxDepth(Size());
        Notify();
        return true;
    }

    std::shared_ptr<T> TryPop()
    {
        for (;;) {
            auto tail = m_tail.load(std::memory_order_acquire);
            const auto head = m_head.load(std::memory_order_acquire);
            if (tail == head) {
                return {};
            }

            const auto next = Increment(tail);
            if (m_tail.compare_exchange_weak(tail, next,
                                             std::memory_order_acq_rel,
                                             std::memory_order_acquire)) {
                auto item = std::move(m_slots[tail]);
                m_slots[tail].reset();
                m_diag.popped.fetch_add(1, std::memory_order_relaxed);
                StoreQueueActivity();
                return item;
            }
        }
    }

    std::shared_ptr<T> TryPopLatest()
    {
        std::shared_ptr<T> latest;
        while (auto item = TryPop()) {
            latest = std::move(item);
        }
        return latest;
    }

    bool PushErased(std::shared_ptr<void> item) override
    {
        return Push(std::static_pointer_cast<T>(std::move(item)));
    }

    std::shared_ptr<void> TryPopErased() override
    {
        return std::static_pointer_cast<void>(TryPop());
    }

    std::shared_ptr<void> TryPopLatestErased() override
    {
        return std::static_pointer_cast<void>(TryPopLatest());
    }

    QueueDiagnosticsSnapshot Diagnostics() const override
    {
        QueueDiagnosticsSnapshot snapshot;
        snapshot.pushed = m_diag.pushed.load(std::memory_order_relaxed);
        snapshot.popped = m_diag.popped.load(std::memory_order_relaxed);
        snapshot.droppedNewest =
            m_diag.droppedNewest.load(std::memory_order_relaxed);
        snapshot.overwrittenOldest =
            m_diag.overwrittenOldest.load(std::memory_order_relaxed);
        snapshot.wakeups = m_diag.wakeups.load(std::memory_order_relaxed);
        snapshot.maxDepthObserved =
            m_diag.maxDepthObserved.load(std::memory_order_relaxed);
        snapshot.firstActivityMs =
            m_diag.firstActivityMs.load(std::memory_order_relaxed);
        snapshot.lastActivityMs =
            m_diag.lastActivityMs.load(std::memory_order_relaxed);
        return snapshot;
    }

    void SetNotifier(std::function<void()> notifier) override
    {
        auto snapshot =
            std::make_shared<const std::function<void()>>(std::move(notifier));
        std::atomic_store_explicit(&m_notifier, std::move(snapshot),
                                   std::memory_order_release);
    }

  private:
    std::size_t Increment(std::size_t value) const
    {
        return (value + 1) % m_capacity;
    }

    void UpdateMaxDepth(std::size_t observed)
    {
        auto current = m_diag.maxDepthObserved.load(std::memory_order_relaxed);
        while (observed > current &&
               !m_diag.maxDepthObserved.compare_exchange_weak(
                   current, observed, std::memory_order_relaxed,
                   std::memory_order_relaxed)) {
        }
    }

    static std::uint64_t SteadyNowMs()
    {
        const auto now = std::chrono::steady_clock::now().time_since_epoch();
        return static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
    }

    void StoreFirstQueueActivity(std::uint64_t nowMs)
    {
        std::uint64_t empty = 0;
        (void)m_diag.firstActivityMs.compare_exchange_strong(
            empty, nowMs, std::memory_order_relaxed,
            std::memory_order_relaxed);
    }

    void StoreQueueActivity()
    {
        const std::uint64_t nowMs = SteadyNowMs();
        StoreFirstQueueActivity(nowMs);
        m_diag.lastActivityMs.store(nowMs, std::memory_order_relaxed);
    }

    void Notify()
    {
        auto notifier = std::atomic_load_explicit(
            &m_notifier, std::memory_order_acquire);
        if (notifier && *notifier) {
            m_diag.wakeups.fetch_add(1, std::memory_order_relaxed);
            (*notifier)();
        }
    }

    std::string m_name;
    std::string m_typeName;
    std::size_t m_depth{};
    std::size_t m_capacity{};
    OverflowPolicy m_overflow{OverflowPolicy::DropNewest};
    std::vector<std::shared_ptr<T>> m_slots;
    mutable QueueDiagnostics m_diag;
    std::atomic<std::size_t> m_head{0};
    std::atomic<std::size_t> m_tail{0};
    std::shared_ptr<const std::function<void()>> m_notifier;
};

class TaskContext {
  public:
    TaskContext(std::unordered_map<PortId, IQueue *> inputs,
                std::unordered_map<PortId, IQueue *> outputs);

    template <class T, class... Args>
    std::shared_ptr<T> Make(Args &&...args)
    {
        return std::make_shared<T>(std::forward<Args>(args)...);
    }

    template <class T>
    std::shared_ptr<T> TryPop(PortId port)
    {
        auto *queue = InputQueue<T>(port);
        return std::static_pointer_cast<T>(queue->TryPopErased());
    }

    template <class T>
    std::shared_ptr<T> TryPopLatest(PortId port)
    {
        auto *queue = InputQueue<T>(port);
        return std::static_pointer_cast<T>(queue->TryPopLatestErased());
    }

    template <class T>
    bool Push(PortId port, std::shared_ptr<T> item)
    {
        auto *queue = OutputQueue<T>(port);
        return queue->PushErased(
            std::static_pointer_cast<void>(std::move(item)));
    }

    bool InputReady(PortId port) const;
    bool InputExists(PortId port) const;
    std::size_t InputSize(PortId port) const;
    bool OutputExists(PortId port) const;
    std::size_t OutputSize(PortId port) const;
    const IQueue *OutputQueueByPort(PortId port) const;
    void AttachDiagnostics(TaskDiagnostics *diagnostics);
    void ReportResourceWait(std::uint64_t waitUs);

  private:
    template <class T>
    IQueue *InputQueue(PortId port)
    {
        auto it = m_inputs.find(port);
        if (it == m_inputs.end()) {
            throw std::runtime_error("missing input port: " +
                                     std::to_string(port));
        }
        if (it->second->TypeIndex() != std::type_index(typeid(T))) {
            throw std::runtime_error("input port type mismatch: " +
                                     std::to_string(port));
        }
        return it->second;
    }

    template <class T>
    IQueue *OutputQueue(PortId port)
    {
        auto it = m_outputs.find(port);
        if (it == m_outputs.end()) {
            throw std::runtime_error("missing output port: " +
                                     std::to_string(port));
        }
        if (it->second->TypeIndex() != std::type_index(typeid(T))) {
            throw std::runtime_error("output port type mismatch: " +
                                     std::to_string(port));
        }
        return it->second;
    }

    std::unordered_map<PortId, IQueue *> m_inputs;
    std::unordered_map<PortId, IQueue *> m_outputs;
    TaskDiagnostics *m_diagnostics{nullptr};
};

class ITask {
  public:
    virtual ~ITask() = default;
    virtual void OnTick(TaskContext &context) = 0;
};

} // namespace Epg
