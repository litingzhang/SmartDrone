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

void ValidateSpscQueueDepth(std::size_t queueDepth);
void FillQueueDiagnosticsSnapshot(const QueueDiagnostics &diag,
                                  QueueDiagnosticsSnapshot &snapshot);

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
        ValidateSpscQueueDepth(queueDepth);
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
        const auto next = Increment(head);
        if (!EnsureWritableSlot(next)) {
            return false;
        }
        CommitPush(head, next, std::move(item));
        return true;
    }

    std::shared_ptr<T> TryPop()
    {
        for (;;) {
            PopClaim claim = TryClaimPopSlot();
            if (claim.status == PopClaimStatus::Claimed) {
                return CommitPop(claim.tail);
            }
            if (QueueWasEmpty(claim)) {
                return {};
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
        FillQueueDiagnosticsSnapshot(m_diag, snapshot);
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
    enum class PopClaimStatus {
        Empty,
        Retry,
        Claimed,
    };

    struct PopClaim {
        PopClaimStatus status{PopClaimStatus::Retry};
        std::size_t tail{0};
    };

    static bool QueueWasEmpty(const PopClaim &claim)
    {
        return claim.status == PopClaimStatus::Empty;
    }

    std::size_t Increment(std::size_t value) const
    {
        return (value + 1) % m_capacity;
    }

    bool EnsureWritableSlot(std::size_t next)
    {
        auto tail = m_tail.load(std::memory_order_acquire);
        return next != tail || HandleFullQueue(tail);
    }

    bool HandleFullQueue(std::size_t &tail)
    {
        if (m_overflow == OverflowPolicy::DropNewest) {
            m_diag.droppedNewest.fetch_add(1, std::memory_order_relaxed);
            StoreQueueActivity();
            return false;
        }
        DropOldestItem(tail);
        return true;
    }

    void DropOldestItem(std::size_t &tail)
    {
        while (true) {
            if (TryAdvanceTail(tail)) {
                RecordOverwriteOldest();
                return;
            }
        }
    }

    PopClaim TryClaimPopSlot()
    {
        std::size_t tail = m_tail.load(std::memory_order_acquire);
        if (tail == m_head.load(std::memory_order_acquire)) {
            return {PopClaimStatus::Empty, tail};
        }
        if (TryAdvanceTail(tail)) {
            return {PopClaimStatus::Claimed, tail};
        }
        return {PopClaimStatus::Retry, tail};
    }

    bool TryAdvanceTail(std::size_t &tail)
    {
        const auto tailNext = Increment(tail);
        return m_tail.compare_exchange_weak(tail, tailNext,
                                            std::memory_order_acq_rel,
                                            std::memory_order_acquire);
    }

    void RecordOverwriteOldest()
    {
        m_diag.overwrittenOldest.fetch_add(1, std::memory_order_relaxed);
    }

    void CommitPush(std::size_t head, std::size_t next,
                    std::shared_ptr<T> item)
    {
        m_slots[head] = std::move(item);
        m_head.store(next, std::memory_order_release);
        m_diag.pushed.fetch_add(1, std::memory_order_relaxed);
        StoreQueueActivity();
        UpdateMaxDepth(Size());
        Notify();
    }

    std::shared_ptr<T> CommitPop(std::size_t tail)
    {
        auto item = std::move(m_slots[tail]);
        m_slots[tail].reset();
        m_diag.popped.fetch_add(1, std::memory_order_relaxed);
        StoreQueueActivity();
        return item;
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
        IQueue *queue = InputQueueByPort(port);
        ValidateQueueType(queue, port, std::type_index(typeid(T)), "input");
        return queue;
    }

    template <class T>
    IQueue *OutputQueue(PortId port)
    {
        IQueue *queue = OutputQueueByPortMutable(port);
        ValidateQueueType(queue, port, std::type_index(typeid(T)), "output");
        return queue;
    }

    IQueue *InputQueueByPort(PortId port) const;
    IQueue *OutputQueueByPortMutable(PortId port) const;
    static void ValidateQueueType(IQueue *queue, PortId port,
                                  std::type_index expectedType,
                                  const char *direction);

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
