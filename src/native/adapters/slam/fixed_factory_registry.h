#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <memory>
#include <utility>

namespace SmartDrone::Adapters::Slam {

template <class Key, class Factory, std::size_t SLOT_COUNT>
class FixedFactoryRegistry {
  public:
    FixedFactoryRegistry()
    {
        std::atomic_store_explicit(&m_snapshot,
                                   std::make_shared<const Snapshot>(),
                                   std::memory_order_release);
    }

    bool Register(Key key, Factory factory)
    {
        if (factory == nullptr) {
            return false;
        }
        return TryRegister(key, factory);
    }

    Factory Find(Key key) const
    {
        std::shared_ptr<const Snapshot> snapshot = LoadSnapshot();
        if (!snapshot) {
            return nullptr;
        }
        return FindInSnapshot(*snapshot, key);
    }

  private:
    struct Entry {
        Key key{};
        Factory factory{nullptr};
    };

    struct Snapshot {
        std::array<Entry, SLOT_COUNT> entries{};
    };

    static bool Upsert(Snapshot &snapshot, Key key, Factory factory)
    {
        Entry *entry = FindWritableEntry(snapshot, key);
        if (entry == nullptr) {
            return false;
        }
        entry->key = key;
        entry->factory = factory;
        return true;
    }

    static Entry *FindEntry(Snapshot &snapshot, Key key)
    {
        for (Entry &entry : snapshot.entries) {
            if (entry.factory != nullptr && entry.key == key) {
                return &entry;
            }
        }
        return nullptr;
    }

    static const Entry *FindEntry(const Snapshot &snapshot, Key key)
    {
        for (const Entry &entry : snapshot.entries) {
            if (entry.factory != nullptr && entry.key == key) {
                return &entry;
            }
        }
        return nullptr;
    }

    static Entry *FindEmptyEntry(Snapshot &snapshot)
    {
        for (Entry &entry : snapshot.entries) {
            if (entry.factory == nullptr) {
                return &entry;
            }
        }
        return nullptr;
    }

    static Entry *FindWritableEntry(Snapshot &snapshot, Key key)
    {
        Entry *entry = FindEntry(snapshot, key);
        return entry != nullptr ? entry : FindEmptyEntry(snapshot);
    }

    static Factory FindInSnapshot(const Snapshot &snapshot, Key key)
    {
        const Entry *entry = FindEntry(snapshot, key);
        return entry != nullptr ? entry->factory : nullptr;
    }

    bool TryRegister(Key key, Factory factory)
    {
        std::shared_ptr<const Snapshot> current = LoadSnapshot();
        while (current) {
            const RegisterStep step = TryRegisterOnce(current, key, factory);
            if (step != RegisterStep::Retry) {
                return step == RegisterStep::Registered;
            }
        }
        return false;
    }

    enum class RegisterStep {
        Retry,
        Full,
        Registered,
    };

    RegisterStep TryRegisterOnce(
        std::shared_ptr<const Snapshot> &current, Key key, Factory factory)
    {
        std::shared_ptr<Snapshot> next = BuildNextSnapshot(
            *current, key, factory);
        if (!next) {
            current.reset();
            return RegisterStep::Full;
        }
        return ReplaceSnapshot(current, next) ? RegisterStep::Registered :
                                               RegisterStep::Retry;
    }

    static std::shared_ptr<Snapshot> BuildNextSnapshot(
        const Snapshot &current, Key key, Factory factory)
    {
        auto next = std::make_shared<Snapshot>(current);
        return Upsert(*next, key, factory) ? next : nullptr;
    }

    std::shared_ptr<const Snapshot> LoadSnapshot() const
    {
        return std::atomic_load_explicit(&m_snapshot,
                                         std::memory_order_acquire);
    }

    bool ReplaceSnapshot(std::shared_ptr<const Snapshot> &expected,
                         std::shared_ptr<const Snapshot> next)
    {
        return std::atomic_compare_exchange_weak_explicit(
            &m_snapshot, &expected, std::move(next), std::memory_order_acq_rel,
            std::memory_order_acquire);
    }

    std::shared_ptr<const Snapshot> m_snapshot;
};

} // namespace SmartDrone::Adapters::Slam
