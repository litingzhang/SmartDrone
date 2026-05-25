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

        std::shared_ptr<const Snapshot> current = LoadSnapshot();
        while (current) {
            auto next = std::make_shared<Snapshot>(*current);
            if (!Upsert(*next, key, factory)) {
                return false;
            }
            if (ReplaceSnapshot(current, std::move(next))) {
                return true;
            }
        }
        return false;
    }

    Factory Find(Key key) const
    {
        std::shared_ptr<const Snapshot> snapshot = LoadSnapshot();
        if (!snapshot) {
            return nullptr;
        }
        for (const Entry &entry : snapshot->entries) {
            if (entry.factory != nullptr && entry.key == key) {
                return entry.factory;
            }
        }
        return nullptr;
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
        for (Entry &entry : snapshot.entries) {
            if (entry.factory != nullptr && entry.key == key) {
                entry.factory = factory;
                return true;
            }
        }
        for (Entry &entry : snapshot.entries) {
            if (entry.factory == nullptr) {
                entry.key = key;
                entry.factory = factory;
                return true;
            }
        }
        return false;
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
