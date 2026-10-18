// rt/spsc.hpp — Lock-free Single-Producer/Single-Consumer ring buffer (C++20).
// Классический Lamport-контур с монотонными uint64-индексами:
//   producer: head_.load(acquire) [full-проверка] → запись в слот → tail_.store(release)
//   consumer: tail_.load(acquire) [empty-проверка] → чтение слота → head_.store(release)
// Никаких CAS, мьютексов, аллокаций: фиксированный std::array, power-of-two,
// индексы и буфер на разных кэш-линиях (alignas(64)) против false sharing.
#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <cstddef>
#include <type_traits>

namespace rt {

template <typename T, size_t Cap>
requires std::is_trivially_copyable_v<T> && ((Cap & (Cap - 1)) == 0) && (Cap >= 2)
struct SpscRing {
    SpscRing() noexcept = default;
    SpscRing(const SpscRing&) = delete;
    SpscRing& operator=(const SpscRing&) = delete;

    // Producer: возвращает false, если очередь заполнена (никогда не блокирует).
    bool try_push(const T& v) noexcept {
        const u64 h = head_.load(std::memory_order_acquire);
        const u64 t = tail_.load(std::memory_order_relaxed);
        if (t - h == Cap) return false;
        buf_[t & (Cap - 1)] = v;
        tail_.store(t + 1, std::memory_order_release);
        return true;
    }

    // Consumer: возвращает false, если очередь пуста (никогда не блокирует).
    bool try_pop(T& out) noexcept {
        const u64 t = tail_.load(std::memory_order_acquire);
        const u64 h = head_.load(std::memory_order_relaxed);
        if (h == t) return false;
        out = buf_[h & (Cap - 1)];
        head_.store(h + 1, std::memory_order_release);
        return true;
    }

    // Только для статистики/тестов (не в hot-path контракте):
    u64 produced() const noexcept { return tail_.load(std::memory_order_relaxed); }
    u64 consumed() const noexcept { return head_.load(std::memory_order_relaxed); }

private:
    alignas(64) std::atomic<u64> head_{0};   // принадлежит consumer'у
    alignas(64) std::atomic<u64> tail_{0};   // принадлежит producer'у
    alignas(64) std::array<T, Cap> buf_{};
};

} // namespace rt
