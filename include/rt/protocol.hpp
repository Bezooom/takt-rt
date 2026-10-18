// rt/protocol.hpp — Zero-Allocation бинарный промышленный протокол (C++20).
// Формат кадра (big-endian):
//   magic(2) ver(1) type(1) len(2) seq(2) ts_ms(4) axis_mask(4) axes(2) | coords axes*4 (Q16.16) | crc16(2)
// Заголовок 18 байт, максимальный кадр 84. Никаких new/malloc после init:
// только std::span, std::array, alignas, побайтовая сборка (нет UB aliasing).
#pragma once
#include <array>
#include <cstdint>
#include <cstddef>
#include <span>
#include <cassert>

namespace rt {

using u8  = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using i32 = std::int32_t;
using u64 = std::uint64_t;

// ---- Константы протокола ---------------------------------------------------
constexpr u16 kMagic      = 0x434Du;          // "CM"
constexpr u8  kVersion    = 1;
constexpr u8  kMsgTrajCmd = 1;                // команда перемещения по траектории
constexpr u8  kMsgHeartbt = 2;                // heartbeat
constexpr size_t kMaxAxes    = 16;
constexpr size_t kFrameHdr   = 18;
constexpr size_t kFrameCrc   = 2;
constexpr size_t kFrameMax   = kFrameHdr + kMaxAxes * 4 + kFrameCrc;

// Q16.16 fixed point: 1 байт = 1/65536 единицы, диапазон ±32767.
struct Q1616 {
    i32 raw;
    static constexpr double to_real(i32 r) noexcept { return static_cast<double>(r) / 65536.0; }
    constexpr double real() const noexcept { return to_real(raw); }
    static constexpr Q1616 from_real(double v) noexcept {
        // saturate в диапазон i32, детерминированно
        double s = v * 65536.0;
        if (s > 2147483647.0)  s = 2147483647.0;
        if (s < -2147483648.0) s = -2147483648.0;
        return Q1616{ static_cast<i32>(s) };
    }
    constexpr bool operator==(const Q1616&) const noexcept = default;
};

// ---- CRC-16-CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflect, xorout 0 -----
namespace crc16_impl {
    constexpr u16 step(u16 crc, u8 b) noexcept {
        crc = static_cast<u16>(crc ^ (static_cast<u16>(b) << 8));
        for (int i = 0; i < 8; ++i)
            crc = static_cast<u16>((crc & 0x8000u) ? ((crc << 1) ^ 0x1021u) : (crc << 1));
        return crc;
    }
    // Однобайтовые остатки (init 0x0000) — каноничная табличная схема.
    constexpr std::array<u16, 256> build_table() noexcept {
        std::array<u16, 256> t{};
        for (size_t i = 0; i < 256; ++i)
            t[i] = step(0x0000, static_cast<u8>(i));
        return t;
    }
}
inline constexpr std::array<u16, 256> kCrcTable = crc16_impl::build_table();

constexpr u16 crc16_ccitt(std::span<const u8> data) noexcept {
    u16 crc = 0xFFFF;
    for (u8 b : data)
        crc = static_cast<u16>((crc << 8) ^ kCrcTable[((crc >> 8) ^ b) & 0xFFu]);
    return crc;
}

// ---- Кольцевой байтовый буфер парсера (статический, стек/класс) -------------
template <size_t N>
struct ByteRing {
    static_assert((N & (N - 1)) == 0, "capacity must be power of two");
    std::array<u8, N> a{};
    size_t head = 0, used = 0;                       // однопоточный контекст парсера
    constexpr size_t cap() const noexcept { return N; }
    size_t push(std::span<const u8> src) noexcept {  // вернуть число реально записанных
        size_t n = 0;
        for (u8 b : src) {
            if (used == N) break;
            a[(head + used) & (N - 1)] = b;
            ++used; ++n;
        }
        return n;
    }
    constexpr u8 peek(size_t i) const noexcept { return a[(head + i) & (N - 1)]; }
    void pop_front(size_t n) noexcept {
        n = n > used ? used : n;
        head = (head + n) & (N - 1);
        used -= n;
    }
    constexpr size_t size() const noexcept { return used; }
};

// ---- Результат парсинга -----------------------------------------------------
enum class PStatus : u32 { NeedMore, Ok, CrcError, LengthMismatch, Resync };

struct ParsedFrame {
    u8   ver  = 0;
    u8   type = 0;
    u16  seq  = 0;
    u32  ts_ms = 0;
    u32  axis_mask = 0;              // битовое поле состояния осей (1 = ось активна/в движении)
    std::array<Q1616, kMaxAxes> coords{}; // координаты осей, Q16.16
    u16  axes = 0;
    std::array<u8, kFrameMax> raw{};  // копия кадра (для логов/верификации)
    size_t raw_n = 0;
};

static_assert(std::is_trivially_copyable_v<ParsedFrame>);
static_assert(sizeof(ParsedFrame) < 512);

// ---- Парсер: байтовый поток → кадры -----------------------------------------
// Детерминированный конечный автомат: ищем magic, валидируем длину, считаем CRC.
// При мусоре — побайтовый ресинк (skip to next magic), без аллокаций.
template <size_t RingN = 256>
struct FrameParser {
    static_assert(RingN >= 2 * kFrameMax, "ring must hold two max frames");
    ByteRing<RingN> ring{};
    u32 stat_ok = 0, stat_crc = 0, stat_len = 0, stat_resync = 0;

    size_t feed(std::span<const u8> bytes) noexcept { return ring.push(bytes); }

    template <typename OnFrame>
    PStatus parse(OnFrame&& on_frame) noexcept {
        // 1. Ресинк: пропуск мусора до magic
        while (ring.size() >= 2 && !(ring.peek(0) == 0x43 && ring.peek(1) == 0x4D)) {
            ring.pop_front(1);
            ++stat_resync;
            if (ring.size() < 2) return PStatus::Resync;
        }
        if (ring.size() < 2) return PStatus::NeedMore;
        // 2. Заголовок фиксированной длины → стековый буфер
        if (ring.size() < kFrameHdr) return PStatus::NeedMore;
        std::array<u8, kFrameMax> buf{};
        for (size_t i = 0; i < kFrameHdr; ++i) buf[i] = ring.peek(i);
        const u16 len = rd16(buf, 4);
        if (len > kMaxAxes * 4) { ++stat_len; ring.pop_front(1); return PStatus::LengthMismatch; } // ресинк 1 байт
        const size_t total = kFrameHdr + len + kFrameCrc;
        if (ring.size() < total) return PStatus::NeedMore;
        for (size_t i = kFrameHdr; i < total; ++i) buf[i] = ring.peek(i);
        // 3. CRC по [0 .. kFrameHdr+len)
        const u16 crc_calc = crc16_ccitt(std::span<const u8>(buf.data(), kFrameHdr + len));
        const u16 crc_wire = static_cast<u16>(buf[total - 2]) << 8 | buf[total - 1];
        const bool bad = (crc_calc != crc_wire) || (buf[0] != 0x43) || (buf[1] != 0x4D);
        // 4. Поля
        ParsedFrame f;
        f.ver   = buf[2];
        f.type  = buf[3];
        f.seq   = rd16(buf, 6);
        f.ts_ms = rd32(buf, 8);
        f.axis_mask = rd32(buf, 12);
        f.axes  = rd16(buf, 16);
        if (len % 4 != 0 || f.axes * 4 != len || f.axes > kMaxAxes) {
            ++stat_len; ring.pop_front(1); return PStatus::LengthMismatch;
        }
        for (size_t i = 0; i < f.axes; ++i)
            f.coords[i] = Q1616{ static_cast<i32>(rd32(buf, kFrameHdr + i * 4)) };
        f.raw_n = total;
        for (size_t i = 0; i < total; ++i) f.raw[i] = buf[i];
        ring.pop_front(total);
        if (bad) { ++stat_crc; return PStatus::CrcError; }
        ++stat_ok;
        on_frame(f);
        return PStatus::Ok;
    }
private:
    static constexpr u16 rd16(const std::array<u8, kFrameMax>& b, size_t o) noexcept {
        return static_cast<u16>(static_cast<u16>(b[o]) << 8 | b[o + 1]);
    }
    static constexpr u32 rd32(const std::array<u8, kFrameMax>& b, size_t o) noexcept {
        return static_cast<u32>(static_cast<u32>(b[o]) << 24 | static_cast<u32>(b[o+1]) << 16
                              | static_cast<u32>(b[o+2]) << 8  | b[o+3]);
    }
};

// ---- Энкодер (для тестов и стенда) ------------------------------------------
struct FrameWriter {
    std::array<u8, kFrameMax> buf{};
    size_t n = 0;
    FrameWriter() noexcept { buf[0] = 0x43; buf[1] = 0x4D; buf[2] = kVersion; }

    void header(u8 type, u16 seq, u32 ts_ms, u32 axis_mask, u16 axes) noexcept {
        buf[3] = type;
        wr16(4, static_cast<u16>(axes * 4));
        wr16(6, seq);
        wr32(8, ts_ms);
        wr32(12, axis_mask);
        wr16(16, axes);
        n = kFrameHdr;
    }
    void coord(Q1616 v) noexcept {
        const i32 r = v.raw;
        buf[n + 0] = static_cast<u8>(static_cast<u32>(r) >> 24 & 0xFFu);
        buf[n + 1] = static_cast<u8>(static_cast<u32>(r) >> 16 & 0xFFu);
        buf[n + 2] = static_cast<u8>(static_cast<u32>(r) >> 8  & 0xFFu);
        buf[n + 3] = static_cast<u8>(static_cast<u32>(r)       & 0xFFu);
        n += 4;
    }
    std::span<const u8> finish() noexcept {
        const u16 crc = crc16_ccitt(std::span<const u8>(buf.data(), n));
        buf[n]     = static_cast<u8>(crc >> 8);
        buf[n + 1] = static_cast<u8>(crc & 0xFFu);
        n += kFrameCrc;
        return std::span<const u8>(buf.data(), n);
    }
    std::span<const u8> frame_so_far() const noexcept { return std::span<const u8>(buf.data(), n); }
private:
    void wr16(size_t o, u16 v) noexcept {
        buf[o]     = static_cast<u8>(v >> 8);
        buf[o + 1] = static_cast<u8>(v & 0xFFu);
    }
    void wr32(size_t o, u32 v) noexcept {
        buf[o]     = static_cast<u8>(v >> 24 & 0xFFu);
        buf[o + 1] = static_cast<u8>(v >> 16 & 0xFFu);
        buf[o + 2] = static_cast<u8>(v >> 8  & 0xFFu);
        buf[o + 3] = static_cast<u8>(v       & 0xFFu);
    }
};

// ---- Выравнивание структур, уходящих в SPSC --------------------------------
struct alignas(64) FrameMsg {          // одна кэш-линия на сообщение
    ParsedFrame frame;
    u32 source_robot = 0;
};

} // namespace rt
