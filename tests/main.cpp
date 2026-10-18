// tests/main.cpp — верификация: битый пакет (CRC), коллизия траекторий в зоне,
// стресс SPSC (гонки/дедлоки), zero-allocation guard (глобальный operator new).
// Сборки: plain (с guard), ASan+UBSan, TSan (guard отключён через RT_NO_ALLOC_GUARD).
#include <atomic>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <thread>

#include "rt/protocol.hpp"
#include "rt/spsc.hpp"
#include "rt/geometry.hpp"
#include "rt/traj.hpp"
#include "rt/scheduler.hpp"

using namespace rt;

#ifndef RT_STRESS_N
#define RT_STRESS_N 100000
#endif

// ---- Zero-allocation guard: глобальные операторы (только plain-сборка) -------
#ifndef RT_NO_ALLOC_GUARD
#include <new>
#include <memory>
namespace rt_hooks {
    std::atomic<bool> g_alloc_locked{false};
    std::atomic<unsigned long long> g_alloc_count{0};
}
void* operator new(std::size_t n) {
    if (rt_hooks::g_alloc_locked.load(std::memory_order_relaxed)) throw std::bad_alloc();
    void* p = std::malloc(n ? n : 1);
    if (!p) throw std::bad_alloc();
    rt_hooks::g_alloc_count.fetch_add(1, std::memory_order_relaxed);
    return p;
}
void* operator new[](std::size_t n) {
    if (rt_hooks::g_alloc_locked.load(std::memory_order_relaxed)) throw std::bad_alloc();
    void* p = std::malloc(n ? n : 1);
    if (!p) throw std::bad_alloc();
    rt_hooks::g_alloc_count.fetch_add(1, std::memory_order_relaxed);
    return p;
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
// nothrow-варианты используются библиотечными заголовками:
void* operator new(std::size_t n, const std::nothrow_t&) noexcept {
    if (rt_hooks::g_alloc_locked.load(std::memory_order_relaxed)) return nullptr;
    void* p = std::malloc(n ? n : 1);
    rt_hooks::g_alloc_count.fetch_add(1, std::memory_order_relaxed);
    return p;
}
void* operator new[](std::size_t n, const std::nothrow_t&) noexcept {
    if (rt_hooks::g_alloc_locked.load(std::memory_order_relaxed)) return nullptr;
    void* p = std::malloc(n ? n : 1);
    rt_hooks::g_alloc_count.fetch_add(1, std::memory_order_relaxed);
    return p;
}
#endif

// ---- Микро-фреймворк --------------------------------------------------------
static int g_fail = 0;
static const char* g_cur = "";
#define CHECK(cond, what) do { if (!(cond)) { ++g_fail; std::printf("  FAIL [%s]: %s\n", g_cur, what); } } while (0)
#define CHECKF(cond, what, val) do { if (!(cond)) { ++g_fail; std::printf("  FAIL [%s]: %s (%.9g)\n", g_cur, what, static_cast<double>(val)); } } while (0)
static void begin(const char* name) { g_cur = name; std::printf("[test] %s\n", name); }

template <typename F>
static void expect_no_alloc(const char* what, F&& f) {
#ifndef RT_NO_ALLOC_GUARD
    rt_hooks::g_alloc_count.store(0, std::memory_order_relaxed);
    rt_hooks::g_alloc_locked.store(true, std::memory_order_relaxed);
    try { f(); } catch (const std::bad_alloc&) {
        ++g_fail; std::printf("  FAIL [%s]: dynamic allocation in %s\n", g_cur, what);
    }
    rt_hooks::g_alloc_locked.store(false, std::memory_order_relaxed);
#else
    (void)what; f();
#endif
}

// ---- 1. CRC и парсер --------------------------------------------------------
static void test_protocol() {
    begin("protocol: CRC-16-CCITT + roundtrip + битый пакет");
    {
        constexpr std::array<u8, 9> probe{'1','2','3','4','5','6','7','8','9'};
        // CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflect, no augment):
        // check "123456789" = 0x29B1 — сверено с binascii.crc_hqx(data, 0xFFFF) (Python stdlib)
        // и каталогом sigurn/crc16. (0xE5CC — вариант с явной аугментацией 16 нулями.)
        static_assert(crc16_ccitt(std::span<const u8>(probe.data(), probe.size())) == 0x29B1,
                      "CRC-16/CCITT-FALSE check value");
        CHECK(crc16_ccitt(std::span<const u8>(probe.data(), probe.size())) == 0x29B1, "check value 0x29B1");
    }
    // Roundtrip: 3 оси, значения в Q16.16
    {
        FrameWriter w;
        w.header(kMsgTrajCmd, 42, 1000, 0x0000000Fu, 3);
        w.coord(Q1616::from_real(1.5));
        w.coord(Q1616::from_real(-2.25));
        w.coord(Q1616::from_real(0.125));
        const auto bytes = w.finish();

        FrameParser<256> p;
        p.feed(bytes);
        int seen = 0;
        const PStatus st = p.parse([&](const ParsedFrame& f) {
            ++seen;
            CHECK(f.ver == kVersion, "version");
            CHECK(f.type == kMsgTrajCmd, "type");
            CHECK(f.seq == 42, "seq");
            CHECK(f.ts_ms == 1000, "ts_ms");
            CHECK(f.axis_mask == 0x0000000Fu, "axis_mask");
            CHECK(f.axes == 3, "axes");
            CHECK(f.coords[0].raw == Q1616::from_real(1.5).raw, "x Q16.16");
            CHECK(f.coords[1].raw == Q1616::from_real(-2.25).raw, "y Q16.16");
            CHECK(f.coords[2].raw == Q1616::from_real(0.125).raw, "z Q16.16");
        });
        CHECK(st == PStatus::Ok, "status Ok");
        CHECK(seen == 1, "on_frame called once");
    }
    // Битый пакет: инверсия байта payload → CrcError, on_frame НЕ вызывается
    {
        FrameWriter w;
        w.header(kMsgTrajCmd, 7, 500, 0x1u, 3);
        w.coord(Q1616::from_real(1.0));
        w.coord(Q1616::from_real(0.0));
        w.coord(Q1616::from_real(0.0));
        auto bytes = w.finish();
        std::array<u8, kFrameMax> corrupt{};
        for (size_t b = 0; b < bytes.size(); ++b) corrupt[b] = bytes[b];
        corrupt[20] = static_cast<u8>(corrupt[20] ^ 0xFFu); // порча payload

        FrameParser<256> p;
        p.feed(corrupt);
        int seen = 0;
        const PStatus st = p.parse([&](const ParsedFrame&) { ++seen; });
        CHECK(st == PStatus::CrcError, "status CrcError");
        CHECK(seen == 0, "corrupt frame never reaches scheduler");
        CHECK(p.stat_crc == 1, "crc stat");
    }
    // LengthMismatch: axes=60 → len=240 > макс
    {
        FrameWriter w;
        w.header(kMsgTrajCmd, 9, 100, 0, 60);
        const auto bytes = w.finish(); // CRC посчитан по 18 байт заголовка — неважно
        FrameParser<256> p;
        p.feed(bytes);
        const PStatus st = p.parse([](const ParsedFrame&) {});
        CHECK(st == PStatus::LengthMismatch, "status LengthMismatch");
        CHECK(p.stat_len == 1, "len stat");
    }
    // Ресинк: мусор до magic
    {
        FrameWriter w;
        w.header(kMsgTrajCmd, 3, 10, 0, 1);
        w.coord(Q1616::from_real(0.5));
        const auto bytes = w.finish();

        FrameParser<256> p;
        const std::array<u8, 4> junk{0xDE, 0xAD, 0xBE, 0xEF};
        p.feed(junk);
        p.feed(bytes);
        int seen = 0;
        const PStatus st = p.parse([&](const ParsedFrame& f) { ++seen; CHECK(f.seq == 3, "frame after resync"); });
        CHECK(st == PStatus::Ok, "status Ok after resync");
        CHECK(seen == 1, "resync found frame");
        CHECK(p.stat_resync > 0, "resync stat");
    }
}

// ---- 2. S-curve инварианты --------------------------------------------------
static void check_scurve_invariants(const SCurve& tr, double vmax, double amax, double jmax) {
    const double eps = 1e-6;
    CHECKF(std::fabs(tr.pos(tr.T_total) - tr.D) < 1e-9, "pos(T)==D", tr.pos(tr.T_total) - tr.D);
    CHECKF(std::fabs(tr.vel(tr.T_total)) < 1e-9, "vel(T)==0", tr.vel(tr.T_total));
    CHECK(tr.T_total > 0, "finite duration");
    // Сетка 200 точек: скорость ≤ vmax+eps, ускорение ≤ amax+eps, jerk ≤ jmax+eps
    const int N = 200;
    for (int i = 0; i <= N; ++i) {
        const double t = tr.T_total * i / N;
        if (!(tr.vel(t) <= vmax + eps))  { CHECKF(tr.vel(t) <= vmax + eps, "vmax breach", tr.vel(t)); break; }
        // ускорение по сегментам:
        const auto acc_at = [&](double tt) {
            size_t k = 0; double acc = 0;
            while (k < size_t(tr.nseg) - 1 && tt > acc + tr.seg[k].T) { acc += tr.seg[k].T; ++k; }
            const double tau = tt - acc;
            return tr.seg[k].a0 + tr.seg[k].j * tau;
        };
        if (!(std::fabs(acc_at(t)) <= amax + eps)) { CHECKF(std::fabs(acc_at(t)) <= amax + eps, "amax breach", acc_at(t)); break; }
        for (int si = 0; si < tr.nseg; ++si)
            if (!(std::fabs(tr.seg[si].j) <= jmax + eps)) { CHECKF(std::fabs(tr.seg[si].j) <= jmax + eps, "jmax breach", tr.seg[si].j); break; }
    }
    // Инверсия time_at: pos(time_at(s)) ≈ s
    for (int i = 0; i <= 10; ++i) {
        const double s = tr.D * i / 10.0;
        const double err = tr.pos(tr.time_at(s)) - s;
        CHECKF(std::fabs(err) <= 1e-5, "time_at inversion", err);
        if (std::fabs(err) > 1e-5) break;
    }
}
static void test_scurve() {
    begin("scurve: jerk-limited инварианты (3 ветки: cruise/промежуточный/треугольный)");
    const Limits L{2.0, 4.0, 20.0};
    {
        SCurve tr = make_scurve(10.0, L);            // cruise (полный трапецеидальный)
        CHECK(tr.v_peak <= L.vmax + 1e-9, "v_peak ≤ vmax (cruise)");
        check_scurve_invariants(tr, L.vmax, L.amax, L.jmax);
    }
    {
        SCurve tr = make_scurve(0.5, L);             // промежуточный (T2>0, без cruise)
        CHECK(tr.v_peak <= L.vmax + 1e-9, "v_peak ≤ vmax (intermediate)");
        check_scurve_invariants(tr, L.vmax, L.amax, L.jmax);
    }
    {
        SCurve tr = make_scurve(0.1, L);             // треугольный (jerk-фазы только)
        CHECK(tr.v_peak <= L.vmax + 1e-9, "v_peak ≤ vmax (triangle)");
        check_scurve_invariants(tr, L.vmax, L.amax, L.jmax);
    }
    {
        // t2_vmax < 0: даже полный разгон до amax даёт v > vmax → бисекция T1
        const Limits Ls{0.3, 4.0, 20.0};             // vmax² = 0.09 < amax²/jmax = 0.8
        SCurve tr = make_scurve(1.0, Ls);
        CHECK(tr.v_peak <= Ls.vmax + 1e-9, "v_peak ≤ vmax (clamped)");
        check_scurve_invariants(tr, Ls.vmax, Ls.amax, Ls.jmax);
    }
}

// ---- 3. Геометрия: slab-тест ------------------------------------------------
static void test_geometry() {
    begin("geometry: AABB/OBB пересечение отрезка с боксом");
    {
        const AABB box{{4, -0.5, 0}, {6, 3.5, 0}};
        double t0 = -1, t1 = -1;
        const bool hit = seg_box_hit(Vec3{0, 0, 0}, Vec3{10, 0, 0}, box, t0, t1);
        CHECK(hit, "AABB hit");
        CHECKF(std::fabs(t0 - 0.4) < 1e-12, "t0 == 0.4", t0 - 0.4);   // x=4 при движении 0→10
        CHECKF(std::fabs(t1 - 0.6) < 1e-12, "t1 == 0.6", t1 - 0.6);
        const bool miss = seg_box_hit(Vec3{0, 5, 0}, Vec3{10, 5, 0}, box, t0, t1);
        CHECK(!miss, "AABB miss");
    }
    {
        // OBB повёрнут на 45° вокруг Z, центр (5, 1.5)
        const double c45 = std::sqrt(2.0) / 2.0;
        OBB box;
        box.c = Vec3{5, 1.5, 0};
        box.ax = {Vec3{c45, c45, 0}, Vec3{-c45, c45, 0}, Vec3{0, 0, 1}};
        box.half = Vec3{2.83, 2.83, 1.0};
        double t0 = -1, t1 = -1;
        const bool hit = seg_box_hit(Vec3{0, 0, 0}, Vec3{10, 0, 0}, box, t0, t1);
        CHECK(hit, "OBB hit");
        const bool miss = seg_box_hit(Vec3{0, 10, 0}, Vec3{10, 10, 0}, box, t0, t1);
        CHECK(!miss, "OBB miss");
    }
}

// ---- 4. Коллизия двух траекторий в зоне -------------------------------------
static bool intervals_overlap(const TInterval& a, const TInterval& b) {
    return a.valid && b.valid && (std::min(a.t1, b.t1) - std::max(a.t0, b.t0)) > 1e-9;
}
static void test_collision() {
    begin("collision: конфликт в зоне → mitigation, робот A нетронут, цикл не остановлен");
    for (int obb_mode = 0; obb_mode < 2; ++obb_mode) {
        Scheduler sched;
        const Limits L{2.0, 4.0, 20.0};
        sched.add_robot(1, L, 0);
        sched.add_robot(2, L, 1, Vec3{0, 3, 0});
        if (obb_mode == 0) {
            sched.add_zone_aabb(AABB{{4, -0.5, 0}, {6, 3.5, 0}});
        } else {
            const double c45 = std::sqrt(2.0) / 2.0;
            OBB box;
            box.c = Vec3{5, 1.5, 0};
            box.ax = {Vec3{c45, c45, 0}, Vec3{-c45, c45, 0}, Vec3{0, 0, 1}};
            box.half = Vec3{2.83, 2.83, 1.0};
            sched.add_zone_obb(box);
        }
        const Zone& z = sched.zones[0];

        // Робот 1 (высший приоритет): (0,0,0) → (10,0,0), старт t=0.
        const AdmitReport r1 = sched.admit_traj(1, Vec3{10, 0, 0}, 0.0);
        CHECK(r1.ok, "robot1 admitted");
        CHECK(r1.mitigation == Mitigation::None, "robot1 no mitigation");
        const RobotCell snap1 = sched.robots[0]; // снапшот ДО admit робота 2

        // Робот 2 (низкий приоритет): (0,3,0) → (10,3,0), старт t=0 → конфликт.
        const AdmitReport r2 = sched.admit_traj(2, Vec3{10, 3, 0}, 0.0);
        CHECK(r2.ok, "robot2 admitted");
        CHECK(sched.collisions.load() >= 1, "collision detected");
        CHECK(r2.mitigation == Mitigation::Respeed || r2.mitigation == Mitigation::DelayStart,
              "mitigation applied");
        CHECK(r2.mitigation != Mitigation::Failed, "not failed");

        // AC2a: зона свободна — интервалы роботов не пересекаются.
        const TInterval i1 = Scheduler::zone_window(sched.robots[0], z);
        const TInterval i2 = Scheduler::zone_window(sched.robots[1], z);
        CHECK(!intervals_overlap(i1, i2), "zone conflict resolved");

        // AC2b: робот 1 нетронут (побайтово).
        const RobotCell& a1 = sched.robots[0];
        const bool untouched =
            a1.t_start == snap1.t_start &&
            a1.traj.D == snap1.traj.D && a1.traj.T_total == snap1.traj.T_total &&
            a1.traj.v_peak == snap1.traj.v_peak && a1.path.total == snap1.path.total;
        CHECK(untouched, "robot1 profile unchanged");

        // AC2c: оба доехали (полный путь, не остановлены).
        CHECKF(std::fabs(sched.robots[0].traj.pos(sched.robots[0].traj.T_total) - 10.0) < 1e-9,
               "robot1 reached target", sched.robots[0].traj.pos(sched.robots[0].traj.T_total) - 10.0);
        CHECKF(std::fabs(sched.robots[1].traj.pos(sched.robots[1].traj.T_total) - 10.0) < 1e-9,
               "robot2 reached target", sched.robots[1].traj.pos(sched.robots[1].traj.T_total) - 10.0);
        CHECK(std::isfinite(r2.finish_at), "finite finish time");

        // AC2d: без полной остановки — скорость робота 2 в зоне ненулевая.
        if (i2.valid) {
            const double tm = 0.5 * (i2.t0 + i2.t1) - sched.robots[1].t_start;
            const double v = sched.robots[1].traj.vel(tm);
            CHECK(v > 0.1, "robot2 keeps moving through zone (no full stop)");
        }
    }
}

// ---- 5. Job-Shop ------------------------------------------------------------
static void test_jobshop() {
    begin("jobshop: монотонные ресурсы станков, без перекрытий");
    Scheduler sched;
    sched.add_machine(1);
    sched.add_machine(2);
    // 3 операции на станок 1 (dur=1), готовность 0:
    const double s11 = sched.schedule_job(1, 1.0, 0.0);
    const double s12 = sched.schedule_job(1, 1.0, 0.0);
    const double s13 = sched.schedule_job(1, 1.0, 0.0);
    // 1 операция на станок 2 (dur=2), параллельно:
    const double s21 = sched.schedule_job(2, 2.0, 0.0);
    CHECK(s11 == 0.0 && s12 == 1.0 && s13 == 2.0, "machine1 serialization");
    CHECK(s21 == 0.0, "machine2 parallel start");
    CHECK(sched.jobs_planned.load() == 4, "jobs planned");
    CHECK(sched.schedule_job(9, 1.0, 0.0) < 0, "unknown machine rejected");
}

// ---- 6. Стресс SPSC + парсер + планировщик ----------------------------------
static void test_spsc_stress() {
    begin("spsc stress: producer/consumer, 50% битых кадров, порядок seq, no-alloc");
    struct Shared {
        SpscRing<FrameMsg, 64> q{};
        std::atomic<u32> produced{0};
        std::atomic<u32> consumed{0};
        std::atomic<u32> ok_frames{0};
        std::atomic<u32> crc_frames{0};
        std::atomic<bool> producer_done{false};
        std::atomic<u32> planned{0};
        std::atomic<u32> rejected{0};
    };
    Shared sh{};

    const u32 N = RT_STRESS_N;
    // Producer: кадры для 3 роботов, 50% битые (инверсия байта payload).
    std::thread prod([&] {
        u32 i = 0;
        while (i < N) {
            FrameMsg m{};
            FrameWriter fw;
            const u8 rid = static_cast<u8>((i % 3u) + 1u);   // роботы 1..3
            const double target_x = (i % 2u == 0) ? 1.0 : 0.0;
            const double target_y = (rid == 0) ? 0.0 : ((rid == 1) ? 1.0 : 0.0);
            const double target_z = (rid == 2) ? 1.0 : 0.0;
            fw.header(kMsgTrajCmd, static_cast<u16>(i & 0xFFFFu), i * 60u, 0x1u << rid, 3);
            fw.coord(Q1616::from_real(target_x));
            fw.coord(Q1616::from_real(target_y));
            fw.coord(Q1616::from_real(target_z));
            auto bytes = fw.finish();
            for (size_t b = 0; b < bytes.size(); ++b) m.frame.raw[b] = bytes[b];
            if (i & 1u) m.frame.raw[20] = static_cast<u8>(m.frame.raw[20] ^ 0xFFu); // битый CRC
            m.frame.raw_n = bytes.size();
            m.source_robot = rid;
            while (!sh.q.try_push(m)) { /* spin: очередь полна */ }
            sh.produced.fetch_add(1, std::memory_order_relaxed);
            ++i;
        }
        sh.producer_done.store(true, std::memory_order_release);
    });

    // Consumer: парсинг → планирование (tick → plan_from_frame).
    std::thread cons([&] {
        FrameParser<256> parser;
        Scheduler sched;
        const Limits L{2.0, 4.0, 20.0};
        sched.add_robot(1, L, 0);
        sched.add_robot(2, L, 1);
        sched.add_robot(3, L, 2);
        sched.add_zone_aabb(AABB{{-0.05, -0.05, -0.05}, {0.05, 0.05, 0.05}});
        u16 last_seq = 0; bool first = true;
        u32 left = N;
        expect_no_alloc("consumer loop", [&] {
            while (left > 0) {
                FrameMsg m{};
                if (!sh.q.try_pop(m)) continue;
                parser.feed(std::span<const u8>(m.frame.raw.data(), m.frame.raw_n));
                PStatus st;
                while ((st = parser.parse([&](const ParsedFrame& f) {
                    const double now = static_cast<double>(f.ts_ms) / 1000.0;
                    sched.tick(now);
                    const AdmitReport rep = sched.plan_from_frame(f, m.source_robot, now);
                    if (rep.ok) sh.planned.fetch_add(1, std::memory_order_relaxed);
                    else sh.rejected.fetch_add(1, std::memory_order_relaxed);
                    // порядок seq у валидных кадров строгий (нет перестановок):
                    // ok-кадры идут через один (шаг seq == 2), потерь и перестановок нет
                    if (first) { last_seq = f.seq; first = false; }
                    else { const u16 step = static_cast<u16>(f.seq - last_seq);
                           CHECK(step == 2, "seq strictly ordered, no loss/permutation");
                           last_seq = f.seq; }
                })) == PStatus::Ok) {
                    sh.ok_frames.fetch_add(1, std::memory_order_relaxed);
                }
                if (st == PStatus::CrcError) sh.crc_frames.fetch_add(1, std::memory_order_relaxed);
                sh.consumed.fetch_add(1, std::memory_order_relaxed);
                --left;
            }
        });
    });

    prod.join();
    cons.join();

    CHECK(sh.produced.load() == N, "all produced");
    CHECK(sh.consumed.load() == N, "all consumed (no deadlock/loss)");
    const u32 ok = sh.ok_frames.load();
    CHECK(ok + sh.crc_frames.load() == N, "parsed = ok + crc");
    CHECK(sh.crc_frames.load() == N / 2, "half corrupted by design");
    CHECK(sh.planned.load() + sh.rejected.load() == ok, "every valid frame planned/rejected");
    std::printf("  stress: N=%u ok=%u crc=%u planned=%u rejected=%u\n",
                N, ok, sh.crc_frames.load(), sh.planned.load(), sh.rejected.load());
}

// ---- main -------------------------------------------------------------------
int main() {
    test_protocol();
    test_scurve();
    test_geometry();
    test_collision();
    test_jobshop();
    test_spsc_stress();

    std::printf("\n=== RESULT: %s (%d failures) ===\n", g_fail == 0 ? "PASS" : "FAIL", g_fail);
    return g_fail == 0 ? 0 : 1;
}
