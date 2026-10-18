// rt/scheduler.hpp — детерминированный планировщик ячейки (C++20, zero-allocation).
//
// Две подсистемы:
//  1. Job-Shop: операции → станки ЧПУ (списковое планирование ресурсов, монотонные busy_until).
//  2. Пространственно-временные коллизии роботов в общих зонах (AABB/OBB):
//     перекрытие интервалов пребывания в зоне → пересчёт профиля жертвы
//     (снижение vmax шагами 0.85 → Respeed) или сдвиг фазы старта (DelayStart).
//     Цикл ячейки НЕ останавливается: жертва едет медленнее/позже, остальные нетронуты.
//
// Всё на стеке: std::array фиксированной ёмкости, без new/malloc, детерминированный
// порядок обхода пар (по индексам массива) → воспроизводимое расписание.
#pragma once
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>

#include "rt/protocol.hpp"
#include "rt/geometry.hpp"
#include "rt/traj.hpp"

namespace rt {

enum class Mitigation : u32 { None = 0, Respeed = 1, DelayStart = 2, Failed = 3 };

struct RobotCell {
    u8 id = 0;
    u8 priority = 0;         // 0 = высший; при конфликте корректируется больший номер
    Limits lim{};            // vmax/amax/jmax физики привода
    Vec3 tcp{};              // текущая точка инструмента
    Path path{};             // текущий сегмент пути [tcp → target]
    SCurve traj{};           // jerk-ограниченный профиль по длине пути
    double t_start = 0;      // фаза старта пути (абсолютное время, с)
    bool active = false;     // есть непогашенный путь
};

struct Zone {
    bool is_obb = false;
    AABB aabb{};
    OBB obb{};
};

struct Machine {             // ресурс job-shop: станок ЧПУ
    u8 id = 0;
    double busy_until = 0;   // монотонная занятость
};

struct TInterval { double t0 = 0, t1 = 0; bool valid = false; };

struct AdmitReport {
    bool ok = false;
    Mitigation mitigation = Mitigation::None;
    double finish_at = 0;    // расчётное время завершения пути
};

// Планировщик ячейки. Не потокобезопасен сам (планирование — один контекст,
// обычно consumer SPSC-очереди); статистика — атомарная для читателей из других потоков.
struct Scheduler {
    static constexpr size_t kMaxRobots = 3;
    static constexpr size_t kMaxMachines = 4;
    static constexpr size_t kMaxZones = 4;
    static constexpr double kTimeEps = 1e-9;

    std::array<RobotCell, kMaxRobots> robots{};
    size_t n_robots = 0;
    std::array<Zone, kMaxZones> zones{};
    size_t n_zones = 0;
    std::array<Machine, kMaxMachines> machines{};
    size_t n_machines = 0;

    std::atomic<u32> jobs_planned{0};
    std::atomic<u32> collisions{0};
    std::atomic<u32> mit_respeed{0};
    std::atomic<u32> mit_delay{0};
    std::atomic<u32> mit_failed{0};

    void add_robot(u8 id, Limits lim, u8 priority, Vec3 tcp = Vec3{}) noexcept {
        robots[n_robots] = RobotCell{};
        robots[n_robots].id = id;
        robots[n_robots].priority = priority;
        robots[n_robots].lim = lim;
        robots[n_robots].tcp = tcp;
        ++n_robots;
    }
    void add_zone_aabb(const AABB& b) noexcept {
        zones[n_zones] = Zone{false, b, OBB{}};
        ++n_zones;
    }
    void add_zone_obb(const OBB& b) noexcept {
        zones[n_zones] = Zone{true, AABB{}, b};
        ++n_zones;
    }
    void add_machine(u8 id) noexcept {
        machines[n_machines] = Machine{id, 0.0};
        ++n_machines;
    }

    // ---- Деактивация завершённых путей (tcp = факт. точка) ---------------------
    void tick(double now) noexcept {
        for (size_t i = 0; i < n_robots; ++i) {
            RobotCell& r = robots[i];
            if (!r.active) continue;
            if (r.t_start + r.traj.T_total <= now) {
                r.tcp = r.path.pts[r.path.n - 1];
                r.active = false;
            }
        }
    }

    // ---- Job-Shop: операция на станок, старт = max(busy_until, ready) ----------
    // Возвращает время старта или -1 при неизвестном станке.
    double schedule_job(u8 machine_id, double dur, double ready_at) noexcept {
        for (size_t i = 0; i < n_machines; ++i) {
            if (machines[i].id != machine_id) continue;
            const double bu = machines[i].busy_until;
            const double start = bu > ready_at ? bu : ready_at;
            machines[i].busy_until = start + dur;
            jobs_planned.fetch_add(1, std::memory_order_relaxed);
            return start;
        }
        return -1.0;
    }

    // ---- Интервал пребывания робота в зоне ------------------------------------
    static TInterval zone_window(const RobotCell& r, const Zone& z) noexcept {
        double s0 = 0, s1 = 0;
        const bool hit = z.is_obb
            ? r.path.occupancy(z.obb, s0, s1)
            : r.path.occupancy(z.aabb, s0, s1);
        if (!hit) return TInterval{};
        return TInterval{ r.t_start + r.traj.time_at(s0),
                          r.t_start + r.traj.time_at(s1), true };
    }

    // ---- Разрешение пары роботов в одной зоне ---------------------------------
    // victim определяется по приоритету (больший номер корректируется; при равных
    // — робот b, как более поздний). Пересчёт: 6 попыток снижения vmax (×0.85),
    // затем сдвиг фазы старта на длину перекрытия. Профиль скорости всегда
    // ненулевой: цикл не останавливается.
    static bool pair_overlap(const RobotCell& a, const RobotCell& b,
                             const Zone& z, double& ovl) noexcept {
        const TInterval ia = zone_window(a, z);
        const TInterval ib = zone_window(b, z);
        if (!ia.valid || !ib.valid) return false;
        const double lo = ia.t0 > ib.t0 ? ia.t0 : ib.t0;
        const double hi = ia.t1 < ib.t1 ? ia.t1 : ib.t1;
        ovl = hi - lo;
        return ovl > kTimeEps;
    }

    // victim определяется по приоритету (больший номер корректируется);
    // при равных — поздний (b) как более поздний по приёму.
    static RobotCell* pick_victim(RobotCell& a, RobotCell& b) noexcept {
        if (a.priority != b.priority)
            return a.priority > b.priority ? &a : &b;
        return &b;
    }

    static Mitigation mitigate(RobotCell& va, RobotCell& vb, const Zone& z) noexcept {
        // va — victim; vb — оставшийся нетронутым.
        for (int attempt = 1; attempt <= 6; ++attempt) {
            Limits lb = va.lim;
            lb.vmax = va.lim.vmax * std::pow(0.85, static_cast<double>(attempt));
            if (lb.vmax <= 1e-6) break;
            va.traj = make_scurve(va.path.total, lb);
            double ovl = 0;
            if (!pair_overlap(vb, va, z, ovl))
                return Mitigation::Respeed;      // профиль снижен, фаза прежняя
        }
        // Respeed не развёл интервалы → сдвиг фазы старта на длину перекрытия.
        double ovl = 0;
        if (!pair_overlap(vb, va, z, ovl))
            return Mitigation::Respeed;          // уже разведено последней попыткой
        va.t_start += ovl + 1e-6;
        double check = 0;
        return pair_overlap(vb, va, z, check) ? Mitigation::Failed : Mitigation::DelayStart;
    }

    // Полный прогон разрешения конфликтов: все пары роботов × зоны,
    // детерминированный порядок, до 8 проходов (результат итеративный).
    // last_mit — последний применённый к жертве тип коррекции.
    bool resolve_all(Mitigation& last_mit) noexcept {
        last_mit = Mitigation::None;
        for (int pass = 0; pass < 8; ++pass) {
            bool changed = false;
            for (size_t zi = 0; zi < n_zones; ++zi)
                for (size_t i = 0; i < n_robots; ++i)
                    for (size_t j = i + 1; j < n_robots; ++j) {
                        RobotCell& ra = robots[i];
                        RobotCell& rb = robots[j];
                        if (!ra.active || !rb.active) continue;
                        double ovl = 0;
                        if (!pair_overlap(ra, rb, zones[zi], ovl)) continue;
                        collisions.fetch_add(1, std::memory_order_relaxed);
                        RobotCell& va_ref = *pick_victim(ra, rb);
                        RobotCell& vb_ref = (&va_ref == &ra) ? rb : ra;
                        const Mitigation m = mitigate(va_ref, vb_ref, zones[zi]);
                        if (m == Mitigation::Failed) { mit_failed.fetch_add(1, std::memory_order_relaxed); return false; }
                        last_mit = m;
                        if (m == Mitigation::Respeed) mit_respeed.fetch_add(1, std::memory_order_relaxed);
                        if (m == Mitigation::DelayStart) mit_delay.fetch_add(1, std::memory_order_relaxed);
                        changed = true;
                    }
            if (!changed) return true;
        }
        mit_failed.fetch_add(1, std::memory_order_relaxed);
        return false;                            // не сошлось за 8 проходов
    }

    // ---- Приём нового пути робота ---------------------------------------------
    // Путь = [tcp → target]; профиль S-curve по длине пути; затем полный прогон
    // разрешения коллизий. Активный робот ПЕРЕПЛАНИРУЕТСЯ на лету: tcp = текущая
    // позиция по старому пути, цикл не останавливается. Возвращает статус.
    AdmitReport admit_traj(u8 robot_id, const Vec3& target, double now) noexcept {
        AdmitReport rep{};
        RobotCell* self = nullptr;
        for (size_t i = 0; i < n_robots; ++i)
            if (robots[i].id == robot_id) { self = &robots[i]; break; }
        if (!self) { rep.mitigation = Mitigation::Failed; return rep; }

        // Перепланирование активного: фактическая позиция на старом пути.
        if (self->active) {
            const double dt = now - self->t_start;
            const double tt = dt < 0 ? 0.0 : (dt > self->traj.T_total ? self->traj.T_total : dt);
            self->tcp = self->path.point_at(self->traj.pos(tt));
            self->active = false;
        }

        std::array<Vec3, 8> pts{};
        pts[0] = self->tcp;
        pts[1] = target;
        self->path = Path::make(pts, 2);
        if (self->path.total <= 0.0) { rep.mitigation = Mitigation::Failed; return rep; }
        self->traj = make_scurve(self->path.total, self->lim);
        self->t_start = now;
        self->active = true;

        Mitigation last_mit = Mitigation::None;
        if (!resolve_all(last_mit)) {
            self->active = false;
            rep.mitigation = Mitigation::Failed;
            return rep;
        }

        rep.ok = true;
        rep.finish_at = self->t_start + self->traj.T_total;
        rep.mitigation = last_mit;
        return rep;
    }

    // ---- Планирование из кадра протокола ---------------------------------------
    // kMsgTrajCmd: coords[0..2] = target (x,y,z, метры, Q16.16). Робот = robot_id.
    AdmitReport plan_from_frame(const ParsedFrame& f, u8 robot_id, double now) noexcept {
        if (f.type != kMsgTrajCmd || f.axes < 3) {
            return AdmitReport{false, Mitigation::Failed, 0.0};
        }
        const Vec3 target{ f.coords[0].real(), f.coords[1].real(), f.coords[2].real() };
        return admit_traj(robot_id, target, now);
    }
};

} // namespace rt
