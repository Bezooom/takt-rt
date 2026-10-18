// rt/traj.hpp — аналитический 7-сегментный S-curve профиль (jerk-limited, C++20).
// Ограничения vmax/amax/jmax; вырожденные случаи: треугольный профиль (без cruise
// и без const-acc фаз) и промежуточный (T2 > 0, без cruise). Всё на стеке,
// позиция/скорость — закрытые формы кубических полиномов по сегментам.
#pragma once
#include <array>
#include <cmath>
#include <cstddef>
#include <cassert>

namespace rt {

struct Limits {
    double vmax = 0, amax = 0, jmax = 0;
};

struct Seg {
    double T = 0;     // длительность сегмента
    double j  = 0;    // jerk на сегменте (d a / d t)
    double a0 = 0;    // ускорение в начале сегмента
    double v0 = 0;    // скорость в начале сегмента
    double s0 = 0;    // позиция (длина дуги) в начале сегмента
};

struct SCurve {
    Limits lim{};
    double D = 0;           // полный путь
    double v_peak = 0;      // пиковая скорость
    double T_total = 0;     // полное время
    int nseg = 0;
    std::array<Seg, 7> seg{};

    static constexpr double kEps = 1e-9;

    double pos(double t) const noexcept {
        t = clamp_t(t);
        size_t i = 0;
        double acc = 0;
        while (i < static_cast<size_t>(nseg) - 1 && t > acc + seg[i].T) { acc += seg[i].T; ++i; }
        const double tau = t - acc;
        return seg[i].s0 + seg[i].v0 * tau + seg[i].a0 * tau * tau / 2.0
             + seg[i].j * tau * tau * tau / 6.0;
    }
    double vel(double t) const noexcept {
        t = clamp_t(t);
        size_t i = 0;
        double acc = 0;
        while (i < static_cast<size_t>(nseg) - 1 && t > acc + seg[i].T) { acc += seg[i].T; ++i; }
        const double tau = t - acc;
        return seg[i].v0 + seg[i].a0 * tau + seg[i].j * tau * tau / 2.0;
    }
    // Инверсия: время достижения длины дуги s (монотонный pos). Бисекция, 32 шага,
    // детерминированно, без аллокаций.
    double time_at(double s) const noexcept {
        double lo = 0.0, hi = T_total;
        for (int it = 0; it < 32; ++it) {
            const double mid = 0.5 * (lo + hi);
            if (pos(mid) < s) lo = mid; else hi = mid;
        }
        return 0.5 * (lo + hi);
    }
private:
    constexpr double clamp_t(double t) const noexcept {
        if (t < 0) return 0;
        if (t > T_total) return T_total;
        return t;
    }
};

// Построение профиля на путь D при лимитах L. Три ветки (сверено Python-моделью):
//   1. Cruise достижим: если vmax ≤ amax²/jmax → разгон только jerk-фазами
//      (T1c = sqrt(vmax/jmax), T2c = 0, s_ramp = jmax·T1c³); иначе T1c = amax/jmax,
//      T2c = (vmax − jmax·T1c²)/amax, s_ramp = v1(T1c+T2c) + amax(T1c²/2+T1c·T2c+T2c²/2).
//      Условие 2·s_ramp ≤ D → cruise-фаза Tc = (D − 2·s_ramp)/vmax.
//   2. Промежуточный (cruise нет, D ≥ 2·jmax·t1³, t1 = amax/jmax): T1 = t1,
//      T2 из кв. уравнения s_ramp(T2) = D/2; v_peak = jmax·t1² + amax·T2 < vmax
//      автоматически (T2 < t2_vmax по монотонности).
//   3. Треугольный (D < 2·jmax·t1³): T1 = cbrt(D/(2·jmax)) < t1,
//      v_peak = jmax·T1² ≤ vmax автоматически (в обеих подветках cruise).
inline SCurve make_scurve(double D, Limits L) noexcept {
    assert(D > 0 && L.vmax > 0 && L.amax > 0 && L.jmax > 0);
    const double t1 = L.amax / L.jmax;

    double T1 = 0, T2 = 0, Tc = 0, v_peak = 0;
    {
        const double t1c = (L.vmax * L.jmax <= L.amax * L.amax)
            ? std::sqrt(L.vmax / L.jmax) : t1;                       // T2c = 0 в первом случае
        const double t2c = (L.vmax * L.jmax <= L.amax * L.amax)
            ? 0.0 : (L.vmax - L.jmax * t1 * t1) / L.amax;
        const double v1c = L.jmax * t1c * t1c / 2.0;
        const double s_ramp_c = (t2c == 0.0)
            ? L.jmax * t1c * t1c * t1c
            : v1c * (t1c + t2c) + L.amax * (t1c * t1c / 2.0 + t1c * t2c + t2c * t2c / 2.0);
        if (2.0 * s_ramp_c <= D + SCurve::kEps) {
            T1 = t1c; T2 = t2c;
            v_peak = L.vmax;
            Tc = (D - 2.0 * s_ramp_c) / v_peak;
        } else if (D >= 2.0 * L.jmax * t1 * t1 * t1 - SCurve::kEps) {
            // Промежуточный: s_ramp(T2) = D/2:
            // (amax/2)T2² + (v1 + amax·t1)T2 + (v1·t1 + amax·t1²/2 − D/2) = 0
            const double v1 = L.jmax * t1 * t1 / 2.0;
            const double a2 = L.amax / 2.0;
            const double b  = v1 + L.amax * t1;
            const double c  = v1 * t1 + L.amax * t1 * t1 / 2.0 - D / 2.0;
            const double disc = b * b - 4.0 * a2 * c;
            T1 = t1;
            T2 = (disc <= 0) ? 0.0 : (-b + std::sqrt(disc)) / (2.0 * a2);
            if (T2 < 0) T2 = 0;
            v_peak = L.jmax * t1 * t1 + L.amax * T2;
        } else {
            // Треугольный: T1 < t1 автоматически, v_peak ≤ amax²/jmax ≤ ... ≤ vmax
            T1 = std::cbrt(D / (2.0 * L.jmax));
            v_peak = L.jmax * T1 * T1;
        }
    }

    // 7 сегментов (торможение — зеркало разгона):
    //   [j+ T1][acc T2][j− T1][cruise Tc][j− T1][dec T2][j+ T1]
    // Интегрирование сегмента (T, j, a0, v0, s0):
    //   v_end = v0 + a0·T + j·T²/2;  s_end = s0 + v0·T + a0·T²/2 + j·T³/6
    std::array<Seg, 7> s{};
    double v_end = 0, s_end = 0;
    const double jmax = L.jmax;
    // Разгон
    s[0] = {T1,  jmax,  0,        0,     0};
    v_end = jmax * T1 * T1 / 2.0;
    s_end = jmax * T1 * T1 * T1 / 6.0;
    s[1] = {T2,  0,     jmax*T1,  v_end, s_end};
    const double v1e = v_end + (jmax*T1) * T2;                       // конец const-acc
    s_end += v_end * T2 + (jmax*T1) * T2 * T2 / 2.0;
    const double s1e = s_end;
    s[2] = {T1, -jmax,  jmax*T1,  v1e,   s1e};
    v_end = v1e + (jmax*T1) * T1 - jmax * T1 * T1 / 2.0;             // a → 0
    s_end += v1e * T1 + (jmax*T1) * T1 * T1 / 2.0 - jmax * T1 * T1 * T1 / 6.0;
    const double s2e = s_end;                                        // конец разгона
    // Cruise
    s[3] = {Tc,  0,     0,        v_end, s2e};
    s_end += v_end * Tc;
    const double s3e = s_end;
    // Торможение
    s[4] = {T1, -jmax,  0,        v_end, s3e};
    const double v4e = v_end - jmax * T1 * T1 / 2.0;
    s_end += v_end * T1 - jmax * T1 * T1 * T1 / 6.0;
    const double s4e = s_end;
    s[5] = {T2,  0,    -jmax*T1,  v4e,   s4e};
    const double v5e = v4e - (jmax*T1) * T2;
    s_end += v4e * T2 - (jmax*T1) * T2 * T2 / 2.0;
    const double s5e = s_end;
    s[6] = {T1,  jmax, -jmax*T1,  v5e,   s5e};
    v_end = v5e - (jmax*T1) * T1 + jmax * T1 * T1 / 2.0;             // → 0
    s_end += v5e * T1 - (jmax*T1) * T1 * T1 / 2.0 + jmax * T1 * T1 * T1 / 6.0;

    SCurve tr{};
    tr.lim = L; tr.D = D; tr.v_peak = v_peak;
    tr.nseg = 7;
    for (int i = 0; i < 7; ++i) tr.seg[i] = s[i];
    tr.T_total = T1 * 4 + T2 * 2 + Tc;
    (void)v_end; (void)s_end; (void)v4e; (void)v5e; (void)s1e; (void)s2e; (void)s3e; (void)s4e; (void)s5e;
    // Инварианты (проверяются тестом): pos(T_total) == D, vel(T_total) == 0,
    // v_peak ≤ vmax во всех трёх ветках.
    return tr;
}

} // namespace rt
