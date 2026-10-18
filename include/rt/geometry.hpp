// rt/geometry.hpp — зоны пересечения роботов: AABB и OBB (выпуклые параллелепипеды).
// Тест отрезок×бокс через slab-метод в локальных координатах OBB.
// Только стековые POD, никаких аллокаций.
#pragma once
#include <array>
#include <cmath>
#include <cstddef>

namespace rt {

struct Vec3 {
    double x = 0, y = 0, z = 0;
    constexpr Vec3() noexcept = default;
    constexpr Vec3(double px, double py, double pz) noexcept : x(px), y(py), z(pz) {}
    constexpr Vec3 operator+(const Vec3& o) const noexcept { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(const Vec3& o) const noexcept { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator*(double k) const noexcept { return {x * k, y * k, z * k}; }
    constexpr double dot(const Vec3& o) const noexcept { return x * o.x + y * o.y + z * o.z; }
    constexpr double len() const noexcept { return std::sqrt(dot(*this)); }
};

struct AABB {
    Vec3 lo, hi;
    bool inside(const Vec3& p) const noexcept {
        return p.x >= lo.x && p.x <= hi.x && p.y >= lo.y && p.y <= hi.y && p.z >= lo.z && p.z <= hi.z;
    }
};

// OBB: центр c, ортонормированные локальные оси ax[3], полустороны half вдоль осей.
struct OBB {
    Vec3 c;
    std::array<Vec3, 3> ax;
    Vec3 half; // полустороны: half.x/half.y/half.z вдоль ax[0], ax[1], ax[2]
    bool inside(const Vec3& p) const noexcept {
        const Vec3 d = p - c;
        const double h[3] = {half.x, half.y, half.z};
        for (int i = 0; i < 3; ++i)
            if (std::fabs(d.dot(ax[i])) > h[i]) return false;
        return true;
    }
};

// Пересечение отрезка [a, b] с боксом. Возвращает параметр [t0, t1] ⊆ [0, 1]
// вдоль отрезка и true, если пересечение есть. t = 0 у a, 1 у b.
inline bool seg_box_hit(const Vec3& a, const Vec3& b, const AABB& box,
                        double& t0, double& t1) noexcept {
    const Vec3 d = b - a;
    t0 = 0.0; t1 = 1.0;
    const double lo[3] = {box.lo.x, box.lo.y, box.lo.z};
    const double hi[3] = {box.hi.x, box.hi.y, box.hi.z};
    const double p[3]  = {a.x, a.y, a.z};
    const double dd[3] = {d.x, d.y, d.z};
    for (int i = 0; i < 3; ++i) {
        if (std::fabs(dd[i]) < 1e-12) {
            if (p[i] < lo[i] || p[i] > hi[i]) return false;
            continue;
        }
        double ta = (lo[i] - p[i]) / dd[i];
        double tb = (hi[i] - p[i]) / dd[i];
        if (ta > tb) { const double tmp = ta; ta = tb; tb = tmp; }
        if (ta > t0) t0 = ta;
        if (tb < t1) t1 = tb;
        if (t0 > t1) return false;
    }
    return true;
}

inline bool seg_box_hit(const Vec3& a, const Vec3& b, const OBB& box,
                        double& t0, double& t1) noexcept {
    // Переводим отрезок в локальные координаты OBB (проекция на оси).
    const Vec3 d = b - a;
    const Vec3 pa = a - box.c;
    double p[3], dd[3];
    for (int i = 0; i < 3; ++i) {
        p[i]  = pa.dot(box.ax[i]);
        dd[i] = d.dot(box.ax[i]);
    }
    t0 = 0.0; t1 = 1.0;
    const double h[3] = {box.half.x, box.half.y, box.half.z};
    for (int i = 0; i < 3; ++i) {
        if (std::fabs(dd[i]) < 1e-12) {
            if (p[i] < -h[i] || p[i] > h[i]) return false;
            continue;
        }
        double ta = (-h[i] - p[i]) / dd[i];
        double tb = ( h[i] - p[i]) / dd[i];
        if (ta > tb) { const double tmp = ta; ta = tb; tb = tmp; }
        if (ta > t0) t0 = ta;
        if (tb < t1) t1 = tb;
        if (t0 > t1) return false;
    }
    return true;
}

// Полилиния фиксированной длины (стековый POD). pts[0..n), длины сегментов — cumulative.
struct Path {
    std::array<Vec3, 8> pts{};
    size_t n = 0;                  // число точек (n-1 сегментов), 2..8
    std::array<double, 7> cum{};   // cum[i] = длина пути до pts[i], cum[0] = 0
    double total = 0;

    constexpr Path() noexcept = default;
    static Path make(const std::array<Vec3, 8>& pts, size_t n) noexcept {
        Path p; p.n = n;
        p.pts = pts;
        p.cum[0] = 0.0;
        for (size_t i = 1; i < n; ++i) {
            const double l = (pts[i] - pts[i - 1]).len();
            p.cum[i] = p.cum[i - 1] + l;
        }
        p.total = p.cum[n - 1];
        return p;
    }
    // Точка на пути на длине дуги s (детерминированно, без аллокаций).
    Vec3 point_at(double s) const noexcept {
        if (s <= 0.0) return pts[0];
        if (s >= total) return pts[n - 1];
        size_t i = 1;
        while (i < n - 1 && s > cum[i]) ++i;
        const double seg = cum[i] - cum[i - 1];
        const double k = seg > 0 ? (s - cum[i - 1]) / seg : 0.0;
        return pts[i - 1] + (pts[i] - pts[i - 1]) * k;
    }
    // Интервал длины дуги [s0, s1] внутри бокса. true, если путь касается бокса.
    template <typename Box>
    bool occupancy(const Box& box, double& s0, double& s1) const noexcept {
        bool found = false;
        s0 = 1e300; s1 = -1e300;
        for (size_t i = 1; i < n; ++i) {
            double ta = 0, tb = 0;
            if (!seg_box_hit(pts[i - 1], pts[i], box, ta, tb)) continue;
            const double l = cum[i] - cum[i - 1];
            const double sa = cum[i - 1] + ta * l;
            const double sb = cum[i - 1] + tb * l;
            if (sa < s0) s0 = sa;
            if (sb > s1) s1 = sb;
            found = true;
        }
        return found;
    }
};

} // namespace rt
