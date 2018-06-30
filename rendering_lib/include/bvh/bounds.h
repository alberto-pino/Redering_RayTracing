//#if defined(_MSC_VER)
//#define NOMINMAX
//#pragma once
//#endif

#ifndef __BOUNDS_H__
#define __BOUNDS_H__

#include <geometry.h>

// Bounds Declarations
template <typename T>
class Bounds2 {
    #if _MSC_VER < 1900
        typedef gmtl::Point<T, 2> Point2T;
        typedef gmtl::Vec<T, 2> Vector2T;
    #else
        typedef Point2<T> Point2T;
        typedef Vector2<T> Vector2T;
    #endif

public:
    // Bounds2 Public Methods
    Bounds2() {
        T minNum = std::numeric_limits<T>::lowest();
        T maxNum = std::numeric_limits<T>::max();
        pMin = Point2T(maxNum, maxNum);
        pMax = Point2T(minNum, minNum);
    }
    explicit Bounds2(const Point2T &p) : pMin(p), pMax(p) {}
    Bounds2(const Point2T &p1, const Point2T &p2) {
        pMin = Point2T(std::min(p1[0], p2[0]), std::min(p1[1], p2[1]));
        pMax = Point2T(std::max(p1[0], p2[0]), std::max(p1[1], p2[1]));
    }
    //template <typename U>
    //explicit operator Bounds2<U>() const {
    //    return Bounds2<U>((Point2<U>)pMin, (Point2<U>)pMax);
    //}

    Vector2T Diagonal() const { return pMax - pMin; }
    T Area() const {
        Vector2T d = pMax - pMin;
        return (d[0] * d[1]);
    }
    int MaximumExtent() const {
        Vector2T diag = Diagonal();
        if (diag[0] > diag[1])
            return 0;
        else
            return 1;
    }
    inline const Point2T &operator[](int i) const {
        DCHECK(i == 0 || i == 1);
        return (i == 0) ? pMin : pMax;
    }
    inline Point2T &operator[](int i) {
        DCHECK(i == 0 || i == 1);
        return (i == 0) ? pMin : pMax;
    }
    bool operator==(const Bounds2<T> &b) const {
        return b.pMin == pMin && b.pMax == pMax;
    }
    bool operator!=(const Bounds2<T> &b) const {
        return b.pMin != pMin || b.pMax != pMax;
    }
    Point2T Lerp(const Point2f &t) const {
        return Point2T(Lerp(t[0], pMin[0], pMax[0]),
            Lerp(t[1], pMin[1], pMax[1]));
    }
    Vector2T Offset(const Point2T &p) const {
        Vector2T o = p - pMin;
        if (pMax[0] > pMin[0]) o[0] /= pMax[0] - pMin[0];
        if (pMax[1] > pMin[1]) o[1] /= pMax[1] - pMin[1];
        return o;
    }

    void Add(const Point2T& p)
    {
        for (int i = 0; i < 2; ++i)
        {
            if (p[i] < pMin[i])
                pMin[i] = p[i];
            if (p[i] > pMax[i])
                pMax[i] = p[i];
        }
    }

    void BoundingSphere(Point2T *c, float *rad) const {
        *c = (pMin + pMax) / 2;
        *rad = Inside(*c, *this) ? Distance(*c, pMax) : 0;
    }
    friend std::ostream &operator<<(std::ostream &os, const Bounds2<T> &b) {
        os << "[ " << b.pMin << " - " << b.pMax << " ]";
        return os;
    }

    // Bounds2 Public Data
    Point2T pMin, pMax;
};

template <typename T>
class Bounds3 {
    #if _MSC_VER < 1900
        typedef gmtl::Point<T, 3> Point3T;
        typedef gmtl::Vec<T, 3> Vector3T;
    #else
        typedef Point3<T> Point3T;
        typedef Vector3<T> Vector3T;
    #endif
public:
    // Bounds3 Public Methods
    Bounds3() {
        T minNum = std::numeric_limits<T>::lowest();
        T maxNum = std::numeric_limits<T>::max();
        pMin = Point3T(maxNum, maxNum, maxNum);
        pMax = Point3T(minNum, minNum, minNum);
    }
    explicit Bounds3(const Point3T &p) : pMin(p), pMax(p) {}
    Bounds3(const Point3T &p1, const Point3T &p2)
        : pMin(std::min(p1[0], p2[0]), std::min(p1[1], p2[1]),
            std::min(p1[2], p2[2])),
        pMax(std::max(p1[0], p2[0]), std::max(p1[1], p2[1]),
            std::max(p1[2], p2[2])) {}
    const Point3T &operator[](int i) const;
    Point3T &operator[](int i);
    bool operator==(const Bounds3<T> &b) const {
        return b.pMin == pMin && b.pMax == pMax;
    }
    bool operator!=(const Bounds3<T> &b) const {
        return b.pMin != pMin || b.pMax != pMax;
    }
    Point3T Corner(int corner) const {
        DCHECK(corner >= 0 && corner < 8);
        return Point3T((*this)[(corner & 1)][0],
            (*this)[(corner & 2) ? 1 : 0][1],
            (*this)[(corner & 4) ? 1 : 0][2]);
    }
    Vector3T Diagonal() const { return pMax - pMin; }
    T SurfaceArea() const {
        Vector3T d = Diagonal();
        return 2 * (d[0] * d[1] + d[0] * d[2] + d[1] * d[2]);
    }

    const Point3T& Center() const { return pCenter;}
    Vector3T HalfSize() const { return (pMax - pMin) / 2.0f;}
    Vector3T Size() const { return (pMax - pMin);}


    T Volume() const {
        Vector3T d = Diagonal();
        return d[0] * d[1] * d[2];
    }

    int MaximumExtent() const {
        Vector3T d = Diagonal();
        if (d[0] > d[1] && d[0] > d[2])
            return 0;
        else if (d[1] > d[2])
            return 1;
        else
            return 2;
    }
    Point3T Lerp(const Point3f &t) const {
        return Point3T(Lerp(t[0], pMin[0], pMax[0]),
            Lerp(t[1], pMin[1], pMax[1]),
            Lerp(t[2], pMin[2], pMax[2]));
    }
    Vector3T Offset(const Point3T &p) const {
        Vector3T o = p - pMin;
        if (pMax[0] > pMin[0]) o[0] /= pMax[0] - pMin[0];
        if (pMax[1] > pMin[1]) o[1] /= pMax[1] - pMin[1];
        if (pMax[2] > pMin[2]) o[2] /= pMax[2] - pMin[2];
        return o;
    }

    void Add(const Point3T& p)
    {
        for (int i = 0; i < 3; ++i)
        {
            if (p[i] < pMin[i])
                pMin[i] = p[i];
            if (p[i] > pMax[i])
                pMax[i] = p[i];
        }
    }

    void Add(const Bounds3<T>& bounds)
    {
        Add(bounds.pMin);
        Add(bounds.pMax);
    }

    void BoundingSphere(Point3T *center, float *radius) const {
        *center = (pMin + pMax) / 2;
        *radius = Inside(*center, *this) ? Distance(*center, pMax) : 0;
    }
    //template <typename U>
    //explicit operator Bounds3<U>() const {
    //    return Bounds3<U>((Point3<U>)pMin, (Point3<U>)pMax);
    //}
    bool intersect(const Ray &ray, float *hitt0 = nullptr,
        float *hitt1 = nullptr) const;
    inline bool intersect(const Ray &ray, const Vector3f &invDir,
        const int dirIsNeg[3]) const;
    friend std::ostream &operator<<(std::ostream &os, const Bounds3<T> &b) {
        os << "[ " << b.pMin << " - " << b.pMax << " ]";
        return os;
    }

    // Bounds3 Public Data
    Point3T pMin, pMax;
    Point3T pCenter;
};

typedef Bounds2<float> Bounds2f;
typedef Bounds2<int> Bounds2i;
typedef Bounds3<float> Bounds3f;
typedef Bounds3<int> Bounds3i;


#if _MSC_VER < 1900
    template <typename T>
    inline const gmtl::Point<T,3> &Bounds3<T>::operator[](int i) const {
        DCHECK(i == 0 || i == 1);
        return (i == 0) ? pMin : pMax;
    }
    template <typename T>
    inline gmtl::Point<T,3> &Bounds3<T>::operator[](int i) {
        DCHECK(i == 0 || i == 1);
        return (i == 0) ? pMin : pMax;
    }

    template <typename T>
    Bounds3<T> Union(const Bounds3<T> &b, const gmtl::Point<T,3> &p) {
        return Bounds3<T>(
            gmtl::Point<T,3>(std::min(b.pMin[0], p[0]), std::min(b.pMin[1], p[1]),
                std::min(b.pMin[2], p[2])),
            gmtl::Point<T,3>(std::max(b.pMax[0], p[0]), std::max(b.pMax[1], p[1]),
                std::max(b.pMax[2], p[2])));
    }

    template <typename T>
    bool Inside(const gmtl::Point<T,3> &p, const Bounds3<T> &b) {
        return (p[0] >= b.pMin[0] && p[0] <= b.pMax[0] && p[1] >= b.pMin[1] &&
            p[1] <= b.pMax[1] && p[2] >= b.pMin[2] && p[2] <= b.pMax[2]);
    }

    template <typename T>
    bool InsideExclusive(const gmtl::Point<T,3> &p, const Bounds3<T> &b) {
        return (p[0] >= b.pMin[0] && p[0] < b.pMax[0] && p[1] >= b.pMin[1] &&
            p[1] < b.pMax[1] && p[2] >= b.pMin[2] && p[2] < b.pMax[2]);
    }

    // Minimum squared distance from point to box; returns zero if point is
    // inside.
    template <typename T, typename U>
    inline float DistanceSquared(const gmtl::Point<T,3> &p, const Bounds3<U> &b) {
        float dx = std::max({ float(0), b.pMin[0] - p[0], p[0] - b.pMax[0] });
        float dy = std::max({ float(0), b.pMin[1] - p[1], p[1] - b.pMax[1] });
        float dz = std::max({ float(0), b.pMin[2] - p[2], p[2] - b.pMax[2] });
        return dx * dx + dy * dy + dz * dz;
    }

    template <typename T, typename U>
    inline float Distance(const gmtl::Point<T,3> &p, const Bounds3<U> &b) {
        return std::sqrt(DistanceSquared(p, b));
    }

    template <typename T>
    Bounds3<T> Union(const Bounds3<T> &b1, const Bounds3<T> &b2) {
        return Bounds3<T>(gmtl::Point<T,3>(std::min(b1.pMin[0], b2.pMin[0]),
            std::min(b1.pMin[1], b2.pMin[1]),
            std::min(b1.pMin[2], b2.pMin[2])),
            gmtl::Point<T,3>(std::max(b1.pMax[0], b2.pMax[0]),
                std::max(b1.pMax[1], b2.pMax[1]),
                std::max(b1.pMax[2], b2.pMax[2])));
    }

    template <typename T>
    Bounds3<T> Intersect(const Bounds3<T> &b1, const Bounds3<T> &b2) {
        return Bounds3<T>(gmtl::Point<T,3>(std::max(b1.pMin[0], b2.pMin[0]),
            std::max(b1.pMin[1], b2.pMin[1]),
            std::max(b1.pMin[2], b2.pMin[2])),
            gmtl::Point<T,3>(std::min(b1.pMax[0], b2.pMax[0]),
                std::min(b1.pMax[1], b2.pMax[1]),
                std::min(b1.pMax[2], b2.pMax[2])));
    }
#else
    template <typename T>
    inline const Point3<T> &Bounds3<T>::operator[](int i) const {
        DCHECK(i == 0 || i == 1);
        return (i == 0) ? pMin : pMax;
    }
    template <typename T>
    inline Point3<T> &Bounds3<T>::operator[](int i) {
        DCHECK(i == 0 || i == 1);
        return (i == 0) ? pMin : pMax;
    }

    template <typename T>
    Bounds3<T> Union(const Bounds3<T> &b, const Point3<T> &p) {
        return Bounds3<T>(
            Point3<T>(std::min(b.pMin[0], p[0]), std::min(b.pMin[1], p[1]),
                std::min(b.pMin[2], p[2])),
            Point3<T>(std::max(b.pMax[0], p[0]), std::max(b.pMax[1], p[1]),
                std::max(b.pMax[2], p[2])));
    }
    template <typename T>
    bool Inside(const Point3<T> &p, const Bounds3<T> &b) {
        return (p[0] >= b.pMin[0] && p[0] <= b.pMax[0] && p[1] >= b.pMin[1] &&
            p[1] <= b.pMax[1] && p[2] >= b.pMin[2] && p[2] <= b.pMax[2]);
    }

    template <typename T>
    bool InsideExclusive(const Point3<T> &p, const Bounds3<T> &b) {
        return (p[0] >= b.pMin[0] && p[0] < b.pMax[0] && p[1] >= b.pMin[1] &&
            p[1] < b.pMax[1] && p[2] >= b.pMin[2] && p[2] < b.pMax[2]);
    }

    // Minimum squared distance from point to box; returns zero if point is
    // inside.
    template <typename T, typename U>
    inline float DistanceSquared(const Point3<T> &p, const Bounds3<U> &b) {
        float dx = std::max({ float(0), b.pMin[0] - p[0], p[0] - b.pMax[0] });
        float dy = std::max({ float(0), b.pMin[1] - p[1], p[1] - b.pMax[1] });
        float dz = std::max({ float(0), b.pMin[2] - p[2], p[2] - b.pMax[2] });
        return dx * dx + dy * dy + dz * dz;
    }

    template <typename T, typename U>
    inline float Distance(const Point3<T> &p, const Bounds3<U> &b) {
        return std::sqrt(DistanceSquared(p, b));
    }

    template <typename T>
    Bounds3<T> Union(const Bounds3<T> &b1, const Bounds3<T> &b2) {
        return Bounds3<T>(Point3<T>(std::min(b1.pMin[0], b2.pMin[0]),
            std::min(b1.pMin[1], b2.pMin[1]),
            std::min(b1.pMin[2], b2.pMin[2])),
            Point3<T>(std::max(b1.pMax[0], b2.pMax[0]),
                std::max(b1.pMax[1], b2.pMax[1]),
                std::max(b1.pMax[2], b2.pMax[2])));
    }

    template <typename T>
    Bounds3<T> Intersect(const Bounds3<T> &b1, const Bounds3<T> &b2) {
        return Bounds3<T>(Point3<T>(std::max(b1.pMin[0], b2.pMin[0]),
            std::max(b1.pMin[1], b2.pMin[1]),
            std::max(b1.pMin[2], b2.pMin[2])),
            Point3<T>(std::min(b1.pMax[0], b2.pMax[0]),
                std::min(b1.pMax[1], b2.pMax[1]),
                std::min(b1.pMax[2], b2.pMax[2])));
    }

#endif


template <typename T>
bool Overlaps(const Bounds3<T> &b1, const Bounds3<T> &b2) {
    bool x = (b1.pMax[0] >= b2.pMin[0]) && (b1.pMin[0] <= b2.pMax[0]);
    bool y = (b1.pMax[1] >= b2.pMin[1]) && (b1.pMin[1] <= b2.pMax[1]);
    bool z = (b1.pMax[2] >= b2.pMin[2]) && (b1.pMin[2] <= b2.pMax[2]);
    return (x && y && z);
}

template <typename T, typename U>
inline Bounds3<T> Expand(const Bounds3<T> &b, U delta) {
    return Bounds3<T>(b.pMin - Vector3T(delta, delta, delta),
        b.pMax + Vector3T(delta, delta, delta));
}

template <typename T>
inline bool Bounds3<T>::intersect(const Ray &ray, float *hitt0,
    float *hitt1) const {
    float t0 = 0, t1 = ray.getMaxLength();
    for (int i = 0; i < 3; ++i) {
        // Update interval for _i_th bounding box slab
        float invRayDir = 1 / ray.getDir()[i];
        float tNear = (pMin[i] - ray.getOrigin()[i]) * invRayDir;
        float tFar = (pMax[i] - ray.getOrigin()[i]) * invRayDir;

        // Update parametric interval from slab intersection $t$ values
        if (tNear > tFar) std::swap(tNear, tFar);

        // Update _tFar_ to ensure robust ray--bounds intersection
        tFar *= 1 + 2 * gamma(3);
        t0 = tNear > t0 ? tNear : t0;
        t1 = tFar < t1 ? tFar : t1;
        if (t0 > t1) return false;
    }
    if (hitt0) *hitt0 = t0;
    if (hitt1) *hitt1 = t1;
    return true;
}

template <typename T>
inline bool Bounds3<T>::intersect(const Ray &ray, const Vector3f &invDir,
    const int dirIsNeg[3]) const {
    const Bounds3f &bounds = *this;
    // Check for ray intersection against $x$ and $y$ slabs
    float tMin = (bounds[dirIsNeg[0]][0] - ray.o[0]) * invDir[0];
    float tMax = (bounds[1 - dirIsNeg[0]][0] - ray.o[0]) * invDir[0];
    float tyMin = (bounds[dirIsNeg[1]][1] - ray.o[1]) * invDir[1];
    float tyMax = (bounds[1 - dirIsNeg[1]][1] - ray.o[1]) * invDir[1];

    // Update _tMax_ and _tyMax_ to ensure robust bounds intersection
    tMax *= 1 + 2 * gamma(3);
    tyMax *= 1 + 2 * gamma(3);
    if (tMin > tyMax || tyMin > tMax) return false;
    if (tyMin > tMin) tMin = tyMin;
    if (tyMax < tMax) tMax = tyMax;

    // Check for ray intersection against $z$ slab
    float tzMin = (bounds[dirIsNeg[2]][2] - ray.o[2]) * invDir[2];
    float tzMax = (bounds[1 - dirIsNeg[2]][2] - ray.o[2]) * invDir[2];

    // Update _tzMax_ to ensure robust bounds intersection
    tzMax *= 1 + 2 * gamma(3);
    if (tMin > tzMax || tzMin > tMax) return false;
    if (tzMin > tMin) tMin = tzMin;
    if (tzMax < tMax) tMax = tzMax;
    return (tMin < ray.tMax) && (tMax > 0);
}

#endif // #define __BOUNDS_H__