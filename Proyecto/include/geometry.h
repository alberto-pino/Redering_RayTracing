//#if defined(_MSC_VER)
//#define NOMINMAX
//#pragma once
//#endif

#ifndef __GEOMETRY_H__
#define __GEOMETRY_H__

#include <gmtl/gmtl.h>

typedef gmtl::Point<float, 2> Point2f;
typedef gmtl::Point<float, 3> Point3f;
typedef gmtl::Point<int, 2> Point2i;
typedef gmtl::Point<int, 3> Point3i;

#if _MSC_VER >= 1900
    template <typename DATA_TYPE>
    using Point2 = gmtl::Point<DATA_TYPE, 2>;
    template <typename DATA_TYPE>
    using Point3 = gmtl::Point<DATA_TYPE, 3>;
#else

#endif

typedef gmtl::Vec<float, 2> Vector2f;
typedef gmtl::Vec<float, 3> Vector3f;

#if _MSC_VER >= 1900
    template <typename DATA_TYPE>
    using Vector2 = gmtl::Vec<DATA_TYPE, 2>;
    template <typename DATA_TYPE>
    using Vector3 = gmtl::Vec<DATA_TYPE, 3>;
#else

#endif

typedef gmtl::Vec<float, 2> Normal2f;
typedef gmtl::Vec<float, 3> Normal3f;

#if _MSC_VER >= 1900
    template <typename DATA_TYPE>
    using Normal2 = gmtl::Vec<DATA_TYPE, 2>;
    template <typename DATA_TYPE>
    using Normal3 = gmtl::Vec<DATA_TYPE, 3>;
#else

#endif

typedef gmtl::Rayf Ray;

typedef gmtl::Matrix43f Matrix43f;

#if _MSC_VER >= 1900
    template<typename T>
    class Triangle
    {
    public:
        Point3<T> v[3];
        inline Point3<T> operator[](int index) const { return v[index]; }
        inline Point3<T>& operator[](int index) { return v[index]; }
    };
    
#else
    template<typename T>
    class Triangle
    {
    public:
        gmtl::Point<T, 3> v[3];
        inline gmtl::Point<T, 3> operator[](int index) const { return v[index];}
        inline gmtl::Point<T, 3>& operator[](int index) { return v[index];}
    };
#endif
typedef Triangle<float> Trianglef;

#endif