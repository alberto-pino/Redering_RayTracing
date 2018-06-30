#ifndef __TRANSFORM_H__
#define __TRANSFORM_H__

#include <gmtl/gmtl.h>
#include <core.h>


struct Matrix4x4
{
    // Matrix4x4 Public Methods
    Matrix4x4()
    {
        m[0][0] = m[1][1] = m[2][2] = m[3][3] = 1.f;
        m[0][1] = m[0][2] = m[0][3] = m[1][0] =
            m[1][2] = m[1][3] = m[2][0] = m[2][1] = m[2][3] =
            m[3][0] = m[3][1] = m[3][2] = 0.f;
    }
    Matrix4x4(float mat[4][4])
    {
        memcpy(m, mat, 16*sizeof(float));
    }
    Matrix4x4(float t00, float t01, float t02, float t03,
        float t10, float t11, float t12, float t13,
        float t20, float t21, float t22, float t23,
        float t30, float t31, float t32, float t33)
    {
        m[0][0] = t00; m[0][1] = t01; m[0][2] = t02; m[0][3] = t03;
        m[1][0] = t10; m[1][1] = t11; m[1][2] = t12; m[1][3] = t13;
        m[2][0] = t20; m[2][1] = t21; m[2][2] = t22; m[2][3] = t23;
        m[3][0] = t30; m[3][1] = t31; m[3][2] = t32; m[3][3] = t33;
    }

    bool operator==(const Matrix4x4 &m2) const {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                if (m[i][j] != m2.m[i][j]) return false;
        return true;
    }
    bool operator!=(const Matrix4x4 &m2) const {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                if (m[i][j] != m2.m[i][j]) return true;
        return false;
    }
    friend Matrix4x4 Transpose(const Matrix4x4 &);
    void Print(FILE *f) const {
        fprintf(f, "[ ");
        for (int i = 0; i < 4; ++i) {
            fprintf(f, "  [ ");
            for (int j = 0; j < 4; ++j)  {
                fprintf(f, "%f", m[i][j]);
                if (j != 3) fprintf(f, ", ");
            }
            fprintf(f, " ]\n");
        }
        fprintf(f, " ] ");
    }
    static Matrix4x4 Mul(const Matrix4x4 &m1, const Matrix4x4 &m2) {
        Matrix4x4 r;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                r.m[i][j] = m1.m[i][0] * m2.m[0][j] +
                m1.m[i][1] * m2.m[1][j] +
                m1.m[i][2] * m2.m[2][j] +
                m1.m[i][3] * m2.m[3][j];
        return r;
    }
    friend Matrix4x4 Inverse(const Matrix4x4 &);
    float m[4][4];
};

class Transform
{
public:
    Matrix4x4 mTransform;
    Matrix4x4 mTransformInv;

    Transform() {}

    Transform(const float mat[4][4]) {
        mTransform = Matrix4x4(mat[0][0], mat[0][1], mat[0][2], mat[0][3],
            mat[1][0], mat[1][1], mat[1][2], mat[1][3],
            mat[2][0], mat[2][1], mat[2][2], mat[2][3],
            mat[3][0], mat[3][1], mat[3][2], mat[3][3]);
        mTransformInv = Inverse(mTransform);
    }

    Transform(const Matrix4x4& m) : mTransform(m)
    {
        mTransformInv = Inverse(mTransform);
    }
    Transform(const Matrix4x4& m, const Matrix4x4& minv) : mTransform(m), mTransformInv(minv) {}

    inline gmtl::Point3f operator()(const gmtl::Point3f &pt) const
    {
        float x = pt[0], y = pt[1], z = pt[2];
        float xp = mTransform.m[0][0]*x + mTransform.m[0][1]*y + mTransform.m[0][2]*z + mTransform.m[0][3];
        float yp = mTransform.m[1][0]*x + mTransform.m[1][1]*y + mTransform.m[1][2]*z + mTransform.m[1][3];
        float zp = mTransform.m[2][0]*x + mTransform.m[2][1]*y + mTransform.m[2][2]*z + mTransform.m[2][3];
        float wp = mTransform.m[3][0]*x + mTransform.m[3][1]*y + mTransform.m[3][2]*z + mTransform.m[3][3];
        if (wp == 1.0f) 
            return gmtl::Point3f(xp, yp, zp);
        else
            return gmtl::Point3f(xp, yp, zp)/wp;
    }

    inline gmtl::Vec3f operator()(const gmtl::Vec3f &v) const
    {
        float x = v[0], y = v[1], z = v[2];
        return gmtl::Vec3f(mTransform.m[0][0]*x + mTransform.m[0][1]*y + mTransform.m[0][2]*z,
            mTransform.m[1][0]*x + mTransform.m[1][1]*y + mTransform.m[1][2]*z,
            mTransform.m[2][0]*x + mTransform.m[2][1]*y + mTransform.m[2][2]*z);
    }

    Transform operator*(const Transform &t2) const
    {
        Matrix4x4 m1 = Matrix4x4::Mul(mTransform, t2.mTransform);
        Matrix4x4 m2 = Matrix4x4::Mul(t2.mTransformInv, mTransformInv);
        return Transform(m1, m2);
    }

    friend Transform Inverse(const Transform &t)
    {
        return Transform(t.mTransformInv, t.mTransform);
    }
};

inline Transform Translate(const gmtl::Vec3f &delta)
{
    Matrix4x4 m(1, 0, 0, delta[0],
        0, 1, 0, delta[1],
        0, 0, 1, delta[2],
        0, 0, 0,       1);
    Matrix4x4 minv(1, 0, 0, -delta[0],
        0, 1, 0, -delta[1],
        0, 0, 1, -delta[2],
        0, 0, 0,        1);
    return Transform(m, minv);
}

inline Transform Scale(float x, float y, float z)
{
    Matrix4x4 m(x, 0, 0, 0,
        0, y, 0, 0,
        0, 0, z, 0,
        0, 0, 0, 1);
    Matrix4x4 minv(1.f/x,     0,     0,     0,
        0,     1.f/y,     0,     0,
        0,         0,     1.f/z, 0,
        0,         0,     0,     1);
    return Transform(m, minv);
}

inline Transform RotateX(float angle) {
    float sin_t = sinf(Radians(angle));
    float cos_t = cosf(Radians(angle));
    Matrix4x4 m(1,     0,      0, 0,
        0, cos_t, -sin_t, 0,
        0, sin_t,  cos_t, 0,
        0,     0,      0, 1);
    return Transform(m, Transpose(m));
}


inline Transform RotateY(float angle) {
    float sin_t = sinf(Radians(angle));
    float cos_t = cosf(Radians(angle));
    Matrix4x4 m( cos_t,   0,  sin_t, 0,
        0,   1,      0, 0,
        -sin_t,   0,  cos_t, 0,
        0,   0,   0,    1);
    return Transform(m, Transpose(m));
}



inline Transform RotateZ(float angle) {
    float sin_t = sinf(Radians(angle));
    float cos_t = cosf(Radians(angle));
    Matrix4x4 m(cos_t, -sin_t, 0, 0,
        sin_t,  cos_t, 0, 0,
        0,      0, 1, 0,
        0,      0, 0, 1);
    return Transform(m, Transpose(m));
}


inline Transform Rotate(float angle, const gmtl::Vec3f &axis) {
    gmtl::Vec3f a = axis;
    normalize(a);
    float s = sinf(Radians(angle));
    float c = cosf(Radians(angle));
    float m[4][4];

    m[0][0] = a[0] * a[0] + (1.f - a[0] * a[0]) * c;
    m[0][1] = a[0] * a[1] * (1.f - c) - a[2] * s;
    m[0][2] = a[0] * a[2] * (1.f - c) + a[1] * s;
    m[0][3] = 0;

    m[1][0] = a[0] * a[1] * (1.f - c) + a[2] * s;
    m[1][1] = a[1] * a[1] + (1.f - a[1] * a[1]) * c;
    m[1][2] = a[1] * a[2] * (1.f - c) - a[0] * s;
    m[1][3] = 0;

    m[2][0] = a[0] * a[2] * (1.f - c) - a[1] * s;
    m[2][1] = a[1] * a[2] * (1.f - c) + a[0] * s;
    m[2][2] = a[2] * a[2] + (1.f - a[2] * a[2]) * c;
    m[2][3] = 0;

    m[3][0] = 0;
    m[3][1] = 0;
    m[3][2] = 0;
    m[3][3] = 1;

    Matrix4x4 mat(m);
    return Transform(mat, Transpose(mat));
}

inline Transform Perspective(float fov, float znear, float zfar)
{
    // Perform projective divide
    Matrix4x4 persp = Matrix4x4(1, 0, 0, 0,
        0, 1,           0,              0,
        0, 0, zfar / (zfar - znear), -zfar * znear / (zfar - znear),
        0, 0,       1,              0);

    // Scale to canonical viewing volume
    float invTanAng = 1.f / tanf(Radians(fov) / 2.f);
    return Scale(invTanAng, invTanAng, 1) * Transform(persp);
}

#endif