#ifndef __PRIMITIVES_H__
#define __PRIMITIVES_H__

#include <geometry.h>
#include <bvh/bounds.h>
#include <material.h>
#include <transform.h>


typedef int ObjectID;
const int InvalidObjectID = -1;
struct IntersectInfo;


class Primitive
{
public:
    Primitive()
    {
        mObjectID = -1;
    }

    virtual bool intersect(const Ray& ray, float& t, IntersectInfo& info) const = 0;
    inline bool intersect(const Ray& ray, IntersectInfo* info) const
    {
        float t;
        return intersect(ray, t, *info);
    }
    inline bool intersect(const Ray& ray, IntersectInfo& info) const
    {
        float t;
        return intersect(ray, t, info);
    }
    virtual bool intersect(const Ray& ray, float& t) const = 0;
    virtual bool intersect(const Ray& ray) const = 0;

    virtual Bounds3f WorldBounds() const { return Bounds3f(); }
    virtual const Material *GetMaterial() const = 0;
    virtual void setMaterial(Material* m) { }

    virtual void CalculateRange(float& minPos, float& maxPos, int a_Axis) {}

    ObjectID mObjectID;
    
    #ifdef NODE_NAME_SUPPORT
    void SetName(const std::string& name) { mName = name;}
    const char* GetName() const { return mName.c_str();}
    std::string mName;
    #endif
};

class GeoPrimitive : public Primitive
{
public:
    GeoPrimitive(const Transform& worldTransform, Material* m) : mWorldTransform(worldTransform)
    {
        mObjectID = -1;
        mMaterial = m;
    }

    virtual void setMaterial(Material* m) { mMaterial = m; }
    virtual const Material *GetMaterial() const { return mMaterial; }

    Material* mMaterial;
    Transform mWorldTransform;
};

class SpherePrimitive: public GeoPrimitive
{
public:
    gmtl::Spheref mGeoSphere;

    SpherePrimitive(const Transform& worldTransform, Material* m, const Point3f& o, float r);
    virtual bool intersect(const Ray& ray, float& t, IntersectInfo& info) const;
    virtual bool intersect(const Ray& ray, float& t) const;
    virtual bool intersect(const Ray& ray) const;
private:
    void getUV(const Vector3f& n, float& u, float& v) const;
};

class MeshShape : public GeoPrimitive
{
    friend class TrianglePrimitive;
public:
    std::vector<Point3f> mVertices;
    std::vector<Vector3f> mNormals;
    std::vector<Point2f> mUVs;
    std::vector<Vector3f> mTangents;
    std::vector<int> mIndices;
    std::vector<int> mNormalIndices;

    MeshShape(const Transform& worldTransform, Material* m);
    virtual bool intersect(const Ray& ray, float& t, IntersectInfo& info) const;
    virtual bool intersect(const Ray& ray, float& t) const;
    virtual bool intersect(const Ray& ray) const;

    void setGeometry(int nt, int nv, const int *vi, const Point3f *P, int nn, const int* ni, const Vector3f *N, const Vector3f *T, int nuv, const Point2f* uv);
    int getNumTriangles() const { return mIndices.size() / 3; }
private:
    bool IntersectTriangle(const Ray &ray, int i, float& tHit, float& tu, float& tv, Vector3f& tn, Vector3f& tt) const;
    void ComputeTangent(int i);
    void GetUVs(int i, float uvs[3][2]) const;
};

class TrianglePrimitive : public Primitive
{
public:
    TrianglePrimitive(int index, MeshShape* mesh) : mIndex(index), mMesh(mesh) {}

    virtual bool intersect(const Ray& ray, float& t, IntersectInfo& info) const;
    virtual bool intersect(const Ray& ray, float& t) const;
    virtual bool intersect(const Ray& ray) const;

    virtual Bounds3f WorldBounds() const;

    virtual const Material *GetMaterial() const { return mMesh->GetMaterial(); }

    virtual void CalculateRange(float& minPos, float& maxPos, int a_Axis);

    Point3f GetVertex(int i) const
    {
        int vertex_index = mMesh->mIndices[mIndex*3 + i];
        return mMesh->mWorldTransform(mMesh->mVertices[vertex_index]);
    }

    void GetTriangle(Trianglef& t) const;

private:
    int mIndex;
    MeshShape* mMesh;
};


#endif
