#ifndef __POINTLIGHT_H__
#define __POINTLIGHT_H__

#include <lights.h>
#include <transform.h>

class PointLight: public Light
{
public:
    PointLight(const Transform& worldTransform, const gmtl::Vec3f& c, float i, gmtl::Point3f& p) : Light(c, i), mPosition(p), mWorldTransform(worldTransform)
    {
        mTransformedPosition = mWorldTransform(mPosition);
    }

    const gmtl::Point3f getWorldPosition() const { return mTransformedPosition;}

    virtual Spectrum Sample(const gmtl::Point3f& position, gmtl::Vec3f& wi, float& pdf, gmtl::Rayf& visibilityRay) const;

protected:
    gmtl::Point3f mPosition;
    Transform mWorldTransform;

private:
    gmtl::Point3f mTransformedPosition;
};

#endif