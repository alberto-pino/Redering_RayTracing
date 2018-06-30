#ifndef __AREALIGHT_H__
#define __AREALIGHT_H__

#include <lights.h>
#include <transform.h>

class AreaLight: public Light
{
public:
    AreaLight(const Transform& worldTransform, const gmtl::Vec3f& c, float i, gmtl::Point3f& p) : Light(c, i), mPosition(p), mWorldTransform(worldTransform)
    {
        mTransformedPosition = mWorldTransform(mPosition);
    }

    const gmtl::Point3f getWorldPosition() const { return mTransformedPosition;}

    virtual Spectrum Sample(const gmtl::Point3f& position, gmtl::Vec3f& wi, float& pdf, gmtl::Rayf& visibilityRay) const;

protected:
    // The area light is a plane with width = 2.0f and height = 2.0f
    // If it's bigger than that, it will be in the scale portion of the mWorldTransform
    gmtl::Point3f mPosition;
    Transform mWorldTransform;

private:
    gmtl::Point3f mTransformedPosition;
};


#endif