#include <lights/pointlight.h>

Spectrum PointLight::Sample(const gmtl::Point3f& position, gmtl::Vec3f& wi, float& pdf, gmtl::Rayf& visibilityRay) const
{
    wi = mTransformedPosition - position;
    float len = normalize(wi);
    visibilityRay.setOrigin(position);
    visibilityRay.setDir(wi);
    visibilityRay.setMaxLength(len);
    pdf = 1.0f;
    float i = mIntensity;
    i /= (len * len);
    return mColor * i;
}