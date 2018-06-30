#include <lights/arealight.h>

Spectrum AreaLight::Sample(const gmtl::Point3f& position, gmtl::Vec3f& wi, float& pdf, gmtl::Rayf& visibilityRay) const
{
    return Spectrum(0.0f);
}