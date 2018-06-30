#ifndef __LIGHTS_H__
#define __LIGHTS_H__

#include <gmtl/gmtl.h>
#include <spectrum.h>

class Light
{
public:
    Light(const gmtl::Vec3f& c, float i):mColor(c), mIntensity(i) {}

    virtual Spectrum Sample(const gmtl::Point3f& position, gmtl::Vec3f& wi, float& pdf, gmtl::Rayf& visibilityRay) const { return InvalidSpectrum;}

    Spectrum mColor;
    float mIntensity;

#ifdef NODE_NAME_SUPPORT
	void SetName(const std::string& name) { mName = name;}
	const char* GetName() const { return mName.c_str();}
	std::string mName;
#endif
};

#endif