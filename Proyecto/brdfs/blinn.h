#ifndef __BLINN_H__
#define __BLINN_H__

#include <material.h>
#include <lambert.h>

class Blinn : public Material
{
public:
	Blinn(const char* name) : Material(name), lambert((std::string(name) + "_diffuse").c_str()), Ks(0.0f) {}
	Blinn(const char* name, float kd, const Color& kd_color, float ks, const Color& ks_color) : Material(name), lambert( (std::string(name) + "_diffuse").c_str(), kd, kd_color), Ks(ks), Ks_color(ks_color) {}

	//Color Kd;
	Lambert lambert;
	float Ks;
	Color Ks_color;

    virtual Spectrum BRDF(const Spectrum& Li, const gmtl::Vec3f& L, const gmtl::Vec3f& V, const IntersectInfo& info) const
    {
        return Spectrum(0.0f);
    }
    virtual bool Sample(gmtl::Vec3f& wi, float& pdf, const IntersectInfo& info) const
    {
        return false;
    }

    virtual float pdf(const gmtl::Vec3f& wi, const gmtl::Vec3f& wo) const
    {
        return 0.0f;
    }
};

#endif