#ifndef __BLINNPHONG_H__
#define __BLINNPHONG_H__

#include <material.h>
#include <lambert.h>

class BlinnPhong : public Material
{
public:
	BlinnPhong(const char* name) : Material(name), lambert((std::string(name) + "_diffuse").c_str()), Ks_color(0.0f), Ks(1.0f), SpecularPower(16.0f) {}
	BlinnPhong(const char* name, float kd, const Color& kd_color, float ks, const Color& ks_color, float power) :
		Material(name), lambert((std::string(name) + "_diffuse").c_str(), kd, kd_color), Ks_color(ks), Ks(ks),
		SpecularPower(power)
	{
	}

	//Color Kd;
	Lambert lambert;
	float SpecularPower;
	Color Ks_color;
	float Ks;

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