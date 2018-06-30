#include <material.h>

class Standard : public Material
{
public:
	Standard(const char* name) : Material(name), Kd(0.0f), Ks(0.0f), Kt(0.0f), Kr(0.0f), Kshi(1.0f), refractionIndex(1.52f) {}

	Color Kd;
	Color Ks;
	Color Kr;
	Color Kt;

	float Kshi;

	float refractionIndex;

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