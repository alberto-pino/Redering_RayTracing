#ifndef __MATERIAL_H__
#define __MATERIAL_H__

#include <gmtl/gmtl.h>
#include <color.h>

//class Material
//{
//public:
//    float Kd;
//    float Ks;
//    float Kt;
//};

class Material
{
public:
    Material(const char* name): mName(name), Ka_color(0.0f), Ka(1.0f) {}
	Material(const char* name, float ka, const Color& ka_color) : mName(name), Ka_color(ka_color), Ka(ka) {}

	float Ka;
    Color Ka_color;

    const char* GetName() const { return mName.c_str();}

	virtual Spectrum BRDF(const Spectrum& Li, const gmtl::Vec3f& L, const gmtl::Vec3f& V, const IntersectInfo& info) const = 0;
	virtual bool Sample(gmtl::Vec3f& wi, float& pdf, const IntersectInfo& info) const = 0;
	virtual float pdf(const gmtl::Vec3f& wi, const gmtl::Vec3f& wo) const = 0;
private:
    std::string mName;
};

struct MaterialInfo
{
    Material* material;
    int objectID;
	gmtl::Vec3f position;
};

#endif
