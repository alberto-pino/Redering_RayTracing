#ifndef __COLOR_H__
#define __COLOR_H__

#include <string>
#include <spectrum.h>
class World;
struct IntersectInfo;

class Color
{
public:
	Color() : mColor(0.0f) {}
    Color(float v) : mColor(v) {}
    Color(const Spectrum& s);
    Color(const char* filename);
	Color(const char* filename, const Spectrum& s);

    Spectrum GetColor(float u, float v, const World* world) const;
	Spectrum GetColor(const IntersectInfo& info) const;
	
private:
    std::string mTextureName;
    Spectrum mColor;
};

#endif