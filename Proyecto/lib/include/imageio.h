#ifndef __IMAGEIO_H__
#define __IMAGEIO_H__

#include <string>
#include <core.h>
#include <spectrum.h>

// ImageIO Declarations
void WriteImage(const std::string& name, float *pixels, float *alpha,
	int XRes, int YRes, int totalXRes, int totalYRes, int xOffset,
	int yOffset);

Spectrum *ReadImageTGA(const std::string &name, int *w, int *h);


#endif // __IMAGEIO_H__
