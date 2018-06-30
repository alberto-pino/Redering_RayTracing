#ifndef __REPORTER_H__
#define __REPORTER_H__

#include <vector>
#include <gmtl/gmtl.h>

#define REPORT_ENABLED

#ifdef REPORT_ENABLED


class Primitive;
class World;

bool InitReporter(const char* filename, World* world);

//void ReportImageSample(const Point& imagePlanePoint);

void ReportImageRay(const gmtl::Rayf& imageRay);

void ReportBsdfDirection(const gmtl::Point3f& o, const gmtl::Vec3f& d, const Primitive* primitive);
void ReportPathRay(const gmtl::Rayf& pathRay, int length);
void ReportPathRays(const std::vector<gmtl::Rayf>& path_rays);

void PrintStats();

#define INITREPORTER(x, y) InitReporter(x, y)

#define REPORTIMAGESAMPLE(x) ReportImageSample(x)
#define REPORTIMAGERAY(x) ReportImageRay(x)

#define REPORTBSDFDIRECTION(x, y, z) ReportBsdfDirection(x, y, z)
#define REPORTPATHRAY(x, l) ReportPathRay(x, l)
#define REPORTPATHRAYS(x) ReportPathRays(x)


#define PRINTSTATS PrintStats()

#else
#define INITREPORTER(x, y) true
#define REPORTIMAGESAMPLE(x)
#define REPORTIMAGERAY(x)

#define REPORTBSDFDIRECTION(x, y, z)
#define REPORTPATHRAY(x)
#define REPORTPATHRAYS(x)
#define PRINTSTATS
#endif

#endif