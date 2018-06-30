#ifndef __MA_WRITER_H__
#define __MA_WRITER_H__

#include <string>
#include <core.h>
#include <gmtl/gmtl.h>

class MAFile
{
public:
    MAFile();
    ~MAFile();

    bool Open(const char* filename);

    bool WritePoint(const gmtl::Point3f& point, const char* name, const char* parent);
    bool WriteRay(const gmtl::Rayf& ray, const char* name, const char* parent = NULL);
    bool WriteDirection(const gmtl::Point3f& origin, const gmtl::Vec3f& dir, const char* name, const char* parent = NULL);
    bool WriteGroup(const char* groupName, const char* parent = NULL, bool hidden = false, bool justCurves = false);

    bool WriteLine(const gmtl::Point3f& origin, const gmtl::Point3f& end, const char* name, const char* parent);

    void Flush() { fflush(m_file);}

    bool Close();
private:
    std::string m_filename;
    FILE* m_file;

    bool m_bCreatedSphere;
};

#endif