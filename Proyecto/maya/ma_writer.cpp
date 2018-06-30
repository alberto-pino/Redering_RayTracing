#include "ma_writer.h"

static const float direction_length = 5.0f;

MAFile::MAFile() : m_file(NULL), m_bCreatedSphere(false)
{

}

MAFile::~MAFile()
{
    Close();
}

bool MAFile::Open(const char* filename)
{
    m_filename = filename;

    m_file = fopen(m_filename.c_str(), "wb");
    if (m_file)
    {
        FILE* empy_file = fopen("./resources/empty.ma", "rb");
        if (empy_file)
        {
            fseek(empy_file, 0, SEEK_END);
            long file_size = ftell(empy_file);
            fseek(empy_file, 0, SEEK_SET);

            char* empty_file_contents = new char[file_size];
            fread(empty_file_contents, 1, file_size, empy_file);

            fwrite(empty_file_contents, 1, file_size, m_file);

			fclose(empy_file);
			fflush(m_file);

            delete[] empty_file_contents;
            return true;
        }
        else
        {
            Error("Could not find empty.ma in resources folder");
            Close();
            return false;
        }
    }
    return false;
}

bool MAFile::WritePoint(const gmtl::Point3f& point, const char* name, const char* parent)
{
    if (!m_file)
        return false;

    fprintf(m_file, "createNode transform -n \"%s\"", name);
    if (parent)
    {
        fprintf(m_file, "-p %s", parent);
    }
    fprintf(m_file, ";\n");
    fprintf(m_file, "\tsetAttr \".t\" -type \"double3\" %f %f %f;\n", point[0], point[1], point[2]);
    if (!m_bCreatedSphere)
    {
        std::string shape_name("refPoint");
        shape_name += "Shape";
        fprintf(m_file, "createNode mesh -n \"%s\" -p \"%s\";\n", shape_name.c_str(), name);
        std::string sphere_shape(name);
        sphere_shape += "Sphere";
        fprintf(m_file, "createNode polySphere -n \"%s\";\n", sphere_shape.c_str());
        fprintf(m_file, "\tconnectAttr \"%s.out\" \"%s.i\";\n", sphere_shape.c_str(), shape_name.c_str());
        fprintf(m_file, "\tsetAttr \"%s.radius\" %f;\n", sphere_shape.c_str(), 0.01f);
        fprintf(m_file, "\tsetAttr \"%s.sa\" %d;\n", sphere_shape.c_str(), 3);
        fprintf(m_file, "\tsetAttr \"%s.sh\" %d;\n", sphere_shape.c_str(), 3);
        m_bCreatedSphere = true;
    }
    else
    {
        fprintf(m_file, "parent -s -nc -r -add \"%s\" \"%s\";\n", "refPointShape", name);
    }

    return true;
}

bool MAFile::WriteRay(const gmtl::Rayf& ray, const char* name, const char* parent)
{
    if (!m_file)
        return false;

    if (!name && !parent)
        return false;

    gmtl::Point3f origin = ray.getOrigin();
    gmtl::Point3f end = ray.getOrigin() + ray.getDir() * (ray.getMaxLength() > 100000.0f ? 30.0f : ray.getMaxLength());

    return WriteLine(origin, end, name, parent);
}

bool MAFile::WriteDirection(const gmtl::Point3f& origin, const gmtl::Vec3f& dir, const char* name, const char* parent)
{
    if (!m_file)
        return false;

    gmtl::Vec3f norm_dir = dir;
	normalize(norm_dir);
    gmtl::Point3f end = origin + norm_dir * direction_length;

    return WriteLine(origin, end, name, parent);
}

bool MAFile::WriteLine(const gmtl::Point3f& origin, const gmtl::Point3f& end, const char* name, const char* parent)
  {
    if (!m_file)
        return false;

    static unsigned int numLines = 0;
    ++numLines;

    char ray_shape[1024];
    const char* real_parent = parent;

    if (name)
    {
        if (!parent)
        {
            fprintf(m_file, "createNode transform -n \"%s\"", name);
            if (parent)
            {
                fprintf(m_file, "-p %s", parent);
            }
            fprintf(m_file, ";\n");
            sprintf(ray_shape, "%sShape", name);
            real_parent = name;
        }
        else
        {
            sprintf(ray_shape, "%s", name);
        }
    }
    else
    {
        sprintf(ray_shape, "%s%d", "ray", numLines);
    }

    gmtl::Vec3f direction = end - origin;
	normalize(direction);
    gmtl::Point3f mid1 = origin + 1.0f/3.0f * direction;
    gmtl::Point3f mid2 = origin + 1.0f/3.0f * direction;

    fprintf(m_file, "createNode nurbsCurve -n \"%s\" -p \"%s\";\n", ray_shape, real_parent);
    fprintf(m_file, "\tsetAttr -k off \".v\";\n");
    fprintf(m_file, "\tsetAttr \".cc\" -type \"nurbsCurve\"\n");
    fprintf(m_file, "\t\t3 1 0 no 3\n\t\t6 0 0 0 1 1 1\n\t\t4\n");
    fprintf(m_file, "\t\t%.2f %.2f %.2f\n", origin[0], origin[1], origin[2]);
    fprintf(m_file, "\t\t%.2f %.2f %.2f\n", mid1[0], mid1[1], mid1[2]);
    fprintf(m_file, "\t\t%.2f %.2f %.2f\n", mid2[0], mid2[1], mid2[2]);
    fprintf(m_file, "\t\t%.2f %.2f %.2f\n\t\t;\n", end[0], end[1], end[2]);

    return true;
}

bool MAFile::WriteGroup(const char* groupName, const char* parent, bool hidden, bool justCurves)
{
    if (!m_file)
        return false;

    fprintf(m_file, "createNode transform -n \"%s\"", groupName);
    if (parent)
    {
        fprintf(m_file, "-p \"%s\"", parent);
    }
    fprintf(m_file, ";\n");

    if (hidden)
    {
        fprintf(m_file, "\tsetAttr \".v\" no;\n");
    }

    if (justCurves)
    {
        fprintf(m_file, "\taddAttr -ci true -sn \"riCurves\" -ln \"riCurves\" -min 0 -max 1 -at \"bool\";\n\tsetAttr \".riCurves\" yes;\n");
    }

	fflush(m_file);

    return true;
}

bool MAFile::Close()
{
    if (m_file)
    {
        fclose(m_file);
        m_file = NULL;
    }
    return true;
}