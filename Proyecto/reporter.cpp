#include "reporter.h"

#include <maya/ma_writer.h>
#include <core.h>
#include <primitives.h>
#include <world.h>

#include <set>

static const int SAMPLE_MAX_DEPTH = 12;

static MAFile reportFile;
static int numImageSamples = 0;
static int numImageRays = 0;
static int numBsdfDirs = 0;
static int numPathRays = 0;
extern int g_RenderMaxDepth;
static int numPathRaysLength[SAMPLE_MAX_DEPTH+1];

std::string GetNameForPrimitive(const Primitive* the_primitive)
{
	std::string primitive_name;
#ifdef NODE_NAME_SUPPORT
	const char* local_pri = the_primitive->GetName();
	if (local_pri)
		primitive_name = local_pri;
#endif
	if (primitive_name.empty())
	{
		char local_name[1024];
		sprintf(local_name, "primitive%d", the_primitive->mObjectID);
		primitive_name = local_name;
	}
	return primitive_name;
}

bool InitReporter(const char* filename, World* world)
{
	if (!reportFile.Open(filename))
	{
		Error("Could not create report file %s", filename);
		return false;
	}

	reportFile.WriteGroup("imageSamples");
	reportFile.WriteGroup("imageRays");
	reportFile.WriteGroup("bsdfSamples");
	vector<Primitive*> scene_primitives = world->mPrimitives;
	int num_primitives = scene_primitives.size();
	std::set<std::string> primitiveNames;
	for (int iPrimitive = 0; iPrimitive < num_primitives; ++iPrimitive)
	{
		const Primitive* the_primitive = scene_primitives[iPrimitive];
		const char* primitive_name = NULL;
#ifdef NODE_NAME_SUPPORT
		primitive_name = the_primitive->GetName();
		primitiveNames.insert(primitive_name);
#endif
		if (!primitive_name)
		{
			char local_name[1024];
			sprintf(local_name, "primitive%d", the_primitive->mObjectID);
			primitiveNames.insert(local_name);
		}
		else
		{
			primitiveNames.insert(primitive_name);
		}
	}

	num_primitives = primitiveNames.size();
	for (std::set<std::string>::iterator it = primitiveNames.begin(); it != primitiveNames.end(); ++it)
	{
		const string& name = (*it);
		reportFile.WriteGroup(name.c_str(), "bsdfSamples", true, true);
	}
	reportFile.WriteGroup("pathRays");
	// More groups....

	numImageSamples = 0;
	numImageRays = 0;
	numBsdfDirs = 0;
	numPathRays = 0;
	for (int iLength = 0; iLength < g_RenderMaxDepth; ++iLength)
		numPathRaysLength[iLength] = 0;
	numPathRaysLength[g_RenderMaxDepth] = 0;
	return true;
}

void ReportImageSample(const gmtl::Point3f& imagePlanePoint)
{
	++numImageSamples;
	return;
	char t_name[1024];
	sprintf(t_name, "%s%d", "point", numImageSamples);
	reportFile.WritePoint(imagePlanePoint, t_name, "imageSamples");
}

void ReportImageRay(const Ray& imageRay)
{
	++numImageRays;
	//reportFile.WriteRay(imageRay, "imageRay", "imageRays");
}

void ReportBsdfDirection(const gmtl::Point3f& o, const gmtl::Vec3f& d, const Primitive* primitive)
{
	++numBsdfDirs;
	std::string primi_name = GetNameForPrimitive(primitive);
	reportFile.WriteDirection(o, d, NULL, primi_name.c_str());
}

void ReportPathRay(const gmtl::Rayf& pathRay, int length)
{
	++numPathRays;
	if (length < g_RenderMaxDepth)
		++numPathRaysLength[length];
	else
		++numPathRaysLength[g_RenderMaxDepth];
	reportFile.WriteRay(pathRay, NULL, "pathRays");
}

void ReportPathRays(const vector<gmtl::Rayf>& path_rays)
{
	static int localNumPathRays = 0;
	char path_ray_name[1024];
	sprintf(path_ray_name, "pathRay%d", localNumPathRays);
	++localNumPathRays;
	reportFile.WriteGroup(path_ray_name, "pathRays", true, true);
	int numRays = path_rays.size();
	for (int iRay = 0; iRay < numRays; ++iRay)
	{
		const gmtl::Rayf& the_ray = path_rays[iRay];
		reportFile.WriteRay(the_ray, NULL, path_ray_name);
		++numPathRays;
	}
    reportFile.Flush();
}

void PrintStats()
{
	printf("Reports:\n");
	printf("\tNum image samples: %d\n", numImageSamples);
	printf("\tNum image rays: %d\n", numImageRays);
	printf("\tNum bsdf directions: %d\n", numBsdfDirs);
	printf("\tNum path rays: %d\n", numPathRays);
	for (int iLength = 0; iLength < g_RenderMaxDepth; ++iLength)
	{
		printf("\t\tbounces %d: %d\n", iLength, numPathRaysLength[iLength]);
	}
	printf("\t\tbounces %d or more: %d\n", g_RenderMaxDepth, numPathRaysLength[g_RenderMaxDepth]);
}