//#include <tbb/tbb.h>
#include <stdio.h>
#include <math.h>
#include <gmtl/gmtl.h>
#include <imageio.h>
#include <standard.h>

#include <world.h>

#include <parsers/ass_parser.h>

#include <reporter.h>


#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <algorithm>

#include <main.h>
#include "lights\pointlight.h"

#include <cmath>

#include "lambert.h"



int g_RenderMaxDepth = 12;

extern int g_pixel_samples;

World* ReadFromFile(const char* filename)
{
	World* world;
	if (!ReadAssFile(filename, world))
	{
		Error("Could not read file: %s", filename);
		return NULL;
	}
	return world;
}




Spectrum calculateDiffuse(World* world, PointLight* light, IntersectInfo& intersectInfo)
{
	Standard* material = (Standard *)intersectInfo.material;
	Spectrum difColor = material->Kd.GetColor(intersectInfo);

	Point3f lightPosition = light->getWorldPosition();

	Vector3f distanceVector = intersectInfo.position - lightPosition;
	float squareLightDistanceToCollision = distanceVector[0] * distanceVector[0] + distanceVector[1] * distanceVector[1] + distanceVector[2] * distanceVector[2];

	IntersectInfo infoShadow;

	//shadow ray generation
	gmtl::Rayf shadowRay;
	gmtl::Vec3f shadowDirection(lightPosition - intersectInfo.position);
	normalize(shadowDirection);
	shadowRay.setOrigin(intersectInfo.position + shadowDirection * 0.01f);
	shadowRay.setDir(shadowDirection);

	Vector3f direction = lightPosition - intersectInfo.position;
	world->intersect(infoShadow, shadowRay, gmtl::length(direction));

	if (infoShadow.objectID == InvalidObjectID)
	{
		Vector3f normalNormalized = intersectInfo.normal;
		normalize(normalNormalized);

		normalize(direction);

		float dot = gmtl::dot(normalNormalized, direction);

		float cosV = dot > 0.0f ? dot : 0.0f;

		return Spectrum(difColor[0] * (light->mIntensity / squareLightDistanceToCollision) * cosV,
			difColor[1] * (light->mIntensity / squareLightDistanceToCollision) * cosV,
			difColor[2] * (light->mIntensity / squareLightDistanceToCollision) * cosV);
	}

	return Spectrum(0.0f, 0.0f, 0.0f);

}


Spectrum calculateSpecular(World* world, PointLight* light, IntersectInfo& intersectInfo)
{
	Standard* material = (Standard *)intersectInfo.material;
	Spectrum espColor = material->Ks.GetColor(intersectInfo);

	Vector3f directionNormalized = -intersectInfo.ray.getDir();
	normalize(directionNormalized);

	Point3f lightPosition = light->getWorldPosition();

	Vector3f distanceVector = intersectInfo.position - lightPosition;
	float squareLightDistanceToCollision = distanceVector[0] * distanceVector[0] + distanceVector[1] * distanceVector[1] + distanceVector[2] * distanceVector[2];

	IntersectInfo infoShadow;

	//shadow ray generation
	gmtl::Rayf shadowRay;
	gmtl::Vec3f shadowDirection(lightPosition - intersectInfo.position);
	normalize(shadowDirection);
	shadowRay.setOrigin(intersectInfo.position + shadowDirection * 0.01f);
	shadowRay.setDir(shadowDirection);

	Vector3f direction = lightPosition - intersectInfo.position;
	world->intersect(infoShadow, shadowRay, gmtl::length(direction));

	if (infoShadow.objectID == InvalidObjectID)
	{

		Vector3f normalNormalized = intersectInfo.normal;
		normalize(normalNormalized);

		normalize(direction);

		float dotND = (gmtl::dot(normalNormalized, direction));
		Vector3f rVector = (2.0f * (dotND * normalNormalized)) - direction;
		normalize(rVector);

		float dot = gmtl::dot(rVector, directionNormalized);

		float cosV = dot > 0.0f ? pow(dot, material->Kshi) : 0.0f;


		return Spectrum(espColor[0] * (light->mIntensity / squareLightDistanceToCollision) * cosV,
			espColor[1] * (light->mIntensity / squareLightDistanceToCollision) * cosV,
			espColor[2] * (light->mIntensity / squareLightDistanceToCollision) * cosV);
	}

	return Spectrum(0.0f, 0.0f, 0.0f);

}


gmtl::Vec3f transmittedDirection(bool& entering, const gmtl::Vec3f& N, const gmtl::Vec3f& V, const Standard* mat)
{
	gmtl::Vec3f normal_refraction = N;
	bool exitingObject = dot(N, -V) < 0.0f;
	float ni = 0.0f;
	float nt = 0.0f;

	if (exitingObject)
	{
		ni = mat->refractionIndex;
		nt = 1.0003f; // air refraction index
		normal_refraction = -normal_refraction;
	}
	else
	{
		ni = 1.0003f; // air refraction index
		nt = mat->refractionIndex;
	}

	//refracted direction calculation
	gmtl::Vec3f T;
	float eta;

	eta = ni / nt;
	float c1 = -dot(V, normal_refraction);
	float c2_op = 1.0f - eta * eta*(1.0f - c1 * c1);
	if (c2_op < 0.0f)
		return gmtl::Vec3f(0.0f);

	float c2 = sqrt(c2_op);
	T = eta * V + (eta*c1 - c2)*normal_refraction;


	entering = !exitingObject;
	return T;
}


Spectrum rayTracing(World* world, Ray& ray, int recursivityDepth = 0)
{
	IntersectInfo info;

	world->intersect(info, ray);

	if (info.objectID != InvalidObjectID)
	{
		Standard* material = (Standard *)info.material;

		//Diffuse and especular

		std::vector<float> difColor = { 0.0f, 0.0f, 0.0f };
		std::vector<float> espColor = { 0.0f, 0.0f, 0.0f };

		for (int i = 0; i < world->mLights.size(); ++i)
		{
			PointLight* light = (PointLight*)world->mLights[i];

			Spectrum dC = calculateDiffuse(world, light, info);
			difColor[0] += dC[0];
			difColor[1] += dC[1];
			difColor[2] += dC[2];

			Spectrum eC = calculateSpecular(world, light, info);
			espColor[0] += eC[0];
			espColor[1] += eC[1];
			espColor[2] += eC[2];
		}

		// Ambient light
		std::vector<float> ambientLight = { material->Ka_color.GetColor(info)[0] * 0.005f,
			material->Ka_color.GetColor(info)[1] * 0.005f,
			material->Ka_color.GetColor(info)[2] * 0.005f };

		Spectrum  reflexionLight = Spectrum();
		Spectrum  refractionLight = Spectrum();
		if (recursivityDepth > 0) {

			// Reflexion and refraction
			gmtl::Vec3f vVector = ray.getDir();
			gmtl::Vec3f rVector;
			gmtl::reflect(rVector, vVector, info.normal);
			gmtl::Rayf reflexionRay = gmtl::Rayf(info.position + rVector * 0.01f, rVector);

			reflexionLight = rayTracing(world, reflexionRay, recursivityDepth - 1) * material->Kr.GetColor(info);

			gmtl::Vec3f normalVector = info.normal;

			if (material->refractionIndex != 0)
			{
				bool entering;
				gmtl::Vec3f t = transmittedDirection(entering, info.normal, info.ray.getDir(), material);

				gmtl::Rayf refractionRay = gmtl::Rayf(info.position + t * 0.01f, t);

				refractionLight = rayTracing(world, refractionRay, recursivityDepth - 1) * material->Kt.GetColor(info);

			}
		}

		return Spectrum(difColor[0] + espColor[0] + ambientLight[0] + reflexionLight[0] + refractionLight[0],
						difColor[1] + espColor[1] + ambientLight[1] + reflexionLight[1] + refractionLight[1],
						difColor[2] + espColor[2] + ambientLight[2] + reflexionLight[2] + refractionLight[2]);

	}
	else
	{
		return Spectrum(0.0f, 0.0f, 0.0f);
	}
}


void render_image(World* world, unsigned int dimX, unsigned int dimY, float* image, float* alpha)
{

	gmtl::Rayf ray;

	for (int j = 0; j < dimY; j++)
	{
		for (int i = 0; i < dimX; i++)
		{

			ray = world->getCamera()->generateRay((float)i, (float)j);
			Spectrum color = rayTracing(world, ray, 2);

			image[(j*(dimX * 3) + i * 3)] = color[0];
			image[(j*(dimX * 3) + i * 3) + 1] = color[1];
			image[(j*(dimX * 3) + i * 3) + 2] = color[2];
			alpha[j*dimX + i] = 1.0;

		}
	}
}

unsigned int g_intersectTriangleCalls;
extern "C"
{
	__declspec(dllexport) void renderSceneFromFile(float*& image, float*& alpha, World*& world, const char* filename)
	{
		google::InitGoogleLogging("rendering.dll");
		FLAGS_logtostderr = 1;

		g_intersectTriangleCalls = 0;

		// Create world from file
		world = ReadFromFile(filename);
		if (!world)
		{
			fprintf(stderr, "Error reading file %s. Press enter to exit", filename);
			getc(stdin);
			return;
		}
		//INITREPORTER("report.ma", world);
		unsigned int dimX = world->getCamera()->getResolution()[0];
		unsigned int dimY = world->getCamera()->getResolution()[1];

		image = new float[dimX*dimY * 3];
		alpha = new float[dimX*dimY];


		// Compute pixel values
		clock_t tStart = clock();
		render_image(world, dimX, dimY, image, alpha);
		clock_t tEnd = clock();
		LOG(INFO) << "Time taken: " << (double)(tEnd - tStart) / CLOCKS_PER_SEC << "s";
		LOG(INFO) << "Triangles intersected: " << g_intersectTriangleCalls;

		google::ShutdownGoogleLogging();
	}

	__declspec(dllexport) void WriteImg(const char* name, float *pixels, float *alpha, int xRes,
		int yRes, int totalXRes, int totalYRes, int xOffset, int yOffset)
	{
		WriteImage(name, pixels, alpha, xRes, yRes, totalXRes, totalYRes, xOffset, yOffset);
	}
}

// dllmain.cpp : Defines the entry point for the DLL application.

BOOL APIENTRY DllMain(HMODULE hModule,
	DWORD  ul_reason_for_call,
	LPVOID lpReserved
)
{
	switch (ul_reason_for_call)
	{
	case DLL_PROCESS_ATTACH:
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	}
	return TRUE;
}
