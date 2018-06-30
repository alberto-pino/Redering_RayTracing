#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <core.h>
#include <transform.h>
#include <gmtl/gmtl.h>

class TextParser;
class Camera
{
public:
    Camera(float fov);

    void setWindow(const float screenWindow[4], const unsigned int resolution[2]);

    gmtl::Rayf generateRay(float pixelX, float pixelY);

    virtual gmtl::Point2ui getResolution() const;

    void setOutputPath(const char* filename);
    const char* getOutputPath() const;

	void setCameraToWorld(const Transform& trans) { mCameraToWorld = trans;}
#ifdef NODE_NAME_SUPPORT
	void SetName(const std::string& name) { mName = name;}
	const char* GetName() const { return mName.c_str();}
	std::string mName;
#endif
private:
    Transform mCameraToWorld;

    Transform mRasterToCamera;
    Transform mCameraToScreen;
    Transform mScreenToRaster;

    gmtl::Point2ui mResolution;

    std::string mOutputPath;
};

#endif