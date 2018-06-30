#include <camera.h>

gmtl::Rayf Camera::generateRay(float pixelX, float pixelY)
{

	gmtl::Rayf ray;

	gmtl::Point3f origen(0, 0, 0);
	gmtl::Point3f cameraFinal = mCameraToWorld(origen);
	ray.setOrigin(cameraFinal);

	gmtl::Point3f pixel(pixelX, pixelY, 0);
	Transform transformation = mCameraToWorld * mRasterToCamera;
	gmtl::Point3f pixelFinal = transformation(pixel);

	gmtl::Vec3f pixelDir(pixelFinal - cameraFinal);
	normalize(pixelDir);
	ray.setDir(pixelDir);

	return ray;

}


gmtl::Point2ui Camera::getResolution() const
{
    return mResolution;
}

void Camera::setOutputPath(const char* filename)
{
    mOutputPath = filename;
}

const char* Camera::getOutputPath() const
{
    return mOutputPath.c_str();
}