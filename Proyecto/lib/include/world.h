#ifndef __WORLD_H__
#define __WORLD_H__

#include <vector>
#include <map>

#include <primitives.h>
#include <lights.h>
#include <camera.h>


struct IntersectInfo
{
    IntersectInfo() : objectID(InvalidObjectID) {}
    ObjectID objectID;
    const Material* material;
    Point3f position;
    Vector3f normal;
    Vector3f tangent;
    Ray ray;
    const World* world;
    float u;
    float v;
};


class World
{
public:
    typedef vector<Material*> Materials;
    typedef std::map<string, string> ShadingEngines;
    struct Texture
    {
        int width;
        int height;
        Spectrum* texture;

        Spectrum getColor(float u, float v) const;
    };
    typedef std::map<string, Texture> Textures;
    typedef std::map<string, string> TextureFiles;

    Primitive* mBVH;
    std::vector<Primitive*> mPrimitives;
    std::vector<std::string> mPrimitivesMaterial;
    std::vector<Light*> mLights;
    Light* mAmbientLight;
    Camera* mCamera;
    Materials mMaterials;
    ShadingEngines mShadingEngines;
    Textures mTextures;
    TextureFiles mTextureFiles;
    std::vector<MeshShape*> mMeshes;
    std::vector<std::string> mMeshesMaterial;
    std::vector<Primitive*> mMeshesPrimitives;

    World() : mAmbientLight(NULL), mCamera(NULL), mBVH(NULL) {}

    bool addPrimitive(Primitive* p, const char* material);
    bool addMesh(MeshShape* p, const char* material);
    bool addLight(Light* l);
    bool addTexture(const std::string& filename);
    bool setCamera(Camera* cam) { mCamera = cam; return true; }

    // Intersect the world with the ray and max length
    // Returns the distance to the closest intersection and fills the info structure if the rays collides with something
    // If the ray didn't collide with anything, return value will be maxLength and info.objectID will be InvalidObjectID
    float intersect(IntersectInfo& info, const Ray& ray, float maxlength = FLT_MAX/*, Spectrum* attenuation = NULL*/) const;
    bool shadow(const Ray& ray/*, Spectrum* attenuation = NULL*/, float maxlength = FLT_MAX) const;

    // Get the normal of the object at the point
    Vector3f getNormal(const Point3f& p, int objectID) const;
    // Get the color of the object at the position with the normal
    Vector3f getColor(const Point3f& p, const Vector3f& n, int objectID) const;
    // Get background color in the direction supplied
    Vector3f getBackground(const Vector3f& d) const;

    unsigned int getNumPrimitives() const;

    // Move all the objects to the right position for the time
    void moveObjects(float time);
    // Move the camera to the right position for the time
    Matrix43f moveCamera(float time);
    Camera* getCamera() { return mCamera;}

    const Texture* getTexture(const char* textureName) const;
    const Texture* getTextureFromNode(const char* nodeName) const;

    // Get the light information, sampling the lights in the scene (direct lighting)
    /////////////////////////////////gmtl::Vec3f applyLighting(const gmtl::Vec3f& pos, const gmtl::Vec3f& nor) const;

    Material* getMaterialFromShadingEngine(const char* shadingEngine) const;
    Material* getMaterial(const char* material) const;
    void addMaterial(Material* m) { mMaterials.push_back(m);}
    void addShadingEngine(const char* shadingEngine, const char* material) { mShadingEngines[string(shadingEngine)] = string(material);}
    void addTextureFile(const char* textureName, const char* filename) { mTextureFiles[string(textureName)] = string(filename);}

private:
    ObjectID getCurrentObjectID() const;
};

#endif