#ifndef __KDTREE_H__
#define __KDTREE_H__

#include <vector>
#include <stack>
#include <set>
#include <primitives.h>
class KDTreeNode
{
public:
    Bounds3f bbox;
    struct Children
    {
        KDTreeNode* left;
        KDTreeNode* right;
    };

    union Data
    {
        Children children;
    };

    Data data;
    std::vector<TrianglePrimitive*> prims;
    int axis;
    float splitPos;
    bool isLeaf;

    KDTreeNode();

    bool intersectLeft() const;
    bool intersectRight() const;

    //void Add(TrianglePrimitive* p, PrimitiveList* l);
    void Add(TrianglePrimitive* p);

    void SetAxis(int a) { axis = a;}
    void SetLeft(KDTreeNode* l) { data.children.left = l;}
    void SetRight(KDTreeNode* r) { data.children.right = r;}
    void SetIsLeaf(bool l) { isLeaf = l;}
    void SetSplitPos(float p) { splitPos = p;}
    void SetBbox(const Bounds3f& b) { bbox = b; }

    int GetAxis() const { return axis;}
    KDTreeNode* GetLeft() const { return data.children.left;}
    KDTreeNode* GetRight() const { return data.children.right;}
    bool IsLeaf() const { return isLeaf;}
    float GetSplitPos() const { return splitPos;}
    Bounds3f& GetBbox() { return bbox; }
    const Bounds3f& GetBbox() const { return bbox; }

    
};

struct SplitNode
{
    float splitPos;
    mutable int leftCount, rightCount;
    bool operator<(const SplitNode& n) const
    {
        return splitPos < n.splitPos;
    }
    bool operator==(const SplitNode& n) const
    {
        return splitPos == n.splitPos;
    }
};

struct kdstack;
class KDTree: public Primitive
{
public:
    KDTree(const std::vector<Primitive*> &prims, const World* w);
    ~KDTree();

    Bounds3f WorldBound() const { return m_Root->bbox; }
    bool intersect(const Ray &ray, float& t, IntersectInfo& isect) const;
    bool intersect(const Ray& ray, float& t) const;
    bool intersect(const Ray &ray) const;

    virtual const Material *GetMaterial() const { return NULL; }
private:
    void Subdivide( KDTreeNode* a_Node, const Bounds3f& bbox, int a_Depth, int a_Prims );
    void InsertSplitPos(std::set<SplitNode>& splitPos, float a_SplitPos);
    bool findLeafIntersection(bool* test_isect, ObjectID& bestID, float& bestT,
        IntersectInfo& bestInfo, const KDTreeNode* node, const Ray& ray) const;

    KDTreeNode* m_Root;
    static const int c_MaxTreeDepth;
    //kdstack* m_Stack;
    mutable std::vector<kdstack> m_Stack;
    int* m_Mod;
    const World* mWorld;

    bool intersectOld(const Ray &ray, float& t, IntersectInfo& isect) const;

};


KDTree* CreateKDTree(const std::vector<Primitive*> &prims, const World* w);

#endif