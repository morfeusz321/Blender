#pragma once
#include "common.h"
#include <array>
#include <framework/ray.h>
#include <vector>

// Forward declaration.
struct Scene;

struct Children{
    int mesh;
    int triangle;
    int node;
};

struct Node{
    bool isLeaf;
    int level;
    std::vector<Children> children;
    AxisAlignedBox bounds;
};

class BoundingVolumeHierarchy {
public:
    // Constructor. Receives the scene and builds the bounding volume hierarchy.
    BoundingVolumeHierarchy(Scene* pScene);

    // Return how many levels there are in the tree that you have constructed.
    [[nodiscard]] int numLevels() const;

    // Return how many leaf nodes there are in the tree that you have constructed.
    [[nodiscard]] int numLeaves() const;

    // Visual Debug 1: Draw the bounding boxes of the nodes at the selected level.
    void debugDrawLevel(int level);

    // Visual Debug 2: Draw the triangles of the i-th leaf
    void debugDrawLeaf(int leafIdx);

    // Return true if something is hit, returns false otherwise.
    // Only find hits if they are closer than t stored in the ray and the intersection
    // is on the correct side of the origin (the new t >= 0).
    bool intersect(Ray& ray, HitInfo& hitInfo, const Features& features) const;


private:
    int m_numLevels{};
    int m_numLeaves{};
    Scene* m_pScene;
    Mesh m_mesh;
    int levels;
    int nodes;
    std::vector<Node> m_nodes;
    //(mesh, triangle, x , y, z)
    std::vector<std::tuple<int,int,float,float,float>> centroids;
    void sortCentroidIndices(int startCentroid, int endCentroid, int axis);
    int createBVL(int start, int end, int level, int maxLevel);
    AxisAlignedBox getBox(int start, int anEnd);
};