#include "bounding_volume_hierarchy.h"
#include "draw.h"
#include "interpolate.h"
#include "intersect.h"
#include "scene.h"
#include "texture.h"
#include <glm/glm.hpp>
#include <iostream>
#include <queue>

bool DEBUG = false;
BoundingVolumeHierarchy::BoundingVolumeHierarchy(Scene* pScene)
    : m_pScene(pScene)
{
    int level = 0;
    Mesh mesh;
    for (int i = 0; i < m_pScene->meshes.size(); i++) {
        mesh = pScene->meshes.at(i);
        for (int j = 0; j < mesh.triangles.size(); j++) {
            auto triangle = mesh.triangles.at(j);
            auto v1 = triangle.x;
            auto v2 = triangle.y;
            auto v3 = triangle.z;
            glm::vec3 a = mesh.vertices.at(v1).position;
            glm::vec3 b = mesh.vertices.at(v2).position;
            glm::vec3 c = mesh.vertices.at(v3).position;
            glm::vec3 centroid = (a + b + c) / 3.0f;
            centroids.push_back({ i, j, centroid.x, centroid.y, centroid.z });
        }
    }
    int maxLevels = std::ceil(std::log2(centroids.size())) + 1;
    createBVL(0, centroids.size(), 0, maxLevels);
    for (auto& node : m_nodes) {
        m_numLevels = std::max(node.level, m_numLevels);
    }
}

// Return the depth of the tree that you constructed. This is used to tell the
// slider in the UI how many steps it should display for Visual Debug 1.
int BoundingVolumeHierarchy::numLevels() const
{
    return m_numLevels + 1;
}

// Return the number of leaf nodes in the tree that you constructed. This is used to tell the
// slider in the UI how many steps it should display for Visual Debug 2.
int BoundingVolumeHierarchy::numLeaves() const
{
    return m_numLeaves - 1;
}

// Use this function to visualize your BVH. This is useful for debugging. Use the functions in
// draw.h to draw the various shapes. We have extended the AABB draw functions to support wireframe
// mode, arbitrary colors and transparency.
void BoundingVolumeHierarchy::debugDrawLevel(int level)
{
    for (auto& node : m_nodes) {
        if (node.level == level) {
            drawAABB(node.bounds, DrawMode::Wireframe);
        }
    }
}

// Use this function to visualize your leaf nodes. This is useful for debugging. The function
// receives the leaf node to be draw (think of the ith leaf node). Draw the AABB of the leaf node and all contained triangles.
// You can draw the triangles with different colors. NoteL leafIdx is not the index in the node vector, it is the
// i-th leaf node in the vector.
void BoundingVolumeHierarchy::debugDrawLeaf(int leafIdx)
{
    // Draw the AABB as a transparent green box.
    // AxisAlignedBox aabb{ glm::vec3(-0.05f), glm::vec3(0.05f, 1.05f, 1.05f) };
    // drawShape(aabb, DrawMode::Filled, glm::vec3(0.0f, 1.0f, 0.0f), 0.2f);
    std::vector<int> leaves;

    for (int i = 0; i < m_nodes.size(); i++) {
        if (m_nodes.at(i).isLeaf) {
            leaves.push_back(i);
        }
    }

    Node node = m_nodes.at(leaves.at(leafIdx));
    for (Children& child : node.children) {
        const Mesh mesh = m_pScene->meshes.at(child.mesh);
        const glm::uvec3 tri = mesh.triangles.at(child.triangle);
        const auto v0 = mesh.vertices[tri[0]];
        const auto v1 = mesh.vertices[tri[1]];
        const auto v2 = mesh.vertices[tri[2]];
        drawTriangle(v0, v1, v2);
    }
    drawAABB(node.bounds, DrawMode::Filled, glm::vec3(0.05f, 1.0f, 0.05f), 0.1f);

    // once you find the leaf node, you can use the function drawTriangle (from draw.h) to draw the contained primitives
}

// Return true if something is hit, returns false otherwise. Only find hits if they are closer than t stored
// in the ray and if the intersection is on the correct side of the origin (the new t >= 0). Replace the code
// by a bounding volume hierarchy acceleration structure as described in the assignment. You can change any
// file you like, including bounding_volume_hierarchy.h.
bool BoundingVolumeHierarchy::intersect(Ray& ray, HitInfo& hitInfo, const Features& features) const
{
    // If BVH is not enabled, use the naive implementation.
    if (!features.enableAccelStructure) {
        bool hit = false;
        // Intersect with all triangles of all meshes.
        for (const auto& mesh : m_pScene->meshes) {
            for (const auto& tri : mesh.triangles) {
                const auto v0 = mesh.vertices[tri[0]];
                const auto v1 = mesh.vertices[tri[1]];
                const auto& v2 = mesh.vertices[tri[2]];
                if (intersectRayWithTriangle(v0.position, v1.position, v2.position, ray, hitInfo)) {
                    // compute barycentric coordinates
                    hitInfo.barycentricCoord = computeBarycentricCoord(v0.position, v1.position, v2.position, ray.origin + ray.direction * ray.t);
                    hitInfo.material = mesh.material;
                    // if textures is enabled

                    if (hitInfo.material.kdTexture) {
                        // interpolate texcoord and set texel
                        glm::vec2 texCoord = interpolateTexCoord(v0.texCoord, v1.texCoord, v2.texCoord, hitInfo.barycentricCoord);
                        hitInfo.material.kd = acquireTexel(*hitInfo.material.kdTexture, texCoord, features);
                    }

                    // check if normal interpretation is on
                    if (features.enableNormalInterp) {
                        // normal interpolation
                        hitInfo.normal = interpolateNormal(v0.normal, v1.normal, v2.normal, hitInfo.barycentricCoord);
                    } else {
                        // no interpolation
                        hitInfo.normal = glm::cross(v1.position - v0.position, v2.position - v0.position);
                    }

                    hit = true;
                }
            }
        }
        // Intersect with spheres.
        for (const auto& sphere : m_pScene->spheres) {
            hit |= intersectRayWithShape(sphere, ray, hitInfo);
        }
        return hit;
    } else {
        std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<std::pair<float, int>>> queue;
        const Node root = m_nodes.back();
        float tHolder = ray.t;
        bool doesIntersect = intersectRayWithShape(root.bounds, ray);
        if (!doesIntersect) {
            return false;
        }
        queue.push({ ray.t, m_nodes.size() - 1 });
        ray.t = tHolder;
        bool hit = false;
        int indexOfTriangle = -1;
        int meshIndex = -1;
        while (!queue.empty()) {
            int indexOfTopNode = (queue.top().second);
            float nodesT = (queue.top().first);
            queue.pop();
            Node topNode = m_nodes.at(indexOfTopNode);
            if (DEBUG) {
                drawAABB(topNode.bounds, DrawMode::Wireframe);
            }
            if (tHolder < nodesT) {
                break;
            }

            if (topNode.isLeaf) {
                for (Children& child : topNode.children) {
                    const Mesh mesh = m_pScene->meshes.at(child.mesh);
                    const glm::uvec3 tri = mesh.triangles.at(child.triangle);
                    const auto v0 = mesh.vertices[tri[0]];
                    const auto v1 = mesh.vertices[tri[1]];
                    const auto v2 = mesh.vertices[tri[2]];
                    bool isIntersecting = intersectRayWithTriangle(v0.position, v1.position, v2.position, ray, hitInfo);
                    if (isIntersecting) {
                        if (ray.t < tHolder) {
                            indexOfTriangle = child.triangle;
                            meshIndex = child.mesh;
                            tHolder = ray.t;
                        }
                    }
                }
            } else {

                tHolder = ray.t;
                int indexOfLeftChild = topNode.children.at(0).node;
                int indexOfRightChild = topNode.children.at(1).node;
                bool intersectsWithLeftChild = intersectRayWithShape(m_nodes.at(indexOfLeftChild).bounds, ray);
                if (intersectsWithLeftChild) {
                    queue.push({ ray.t, indexOfLeftChild });
                }
                ray.t = tHolder;
                bool intersectsWithRightChild = intersectRayWithShape(m_nodes.at(indexOfRightChild).bounds, ray);
                if (intersectsWithRightChild) {
                    queue.push({ ray.t, indexOfRightChild });
                }
                ray.t = tHolder;
            }
        }
        // draw intersected but not visited nodes
        if (DEBUG) {
            while (!queue.empty()) {
                drawAABB(m_nodes.at(queue.top().second).bounds, DrawMode::Wireframe, { 1.0f, 1.0f, 0.5f });
                queue.pop();
            }
        }
        if (indexOfTriangle != -1) {
            const Mesh mesh = m_pScene->meshes.at(meshIndex);
            const glm::uvec3 tri = mesh.triangles.at(indexOfTriangle);
            const auto v0 = mesh.vertices[tri[0]];
            const auto v1 = mesh.vertices[tri[1]];
            const auto v2 = mesh.vertices[tri[2]];
            if (DEBUG) {
                drawTriangle(v0, v1, v2);
            }
            // compute barycentric coordinates
            hitInfo.barycentricCoord = computeBarycentricCoord(v0.position, v1.position, v2.position, ray.origin + ray.direction * ray.t);
            hitInfo.material = mesh.material;
            // if textures is enabled

            if (hitInfo.material.kdTexture) {
                // interpolate texcoord and set texel
                glm::vec2 texCoord = interpolateTexCoord(v0.texCoord, v1.texCoord, v2.texCoord, hitInfo.barycentricCoord);
                hitInfo.material.kd = acquireTexel(*hitInfo.material.kdTexture, texCoord, features);
            }

            // check if normal interpretation is on
            if (features.enableNormalInterp) {
                // normal interpolation
                hitInfo.normal = interpolateNormal(v0.normal, v1.normal, v2.normal, hitInfo.barycentricCoord);
            } else {
                // no interpolation
                hitInfo.normal = glm::cross(v1.position - v0.position, v2.position - v0.position);
            }

            hit = true;
        }
        return hit;
    }
}

void BoundingVolumeHierarchy::sortCentroidIndices(int startCentroid, int endCentroid, int axis)
{
    switch (axis) {
    case 0:
        std::sort(centroids.begin() + startCentroid, centroids.begin() + endCentroid, [](std::tuple<int, int, float, float, float> a, std::tuple<int, int, float, float, float> b) { return get<2>(a) > get<2>(b); });
        break;
    case 1:
        std::sort(centroids.begin() + startCentroid, centroids.begin() + endCentroid, [](std::tuple<int, int, float, float, float> a, std::tuple<int, int, float, float, float> b) { return get<3>(a) > get<3>(b); });
        break;
    case 2:
        std::sort(centroids.begin() + startCentroid, centroids.begin() + endCentroid, [](std::tuple<int, int, float, float, float> a, std::tuple<int, int, float, float, float> b) { return get<4>(a) > get<4>(b); });
        break;
    }
}
int BoundingVolumeHierarchy::createBVL(int start, int end, int level, int maxLevel)
{
    AxisAlignedBox currentAABB = getBox(start, end);
    if (end - start <= 1 || level == maxLevel) {
        this->m_numLeaves += 1;

        std::vector<Children> children;
        for (int i = start; i < end; i++) {
            int mesh = get<0>(centroids.at(i));
            int triangle = get<1>(centroids.at(i));
            children.push_back({ mesh, triangle, 0 });
        }
        m_nodes.push_back({ true,
            level,
            children,
            currentAABB });
        return m_nodes.size() - 1;
    }
    sortCentroidIndices(start, end, level % 3);
    int median = (start + end) / 2;
    int leftNode = createBVL(start, median, level + 1, maxLevel);
    int rightNode = createBVL(median, end, level + 1, maxLevel);
    std::vector<Children> children = { { 0, 0, leftNode }, { 0, 0, rightNode } };
    m_nodes.push_back({ false, level, children, currentAABB });
    return m_nodes.size() - 1;
}
AxisAlignedBox BoundingVolumeHierarchy::getBox(int start, int anEnd)
{
    AxisAlignedBox box;
    box.lower.x = std::numeric_limits<float>::max();
    box.lower.y = std::numeric_limits<float>::max();
    box.lower.z = std::numeric_limits<float>::max();
    box.upper.x = std::numeric_limits<float>::lowest();
    box.upper.y = std::numeric_limits<float>::lowest();
    box.upper.z = std::numeric_limits<float>::lowest();

    for (int i = start; i < anEnd; i++) {
        Mesh mesh = m_pScene->meshes.at(get<0>(centroids.at(i)));
        auto triangle = mesh.triangles.at(get<1>(centroids.at(i)));
        auto v1 = triangle.x;
        auto v2 = triangle.y;
        auto v3 = triangle.z;
        glm::vec3 a = mesh.vertices.at(v1).position;
        glm::vec3 b = mesh.vertices.at(v2).position;
        glm::vec3 c = mesh.vertices.at(v3).position;
        box.lower.x = std::min(box.lower.x, std::min(a.x, std::min(b.x, c.x)));
        box.lower.y = std::min(box.lower.y, std::min(a.y, std::min(b.y, c.y)));
        box.lower.z = std::min(box.lower.z, std::min(a.z, std::min(b.z, c.z)));
        box.upper.x = std::max(box.upper.x, std::max(a.x, std::max(b.x, c.x)));
        box.upper.y = std::max(box.upper.y, std::max(a.y, std::max(b.y, c.y)));
        box.upper.z = std::max(box.upper.z, std::max(a.z, std::max(b.z, c.z)));
    }
    return box;
}