#include "interpolate.h"
#include <glm/geometric.hpp>

float areaOfTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2) {
    glm::vec3 v1v0 = v1 - v0;
    glm::vec3 v2v0 = v2 - v0;
    return 0.5 * glm::length(glm::cross(v2v0, v1v0));
}


glm::vec3 computeBarycentricCoord (const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& p)
{
    // Computes barycentric coord as percentage of triangle
    float totalArea = areaOfTriangle(v0, v1, v2);
    float b0 = areaOfTriangle(p, v1, v2) / totalArea;
    float b1 = areaOfTriangle(p, v0, v2) / totalArea;
    float b2 = areaOfTriangle(p, v0, v1) / totalArea;
    return glm::vec3 {b0, b1, b2};
}

glm::vec3 interpolateNormal(const glm::vec3& n0, const glm::vec3& n1, const glm::vec3& n2, const glm::vec3 barycentricCoord)
{
    // Computes normal by scaling each vertex normal by respective percentage of barycentric  coord.
    float b0 = barycentricCoord.x;
    float b1 = barycentricCoord.y;
    float b2 = barycentricCoord.z;

    return b0 * n0 + b1 * n1 + b2 * n2;
}

glm::vec2 interpolateTexCoord (const glm::vec2& t0, const glm::vec2& t1, const glm::vec2& t2, const glm::vec3 barycentricCoord)
{
    // Computes interpolated texture coord by scaling given texture coords by respective percentage of barycentric  coord.
    float b0 = barycentricCoord.x;
    float b1 = barycentricCoord.y;
    float b2 = barycentricCoord.z;

    return b0 * t0 + b1 * t1 + b2 * t2 ;
}
