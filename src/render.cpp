#include "render.h"
#include "intersect.h"
#include "light.h"
#include "screen.h"
#include <framework/trackball.h>
#include <random>
#ifdef NDEBUG
#include <iostream>
#include <omp.h>
#endif
#include <iostream>

std::pair<glm::vec3, glm::vec3> constructBasis(const glm::vec3& r)
{
    // first basis vector is normalized r
    glm::vec3 w = glm::normalize(r);

    // find some non-collinear vector by changing the smallest magnitude component to 1
    glm::vec3 t = w;
    if (std::abs(t.x) < std::abs(t.y)) {
        if (std::abs(t.x) < std::abs(t.z)) {
            t.x = 1;
        } else {
            t.z = 1;
        }
    } else {
        if (std::abs(t.y) < std::abs(t.z)) {
            t.y = 1;
        } else {
            t.z = 1;
        }
    }
    // second basis vector u is cross of t,w
    glm::vec3 u = glm::cross(t, w);
    u = glm::normalize(u);

    // third basis vector is cross of first two basis vectors
    glm::vec3 v = glm::cross(u, w);
    v = glm::normalize(v);

    return std::pair { u, v };
}

glm::vec2 getRandomSquareSample(const glm::vec3& u, const glm::vec3& v, float a)
{
    float n1 = (float)rand() / RAND_MAX;
    float n2 = (float)rand() / RAND_MAX;

    float u1 = -a / (float)2 + n1 * a;
    float u2 = -a / (float)2 + n2 * a;
    return glm::vec2 { u1, u2 };
}

std::vector<Ray> getGlossyRays(Ray reflection, float a, int samples)
{
    std::vector<Ray> glossyRays;

    // construct a square perpendicular to the ray
    std::pair<glm::vec3, glm::vec3> basis = constructBasis(reflection.direction);
    // for each sample, construct new ray
    for (int i = 0; i < samples; i++) {
        Ray randomizedReflection = reflection;

        // get a random sample from the square and construct new randomzied direction such that
        // r'= r + c_1*u + c_2*v where u,v are basis vectors and c_1, c_2 are coefficients
        glm::vec2 coefficients = getRandomSquareSample(basis.first, basis.second, a);
        randomizedReflection.direction = glm::normalize(reflection.direction + coefficients.x * basis.first + coefficients.y + basis.second);

        // add ray to list
        glossyRays.push_back(randomizedReflection);
    }
    return glossyRays;
}


glm::vec3 getFinalColor(const Scene& scene, const BvhInterface& bvh, Ray ray, const Features& features, int rayDepth)
{
    HitInfo hitInfo;
    // max depth for recursive ray trace
    int maxDepth = 3;
    if (bvh.intersect(ray, hitInfo, features)) {
        if (features.extra.enableDepthOfField && rayDepth == 0) {
            // The amount of bins in the x and y direction used to calculate the depth of field effect
            const int bins = 10;

            // The focal distance and aperture used to calculate the depth of field effect
            const float focalDistance = 3.0f;
            const float aperture = 0.1f;

            glm::vec3 result = glm::vec3(0.0f);

            // Calculate the convergence point through which all points used will pass
            const glm::vec3 convergencePoint = ray.origin + ray.direction * focalDistance;

            // find jittered points on the square lens
            auto points = jitterSquare(bins, bins);

            for (const auto& point : points) {
                // Calculate the offset origin of the point from which the ray will be shot
                const glm::vec3 offset = glm::vec3(point.first - 0.5f, point.second - 0.5f, 0.0f);
                const glm::vec3 offsetOrigin = ray.origin + offset * aperture;

                // Calculate the offset ray which passed through the convergence point
                Ray offsetRay = Ray { offsetOrigin, glm::normalize(convergencePoint - offsetOrigin) };

                if (bvh.intersect(offsetRay, hitInfo, features)) {

                    glm::vec3 Lo = computeLightContribution(scene, bvh, features, offsetRay, hitInfo);

                    if (features.enableRecursive) {
                        Ray reflection = computeReflectionRay(offsetRay, hitInfo);
                        // if ray depth exceeds maxDepth
                        if (rayDepth < maxDepth) {
                            if (!(hitInfo.material.ks.x == 0 && hitInfo.material.ks.y == 0 && hitInfo.material.ks.z == 0)) {
                                if (features.extra.enableGlossyReflection) {
                                    float a = 0.5; // degree of blur
                                    int samples = 10; // amount of samples to gloss
                                    std::vector<Ray> glossyRays = getGlossyRays(reflection, a, samples);
                                    glm::vec3 normalizedColor { 0, 0, 0 };
                                    // for each random ray, get color
                                    for (Ray& randRay : glossyRays) {
                                        normalizedColor += getFinalColor(scene, bvh, randRay, features, rayDepth + 1);
                                    }
                                    // average the color before adding
                                    normalizedColor = normalizedColor / (float)samples;
                                    Lo += normalizedColor;
                                } else {
                                    Lo += getFinalColor(scene, bvh, reflection, features, rayDepth + 1);
                                }
                            }
                        }
                    }

                    // Draw a white debug ray if the ray hits.
                    drawRay(offsetRay, Lo);

                    // Draw the ray of the normal for debug
                    if (features.enableNormalInterp) {
                        Ray normalRay { offsetRay.origin + offsetRay.direction * offsetRay.t, glm::normalize(hitInfo.normal), 5 };
                        drawRay(normalRay, { 0, 1, 0 });
                    }

                    // Set the color of the pixel to white if the ray hits.
                    result += Lo;
                } else {
                    // Draw a red debug ray if the ray missed.
                    drawRay(offsetRay, glm::vec3(1.0f, 0.0f, 0.0f));
                    // Set the color of the pixel to black if the ray misses.
                    result += glm::vec3(0.0f);
                }
            }

            return result / (float)(points.size());
        } else {
            glm::vec3 Lo = computeLightContribution(scene, bvh, features, ray, hitInfo);

            if (features.enableRecursive) {
                Ray reflection = computeReflectionRay(ray, hitInfo);
                // if ray depth exceeds maxDepth
                if (rayDepth < maxDepth) {
                    if (!(hitInfo.material.ks.x == 0 && hitInfo.material.ks.y == 0 && hitInfo.material.ks.z == 0)) {
                        if (features.extra.enableGlossyReflection) {
                            float a = 0.5; // degree of blur
                            int samples = 10; // amount of samples to gloss
                            std::vector<Ray> glossyRays = getGlossyRays(reflection, a, samples);
                            glm::vec3 normalizedColor { 0, 0, 0 };
                            // for each random ray, get color
                            for (Ray& randRay : glossyRays) {
                                normalizedColor += getFinalColor(scene, bvh, randRay, features, rayDepth + 1);
                            }
                            // average the color before adding
                            normalizedColor = normalizedColor / (float)samples;
                            Lo += normalizedColor;
                        } else {
                            Lo += getFinalColor(scene, bvh, reflection, features, rayDepth + 1);
                        }
                    }
                }
            }

            // Draw a white debug ray if the ray hits
            drawRay(ray, Lo);

            // Draw the ray of the normal for debug
            if (features.enableNormalInterp) {
                Ray normalRay { ray.origin + ray.direction * ray.t, glm::normalize(hitInfo.normal), 5 };
                drawRay(normalRay, { 0, 1, 0 });
            }

            // Set the color of the pixel to white if the ray hits.
            return Lo;
        }
    } else {
        // Draw a red debug ray if the ray missed.
        drawRay(ray, glm::vec3(1.0f, 0.0f, 0.0f));
        // Set the color of the pixel to black if the ray misses.
        return glm::vec3(0.0f);
    }
}

std::pair<glm::vec3, std::vector<Ray>> jitterSampling(int x, int y, glm::ivec2 windowResolution, const Scene& scene, const Trackball& camera, const BvhInterface& bvh, Screen& screen, const Features& features, bool debug)
{
    // random seed for random distribution
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist(0.001, 0.999);

    // number of samples (will be n^2)
    int n = 5;

    // vector for rays
    std::vector<Ray> rayList;

    glm::vec3 C { 0, 0, 0 };
    for (int p = 0; p < n; p++) {
        for (int q = 0; q < n; q++) {
            // calculate normalized pos of x + p + random number so that it gets jittered rays along a grid
            const glm::vec2 normalizedPixelPos {
                float(x + (p + dist(gen)) / n) / float(windowResolution.x) * 2.0f - 1.0f,
                float(y + (q + dist(gen)) / n) / float(windowResolution.y) * 2.0f - 1.0f
            };

            // get new camera ray and get its respective color
            const Ray cameraRay = camera.generateRay(normalizedPixelPos);
            glm::vec3 color = getFinalColor(scene, bvh, cameraRay, features);

            if (debug) {
                rayList.push_back(cameraRay);
            }

            C += color;
        }
    }

    // average final color
    C = ((float)1 / (float)(n * n)) * C;
    return std::pair { C, rayList };
}

void renderRayTracing(const Scene& scene, const Trackball& camera, const BvhInterface& bvh, Screen& screen, const Features& features)
{
    glm::ivec2 windowResolution = screen.resolution();
    // Enable multi threading in Release mode
#ifdef NDEBUG
#pragma omp parallel for schedule(guided)
#endif
    for (int y = 0; y < windowResolution.y; y++) {
        for (int x = 0; x != windowResolution.x; x++) {
            if (features.extra.enableMultipleRaysPerPixel) {
                screen.setPixel(x, y, jitterSampling(x, y, windowResolution, scene, camera, bvh, screen, features, false).first);
            } else {
                const glm::vec2 normalizedPixelPos {
                    float(x) / float(windowResolution.x) * 2.0f - 1.0f,
                    float(y) / float(windowResolution.y) * 2.0f - 1.0f
                };
                const Ray cameraRay = camera.generateRay(normalizedPixelPos);
                screen.setPixel(x, y, getFinalColor(scene, bvh, cameraRay, features));
            }
        }
    }
}
