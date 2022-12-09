#include "light.h"
#include "config.h"
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/geometric.hpp>
DISABLE_WARNINGS_POP()
#include <cmath>

std::vector<float> jitterLine(int bins)
{
    std::vector<float> result;

    // calculate the distance between bins
    const float d = 1 / (float)bins;

    for (int i = 0; i < bins; i++) {
        // find the t position which is within the current bin
        float t = i * d + ((float)rand() / (RAND_MAX)) * d;

        if (t > 1.0f) {
            t = 1.0f;
        }

        result.push_back(t);
    }

    return result;

}

// jitterSquare returns the tx and ty values for a jittered square with binsX x binsY bins
std::vector<std::pair<float, float>> jitterSquare(int binsX, int binsY)
{
    std::vector<std::pair<float, float>> result;

    // calculate the distance between bins
    const float dx = 1 / (float)binsX;
    const float dy = 1 / (float)binsY;

    for (int i = 0; i < binsX; i++) {
        for (int j = 0; j < binsY; j++) {
            // find the x position which is within the current x bin
            float x = i * dx + ((float)rand() / (RAND_MAX)) * dx;
            // find the y position which is within the current y bin
            float y = j * dy + ((float)rand() / (RAND_MAX)) * dy;

            if (x > 1.0f) {
                x = 1.0f;
            }
            if (y > 1.0f) {
                y = 1.0f;
            }

            result.push_back(std::pair(x, y));
        }
    }

    return result;
}

// samples a segment light source
// you should fill in the vectors position and color with the sampled position and color
// The parameter t was added in order to support jittering of the light samples
void sampleSegmentLight(const SegmentLight& segmentLight, const float t, glm::vec3& position, glm::vec3& color)
{
    position = glm::vec3(0.0);
    color = glm::vec3(0.0);
    // TODO: implement this function.

    // Linear interpolation over the light with the t value as the position along the light
    position = segmentLight.endpoint0 + t * (segmentLight.endpoint1 - segmentLight.endpoint0);
    color = segmentLight.color0 + t * (segmentLight.color1 - segmentLight.color0);

}

// samples a parallelogram light source
// you should fill in the vectors position and color with the sampled position and color
// The parameters t0 and t1 were added in order to support jittering of the light samples
void sampleParallelogramLight(const ParallelogramLight& parallelogramLight, const float t0, const float t1, glm::vec3& position, glm::vec3& color)
{
    position = glm::vec3(0.0);
    color = glm::vec3(0.0);
    // TODO: implement this function.

    position = parallelogramLight.v0 + t0 * parallelogramLight.edge01 + t1 * parallelogramLight.edge02;

    // For bilinear interpolation first do linear interpolation along opposite edges of the parallelogram
    const glm::vec3 c0 = parallelogramLight.color0 + t0 * (parallelogramLight.color1 - parallelogramLight.color0);
    const glm::vec3 c1 = parallelogramLight.color2 + t0 * (parallelogramLight.color3 - parallelogramLight.color2);

    // Next do linear interpolation between the edges
    color = c0 + t1 * (c1 - c0);
}

// test the visibility at a given light sample
// returns 1.0 if sample is visible, 0.0 otherwise
float testVisibilityLightSample(const glm::vec3& samplePos, const glm::vec3& debugColor, const BvhInterface& bvh, const Features& features, Ray ray, HitInfo hitInfo)
{
    // The factor by which the shadow ray gets offset by the direction vector
    // This value was found by trial and error
    const float DEPTH_BIAS = 0.001f;


    // TODO: implement this function.
    if (!features.enableHardShadow) {
        return 1.0f;
    }

    // TODO (Friso): Add orientation and depth dependent factors to the offset

    // calculate the position of where the ray intersects the scene
    const glm::vec3 hitPosition = ray.origin + ray.direction * ray.t;

    // Calculate the values for the shadow position with the offset applied
    // The position from where we calculate the ray is slightly offset in the direction of the camera
    // This is done in order to prevent shadow acne
    const glm::vec3 offsetPosition = hitPosition - ray.direction * (DEPTH_BIAS);
    glm::vec3 offsetDirection = samplePos - offsetPosition;

    // Calculate the values for the ray from the offset intersection point to the light point
    float offsetLength = glm::length(offsetDirection);
    offsetDirection = glm::normalize(offsetDirection);

    // populate the ray apply with the offset values
    Ray directionRay = Ray
    {
        offsetPosition, offsetDirection, offsetLength
    };

    // Find out if the light is visible from the intersection point
    const bool obstructed = bvh.intersect(directionRay, hitInfo, features);

    if (obstructed) {
        drawRay(directionRay, glm::vec3(1.0f, 0.0f, 0.0f));
        return 0.0f;
    } else {
        drawRay(directionRay, debugColor);
        return 1.0f;
    }
}

// given an intersection, computes the contribution from all light sources at the intersection point
// in this method you should cycle the light sources and for each one compute their contribution
// don't forget to check for visibility (shadows!)

// Lights are stored in a single array (scene.lights) where each item can be either a PointLight, SegmentLight or ParallelogramLight.
// You can check whether a light at index i is a PointLight using std::holds_alternative:
// std::holds_alternative<PointLight>(scene.lights[i])
//
// If it is indeed a point light, you can "convert" it to the correct type using std::get:
// PointLight pointLight = std::get<PointLight>(scene.lights[i]);
//
//
// The code to iterate over the lights thus looks like this:
// for (const auto& light : scene.lights) {
//     if (std::holds_alternative<PointLight>(light)) {
//         const PointLight pointLight = std::get<PointLight>(light);
//         // Perform your calculations for a point light.
//     } else if (std::holds_alternative<SegmentLight>(light)) {
//         const SegmentLight segmentLight = std::get<SegmentLight>(light);
//         // Perform your calculations for a segment light.
//     } else if (std::holds_alternative<ParallelogramLight>(light)) {
//         const ParallelogramLight parallelogramLight = std::get<ParallelogramLight>(light);
//         // Perform your calculations for a parallelogram light.
//     }
// }
//
// Regarding the soft shadows for **other** light sources **extra** feature:
// To add a new light source, define your new light struct in scene.h and modify the Scene struct (also in scene.h)
// by adding your new custom light type to the lights std::variant. For example:
// std::vector<std::variant<PointLight, SegmentLight, ParallelogramLight, MyCustomLightType>> lights;
//
// You can add the light sources programmatically by creating a custom scene (modify the Custom case in the
// loadScene function in scene.cpp). Custom lights will not be visible in rasterization view.
glm::vec3 computeLightContribution(const Scene& scene, const BvhInterface& bvh, const Features& features, Ray ray, HitInfo hitInfo)
{
    // The amount of bins to use for the jittering of segment lights
    const int segmentBins = 25;
    // the amount of bins to use for the jittering of of parallelogram lights
    // along both the direction of edge 00 and edge 01
    const int parallelogramBins = 10;

    if (features.enableShading) {
         glm::vec3 totalContribution {0,0,0}; 
         // If shading is enabled, compute the contribution from all lights.
         
         //position and color for sampled 
         glm::vec3 position;
         glm::vec3 color;
         for (const auto& light : scene.lights) {

             if (std::holds_alternative<PointLight>(light)) {
                 const PointLight pointLight = std::get<PointLight>(light);
                 totalContribution += computeShading(pointLight.position, pointLight.color, features, ray, hitInfo) 
                     * testVisibilityLightSample(pointLight.position, glm::vec3(0.0f), bvh, features, ray, hitInfo);
             }

             else if (std::holds_alternative<SegmentLight>(light)) {
                 const SegmentLight segmentLight = std::get<SegmentLight>(light);

                 if (features.enableSoftShadow) {
                     glm::vec3 avg = glm::vec3(0.0f);

                     auto points = jitterLine(segmentBins);

                     // Take n samples of the light and calculate the average of those samples
                     for (const float t : points) {
                         sampleSegmentLight(segmentLight, t, position, color);

                         avg += computeShading(position, color, features, ray, hitInfo)
                             * testVisibilityLightSample(position, glm::vec3(0.0f), bvh, features, ray, hitInfo);
                     }
                     // Add the average of the samples to the total
                     totalContribution += avg / (float)(points.size());
                 } else {
                     totalContribution += computeShading(position, color, features, ray, hitInfo);
                 }
             }

             else if (std::holds_alternative<ParallelogramLight>(light)) {
                 const ParallelogramLight parallelogramLight = std::get<ParallelogramLight>(light);

                 if (features.enableSoftShadow) {
                     glm::vec3 avg = glm::vec3(0.0f);

                     auto points = jitterSquare(parallelogramBins, parallelogramBins);

                     // Take n samples of the light and calculate the average of those samples
                     for (const auto point: points) {
                         sampleParallelogramLight(parallelogramLight, point.first, point.second, position, color);

                         avg += computeShading(position, color, features, ray, hitInfo)
                             * testVisibilityLightSample(position, glm::vec3(0.0f), bvh, features, ray, hitInfo);
                     }
                     // Add the average of the samples to the total
                     totalContribution += avg / (float)(points.size());
                 } else {
                     totalContribution += computeShading(position, color, features, ray, hitInfo);
                 }
             }
         }
         return totalContribution;
    } else {
        // If shading is disabled, return the albedo of the material.
        return hitInfo.material.kd;
    }
}
