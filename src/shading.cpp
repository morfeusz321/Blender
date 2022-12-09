#include "texture.h"
#include <cmath>
#include <glm/geometric.hpp>
#include <shading.h>
#include <iostream>


const glm::vec3 computeShading(const glm::vec3& lightPosition, const glm::vec3& lightColor, const Features& features, Ray ray, HitInfo hitInfo)
{
    float epsilon = 0.00001;

    if (!features.enableShading) {
        return hitInfo.material.kd;
    }

    
    glm::vec3 hitPosition = ray.direction * ray.t + ray.origin; //intersect position
    glm::vec3 lightVector = glm::normalize(lightPosition - hitPosition); //light vector
    glm::vec3 cameraVector = glm::normalize(ray.direction); // camera vector using ray.origin as camera position
    glm::vec3 normal = glm::normalize(hitInfo.normal);


    // diffuse term
    float diffuseAngle = glm::dot(normal, lightVector);
    glm::vec3 diffuseTerm;
    if (diffuseAngle > epsilon) {
        diffuseTerm = lightColor * hitInfo.material.kd * diffuseAngle;
    } else {
        diffuseTerm = { 0, 0, 0 };
    }

    // specular term
    float specularAngle = glm::dot(cameraVector, glm::reflect(-lightVector, normal));
    glm::vec3 specularTerm;
    if (specularAngle > epsilon) {
        specularTerm = lightColor * hitInfo.material.ks * glm::pow(specularAngle, hitInfo.material.shininess);
    } else {
        specularTerm = { 0, 0, 0 };
    }

    return diffuseTerm + specularTerm;
}


const Ray computeReflectionRay (Ray ray, HitInfo hitInfo)
{
    // Do NOT use glm::reflect!! write your own code.
    float epsilon = 0.001;


    Ray reflectionRay {};

    //reflect ray starts from intersection 
    reflectionRay.origin = ray.origin + ray.direction * (ray.t - epsilon) ;

    //reflection direction calculation R = I - 2(N.I)*N
    glm::vec3 incident = glm::normalize(ray.direction);
    glm::vec3 normal = glm::normalize(hitInfo.normal);
    reflectionRay.direction = glm::normalize(incident - 2.0f * (glm::dot(normal, incident) * normal));

    //set t to max because no intersection defined yet
    reflectionRay.t = INT_MAX;

    return reflectionRay;
}

