#include "texture.h"
#include <framework/image.h>
#include <iostream>
glm::vec3 acquireTexel(const Image& image, const glm::vec2& texCoord, const Features& features)
{
    // TODO: implement this function.
    // Given texcoords, return the corresponding pixel of the image
    // The pixel are stored in a 1D array of row major order
    // you can convert from position (i,j) to an index using the method seen in the lecture
    // Note, the center of the first pixel is at image coordinates (0.5, 0.5)

    if (features.enableTextureMapping) {
        int i = (int) std::round(texCoord.x * image.width - 0.5) % image.width;
        int j = (int) std::round(texCoord.y * image.height - 0.5) % image.height;

        //clamp values so no index out of bounds occur
        if (i < 0) {
            i = 0;
        }
        if (j < 0) {
            j = 0;
        }
        int index = j * image.width + i;

        return image.pixels[index];

    } else {
        return image.pixels[0];
    }   
}