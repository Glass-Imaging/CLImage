/*******************************************************************************
 * Copyright (c) 2021 Glass Imaging Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************/

const sampler_t sampler = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_CLAMP_TO_EDGE | CLK_FILTER_NEAREST;

float4 boxBlur(image2d_t blurMap, int2 imageCoordinates) {
    // Blur radius at the given image coordinates, average out over a neighborhood to reduce the cutout effect
    const int filterSize = 15;
    float4 blur = 0;
    for (int y = -filterSize / 2; y <= filterSize / 2; y++) {
        for (int x = -filterSize / 2; x <= filterSize / 2; x++) {
            int2 sampleCoordinate = imageCoordinates + (int2)(x, y);
            float4 blurSample = read_imagef(blurMap, sampler, sampleCoordinate);
            blur += blurSample;
        }
    }
    return blur / (filterSize * filterSize);
}

// Use the PSF data at various disk sizes, no kernel interpolation
kernel void blur(read_only image2d_t input, write_only image2d_t output)
{
    const int2 imageCoordinates = (int2) (get_global_id(0), get_global_id(1));
    float4 result = boxBlur(input, imageCoordinates);
    write_imagef(output, imageCoordinates, result);
}
