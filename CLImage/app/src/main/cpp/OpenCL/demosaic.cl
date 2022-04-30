// Copyright (c) 2021-2022 Glass Imaging Inc.
// Author: Fabio Riccardi <fabio@glass-imaging.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

enum BayerPattern {
    grbg = 0,
    gbrg = 1,
    rggb = 2,
    bggr = 3
};

enum { raw_red = 0, raw_green = 1, raw_blue = 2, raw_green2 = 3 };

constant const int2 bayerOffsets[4][4] = {
    { {1, 0}, {0, 0}, {0, 1}, {1, 1} }, // grbg
    { {0, 1}, {0, 0}, {1, 0}, {1, 1} }, // gbrg
    { {0, 0}, {1, 0}, {1, 1}, {0, 1} }, // rggb
    { {1, 1}, {1, 0}, {0, 0}, {0, 1} }  // bggr
};

#if defined(__QCOMM_QGPU_A3X__) || \
    defined(__QCOMM_QGPU_A4X__) || \
    defined(__QCOMM_QGPU_A5X__) || \
    defined(__QCOMM_QGPU_A6X__) || \
    defined(__QCOMM_QGPU_A7V__) || \
    defined(__QCOMM_QGPU_A7P__)

// Qualcomm's smoothstep implementation can be really slow...

#define _fabs(a) \
   ({ __typeof__ (a) _a = (a); \
     _a >= 0 ? _a : - _a; })

#define _fmax(a, b) \
   ({ __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define _fmin(a, b) \
   ({ __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

#define _clamp(x, minval, maxval) \
   ({ __typeof__ (x) _x = (x); \
      __typeof__ (minval) _minval = (minval); \
      __typeof__ (maxval) _maxval = (maxval); \
     _x < _minval ? _minval : _x > _maxval ? _maxval : _x; })

#define _mix(x, y, a) \
   ({ __typeof__ (x) _x = (x); \
      __typeof__ (y) _y = (y); \
      __typeof__ (a) _a = (a); \
      _x + (_y - _x) * _a; })

#define _smoothstep(a, b, x) \
   ({ __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
      __typeof__ (x) _x = (x), t; \
      t = _clamp(_x * (1 / (_b - _a)) - (_a / (_b - _a)), 0, 1); \
      t * t * (3 - 2 * t); })

#define fabs _fabs
#define fmax _fmax
#define fmin _fmin
#define clamp _clamp
#define mix _mix
#define smoothstep _smoothstep

#endif

// Work on one Quad (2x2) at a time
kernel void scaleRawData(read_only image2d_t rawImage, write_only image2d_t scaledRawImage,
                         int bayerPattern, constant float scaleMul[4], float blackLevel) {
    const int2 imageCoordinates = (int2) (2 * get_global_id(0), 2 * get_global_id(1));
    for (int c = 0; c < 4; c++) {
        int2 o = bayerOffsets[bayerPattern][c];
        write_imagef(scaledRawImage, imageCoordinates + (int2) (o.x, o.y),
                     clamp(scaleMul[c] * (read_imagef(rawImage, imageCoordinates + (int2) (o.x, o.y)).x - blackLevel), 0.0, 1.0));
    }
}

kernel void interpolateGreen(read_only image2d_t rawImage, write_only image2d_t greenImage, int bayerPattern, float lumaSigma) {
    const int2 imageCoordinates = (int2) (get_global_id(0), get_global_id(1));

    const int x = imageCoordinates.x;
    const int y = imageCoordinates.y;

    const int2 g = bayerOffsets[bayerPattern][raw_green];
    int x0 = (y & 1) == (g.y & 1) ? g.x + 1 : g.x;

    if ((x0 & 1) == (x & 1)) {
        float g_left  = read_imagef(rawImage, (int2)(x - 1, y)).x;
        float g_right = read_imagef(rawImage, (int2)(x + 1, y)).x;
        float g_up    = read_imagef(rawImage, (int2)(x, y - 1)).x;
        float g_down  = read_imagef(rawImage, (int2)(x, y + 1)).x;

        float c_xy    = read_imagef(rawImage, (int2)(x, y)).x;

        float c_left  = read_imagef(rawImage, (int2)(x - 2, y)).x;
        float c_right = read_imagef(rawImage, (int2)(x + 2, y)).x;
        float c_up    = read_imagef(rawImage, (int2)(x, y - 2)).x;
        float c_down  = read_imagef(rawImage, (int2)(x, y + 2)).x;

        float c2_top_left       = read_imagef(rawImage, (int2)(x - 1, y - 1)).x;
        float c2_top_right      = read_imagef(rawImage, (int2)(x + 1, y - 1)).x;
        float c2_bottom_left    = read_imagef(rawImage, (int2)(x - 1, y + 1)).x;
        float c2_bottom_right   = read_imagef(rawImage, (int2)(x + 1, y + 1)).x;
        float c2_ave = (c2_top_left + c2_top_right + c2_bottom_left + c2_bottom_right) / 4;

        float g_ave = (g_left + g_right + g_up + g_down) / 4;

        float2 dv = (float2) (fabs(g_left - g_right), fabs(g_up - g_down));
        dv += (float2) (fabs(c_left + c_right - 2 * c_xy) / 2, fabs(c_up + c_down - 2 * c_xy) / 2);

        // If the gradient estimation at this location is too weak, try it on a 3x3 patch
        if (length(dv) < 0.1) {
            dv = 0;
            for (int j = -1; j <= 1; j++) {
                for (int i = -1; i <= 1; i++) {
                    float v_left  = read_imagef(rawImage, (int2)(x+i - 1, j+y)).x;
                    float v_right = read_imagef(rawImage, (int2)(x+i + 1, j+y)).x;
                    float v_up    = read_imagef(rawImage, (int2)(x+i, j+y - 1)).x;
                    float v_down  = read_imagef(rawImage, (int2)(x+i, j+y + 1)).x;

                    dv += (float2) (fabs(v_left - v_right), fabs(v_up - v_down));
                }
            }
        }

        // we're doing edge directed bilinear interpolation on the green channel,
        // which is a low pass operation (averaging), so we add some signal from the
        // high frequencies of the observed color channel

        // Estimate the whiteness of the pixel value and use that to weight the amount of HF correction
        float cMax = fmax(c_xy, fmax(g_ave, c2_ave));
        float cMin = fmin(c_xy, fmin(g_ave, c2_ave));
        float whiteness = smoothstep(0.25, 0.35, cMin/cMax);

        float sample_h = (g_left + g_right) / 2 + whiteness * (c_xy - (c_left + c_right) / 2) / 4;
        float sample_v = (g_up + g_down) / 2 + whiteness * (c_xy - (c_up + c_down) / 2) / 4;
        float sample_flat = g_ave + whiteness * (c_xy - (c_left + c_right + c_up + c_down) / 4) / 4;

        // Estimate the flatness of the image using the raw noise model
        float eps = lumaSigma * sqrt(g_ave);
        float flatness = 1 - smoothstep(eps / 2.0, eps, fabs(dv.x - dv.y));
        float sample = mix(dv.x > dv.y ? sample_v : sample_h, sample_flat, flatness);

        write_imagef(greenImage, imageCoordinates, clamp(sample, 0.0, 1.0));
    } else {
        write_imagef(greenImage, imageCoordinates, read_imagef(rawImage, (int2)(x, y)).x);
    }
}

kernel void interpolateRedBlue(read_only image2d_t rawImage, read_only image2d_t greenImage,
                               write_only image2d_t rgbImage, int bayerPattern, float chromaSigma,
                               int rotate_180) {
    const int2 imageCoordinates = (int2) (get_global_id(0), get_global_id(1));

    const int x = imageCoordinates.x;
    const int y = imageCoordinates.y;

    const int2 r = bayerOffsets[bayerPattern][raw_red];
    const int2 g = bayerOffsets[bayerPattern][raw_green];
    const int2 b = bayerOffsets[bayerPattern][raw_blue];

    int color = (r.x & 1) == (x & 1) && (r.y & 1) == (y & 1) ? raw_red :
                (g.x & 1) == (x & 1) && (g.y & 1) == (y & 1) ? raw_green :
                (b.x & 1) == (x & 1) && (b.y & 1) == (y & 1) ? raw_blue : raw_green2;

    float green = read_imagef(greenImage, imageCoordinates).x;
    float red;
    float blue;
    switch (color) {
        case raw_red:
        case raw_blue:
        {
            float c1 = read_imagef(rawImage, imageCoordinates).x;

            float g_top_left      = read_imagef(greenImage, (int2)(x - 1, y - 1)).x;
            float g_top_right     = read_imagef(greenImage, (int2)(x + 1, y - 1)).x;
            float g_bottom_left   = read_imagef(greenImage, (int2)(x - 1, y + 1)).x;
            float g_bottom_right  = read_imagef(greenImage, (int2)(x + 1, y + 1)).x;

            float c2_top_left     = g_top_left     - read_imagef(rawImage, (int2)(x - 1, y - 1)).x;
            float c2_top_right    = g_top_right    - read_imagef(rawImage, (int2)(x + 1, y - 1)).x;
            float c2_bottom_left  = g_bottom_left  - read_imagef(rawImage, (int2)(x - 1, y + 1)).x;
            float c2_bottom_right = g_bottom_right - read_imagef(rawImage, (int2)(x + 1, y + 1)).x;

            // Estimate the flatness of the image using the raw noise model
            float eps = chromaSigma * sqrt(c1);
            float2 dc = (float2) (fabs(c2_top_left - c2_bottom_right), fabs(c2_top_right - c2_bottom_left));
            float alpha = length(dc) > eps ? atan2(dc.y, dc.x) / M_PI_2_F : 0.5;
            float c2 = green - mix((c2_top_right + c2_bottom_left) / 2,
                                   (c2_top_left + c2_bottom_right) / 2, alpha);

            if (color == raw_red) {
                red = c1;
                blue = c2;
            } else {
                blue = c1;
                red = c2;
            }
        }
        break;

        case raw_green:
        case raw_green2:
        {
            float g_left    = read_imagef(greenImage, (int2)(x - 1, y)).x;
            float g_right   = read_imagef(greenImage, (int2)(x + 1, y)).x;
            float g_up      = read_imagef(greenImage, (int2)(x, y - 1)).x;
            float g_down    = read_imagef(greenImage, (int2)(x, y + 1)).x;

            float c1_left   = g_left  - read_imagef(rawImage, (int2)(x - 1, y)).x;
            float c1_right  = g_right - read_imagef(rawImage, (int2)(x + 1, y)).x;
            float c2_up     = g_up    - read_imagef(rawImage, (int2)(x, y - 1)).x;
            float c2_down   = g_down  - read_imagef(rawImage, (int2)(x, y + 1)).x;

            float c1 = green - (c1_left + c1_right) / 2;
            float c2 = green - (c2_up + c2_down) / 2;

            if (color == raw_green2) {
                red = c1;
                blue = c2;
            } else {
                blue = c1;
                red = c2;
            }
        }
        break;
    }

    int2 outputCoordinates = imageCoordinates;
    if (rotate_180) {
        outputCoordinates = get_image_dim(rgbImage) - outputCoordinates;
    }

    write_imagef(rgbImage, outputCoordinates, (float4)(clamp((float3)(red, green, blue), 0.0, 1.0), 0));
}

kernel void fastDebayer(read_only image2d_t rawImage, write_only image2d_t rgbImage, int bayerPattern) {
    const int2 imageCoordinates = (int2) (get_global_id(0), get_global_id(1));

    const int2 r = bayerOffsets[bayerPattern][raw_red];
    const int2 g = bayerOffsets[bayerPattern][raw_green];
    const int2 b = bayerOffsets[bayerPattern][raw_blue];
    const int2 g2 = bayerOffsets[bayerPattern][raw_green2];

    float red    = read_imagef(rawImage, 2 * imageCoordinates + r).x;
    float green  = read_imagef(rawImage, 2 * imageCoordinates + g).x;
    float blue   = read_imagef(rawImage, 2 * imageCoordinates + b).x;
    float green2 = read_imagef(rawImage, 2 * imageCoordinates + g2).x;

    write_imagef(rgbImage, imageCoordinates, (float4)(red, (green + green2) / 2, blue, 0.0));
}

/// ---- Median Filter ----

void median_load_data_3x3(float v[9], image2d_t inputImage, int2 imageCoordinates) {
    for(int x = -1; x <= 1; x++) {
        for(int y = -1; y <= 1; y++) {
            v[(x + 1) * 3 + (y + 1)] = read_imagef(inputImage, imageCoordinates + (int2)(x, y)).x;
        }
    }
}

#define s2(a, b)                temp = a; a = min(a, b); b = max(temp, b);
#define mn3(a, b, c)            s2(a, b); s2(a, c);
#define mx3(a, b, c)            s2(b, c); s2(a, c);

#define mnmx3(a, b, c)          mx3(a, b, c); s2(a, b);                                     // 3 exchanges
#define mnmx4(a, b, c, d)       s2(a, b); s2(c, d); s2(a, c); s2(b, d);                     // 4 exchanges
#define mnmx5(a, b, c, d, e)    s2(a, b); s2(c, d); mn3(a, c, e); mx3(b, d, e);             // 6 exchanges
#define mnmx6(a, b, c, d, e, f) s2(a, d); s2(b, e); s2(c, f); mn3(a, b, c); mx3(d, e, f);   // 7 exchanges

void median_sort_data_3x3(float v[9]) {
    float temp;

    // Starting with a subset of size 6, remove the min and max each time
    mnmx6(v[0], v[1], v[2], v[3], v[4], v[5]);
    mnmx5(v[1], v[2], v[3], v[4], v[6]);
    mnmx4(v[2], v[3], v[4], v[7]);
    mnmx3(v[3], v[4], v[8]);
}

#undef s2
#undef mn3
#undef mx3

#undef mnmx3
#undef mnmx4
#undef mnmx5
#undef mnmx6

float median_filter_3x3(image2d_t inputImage, int2 imageCoordinates) {
    float v[9];
    median_load_data_3x3(v, inputImage, imageCoordinates);
    median_sort_data_3x3(v);
    return v[4];
}

float despeckle_3x3(image2d_t inputImage, int2 imageCoordinates) {
    float sample = 0, firstMax = 0, secMax = 0;
    float firstMin = (float) 0xffff, secMin = (float) 0xffff;

    for (int x = -1; x <= 1; x++) {
        for (int y = -1; y <= 1; y++) {
            float v = read_imagef(inputImage, imageCoordinates + (int2)(x, y)).x;

            if (v > firstMax) {
                secMax = firstMax;
                firstMax = v;
            } else if (v > secMax) {
                secMax = v;
            }

            if (v < firstMin) {
                secMin = firstMin;
                firstMin = v;
            } else if (v < secMin) {
                secMin = v;
            }

            if (x == 0 && y == 0) {
                sample = v;
            }
        }
    }

    return clamp(sample, secMin, secMax);
}

kernel void despeckleYCbCrImage(read_only image2d_t inputImage, write_only image2d_t denoisedImage) {
    const int2 imageCoordinates = (int2) (get_global_id(0), get_global_id(1));

    float denoisedLuma = despeckle_3x3(inputImage, imageCoordinates);

    float3 pixel = read_imagef(inputImage, imageCoordinates).xyz;
    write_imagef(denoisedImage, imageCoordinates, (float4) (denoisedLuma, pixel.yz, 0.0));
}

kernel void despeckleLumaImage(read_only image2d_t inputImage, write_only image2d_t denoisedImage) {
    const int2 imageCoordinates = (int2) (get_global_id(0), get_global_id(1));

    float denoisedLuma = despeckle_3x3(inputImage, imageCoordinates);

    write_imagef(denoisedImage, imageCoordinates, denoisedLuma);
}

kernel void medianFilterImage(read_only image2d_t inputImage, write_only image2d_t denoisedImage) {
    const int2 imageCoordinates = (int2) (get_global_id(0), get_global_id(1));

    float denoisedLuma = median_filter_3x3(inputImage, imageCoordinates);

    float3 pixel = read_imagef(inputImage, imageCoordinates).xyz;
    write_imagef(denoisedImage, imageCoordinates, (float4) (denoisedLuma, pixel.yz, 0.0));
}

/// ---- Image Denoising ----

kernel void transformImage(read_only image2d_t inputImage, write_only image2d_t outputImage, constant float3 matrix[3]) {
    const int2 imageCoordinates = (int2) (get_global_id(0), get_global_id(1));
    float3 inputValue = read_imagef(inputImage, imageCoordinates).xyz;
    float3 outputPixel = (float3) (dot(matrix[0], inputValue), dot(matrix[1], inputValue), dot(matrix[2], inputValue));
    write_imagef(outputImage, imageCoordinates, (float4) (outputPixel, 0.0));
}

typedef struct DenoiseParameters {
    const float lumaSigma;
    const float cbSigma;
    const float crSigma;
    const float sharpening;
} DenoiseParameters;

float3 denoiseLumaChromaTight(constant DenoiseParameters* parameters, image2d_t inputImage, int2 imageCoordinates) {
    const float3 inputYCC = read_imagef(inputImage, imageCoordinates).xyz;

    float3 sigma = (float3) (parameters->lumaSigma * sqrt(inputYCC.x), parameters->cbSigma, parameters->crSigma);

    float3 filtered_pixel = 0;
    float3 kernel_norm = 0;
    for (int y = -2; y <= 2; y++) {
        for (int x = -2; x <= 2; x++) {
            float3 inputSampleYCC = read_imagef(inputImage, imageCoordinates + (int2)(x, y)).xyz;

            float3 inputDiff = fabs(inputSampleYCC - inputYCC);
            float3 sampleWeight = 1 - step(sigma, length(inputDiff));

            filtered_pixel += sampleWeight * inputSampleYCC;
            kernel_norm += sampleWeight;
        }
    }
    return filtered_pixel / kernel_norm;
}

float3 denoiseLumaChromaLoose(constant DenoiseParameters* parameters, image2d_t inputImage, int2 imageCoordinates) {
    const float3 inputYCC = read_imagef(inputImage, imageCoordinates).xyz;

    float3 sigma = (float3) (parameters->lumaSigma * sqrt(inputYCC.x), parameters->cbSigma, parameters->crSigma);

    float3 filtered_pixel = 0;
    float3 kernel_norm = 0;
    for (int y = -2; y <= 2; y++) {
        for (int x = -2; x <= 2; x++) {
            float3 inputSampleYCC = read_imagef(inputImage, imageCoordinates + (int2)(x, y)).xyz;

            float3 inputDiff = fabs(inputSampleYCC - inputYCC);
            float3 sampleWeight = 1 - step(sigma, inputDiff);

            filtered_pixel += sampleWeight * inputSampleYCC;
            kernel_norm += sampleWeight;
        }
    }
    return filtered_pixel / kernel_norm;
}

kernel void denoiseImageTight(read_only image2d_t inputImage, constant DenoiseParameters* parameters, write_only image2d_t denoisedImage) {
    const int2 imageCoordinates = (int2) (get_global_id(0), get_global_id(1));

    float3 denoisedPixel = denoiseLumaChromaTight(parameters, inputImage, imageCoordinates);

    write_imagef(denoisedImage, imageCoordinates, (float4) (denoisedPixel, 0.0));
}

kernel void denoiseImageLoose(read_only image2d_t inputImage, constant DenoiseParameters* parameters, write_only image2d_t denoisedImage) {
    const int2 imageCoordinates = (int2) (get_global_id(0), get_global_id(1));

    float3 denoisedPixel = denoiseLumaChromaLoose(parameters, inputImage, imageCoordinates);

    write_imagef(denoisedImage, imageCoordinates, (float4) (denoisedPixel, 0.0));
}

kernel void downsampleImage(read_only image2d_t inputImage, write_only image2d_t outputImage, sampler_t linear_sampler) {
    const int2 output_pos = (int2) (get_global_id(0), get_global_id(1));
    const float2 input_norm = 1.0 / convert_float2(get_image_dim(outputImage));
    const float2 input_pos = (convert_float2(output_pos) + 0.5) * input_norm;
    const float2 s = 0.4 * input_norm;

    float3 outputPixel = read_imagef(inputImage, linear_sampler, input_pos + (float2)(-s.x, -s.y)).xyz;
    outputPixel +=       read_imagef(inputImage, linear_sampler, input_pos + (float2)( s.x, -s.y)).xyz;
    outputPixel +=       read_imagef(inputImage, linear_sampler, input_pos + (float2)(-s.x,  s.y)).xyz;
    outputPixel +=       read_imagef(inputImage, linear_sampler, input_pos + (float2)( s.x,  s.y)).xyz;
    write_imagef(outputImage, output_pos, (float4) (outputPixel / 4, 0.0));
}

kernel void reassembleImage(read_only image2d_t inputImageDenoised0, read_only image2d_t inputImage1,
                            read_only image2d_t inputImageDenoised1, float sharpening, float lumaSigma,
                            write_only image2d_t outputImage, sampler_t linear_sampler) {
    const int2 output_pos = (int2) (get_global_id(0), get_global_id(1));
    const float2 inputNorm = 1.0 / convert_float2(get_image_dim(outputImage));
    const float2 input_pos = (convert_float2(output_pos) + 0.5) * inputNorm;

    float3 inputPixelDenoised0 = read_imagef(inputImageDenoised0, output_pos).xyz;
    float3 inputPixel1 = read_imagef(inputImage1, linear_sampler, input_pos).xyz;
    float3 inputPixelDenoised1 = read_imagef(inputImageDenoised1, linear_sampler, input_pos).xyz;

    float dx = read_imagef(inputImageDenoised0, output_pos + (int2)(1, 0)).x - inputPixelDenoised0.x;
    float dy = read_imagef(inputImageDenoised0, output_pos + (int2)(0, 1)).x - inputPixelDenoised0.x;
    float threshold = 4 * lumaSigma * sqrt(inputPixelDenoised0.x);
    float detail = smoothstep(0.5 * threshold, 2 * threshold, length((float2) (dx, dy)));

    float3 denoisedPixel = inputPixelDenoised0 - (inputPixel1 - inputPixelDenoised1);

    // Smart sharpening
    float amount = sharpening * detail                                                  // Gradient magnitude thresholding
                              * (1.0 - smoothstep(0.95, 1.0, denoisedPixel.x))          // Highlights ringing protection
                              * (0.6 + 0.4 * smoothstep(0.0, 0.1, denoisedPixel.x));    // Shadows ringing protection

    // Only sharpen the luma channel
    denoisedPixel.x = mix(inputPixelDenoised1.x, denoisedPixel.x, amount < 1.0f ? max(amount, sharpening) : amount);

    write_imagef(outputImage, output_pos, (float4) (denoisedPixel, 0.0));
}

/// ---- Image Sharpening ----

float3 gaussianBlur(float radius, image2d_t inputImage, int2 imageCoordinates) {
    const int kernelSize = (int) radius;
    const float sigmaS = (float) radius / 3.0;

    float3 blurred_pixel = 0;
    float3 kernel_norm = 0;
    for (int y = -kernelSize / 2; y <= kernelSize / 2; y++) {
        for (int x = -kernelSize / 2; x <= kernelSize / 2; x++) {
            float kernelWeight = native_exp(-((float)(x * x + y * y) / (2 * sigmaS * sigmaS)));
            blurred_pixel += kernelWeight * read_imagef(inputImage, imageCoordinates + (int2)(x, y)).xyz;
            kernel_norm += kernelWeight;
        }
    }
    return blurred_pixel / kernel_norm;
}

float3 sharpen(float3 pixel_value, float amount, float radius, image2d_t inputImage, int2 imageCoordinates) {
    float3 dx = read_imagef(inputImage, imageCoordinates + (int2)(1, 0)).xyz - pixel_value;
    float3 dy = read_imagef(inputImage, imageCoordinates + (int2)(0, 1)).xyz - pixel_value;

    // Smart sharpening
    float3 sharpening = amount * smoothstep(0.0, 0.03, length(dx) + length(dy))     // Gradient magnitude thresholding
                               * (1.0 - smoothstep(0.95, 1.0, pixel_value))         // Highlight ringing protection
                               * (0.6 + 0.4 * smoothstep(0.0, 0.1, pixel_value));   // Shadows ringing protection

    float3 blurred_pixel = gaussianBlur(radius, inputImage, imageCoordinates);

    return mix(blurred_pixel, pixel_value, fmax(sharpening, 1.0));
}

/// ---- Tone Curve ----

float3 sigmoid(float3 x, float s) {
    return 0.5 * (tanh(s * x - 0.3 * s) + 1);
}

// This tone curve is designed to mostly match the default curve from DNG files
// TODO: it would be nice to have separate control on highlights and shhadows contrast

float3 toneCurve(float3 x, float s) {
    return (sigmoid(native_powr(0.95 * x, 0.5), s) - sigmoid(0, s)) / (sigmoid(1, s) - sigmoid(0, s));
}

float3 saturationBoost(float3 value, float saturation) {
    // Saturation boost with highlight protection
    const float luma = 0.2126 * value.x + 0.7152 * value.y + 0.0722 * value.z; // BT.709-2 (sRGB) luma primaries
    const float3 clipping = smoothstep(0.75, 2.0, value);
    return mix(luma, value, mix(saturation, 1.0, clipping));
}

float3 contrastBoost(float3 value, float contrast) {
    const float gray = 0.2;
    const float3 clipping = smoothstep(0.9, 2.0, value);
    return mix(gray, value, mix(contrast, 1.0, clipping));
}

// Make sure this struct is in sync with the declaration in demosaic.hpp
typedef struct DemosaicParameters {
    float contrast;
    float saturation;
    float toneCurveSlope;
    float sharpening;
    float sharpeningRadius;
} DemosaicParameters;

kernel void convertTosRGB(read_only image2d_t linearImage, write_only image2d_t rgbImage,
                          constant float3 transform[3], constant DemosaicParameters *demosaicParameters) {
    const int2 imageCoordinates = (int2) (get_global_id(0), get_global_id(1));

    float3 pixel_value = read_imagef(linearImage, imageCoordinates).xyz;

    // pixel_value = saturationBoost(pixel_value, demosaicParameters->contrast);
    pixel_value = contrastBoost(pixel_value, demosaicParameters->contrast);

    float3 rgb = (float3) (dot(transform[0], pixel_value),
                           dot(transform[1], pixel_value),
                           dot(transform[2], pixel_value));

    write_imagef(rgbImage, imageCoordinates, (float4) (toneCurve(clamp(rgb, 0.0, 1.0), demosaicParameters->toneCurveSlope), 0.0));
}

kernel void resample(read_only image2d_t inputImage, write_only image2d_t outputImage, sampler_t linear_sampler) {
    const int2 imageCoordinates = (int2) (get_global_id(0), get_global_id(1));
    const float2 inputNorm = 1.0 / convert_float2(get_image_dim(outputImage));

    float3 outputPixel = read_imagef(inputImage, linear_sampler, convert_float2(imageCoordinates) * inputNorm + 0.5 * inputNorm).xyz;
    write_imagef(outputImage, imageCoordinates, (float4) (outputPixel, 0.0));
}
