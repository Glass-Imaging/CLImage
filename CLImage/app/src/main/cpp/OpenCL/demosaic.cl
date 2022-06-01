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
    { {0, 0}, {0, 1}, {1, 1}, {1, 0} }, // rggb
    { {1, 1}, {0, 1}, {0, 0}, {1, 0} }  // bggr
};

#if defined(__QCOMM_QGPU_A3X__) || \
    defined(__QCOMM_QGPU_A4X__) || \
    defined(__QCOMM_QGPU_A5X__) || \
    defined(__QCOMM_QGPU_A6X__) || \
    defined(__QCOMM_QGPU_A7V__) || \
    defined(__QCOMM_QGPU_A7P__)

// Qualcomm's smoothstep implementation can be really slow...

#define smoothstep(edge0, edge1, x) \
   ({ typedef __typeof__ (x) type_of_x; \
      type_of_x _edge0 = (edge0); \
      type_of_x _edge1 = (edge1); \
      type_of_x _x = (x); \
      type_of_x t = clamp((_x - _edge0) / (_edge1 - _edge0), 0.0f, 1.0f); \
      t * t * (3.0f - 2.0f * t); })

#endif

// Work on one Quad (2x2) at a time
kernel void scaleRawData(read_only image2d_t rawImage, write_only image2d_t scaledRawImage,
                         int bayerPattern, float4 vScaleMul, float blackLevel) {
    float *scaleMul = (float *) &vScaleMul;
    const int2 imageCoordinates = (int2) (2 * get_global_id(0), 2 * get_global_id(1));
    for (int c = 0; c < 4; c++) {
        int2 o = bayerOffsets[bayerPattern][c];
        write_imagef(scaledRawImage, imageCoordinates + (int2) (o.x, o.y),
                     clamp(scaleMul[c] * (read_imagef(rawImage, imageCoordinates + (int2) (o.x, o.y)).x - blackLevel), 0.0, 1.0));
    }
}

constant float2 sobelXY[3][3] = {
    { {-1,  1 },  { 0,  2 }, { 1,  1 } },
    { {-2,  0 } , { 0,  0 }, { 2,  0 } },
    { {-1, -1 },  { 0, -2 }, { 1, -1 } },
};

//constant float2 sobelXY[3][3] = {
//    { {-1,  1 },  { 0,  1 }, { 1,  1 } },
//    { {-1,  0 } , { 0,  0 }, { 1,  0 } },
//    { {-1, -1 },  { 0, -1 }, { 1, -1 } },
//};

float2 gradientDirection(read_only image2d_t inputImage, int2 imageCoordinates) {
    float2 sobel = 0;
    for (int y = 0; y < 3; y++) {
        for (int x = 0; x < 3; x++) {
            float sample = read_imagef(inputImage, imageCoordinates + (int2)(x - 1, y - 1)).x;
            sobel += sobelXY[y][x] * sample;
        }
    }
    return sobel;
}

kernel void interpolateGreen(read_only image2d_t rawImage, write_only image2d_t greenImage, int bayerPattern, float lumaVariance) {
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

        // Average gradient on a 3x3 patch
        float2 dv = 0;
        for (int j = -1; j <= 1; j++) {
            for (int i = -1; i <= 1; i++) {
                float v_left  = read_imagef(rawImage, (int2)(x+i - 1, j+y)).x;
                float v_right = read_imagef(rawImage, (int2)(x+i + 1, j+y)).x;
                float v_up    = read_imagef(rawImage, (int2)(x+i, j+y - 1)).x;
                float v_down  = read_imagef(rawImage, (int2)(x+i, j+y + 1)).x;

                dv += (float2) (fabs(v_left - v_right), fabs(v_up - v_down));
            }
        }
        dv /= 9;

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
        float eps = sqrt(lumaVariance * g_ave);
        float flatness = 1 - smoothstep(0.5 * eps, 2 * eps, 16 * fabs(dv.x - dv.y));
        float sample = mix(dv.x > dv.y ? sample_v : sample_h, sample_flat, flatness);

        write_imagef(greenImage, imageCoordinates, clamp(sample, 0.0, 1.0));
    } else {
        write_imagef(greenImage, imageCoordinates, read_imagef(rawImage, (int2)(x, y)).x);
    }
}

kernel void interpolateRedBlue(read_only image2d_t rawImage, read_only image2d_t greenImage,
                               write_only image2d_t rgbImage, int bayerPattern, float chromaVariance,
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
            float eps = sqrt(chromaVariance * green);
            float2 dv = (float2) (fabs(c2_top_left - c2_bottom_right), fabs(c2_top_right - c2_bottom_left));
            float flatness = 1 - smoothstep(0.25 * eps, eps, length(dv));
            float alpha = mix(2 * atan2pi(dv.y, dv.x), 0.5, flatness);
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

void median_load_data_3x3_chroma(float2 v[9], image2d_t inputImage, int2 imageCoordinates) {
    for(int x = -1; x <= 1; x++) {
        for(int y = -1; y <= 1; y++) {
            v[(x + 1) * 3 + (y + 1)] = read_imagef(inputImage, imageCoordinates + (int2)(x, y)).yz;
        }
    }
}

void median_load_data_3x3x3(float3 v[9], image2d_t inputImage, int2 imageCoordinates) {
    for(int x = -1; x <= 1; x++) {
        for(int y = -1; y <= 1; y++) {
            v[(x + 1) * 3 + (y + 1)] = read_imagef(inputImage, imageCoordinates + (int2)(x, y)).xyz;
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

void median_sort_data_3x3x2(float2 v[9]) {
    float2 temp;

    // Starting with a subset of size 6, remove the min and max each time
    mnmx6(v[0], v[1], v[2], v[3], v[4], v[5]);
    mnmx5(v[1], v[2], v[3], v[4], v[6]);
    mnmx4(v[2], v[3], v[4], v[7]);
    mnmx3(v[3], v[4], v[8]);
}

void median_sort_data_3x3x3(float3 v[9]) {
    float3 temp;

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

float3 median_filter_3x3(image2d_t inputImage, int2 imageCoordinates) {
    float3 v[9];
    median_load_data_3x3x3(v, inputImage, imageCoordinates);
    median_sort_data_3x3x3(v);
    return v[4];
}

float2 median_filter_3x3_chroma(image2d_t inputImage, int2 imageCoordinates) {
    float2 v[9];
    median_load_data_3x3_chroma(v, inputImage, imageCoordinates);
    median_sort_data_3x3x2(v);
    return v[4];
}

float despeckle_3x3(image2d_t inputImage, int2 imageCoordinates) {
    float sample = 0, firstMax = 0, secMax = 0;
    float firstMin = (float) 0xffff, secMin = (float) 0xffff;

    for (int x = -1; x <= 1; x++) {
        for (int y = -1; y <= 1; y++) {
            float v = read_imagef(inputImage, imageCoordinates + (int2)(x, y)).x;

            secMax = v <= firstMax && v > secMax ? v : secMax;
            secMax = v > firstMax ? firstMax : secMax;
            firstMax = v > firstMax ? v : firstMax;

            secMin = v >= firstMin && v < secMin ? v : secMin;
            secMin = v < firstMin ? firstMin : secMin;
            firstMin = v < firstMin ? v : firstMin;

            if (x == 0 && y == 0) {
                sample = v;
            }
        }
    }

    return clamp(sample, secMin, secMax);
}

float4 despeckle_3x3x4(image2d_t inputImage, int2 imageCoordinates) {
    float4 sample = 0, firstMax = 0, secMax = 0;
    float4 firstMin = (float) 0xffff, secMin = (float) 0xffff;

    for (int x = -1; x <= 1; x++) {
        for (int y = -1; y <= 1; y++) {
            float4 v = read_imagef(inputImage, imageCoordinates + (int2)(x, y));

            secMax = v <= firstMax && v > secMax ? v : secMax;
            secMax = v > firstMax ? firstMax : secMax;
            firstMax = v > firstMax ? v : firstMax;

            secMin = v >= firstMin && v < secMin ? v : secMin;
            secMin = v < firstMin ? firstMin : secMin;
            firstMin = v < firstMin ? v : firstMin;

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
    // float2 denoisedChroma = median_filter_3x3_chroma(inputImage, imageCoordinates);

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

    float2 denoisedPixel = median_filter_3x3_chroma(inputImage, imageCoordinates);

    float luma = read_imagef(inputImage, imageCoordinates).x;
    write_imagef(denoisedImage, imageCoordinates, (float4) (luma, denoisedPixel, 0.0));
}

/*
 * 5 x 5 Fast Median Filter Implementation for Chroma Antialiasing
 */

float3 antialias_load_data_5x5(float2 v[25], image2d_t inputImage, int2 imageCoordinates) {
    float3 max = -100;
    float3 min =  100;
    for(int x = -2; x <= 2; x++) {
        for(int y = -2; y <= 2; y++) {
            float3 sample = read_imagef(inputImage, imageCoordinates + (int2)(x, y)).xyz;
            max = sample > max ? sample : max;
            min = sample < min ? sample : min;
            v[(x + 2) * 5 + (y + 2)] = sample.yz;
        }
    }
    return max - min;
}

#define s2(a, b)                            temp = a; a = min(a, b); b = max(temp, b);
#define t2(a, b)                            s2(v[a], v[b]);
#define t24(a, b, c, d, e, f, g, h)         t2(a, b); t2(c, d); t2(e, f); t2(g, h);
#define t25(a, b, c, d, e, f, g, h, i, j)   t24(a, b, c, d, e, f, g, h); t2(i, j);

void median_sort_data_5x5(float2 v[25]) {
    float2 temp;

    t25(0,  1,        3,  4,       2,  4,         2, 3,          6,  7);
    t25(5,  7,        5,  6,       9,  7,         1, 7,          1,  4);
    t25(12, 13,       11, 13,      11, 12,        15, 16,        14, 16);
    t25(14, 15,       18, 19,      17, 19,        17, 18,        21, 22);
    t25(20, 22,       20, 21,      23, 24,        2, 5,          3,  6);
    t25(0,  6,        0,  3,       4,  7,         1, 7,          1,  4);
    t25(11, 14,       8,  14,      8,  11,        12, 15,        9, 15);
    t25(9,  12,       13, 16,      10, 16,        10, 13,        20, 23);
    t25(17, 23,       17, 20,      21, 24,        18, 24,        18, 21);
    t25(19, 22,       8,  17,      9,  18,        0, 18,         0,  9);
    t25(10, 19,       1,  19,      1,  10,        11, 20,        2,  20);
    t25(2,  11,       12, 21,      3,  21,        3, 12,         13, 22);
    t25(4,  22,       4,  13,      14, 23,        5, 23,         5,  14);
    t25(15, 24,       6,  24,      6,  15,        7, 16,         7,  19);
    t25(3,  11,       5,  17,      11, 17,        9, 17,         4,  10);
    t25(6,  12,       7,  14,      4,  6,         4, 7,         12,  14);
    t25(10, 14,       6,  7,       10, 12,        6, 10,         6,  17);
    t25(12, 17,       7,  17,      7,  10,        12, 18,        7,  12);
    t24(10, 18,       12, 20,      10, 20,        10, 12);
}

#undef t2
#undef t24
#undef t25
#undef s2

float3 antiAliasFilter5x5(image2d_t inputImage, int2 imageCoordinates) {
    float2 v[25];
    float3 D = antialias_load_data_5x5(v, inputImage, imageCoordinates);
    median_sort_data_5x5(v);
    float2 medianC = v[12];

    float cf = D.x < D.y && D.x < D.x ? D.x : max(D.x, max(D.y, D.z));
    cf = exp(-312.5 * cf * cf);

    float3 value = read_imagef(inputImage, imageCoordinates).xyz;
    float2 chroma = medianC + cf * (value.yz - medianC);
    return (float3) (value.x, chroma);
}

kernel void antiAliasImage5x5(read_only image2d_t inputImage, write_only image2d_t denoisedImage) {
    const int2 imageCoordinates = (int2) (get_global_id(0), get_global_id(1));

    float3 denoisedPixel = antiAliasFilter5x5(inputImage, imageCoordinates);

    write_imagef(denoisedImage, imageCoordinates, (float4) (denoisedPixel, 0.0));
}

/// ---- Image Denoising ----

typedef struct {
    float3 m[3];
} Matrix3x3;

kernel void transformImage(read_only image2d_t inputImage, write_only image2d_t outputImage, Matrix3x3 transform) {
    const int2 imageCoordinates = (int2) (get_global_id(0), get_global_id(1));
    float3 inputValue = read_imagef(inputImage, imageCoordinates).xyz;
    float3 outputPixel = (float3) (dot(transform.m[0], inputValue), dot(transform.m[1], inputValue), dot(transform.m[2], inputValue));
    write_imagef(outputImage, imageCoordinates, (float4) (outputPixel, 0.0));
}

float3 denoiseLumaChromaTight(float3 sigma_a, float3 sigma_b, image2d_t inputImage, int2 imageCoordinates) {
    const float3 inputYCC = read_imagef(inputImage, imageCoordinates).xyz;

    float3 sigma = sqrt(sigma_a + sigma_b * inputYCC.x);

    // TODO: make this a calibration parameter
//    // Decrease denoising on edges
//    float dx = (read_imagef(inputImage, imageCoordinates + (int2)(1, 0)).x -
//                read_imagef(inputImage, imageCoordinates - (int2)(1, 0)).x) / 2;
//    float dy = (read_imagef(inputImage, imageCoordinates + (int2)(0, 1)).x -
//                read_imagef(inputImage, imageCoordinates - (int2)(0, 1)).x) / 2;
//    float sigmaBoost = smoothstep(0.5 * sigma.x, 2 * sigma.x, length((float2) (dx, dy)));
//    sigma *= 1 - 0.5 * sigmaBoost;

    float3 filtered_pixel = 0;
    float3 kernel_norm = 0;
    for (int y = -2; y <= 2; y++) {
        for (int x = -2; x <= 2; x++) {
            float3 inputSampleYCC = read_imagef(inputImage, imageCoordinates + (int2)(x, y)).xyz;

            float3 inputDiff = inputSampleYCC - inputYCC;
            float3 sampleWeight = 1 - step(sigma, length(inputDiff));

            filtered_pixel += sampleWeight * inputSampleYCC;
            kernel_norm += sampleWeight;
        }
    }
    return filtered_pixel / kernel_norm;
}

kernel void denoiseImageTight(read_only image2d_t inputImage, float3 sigma_a, float3 sigma_b, write_only image2d_t denoisedImage) {
    const int2 imageCoordinates = (int2) (get_global_id(0), get_global_id(1));

    float3 denoisedPixel = denoiseLumaChromaTight(sigma_a, sigma_b, inputImage, imageCoordinates);

    write_imagef(denoisedImage, imageCoordinates, (float4) (denoisedPixel, 0.0));
}

float3 denoiseLumaChromaLoose(float3 sigma_a, float3 sigma_b, image2d_t inputImage, int2 imageCoordinates) {
    const float3 inputYCC = read_imagef(inputImage, imageCoordinates).xyz;

    float3 sigma = sqrt(sigma_a + sigma_b * inputYCC.x);

    // TODO: make this a calibration parameter
//    // Decrease denoising on edges
//    float dx = (read_imagef(inputImage, imageCoordinates + (int2)(1, 0)).x -
//                read_imagef(inputImage, imageCoordinates - (int2)(1, 0)).x) / 2;
//    float dy = (read_imagef(inputImage, imageCoordinates + (int2)(0, 1)).x -
//                read_imagef(inputImage, imageCoordinates - (int2)(0, 1)).x) / 2;
//    float sigmaBoost = smoothstep(0.5 * sigma.x, 2 * sigma.x, length((float2) (dx, dy)));
//    sigma *= 1 - 0.5 * sigmaBoost;

    float3 filtered_pixel = 0;
    float3 kernel_norm = 0;
    for (int y = -2; y <= 2; y++) {
        for (int x = -2; x <= 2; x++) {
            float3 inputSampleYCC = read_imagef(inputImage, imageCoordinates + (int2)(x, y)).xyz;

            float3 inputDiff = inputSampleYCC - inputYCC;
            float3 sampleWeight = 1 - step(sigma, fabs(inputDiff));

            filtered_pixel += sampleWeight * inputSampleYCC;
            kernel_norm += sampleWeight;
        }
    }
    return filtered_pixel / kernel_norm;
}

kernel void denoiseImageLoose(read_only image2d_t inputImage, float3 sigma_a, float3 sigma_b, write_only image2d_t denoisedImage) {
    const int2 imageCoordinates = (int2) (get_global_id(0), get_global_id(1));

    float3 denoisedPixel = denoiseLumaChromaLoose(sigma_a, sigma_b, inputImage, imageCoordinates);

    write_imagef(denoisedImage, imageCoordinates, (float4) (denoisedPixel, 0.0));
}

float3 denoiseLumaChromaGuided(float3 var_a, float3 var_b, image2d_t inputImage, int2 imageCoordinates) {
    const float3 input = read_imagef(inputImage, imageCoordinates).xyz;

    float3 eps = var_a + var_b * input.x;

    const int radius = 2;
    const float norm = 1.0 / ((2 * radius + 1) * (2 * radius + 1));

    float3 mean_I = 0;
    float mean_I_rr = 0;
    float mean_I_rg = 0;
    float mean_I_rb = 0;
    float mean_I_gg = 0;
    float mean_I_gb = 0;
    float mean_I_bb = 0;
    for (int y = -radius; y <= radius; y++) {
        for (int x = -radius; x <= radius; x++) {
            float3 sample = read_imagef(inputImage, imageCoordinates + (int2)(x, y)).xyz;
            mean_I += sample;
            mean_I_rr += sample.x * sample.x;
            mean_I_rg += sample.x * sample.y;
            mean_I_rb += sample.x * sample.z;
            mean_I_gg += sample.y * sample.y;
            mean_I_gb += sample.y * sample.z;
            mean_I_bb += sample.z * sample.z;
        }
    }
    mean_I *= norm;
    mean_I_rr *= norm;
    mean_I_rg *= norm;
    mean_I_rb *= norm;
    mean_I_gg *= norm;
    mean_I_gb *= norm;
    mean_I_bb *= norm;

    float var_I_rr = mean_I_rr - mean_I.x * mean_I.x;
    float var_I_rg = mean_I_rg - mean_I.x * mean_I.y;
    float var_I_rb = mean_I_rb - mean_I.x * mean_I.z;
    float var_I_gg = mean_I_gg - mean_I.y * mean_I.y;
    float var_I_gb = mean_I_gb - mean_I.y * mean_I.z;
    float var_I_bb = mean_I_bb - mean_I.z * mean_I.z;

    float var_I_rr_eps = var_I_rr + eps.x;
    float var_I_gg_eps = var_I_gg + eps.y;
    float var_I_bb_eps = var_I_bb + eps.z;

    float invrr = var_I_gg_eps * var_I_bb_eps - var_I_gb     * var_I_gb;
    float invrg = var_I_gb     * var_I_rb     - var_I_rg     * var_I_bb_eps;
    float invrb = var_I_rg     * var_I_gb     - var_I_gg_eps * var_I_rb;
    float invgg = var_I_rr_eps * var_I_bb_eps - var_I_rb     * var_I_rb;
    float invgb = var_I_rb     * var_I_rg     - var_I_rr_eps * var_I_gb;
    float invbb = var_I_rr_eps * var_I_gg_eps - var_I_rg     * var_I_rg;

    float invCovDet = 1 / (invrr * var_I_rr_eps + invrg * var_I_rg + invrb * var_I_rb);

    invrr *= invCovDet;
    invrg *= invCovDet;
    invrb *= invCovDet;
    invgg *= invCovDet;
    invgb *= invCovDet;
    invbb *= invCovDet;

    // Compute the result

    // covariance of (I, p) in each local patch.
    float3 cov_Ip_r = (float3) (var_I_rr, var_I_rg, var_I_rb);
    float3 cov_Ip_g = (float3) (var_I_rg, var_I_gg, var_I_gb);
    float3 cov_Ip_b = (float3) (var_I_rb, var_I_gb, var_I_bb);

    float3 a_r = invrr * cov_Ip_r + invrg * cov_Ip_g + invrb * cov_Ip_b;
    float3 a_g = invrg * cov_Ip_r + invgg * cov_Ip_g + invgb * cov_Ip_b;
    float3 a_b = invrb * cov_Ip_r + invgb * cov_Ip_g + invbb * cov_Ip_b;

    float3 b = mean_I - a_r * mean_I.x - a_g * mean_I.y - a_b * mean_I.z; // Eqn. (15) in the paper;

    return a_r * input.x + a_g * input.y + a_b * input.z + b;
}

kernel void denoiseImageGuided(read_only image2d_t inputImage, float3 var_a, float3 var_b, write_only image2d_t denoisedImage) {
    const int2 imageCoordinates = (int2) (get_global_id(0), get_global_id(1));

    float3 denoisedPixel = denoiseLumaChromaGuided(var_a, var_b, inputImage, imageCoordinates);

    write_imagef(denoisedImage, imageCoordinates, (float4) (denoisedPixel, 0.0));
}

kernel void downsampleImage(read_only image2d_t inputImage, write_only image2d_t outputImage, sampler_t linear_sampler) {
    const int2 output_pos = (int2) (get_global_id(0), get_global_id(1));
    const float2 input_norm = 1.0 / convert_float2(get_image_dim(outputImage));
    const float2 input_pos = (convert_float2(output_pos) + 0.5) * input_norm;
    const float2 s = 0.5 * input_norm;

    float3 outputPixel = read_imagef(inputImage, linear_sampler, input_pos + (float2)(-s.x, -s.y)).xyz;
    outputPixel +=       read_imagef(inputImage, linear_sampler, input_pos + (float2)( s.x, -s.y)).xyz;
    outputPixel +=       read_imagef(inputImage, linear_sampler, input_pos + (float2)(-s.x,  s.y)).xyz;
    outputPixel +=       read_imagef(inputImage, linear_sampler, input_pos + (float2)( s.x,  s.y)).xyz;
    write_imagef(outputImage, output_pos, (float4) (outputPixel / 4, 0.0));
}

kernel void reassembleImage(read_only image2d_t inputImageDenoised0, read_only image2d_t inputImage1,
                            read_only image2d_t inputImageDenoised1, float sharpening, float2 nlf,
                            write_only image2d_t outputImage, sampler_t linear_sampler) {
    const int2 output_pos = (int2) (get_global_id(0), get_global_id(1));
    const float2 inputNorm = 1.0 / convert_float2(get_image_dim(outputImage));
    const float2 input_pos = (convert_float2(output_pos) + 0.5) * inputNorm;

    float3 inputPixelDenoised0 = read_imagef(inputImageDenoised0, output_pos).xyz;
    float3 inputPixel1 = read_imagef(inputImage1, linear_sampler, input_pos).xyz;
    float3 inputPixelDenoised1 = read_imagef(inputImageDenoised1, linear_sampler, input_pos).xyz;

    float3 denoisedPixel = inputPixelDenoised0 - (inputPixel1 - inputPixelDenoised1);

    if (sharpening > 1.0) {
        float dx = (read_imagef(inputImageDenoised1, linear_sampler, input_pos + (float2)(1, 0) * inputNorm).x -
                    read_imagef(inputImageDenoised1, linear_sampler, input_pos - (float2)(1, 0) * inputNorm).x) / 2;
        float dy = (read_imagef(inputImageDenoised1, linear_sampler, input_pos + (float2)(0, 1) * inputNorm).x -
                    read_imagef(inputImageDenoised1, linear_sampler, input_pos - (float2)(0, 1) * inputNorm).x) / 2;

        float threshold = 0.25 * sqrt(nlf.x + nlf.y * inputPixelDenoised1.x);
        float detail = smoothstep(0.25 * threshold, threshold, length((float2) (dx, dy)))
                       * (1.0 - smoothstep(0.95, 1.0, denoisedPixel.x))          // Highlights ringing protection
                       * (0.6 + 0.4 * smoothstep(0.0, 0.1, denoisedPixel.x));    // Shadows ringing protection
        sharpening = 1 + (sharpening - 1) * detail;
    }

    denoisedPixel.x = mix(inputPixelDenoised1.x, denoisedPixel.x, sharpening);

    write_imagef(outputImage, output_pos, (float4) (denoisedPixel, 0.0));
}

kernel void bayerToRawRGBA(read_only image2d_t rawImage, write_only image2d_t rgbaImage, int bayerPattern) {
    const int2 imageCoordinates = (int2) (get_global_id(0), get_global_id(1));

    const int2 r = bayerOffsets[bayerPattern][raw_red];
    const int2 g = bayerOffsets[bayerPattern][raw_green];
    const int2 b = bayerOffsets[bayerPattern][raw_blue];
    const int2 g2 = bayerOffsets[bayerPattern][raw_green2];

    float red    = read_imagef(rawImage, 2 * imageCoordinates + r).x;
    float green  = read_imagef(rawImage, 2 * imageCoordinates + g).x;
    float blue   = read_imagef(rawImage, 2 * imageCoordinates + b).x;
    float green2 = read_imagef(rawImage, 2 * imageCoordinates + g2).x;

    write_imagef(rgbaImage, imageCoordinates, (float4)(red, green, blue, green2));
}

kernel void rawRGBAToBayer(read_only image2d_t rgbaImage, write_only image2d_t rawImage, int bayerPattern) {
    const int2 imageCoordinates = (int2) (get_global_id(0), get_global_id(1));

    float4 rgba = read_imagef(rgbaImage, imageCoordinates);

    const int2 r = bayerOffsets[bayerPattern][raw_red];
    const int2 g = bayerOffsets[bayerPattern][raw_green];
    const int2 b = bayerOffsets[bayerPattern][raw_blue];
    const int2 g2 = bayerOffsets[bayerPattern][raw_green2];

    write_imagef(rawImage, 2 * imageCoordinates + r, rgba.x);
    write_imagef(rawImage, 2 * imageCoordinates + g, rgba.y);
    write_imagef(rawImage, 2 * imageCoordinates + b, rgba.z);
    write_imagef(rawImage, 2 * imageCoordinates + g2, rgba.w);
}

float4 denoiseRawRGBA(float4 rawVariance, image2d_t inputImage, int2 imageCoordinates) {
    const float4 input = read_imagef(inputImage, imageCoordinates);

    float4 sigma = 0.5 * sqrt(rawVariance * input);

    float4 filtered_pixel = 0;
    float4 kernel_norm = 0;
    for (int y = -2; y <= 2; y++) {
        for (int x = -2; x <= 2; x++) {
            float4 inputSample = read_imagef(inputImage, imageCoordinates + (int2)(x, y));

            float4 inputDiff = fabs(inputSample - input);
            float4 sampleWeight = 1 - step(sigma, inputDiff);

            filtered_pixel += sampleWeight * inputSample;
            kernel_norm += sampleWeight;
        }
    }
    return filtered_pixel / kernel_norm;
}

float4 denoiseRawRGBAGuided(float4 rawVariance, image2d_t inputImage, int2 imageCoordinates) {
    const float4 input = read_imagef(inputImage, imageCoordinates);

    float4 noiseVar = 0.1 * rawVariance * input;

    int radius = 5;
    int count = (2 * radius + 1) * (2 * radius + 1);

    float4 sum = 0;
    float4 sumSq = 0;
    for (int y = -radius; y <= radius; y++) {
        for (int x = -radius; x <= radius; x++) {
            float4 inputSample = read_imagef(inputImage, imageCoordinates + (int2)(x, y));
            sum += inputSample;
            sumSq += inputSample * inputSample;
        }
    }
    float4 mean = sum / count;
    float4 var = (sumSq - (sum * sum) / count) / count;

    float4 filtered_pixel = 0;
    float4 kernel_norm = 0;
    for (int y = -radius; y <= radius; y++) {
        for (int x = -radius; x <= radius; x++) {
            float4 inputSample = read_imagef(inputImage, imageCoordinates + (int2)(x, y));

            float4 sampleWeight = 1 + (inputSample - mean) * (input - mean) / (var + noiseVar);

            filtered_pixel += sampleWeight * inputSample;
            kernel_norm += sampleWeight;
        }
    }
    return filtered_pixel / kernel_norm;
}

float4 denoiseRawRGBAGuidedCov(float4 eps, image2d_t inputImage, int2 imageCoordinates) {
    const float4 input = read_imagef(inputImage, imageCoordinates);

    const int radius = 2;
    const float norm = 1.0 / ((2 * radius + 1) * (2 * radius + 1));

    float3 mean_I = 0;
    float mean_I_rr = 0;
    float mean_I_rg = 0;
    float mean_I_rb = 0;
    float mean_I_gg = 0;
    float mean_I_gb = 0;
    float mean_I_bb = 0;
    for (int y = -radius; y <= radius; y++) {
        for (int x = -radius; x <= radius; x++) {
            float4 sampleRGBA = read_imagef(inputImage, imageCoordinates + (int2)(x, y));
            float3 sample = (float3) (sampleRGBA.x, (sampleRGBA.y + sampleRGBA.w) / 2, sampleRGBA.z);
            mean_I += sample;
            mean_I_rr += sample.x * sample.x;
            mean_I_rg += sample.x * sample.y;
            mean_I_rb += sample.x * sample.z;
            mean_I_gg += sample.y * sample.y;
            mean_I_gb += sample.y * sample.z;
            mean_I_bb += sample.z * sample.z;
        }
    }
    mean_I *= norm;
    mean_I_rr *= norm;
    mean_I_rg *= norm;
    mean_I_rb *= norm;
    mean_I_gg *= norm;
    mean_I_gb *= norm;
    mean_I_bb *= norm;

    float var_I_rr = mean_I_rr - mean_I.x * mean_I.x;
    float var_I_rg = mean_I_rg - mean_I.x * mean_I.y;
    float var_I_rb = mean_I_rb - mean_I.x * mean_I.z;
    float var_I_gg = mean_I_gg - mean_I.y * mean_I.y;
    float var_I_gb = mean_I_gb - mean_I.y * mean_I.z;
    float var_I_bb = mean_I_bb - mean_I.z * mean_I.z;

    float var_I_rr_eps = var_I_rr + 0.2 * eps.x * input.x;
    float var_I_gg_eps = var_I_gg + 0.2 * eps.y * (input.y + input.w) / 2;
    float var_I_bb_eps = var_I_bb + 0.2 * eps.z * input.z;

    float invrr = var_I_gg_eps * var_I_bb_eps - var_I_gb     * var_I_gb;
    float invrg = var_I_gb     * var_I_rb     - var_I_rg     * var_I_bb_eps;
    float invrb = var_I_rg     * var_I_gb     - var_I_gg_eps * var_I_rb;
    float invgg = var_I_rr_eps * var_I_bb_eps - var_I_rb     * var_I_rb;
    float invgb = var_I_rb     * var_I_rg     - var_I_rr_eps * var_I_gb;
    float invbb = var_I_rr_eps * var_I_gg_eps - var_I_rg     * var_I_rg;

    float invCovDet = 1 / (invrr * var_I_rr_eps + invrg * var_I_rg + invrb * var_I_rb);

    invrr *= invCovDet;
    invrg *= invCovDet;
    invrb *= invCovDet;
    invgg *= invCovDet;
    invgb *= invCovDet;
    invbb *= invCovDet;

    // Compute the result

    // covariance of (I, p) in each local patch.
    float3 cov_Ip_r = (float3) (var_I_rr, var_I_rg, var_I_rb);
    float3 cov_Ip_g = (float3) (var_I_rg, var_I_gg, var_I_gb);
    float3 cov_Ip_b = (float3) (var_I_rb, var_I_gb, var_I_bb);

    float3 a_r = invrr * cov_Ip_r + invrg * cov_Ip_g + invrb * cov_Ip_b;
    float3 a_g = invrg * cov_Ip_r + invgg * cov_Ip_g + invgb * cov_Ip_b;
    float3 a_b = invrb * cov_Ip_r + invgb * cov_Ip_g + invbb * cov_Ip_b;

    float3 b = mean_I - a_r * mean_I.x - a_g * mean_I.y - a_b * mean_I.z; // Eqn. (15) in the paper;

    return (float4) (a_r * input.x + a_g * input.y + a_b * input.z + b,
                     a_r.y * input.x + a_g.y * input.w + a_b.y * input.z + b.y);
}

kernel void denoiseRawRGBAImage(read_only image2d_t inputImage, float4 rawVariance, write_only image2d_t denoisedImage) {
    const int2 imageCoordinates = (int2) (get_global_id(0), get_global_id(1));

    float4 denoisedPixel = denoiseRawRGBAGuidedCov(rawVariance, inputImage, imageCoordinates);

    write_imagef(denoisedImage, imageCoordinates, denoisedPixel);
}

kernel void despeckleRawRGBAImage(read_only image2d_t inputImage, write_only image2d_t denoisedImage) {
    const int2 imageCoordinates = (int2) (get_global_id(0), get_global_id(1));

    float4 despeckledPixel = despeckle_3x3x4(inputImage, imageCoordinates);

    write_imagef(denoisedImage, imageCoordinates, despeckledPixel);
}

kernel void sobelFilterImage(read_only image2d_t inputImage, write_only image2d_t outputImage) {
    const int2 imageCoordinates = (int2) (get_global_id(0), get_global_id(1));

    constant float sobelX[3][3] = {
        { -1, -2, -1 },
        {  0,  0,  0 },
        {  1,  2,  1 },
    };
    constant float sobelY[3][3] = {
        { -1,  0,  1 },
        { -2,  0,  2 },
        { -1,  0,  1 },
    };

    float3 valueX = 0;
    float3 valueY = 0;
    for (int y = 0; y < 3; y++) {
        for (int x = 0; x < 3; x++) {
            float3 sample = read_imagef(inputImage, imageCoordinates + (int2)(x - 1, y - 1)).xyz;
            valueX += sobelX[y][x] * sample;
            valueY += sobelY[y][x] * sample;
        }
    }

    write_imagef(outputImage, imageCoordinates, (float4) (sqrt(valueX * valueX + valueY * valueY), 0));
}

kernel void desaturateEdges(read_only image2d_t inputImage, write_only image2d_t outputImage) {
    const int2 imageCoordinates = (int2) (get_global_id(0), get_global_id(1));

    float d2x = read_imagef(inputImage, imageCoordinates + (int2)(-1, 0)).x -
                2 * read_imagef(inputImage, imageCoordinates + (int2)(0, 0)).x +
                read_imagef(inputImage, imageCoordinates + (int2)( 1, 0)).x;

    float d2y = read_imagef(inputImage, imageCoordinates + (int2)(0, -1)).x -
                2 * read_imagef(inputImage, imageCoordinates + (int2)(0, 0)).x +
                read_imagef(inputImage, imageCoordinates + (int2)(0,  1)).x;

    float3 pixel = read_imagef(inputImage, imageCoordinates).xyz;
    float desaturate = 1 - smoothstep(0.25, 0.5, 50 * (d2x * d2x + d2y * d2y));

    write_imagef(outputImage, imageCoordinates, (float4) (pixel.x, desaturate * pixel.yz, 0));
}

kernel void laplacianFilterImage(read_only image2d_t inputImage, write_only image2d_t outputImage) {
    const int2 imageCoordinates = (int2) (get_global_id(0), get_global_id(1));

    constant float laplacian[3][3] = {
        {  1, -2,  1 },
        { -2,  4, -2 },
        {  1, -2,  1 },
    };

    float3 value = 0;
    for (int y = 0; y < 3; y++) {
        for (int x = 0; x < 3; x++) {
            float3 sample = read_imagef(inputImage, imageCoordinates + (int2)(x - 1, y - 1)).xyz;
            value += laplacian[y][x] * sample;
        }
    }

    write_imagef(outputImage, imageCoordinates, (float4) (value, 0));
}

kernel void noiseStatistics(read_only image2d_t inputImage, write_only image2d_t outputImage) {
    const int2 imageCoordinates = (int2) (get_global_id(0), get_global_id(1));

    int radius = 5;
    int count = (2 * radius + 1) * (2 * radius + 1);

    float3 sum = 0;
    float3 sumSq = 0;
    for (int y = -radius; y <= radius; y++) {
        for (int x = -radius; x <= radius; x++) {
            float3 inputSample = read_imagef(inputImage, imageCoordinates + (int2)(x, y)).xyz;
            sum += inputSample;
            sumSq += inputSample * inputSample;
        }
    }
    float3 mean = sum / count;
    float3 var = (sumSq - (sum * sum) / count) / count;

    write_imagef(outputImage, imageCoordinates, (float4) (mean.x, var));
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

float3 desaturateBlacks(float3 value) {
    // Saturation boost with highlight protection
    const float luma = 0.2126 * value.x + 0.7152 * value.y + 0.0722 * value.z; // BT.709-2 (sRGB) luma primaries
    const float desaturate = smoothstep(0.005, 0.04, luma);
    return mix(luma, value, desaturate);
}

float3 contrastBoost(float3 value, float contrast) {
    const float gray = 0.2;
    const float3 clipping = smoothstep(0.9, 2.0, value);
    return mix(gray, value, mix(contrast, 1.0, clipping));
}

// Make sure this struct is in sync with the declaration in demosaic.hpp
typedef struct RGBConversionParameters {
    float contrast;
    float saturation;
    float toneCurveSlope;
} RGBConversionParameters;

kernel void convertTosRGB(read_only image2d_t linearImage, write_only image2d_t rgbImage,
                          Matrix3x3 transform, RGBConversionParameters rgbConversionParameters) {
    const int2 imageCoordinates = (int2) (get_global_id(0), get_global_id(1));

    float3 pixel_value = read_imagef(linearImage, imageCoordinates).xyz;

    // pixel_value = saturationBoost(pixel_value, rgbConversionParameters.saturation);
    // pixel_value = desaturateBlacks(pixel_value);
    pixel_value = contrastBoost(pixel_value, rgbConversionParameters.contrast);

    float3 rgb = (float3) (dot(transform.m[0], pixel_value),
                           dot(transform.m[1], pixel_value),
                           dot(transform.m[2], pixel_value));

    write_imagef(rgbImage, imageCoordinates, (float4) (clamp(toneCurve(rgb, rgbConversionParameters.toneCurveSlope), 0.0, 1.0), 0.0));
}

kernel void resample(read_only image2d_t inputImage, write_only image2d_t outputImage, sampler_t linear_sampler) {
    const int2 imageCoordinates = (int2) (get_global_id(0), get_global_id(1));
    const float2 inputNorm = 1.0 / convert_float2(get_image_dim(outputImage));

    float3 outputPixel = read_imagef(inputImage, linear_sampler, convert_float2(imageCoordinates) * inputNorm + 0.5 * inputNorm).xyz;
    write_imagef(outputImage, imageCoordinates, (float4) (outputPixel, 0.0));
}
