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

#ifndef demosaic_hpp
#define demosaic_hpp

#include "gls_image.hpp"
#include "gls_tiff_metadata.hpp"
#include "gls_linalg.hpp"

enum BayerPattern {
    grbg = 0,
    gbrg = 1,
    rggb = 2,
    bggr = 3
};

extern const char* BayerPatternName[4];

typedef struct DenoiseParameters {
    float luma;
    float chroma;
    float sharpening;
} DenoiseParameters;

typedef struct RGBConversionParameters {
    float contrast = 1.05;
    float saturation = 1.0;
    float toneCurveSlope = 3.5;
    int localToneMapping = 0;
} RGBConversionParameters;

typedef struct NoiseModel {
    gls::Vector<4> rawNlf;          // NLF for raw data
    gls::Matrix<5, 6> pyramidNlf;   // NLF for interpolated data on a 5-level pyramid
} NoiseModel;

typedef struct LTMParameters {
    float guidedFilterEps = 0.01;
    float shadows = 1.25;
    float highlights = 1.0;
    float detail = 1.1;
} LTMParameters;

typedef struct DemosaicParameters {
    // Basic Debayering Parameters
    BayerPattern bayerPattern;
    float black_level;
    float white_level;
    float exposure_multiplier;
    gls::Vector<4> scale_mul;
    gls::Matrix<3, 3> rgb_cam;

    // Noise Estimation and Reduction parameters
    NoiseModel noiseModel;
    std::array<DenoiseParameters, 5> denoiseParameters;
    // In the [0..1] range, used to scale various denoising coefficients
    float noiseLevel;

    // Camera Color Space to RGB Parameters
    RGBConversionParameters rgbConversionParameters;

    // Local Tone Mapping Parameters
    LTMParameters ltmParameters;
} DemosaicParameters;

const gls::point bayerOffsets[4][4] = {
    { {1, 0}, {0, 0}, {0, 1}, {1, 1} }, // grbg
    { {0, 1}, {0, 0}, {1, 0}, {1, 1} }, // gbrg
    { {0, 0}, {1, 0}, {1, 1}, {0, 1} }, // rggb
    { {1, 1}, {1, 0}, {0, 0}, {0, 1} }  // bggr
};

// sRGB -> XYZ D65 Transform: xyz_rgb * rgb_color -> xyz_color
const gls::Matrix<3, 3> xyz_rgb = {
    { 0.4124564, 0.3575761, 0.1804375 },
    { 0.2126729, 0.7151522, 0.0721750 },
    { 0.0193339, 0.1191920, 0.9503041 }
};

// XYZ D65 -> sRGB Transform: rgb_xyz * xyx_color -> rgb_color
const gls::Matrix<3, 3> rgb_xyz = {
    {  3.2404542, -1.5371385, -0.4985314 },
    { -0.9692660,  1.8760108,  0.0415560 },
    {  0.0556434, -0.2040259,  1.0572252 }
};

inline uint16_t clamp_uint16(int x) { return x < 0 ? 0 : x > 0xffff ? 0xffff : x; }
inline uint8_t clamp_uint8(int x) { return x < 0 ? 0 : x > 0xff ? 0xff : x; }

template <typename T>
gls::rectangle rotate180(const gls::rectangle& rect, const gls::image<T>& image) {
    return {
        image.width - rect.x - rect.width,
        image.height - rect.y - rect.height,
        rect.width,
        rect.height
    };
}

inline static gls::Vector<4> lerpRawNLF(const gls::Vector<4>& NLFData0, const gls::Vector<4>& NLFData1, float a) {
    gls::Vector<4> result;
    for (int i = 0; i < 4; i++) {
        result[i] = std::lerp(NLFData0[i], NLFData1[i], a);
    }
    return result;
}

template <int levels>
gls::Matrix<levels, 6> lerpNLF(const gls::Matrix<levels, 6>& NLFData0, const gls::Matrix<levels, 6>& NLFData1, float a) {
    gls::Matrix<levels, 6> result;
    for (int j = 0; j < levels; j++) {
        for (int i = 0; i < 6; i++) {
            result[j][i] = std::lerp(NLFData0[j][i], NLFData1[j][i], a);
        }
    }
    return result;
}

void white_balance(const gls::image<gls::luma_pixel_16>& rawImage, gls::Vector<3>* wb_mul, uint32_t white, uint32_t black, BayerPattern bayerPattern);

void interpolateGreen(const gls::image<gls::luma_pixel_16>& rawImage,
                      gls::image<gls::rgb_pixel_16>* rgbImage, BayerPattern bayerPattern);

void interpolateRedBlue(gls::image<gls::rgb_pixel_16>* image, BayerPattern bayerPattern);

gls::image<gls::rgb_pixel_16>::unique_ptr demosaicImageCPU(const gls::image<gls::luma_pixel_16>& rawImage,
                                                        gls::tiff_metadata* metadata, bool auto_white_balance);

gls::image<gls::rgb_pixel>::unique_ptr demosaicImage(const gls::image<gls::luma_pixel_16>& rawImage,
                                                     DemosaicParameters* demosaicParameters,
                                                     const gls::rectangle* gmb_position, bool rotate_180);

gls::image<gls::rgba_pixel>::unique_ptr fastDemosaicImage(const gls::image<gls::luma_pixel_16>& rawImage,
                                                         const DemosaicParameters& demosaicParameters);

gls::Matrix<3, 3> cam_xyz_coeff(gls::Vector<3>* pre_mul, const gls::Matrix<3, 3>& cam_xyz);

void colorcheck(const gls::image<gls::luma_pixel_16>& rawImage, BayerPattern bayerPattern, uint32_t black, std::array<gls::rectangle, 24> gmb_samples);

void white_balance(const gls::image<gls::luma_pixel_16>& rawImage, gls::Vector<3>* wb_mul, uint32_t white, uint32_t black, BayerPattern bayerPattern);

float unpackDNGMetadata(const gls::image<gls::luma_pixel_16>& rawImage,
                        gls::tiff_metadata* dng_metadata,
                        DemosaicParameters* demosaicParameters,
                        bool auto_white_balance, const gls::rectangle* gmb_position, bool rotate_180);

gls::Matrix<3, 3> cam_ycbcr(const gls::Matrix<3, 3>& rgb_cam);

gls::Vector<3> extractNlfFromColorChecker(gls::image<gls::rgba_pixel_float>* yCbCrImage, const gls::rectangle gmb_position, bool rotate_180, int scale);

extern const gls::Matrix<3, 3> srgb_ycbcr;
extern const gls::Matrix<3, 3> ycbcr_srgb;

enum GMBColors {
    DarkSkin        = 0,
    LightSkin       = 1,
    BlueSky         = 2,
    Foliage         = 3,
    BlueFlower      = 4,
    BluishGreen     = 5,
    Orange          = 6,
    PurplishBlue    = 7,
    ModerateRed     = 8,
    Purple          = 9,
    YellowGreen     = 10,
    OrangeYellow    = 11,
    Blue            = 12,
    Green           = 13,
    Red             = 14,
    Yellow          = 15,
    Magenta         = 16,
    Cyan            = 17,
    White           = 18,
    Neutral_8       = 19,
    Neutral_6_5     = 20,
    Neutral_5       = 21,
    Neutral_3_5     = 22,
    Black           = 23
};

extern const char* GMBColorNames[24];

struct PatchStats {
    gls::Vector<3> mean;
    gls::Vector<3> variance;
};

struct RawPatchStats {
    gls::Vector<4> mean;
    gls::Vector<4> variance;
};

void colorCheckerRawStats(const gls::image<gls::luma_pixel_16>& rawImage, float black_level, float white_level, BayerPattern bayerPattern,
                          const gls::rectangle& gmb_position, bool rotate_180, std::array<RawPatchStats, 24>* stats);

gls::Vector<4> estimateRawParameters(const gls::image<gls::luma_pixel_16>& rawImage, gls::Matrix<3, 3>* cam_xyz, gls::Vector<3>* pre_mul,
                                     float black_level, float white_level, BayerPattern bayerPattern, const gls::rectangle& gmb_position, bool rotate_180);

void colorcheck(const std::array<RawPatchStats, 24>& rawStats);

gls::Vector<3> autoWhiteBalance(const gls::image<gls::luma_pixel_16>& rawImage, const gls::Matrix<3, 3>& rgb_ycbcr,
                                const gls::Vector<4>& scale_mul, float white, float black, BayerPattern bayerPattern);

void KernelOptimizeBilinear2d(int width, const std::vector<float>& weightsIn,
                              std::vector<std::tuple</* w */ float, /* x */ float, /* y */ float>>* weightsOut);
#endif /* demosaic_hpp */
