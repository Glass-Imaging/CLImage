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

#include "demosaic.hpp"
#include "demosaic_cl.hpp"

#include "raw_converter.hpp"

#include "guided_filter.hpp"

#include "gls_logging.h"

const char* BayerPatternName[4] = {
    "GRBG",
    "GBRG",
    "RGGB",
    "BGGR"
};

static const char* TAG = "CLImage Pipeline";

gls::image<gls::rgb_pixel>::unique_ptr demosaicImage(const gls::image<gls::luma_pixel_16>& rawImage,
                                                     DemosaicParameters* demosaicParameters,
                                                     const gls::rectangle* gmb_position, bool rotate_180) {
    gls::OpenCLContext glsContext("");

    auto rawConverter = std::make_unique<RawConverter>(&glsContext);

    auto t_start = std::chrono::high_resolution_clock::now();

    auto clsRGBImage = rawConverter->demosaicImage(rawImage, demosaicParameters, gmb_position, rotate_180);

    auto rgbImage = RawConverter::convertToRGBImage(*clsRGBImage);

    auto t_end = std::chrono::high_resolution_clock::now();
    double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();

    LOG_INFO(TAG) << "OpenCL Pipeline Execution Time: " << (int) elapsed_time_ms << "ms for image of size: " << rawImage.width << " x " << rawImage.height << std::endl;

    return rgbImage;
}

gls::image<gls::rgba_pixel>::unique_ptr fastDemosaicImage(const gls::image<gls::luma_pixel_16>& rawImage,
                                                          const DemosaicParameters& demosaicParameters) {
    gls::OpenCLContext glsContext("");

    auto rawConverter = std::make_unique<RawConverter>(&glsContext);

    auto t_start = std::chrono::high_resolution_clock::now();

    auto clsRGBImage = rawConverter->fastDemosaicImage(rawImage, demosaicParameters);

    auto rgbaImage = RawConverter::convertToRGBAImage(*clsRGBImage);

    auto t_end = std::chrono::high_resolution_clock::now();
    double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();

    LOG_INFO(TAG) << "OpenCL Pipeline Execution Time: " << (int) elapsed_time_ms << "ms for image of size: " << rawImage.width << " x " << rawImage.height << std::endl;

    return rgbaImage;
}
