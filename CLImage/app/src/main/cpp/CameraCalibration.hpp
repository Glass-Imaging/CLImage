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

#ifndef CameraCalibration_hpp
#define CameraCalibration_hpp

#include <filesystem>

#include "demosaic.hpp"
#include "raw_converter.hpp"

void IMX492DngMetadata(gls::tiff_metadata *dng_metadata);
void IMX492ExifMetadata(gls::tiff_metadata *exif_metadata, uint16_t analogGain, float exposureDuration);
DemosaicParameters IMX492DemosaicParameters(gls::tiff_metadata *exif_metadata, bool useLTM);
gls::image<gls::rgb_pixel>::unique_ptr demosaicIMX492DNG(RawConverter* rawConverter, const std::filesystem::path& input_path);
void calibrateIMX492(RawConverter* rawConverter, const std::filesystem::path& input_dir);

void IMX571DngMetadata(gls::tiff_metadata *dng_metadata);
void IMX571ExifMetadata(gls::tiff_metadata *exif_metadata, uint16_t analogGain, float exposureDuration);
DemosaicParameters IMX571DemosaicParameters(gls::tiff_metadata *exif_metadata, bool useLTM);
gls::image<gls::rgb_pixel>::unique_ptr demosaicIMX571DNG(RawConverter* rawConverter, const std::filesystem::path& input_path);
void calibrateIMX571(RawConverter* rawConverter, const std::filesystem::path& input_dir);

gls::image<gls::rgb_pixel>::unique_ptr demosaicLeicaQ2DNG(RawConverter* rawConverter, const std::filesystem::path& input_path);
void calibrateLeicaQ2(RawConverter* rawConverter, const std::filesystem::path& input_dir);

gls::image<gls::rgb_pixel>::unique_ptr demosaicCanonEOSRPDNG(RawConverter* rawConverter, const std::filesystem::path& input_path);
void calibrateCanonEOSRP(RawConverter* rawConverter, const std::filesystem::path& input_dir);

gls::image<gls::rgb_pixel>::unique_ptr demosaicSonya6400DNG(RawConverter* rawConverter, const std::filesystem::path& input_path);
void calibrateSonya6400(RawConverter* rawConverter, const std::filesystem::path& input_dir);

void calibrateRicohGRIII(RawConverter* rawConverter, const std::filesystem::path& input_dir);
gls::image<gls::rgb_pixel>::unique_ptr demosaicRicohGRIII2DNG(RawConverter* rawConverter, const std::filesystem::path& input_path);

void calibrateiPhone11(RawConverter* rawConverter, const std::filesystem::path& input_dir);
gls::image<gls::rgb_pixel>::unique_ptr demosaiciPhone11(RawConverter* rawConverter, const std::filesystem::path& input_path);

#endif /* CameraCalibration_hpp */
