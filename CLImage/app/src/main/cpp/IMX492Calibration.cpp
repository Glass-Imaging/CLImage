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

#include "CameraCalibration.hpp"

#include "demosaic.hpp"
#include "raw_converter.hpp"

#include <array>
#include <cmath>
#include <filesystem>

static const std::array<NoiseModel, 6> NLF_IMX492 = {{
    {
        { 5.8995e-05, 5.4218e-05, 5.1745e-05, 5.1355e-05 },
        {
            4.6078e-05, 3.3181e-06, 3.0636e-06, 0, 0, 0,
            1.3286e-05, 2.5259e-06, 2.4235e-06, 0, 0, 0,
            5.0580e-06, 2.0399e-06, 2.0061e-06, 0, 0, 0,
            2.2516e-06, 1.8550e-06, 1.8422e-06, 0, 0, 0,
            2.4016e-06, 1.7470e-06, 1.7424e-06, 0, 0, 0,
        },
    },
    {
        { 1.0815e-04, 9.9500e-05, 9.7302e-05, 1.0157e-04 },
        {
            8.7183e-05, 8.2809e-06, 7.6248e-06, 0, 0, 0,
            2.5632e-05, 6.0580e-06, 5.8181e-06, 0, 0, 0,
            9.9401e-06, 4.6615e-06, 4.6154e-06, 0, 0, 0,
            4.5783e-06, 4.1732e-06, 4.1538e-06, 0, 0, 0,
            3.9608e-06, 3.9518e-06, 3.9546e-06, 0, 0, 0,
        },
    },
    {
        { 2.0683e-04, 1.9365e-04, 1.9477e-04, 1.9835e-04 },
        {
            1.6720e-04, 8.2837e-06, 7.2979e-06, 0, 0, 0,
            4.6571e-05, 5.2380e-06, 4.8545e-06, 0, 0, 0,
            1.5780e-05, 3.2460e-06, 3.1590e-06, 0, 0, 0,
            6.0399e-06, 2.5417e-06, 2.5231e-06, 0, 0, 0,
            3.9350e-06, 2.2943e-06, 2.2953e-06, 0, 0, 0,
        },
    },
    {
        { 4.2513e-04, 3.7694e-04, 4.0732e-04, 3.8399e-04 },
        {
            3.1399e-04, 1.4670e-05, 1.3769e-05, 0, 0, 0,
            7.4048e-05, 8.6166e-06, 8.4927e-06, 0, 0, 0,
            2.1717e-05, 4.8974e-06, 4.9674e-06, 0, 0, 0,
            5.9704e-06, 3.6236e-06, 3.6382e-06, 0, 0, 0,
            2.4477e-06, 3.1328e-06, 3.1489e-06, 0, 0, 0,
        },
    },
    {
        { 8.7268e-04, 7.4568e-04, 8.0404e-04, 7.9171e-04 },
        {
            6.3230e-04, 4.4344e-05, 4.1359e-05, 0, 0, 0,
            1.4667e-04, 2.4977e-05, 2.4449e-05, 0, 0, 0,
            3.9296e-05, 1.2878e-05, 1.3054e-05, 0, 0, 0,
            1.1223e-05, 8.5536e-06, 8.6911e-06, 0, 0, 0,
            5.0937e-06, 7.2092e-06, 7.2716e-06, 0, 0, 0,
        },
    },
    {
        { 2.5013e-03, 1.7729e-03, 2.2981e-03, 1.7412e-03 },
        {
            8.2186e-04, 6.6875e-05, 6.9377e-05, 0, 0, 0,
            2.2212e-04, 3.9264e-05, 4.2434e-05, 0, 0, 0,
            7.0234e-05, 2.1307e-05, 2.3023e-05, 0, 0, 0,
            2.1599e-05, 1.4505e-05, 1.4757e-05, 0, 0, 0,
            7.8263e-06, 1.2044e-05, 1.1925e-05, 0, 0, 0,
        },
    },
}};

template <int levels>
std::pair<gls::Vector<4>, gls::Matrix<levels, 6>> nlfFromIso(const std::array<NoiseModel, 6>& NLFData, int iso) {
    iso = std::clamp(iso, 100, 3200);
    if (iso >= 100 && iso < 200) {
        float a = (iso - 100) / 100;
        return std::pair(lerpRawNLF(NLFData[0].rawNlf, NLFData[1].rawNlf, a), lerpNLF<levels>(NLFData[0].pyramidNlf, NLFData[1].pyramidNlf, a));
    } else if (iso >= 200 && iso < 400) {
        float a = (iso - 200) / 200;
        return std::pair(lerpRawNLF(NLFData[1].rawNlf, NLFData[2].rawNlf, a), lerpNLF<levels>(NLFData[1].pyramidNlf, NLFData[2].pyramidNlf, a));
    } else if (iso >= 400 && iso < 800) {
        float a = (iso - 400) / 400;
        return std::pair(lerpRawNLF(NLFData[2].rawNlf, NLFData[3].rawNlf, a), lerpNLF<levels>(NLFData[2].pyramidNlf, NLFData[3].pyramidNlf, a));
    } else if (iso >= 800 && iso < 1600) {
        float a = (iso - 800) / 800;
        return std::pair(lerpRawNLF(NLFData[3].rawNlf, NLFData[4].rawNlf, a), lerpNLF<levels>(NLFData[3].pyramidNlf, NLFData[4].pyramidNlf, a));
    } else /* if (iso >= 1600 && iso <= 3200) */ {
        float a = (iso - 1600) / 1600;
        return std::pair(lerpRawNLF(NLFData[4].rawNlf, NLFData[5].rawNlf, a), lerpNLF<levels>(NLFData[4].pyramidNlf, NLFData[5].pyramidNlf, a));
    }
}

std::pair<float, std::array<DenoiseParameters, 5>> IMX492DenoiseParameters(int iso) {
    const auto nlf_params = nlfFromIso<5>(NLF_IMX492, iso);

    // A reasonable denoising calibration on a fairly large range of Noise Variance values
    const float min_green_variance = NLF_IMX492[0].rawNlf[1];
    const float max_green_variance = NLF_IMX492[NLF_IMX492.size()-1].rawNlf[1];
    const float nlf_green_variance = std::clamp(nlf_params.first[1], min_green_variance, max_green_variance);
    const float nlf_alpha = log2(nlf_green_variance / min_green_variance) / log2(max_green_variance / min_green_variance);

    std::cout << "IMX492DenoiseParameters nlf_alpha: " << nlf_alpha << " for ISO " << iso << std::endl;

    std::array<DenoiseParameters, 5> denoiseParameters = {{
        {
            .luma = 0.125f * std::lerp(1.0f, 2.0f, nlf_alpha),
            .chroma = std::lerp(1.0f, 8.0f, nlf_alpha),
            .sharpening = std::lerp(1.5f, 1.0f, nlf_alpha)
        },
        {
            .luma = 0.25f * std::lerp(1.0f, 2.0f, nlf_alpha),
            .chroma = std::lerp(1.0f, 8.0f, nlf_alpha),
            .sharpening = 1.1 // std::lerp(1.0f, 0.8f, nlf_alpha),
        },
        {
            .luma = 0.5f * std::lerp(1.0f, 2.0f, nlf_alpha),
            .chroma = std::lerp(1.0f, 4.0f, nlf_alpha),
            .sharpening = 1
        },
        {
            .luma = 0.25f * std::lerp(1.0f, 2.0f, nlf_alpha),
            .chroma = std::lerp(1.0f, 4.0f, nlf_alpha),
            .sharpening = 1
        },
        {
            .luma = 0.125f * std::lerp(1.0f, 2.0f, nlf_alpha),
            .chroma = std::lerp(1.0f, 8.0f, nlf_alpha),
            .sharpening = 1
        }
    }};

    return { nlf_alpha, denoiseParameters };
}

gls::image<gls::rgb_pixel>::unique_ptr calibrateIMX492DNG(RawConverter* rawConverter, const std::filesystem::path& input_path,
                                                          DemosaicParameters* demosaicParameters, int iso,
                                                          const gls::rectangle& rotated_gmb_position, bool rotate_180) {
    gls::tiff_metadata dng_metadata, exif_metadata;
    dng_metadata.insert({ TIFFTAG_COLORMATRIX1, std::vector<float>{ 1, 0, 0, 0, 1, 0, 0, 0, 1 } });
    dng_metadata.insert({ TIFFTAG_ASSHOTNEUTRAL, std::vector<float>{ 1, 1, 1 } });
    dng_metadata.insert({ TIFFTAG_CFAREPEATPATTERNDIM, std::vector<uint16_t>{ 2, 2 } });
    dng_metadata.insert({ TIFFTAG_CFAPATTERN, std::vector<uint8_t>{ 1, 2, 0, 1 } });
    dng_metadata.insert({ TIFFTAG_BLACKLEVEL, std::vector<float>{ 0 } });
    dng_metadata.insert({ TIFFTAG_WHITELEVEL, std::vector<uint32_t>{ 0xfff } });
    dng_metadata.insert({ TIFFTAG_MAKE, "Glass Imaging" });
    dng_metadata.insert({ TIFFTAG_UNIQUECAMERAMODEL, "Glass 1" });

    const auto inputImage = gls::image<gls::luma_pixel_16>::read_dng_file(input_path.string(), &dng_metadata, &exif_metadata);

    const gls::rectangle gmb_position = rotate180(rotated_gmb_position, *inputImage);

    unpackDNGMetadata(*inputImage, &dng_metadata, demosaicParameters, /*auto_white_balance=*/ false, &gmb_position, rotate_180);

    // See if the ISO value is present and override
    const auto exifIsoSpeedRatings = getVector<uint16_t>(exif_metadata, EXIFTAG_ISOSPEEDRATINGS);
    if (exifIsoSpeedRatings.size() > 0) {
        iso = exifIsoSpeedRatings[0];
    }

    const auto denoiseParameters = IMX492DenoiseParameters(iso);
    demosaicParameters->noiseLevel = denoiseParameters.first;
    demosaicParameters->denoiseParameters = denoiseParameters.second;

    return RawConverter::convertToRGBImage(*rawConverter->demosaicImage(*inputImage, demosaicParameters, &rotated_gmb_position, rotate_180));
}

void IMX492DngMetadata(gls::tiff_metadata *dng_metadata) {
    // Leaky IR
    dng_metadata->insert({ TIFFTAG_COLORMATRIX1, std::vector<float>{ 1.4955, -0.6760, -0.1453, -0.1341, 1.0072, 0.1269, -0.0647, 0.1987, 0.4304 } });
    dng_metadata->insert({ TIFFTAG_ASSHOTNEUTRAL, std::vector<float>{ 1 / 1.73344, 1, 1 / 1.68018 } });

    // Gzikai
    // dng_metadata.insert({ TIFFTAG_COLORMATRIX1, std::vector<float>{ 1.6864, -0.7955, -0.3059, 0.0838, 1.0317, -0.1155, 0.0586, 0.0492, 0.3925 } });
    // dng_metadata.insert({ TIFFTAG_ASSHOTNEUTRAL, std::vector<float>{ 1 / 2.0777, 1.0000, 1 / 1.8516 } });

    // Kolari
    // dng_metadata.insert({ TIFFTAG_COLORMATRIX1, std::vector<float>{ 0.6963, -0.2447, -0.1353, -0.3240, 1.2994, 0.0246, -0.0190, 0.1222, 0.4874 } });
    // dng_metadata.insert({ TIFFTAG_ASSHOTNEUTRAL, std::vector<float>{ 1 / 3.7730, 1.0000, 1 / 1.6040 } });

    dng_metadata->insert({ TIFFTAG_CFAREPEATPATTERNDIM, std::vector<uint16_t>{ 2, 2 } });
    dng_metadata->insert({ TIFFTAG_CFAPATTERN, std::vector<uint8_t>{ 1, 2, 0, 1 } });
    dng_metadata->insert({ TIFFTAG_BLACKLEVEL, std::vector<float>{ 0 } });
    dng_metadata->insert({ TIFFTAG_WHITELEVEL, std::vector<uint32_t>{ 0xfff } });
    dng_metadata->insert({ TIFFTAG_MAKE, "Glass Imaging" });
    dng_metadata->insert({ TIFFTAG_UNIQUECAMERAMODEL, "Glass 1" });
    dng_metadata->insert({ TIFFTAG_ORIENTATION,(uint16_t) ORIENTATION_BOTRIGHT}); // Image is Rotated 180 degrees


    /*
    dng_metadata.insert({ TIFFTAG_COLORMATRIX1, std::vector<float>{ 0.6963, -0.2447, -0.1353, -0.3240, 1.2994, 0.0246, -0.0190, 0.1222, 0.4874 } });
    dng_metadata.insert({ TIFFTAG_ASSHOTNEUTRAL, std::vector<float>{ 1 / 3.7730, 1.0000, 1 / 1.6040 } });

    dng_metadata.insert({ TIFFTAG_CFAREPEATPATTERNDIM, std::vector<uint16_t>{ 2, 2 } });
    dng_metadata.insert({ TIFFTAG_CFAPATTERN, std::vector<uint8_t>{ 1, 2, 0, 1 } });
    dng_metadata.insert({ TIFFTAG_BLACKLEVEL, std::vector<float>{ 0 } });
    dng_metadata.insert({ TIFFTAG_WHITELEVEL, std::vector<uint32_t>{ 0xfff } });
    dng_metadata.insert({ TIFFTAG_MAKE, "Glass Imaging" });
    dng_metadata.insert({ TIFFTAG_UNIQUECAMERAMODEL, "Glass 1" });
     */
}

void IMX492ExifMetadata(gls::tiff_metadata *exif_metadata,
                        uint16_t analogGain,
                        float exposureDuration) {
    exif_metadata->insert({EXIFTAG_ISOSPEEDRATINGS, std::vector<uint16_t>{analogGain}});
    exif_metadata->insert({EXIFTAG_EXPOSURETIME, exposureDuration});
}

DemosaicParameters IMX492DemosaicParameters(gls::tiff_metadata *exif_metadata, bool useLTM) {
    DemosaicParameters demosaicParameters = {
        .rgbConversionParameters = {
            .contrast = 1.05,
            .saturation = 1.0,
            .toneCurveSlope = 3.5,
            .localToneMapping = useLTM
        }
    };

    float iso = 100;
    const auto exifIsoSpeedRatings = getVector<uint16_t>(*exif_metadata, EXIFTAG_ISOSPEEDRATINGS);
    if (exifIsoSpeedRatings.size() > 0) {
        iso = exifIsoSpeedRatings[0];
    }

    const auto nlfParams = nlfFromIso<5>(NLF_IMX492, iso);
    const auto denoiseParameters = IMX492DenoiseParameters(iso);
    demosaicParameters.noiseModel.rawNlf = nlfParams.first;
    demosaicParameters.noiseModel.pyramidNlf = nlfParams.second;
    demosaicParameters.noiseLevel = denoiseParameters.first;
    demosaicParameters.denoiseParameters = denoiseParameters.second;

    return demosaicParameters;
}

void calibrateIMX492(RawConverter* rawConverter, const std::filesystem::path& input_dir) {
    struct CalibrationEntry {
        int iso;
        const char* fileName;
        gls::rectangle gmb_position;
        bool rotated;
    };

    std::array<CalibrationEntry, 6> calibration_files = {{
        { 100, "calibration_100_33333_2022-04-21-17-52-58-947.dng", { 4537-10, 2370, 1652, 1068 }, true },
        { 200, "calibration_200_25000_2022-04-21-17-53-05-454.dng", { 4537-10, 2370, 1652, 1068 }, true },
        { 400, "calibration_400_8333_2022-04-21-17-53-12-201.dng", { 4537-10, 2370, 1652, 1068 }, true },
        { 800, "calibration_800_33333_2022-04-21-17-54-56-251.dng", { 4537-80, 2351, 1652, 1068 }, true },
        { 1600, "calibration_1600_25000_2022-04-21-17-55-04-540.dng", { 4537-80, 2351, 1652, 1068 }, true },
        { 3200, "calibration_3200_8333_2022-04-21-17-55-21-836.dng", { 4537-80, 2351, 1652, 1068 }, true },
    }};

    std::array<NoiseModel, 6> noiseModel;

    for (int i = 0; i < calibration_files.size(); i++) {
        auto& entry = calibration_files[i];
        const auto input_path = input_dir / entry.fileName;

        DemosaicParameters demosaicParameters = {
            .rgbConversionParameters = {
                .contrast = 1.05,
                .saturation = 1.0,
                .toneCurveSlope = 3.5,
            }
        };

        const auto rgb_image = calibrateIMX492DNG(rawConverter, input_path, &demosaicParameters, entry.iso, entry.gmb_position, entry.rotated);
        rgb_image->write_png_file((input_path.parent_path() / input_path.stem()).string() + "_cal_rgb.png", /*skip_alpha=*/ true);

        noiseModel[i] = demosaicParameters.noiseModel;
    }

    std::cout << "Calibration table for IMX492:" << std::endl;
    for (int i = 0; i < calibration_files.size(); i++) {
        std::cout << "// ISO " << calibration_files[i].iso << std::endl;
        std::cout << "{" << std::endl;
        std::cout << "{ " << noiseModel[i].rawNlf << " }," << std::endl;
        std::cout << "{\n" << noiseModel[i].pyramidNlf << "\n}," << std::endl;
        std::cout << "}," << std::endl;
    }
}

gls::image<gls::rgb_pixel>::unique_ptr demosaicIMX492DNG(RawConverter* rawConverter, const std::filesystem::path& input_path) {
    DemosaicParameters demosaicParameters = {
        .rgbConversionParameters = {
            .contrast = 1.05,
            .saturation = 1.0,
            .toneCurveSlope = 3.5,
        }
    };

    gls::tiff_metadata dng_metadata, exif_metadata;
    // Leaky IR
//    dng_metadata.insert({ TIFFTAG_COLORMATRIX1, std::vector<float>{ 1.4955, -0.6760, -0.1453, -0.1341, 1.0072, 0.1269, -0.0647, 0.1987, 0.4304 } });
//    dng_metadata.insert({ TIFFTAG_ASSHOTNEUTRAL, std::vector<float>{ 1 / 1.73344, 1, 1 / 1.68018 } });

    // Gzikai
//    dng_metadata.insert({ TIFFTAG_COLORMATRIX1, std::vector<float>{ 1.6864, -0.7955, -0.3059, 0.0838, 1.0317, -0.1155, 0.0586, 0.0492, 0.3925 } });
//    dng_metadata.insert({ TIFFTAG_ASSHOTNEUTRAL, std::vector<float>{ 1 / 2.0777, 1.0000, 1 / 1.8516 } });

//    // Kolari
    dng_metadata.insert({ TIFFTAG_COLORMATRIX1, std::vector<float>{ 0.6963, -0.2447, -0.1353, -0.3240, 1.2994, 0.0246, -0.0190, 0.1222, 0.4874 } });
    dng_metadata.insert({ TIFFTAG_ASSHOTNEUTRAL, std::vector<float>{ 1 / 3.7730, 1.0000, 1 / 1.6040 } });

    dng_metadata.insert({ TIFFTAG_CFAREPEATPATTERNDIM, std::vector<uint16_t>{ 2, 2 } });
    dng_metadata.insert({ TIFFTAG_CFAPATTERN, std::vector<uint8_t>{ 1, 2, 0, 1 } });
    dng_metadata.insert({ TIFFTAG_BLACKLEVEL, std::vector<float>{ 0 } });
    dng_metadata.insert({ TIFFTAG_WHITELEVEL, std::vector<uint32_t>{ 0xfff } });
    dng_metadata.insert({ TIFFTAG_MAKE, "Glass Imaging" });
    dng_metadata.insert({ TIFFTAG_UNIQUECAMERAMODEL, "Glass 1" });

    const auto inputImage = gls::image<gls::luma_pixel_16>::read_dng_file(input_path.string(), &dng_metadata, &exif_metadata);

    unpackDNGMetadata(*inputImage, &dng_metadata, &demosaicParameters, /*auto_white_balance=*/ false, /*gmb_position=*/ nullptr, /*rotate_180=*/ true);

    float iso = 100;
    const auto exifIsoSpeedRatings = getVector<uint16_t>(exif_metadata, EXIFTAG_ISOSPEEDRATINGS);
    if (exifIsoSpeedRatings.size() > 0) {
        iso = exifIsoSpeedRatings[0];
    }

    const auto nlfParams = nlfFromIso<5>(NLF_IMX492, iso);
    const auto denoiseParameters = IMX492DenoiseParameters(iso);
    demosaicParameters.noiseModel.rawNlf = nlfParams.first;
    demosaicParameters.noiseModel.pyramidNlf = nlfParams.second;
    demosaicParameters.noiseLevel = denoiseParameters.first;
    demosaicParameters.denoiseParameters = denoiseParameters.second;

    return RawConverter::convertToRGBImage(*rawConverter->demosaicImage(*inputImage, &demosaicParameters, nullptr, /*rotate_180=*/ true));
    // return RawConverter::convertToRGBImage(*rawConverter->fastDemosaicImage(*inputImage, demosaicParameters));
}
