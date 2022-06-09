//
//  raw_converter.cpp
//  RawPipeline
//
//  Created by Fabio Riccardi on 5/16/22.
//

#include "raw_converter.hpp"

#include "gls_logging.h"

static const char* TAG = "RAW Converter";

#define PRINT_EXECUTION_TIME true

void RawConverter::allocateTextures(gls::OpenCLContext* glsContext, int width, int height) {
    auto clContext = glsContext->clContext();

    if (!clRawImage) {
        clRawImage = std::make_unique<gls::cl_image_2d<gls::luma_pixel_16>>(clContext, width, height);
        clScaledRawImage = std::make_unique<gls::cl_image_2d<gls::luma_pixel_float>>(clContext, width, height);
        clGreenImage = std::make_unique<gls::cl_image_2d<gls::luma_pixel_float>>(clContext, width, height);
        clLinearRGBImageA = std::make_unique<gls::cl_image_2d<gls::rgba_pixel_float>>(clContext, width, height);
        clLinearRGBImageB = std::make_unique<gls::cl_image_2d<gls::rgba_pixel_float>>(clContext, width, height);
        clsRGBImage = std::make_unique<gls::cl_image_2d<gls::rgba_pixel>>(clContext, width, height);

        // Placeholder, only allocated if LTM is used
        ltmMaskImage = std::make_unique<gls::cl_image_2d<gls::luma_pixel_float>>(clContext, 1, 1);

        pyramidalDenoise = std::make_unique<PyramidalDenoise<5>>(glsContext, width, height);
    }
}

void RawConverter::allocateLtmMaskImage(gls::OpenCLContext* glsContext, int width, int height) {
    auto clContext = glsContext->clContext();

    if (ltmMaskImage->width != width || ltmMaskImage->height != height) {
        ltmMaskImage = std::make_unique<gls::cl_image_2d<gls::luma_pixel_float>>(clContext, width, height);
    }
}

void RawConverter::allocateHighNoiseTextures(gls::OpenCLContext* glsContext, int width, int height) {
    auto clContext = glsContext->clContext();

    if (!rgbaRawImage) {
        rgbaRawImage = std::make_unique<gls::cl_image_2d<gls::rgba_pixel_float>>(clContext, width/2, height/2);
        denoisedRgbaRawImage = std::make_unique<gls::cl_image_2d<gls::rgba_pixel_float>>(clContext, width/2, height/2);
    }
}

void RawConverter::allocateFastDemosaicTextures(gls::OpenCLContext* glsContext, int width, int height) {
    auto clContext = glsContext->clContext();

    if (!clFastLinearRGBImage) {
        clRawImage = std::make_unique<gls::cl_image_2d<gls::luma_pixel_16>>(clContext, width, height);
        clScaledRawImage = std::make_unique<gls::cl_image_2d<gls::luma_pixel_float>>(clContext, width, height);
        clFastLinearRGBImage = std::make_unique<gls::cl_image_2d<gls::rgba_pixel_float>>(clContext, width/2, height/2);
        clsFastRGBImage = std::make_unique<gls::cl_image_2d<gls::rgba_pixel>>(clContext, width/2, height/2);

        // Placeholder, not used in Fast Demosaic
        ltmMaskImage = std::make_unique<gls::cl_image_2d<gls::luma_pixel_float>>(clContext, 1, 1);
    }
}

gls::cl_image_2d<gls::rgba_pixel>* RawConverter::demosaicImage(const gls::image<gls::luma_pixel_16>& rawImage,
                                                               DemosaicParameters* demosaicParameters,
                                                               const gls::rectangle* gmb_position, bool rotate_180) {
    auto clContext = _glsContext->clContext();

    LOG_INFO(TAG) << "Begin Demosaicing..." << std::endl;

    NoiseModel* noiseModel = &demosaicParameters->noiseModel;

    const bool high_noise_image = true; // demosaicParameters->noiseLevel > 0.6;

    LOG_INFO(TAG) << "NoiseLevel: " << demosaicParameters->noiseLevel << std::endl;

    allocateTextures(_glsContext, rawImage.width, rawImage.height);

    if (demosaicParameters->rgbConversionParameters.localToneMapping) {
        allocateLtmMaskImage(_glsContext, rawImage.width, rawImage.height);
    }

    if (high_noise_image) {
        allocateHighNoiseTextures(_glsContext, rawImage.width, rawImage.height);
    }

#ifdef PRINT_EXECUTION_TIME
    auto t_start = std::chrono::high_resolution_clock::now();
#endif
    // Copy input data to the OpenCL input buffer
    clRawImage->copyPixelsFrom(rawImage);

    // --- Image Demosaicing ---

//    if (high_noise_image) {
//        std::cout << "denoiseRawRGBAImage" << std::endl;
//
//        bayerToRawRGBA(_glsContext, *clRawImage, rgbaRawImage.get(), demosaicParameters->bayerPattern);
//
//        despeckleRawRGBAImage(_glsContext,
//                              *rgbaRawImage,
//                              denoisedRgbaRawImage.get());
//
//        rawRGBAToBayer(_glsContext, *denoisedRgbaRawImage, clRawImage.get(), demosaicParameters->bayerPattern);
//    }

    scaleRawData(_glsContext, *clRawImage, clScaledRawImage.get(), demosaicParameters->bayerPattern, demosaicParameters->scale_mul,
                 demosaicParameters->black_level / 0xffff);

    interpolateGreen(_glsContext, *clScaledRawImage, clGreenImage.get(), demosaicParameters->bayerPattern, sqrt(noiseModel->rawNlf[1]));

    interpolateRedBlue(_glsContext, *clScaledRawImage, *clGreenImage, clLinearRGBImageA.get(), demosaicParameters->bayerPattern,
                       sqrt((noiseModel->rawNlf[0] + noiseModel->rawNlf[2]) / 2), rotate_180);

    // --- Image Denoising ---

    // Convert linear image to YCbCr
    auto cam_to_ycbcr = cam_ycbcr(demosaicParameters->rgb_cam);

    std::cout << "cam_to_ycbcr: " << cam_to_ycbcr.span() << std::endl;

    transformImage(_glsContext, *clLinearRGBImageA, clLinearRGBImageA.get(), cam_to_ycbcr);

    // False Color Removal
    // TODO: Make this an optional stage
    applyKernel(_glsContext, "falseColorsRemovalImage", *clLinearRGBImageA, clLinearRGBImageB.get());

    if (high_noise_image) {
        std::cout << "despeckleImage" << std::endl;
        const auto& np = noiseModel->pyramidNlf[0];
        despeckleImage(_glsContext, *clLinearRGBImageB, { np[0], np[1], np[2] }, { np[3], np[4], np[5] }, clLinearRGBImageA.get());
    }

    auto clDenoisedImage = pyramidalDenoise->denoise(_glsContext, &(demosaicParameters->denoiseParameters),
                                                     high_noise_image ? clLinearRGBImageA.get() : clLinearRGBImageB.get(),
                                                     demosaicParameters->rgb_cam, gmb_position, false,
                                                     &(noiseModel->pyramidNlf));

//    if (high_noise_image) {
//        gaussianBlurImage(_glsContext, *clDenoisedImage, 0.5, clLinearRGBImageA.get());
//        clDenoisedImage = clLinearRGBImageA.get();
//    }

    std::cout << "pyramidNlf:\n" << std::scientific << noiseModel->pyramidNlf << std::endl;

    if (demosaicParameters->rgbConversionParameters.localToneMapping) {
        localToneMappingMask(_glsContext, *clDenoisedImage, *(pyramidalDenoise->imagePyramid[2]), demosaicParameters->ltmParameters,
                             inverse(cam_to_ycbcr) * demosaicParameters->exposure_multiplier, ltmMaskImage.get());
    }

    // Convert result back to camera RGB
    transformImage(_glsContext, *clDenoisedImage, clDenoisedImage, inverse(cam_to_ycbcr) * demosaicParameters->exposure_multiplier);

    // --- Image Post Processing ---

    convertTosRGB(_glsContext, *clDenoisedImage, *ltmMaskImage, clsRGBImage.get(), *demosaicParameters);

#ifdef PRINT_EXECUTION_TIME
    cl::CommandQueue queue = cl::CommandQueue::getDefault();
    queue.finish();
    auto t_end = std::chrono::high_resolution_clock::now();
    double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();

    LOG_INFO(TAG) << "OpenCL Pipeline Execution Time: " << (int) elapsed_time_ms << "ms for image of size: " << rawImage.width << " x " << rawImage.height << std::endl;
#endif

    return clsRGBImage.get();
}

gls::cl_image_2d<gls::rgba_pixel>* RawConverter::fastDemosaicImage(const gls::image<gls::luma_pixel_16>& rawImage,
                                                                   const DemosaicParameters& demosaicParameters) {
    allocateFastDemosaicTextures(_glsContext, rawImage.width, rawImage.height);

    LOG_INFO(TAG) << "Begin Fast Demosaicing (GPU)..." << std::endl;

    // Copy input data to the OpenCL input buffer
    clRawImage->copyPixelsFrom(rawImage);

    // --- Image Demosaicing ---

    scaleRawData(_glsContext, *clRawImage, clScaledRawImage.get(), demosaicParameters.bayerPattern, demosaicParameters.scale_mul,
                 demosaicParameters.black_level / 0xffff);

    fasteDebayer(_glsContext, *clScaledRawImage, clFastLinearRGBImage.get(), demosaicParameters.bayerPattern);

    // --- Image Post Processing ---

    convertTosRGB(_glsContext, *clFastLinearRGBImage, *ltmMaskImage, clsFastRGBImage.get(), demosaicParameters);

    return clsFastRGBImage.get();
}

/*static*/ gls::image<gls::rgb_pixel>::unique_ptr RawConverter::convertToRGBImage(const gls::cl_image_2d<gls::rgba_pixel>& clRGBAImage) {
    auto rgbImage = std::make_unique<gls::image<gls::rgb_pixel>>(clRGBAImage.width, clRGBAImage.height);
    auto rgbaImage = clRGBAImage.mapImage();
    for (int y = 0; y < clRGBAImage.height; y++) {
        for (int x = 0; x < clRGBAImage.width; x++) {
            const auto& p = rgbaImage[y][x];
            (*rgbImage)[y][x] = { p.red, p.green, p.blue };
        }
    }
    clRGBAImage.unmapImage(rgbaImage);
    return rgbImage;
}

/*static*/ gls::image<gls::rgba_pixel>::unique_ptr RawConverter::convertToRGBAImage(const gls::cl_image_2d<gls::rgba_pixel>& clRGBAImage) {
    auto rgbaImage = std::make_unique<gls::image<gls::rgba_pixel>>(clRGBAImage.width, clRGBAImage.height);
    clRGBAImage.copyPixelsTo(&(*rgbaImage));

    return rgbaImage;
}
