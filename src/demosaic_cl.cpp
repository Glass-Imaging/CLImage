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

#include "gls_cl.hpp"
#include "gls_cl_image.hpp"

/*
 OpenCL RAW Image Demosaic.
 NOTE: This code can throw exceptions, to facilitate debugging no exception handler is provided, so things can crash in place.
 */

void scaleRawData(gls::OpenCLContext* glsContext,
                 const gls::cl_image_2d<gls::luma_pixel_16>& rawImage,
                 gls::cl_image_2d<gls::luma_pixel_float>* scaledRawImage,
                 BayerPattern bayerPattern, gls::Vector<4> scaleMul, float blackLevel) {
    // Load the shader source
    const auto program = glsContext->loadProgram("demosaic");

    // Bind the kernel parameters
    auto kernel = cl::KernelFunctor<cl::Image2D,  // rawImage
                                    cl::Image2D,  // scaledRawImage
                                    int,          // bayerPattern
                                    cl::Buffer,   // scaleMul
                                    float         // blackLevel
                                    >(program, "scaleRawData");

    cl::Buffer scaleMulBuffer(scaleMul.begin(), scaleMul.end(), true);

    // Work on one Quad (2x2) at a time
    kernel(gls::OpenCLContext::buildEnqueueArgs(scaledRawImage->width/2, scaledRawImage->height/2),
           rawImage.getImage2D(), scaledRawImage->getImage2D(), bayerPattern, scaleMulBuffer, blackLevel);
}

void interpolateGreen(gls::OpenCLContext* glsContext,
                     const gls::cl_image_2d<gls::luma_pixel_float>& rawImage,
                     gls::cl_image_2d<gls::luma_pixel_float>* greenImage,
                     BayerPattern bayerPattern, float lumaVariance) {
    // Load the shader source
    const auto program = glsContext->loadProgram("demosaic");

    // Bind the kernel parameters
    auto kernel = cl::KernelFunctor<cl::Image2D,  // rawImage
                                    cl::Image2D,  // greenImage
                                    int,          // bayerPattern
                                    float         // lumaVariance
                                    >(program, "interpolateGreen");

    // Schedule the kernel on the GPU
    kernel(gls::OpenCLContext::buildEnqueueArgs(greenImage->width, greenImage->height),
           rawImage.getImage2D(), greenImage->getImage2D(), bayerPattern, lumaVariance);
}

void interpolateRedBlue(gls::OpenCLContext* glsContext,
                       const gls::cl_image_2d<gls::luma_pixel_float>& rawImage,
                       const gls::cl_image_2d<gls::luma_pixel_float>& greenImage,
                       gls::cl_image_2d<gls::rgba_pixel_float>* rgbImage,
                       BayerPattern bayerPattern, float chromaVariance, bool rotate_180) {
    // Load the shader source
    const auto program = glsContext->loadProgram("demosaic");

    // Bind the kernel parameters
    auto kernel = cl::KernelFunctor<cl::Image2D,  // rawImage
                                    cl::Image2D,  // greenImage
                                    cl::Image2D,  // rgbImage
                                    int,          // bayerPattern
                                    float,        // chromaVariance
                                    int           // rotate_180
                                    >(program, "interpolateRedBlue");

    // Schedule the kernel on the GPU
    kernel(gls::OpenCLContext::buildEnqueueArgs(rgbImage->width, rgbImage->height),
           rawImage.getImage2D(), greenImage.getImage2D(), rgbImage->getImage2D(), bayerPattern, chromaVariance, rotate_180);
}

void fasteDebayer(gls::OpenCLContext* glsContext,
                  const gls::cl_image_2d<gls::luma_pixel_float>& rawImage,
                  gls::cl_image_2d<gls::rgba_pixel_float>* rgbImage,
                  BayerPattern bayerPattern) {
    assert(rawImage.width == 2 * rgbImage->width && rawImage.height == 2 * rgbImage->height);

    // Load the shader source
    const auto program = glsContext->loadProgram("demosaic");

    // Bind the kernel parameters
    auto kernel = cl::KernelFunctor<cl::Image2D,  // rawImage
                                    cl::Image2D,  // rgbImage
                                    int           // bayerPattern
                                    >(program, "fastDebayer");

    // Schedule the kernel on the GPU
    kernel(gls::OpenCLContext::buildEnqueueArgs(rgbImage->width, rgbImage->height),
           rawImage.getImage2D(), rgbImage->getImage2D(), bayerPattern);
}

template <typename T>
void applyKernel(gls::OpenCLContext* glsContext, const std::string& kernelName,
                const gls::cl_image_2d<T>& inputImage,
                gls::cl_image_2d<T>* outputImage) {
    // Load the shader source
    const auto program = glsContext->loadProgram("demosaic");

    // Bind the kernel parameters
    auto kernel = cl::KernelFunctor<cl::Image2D,  // inputImage
                                    cl::Image2D   // outputImage
                                    >(program, kernelName);

    // Schedule the kernel on the GPU
    kernel(gls::OpenCLContext::buildEnqueueArgs(outputImage->width, outputImage->height),
           inputImage.getImage2D(), outputImage->getImage2D());
}

template <typename T>
void resampleImage(gls::OpenCLContext* glsContext, const std::string& kernelName, const gls::cl_image_2d<T>& inputImage,
                   gls::cl_image_2d<T>* outputImage) {
    // Load the shader source
    const auto program = glsContext->loadProgram("demosaic");

    const auto linear_sampler = cl::Sampler(glsContext->clContext(), true, CL_ADDRESS_CLAMP_TO_EDGE, CL_FILTER_LINEAR);

    // Bind the kernel parameters
    auto kernel = cl::KernelFunctor<cl::Image2D,  // inputImage
                                    cl::Image2D,  // outputImage
                                    cl::Sampler
                                    >(program, kernelName);

    // Schedule the kernel on the GPU
    kernel(gls::OpenCLContext::buildEnqueueArgs(outputImage->width, outputImage->height),
           inputImage.getImage2D(), outputImage->getImage2D(), linear_sampler);
}

template
void resampleImage(gls::OpenCLContext* glsContext, const std::string& kernelName, const gls::cl_image_2d<gls::rgba_pixel_float>& inputImage,
                   gls::cl_image_2d<gls::rgba_pixel_float>* outputImage);

template <typename T>
void reassembleImage(gls::OpenCLContext* glsContext, const gls::cl_image_2d<T>& inputImageDenoised0,
                     const gls::cl_image_2d<T>& inputImage1, const gls::cl_image_2d<T>& inputImageDenoised1,
                     float sharpening, gls::cl_image_2d<T>* outputImage) {
    // Load the shader source
    const auto program = glsContext->loadProgram("demosaic");

    const auto linear_sampler = cl::Sampler(glsContext->clContext(), true, CL_ADDRESS_CLAMP_TO_EDGE, CL_FILTER_LINEAR);

    // Bind the kernel parameters
    auto kernel = cl::KernelFunctor<cl::Image2D,  // inputImageDenoised0
                                    cl::Image2D,  // inputImage1
                                    cl::Image2D,  // inputImageDenoised1
                                    float,        // sharpening
                                    cl::Image2D,  // outputImage
                                    cl::Sampler   // linear_sampler
                                    >(program, "reassembleImage");

    // Schedule the kernel on the GPU
    kernel(gls::OpenCLContext::buildEnqueueArgs(outputImage->width, outputImage->height),
           inputImageDenoised0.getImage2D(), inputImage1.getImage2D(), inputImageDenoised1.getImage2D(),
           sharpening, outputImage->getImage2D(), linear_sampler);
}

template
void reassembleImage(gls::OpenCLContext* glsContext, const gls::cl_image_2d<gls::rgba_pixel_float>& inputImageDenoised0,
                     const gls::cl_image_2d<gls::rgba_pixel_float>& inputImage1, const gls::cl_image_2d<gls::rgba_pixel_float>& inputImageDenoised1,
                     float sharpening, gls::cl_image_2d<gls::rgba_pixel_float>* outputImage);

void transformImage(gls::OpenCLContext* glsContext,
                    const gls::cl_image_2d<gls::rgba_pixel_float>& linearImage,
                    gls::cl_image_2d<gls::rgba_pixel_float>* rgbImage,
                    const gls::Matrix<3, 3>& transform) {
    // Load the shader source
    const auto program = glsContext->loadProgram("demosaic");

    // Bind the kernel parameters
    auto kernel = cl::KernelFunctor<cl::Image2D,  // linearImage
                                    cl::Image2D,  // rgbImage
                                    cl::Buffer    // transform
                                    >(program, "transformImage");

    // parameter "constant float3 transform[3]". NOTE: float3 are mapped in memory as float4
    std::array<std::array<float, 4>, 3> paddedTransform;
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            paddedTransform[r][c] = transform[r][c];
        }
    }
    cl::Buffer transformBuffer(paddedTransform.begin(), paddedTransform.end(), true);

    // Schedule the kernel on the GPU
    kernel(gls::OpenCLContext::buildEnqueueArgs(rgbImage->width, rgbImage->height),
           linearImage.getImage2D(), rgbImage->getImage2D(), transformBuffer);
}

void convertTosRGB(gls::OpenCLContext* glsContext,
                  const gls::cl_image_2d<gls::rgba_pixel_float>& linearImage,
                  gls::cl_image_2d<gls::rgba_pixel>* rgbImage,
                  const gls::Matrix<3, 3>& transform, const DemosaicParameters& demosaicParameters) {
    // Load the shader source
    const auto program = glsContext->loadProgram("demosaic");

    // Bind the kernel parameters
    auto kernel = cl::KernelFunctor<cl::Image2D,  // linearImage
                                    cl::Image2D,  // rgbImage
                                    cl::Buffer,   // transform
                                    cl::Buffer    // demosaicParameters
                                    >(program, "convertTosRGB");

    // parameter "constant float3 transform[3]". NOTE: float3 are mapped in memory as float4
    std::array<std::array<float, 4>, 3> paddedTransform;
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            paddedTransform[r][c] = transform[r][c];
        }
    }
    cl::Buffer transformBuffer(paddedTransform.begin(), paddedTransform.end(), true);

    // parameter "constant DemosaicParameters *demosaicParameters".
    cl::Buffer demosaicParametersBuffer(glsContext->clContext(), CL_MEM_USE_HOST_PTR, sizeof(DemosaicParameters), (void *) &demosaicParameters);

    // Schedule the kernel on the GPU
    kernel(gls::OpenCLContext::buildEnqueueArgs(rgbImage->width, rgbImage->height),
           linearImage.getImage2D(), rgbImage->getImage2D(), transformBuffer, demosaicParametersBuffer);
}

// --- Multiscale Noise Reduction ---
// https://www.cns.nyu.edu/pub/lcv/rajashekar08a.pdf

void denoiseImage(gls::OpenCLContext* glsContext,
                  const gls::cl_image_2d<gls::rgba_pixel_float>& inputImage,
                  const DenoiseParameters& denoiseParameters,
                  gls::cl_image_2d<gls::rgba_pixel_float>* outputImage) {
    // Load the shader source
    const auto program = glsContext->loadProgram("demosaic");

    // Bind the kernel parameters
    auto kernel = cl::KernelFunctor<cl::Image2D,  // inputImage
                                    cl::Buffer,   // denoiseParameters
                                    cl::Image2D   // outputImage
                                    >(program, "denoiseImage");

    cl::Buffer denoiseParametersBuffer(glsContext->clContext(), CL_MEM_USE_HOST_PTR, sizeof(DenoiseParameters), (void *) &denoiseParameters);

    // Schedule the kernel on the GPU
    kernel(gls::OpenCLContext::buildEnqueueArgs(outputImage->width, outputImage->height),
           inputImage.getImage2D(), denoiseParametersBuffer, outputImage->getImage2D());
}
