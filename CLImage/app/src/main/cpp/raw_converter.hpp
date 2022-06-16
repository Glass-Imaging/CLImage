//
//  raw_converter.hpp
//  RawPipeline
//
//  Created by Fabio Riccardi on 5/16/22.
//

#ifndef raw_converter_hpp
#define raw_converter_hpp

#include "gls_cl_image.hpp"
#include "pyramidal_denoise.hpp"

class RawConverter {
    gls::OpenCLContext* _glsContext;

    // RawConverter base work textures
    gls::cl_image_2d<gls::luma_pixel_16>::unique_ptr clRawImage;
    gls::cl_image_2d<gls::luma_pixel_float>::unique_ptr clScaledRawImage;
    gls::cl_image_2d<gls::luma_pixel_float>::unique_ptr clGreenImage;
    gls::cl_image_2d<gls::rgba_pixel_float>::unique_ptr clLinearRGBImageA;
    gls::cl_image_2d<gls::rgba_pixel_float>::unique_ptr clLinearRGBImageB;
    gls::cl_image_2d<gls::rgba_pixel>::unique_ptr clsRGBImage;

    std::unique_ptr<PyramidalDenoise<5>> pyramidalDenoise;

    gls::cl_image_2d<gls::luma_pixel_float>::unique_ptr ltmMaskImage;

    // RawConverter HighNoise textures
    gls::cl_image_2d<gls::rgba_pixel_float>::unique_ptr rgbaRawImage;
    gls::cl_image_2d<gls::rgba_pixel_float>::unique_ptr denoisedRgbaRawImage;

    // Fast (half resolution) RawConverter textures
    gls::cl_image_2d<gls::rgba_pixel_float>::unique_ptr clFastLinearRGBImage;
    gls::cl_image_2d<gls::rgba_pixel>::unique_ptr clsFastRGBImage;

    void allocateTextures(gls::OpenCLContext* glsContext, int width, int height);
    void allocateLtmMaskImage(gls::OpenCLContext* glsContext, int width, int height);
    void allocateHighNoiseTextures(gls::OpenCLContext* glsContext, int width, int height);
    void allocateFastDemosaicTextures(gls::OpenCLContext* glsContext, int width, int height);

public:
    RawConverter(gls::OpenCLContext* glsContext) : _glsContext(glsContext) { }

    gls::cl_image_2d<gls::rgba_pixel>* demosaicImage(const gls::image<gls::luma_pixel_16>& rawImage,
                                                     DemosaicParameters* demosaicParameters,
                                                     const gls::rectangle* gmb_position, bool rotate_180);

    gls::cl_image_2d<gls::rgba_pixel>* fastDemosaicImage(const gls::image<gls::luma_pixel_16>& rawImage,
                                                         const DemosaicParameters& demosaicParameters);

    static gls::image<gls::rgb_pixel>::unique_ptr convertToRGBImage(const gls::cl_image_2d<gls::rgba_pixel>& clRGBAImage);

    static gls::image<gls::rgba_pixel>::unique_ptr convertToRGBAImage(const gls::cl_image_2d<gls::rgba_pixel> &clRGBAImage);
};

#endif /* raw_converter_hpp */
