/*******************************************************************************
 * Copyright (c) 2021-2022 Glass Imaging Inc.
 * Author: Fabio Riccardi <fabio@glass-imaging.com>
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

#ifndef GLS_ANDROID_SUPPORT_H
#define GLS_ANDROID_SUPPORT_H

#include <android/bitmap.h>

#include <cassert>
#include <map>
#include <span>

namespace gls {

std::string toString(JNIEnv* env, jstring jStr);

void loadResourceData(JNIEnv* env, jobject assetManager, std::vector<std::byte>* resourceData,
                      const std::string& resourceName);

void loadOpenCLShaders(JNIEnv* env, jobject assetManager, std::map<std::string, std::string>* shaders);

void loadOpenCLBytecode(JNIEnv* env, jobject assetManager,
                        std::map<std::string, std::vector<unsigned char>>* bytecodes);

template <class T>
class JavaArray : public std::vector<T> {
   public:
    JavaArray(JNIEnv* env, jfloatArray array) {
        jsize arrayLength = env->GetArrayLength(array);
        auto* arrayData = (T*)env->GetPrimitiveArrayCritical(array, nullptr);
        this->assign(arrayData, arrayData + arrayLength);
        env->ReleasePrimitiveArrayCritical(array, arrayData, 0);
    }
};

template <class T>
class JavaArrayCritical : public std::span<T> {
    JNIEnv* _env;
    jfloatArray _array;

   public:
    JavaArrayCritical(JNIEnv* env, jfloatArray array)
        : _env(env),
          _array(array),
          std::span<T>((T*)env->GetPrimitiveArrayCritical(array, nullptr), env->GetArrayLength(array)) {}

    ~JavaArrayCritical() { _env->ReleasePrimitiveArrayCritical(_array, this->data(), 0); }
};

struct AndroidBitmap {
    JNIEnv* _env;
    jobject _bitmap;
    AndroidBitmapInfo _info;

    AndroidBitmap(JNIEnv* env, jobject bitmap) : _env(env), _bitmap(bitmap) {
        int status = AndroidBitmap_getInfo(env, bitmap, &this->_info);
        if (status != ANDROID_BITMAP_RESULT_SUCCESS) {
            throw std::runtime_error("Failed accessing Android Bitmap object: " + std::to_string(status));
        }
    }

    [[nodiscard]] const AndroidBitmapInfo& info() const { return _info; }

    template <typename T>
    [[nodiscard]] std::span<T> lockPixels() const {
        uint8_t* bitmapData = nullptr;
        int status = AndroidBitmap_lockPixels(_env, _bitmap, (void**)&bitmapData);
        if (status != ANDROID_BITMAP_RESULT_SUCCESS) {
            throw std::runtime_error("AndroidBitmapInfo::lockPixels failure: " + std::to_string(status));
        }
        int pixelSize = 0;
        switch (_info.format) {
            case ANDROID_BITMAP_FORMAT_RGBA_8888:
                pixelSize = 4;
                break;
            case ANDROID_BITMAP_FORMAT_RGB_565:
            case ANDROID_BITMAP_FORMAT_RGBA_4444:
                pixelSize = 2;
                break;
            case ANDROID_BITMAP_FORMAT_A_8:
                pixelSize = 1;
                break;
            case ANDROID_BITMAP_FORMAT_RGBA_F16:
                pixelSize = 8;
                break;
            default:
                throw std::runtime_error("Unexpected Bitmap format: " + std::to_string(_info.format));
        }
        assert(pixelSize * _info.width == _info.stride);
        return std::span((T*)bitmapData, pixelSize * _info.width * _info.height / sizeof(T));
    }

    void unLockPixels() const {
        int status = AndroidBitmap_unlockPixels(_env, _bitmap);
        if (status != ANDROID_BITMAP_RESULT_SUCCESS) {
            throw std::runtime_error("AndroidBitmapInfo::unLockPixels failure: " + std::to_string(status));
        }
    }
};

struct AndroidBitmapThreadsafe {
  jobject _bitmap;
  AndroidBitmapInfo _info;

  AndroidBitmapThreadsafe(JNIEnv *env, jobject bitmap) {
      _bitmap = bitmap;
      int status = AndroidBitmap_getInfo(env, bitmap, &this->_info);
      if (status != ANDROID_BITMAP_RESULT_SUCCESS) {
          throw std::runtime_error(
              "Failed accessing Android Bitmap object: " + std::to_string(status));
      }
  }

  [[nodiscard]] const AndroidBitmapInfo &info() const { return _info; }

  template<typename T>
  [[nodiscard]] std::span<T> lockPixels(JNIEnv *env) const {
      uint8_t *bitmapData = nullptr;
      int status = AndroidBitmap_lockPixels(env, _bitmap, (void **) &bitmapData);
      if (status != ANDROID_BITMAP_RESULT_SUCCESS) {
          throw std::runtime_error(
              "AndroidBitmapInfo::lockPixels failure: " + std::to_string(status));
      }
      int pixelSize = 0;
      switch (_info.format) {
          case ANDROID_BITMAP_FORMAT_RGBA_8888:pixelSize = 4;
              break;
          case ANDROID_BITMAP_FORMAT_RGB_565:
          case ANDROID_BITMAP_FORMAT_RGBA_4444:pixelSize = 2;
              break;
          case ANDROID_BITMAP_FORMAT_A_8:pixelSize = 1;
              break;
          case ANDROID_BITMAP_FORMAT_RGBA_F16:pixelSize = 8;
              break;
          default:
              throw std::runtime_error("Unexpected Bitmap format: " + std::to_string(_info.format));
      }
      assert(pixelSize * _info.width == _info.stride);
      return std::span((T *) bitmapData, pixelSize * _info.width * _info.height / sizeof(T));
  }

  void unLockPixels(JNIEnv *env) const {
      int status = AndroidBitmap_unlockPixels(env, _bitmap);
      if (status != ANDROID_BITMAP_RESULT_SUCCESS) {
          throw std::runtime_error(
              "AndroidBitmapInfo::unLockPixels failure: " + std::to_string(status));
      }
  }
};

}  // namespace gls

#endif  // GLS_ANDROID_SUPPORT_H
