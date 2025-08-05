// BSD 3-Clause License
//
// Copyright (c) 2025, Jianhui Dai
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef __TEXTURE_UTILITY_H__
#define __TEXTURE_UTILITY_H__

#include <webgpu/webgpu_cpp.h>

#include <cstdint>
#include <vector>

#include "math_utility.h"

inline uint32_t get_texture_bytes_per_pixel(
    wgpu::TextureFormat texture_format) {
  switch (texture_format) {
    case wgpu::TextureFormat::RGBA8Sint:
    case wgpu::TextureFormat::RGBA8Uint:
      return 4;
    case wgpu::TextureFormat::RGBA16Float:
    case wgpu::TextureFormat::RGBA16Sint:
    case wgpu::TextureFormat::RGBA16Uint:
      return 8;
    case wgpu::TextureFormat::RGBA32Float:
    case wgpu::TextureFormat::RGBA32Sint:
    case wgpu::TextureFormat::RGBA32Uint:
      return 16;
    default:
      std::cout << "Warning: Cannot get simple bytes per pixel.\n";
      CHECK(0);
      return 0;
  }

  return 0;
}

inline wgpu::TextureSampleType get_texture_sample_type(
    wgpu::TextureFormat format) {
  switch (format) {
    case wgpu::TextureFormat::RGBA32Float:
    case wgpu::TextureFormat::RGBA16Float:
      return wgpu::TextureSampleType::UnfilterableFloat;
    case wgpu::TextureFormat::RGBA32Sint:
    case wgpu::TextureFormat::RGBA16Sint:
    case wgpu::TextureFormat::RGBA8Sint:
      return wgpu::TextureSampleType::Sint;
    case wgpu::TextureFormat::RGBA32Uint:
    case wgpu::TextureFormat::RGBA16Uint:
    case wgpu::TextureFormat::RGBA8Uint:
      return wgpu::TextureSampleType::Uint;
    default:
      std::cout << "Warning: Cannot get texture sample type.\n";
      CHECK(0);
      return wgpu::TextureSampleType::Undefined;
  }

  return wgpu::TextureSampleType::Undefined;
}

inline uint32_t get_texture_stride(wgpu::TextureFormat format,
                                   uint32_t texture_width) {
  uint32_t bytes_per_pixel = get_texture_bytes_per_pixel(format);
  CHECK(256 % bytes_per_pixel == 0);

  uint32_t alignment = 256 / bytes_per_pixel;
  return ALIGN_UP(texture_width, alignment);
}

#endif  // __TEXTURE_UTILITY_H__
