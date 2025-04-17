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

#ifndef __WGPU_BUFFER_MANAGER_H__
#define __WGPU_BUFFER_MANAGER_H__

#include <dawn/dawn_proc.h>
#include <dawn/native/DawnNative.h>
#include <dawn/webgpu_cpp_print.h>
#include <webgpu/webgpu_cpp.h>

#include <cstdint>
#include <iostream>
#include <vector>

#include "check.h"
#include "data_types.h"
#include "math_utility.h"
#include "wgpu_context.h"

class WGPUBufferManager {
  // wgpu::Buffer minimum size and alignment requirements.
  const uint32_t kMinWGPUBufferSize = 96;
  const uint32_t kWGPUBufferAlign = 16;

 public:
  WGPUBufferManager(WGPUContext* wgpu_context);
  ~WGPUBufferManager();

  bool initialize();

  wgpu::Buffer create_buffer(size_t size, wgpu::BufferUsage usage);

  std::vector<uint8_t> read_buffer(wgpu::Buffer buffer);
  void write_buffer(wgpu::Buffer buffer, void* data, uint64_t size);

  template <typename T>
  void fill_buffer(wgpu::Buffer buffer,
                   std::vector<T> data,
                   uint64_t element_size) {
    uint64_t data_size = data.size() * sizeof(T);
    uint64_t wgpu_buffer_size = element_size * sizeof(T);

    if (data_size < static_cast<uint64_t>(kMinWGPUBufferSize)) {
      CHECK(wgpu_buffer_size == kMinWGPUBufferSize);
    } else {
      CHECK(wgpu_buffer_size == data_size);
    }

    write_buffer(buffer, data.data(), data_size);
  }

  template <typename T>
  std::string type_to_string() {
    if constexpr (std::is_same<T, float>::value) {
      return "float";
    } else if constexpr (std::is_same<T, Float16>::value) {
      return "Float16";
    } else if constexpr (std::is_same<T, uint32_t>::value) {
      return "uint32_t";
    } else if constexpr (std::is_same<T, int32_t>::value) {
      return "int32_t";
    } else {
      return "not implememt";
    }
  }

  template <typename T>
  void log_vector(std::vector<uint8_t> data, size_t log_elements) {
    const size_t max_logging = ROUND_DOWN_TO_MULTIPLE_INT_DIV(log_elements, 16);

    T* ptr = (T*)data.data();
    size_t element_size = sizeof(T);
    size_t num_elements = data.size() / element_size;

    double abs_sum = 0.0;
    for (size_t i = 0; i < num_elements; ++i) {
      T value = *reinterpret_cast<const T*>(data.data() + i * element_size);

      float float16_value = 0;
      if constexpr (std::is_same<T, Float16>::value) {
        float16_value = fp16_ieee_to_fp32_value(ptr[i]);
      }

      if (i < max_logging) {
        if (i % 16 == 0) {
          std::cout << "[" << std::setfill('0') << std::right << std::setw(6)
                    << i << "]" << ": ";
        }

        std::cout << std::left;
        if constexpr (std::is_same<T, uint32_t>::value) {
          std::cout << value << "(" << std::showbase << std::hex << value << ")"
                    << " " << std::noshowbase << std::dec;
        } else if constexpr (std::is_same<T, int32_t>::value) {
          std::cout << value << " ";
        } else if constexpr (std::is_same<T, Float16>::value) {
          std::cout << std::setprecision(3) << std::fixed << float16_value
                    << " ";
        } else if constexpr (std::is_same<T, float>::value) {
          std::cout << std::setprecision(3) << std::fixed << value << " ";
        } else {
          CHECK(0);
        }

        if ((i + 1) % 16 == 0) {
          std::cout << "\n";
        }
      }

      if constexpr (std::is_same<T, Float16>::value) {
        abs_sum += std::abs(float16_value);
      } else {
        abs_sum += std::abs(value);
      }
    }
    if (num_elements % 16 != 0) {
      std::cout << "\n";
    }

    std::cout << "Elements (" << type_to_string<T>() << ") " << num_elements
              << ", Abs Accumulate " << std::left << std::setprecision(3)
              << std::fixed << abs_sum << std::endl;
  }

  template <typename T>
  void log_buffer(wgpu::Buffer buffer, size_t log_elements) {
    std::vector<uint8_t> data = read_buffer(buffer);
    log_vector<T>(data, log_elements);
  }

 private:
  WGPUContext* wgpu_context_ = nullptr;
};
#endif  // __WGPU_BUFFER_MANAGER_H__
