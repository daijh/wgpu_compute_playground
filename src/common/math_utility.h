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

#ifndef __MATH_UTILITY_H__
#define __MATH_UTILITY_H__

#define ALIGN_DOWN(x, base) (((x) / (base)) * (base))

#define ALIGN_UP(x, base) (((x + base - 1) / (base)) * (base))

#define CEIL_DIVIDE(x, base) ((x + base - 1) / (base))

#include <cmath>  // For std::abs
#include <iomanip>
#include <iostream>     // For std::cerr and std::cout
#include <type_traits>  // For std::common_type

#include "fp16.h"

#include "data_types.h"

// Function to check if two arrays with potentially different data types are
// "close". It uses specific logic for integer vs. floating-point types.
template <typename T1, typename T2>
inline bool is_close(const T1* input_data,
                     size_t input_size,
                     const T2* ground_true_data,
                     size_t ground_true_size,
                     // Default tolerances are now based on the common_type.
                     // Note: For integer types, only atol is used.
                     typename std::common_type<T1, T2>::type rtol = 0.1,
                     typename std::common_type<T1, T2>::type atol = 0.02) {
  uint32_t errors = 0;

  if (input_size != ground_true_size) {
    std::cerr << "Error: Input tensors must have the same size: " << input_size
              << ", " << ground_true_size << std::endl;
    return false;
  }

  using CommonType = typename std::common_type<T1, T2>::type;

  bool result = true;
  for (size_t i = 0; i < input_size; ++i) {
    CommonType val_input = static_cast<CommonType>(input_data[i]);
    CommonType val_ground_true = static_cast<CommonType>(ground_true_data[i]);

    // Use 'if constexpr' to provide distinct logic for integer and
    // floating-point types at compile time.
    if constexpr (std::is_integral_v<CommonType>) {
      // --- Logic for Integral Types (like uint32_t, int, etc.) ---
      CommonType diff;
      // Safely calculate the absolute difference to avoid unsigned underflow.
      if (val_input > val_ground_true) {
        diff = val_input - val_ground_true;
      } else {
        diff = val_ground_true - val_input;
      }

      // For integers, relative tolerance is often not meaningful. We use
      // the absolute tolerance 'atol'. Note that the default atol=0.02
      // will be cast to 0 for integers, meaning exact equality is required by
      // default. Provide a non-zero integer 'atol' for tolerant comparison.
      CommonType integer_atol = static_cast<CommonType>(atol);
      if (diff > integer_atol) {
        std::cout << "Index " << i << ": input " << val_input << ", expected "
                  << val_ground_true << std::endl;
        std::cout << "Difference: " << diff
                  << " > Tolerance (atol): " << integer_atol << std::endl;
        result = false;
        ++errors;
      }
    } else {
      // --- Logic for Floating-Point Types (float, double) ---
      CommonType diff = std::abs(val_input - val_ground_true);
      CommonType tolerance =
          static_cast<CommonType>(atol) +
          static_cast<CommonType>(rtol) * std::abs(val_ground_true);

      if (diff > tolerance) {
        std::cout << "Index " << i << ": input " << input_data[i]
                  << " (converted to " << val_input << "), expected "
                  << ground_true_data[i] << " (converted to " << val_ground_true
                  << ")" << std::endl;
        std::cout << "Converted Difference: " << std::fixed
                  << std::setprecision(20) << diff << std::endl;
        std::cout << "Tolerance: " << std::fixed << std::setprecision(20)
                  << tolerance << std::endl;
        result = false;
        ++errors;
      }
    }
    if (errors >= 10) {
      return false;
    }
  }

  return result;
}

#endif  // __MATH_UTILITY_H__
