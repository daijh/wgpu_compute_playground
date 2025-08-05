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

#ifndef __DATA_TYPES_H__
#define __DATA_TYPES_H__

#include <cstdint>
#include <cstring>

#include <fp16.h>

using Float16 = uint16_t;

inline std::vector<float> float16_to_float(Float16* data, size_t size) {
  std::vector<float> result(size);
  for (uint32_t i = 0; i < size; ++i) {
    result[i] = fp16_ieee_to_fp32_value(data[i]);
  }

  return result;
}

// Template function to reinterpret a vector of one type as a vector of another
// type
template <typename OutputType, typename InputType>
std::vector<OutputType> reinterpret_bytes_as_type(
    const std::vector<InputType>& input_data) {
  // Ensure the size of an InputType element is 1 byte for this interpretation
  // to be a direct byte-reinterpretation. If InputType is not 1 byte, the logic
  // might need adjustment based on the desired behavior (e.g., reinterpreting
  // elements). For now, we'll assume InputType is a byte-like type (e.g.,
  // uint8_t, char, std::byte). A static_assert could be used for a compile-time
  // check if InputType must be 1 byte. static_assert(sizeof(InputType) == 1,
  // "InputType must be a single-byte type for direct reinterpretation.");

  // Calculate total size of input data in bytes
  size_t total_input_bytes = input_data.size() * sizeof(InputType);

  // Ensure the total size of the input data is a multiple of the size of an
  // OutputType
  if (total_input_bytes % sizeof(OutputType) != 0) {
    std::cerr << "Error: Total input data size (" << total_input_bytes
              << " bytes) is not a multiple of sizeof(OutputType) ("
              << sizeof(OutputType) << " bytes)." << std::endl;
    // You could also throw an exception here:
    // throw std::runtime_error("Input data size not a multiple of OutputType
    // size");
    return {};  // Return an empty vector to indicate failure
  }

  // Calculate the number of OutputType elements
  size_t num_output_elements = total_input_bytes / sizeof(OutputType);

  // Create the result vector of OutputType with the correct size
  std::vector<OutputType> output_vector(num_output_elements);

  // Use memcpy to copy the raw bytes from the source to the destination
  // This reinterprets the byte pattern as OutputType values
  if (!input_data.empty()) {  // Avoid memcpy with null pointers if input_data
                              // is empty
    std::memcpy(output_vector.data(), input_data.data(), total_input_bytes);
  }

  return output_vector;
}

#endif  // __DATA_TYPES_H__
