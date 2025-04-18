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

#include <dawn/dawn_proc.h>
#include <dawn/native/DawnNative.h>
#include <dawn/webgpu_cpp_print.h>
#include <webgpu/webgpu_cpp.h>

#include <iostream>

#include "base_compute_runner.h"
#include "wgpu_context.h"

// WGSL shader code for a simple compute operation: output = 2 * input + 1
static const std::string kCode = R"(
@group(0) @binding(0) var<storage, read> input_buffer : array<f32, 64>;
@group(0) @binding(1) var<storage, read_write> output_buffer : array<f32, 64>;

@compute @workgroup_size(32, 1, 1)
fn main(@builtin(global_invocation_id) gid : vec3<u32>) {
  output_buffer[gid.x] = 2.0 * input_buffer[gid.x] + 1.0;
}
)";

int main() {
  // The number of elements in the buffers.
  constexpr uint32_t kElements = 64;
  // The number of work items in a workgroup.
  constexpr uint32_t kWorkgroupSize = 32;

  // Initialize Dawn's function pointers.
  dawnProcSetProcs(&dawn::native::GetProcs());

  // Create WGPUContext.
  std::unique_ptr<WGPUContext> wgpu_context = std::make_unique<WGPUContext>();
  wgpu_context->initialize();

  // Create BaseComputeRunner, passing the WGPUContext.
  std::unique_ptr<BaseComputeRunner> compute_runner =
      std::make_unique<BaseComputeRunner>(wgpu_context.get());
  compute_runner->initialize();

  // Create input/output buffer on the GPU.
  // Buffer creation sequence must align with shader binding indices.
  wgpu::BufferUsage usage = wgpu::BufferUsage::Storage |
                            wgpu::BufferUsage::CopySrc |
                            wgpu::BufferUsage::CopyDst;
  wgpu::Buffer input_buffer =
      compute_runner->add_buffer(kElements * sizeof(float), usage,
                                 wgpu::BufferBindingType::ReadOnlyStorage);
  wgpu::Buffer output_buffer = compute_runner->add_buffer(
      kElements * sizeof(float), usage, wgpu::BufferBindingType::Storage);

  // Set the shader code and the entry point function.
  compute_runner->set_shader(kCode, "main");
  // Set the dispatch size for the compute shader.
  compute_runner->set_dispatch(kElements / kWorkgroupSize, 1, 1);
  // Initialize the compute pipeline.
  compute_runner->initialize_pipeline();

  // Create host-side data for the input buffer.
  std::vector<float> input_data(kElements);
  for (size_t i = 0; i < input_data.size(); ++i) {
    input_data[i] = static_cast<float>(i);
  }
  // Write the input data to the input buffer on the GPU.
  compute_runner->write_buffer(input_buffer, input_data.data(),
                               input_data.size() * sizeof(input_data[0]));

  // Run the compute shader.
  auto times_ns = compute_runner->run();

  // Print the contents of the input buffer.
  std::cout << "======\n";
  std::cout << "Input Buffer:" << std::endl;
  compute_runner->log_buffer<float>(input_buffer);

  // Print the contents of the output buffer.
  std::cout << "======\n";
  std::cout << "Output Buffer:" << std::endl;
  compute_runner->log_buffer<float>(output_buffer);

  // Print the latency.
  std::cout << "======\n";
  std::cout << "Latency (nanoseconds):\n";
  std::cout << "  CPU Time: " << times_ns.first << " ns\n";
  std::cout << "  GPU Time: " << times_ns.second << " ns\n";

  return 0;
}
