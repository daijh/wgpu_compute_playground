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

#ifndef __BASE_COMPUTE_RUNNER_H__
#define __BASE_COMPUTE_RUNNER_H__

#include "wgpu_buffer_manager.h"
#include "wgpu_context.h"

class BaseComputeRunner {
 public:
  BaseComputeRunner(WGPUContext* wgpu_context);
  ~BaseComputeRunner() {}

  /**
   * @brief Sequence of steps.
   *
   * 1. **Initialization:** Call `initialize()` to set up the WebGPU environment
   * (instance, adapter, device). This must be the first step.
   *
   * 2. **Buffer Creation:** Call `add_buffer()` for each buffer your compute
   * shader will use. The order of these calls is important as it implicitly
   * corresponds to the binding IDs in your WGSL shader
   * (e.g., first `add_buffer` might be `@binding(0)`).
   *
   * 3. **Shader Configuration:** Call `set_shader()` to provide the WGSL shader
   * code and specify the entry point function.
   *
   * 4. **Dispatch Configuration:** Call `set_dispatch()` to set the dimensions
   * (x, y, z) for the compute shader dispatch. This determines the number of
   * workgroups that will be launched.
   *
   * 5. **Pipeline Initialization:** Call `initialize_pipeline()` to create the
   * WebGPU compute pipeline based on the configured shader and (implicitly)
   * the buffers added in the previous steps.
   *
   * 6. **Data Transfer (Input):** Call `write_buffer()` to copy data from your
   * application's memory to the WebGPU buffers you created. Do this *before*
   * running the computation.
   *
   * 7. **Compute Execution:** Call `run()` to execute the compute pipeline on
   * the GPU. This will perform the calculations defined in your shader.
   *
   * 8. **Data Retrieval (Output/Debugging):** (Optional) Call `log_buffer()` to
   * read the contents of a WebGPU buffer back to the CPU and print them to
   * the console. This is useful for inspecting the results of the computation
   * or for debugging.
   */

  /**
   * @brief Initializes the WebGPU environment.
   *
   * This function performs the necessary steps to set up the WebGPU instance,
   * adapter, and device. It must be called before any other methods.
   *
   * @return true if initialization was successful, false otherwise.
   */
  bool initialize();

  /**
   * @brief Adds a buffer to be used in the compute pipeline.
   *
   * This function creates a WebGPU buffer with the specified size and usage
   * flags. The `binding_type` hints at how this buffer will be used in the
   * shader. **Crucially, the order in which you call `add_buffer` should
   * correspond to the binding IDs declared for these resources in your WGSL
   * shader code. For example, the first buffer added might correspond to
   * `@group(0) @binding(0)`, the second to `@group(0) @binding(1)`, and so
   * on.**
   *
   * @param size The size of the buffer in bytes.
   * @param usage Flags indicating how the buffer will be used (e.g., storage,
   * uniform, copy source/destination).
   * @param binding_type The expected binding type of this buffer in the shader
   * (e.g., Storage, ReadOnlyStorage, Uniform).
   * @return A wgpu::Buffer object representing the created buffer.
   */
  wgpu::Buffer add_buffer(uint64_t size,
                          wgpu::BufferUsage usage,
                          wgpu::BufferBindingType binding_type);

  /**
   * @brief Sets the compute shader code and entry point.
   *
   * This function takes the shader source code as a string and the name of the
   * entry point function within the shader.
   *
   * @param code The WGSL shader source code.
   * @param entry_point The name of the function to execute as the compute
   * shader.
   * @return An integer indicating the success or failure of setting the shader
   * (e.g., 0 for success, non-zero for error).
   */
  int set_shader(std::string code, std::string entry_point);

  /**
   * @brief Sets the dispatch dimensions for the compute shader execution.
   *
   * This function specifies the number of workgroups to launch along the X, Y,
   * and Z axes.
   *
   * @param dispatch_x The number of workgroups to dispatch in the X dimension.
   * @param dispatch_y The number of workgroups to dispatch in the Y dimension.
   * @param dispatch_z The number of workgroups to dispatch in the Z dimension.
   * @return An integer indicating the success or failure of setting the
   * dispatch dimensions.
   */
  int set_dispatch(uint32_t dispatch_x,
                   uint32_t dispatch_y,
                   uint32_t dispatch_z);

  /**
   * @brief Initializes the compute pipeline.
   *
   * This function creates the WebGPU compute pipeline based on the set shader,
   * buffer bindings, and other configurations. This must be called after
   * setting the shader and adding the necessary buffers.
   *
   * @return An integer indicating the success or failure of initializing the
   * pipeline.
   */
  int initialize_pipeline();

  /**
   * @brief Writes data to a WebGPU buffer.
   *
   * This function copies data from the provided memory location to the
   * specified WebGPU buffer.
   *
   * @param buffer The target WebGPU buffer to write to.
   * @param data A pointer to the source data in system memory.
   * @param size The size of the data to write in bytes.
   */
  void write_buffer(wgpu::Buffer buffer, void* data, uint64_t size);

  /**
   * @brief Measures the latency of the compute operation.
   *
   * This function executes the configured compute pipeline and measures the
   * time taken on both the CPU and the GPU. The precision of the measurements
   * depends on the `high_resolution_clock` parameter.
   *
   * @param high_resolution_clock If true, uses a high-resolution clock for
   * finer-grained timing, returning times in nanoseconds. If false,
   * uses a standard clock, returning times in microseconds. Defaults to false.
   * @return A pair containing the CPU time and the GPU time taken for the
   * operation.
   * - `first`: CPU time in nanoseconds (if `high_resolution_clock` is true)
   * or microseconds (if `high_resolution_clock` is false).
   * - `second`: GPU time in nanoseconds (if `high_resolution_clock` is true)
   * or microseconds (if `high_resolution_clock` is false).
   */
  std::pair<double, double> run(bool high_resolution_clock = false);

  /**
   * @brief Logs the contents of a WebGPU buffer to the console.
   *
   * This is a template function that allows logging the data in the buffer
   * as elements of a specific type `T`. It reads the buffer's contents
   * from the GPU and prints them to the standard output.
   *
   * @tparam T The data type of the elements stored in the buffer (e.g., float,
   * int, custom struct).
   * @param buffer The wgpu::Buffer to log.
   * @param log_elements The maximum number of elements of type `T` to log.
   * Defaults to `SIZE_MAX` to log the entire buffer.
   */
  template <typename T>
  void log_buffer(wgpu::Buffer buffer, size_t log_elements = SIZE_MAX) {
    wgpu_buffer_manager_->log_buffer<T>(buffer, log_elements);
  }

 private:
  void init_buffer_resources();
  void init_compute_pipeling();
  void create_query_set_buffer();

  wgpu::CommandBuffer record_command_buffer();

  WGPUContext* wgpu_context_ = nullptr;
  std::unique_ptr<WGPUBufferManager> wgpu_buffer_manager_;

  struct BufferInfo {
    wgpu::Buffer buffer;
    uint64_t size;
    wgpu::BufferUsage usage;
    wgpu::BufferBindingType binding_type;
  };
  std::vector<BufferInfo> buffer_infos_;

  std::string code_;
  std::string entry_point_;

  std::vector<uint32_t> dispatch_count_;

  wgpu::BindGroupLayout bind_group_layout_;
  wgpu::BindGroup bind_group_;

  wgpu::PipelineLayout pipeline_layout_;
  wgpu::ComputePipeline compute_pipeline_;

  wgpu::Buffer query_set_buffer_;
};
#endif  // __BASE_COMPUTE_RUNNER_H__
