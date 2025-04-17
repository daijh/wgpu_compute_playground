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

#include <algorithm>
#include <iostream>
#include <utility>

#include "base_compute_runner.h"
#include "check.h"
#include "time_utility.h"

BaseComputeRunner::BaseComputeRunner(WGPUContext* wgpu_context)
    : wgpu_context_(wgpu_context) {
  CHECK(wgpu_context_);
}

bool BaseComputeRunner::initialize() {
  wgpu_buffer_manager_ = std::make_unique<WGPUBufferManager>(wgpu_context_);
  wgpu_buffer_manager_->initialize();

  return true;
}

wgpu::Buffer BaseComputeRunner::add_buffer(
    uint64_t size,
    wgpu::BufferUsage usage,
    wgpu::BufferBindingType binding_type) {
  wgpu::Buffer buffer = wgpu_buffer_manager_->create_buffer(size, usage);
  buffer_infos_.emplace_back(buffer, size, usage, binding_type);
  return buffer;
}

int BaseComputeRunner::set_shader(std::string code, std::string entry_point) {
  code_ = code;
  entry_point_ = entry_point;
  return 0;
}

int BaseComputeRunner::set_dispatch(uint32_t dispatch_x,
                                    uint32_t dispatch_y,
                                    uint32_t dispatch_z) {
  dispatch_count_.resize(3);
  dispatch_count_[0] = dispatch_x;
  dispatch_count_[1] = dispatch_y;
  dispatch_count_[2] = dispatch_z;

  return 0;
}

void BaseComputeRunner::init_buffer_resources() {
  wgpu::Device device = wgpu_context_->device();

  std::vector<wgpu::BindGroupLayoutEntry> bind_group_layout;
  std::vector<wgpu::BindGroupEntry> bind_group;

  uint32_t binding_index = 0;
  for (auto& info : buffer_infos_) {
    wgpu::BindGroupLayoutEntry bind_group_layout_entry = {};
    bind_group_layout_entry.binding = binding_index;
    bind_group_layout_entry.buffer.type = info.binding_type;
    bind_group_layout_entry.visibility = wgpu::ShaderStage::Compute;
    bind_group_layout.push_back(bind_group_layout_entry);

    wgpu::BindGroupEntry bind_group_entry;
    bind_group_entry.binding = binding_index;
    bind_group_entry.buffer = info.buffer;
    bind_group_entry.offset = 0;
    bind_group_entry.size = info.buffer.GetSize();
    bind_group.push_back(bind_group_entry);

    ++binding_index;
  }

  wgpu::BindGroupLayoutDescriptor bind_group_layout_desc;
  bind_group_layout_desc.entryCount = bind_group_layout.size();
  bind_group_layout_desc.entries = bind_group_layout.data();
  bind_group_layout_ = device.CreateBindGroupLayout(&bind_group_layout_desc);

  wgpu::BindGroupDescriptor bind_group_desc;
  bind_group_desc.layout = bind_group_layout_;
  bind_group_desc.entryCount = bind_group.size();
  bind_group_desc.entries = bind_group.data();
  bind_group_ = device.CreateBindGroup(&bind_group_desc);
}

void BaseComputeRunner::init_compute_pipeling() {
  wgpu::Device device = wgpu_context_->device();
  std::string code = code_;

  // Create shader module
  wgpu::ShaderModuleWGSLDescriptor wgsl_desc;
  wgsl_desc.code = code.c_str();
  wgpu::ShaderModuleDescriptor descriptor{};
  descriptor.nextInChain = &wgsl_desc;
  wgpu::ShaderModule compute_shader_module =
      device.CreateShaderModule(&descriptor);
  CHECK(compute_shader_module != nullptr);

  // Create compute pipeline layout
  wgpu::PipelineLayoutDescriptor pipeline_layout_desc;
  pipeline_layout_desc.bindGroupLayoutCount = 1;
  pipeline_layout_desc.bindGroupLayouts = &bind_group_layout_;
  pipeline_layout_ = device.CreatePipelineLayout(&pipeline_layout_desc);

  // Create compute pipeline
  wgpu::ComputePipelineDescriptor compute_pipeline_desc;
  compute_pipeline_desc.compute.constantCount = 0;
  compute_pipeline_desc.compute.constants = nullptr;
  compute_pipeline_desc.compute.entryPoint = entry_point_.c_str();
  compute_pipeline_desc.compute.module = compute_shader_module;
  compute_pipeline_desc.layout = pipeline_layout_;
  compute_pipeline_ = device.CreateComputePipeline(&compute_pipeline_desc);
}

void BaseComputeRunner::create_query_set_buffer() {
  wgpu::Device device = wgpu_context_->device();

  wgpu::BufferDescriptor buffer_desc{};
  buffer_desc.size = 2 * sizeof(uint64_t);
  buffer_desc.usage =
      wgpu::BufferUsage::QueryResolve | wgpu::BufferUsage::CopySrc;
  query_set_buffer_ = device.CreateBuffer(&buffer_desc);
}

int BaseComputeRunner::initialize_pipeline() {
  init_buffer_resources();
  init_compute_pipeling();
  create_query_set_buffer();
  return 0;
}

wgpu::CommandBuffer BaseComputeRunner::record_command_buffer() {
  wgpu::Device device = wgpu_context_->device();

  // Create a query set for timestamps
  wgpu::QuerySetDescriptor query_set_desc{};
  query_set_desc.type = wgpu::QueryType::Timestamp;
  query_set_desc.count = 2;
  wgpu::QuerySet query_set = device.CreateQuerySet(&query_set_desc);

  // Set up timestamp writes
  wgpu::PassTimestampWrites timestamp_writes{};
  timestamp_writes.querySet = query_set;
  timestamp_writes.beginningOfPassWriteIndex = 0;
  timestamp_writes.endOfPassWriteIndex = 1;

  // Initialize a command encoder
  wgpu::CommandEncoderDescriptor encoder_desc{};
  wgpu::CommandEncoder command_encoder =
      device.CreateCommandEncoder(&encoder_desc);

  // Create compute pass
  wgpu::ComputePassDescriptor compute_pass_desc{};
  compute_pass_desc.timestampWrites = &timestamp_writes;
  wgpu::ComputePassEncoder compute_pass =
      command_encoder.BeginComputePass(&compute_pass_desc);

  // Use compute pass
  compute_pass.SetPipeline(compute_pipeline_);
  compute_pass.SetBindGroup(0, bind_group_, 0, nullptr);
  compute_pass.DispatchWorkgroups(dispatch_count_[0], dispatch_count_[1],
                                  dispatch_count_[2]);

  // Finalize compute pass
  compute_pass.End();

  // Resolve the query set to a buffer
  command_encoder.ResolveQuerySet(query_set, 0, 2, query_set_buffer_, 0);

  // Encode the GPU commands
  wgpu::CommandBuffer command_buffer = command_encoder.Finish();
  return command_buffer;
}

std::pair<double, double> BaseComputeRunner::run(bool high_resolution_clock) {
  wgpu::Instance instance = wgpu_context_->instance();
  wgpu::Device device = wgpu_context_->device();

  wgpu::CommandBuffer command_buffer = record_command_buffer();

  wgpu::Queue queue = device.GetQueue();

  TimeMeasurer time_measurer;
  time_measurer.begin();

  queue.Submit(1, &command_buffer);

  {
    wgpu::Future future = queue.OnSubmittedWorkDone(
        wgpu::CallbackMode::WaitAnyOnly, [&](wgpu::QueueWorkDoneStatus status) {
          CHECK(wgpu::QueueWorkDoneStatus::Success == status);

          time_measurer.end();
        });
    instance.WaitAny(future, UINT64_MAX);
  }

  std::vector<uint8_t> timestamps =
      wgpu_buffer_manager_->read_buffer(query_set_buffer_);
  uint64_t* timestamp_data = reinterpret_cast<uint64_t*>(timestamps.data());
  uint64_t latency_ns = timestamp_data[1] - timestamp_data[0];

  double cpu_latency = static_cast<double>(time_measurer.elapsed_ns()) /
                       (high_resolution_clock ? 1.0 : 1000.0);
  double gpu_latency =
      static_cast<double>(latency_ns) / (high_resolution_clock ? 1.0 : 1000.0);

  return std::make_pair(cpu_latency, gpu_latency);
}

void BaseComputeRunner::write_buffer(wgpu::Buffer buffer,
                                     void* data,
                                     uint64_t size) {
  wgpu_buffer_manager_->write_buffer(buffer, data, size);
}
