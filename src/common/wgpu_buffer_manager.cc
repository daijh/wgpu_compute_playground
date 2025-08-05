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

#include <cstring>
#include <iostream>
#include <sstream>

#include "check.h"
#include "texture_utility.h"
#include "wgpu_buffer_manager.h"

WGPUBufferManager::WGPUBufferManager(WGPUContext* wgpu_context)
    : wgpu_context_(wgpu_context) {
  CHECK(wgpu_context_);
}

WGPUBufferManager::~WGPUBufferManager() {}

bool WGPUBufferManager::initialize() {
  return true;
}

wgpu::Buffer WGPUBufferManager::create_buffer(size_t size,
                                              wgpu::BufferUsage usage) {
  auto device = wgpu_context_->device();

  size_t buffer_size = std::max(size, static_cast<size_t>(kMinWGPUBufferSize));
  buffer_size = ALIGN_UP(buffer_size, static_cast<size_t>(kWGPUBufferAlign));

  wgpu::BufferDescriptor bufferDesc;
  bufferDesc.mappedAtCreation = false;
  bufferDesc.size = buffer_size;
  bufferDesc.usage = usage;
  wgpu::Buffer buffer = device.CreateBuffer(&bufferDesc);

  return buffer;
}

std::vector<uint8_t> WGPUBufferManager::read_buffer(wgpu::Buffer buffer) {
  auto instance = wgpu_context_->instance();
  auto device = wgpu_context_->device();
  auto queue = device.GetQueue();

  uint64_t size = buffer.GetSize();
  std::vector<uint8_t> data(size);

  // Create staging buffer
  wgpu::Buffer staging_buffer = create_buffer(
      size, wgpu::BufferUsage::MapRead | wgpu::BufferUsage::CopyDst);

  wgpu::CommandEncoder encoder = device.CreateCommandEncoder();
  encoder.CopyBufferToBuffer(buffer, 0, staging_buffer, 0, size);
  wgpu::CommandBuffer commands = encoder.Finish();
  queue.Submit(1, &commands);

  {
    wgpu::Future future = queue.OnSubmittedWorkDone(
        wgpu::CallbackMode::WaitAnyOnly,
        [](wgpu::QueueWorkDoneStatus status, wgpu::StringView message) {
          CHECK(wgpu::QueueWorkDoneStatus::Success == status);
        });
    instance.WaitAny(future, UINT64_MAX);
  }

  {
    wgpu::Future future = staging_buffer.MapAsync(
        wgpu::MapMode::Read, 0, size, wgpu::CallbackMode::WaitAnyOnly,
        [&](wgpu::MapAsyncStatus status, const char*) {
          CHECK(wgpu::MapAsyncStatus::Success == status);
          const uint8_t* mapped =
              (const uint8_t*)staging_buffer.GetConstMappedRange(0, size);
          memcpy(data.data(), mapped, size);
          staging_buffer.Unmap();
        });
    instance.WaitAny(future, UINT64_MAX);
  }

  return data;
}

void WGPUBufferManager::write_buffer(wgpu::Buffer buffer,
                                     void* data,
                                     uint64_t size) {
  wgpu::Instance instance = wgpu_context_->instance();
  wgpu::Device device = wgpu_context_->device();
  wgpu::Queue queue = device.GetQueue();

  CHECK(buffer.GetSize() >= kMinWGPUBufferSize);
  CHECK(buffer.GetSize() >= size);
  queue.WriteBuffer(buffer, 0, data, size);

  {
    wgpu::Future future = queue.OnSubmittedWorkDone(
        wgpu::CallbackMode::WaitAnyOnly,
        [](wgpu::QueueWorkDoneStatus status, wgpu::StringView message) {
          CHECK(wgpu::QueueWorkDoneStatus::Success == status);
        });
    instance.WaitAny(future, UINT64_MAX);
  }
}

wgpu::Texture WGPUBufferManager::create_texture(
    uint32_t width,
    uint32_t height,
    wgpu::TextureFormat texture_format,
    wgpu::TextureUsage texture_usage) {
  auto device = wgpu_context_->device();

  wgpu::TextureDescriptor texture_desc = {};
  texture_desc.size = {width, height, 1};
  texture_desc.mipLevelCount = 1;
  texture_desc.sampleCount = 1;
  texture_desc.dimension = wgpu::TextureDimension::e2D;
  texture_desc.format = texture_format;
  texture_desc.usage = texture_usage;

  wgpu::Texture texture = device.CreateTexture(&texture_desc);

  CHECK(texture);
  return texture;
}

std::vector<uint8_t> WGPUBufferManager::read_texture(wgpu::Texture texture) {
  auto instance = wgpu_context_->instance();
  auto device = wgpu_context_->device();
  auto queue = device.GetQueue();

  const uint32_t width = texture.GetWidth();
  const uint32_t height = texture.GetHeight();

  const uint32_t bytes_per_row =
      get_texture_bytes_per_pixel(texture.GetFormat()) *
      get_texture_stride(texture.GetFormat(), width);
  uint64_t size = bytes_per_row * height;
  std::vector<uint8_t> data(size);

  // Create staging buffer
  wgpu::Buffer staging_buffer = create_buffer(
      size, wgpu::BufferUsage::MapRead | wgpu::BufferUsage::CopyDst);

  wgpu::CommandEncoder encoder = device.CreateCommandEncoder();

  wgpu::TexelCopyTextureInfo source = {};
  source.texture = texture;

  wgpu::TexelCopyBufferInfo destination = {};
  destination.buffer = staging_buffer;
  destination.layout.bytesPerRow = bytes_per_row;
  destination.layout.rowsPerImage = height;

  wgpu::Extent3D read_size = {};
  read_size.width = width;
  read_size.height = height;

  encoder.CopyTextureToBuffer(&source, &destination, &read_size);
  wgpu::CommandBuffer commands = encoder.Finish();
  queue.Submit(1, &commands);

  {
    wgpu::Future future = queue.OnSubmittedWorkDone(
        wgpu::CallbackMode::WaitAnyOnly,
        [](wgpu::QueueWorkDoneStatus status, wgpu::StringView message) {
          CHECK(wgpu::QueueWorkDoneStatus::Success == status);
        });
    instance.WaitAny(future, UINT64_MAX);
  }

  {
    wgpu::Future future = staging_buffer.MapAsync(
        wgpu::MapMode::Read, 0, size, wgpu::CallbackMode::WaitAnyOnly,
        [&](wgpu::MapAsyncStatus status, const char*) {
          CHECK(wgpu::MapAsyncStatus::Success == status);
          const uint8_t* mapped =
              (const uint8_t*)staging_buffer.GetConstMappedRange(0, size);
          memcpy(data.data(), mapped, size);
          staging_buffer.Unmap();
        });
    instance.WaitAny(future, UINT64_MAX);
  }

  return data;
}

void WGPUBufferManager::write_texture(wgpu::Texture texture,
                                      void* data,
                                      uint64_t size) {
  wgpu::Instance instance = wgpu_context_->instance();
  wgpu::Device device = wgpu_context_->device();
  wgpu::Queue queue = device.GetQueue();

  const uint32_t width = texture.GetWidth();

  const uint32_t bytes_per_row =
      get_texture_bytes_per_pixel(texture.GetFormat()) *
      get_texture_stride(texture.GetFormat(), width);
  const uint32_t rows = static_cast<uint32_t>(size / bytes_per_row);

  wgpu::TexelCopyTextureInfo destination = {};
  destination.texture = texture;

  wgpu::TexelCopyBufferLayout dataLayout = {};
  dataLayout.bytesPerRow = bytes_per_row;
  dataLayout.rowsPerImage = rows;

  wgpu::Extent3D writeSize = {};
  writeSize.width = texture.GetWidth();
  writeSize.height = rows;

  queue.WriteTexture(&destination, data, size, &dataLayout, &writeSize);

  {
    wgpu::Future future = queue.OnSubmittedWorkDone(
        wgpu::CallbackMode::WaitAnyOnly,
        [](wgpu::QueueWorkDoneStatus status, wgpu::StringView message) {
          CHECK(wgpu::QueueWorkDoneStatus::Success == status);
        });
    instance.WaitAny(future, UINT64_MAX);
  }
}
