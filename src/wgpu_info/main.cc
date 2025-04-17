// Copyright 2017-2023 The Dawn & Tint Authors
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

#include <algorithm>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "check.h"

// Do nothing.
#define FormatNumber(x) (x)

// Wraps a string to about 75 characters and prints indented. Splits on
// whitespace instead of between characters in a word.
std::string WrapString(const std::string& in, const std::string& indent) {
  std::stringstream out;

  size_t last_space = 0;
  size_t start_pos = 0;
  for (size_t i = 0; i < in.size(); ++i) {
    if (in[i] == ' ') {
      last_space = i;
    } else if (in[i] == '\n') {
      last_space = i;
    }

    if ((i - start_pos) != 0 && ((i - start_pos) % 75) == 0) {
      out << indent << in.substr(start_pos, last_space - start_pos) << "\n";
      start_pos = last_space + 1;
      last_space = start_pos;
    }
  }
  out << indent << in.substr(start_pos, in.size() - start_pos);

  return out.str();
}

std::string AdapterTypeToString(wgpu::AdapterType type) {
  switch (type) {
    case wgpu::AdapterType::DiscreteGPU:
      return "discrete GPU";
    case wgpu::AdapterType::IntegratedGPU:
      return "integrated GPU";
    case wgpu::AdapterType::CPU:
      return "CPU";
    case wgpu::AdapterType::Unknown:
      break;
  }
  return "unknown";
}

std::string BackendTypeToString(wgpu::BackendType type) {
  switch (type) {
    case wgpu::BackendType::Null:
      return "Null";
    case wgpu::BackendType::WebGPU:
      return "WebGPU";
    case wgpu::BackendType::D3D11:
      return "D3D11";
    case wgpu::BackendType::D3D12:
      return "D3D12";
    case wgpu::BackendType::Metal:
      return "Metal";
    case wgpu::BackendType::Vulkan:
      return "Vulkan";
    case wgpu::BackendType::OpenGL:
      return "OpenGL";
    case wgpu::BackendType::OpenGLES:
      return "OpenGLES";
    case wgpu::BackendType::Undefined:
      return "Undefined";
  }
  return "unknown";
}

std::string LimitsToString(const wgpu::Limits& limits,
                           const std::string& indent) {
  std::stringstream out;

  out << indent
      << "maxTextureDimension1D: " << FormatNumber(limits.maxTextureDimension1D)
      << "\n";
  out << indent
      << "maxTextureDimension2D: " << FormatNumber(limits.maxTextureDimension2D)
      << "\n";
  out << indent
      << "maxTextureDimension3D: " << FormatNumber(limits.maxTextureDimension3D)
      << "\n";
  out << indent
      << "maxTextureArrayLayers: " << FormatNumber(limits.maxTextureArrayLayers)
      << "\n";
  out << indent << "maxBindGroups: " << FormatNumber(limits.maxBindGroups)
      << "\n";
  out << indent << "maxBindGroupsPlusVertexBuffers: "
      << FormatNumber(limits.maxBindGroupsPlusVertexBuffers) << "\n";
  out << indent << "maxBindingsPerBindGroup: "
      << FormatNumber(limits.maxBindingsPerBindGroup) << "\n";
  out << indent << "maxDynamicUniformBuffersPerPipelineLayout: "
      << FormatNumber(limits.maxDynamicUniformBuffersPerPipelineLayout) << "\n";
  out << indent << "maxDynamicStorageBuffersPerPipelineLayout: "
      << FormatNumber(limits.maxDynamicStorageBuffersPerPipelineLayout) << "\n";
  out << indent << "maxSampledTexturesPerShaderStage: "
      << FormatNumber(limits.maxSampledTexturesPerShaderStage) << "\n";
  out << indent << "maxSamplersPerShaderStage: "
      << FormatNumber(limits.maxSamplersPerShaderStage) << "\n";
  out << indent << "maxStorageBuffersPerShaderStage: "
      << FormatNumber(limits.maxStorageBuffersPerShaderStage) << "\n";
  out << indent << "maxStorageTexturesPerShaderStage: "
      << FormatNumber(limits.maxStorageTexturesPerShaderStage) << "\n";
  out << indent << "maxUniformBuffersPerShaderStage: "
      << FormatNumber(limits.maxUniformBuffersPerShaderStage) << "\n";
  out << indent << "maxUniformBufferBindingSize: "
      << FormatNumber(limits.maxUniformBufferBindingSize) << "\n";
  out << indent << "maxStorageBufferBindingSize: "
      << FormatNumber(limits.maxStorageBufferBindingSize) << "\n";
  out << indent << "minUniformBufferOffsetAlignment: "
      << FormatNumber(limits.minUniformBufferOffsetAlignment) << "\n";
  out << indent << "minStorageBufferOffsetAlignment: "
      << FormatNumber(limits.minStorageBufferOffsetAlignment) << "\n";
  out << indent << "maxVertexBuffers: " << FormatNumber(limits.maxVertexBuffers)
      << "\n";
  out << indent << "maxBufferSize: " << FormatNumber(limits.maxBufferSize)
      << "\n";
  out << indent
      << "maxVertexAttributes: " << FormatNumber(limits.maxVertexAttributes)
      << "\n";
  out << indent << "maxVertexBufferArrayStride: "
      << FormatNumber(limits.maxVertexBufferArrayStride) << "\n";
  out << indent << "maxInterStageShaderVariables: "
      << FormatNumber(limits.maxInterStageShaderVariables) << "\n";
  out << indent
      << "maxColorAttachments: " << FormatNumber(limits.maxColorAttachments)
      << "\n";
  out << indent << "maxColorAttachmentBytesPerSample: "
      << FormatNumber(limits.maxColorAttachmentBytesPerSample) << "\n";
  out << indent << "maxComputeWorkgroupStorageSize: "
      << FormatNumber(limits.maxComputeWorkgroupStorageSize) << "\n";
  out << indent << "maxComputeInvocationsPerWorkgroup: "
      << FormatNumber(limits.maxComputeInvocationsPerWorkgroup) << "\n";
  out << indent << "maxComputeWorkgroupSizeX: "
      << FormatNumber(limits.maxComputeWorkgroupSizeX) << "\n";
  out << indent << "maxComputeWorkgroupSizeY: "
      << FormatNumber(limits.maxComputeWorkgroupSizeY) << "\n";
  out << indent << "maxComputeWorkgroupSizeZ: "
      << FormatNumber(limits.maxComputeWorkgroupSizeZ) << "\n";
  out << indent << "maxComputeWorkgroupsPerDimension: "
      << FormatNumber(limits.maxComputeWorkgroupsPerDimension) << "\n";

  return out.str();
}

void DumpAdapterProperties(const wgpu::Adapter& adapter,
                           const std::string& indent = "  ") {
  wgpu::Status ret;

  wgpu::AdapterPropertiesSubgroups subgroup_properties;
  subgroup_properties.subgroupMinSize = 0;
  subgroup_properties.subgroupMaxSize = 0;

  wgpu::AdapterInfo info = {};
  info.nextInChain = &subgroup_properties;

  ret = adapter.GetInfo(&info);
  CHECK(ret == wgpu::Status::Success);
  CHECK(subgroup_properties.nextInChain == nullptr);

  std::cout << indent << "vendor: " << info.vendor << "\n";
  std::cout << indent << "architecture: " << info.architecture << "\n";
  std::cout << indent << "device: " << info.device << "\n";
  std::cout << indent << "description: " << info.description << "\n";
  std::cout << indent
            << "backendType: " << BackendTypeToString(info.backendType) << "\n";
  std::cout << indent
            << "adapterType: " << AdapterTypeToString(info.adapterType) << "\n";
  std::cout << indent
            << "subgroupMinSize: " << subgroup_properties.subgroupMinSize
            << "\n";
  std::cout << indent
            << "subgroupMaxSize: " << subgroup_properties.subgroupMaxSize
            << "\n";
}

void DumpAdapterFeatures(const wgpu::Adapter& adapter,
                         const std::string& indent = "  ") {
  wgpu::SupportedFeatures supportedFeatures;
  adapter.GetFeatures(&supportedFeatures);

  std::cout << "\n";
  std::cout << indent << "Features\n";
  std::cout << indent << "========\n";

  std::string local_indent = indent + "  ";
  for (size_t i = 0; i < supportedFeatures.featureCount; ++i) {
    std::cout << local_indent << supportedFeatures.features[i] << "\n";
  }
}

void DumpAdapterLimits(const wgpu::Adapter& adapter,
                       const std::string& indent = "  ") {
  std::string local_indent = indent + "  ";

  wgpu::Limits adapterLimits;
  if (adapter.GetLimits(&adapterLimits)) {
    std::cout << "\n";
    std::cout << indent << "Adapter Limits\n";
    std::cout << indent << "==============\n";
    std::cout << LimitsToString(adapterLimits, local_indent) << "\n";
  }
}

void DumpAdapter(const wgpu::Adapter& adapter) {
  std::cout << "Adapter\n";
  std::cout << "=======\n";

  DumpAdapterProperties(adapter, "  ");
  DumpAdapterFeatures(adapter, "  ");
  DumpAdapterLimits(adapter, "  ");
}

std::string Adapter_ID(const wgpu::Adapter& adapter) {
  wgpu::Status ret;

  wgpu::AdapterInfo info = {};
  ret = adapter.GetInfo(&info);
  CHECK(ret == wgpu::Status::Success);

  std::string id = std::string(info.vendor);
  id += std::string("\n") + std::string(info.architecture);
  id += std::string("\n") + std::string(info.device);
  id += std::string("\n") + std::string(info.description);
  id += std::string("\n") + BackendTypeToString(info.backendType);
  id += std::string("\n") + AdapterTypeToString(info.adapterType);

  return id;
}

wgpu::Adapter RequireAdapter(wgpu::Instance instance,
                             wgpu::BackendType backend_type,
                             wgpu::PowerPreference power_preference) {
  wgpu::RequestAdapterOptions options = {};
  options.backendType = backend_type;
  options.powerPreference = power_preference;

  wgpu::Adapter result;
  wgpu::Future future = instance.RequestAdapter(
      &options, wgpu::CallbackMode::WaitAnyOnly,
      [&result](wgpu::RequestAdapterStatus status, wgpu::Adapter adapter,
                wgpu::StringView message) {
        if (status != wgpu::RequestAdapterStatus::Success) {
          return;
        }
        result = std::move(adapter);
      });
  instance.WaitAny(future, UINT64_MAX);

  return result;
}

void dump_toggles() {
  auto toggles = dawn::native::AllToggleInfos();
  std::sort(toggles.begin(), toggles.end(), [](const auto* a, const auto* b) {
    return std::string(a->name) < std::string(b->name);
  });

  std::cout << "Toggles\n";
  std::cout << "=======\n";
  bool first = true;
  for (const auto* info : toggles) {
    if (!first) {
      std::cout << "\n";
    }
    first = false;
    std::cout << "  Name: " << info->name << "\n";
    std::cout << WrapString(info->description, "    ") << "\n";
    std::cout << "    " << info->url << "\n";
    std::cout << "    " << "stage " << static_cast<int>(info->stage) << "\n";
  }
  std::cout << "\n";
}

int main(int argc, char** argv) {
  dawnProcSetProcs(&dawn::native::GetProcs());

  dump_toggles();

  wgpu::InstanceDescriptor instanceDescriptor{};
  instanceDescriptor.capabilities.timedWaitAnyEnable = true;
  wgpu::Instance instance = wgpu::CreateInstance(&instanceDescriptor);

  std::vector<wgpu::PowerPreference> power_preferences = {
      wgpu::PowerPreference::LowPower, wgpu::PowerPreference::HighPerformance};
  std::vector<wgpu::BackendType> backend_types = {wgpu::BackendType::D3D12,
                                                  wgpu::BackendType::Vulkan};

  std::vector<std::string> cached_ids;
  for (auto& power_preference : power_preferences) {
    for (auto& backend_type : backend_types) {
      wgpu::Adapter adapter =
          RequireAdapter(instance, backend_type, power_preference);
      if (adapter != nullptr) {
        std::string id = Adapter_ID(adapter);
        if (std::find(cached_ids.begin(), cached_ids.end(), id) !=
            cached_ids.end()) {
          continue;
        }

        cached_ids.push_back(id);
        std::cout << "=======\n";
        DumpAdapter(adapter);
      }
    }
  }

  return 0;
}
