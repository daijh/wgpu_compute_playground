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

#include <iostream>

#include "check.h"
#include "wgpu_context.h"

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

void DumpAdapter(const wgpu::Adapter& adapter) {
  wgpu::Status ret;

  wgpu::AdapterInfo info = {};
  ret = adapter.GetInfo(&info);
  CHECK(ret == wgpu::Status::Success);

  std::cout << "======" << std::endl;
  std::cout << std::left;
  std::cout << std::setw(13) << "Vendor" << ": " << info.vendor << "\n";
  std::cout << std::setw(13) << "Architecture" << ": " << info.architecture
            << "\n";
  std::cout << std::setw(13) << "Device" << ": " << info.device << "\n";
  std::cout << std::setw(13) << "Description" << ": " << info.description
            << "\n";
  std::cout << std::setw(13) << "BackendType" << ": "
            << BackendTypeToString(info.backendType) << "\n";
  std::cout << std::setw(13) << "AdapterType" << ": "
            << AdapterTypeToString(info.adapterType) << "\n";
}

void DumpDevice(const wgpu::Device& device) {
  std::cout << "======" << std::endl;

#if 0
  auto toggles = dawn::native::GetTogglesUsed(device.Get());
  std::cout << "Toggles\n";
  for (auto toggle : toggles) {
    std::cout << "  " << toggle << "\n";
  }
#endif

  wgpu::SupportedFeatures features;
  device.GetFeatures(&features);
  std::cout << "Features\n";
  for (size_t i = 0; i < features.featureCount; ++i) {
    std::cout << "  " << features.features[i] << "\n";
  }
}

WGPUContext::WGPUContext(wgpu::BackendType backend_type,
                         wgpu::PowerPreference power_preference)
    : backend_type_(backend_type), power_preference_(power_preference) {}

WGPUContext::~WGPUContext() {}

wgpu::Instance WGPUContext::CreateDawnInstance() {
  wgpu::InstanceDescriptor instanceDescriptor{};
  instanceDescriptor.nextInChain = togglesChain_;
  instanceDescriptor.capabilities.timedWaitAnyEnable = true;

  wgpu::Instance instance = wgpu::CreateInstance(&instanceDescriptor);
  return instance;
}

wgpu::Adapter WGPUContext::RequireAdapter(
    wgpu::Instance instance,
    wgpu::BackendType backend_type,
    wgpu::PowerPreference power_preference) {
  wgpu::RequestAdapterOptions options = {};
  options.powerPreference = power_preference;
  options.nextInChain = togglesChain_;

  if (backend_type == wgpu::BackendType::Undefined) {
#ifdef _WIN32
    options.backendType = wgpu::BackendType::D3D12;
#else
    options.backendType = wgpu::BackendType::Vulkan;
#endif
  } else {
    options.backendType = backend_type;
  }

  wgpu::AdapterType adapterType = wgpu::AdapterType::Unknown;
  switch (adapterType) {
    case wgpu::AdapterType::CPU:
      options.forceFallbackAdapter = true;
      break;
    case wgpu::AdapterType::DiscreteGPU:
      options.powerPreference = wgpu::PowerPreference::HighPerformance;
      break;
    case wgpu::AdapterType::IntegratedGPU:
      options.powerPreference = wgpu::PowerPreference::LowPower;
      break;
    case wgpu::AdapterType::Unknown:
      break;
  }

  wgpu::Adapter result;
  wgpu::Future future = instance.RequestAdapter(
      &options, wgpu::CallbackMode::WaitAnyOnly,
      [&result](wgpu::RequestAdapterStatus status, wgpu::Adapter adapter,
                wgpu::StringView message) {
        if (status != wgpu::RequestAdapterStatus::Success) {
          std::cout << "Failed to get an adapter:" << message << std::endl;
          return;
        }
        result = std::move(adapter);
      });
  instance.WaitAny(future, UINT64_MAX);

  if (result == nullptr) {
    return wgpu::Adapter();
  }

  DumpAdapter(result);
  return result;
}

wgpu::Device WGPUContext::RequireDevice(
    wgpu::Instance instance,
    wgpu::Adapter adapter,
    std::vector<wgpu::FeatureName> requiredFeatures) {
  wgpu::DeviceDescriptor deviceDesc{};
  deviceDesc.nextInChain = togglesChain_;

  wgpu::Limits requiredLimits;
  adapter.GetLimits(&requiredLimits);
  deviceDesc.requiredLimits = &requiredLimits;

  deviceDesc.SetDeviceLostCallback(
      wgpu::CallbackMode::AllowSpontaneous,
      [](const wgpu::Device&, wgpu::DeviceLostReason reason,
         wgpu::StringView message) {
        if (reason != wgpu::DeviceLostReason::Destroyed) {
          std::cout << "DeviceLostCallback: " << message << std::endl;
        }
      });

  deviceDesc.SetUncapturedErrorCallback([](const wgpu::Device& device,
                                           wgpu::ErrorType type,
                                           wgpu::StringView message) {
    std::cout << "UncapturedErrorCallback: " << message << std::endl;
    CHECK(0);
  });
  deviceDesc.requiredFeatures = requiredFeatures.data();
  deviceDesc.requiredFeatureCount = requiredFeatures.size();

  wgpu::Device device;
  wgpu::Future future = adapter.RequestDevice(
      &deviceDesc, wgpu::CallbackMode::WaitAnyOnly,
      [&](wgpu::RequestDeviceStatus status, wgpu::Device deviceIn,
          wgpu::StringView message) {
        if (status != wgpu::RequestDeviceStatus::Success) {
          std::cout << "Failed to get a device:" << message << std::endl;
          CHECK(0);
        }
        device = deviceIn;
      });
  instance.WaitAny(future, UINT64_MAX);

  if (device == nullptr) {
    return wgpu::Device();
  }

  DumpDevice(device);
  return device;
}

bool has_feature(wgpu::SupportedFeatures& supported_features,
                 wgpu::FeatureName feature) {
  for (size_t i = 0; i < supported_features.featureCount; ++i) {
    if (supported_features.features[i] == feature) {
      return true;
    }
  }
  std::cout << "WARN " << feature << " is not supported by adapter"
            << std::endl;
  return false;
}

void WGPUContext::initialize_features(wgpu::Adapter adapter,
                                      std::vector<wgpu::FeatureName> features) {
  wgpu::SupportedFeatures supported_features;
  adapter.GetFeatures(&supported_features);

  requiredFeatures_.push_back(wgpu::FeatureName::ShaderF16);
  requiredFeatures_.push_back(wgpu::FeatureName::SubgroupsF16);
  requiredFeatures_.push_back(wgpu::FeatureName::Subgroups);
  requiredFeatures_.push_back(wgpu::FeatureName::TimestampQuery);
  for (auto& feature : features) {
    requiredFeatures_.push_back(feature);
  }

  requiredFeatures_.erase(
      std::remove_if(requiredFeatures_.begin(), requiredFeatures_.end(),
                     [&](wgpu::FeatureName feature) {
                       bool supports_feature =
                           has_feature(supported_features, feature);
                       return !supports_feature;
                     }),
      requiredFeatures_.end());

#if 0
  std::cout << "======" << std::endl;
  std::cout << "RequiredFeatures " << std::endl;
  for (auto& feature : requiredFeatures_) {
    std::cout << "  " << feature << std::endl;
  }
#endif
}

void WGPUContext::initialize_toggles(std::vector<std::string> toggles) {
  // enable toggles
  enableToggles_.push_back("allow_unsafe_apis");
  for (auto toggle : toggles) {
    enableToggles_.push_back(toggle);
  }

  enableToggleNames_.clear();
  std::cout << "======" << std::endl;
  std::cout << "Enable toggles " << std::endl;
  for (const std::string& toggle : enableToggles_) {
    std::cout << "  " << toggle << std::endl;
    enableToggleNames_.push_back(toggle.c_str());
  }

  // disable toggles
  disableToggles_.push_back("timestamp_quantization");
  disableToggleNames_.clear();
  std::cout << "======" << std::endl;
  std::cout << "Disable toggles " << std::endl;
  for (const std::string& toggle : disableToggles_) {
    std::cout << "  " << toggle << std::endl;
    disableToggleNames_.push_back(toggle.c_str());
  }

  // toggles descriptor
  toggles_ = {};
  toggles_.enabledToggles = enableToggleNames_.data();
  toggles_.enabledToggleCount = enableToggleNames_.size();
  toggles_.disabledToggles = disableToggleNames_.data();
  toggles_.disabledToggleCount = disableToggleNames_.size();

  togglesChain_ = &toggles_;
}

bool WGPUContext::initialize(std::vector<wgpu::FeatureName> features,
                             std::vector<std::string> toggles) {
  initialize_toggles(toggles);

  instance_ = CreateDawnInstance();
  CHECK(instance_ != nullptr);

  adapter_ = RequireAdapter(instance_, backend_type_, power_preference_);
  if (adapter_ == nullptr) {
    std::cout << "Failed to get backend type: " << backend_type_ << "\n";
    return false;
  }
  CHECK(adapter_ != nullptr);

  initialize_features(adapter_, features);
  device_ = RequireDevice(instance_, adapter_, requiredFeatures_);
  CHECK(device_);

  return true;
}

bool WGPUContext::supports_feature(wgpu::FeatureName feature) {
  wgpu::SupportedFeatures supported_features;
  adapter_.GetFeatures(&supported_features);

  return has_feature(supported_features, feature);
}

bool WGPUContext::is_vulkan_backend() {
  wgpu::AdapterInfo info = {};
  adapter_.GetInfo(&info);

  return (info.backendType == wgpu::BackendType::Vulkan);
}

bool WGPUContext::is_intel_vender() {
  wgpu::AdapterInfo info = {};
  adapter_.GetInfo(&info);

  std::string vender(info.vendor.data, info.vendor.length);
  return (vender == "intel");
}

bool WGPUContext::is_nvidia_vender() {
  wgpu::AdapterInfo info = {};
  adapter_.GetInfo(&info);

  std::string vender(info.vendor.data, info.vendor.length);
  return (vender == "nvidia");
}
