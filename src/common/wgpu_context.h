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

#ifndef __WGPU_CONTEXT_H__
#define __WGPU_CONTEXT_H__

#include <dawn/dawn_proc.h>
#include <dawn/native/DawnNative.h>
#include <dawn/webgpu_cpp_print.h>
#include <webgpu/webgpu_cpp.h>

class WGPUContext {
 public:
  WGPUContext(wgpu::BackendType backend_type,
              wgpu::PowerPreference power_preference =
                  wgpu::PowerPreference::Undefined);
  ~WGPUContext();

  bool initialize(std::vector<wgpu::FeatureName> features = {},
                  std::vector<std::string> toggles = {});

  wgpu::Instance instance() { return instance_; }
  wgpu::Adapter adapter() { return adapter_; }
  wgpu::Device device() { return device_; }

  bool supports_feature(wgpu::FeatureName feature);

  bool is_vulkan_backend();

  bool is_intel_vender();
  bool is_nvidia_vender();

 private:
  void initialize_toggles(std::vector<std::string> toggles);
  void initialize_features(wgpu::Adapter adapter,
                           std::vector<wgpu::FeatureName> features);

  wgpu::Instance CreateDawnInstance();

  wgpu::Adapter RequireAdapter(
      wgpu::Instance instance,
      wgpu::BackendType backend_type = wgpu::BackendType::Undefined,
      wgpu::PowerPreference power_preference =
          wgpu::PowerPreference::Undefined);

  wgpu::Device RequireDevice(
      wgpu::Instance instance,
      wgpu::Adapter adapter,
      std::vector<wgpu::FeatureName> requiredFeatures = {});

  // Features
  std::vector<wgpu::FeatureName> requiredFeatures_;

  // Toggles
  std::vector<std::string> enableToggles_;
  std::vector<const char*> enableToggleNames_;

  std::vector<std::string> disableToggles_;
  std::vector<const char*> disableToggleNames_;

  wgpu::DawnTogglesDescriptor toggles_ = {};
  wgpu::ChainedStruct* togglesChain_ = nullptr;

  wgpu::BackendType backend_type_ = wgpu::BackendType::Undefined;
  wgpu::PowerPreference power_preference_ = wgpu::PowerPreference::Undefined;

  wgpu::Instance instance_ = nullptr;
  wgpu::Adapter adapter_ = nullptr;
  wgpu::Device device_ = nullptr;
};
#endif  // __WGPU_CONTEXT_H__
