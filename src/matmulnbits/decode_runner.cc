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
#include <random>
#include <sstream>

#include "check.h"
#include "decode_runner.h"
#include "matmulnbits_common.h"

DecodeRunner::DecodeRunner(WGPUContext* wgpu_context)
    : wgpu_context_(wgpu_context) {
  CHECK(wgpu_context);
  std::cout << __func__ << std::endl;

  CHECK(false);
}

DecodeRunner::~DecodeRunner() {}

bool DecodeRunner::initialize(uint32_t m, uint32_t k, uint32_t n) {
  M_ = m;
  K_ = k;
  N_ = n;

  CHECK(M_ == 1);
  CHECK(K_ % 32 == 0);
  CHECK(N_ % 32 == 0);

  compute_runner_ = std::make_shared<BaseComputeRunner>(wgpu_context_);
  compute_runner_->initialize();

  if (!configure()) {
    return false;
  }

  return true;
}

bool DecodeRunner::configure() {
  std::string source = generate_shader();
  if (source.empty()) {
    return false;
  }

  compute_runner_->set_shader(source, "main");
  std::cout << "======\n";
  std::cout << source << std::endl;

  // After `generate_shader()` decides `tile_m_` and `tile_n_`.
  create_buffers();

  compute_runner_->set_dispatch(dispatch_size_[0], dispatch_size_[1],
                                dispatch_size_[2]);
  compute_runner_->initialize_pipeline();

  return true;
}

void DecodeRunner::create_buffers() {
  unsigned int kSeed = 0xDEADBEEF;  // Fixed seed for consistent results
  std::mt19937 gen(kSeed);  // Initialize the generator with the fixed seed
}

std::string DecodeRunner::generate_shader() {
  tile_m_ = 64;
  tile_n_ = 64;

  workgroup_size_.resize(3);
  workgroup_size_ = {256, 1, 1};

  std::cout << "======\n";
  std::cout << "Workgroup Size: " << workgroup_size_[0] << "x"
            << workgroup_size_[1] << "x" << workgroup_size_[2] << std::endl;

  dispatch_size_.resize(3);
  dispatch_size_[0] = CEIL_DIVIDE(M_, tile_m_) * CEIL_DIVIDE(N_, tile_n_);
  dispatch_size_[1] = 1;
  dispatch_size_[2] = 1;

  std::cout << "Dispatch Size: " << dispatch_size_[0] << "x"
            << dispatch_size_[1] << "x" << dispatch_size_[2] << std::endl;

  std::stringstream code;

  // Reset the stringstream:
  code.str("");  // Clear the string buffer
  code.clear();  // Clear error flags

  code << R"(
)";

  return code.str();
}

void DecodeRunner::compute(uint32_t loop) {
  std::cout << "======\n";
  std::cout << "GPU Compute" << std::endl;

  std::vector<double> latency_list;
  std::vector<uint8_t> readback_output;
  for (uint32_t i = 0; i < loop; ++i) {
    std::pair<double, double> latency = compute_runner_->run();
    double gpu_latency = latency.second;
    latency_list.push_back(gpu_latency);

    readback_output = compute_runner_->read_buffer(output_y_buffer_);

    std::cout << "\rProgress: " << std::setw(2) << 100.0f * i / loop << "%"
              << std::flush;
  }
  std::cout << std::endl;

  std::cout << "======\n";
  std::cout << "output:" << std::endl;
  compute_runner_->log_vector<Float16>(readback_output.data(),
                                       readback_output.size(), 128);

  std::cout << "======" << std::endl;
  log_result(latency_list);
}

void DecodeRunner::verify() {}
