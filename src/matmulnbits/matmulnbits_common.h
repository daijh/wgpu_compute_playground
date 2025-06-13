
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

#ifndef __MATMULNBITS_COMMON_H__
#define __MATMULNBITS_COMMON_H__

#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <numeric>

#include "check.h"

static void log_result(std::vector<double> latencies) {
  std::vector<double> sorted_time = latencies;

  CHECK(latencies.size());
  double average = std::accumulate(latencies.begin(), latencies.end(), 0.0) /
                   latencies.size();

  size_t total = sorted_time.size();
  size_t n10 = static_cast<size_t>(total * 0.1);
  size_t n30 = static_cast<size_t>(total * 0.3);
  size_t n50 = static_cast<size_t>(total * 0.5);

  std::sort(sorted_time.begin(), sorted_time.end());

  std::cout << std::left << std::setprecision(3) << std::fixed;
  std::cout << "Total samples: " << sorted_time.size() << "\n";
  std::cout << "Min Latency: " << sorted_time[0] << " us\n";
  std::cout << "P10 Latency: " << sorted_time[n10] << " us\n";
  std::cout << "P30 Latency: " << sorted_time[n30] << " us\n";
  std::cout << "P50 Latency: " << sorted_time[n50] << " us\n";
  std::cout << "Max Latency: " << sorted_time[total - 1] << " us\n";

  std::cout << "Average Latency: " << average << " us" << std::endl;
}

#endif  // __MATMULNBITS_COMMON_H__
