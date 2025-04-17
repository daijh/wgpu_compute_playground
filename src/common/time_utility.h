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

#ifndef __TIME_UTILITY_H__
#define __TIME_UTILITY_H__

#include <chrono>
#include <optional>

#include "check.h"

class TimeMeasurer {
 public:
  TimeMeasurer() {}
  ~TimeMeasurer() {}

  void begin() {
    end_.reset();

    begin_.emplace(std::chrono::high_resolution_clock::now());
  }

  void end() {
    CHECK(begin_);

    end_.emplace(std::chrono::high_resolution_clock::now());
  }

  uint64_t elapsed_ms() {
    CHECK(begin_);
    CHECK(end_);

    std::chrono::milliseconds ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(*end_ - *begin_);

    return ms.count();
  }

  uint64_t elapsed_us() {
    CHECK(begin_);
    CHECK(end_);

    std::chrono::microseconds us =
        std::chrono::duration_cast<std::chrono::microseconds>(*end_ - *begin_);

    return us.count();
  }

  uint64_t elapsed_ns() {
    CHECK(begin_);
    CHECK(end_);

    std::chrono::nanoseconds ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(*end_ - *begin_);

    return ns.count();
  }

 private:
  std::optional<std::chrono::time_point<std::chrono::high_resolution_clock>>
      begin_;
  std::optional<std::chrono::time_point<std::chrono::high_resolution_clock>>
      end_;
};
#endif  // __TIME_UTILITY_H__
