// Copyright 2026 ASTRO
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DISTRIBUTED_SATELLITE_SIM__CIRCULAR_BUFFER_HPP_
#define DISTRIBUTED_SATELLITE_SIM__CIRCULAR_BUFFER_HPP_

#include <cstddef>
#include <deque>
#include <vector>

namespace distributed_satellite_sim
{

template<typename T>
class CircularBuffer
{
public:
  explicit CircularBuffer(std::size_t capacity)
  : capacity_(capacity) {}

  void push(T item)
  {
    if (buf_.size() >= capacity_) {
      buf_.pop_front();
    }
    buf_.push_back(std::move(item));
  }

  // Returns oldest-to-newest. limit=0 means return all retained entries.
  std::vector<T> get_recent(std::size_t limit) const
  {
    if (limit == 0 || limit >= buf_.size()) {
      return {buf_.begin(), buf_.end()};
    }
    return {buf_.end() - static_cast<std::ptrdiff_t>(limit), buf_.end()};
  }

  std::size_t size() const {return buf_.size();}
  std::size_t capacity() const {return capacity_;}

private:
  std::deque<T> buf_;
  std::size_t capacity_;
};

}  // namespace distributed_satellite_sim

#endif  // DISTRIBUTED_SATELLITE_SIM__CIRCULAR_BUFFER_HPP_
