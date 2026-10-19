// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#ifndef POINT_CLOUD_TRANSPORT_PY__BORROWED_SERIALIZED_MESSAGE_HPP_
#define POINT_CLOUD_TRANSPORT_PY__BORROWED_SERIALIZED_MESSAGE_HPP_

#include <rcl/allocator.h>
#include <rmw/serialized_message.h>

#include <cstdint>
#include <utility>

#include <rclcpp/serialized_message.hpp>

#include "pybind11.hpp"

namespace point_cloud_transport_python
{

/// Exposes a Python bytes-like object as an rclcpp::SerializedMessage without copying it.
/**
 * The memory stays owned by Python: the message is detached from it again before
 * destruction, so rclcpp never tries to free it. Keep the instance alive for as long
 * as the SerializedMessage returned by get() is in use.
 */
class BorrowedSerializedMessage
{
public:
  explicit BorrowedSerializedMessage(const pybind11::buffer & source)
  {
    // PyBUF_SIMPLE requests a contiguous, byte-oriented view; exporters that cannot
    // provide one raise instead of handing out a strided buffer.
    if (PyObject_GetBuffer(source.ptr(), &view_, PyBUF_SIMPLE) != 0) {
      throw pybind11::error_already_set();
    }
    rcl_serialized_message_t raw = rmw_get_zero_initialized_serialized_message();
    raw.buffer = static_cast<uint8_t *>(view_.buf);
    raw.buffer_length = static_cast<size_t>(view_.len);
    raw.buffer_capacity = static_cast<size_t>(view_.len);
    raw.allocator = rcl_get_default_allocator();
    message_ = rclcpp::SerializedMessage(std::move(raw));
  }

  ~BorrowedSerializedMessage()
  {
    message_.release_rcl_serialized_message();  // hand the memory back to Python
    PyBuffer_Release(&view_);
  }

  BorrowedSerializedMessage(const BorrowedSerializedMessage &) = delete;
  BorrowedSerializedMessage & operator=(const BorrowedSerializedMessage &) = delete;

  const rclcpp::SerializedMessage & get() const {return message_;}

private:
  Py_buffer view_;
  rclcpp::SerializedMessage message_;
};

/// Copies the payload of a SerializedMessage into a new Python bytes object.
inline pybind11::bytes serialized_message_to_bytes(const rclcpp::SerializedMessage & message)
{
  const auto & raw = message.get_rcl_serialized_message();
  return pybind11::bytes(
    reinterpret_cast<const char *>(raw.buffer), static_cast<pybind11::ssize_t>(raw.buffer_length));
}

}  // namespace point_cloud_transport_python

#endif  // POINT_CLOUD_TRANSPORT_PY__BORROWED_SERIALIZED_MESSAGE_HPP_
