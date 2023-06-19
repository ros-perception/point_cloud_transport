// // SPDX-License-Identifier: BSD-3-Clause
// // SPDX-FileCopyrightText: Czech Technical University in Prague

// /**
//  * \file
//  * \brief Support definitions for declaration of a C API of modules
//  * \author Martin Pecka
//  */

// #include <cstring>
// #include <string>
// #include <vector>

// #include <cras_cpp_common/c_api.h>

// namespace cras
// {

// char* outputString(cras::allocator_t allocator, const char* string, size_t length)
// {
//   const auto buffer = static_cast<char*>(allocator(length));
//   strncpy(buffer, string, length);
//   return buffer;
// }

// char* outputString(allocator_t allocator, const std::string& string)
// {
//   return outputString(allocator, string.c_str(), string.size() + 1);
// }

// uint8_t* outputByteBuffer(allocator_t allocator, const uint8_t* bytes, size_t length)
// {
//   const auto buffer = allocator(length);
//   return static_cast<uint8_t*>(memcpy(buffer, bytes, length));
// }

// uint8_t* outputByteBuffer(allocator_t allocator, const std::vector<uint8_t>& bytes)
// {
//   return outputByteBuffer(allocator, bytes.data(), bytes.size());
// }

// }
