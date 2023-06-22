/*
 * Copyright (c) 2023, Czech Technical University in Prague
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *    * Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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
