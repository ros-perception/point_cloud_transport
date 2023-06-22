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
// #pragma once

#ifndef POINT_CLOUD_TRANSPORT__C_API_H_
#define POINT_CLOUD_TRANSPORT__C_API_H_

// /**
//  * \file
//  * \brief Support definitions for declaration of a C API of modules.
//  * \author Martin Pecka
//  */

// #include <string>
// #include <vector>

// namespace cras
// {

// /**
// * \brief Allocator function that should allocate a buffer of the given size on the caller
// side and return a pointer
//  *        to it.
//  *
//  * This is used throughout the C API to ease returning strings and byte buffers of dynamic
// size to the caller. Instead
//  * of taking a writable `char*` argument or returning a `new`-allocated `const char*`,
// the function takes the allocator
//  * as the argument. When it is about to return to the caller, it uses the allocator to get a
// caller-side-managed buffer
//  * of the correct size and writes the string/byte buffer to this buffer.
//  *
//  * cras_py_common provides the `BytesAllocator` and `StringAllocator` classes that can be
// passed as this allocator
//  * argument. Once the ctypes function call finishes, the caller can access `allocator.value`
// to get the string or
//  * buffer returned by the function.
//  */
// typedef void* (*allocator_t)(size_t);

// /**
//  * \brief Allocate enough bytes using the given allocator and copy the given string into
// the buffer (including null
//  *        termination byte).
//  * \param[in] allocator The allocator to use.
//  * \param[in] string The zero-terminated string to copy into the allocated buffer.
//  * \param[in] length Length of the string to copy including the null termination character.
//  * \return Pointer to the allocated buffer.
//  */
// char* outputString(allocator_t allocator, const char* string, size_t length);

// /**
//  * \brief Allocate enough bytes using the given allocator and copy the given string into
// the buffer (including null
//  *        termination byte).
//  * \param[in] allocator The allocator to use.
//  * \param[in] string The string to copy into the allocated buffer.
//  * \return Pointer to the allocated buffer.
//  */
// char* outputString(allocator_t allocator, const std::string& string);

// /**
//  * \brief Allocate enough bytes using the given allocator and copy the given bytes into
// the buffer.
//  * \param[in] allocator The allocator to use.
//  * \param[in] bytes The bytes to copy into the allocated buffer.
//  * \param[in] length Length of `bytes`.
//  * \return Pointer to the allocated buffer.
//  */
// uint8_t* outputByteBuffer(allocator_t allocator, const uint8_t* bytes, size_t length);

// /**
//  * \brief Allocate enough bytes using the given allocator and copy the given bytes into
// the buffer.
//  * \param[in] allocator The allocator to use.
//  * \param[in] bytes The bytes to copy into the allocated buffer.
//  * \return Pointer to the allocated buffer.
//  */
// uint8_t* outputByteBuffer(allocator_t allocator, const std::vector<uint8_t>& bytes);


// }

#endif  // POINT_CLOUD_TRANSPORT__C_API_H_
