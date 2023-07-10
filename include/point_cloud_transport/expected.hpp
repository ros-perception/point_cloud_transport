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

#ifndef POINT_CLOUD_TRANSPORT__EXPECTED_HPP_
#define POINT_CLOUD_TRANSPORT__EXPECTED_HPP_


// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief An implementation of the `std::expected` proposal.
 *
 * `std::expected` should be used in functions that can either succeed and return a value, or fail and return an error.
 * \author Martin Pecka
 */

#include <tl/expected.hpp>

namespace cras
{
using ::tl::bad_expected_access;
using ::tl::expected;
using ::tl::in_place;
using ::tl::make_unexpected;
using ::tl::unexpect;
using ::tl::unexpected;
}  // namespace cras

#include <type_traits>

namespace cras
{
/**
 * \brief Type trait determining whether type T is cras::expected or not.
 * \tparam T The type to test.
 */
template<typename T>
struct is_cras_expected : public ::std::false_type {};

/**
 * \brief Type trait determining whether type T is std::optional or not.
 * \tparam T Type of the expected value.
 * \tparam E Type of the expected error.
 */
template<typename T, typename E>
struct is_cras_expected<::cras::expected<T, E>>: public ::std::true_type {};
}  // namespace cras
#endif  // POINT_CLOUD_TRANSPORT__EXPECTED_HPP_
