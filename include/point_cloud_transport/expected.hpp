#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief An implementation of the `std::expected` proposal.
 * 
 * `std::expected` should be used in functions that can either succeed and return a value, or fail and return an error.
 * \author Martin Pecka
 */

#include <point_cloud_transport/tl/expected.hpp>

namespace cras
{
  using ::tl::bad_expected_access;
  using ::tl::expected;
  using ::tl::in_place;
  using ::tl::make_unexpected;
  using ::tl::unexpect;
  using ::tl::unexpected;
}

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
struct is_cras_expected<::cras::expected<T, E>> : public ::std::true_type {};
}
