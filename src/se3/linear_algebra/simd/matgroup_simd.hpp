// Copyright (c) 2025, NVIDIA CORPORATION.  All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto.  Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.


#pragma once

#include "vectors_simd.hpp"

#include "se3/linear_algebra/type_traits.hpp"

namespace se3 {

struct SIMD {};

template <std::floating_point T>
struct MatGroup<SIMD, T> {
  using Vec3 = simd::Vector3<T>;
  using Vec4 = generic::Vector4<T>;
};

}