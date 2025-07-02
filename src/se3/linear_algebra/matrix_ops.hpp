// Copyright (c) 2025, NVIDIA CORPORATION.  All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto.  Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.


#pragma once

#include "matrices.hpp"

namespace se3 {

template <Mat3 M>
M transpose(const M& m);

template <Mat3 M>
M inverse(const M& m);

template <Mat4 M>
M transpose(const M& m);

template <Mat4 M>
M inverse(const M& m);

}