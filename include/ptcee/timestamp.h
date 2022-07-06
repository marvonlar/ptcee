// Copyright (c) 2022 Norwegian Defence Research Establishment (FFI)

#pragma once

namespace ptc
{
using Timestamp = double;

struct NoisyTimestamp
{
  Timestamp t;
  double sigma_t;
};

inline bool operator<(const NoisyTimestamp& lhs, const NoisyTimestamp& rhs)
{
  return lhs.t < rhs.t;
}
}
