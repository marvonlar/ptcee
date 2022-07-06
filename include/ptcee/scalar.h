// Copyright (c) 2022 Norwegian Defence Research Establishment (FFI)

#pragma once

#include "Eigen/Core"
#include "gtsam/base/VectorSpace.h"

namespace ptc
{
struct Scalar : public Eigen::Matrix<double, 1, 1>
{
  enum { dimension = 1 };

  using Matrix<double, 1, 1>::Matrix;

  GTSAM_EXPORT void print(const std::string& = "") const
  {}

  GTSAM_EXPORT bool equals(const Scalar& s, const double tol = 1e-9) const
  {
    return std::abs(*this - s(0)) < tol;
  }

  const Eigen::Matrix<double, 1, 1>& vector() const
  {
    return *this;
  }

  operator double() const
  {
    return this->operator()(0);
  }

  inline static Scalar identity()
  {
    return Eigen::Matrix<double, 1, 1>::Identity();
  }
};

/*
struct PositiveScalar : public Eigen::Matrix<double, 1, 1>
{
  using Matrix<double, 1, 1>::Matrix;

  operator double() const
  {
    return this->operator()(0);
  }
};
*/
using PositiveScalar = Scalar;
}

template<>
struct gtsam::traits<ptc::Scalar>
: public gtsam::internal::VectorSpace<ptc::Scalar>
{};

/*
template<>
struct gtsam::traits<ptc::PositiveScalar>
{
  using TangentVector = Eigen::Vector<double, 1>;

  static void Print(const ptc::PositiveScalar&, const std::string& = "")
  {}

  [[nodiscard]] static bool Equals(
    const ptc::PositiveScalar& v1,
    const ptc::PositiveScalar& v2,
    const double tol = 1e-8
  )
  {
    return std::abs(v1(0) - v2(0)) < tol;
  }

  [[nodiscard]] static int GetDimension(const ptc::PositiveScalar&)
  {
    return 1;
  }

  [[nodiscard]] static TangentVector Local(
    const ptc::PositiveScalar& origin,
    const ptc::PositiveScalar& other,
    gtsam::OptionalJacobian<1, 1> H1 = boost::none,
    gtsam::OptionalJacobian<1, 1> H2 = boost::none
  )
  {
    if (H1)
    {
      *H1 = -Eigen::Matrix<double, 1, 1>::Identity();
    }

    if (H2)
    {
      *H2 = Eigen::Matrix<double, 1, 1>::Identity();
    }

    return other - origin;
  }

  [[nodiscard]] static ptc::PositiveScalar Retract(
    const ptc::PositiveScalar& origin,
    const TangentVector& v,
    gtsam::OptionalJacobian<1, 1> H1 = boost::none,
    gtsam::OptionalJacobian<1, 1> H2 = boost::none
  )
  {
    const auto sum = (origin + v).array().eval();

    if (H1)
    {
      *H1 = Eigen::Matrix<double, 1, 1>::Identity();
    }

    if (H2)
    {
      *H2 = Eigen::Matrix<double, 1, 1>::Identity();
    }

    return sum.abs().matrix();
  }
};
*/
