/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/dart.hpp>
#include <eigen_geometry_pybind.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace dart {
namespace python {

void BallJoint(pybind11::module& m)
{
  ::pybind11::class_<dart::dynamics::BallJoint::Properties>(
      m, "BallJointProperties")
      .def(::pybind11::init<>())
      .def(
          ::pybind11::init<const dart::dynamics::GenericJoint<
              dart::math::SO3Space>::Properties&>(),
          ::pybind11::arg("properties"));

  ::pybind11::class_<
      dart::dynamics::BallJoint,
      dart::dynamics::GenericJoint<dart::math::SO3Space>>(m, "BallJoint")
      .def(
          "getType",
          +[](const dart::dynamics::BallJoint* self) -> const std::string& {
            return self->getType();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "isCyclic",
          +[](const dart::dynamics::BallJoint* self,
              std::size_t _index) -> bool { return self->isCyclic(_index); },
          ::pybind11::arg("index"))
      .def(
          "getBallJointProperties",
          +[](const dart::dynamics::BallJoint* self)
              -> dart::dynamics::BallJoint::Properties {
            return self->getBallJointProperties();
          })
      .def(
          "getRelativeJacobianStatic",
          +[](const dart::dynamics::BallJoint* self,
              const Eigen::Vector3d& _positions)
              -> Eigen::Matrix<double, 6, 3> {
            return self->getRelativeJacobianStatic(_positions);
          },
          ::pybind11::arg("positions"))
      .def(
          "getPositionDifferencesStatic",
          +[](const dart::dynamics::BallJoint* self,
              const Eigen::Vector3d& _q2,
              const Eigen::Vector3d& _q1) -> Eigen::Vector3d {
            return self->getPositionDifferencesStatic(_q2, _q1);
          },
          ::pybind11::arg("q2"),
          ::pybind11::arg("q1"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::BallJoint::getStaticType();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def_static(
          "convertToTransform",
          +[](const Eigen::Vector3d& _positions) -> Eigen::Isometry3d {
            return dart::dynamics::BallJoint::convertToTransform(_positions);
          },
          ::pybind11::arg("positions"))
      .def_static(
          "convertToRotation",
          +[](const Eigen::Vector3d& _positions) -> Eigen::Matrix3d {
            return dart::dynamics::BallJoint::convertToRotation(_positions);
          },
          ::pybind11::arg("positions"));
}

} // namespace python
} // namespace dart
