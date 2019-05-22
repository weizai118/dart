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

void GenericJoint(pybind11::module& m)
{
  ::pybind11::class_<
      dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>
      // dart::common::EmbedStateAndPropertiesOnTopOf<
      //     dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>,
      //     dart::dynamics::detail::GenericJointState<
      //         dart::math::RealVectorSpace<1>>,
      //     dart::dynamics::detail::GenericJointUniqueProperties<
      //         dart::math::RealVectorSpace<1>>,
      //     dart::dynamics::Joint>,
      >(m, "GenericJoint_R1")
      .def(
          "hasGenericJointAspect",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<1>>* self) -> bool {
            return self->hasGenericJointAspect();
          })
      .def(
          "setGenericJointAspect",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>::Aspect* aspect) {
            self->setGenericJointAspect(aspect);
          },
          ::pybind11::arg("aspect"))
      .def(
          "removeGenericJointAspect",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self) { self->removeGenericJointAspect(); })
      .def(
          "releaseGenericJointAspect",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self)
              -> std::unique_ptr<dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>::Aspect> {
            return self->releaseGenericJointAspect();
          })
      .def(
          "setProperties",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>::Properties& properties) {
            self->setProperties(properties);
          },
          ::pybind11::arg("properties"))
      .def(
          "setProperties",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>::UniqueProperties&
                  properties) { self->setProperties(properties); },
          ::pybind11::arg("properties"))
      .def(
          "setAspectState",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>::AspectState& state) {
            self->setAspectState(state);
          },
          ::pybind11::arg("state"))
      .def(
          "setAspectProperties",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>::AspectProperties&
                  properties) { self->setAspectProperties(properties); },
          ::pybind11::arg("properties"))
      .def(
          "getGenericJointProperties",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<1>>* self)
              -> dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>::Properties {
            return self->getGenericJointProperties();
          })
      .def(
          "copy",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>::ThisClass& otherJoint) {
            self->copy(otherJoint);
          },
          ::pybind11::arg("otherJoint"))
      .def(
          "copy",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>::ThisClass* otherJoint) {
            self->copy(otherJoint);
          },
          ::pybind11::arg("otherJoint"))
      .def(
          "getNumDofs",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<1>>* self) -> std::size_t {
            return self->getNumDofs();
          })
      .def(
          "setDofName",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              size_t index,
              const std::string& name) -> const std::string& {
            return self->setDofName(index, name);
          },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("index"),
          ::pybind11::arg("name"))
      .def(
          "setDofName",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              size_t index,
              const std::string& name,
              bool preserveName) -> const std::string& {
            return self->setDofName(index, name, preserveName);
          },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("index"),
          ::pybind11::arg("name"),
          ::pybind11::arg("preserveName"))
      .def(
          "preserveDofName",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              size_t index,
              bool preserve) { self->preserveDofName(index, preserve); },
          ::pybind11::arg("index"),
          ::pybind11::arg("preserve"))
      .def(
          "isDofNamePreserved",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              size_t index) -> bool { return self->isDofNamePreserved(index); },
          ::pybind11::arg("index"))
      .def(
          "getDofName",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              size_t index) -> const std::string& {
            return self->getDofName(index);
          },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("index"))
      .def(
          "getIndexInSkeleton",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              size_t index) -> size_t {
            return self->getIndexInSkeleton(index);
          },
          ::pybind11::arg("index"))
      .def(
          "getIndexInTree",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              size_t index) -> size_t { return self->getIndexInTree(index); },
          ::pybind11::arg("index"))
      .def(
          "setCommand",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              std::size_t index,
              double command) { self->setCommand(index, command); },
          ::pybind11::arg("index"),
          ::pybind11::arg("command"))
      .def(
          "getCommand",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              std::size_t index) -> double { return self->getCommand(index); },
          ::pybind11::arg("index"))
      .def(
          "setCommands",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              const Eigen::VectorXd& commands) { self->setCommands(commands); },
          ::pybind11::arg("commands"))
      .def(
          "getCommands",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<1>>* self) -> Eigen::VectorXd {
            return self->getCommands();
          })
      .def(
          "resetCommands",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self) { self->resetCommands(); })
      .def(
          "setPosition",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              std::size_t index,
              double position) { self->setPosition(index, position); },
          ::pybind11::arg("index"),
          ::pybind11::arg("position"))
      .def(
          "getPosition",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              std::size_t index) -> double { return self->getPosition(index); },
          ::pybind11::arg("index"))
      .def(
          "setPositions",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              const Eigen::VectorXd& positions) {
            self->setPositions(positions);
          },
          ::pybind11::arg("positions"))
      .def(
          "getPositions",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<1>>* self) -> Eigen::VectorXd {
            return self->getPositions();
          })
      .def(
          "setPositionLowerLimit",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              size_t index,
              double position) {
            self->setPositionLowerLimit(index, position);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("position"))
      .def(
          "getPositionLowerLimit",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              std::size_t index) -> double {
            return self->getPositionLowerLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setPositionLowerLimits",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              const Eigen::VectorXd& lowerLimits) {
            self->setPositionLowerLimits(lowerLimits);
          },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getPositionLowerLimits",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<1>>* self) -> Eigen::VectorXd {
            return self->getPositionLowerLimits();
          })
      .def(
          "setPositionUpperLimit",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              size_t index,
              double position) {
            self->setPositionUpperLimit(index, position);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("position"))
      .def(
          "getPositionUpperLimit",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              std::size_t index) -> double {
            return self->getPositionUpperLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setPositionUpperLimits",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              const Eigen::VectorXd& upperLimits) {
            self->setPositionUpperLimits(upperLimits);
          },
          ::pybind11::arg("upperLimits"))
      .def(
          "getPositionUpperLimits",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<1>>* self) -> Eigen::VectorXd {
            return self->getPositionUpperLimits();
          })
      .def(
          "hasPositionLimit",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              std::size_t index) -> bool {
            return self->hasPositionLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "resetPosition",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              std::size_t index) { self->resetPosition(index); },
          ::pybind11::arg("index"))
      .def(
          "resetPositions",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self) { self->resetPositions(); })
      .def(
          "setInitialPosition",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              size_t index,
              double initial) { self->setInitialPosition(index, initial); },
          ::pybind11::arg("index"),
          ::pybind11::arg("initial"))
      .def(
          "getInitialPosition",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              std::size_t index) -> double {
            return self->getInitialPosition(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setInitialPositions",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              const Eigen::VectorXd& initial) {
            self->setInitialPositions(initial);
          },
          ::pybind11::arg("initial"))
      .def(
          "getInitialPositions",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<1>>* self) -> Eigen::VectorXd {
            return self->getInitialPositions();
          })
      .def(
          "setPositionsStatic",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>::Vector& positions) {
            self->setPositionsStatic(positions);
          },
          ::pybind11::arg("positions"))
      .def(
          "setVelocitiesStatic",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>::Vector& velocities) {
            self->setVelocitiesStatic(velocities);
          },
          ::pybind11::arg("velocities"))
      .def(
          "setAccelerationsStatic",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>::Vector& accels) {
            self->setAccelerationsStatic(accels);
          },
          ::pybind11::arg("accels"))
      .def(
          "setVelocity",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              std::size_t index,
              double velocity) { self->setVelocity(index, velocity); },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocity"))
      .def(
          "getVelocity",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              std::size_t index) -> double { return self->getVelocity(index); },
          ::pybind11::arg("index"))
      .def(
          "setVelocities",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              const Eigen::VectorXd& velocities) {
            self->setVelocities(velocities);
          },
          ::pybind11::arg("velocities"))
      .def(
          "getVelocities",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<1>>* self) -> Eigen::VectorXd {
            return self->getVelocities();
          })
      .def(
          "setVelocityLowerLimit",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              size_t index,
              double velocity) {
            self->setVelocityLowerLimit(index, velocity);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocity"))
      .def(
          "getVelocityLowerLimit",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              std::size_t index) -> double {
            return self->getVelocityLowerLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setVelocityLowerLimits",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              const Eigen::VectorXd& lowerLimits) {
            self->setVelocityLowerLimits(lowerLimits);
          },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getVelocityLowerLimits",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<1>>* self) -> Eigen::VectorXd {
            return self->getVelocityLowerLimits();
          })
      .def(
          "setVelocityUpperLimit",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              size_t index,
              double velocity) {
            self->setVelocityUpperLimit(index, velocity);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocity"))
      .def(
          "getVelocityUpperLimit",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              std::size_t index) -> double {
            return self->getVelocityUpperLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setVelocityUpperLimits",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              const Eigen::VectorXd& upperLimits) {
            self->setVelocityUpperLimits(upperLimits);
          },
          ::pybind11::arg("upperLimits"))
      .def(
          "getVelocityUpperLimits",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<1>>* self) -> Eigen::VectorXd {
            return self->getVelocityUpperLimits();
          })
      .def(
          "resetVelocity",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              std::size_t index) { self->resetVelocity(index); },
          ::pybind11::arg("index"))
      .def(
          "resetVelocities",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self) { self->resetVelocities(); })
      .def(
          "setInitialVelocity",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              size_t index,
              double initial) { self->setInitialVelocity(index, initial); },
          ::pybind11::arg("index"),
          ::pybind11::arg("initial"))
      .def(
          "getInitialVelocity",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              std::size_t index) -> double {
            return self->getInitialVelocity(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setInitialVelocities",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              const Eigen::VectorXd& initial) {
            self->setInitialVelocities(initial);
          },
          ::pybind11::arg("initial"))
      .def(
          "getInitialVelocities",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<1>>* self) -> Eigen::VectorXd {
            return self->getInitialVelocities();
          })
      .def(
          "setAcceleration",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              std::size_t index,
              double acceleration) {
            self->setAcceleration(index, acceleration);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("acceleration"))
      .def(
          "getAcceleration",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              std::size_t index) -> double {
            return self->getAcceleration(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setAccelerations",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              const Eigen::VectorXd& accelerations) {
            self->setAccelerations(accelerations);
          },
          ::pybind11::arg("accelerations"))
      .def(
          "getAccelerations",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<1>>* self) -> Eigen::VectorXd {
            return self->getAccelerations();
          })
      .def(
          "setAccelerationLowerLimit",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              size_t index,
              double acceleration) {
            self->setAccelerationLowerLimit(index, acceleration);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("acceleration"))
      .def(
          "getAccelerationLowerLimit",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              std::size_t index) -> double {
            return self->getAccelerationLowerLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setAccelerationLowerLimits",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              const Eigen::VectorXd& lowerLimits) {
            self->setAccelerationLowerLimits(lowerLimits);
          },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getAccelerationLowerLimits",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<1>>* self) -> Eigen::VectorXd {
            return self->getAccelerationLowerLimits();
          })
      .def(
          "setAccelerationUpperLimit",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              size_t index,
              double acceleration) {
            self->setAccelerationUpperLimit(index, acceleration);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("acceleration"))
      .def(
          "getAccelerationUpperLimit",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              std::size_t index) -> double {
            return self->getAccelerationUpperLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setAccelerationUpperLimits",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              const Eigen::VectorXd& upperLimits) {
            self->setAccelerationUpperLimits(upperLimits);
          },
          ::pybind11::arg("upperLimits"))
      .def(
          "getAccelerationUpperLimits",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<1>>* self) -> Eigen::VectorXd {
            return self->getAccelerationUpperLimits();
          })
      .def(
          "resetAccelerations",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self) { self->resetAccelerations(); })
      .def(
          "setForce",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              std::size_t index,
              double force) { self->setForce(index, force); },
          ::pybind11::arg("index"),
          ::pybind11::arg("force"))
      .def(
          "getForce",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              std::size_t index) -> double { return self->getForce(index); },
          ::pybind11::arg("index"))
      .def(
          "setForces",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              const Eigen::VectorXd& forces) { self->setForces(forces); },
          ::pybind11::arg("forces"))
      .def(
          "getForces",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<1>>* self) -> Eigen::VectorXd {
            return self->getForces();
          })
      .def(
          "setForceLowerLimit",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              size_t index,
              double force) { self->setForceLowerLimit(index, force); },
          ::pybind11::arg("index"),
          ::pybind11::arg("force"))
      .def(
          "getForceLowerLimit",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              std::size_t index) -> double {
            return self->getForceLowerLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setForceLowerLimits",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              const Eigen::VectorXd& lowerLimits) {
            self->setForceLowerLimits(lowerLimits);
          },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getForceLowerLimits",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<1>>* self) -> Eigen::VectorXd {
            return self->getForceLowerLimits();
          })
      .def(
          "setForceUpperLimit",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              size_t index,
              double force) { self->setForceUpperLimit(index, force); },
          ::pybind11::arg("index"),
          ::pybind11::arg("force"))
      .def(
          "getForceUpperLimit",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              size_t index) -> double {
            return self->getForceUpperLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setForceUpperLimits",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              const Eigen::VectorXd& upperLimits) {
            self->setForceUpperLimits(upperLimits);
          },
          ::pybind11::arg("upperLimits"))
      .def(
          "getForceUpperLimits",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<1>>* self) -> Eigen::VectorXd {
            return self->getForceUpperLimits();
          })
      .def(
          "resetForces",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self) { self->resetForces(); })
      .def(
          "setVelocityChange",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              std::size_t index,
              double velocityChange) {
            self->setVelocityChange(index, velocityChange);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocityChange"))
      .def(
          "getVelocityChange",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              std::size_t index) -> double {
            return self->getVelocityChange(index);
          },
          ::pybind11::arg("index"))
      .def(
          "resetVelocityChanges",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self) { self->resetVelocityChanges(); })
      .def(
          "setConstraintImpulse",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              std::size_t index,
              double impulse) { self->setConstraintImpulse(index, impulse); },
          ::pybind11::arg("index"),
          ::pybind11::arg("impulse"))
      .def(
          "getConstraintImpulse",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              std::size_t index) -> double {
            return self->getConstraintImpulse(index);
          },
          ::pybind11::arg("index"))
      .def(
          "resetConstraintImpulses",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self) { self->resetConstraintImpulses(); })
      .def(
          "integratePositions",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              double dt) { self->integratePositions(dt); },
          ::pybind11::arg("dt"))
      .def(
          "integrateVelocities",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              double dt) { self->integrateVelocities(dt); },
          ::pybind11::arg("dt"))
      .def(
          "getPositionDifferences",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              const Eigen::VectorXd& q2,
              const Eigen::VectorXd& q1) -> Eigen::VectorXd {
            return self->getPositionDifferences(q2, q1);
          },
          ::pybind11::arg("q2"),
          ::pybind11::arg("q1"))
      .def(
          "getPositionDifferencesStatic",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>::Vector& q2,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>::Vector& q1)
              -> dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>::Vector {
            return self->getPositionDifferencesStatic(q2, q1);
          },
          ::pybind11::arg("q2"),
          ::pybind11::arg("q1"))
      .def(
          "setSpringStiffness",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              size_t index,
              double k) { self->setSpringStiffness(index, k); },
          ::pybind11::arg("index"),
          ::pybind11::arg("k"))
      .def(
          "getSpringStiffness",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              std::size_t index) -> double {
            return self->getSpringStiffness(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setRestPosition",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              size_t index,
              double q0) { self->setRestPosition(index, q0); },
          ::pybind11::arg("index"),
          ::pybind11::arg("q0"))
      .def(
          "getRestPosition",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              std::size_t index) -> double {
            return self->getRestPosition(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setDampingCoefficient",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              size_t index,
              double d) { self->setDampingCoefficient(index, d); },
          ::pybind11::arg("index"),
          ::pybind11::arg("d"))
      .def(
          "getDampingCoefficient",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              std::size_t index) -> double {
            return self->getDampingCoefficient(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setCoulombFriction",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<1>>*
                  self,
              size_t index,
              double friction) { self->setCoulombFriction(index, friction); },
          ::pybind11::arg("index"),
          ::pybind11::arg("friction"))
      .def(
          "getCoulombFriction",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              std::size_t index) -> double {
            return self->getCoulombFriction(index);
          },
          ::pybind11::arg("index"))
      .def(
          "computePotentialEnergy",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<1>>* self) -> double {
            return self->computePotentialEnergy();
          })
      .def(
          "getBodyConstraintWrench",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<1>>* self) -> Eigen::Vector6d {
            return self->getBodyConstraintWrench();
          })
      .def(
          "getRelativeJacobian",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<1>>* self)
              -> const dart::math::Jacobian {
            return self->getRelativeJacobian();
          })
      .def(
          "getRelativeJacobian",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              const Eigen::VectorXd& _positions) -> dart::math::Jacobian {
            return self->getRelativeJacobian(_positions);
          },
          ::pybind11::arg("_positions"))
      .def(
          "getRelativeJacobianStatic",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>* self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>::Vector& positions)
              -> dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<1>>::JacobianMatrix {
            return self->getRelativeJacobianStatic(positions);
          },
          ::pybind11::arg("positions"))
      .def(
          "getRelativeJacobianTimeDeriv",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<1>>* self)
              -> const dart::math::Jacobian {
            return self->getRelativeJacobianTimeDeriv();
          })
      .def_readonly_static(
          "NumDofs",
          &dart::dynamics::GenericJoint<
              dart::math::RealVectorSpace<1>>::NumDofs);

  ::pybind11::class_<
      dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>
      // dart::common::EmbedStateAndPropertiesOnTopOf<
      //     dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>,
      //     dart::dynamics::detail::GenericJointState<
      //         dart::math::RealVectorSpace<2>>,
      //     dart::dynamics::detail::GenericJointUniqueProperties<
      //         dart::math::RealVectorSpace<2>>,
      //     dart::dynamics::Joint>,
      >(m, "GenericJoint_R2")
      .def(
          "hasGenericJointAspect",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<2>>* self) -> bool {
            return self->hasGenericJointAspect();
          })
      .def(
          "setGenericJointAspect",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>::Aspect* aspect) {
            self->setGenericJointAspect(aspect);
          },
          ::pybind11::arg("aspect"))
      .def(
          "removeGenericJointAspect",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self) { self->removeGenericJointAspect(); })
      .def(
          "releaseGenericJointAspect",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self)
              -> std::unique_ptr<dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>::Aspect> {
            return self->releaseGenericJointAspect();
          })
      .def(
          "setProperties",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>::Properties& properties) {
            self->setProperties(properties);
          },
          ::pybind11::arg("properties"))
      .def(
          "setProperties",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>::UniqueProperties&
                  properties) { self->setProperties(properties); },
          ::pybind11::arg("properties"))
      .def(
          "setAspectState",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>::AspectState& state) {
            self->setAspectState(state);
          },
          ::pybind11::arg("state"))
      .def(
          "setAspectProperties",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>::AspectProperties&
                  properties) { self->setAspectProperties(properties); },
          ::pybind11::arg("properties"))
      .def(
          "getGenericJointProperties",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<2>>* self)
              -> dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>::Properties {
            return self->getGenericJointProperties();
          })
      .def(
          "copy",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>::ThisClass& otherJoint) {
            self->copy(otherJoint);
          },
          ::pybind11::arg("otherJoint"))
      .def(
          "copy",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>::ThisClass* otherJoint) {
            self->copy(otherJoint);
          },
          ::pybind11::arg("otherJoint"))
      .def(
          "getNumDofs",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<2>>* self) -> std::size_t {
            return self->getNumDofs();
          })
      .def(
          "setDofName",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              size_t index,
              const std::string& name) -> const std::string& {
            return self->setDofName(index, name);
          },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("index"),
          ::pybind11::arg("name"))
      .def(
          "setDofName",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              size_t index,
              const std::string& name,
              bool preserveName) -> const std::string& {
            return self->setDofName(index, name, preserveName);
          },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("index"),
          ::pybind11::arg("name"),
          ::pybind11::arg("preserveName"))
      .def(
          "preserveDofName",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              size_t index,
              bool preserve) { self->preserveDofName(index, preserve); },
          ::pybind11::arg("index"),
          ::pybind11::arg("preserve"))
      .def(
          "isDofNamePreserved",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              size_t index) -> bool { return self->isDofNamePreserved(index); },
          ::pybind11::arg("index"))
      .def(
          "getDofName",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              size_t index) -> const std::string& {
            return self->getDofName(index);
          },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("index"))
      .def(
          "getIndexInSkeleton",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              size_t index) -> size_t {
            return self->getIndexInSkeleton(index);
          },
          ::pybind11::arg("index"))
      .def(
          "getIndexInTree",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              size_t index) -> size_t { return self->getIndexInTree(index); },
          ::pybind11::arg("index"))
      .def(
          "setCommand",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              std::size_t index,
              double command) { self->setCommand(index, command); },
          ::pybind11::arg("index"),
          ::pybind11::arg("command"))
      .def(
          "getCommand",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              std::size_t index) -> double { return self->getCommand(index); },
          ::pybind11::arg("index"))
      .def(
          "setCommands",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              const Eigen::VectorXd& commands) { self->setCommands(commands); },
          ::pybind11::arg("commands"))
      .def(
          "getCommands",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<2>>* self) -> Eigen::VectorXd {
            return self->getCommands();
          })
      .def(
          "resetCommands",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self) { self->resetCommands(); })
      .def(
          "setPosition",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              std::size_t index,
              double position) { self->setPosition(index, position); },
          ::pybind11::arg("index"),
          ::pybind11::arg("position"))
      .def(
          "getPosition",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              std::size_t index) -> double { return self->getPosition(index); },
          ::pybind11::arg("index"))
      .def(
          "setPositions",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              const Eigen::VectorXd& positions) {
            self->setPositions(positions);
          },
          ::pybind11::arg("positions"))
      .def(
          "getPositions",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<2>>* self) -> Eigen::VectorXd {
            return self->getPositions();
          })
      .def(
          "setPositionLowerLimit",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              size_t index,
              double position) {
            self->setPositionLowerLimit(index, position);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("position"))
      .def(
          "getPositionLowerLimit",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              std::size_t index) -> double {
            return self->getPositionLowerLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setPositionLowerLimits",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              const Eigen::VectorXd& lowerLimits) {
            self->setPositionLowerLimits(lowerLimits);
          },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getPositionLowerLimits",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<2>>* self) -> Eigen::VectorXd {
            return self->getPositionLowerLimits();
          })
      .def(
          "setPositionUpperLimit",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              size_t index,
              double position) {
            self->setPositionUpperLimit(index, position);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("position"))
      .def(
          "getPositionUpperLimit",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              std::size_t index) -> double {
            return self->getPositionUpperLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setPositionUpperLimits",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              const Eigen::VectorXd& upperLimits) {
            self->setPositionUpperLimits(upperLimits);
          },
          ::pybind11::arg("upperLimits"))
      .def(
          "getPositionUpperLimits",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<2>>* self) -> Eigen::VectorXd {
            return self->getPositionUpperLimits();
          })
      .def(
          "hasPositionLimit",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              std::size_t index) -> bool {
            return self->hasPositionLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "resetPosition",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              std::size_t index) { self->resetPosition(index); },
          ::pybind11::arg("index"))
      .def(
          "resetPositions",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self) { self->resetPositions(); })
      .def(
          "setInitialPosition",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              size_t index,
              double initial) { self->setInitialPosition(index, initial); },
          ::pybind11::arg("index"),
          ::pybind11::arg("initial"))
      .def(
          "getInitialPosition",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              std::size_t index) -> double {
            return self->getInitialPosition(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setInitialPositions",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              const Eigen::VectorXd& initial) {
            self->setInitialPositions(initial);
          },
          ::pybind11::arg("initial"))
      .def(
          "getInitialPositions",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<2>>* self) -> Eigen::VectorXd {
            return self->getInitialPositions();
          })
      .def(
          "setPositionsStatic",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>::Vector& positions) {
            self->setPositionsStatic(positions);
          },
          ::pybind11::arg("positions"))
      .def(
          "setVelocitiesStatic",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>::Vector& velocities) {
            self->setVelocitiesStatic(velocities);
          },
          ::pybind11::arg("velocities"))
      .def(
          "setAccelerationsStatic",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>::Vector& accels) {
            self->setAccelerationsStatic(accels);
          },
          ::pybind11::arg("accels"))
      .def(
          "setVelocity",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              std::size_t index,
              double velocity) { self->setVelocity(index, velocity); },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocity"))
      .def(
          "getVelocity",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              std::size_t index) -> double { return self->getVelocity(index); },
          ::pybind11::arg("index"))
      .def(
          "setVelocities",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              const Eigen::VectorXd& velocities) {
            self->setVelocities(velocities);
          },
          ::pybind11::arg("velocities"))
      .def(
          "getVelocities",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<2>>* self) -> Eigen::VectorXd {
            return self->getVelocities();
          })
      .def(
          "setVelocityLowerLimit",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              size_t index,
              double velocity) {
            self->setVelocityLowerLimit(index, velocity);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocity"))
      .def(
          "getVelocityLowerLimit",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              std::size_t index) -> double {
            return self->getVelocityLowerLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setVelocityLowerLimits",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              const Eigen::VectorXd& lowerLimits) {
            self->setVelocityLowerLimits(lowerLimits);
          },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getVelocityLowerLimits",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<2>>* self) -> Eigen::VectorXd {
            return self->getVelocityLowerLimits();
          })
      .def(
          "setVelocityUpperLimit",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              size_t index,
              double velocity) {
            self->setVelocityUpperLimit(index, velocity);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocity"))
      .def(
          "getVelocityUpperLimit",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              std::size_t index) -> double {
            return self->getVelocityUpperLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setVelocityUpperLimits",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              const Eigen::VectorXd& upperLimits) {
            self->setVelocityUpperLimits(upperLimits);
          },
          ::pybind11::arg("upperLimits"))
      .def(
          "getVelocityUpperLimits",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<2>>* self) -> Eigen::VectorXd {
            return self->getVelocityUpperLimits();
          })
      .def(
          "resetVelocity",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              std::size_t index) { self->resetVelocity(index); },
          ::pybind11::arg("index"))
      .def(
          "resetVelocities",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self) { self->resetVelocities(); })
      .def(
          "setInitialVelocity",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              size_t index,
              double initial) { self->setInitialVelocity(index, initial); },
          ::pybind11::arg("index"),
          ::pybind11::arg("initial"))
      .def(
          "getInitialVelocity",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              std::size_t index) -> double {
            return self->getInitialVelocity(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setInitialVelocities",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              const Eigen::VectorXd& initial) {
            self->setInitialVelocities(initial);
          },
          ::pybind11::arg("initial"))
      .def(
          "getInitialVelocities",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<2>>* self) -> Eigen::VectorXd {
            return self->getInitialVelocities();
          })
      .def(
          "setAcceleration",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              std::size_t index,
              double acceleration) {
            self->setAcceleration(index, acceleration);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("acceleration"))
      .def(
          "getAcceleration",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              std::size_t index) -> double {
            return self->getAcceleration(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setAccelerations",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              const Eigen::VectorXd& accelerations) {
            self->setAccelerations(accelerations);
          },
          ::pybind11::arg("accelerations"))
      .def(
          "getAccelerations",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<2>>* self) -> Eigen::VectorXd {
            return self->getAccelerations();
          })
      .def(
          "setAccelerationLowerLimit",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              size_t index,
              double acceleration) {
            self->setAccelerationLowerLimit(index, acceleration);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("acceleration"))
      .def(
          "getAccelerationLowerLimit",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              std::size_t index) -> double {
            return self->getAccelerationLowerLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setAccelerationLowerLimits",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              const Eigen::VectorXd& lowerLimits) {
            self->setAccelerationLowerLimits(lowerLimits);
          },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getAccelerationLowerLimits",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<2>>* self) -> Eigen::VectorXd {
            return self->getAccelerationLowerLimits();
          })
      .def(
          "setAccelerationUpperLimit",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              size_t index,
              double acceleration) {
            self->setAccelerationUpperLimit(index, acceleration);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("acceleration"))
      .def(
          "getAccelerationUpperLimit",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              std::size_t index) -> double {
            return self->getAccelerationUpperLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setAccelerationUpperLimits",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              const Eigen::VectorXd& upperLimits) {
            self->setAccelerationUpperLimits(upperLimits);
          },
          ::pybind11::arg("upperLimits"))
      .def(
          "getAccelerationUpperLimits",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<2>>* self) -> Eigen::VectorXd {
            return self->getAccelerationUpperLimits();
          })
      .def(
          "resetAccelerations",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self) { self->resetAccelerations(); })
      .def(
          "setForce",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              std::size_t index,
              double force) { self->setForce(index, force); },
          ::pybind11::arg("index"),
          ::pybind11::arg("force"))
      .def(
          "getForce",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              std::size_t index) -> double { return self->getForce(index); },
          ::pybind11::arg("index"))
      .def(
          "setForces",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              const Eigen::VectorXd& forces) { self->setForces(forces); },
          ::pybind11::arg("forces"))
      .def(
          "getForces",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<2>>* self) -> Eigen::VectorXd {
            return self->getForces();
          })
      .def(
          "setForceLowerLimit",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              size_t index,
              double force) { self->setForceLowerLimit(index, force); },
          ::pybind11::arg("index"),
          ::pybind11::arg("force"))
      .def(
          "getForceLowerLimit",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              std::size_t index) -> double {
            return self->getForceLowerLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setForceLowerLimits",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              const Eigen::VectorXd& lowerLimits) {
            self->setForceLowerLimits(lowerLimits);
          },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getForceLowerLimits",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<2>>* self) -> Eigen::VectorXd {
            return self->getForceLowerLimits();
          })
      .def(
          "setForceUpperLimit",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              size_t index,
              double force) { self->setForceUpperLimit(index, force); },
          ::pybind11::arg("index"),
          ::pybind11::arg("force"))
      .def(
          "getForceUpperLimit",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              size_t index) -> double {
            return self->getForceUpperLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setForceUpperLimits",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              const Eigen::VectorXd& upperLimits) {
            self->setForceUpperLimits(upperLimits);
          },
          ::pybind11::arg("upperLimits"))
      .def(
          "getForceUpperLimits",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<2>>* self) -> Eigen::VectorXd {
            return self->getForceUpperLimits();
          })
      .def(
          "resetForces",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self) { self->resetForces(); })
      .def(
          "setVelocityChange",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              std::size_t index,
              double velocityChange) {
            self->setVelocityChange(index, velocityChange);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocityChange"))
      .def(
          "getVelocityChange",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              std::size_t index) -> double {
            return self->getVelocityChange(index);
          },
          ::pybind11::arg("index"))
      .def(
          "resetVelocityChanges",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self) { self->resetVelocityChanges(); })
      .def(
          "setConstraintImpulse",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              std::size_t index,
              double impulse) { self->setConstraintImpulse(index, impulse); },
          ::pybind11::arg("index"),
          ::pybind11::arg("impulse"))
      .def(
          "getConstraintImpulse",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              std::size_t index) -> double {
            return self->getConstraintImpulse(index);
          },
          ::pybind11::arg("index"))
      .def(
          "resetConstraintImpulses",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self) { self->resetConstraintImpulses(); })
      .def(
          "integratePositions",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              double dt) { self->integratePositions(dt); },
          ::pybind11::arg("dt"))
      .def(
          "integrateVelocities",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              double dt) { self->integrateVelocities(dt); },
          ::pybind11::arg("dt"))
      .def(
          "getPositionDifferences",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              const Eigen::VectorXd& q2,
              const Eigen::VectorXd& q1) -> Eigen::VectorXd {
            return self->getPositionDifferences(q2, q1);
          },
          ::pybind11::arg("q2"),
          ::pybind11::arg("q1"))
      .def(
          "getPositionDifferencesStatic",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>::Vector& q2,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>::Vector& q1)
              -> dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>::Vector {
            return self->getPositionDifferencesStatic(q2, q1);
          },
          ::pybind11::arg("q2"),
          ::pybind11::arg("q1"))
      .def(
          "setSpringStiffness",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              size_t index,
              double k) { self->setSpringStiffness(index, k); },
          ::pybind11::arg("index"),
          ::pybind11::arg("k"))
      .def(
          "getSpringStiffness",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              std::size_t index) -> double {
            return self->getSpringStiffness(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setRestPosition",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              size_t index,
              double q0) { self->setRestPosition(index, q0); },
          ::pybind11::arg("index"),
          ::pybind11::arg("q0"))
      .def(
          "getRestPosition",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              std::size_t index) -> double {
            return self->getRestPosition(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setDampingCoefficient",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              size_t index,
              double d) { self->setDampingCoefficient(index, d); },
          ::pybind11::arg("index"),
          ::pybind11::arg("d"))
      .def(
          "getDampingCoefficient",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              std::size_t index) -> double {
            return self->getDampingCoefficient(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setCoulombFriction",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<2>>*
                  self,
              size_t index,
              double friction) { self->setCoulombFriction(index, friction); },
          ::pybind11::arg("index"),
          ::pybind11::arg("friction"))
      .def(
          "getCoulombFriction",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              std::size_t index) -> double {
            return self->getCoulombFriction(index);
          },
          ::pybind11::arg("index"))
      .def(
          "computePotentialEnergy",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<2>>* self) -> double {
            return self->computePotentialEnergy();
          })
      .def(
          "getBodyConstraintWrench",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<2>>* self) -> Eigen::Vector6d {
            return self->getBodyConstraintWrench();
          })
      .def(
          "getRelativeJacobian",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<2>>* self)
              -> const dart::math::Jacobian {
            return self->getRelativeJacobian();
          })
      .def(
          "getRelativeJacobian",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              const Eigen::VectorXd& _positions) -> dart::math::Jacobian {
            return self->getRelativeJacobian(_positions);
          },
          ::pybind11::arg("_positions"))
      .def(
          "getRelativeJacobianStatic",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>* self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>::Vector& positions)
              -> dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<2>>::JacobianMatrix {
            return self->getRelativeJacobianStatic(positions);
          },
          ::pybind11::arg("positions"))
      .def(
          "getRelativeJacobianTimeDeriv",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<2>>* self)
              -> const dart::math::Jacobian {
            return self->getRelativeJacobianTimeDeriv();
          })
      .def_readonly_static(
          "NumDofs",
          &dart::dynamics::GenericJoint<
              dart::math::RealVectorSpace<2>>::NumDofs);

  ::pybind11::class_<
      dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>
      // dart::common::EmbedStateAndPropertiesOnTopOf<
      //     dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>,
      //     dart::dynamics::detail::GenericJointState<
      //         dart::math::RealVectorSpace<3>>,
      //     dart::dynamics::detail::GenericJointUniqueProperties<
      //         dart::math::RealVectorSpace<3>>,
      //     dart::dynamics::Joint>,
      >(m, "GenericJoint_R3")
      .def(
          "hasGenericJointAspect",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<3>>* self) -> bool {
            return self->hasGenericJointAspect();
          })
      .def(
          "setGenericJointAspect",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>::Aspect* aspect) {
            self->setGenericJointAspect(aspect);
          },
          ::pybind11::arg("aspect"))
      .def(
          "removeGenericJointAspect",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self) { self->removeGenericJointAspect(); })
      .def(
          "releaseGenericJointAspect",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self)
              -> std::unique_ptr<dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>::Aspect> {
            return self->releaseGenericJointAspect();
          })
      .def(
          "setProperties",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>::Properties& properties) {
            self->setProperties(properties);
          },
          ::pybind11::arg("properties"))
      .def(
          "setProperties",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>::UniqueProperties&
                  properties) { self->setProperties(properties); },
          ::pybind11::arg("properties"))
      .def(
          "setAspectState",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>::AspectState& state) {
            self->setAspectState(state);
          },
          ::pybind11::arg("state"))
      .def(
          "setAspectProperties",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>::AspectProperties&
                  properties) { self->setAspectProperties(properties); },
          ::pybind11::arg("properties"))
      .def(
          "getGenericJointProperties",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<3>>* self)
              -> dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>::Properties {
            return self->getGenericJointProperties();
          })
      .def(
          "copy",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>::ThisClass& otherJoint) {
            self->copy(otherJoint);
          },
          ::pybind11::arg("otherJoint"))
      .def(
          "copy",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>::ThisClass* otherJoint) {
            self->copy(otherJoint);
          },
          ::pybind11::arg("otherJoint"))
      .def(
          "getNumDofs",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<3>>* self) -> std::size_t {
            return self->getNumDofs();
          })
      .def(
          "setDofName",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              size_t index,
              const std::string& name) -> const std::string& {
            return self->setDofName(index, name);
          },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("index"),
          ::pybind11::arg("name"))
      .def(
          "setDofName",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              size_t index,
              const std::string& name,
              bool preserveName) -> const std::string& {
            return self->setDofName(index, name, preserveName);
          },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("index"),
          ::pybind11::arg("name"),
          ::pybind11::arg("preserveName"))
      .def(
          "preserveDofName",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              size_t index,
              bool preserve) { self->preserveDofName(index, preserve); },
          ::pybind11::arg("index"),
          ::pybind11::arg("preserve"))
      .def(
          "isDofNamePreserved",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              size_t index) -> bool { return self->isDofNamePreserved(index); },
          ::pybind11::arg("index"))
      .def(
          "getDofName",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              size_t index) -> const std::string& {
            return self->getDofName(index);
          },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("index"))
      .def(
          "getIndexInSkeleton",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              size_t index) -> size_t {
            return self->getIndexInSkeleton(index);
          },
          ::pybind11::arg("index"))
      .def(
          "getIndexInTree",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              size_t index) -> size_t { return self->getIndexInTree(index); },
          ::pybind11::arg("index"))
      .def(
          "setCommand",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              std::size_t index,
              double command) { self->setCommand(index, command); },
          ::pybind11::arg("index"),
          ::pybind11::arg("command"))
      .def(
          "getCommand",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              std::size_t index) -> double { return self->getCommand(index); },
          ::pybind11::arg("index"))
      .def(
          "setCommands",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              const Eigen::VectorXd& commands) { self->setCommands(commands); },
          ::pybind11::arg("commands"))
      .def(
          "getCommands",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<3>>* self) -> Eigen::VectorXd {
            return self->getCommands();
          })
      .def(
          "resetCommands",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self) { self->resetCommands(); })
      .def(
          "setPosition",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              std::size_t index,
              double position) { self->setPosition(index, position); },
          ::pybind11::arg("index"),
          ::pybind11::arg("position"))
      .def(
          "getPosition",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              std::size_t index) -> double { return self->getPosition(index); },
          ::pybind11::arg("index"))
      .def(
          "setPositions",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              const Eigen::VectorXd& positions) {
            self->setPositions(positions);
          },
          ::pybind11::arg("positions"))
      .def(
          "getPositions",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<3>>* self) -> Eigen::VectorXd {
            return self->getPositions();
          })
      .def(
          "setPositionLowerLimit",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              size_t index,
              double position) {
            self->setPositionLowerLimit(index, position);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("position"))
      .def(
          "getPositionLowerLimit",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              std::size_t index) -> double {
            return self->getPositionLowerLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setPositionLowerLimits",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              const Eigen::VectorXd& lowerLimits) {
            self->setPositionLowerLimits(lowerLimits);
          },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getPositionLowerLimits",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<3>>* self) -> Eigen::VectorXd {
            return self->getPositionLowerLimits();
          })
      .def(
          "setPositionUpperLimit",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              size_t index,
              double position) {
            self->setPositionUpperLimit(index, position);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("position"))
      .def(
          "getPositionUpperLimit",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              std::size_t index) -> double {
            return self->getPositionUpperLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setPositionUpperLimits",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              const Eigen::VectorXd& upperLimits) {
            self->setPositionUpperLimits(upperLimits);
          },
          ::pybind11::arg("upperLimits"))
      .def(
          "getPositionUpperLimits",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<3>>* self) -> Eigen::VectorXd {
            return self->getPositionUpperLimits();
          })
      .def(
          "hasPositionLimit",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              std::size_t index) -> bool {
            return self->hasPositionLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "resetPosition",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              std::size_t index) { self->resetPosition(index); },
          ::pybind11::arg("index"))
      .def(
          "resetPositions",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self) { self->resetPositions(); })
      .def(
          "setInitialPosition",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              size_t index,
              double initial) { self->setInitialPosition(index, initial); },
          ::pybind11::arg("index"),
          ::pybind11::arg("initial"))
      .def(
          "getInitialPosition",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              std::size_t index) -> double {
            return self->getInitialPosition(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setInitialPositions",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              const Eigen::VectorXd& initial) {
            self->setInitialPositions(initial);
          },
          ::pybind11::arg("initial"))
      .def(
          "getInitialPositions",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<3>>* self) -> Eigen::VectorXd {
            return self->getInitialPositions();
          })
      .def(
          "setPositionsStatic",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>::Vector& positions) {
            self->setPositionsStatic(positions);
          },
          ::pybind11::arg("positions"))
      .def(
          "getPositionsStatic",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<3>>* self)
              -> const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>::Vector& {
            return self->getPositionsStatic();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "setVelocitiesStatic",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>::Vector& velocities) {
            self->setVelocitiesStatic(velocities);
          },
          ::pybind11::arg("velocities"))
      .def(
          "getVelocitiesStatic",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<3>>* self)
              -> const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>::Vector& {
            return self->getVelocitiesStatic();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "setAccelerationsStatic",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>::Vector& accels) {
            self->setAccelerationsStatic(accels);
          },
          ::pybind11::arg("accels"))
      .def(
          "getAccelerationsStatic",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<3>>* self)
              -> const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>::Vector& {
            return self->getAccelerationsStatic();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "setVelocity",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              std::size_t index,
              double velocity) { self->setVelocity(index, velocity); },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocity"))
      .def(
          "getVelocity",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              std::size_t index) -> double { return self->getVelocity(index); },
          ::pybind11::arg("index"))
      .def(
          "setVelocities",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              const Eigen::VectorXd& velocities) {
            self->setVelocities(velocities);
          },
          ::pybind11::arg("velocities"))
      .def(
          "getVelocities",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<3>>* self) -> Eigen::VectorXd {
            return self->getVelocities();
          })
      .def(
          "setVelocityLowerLimit",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              size_t index,
              double velocity) {
            self->setVelocityLowerLimit(index, velocity);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocity"))
      .def(
          "getVelocityLowerLimit",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              std::size_t index) -> double {
            return self->getVelocityLowerLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setVelocityLowerLimits",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              const Eigen::VectorXd& lowerLimits) {
            self->setVelocityLowerLimits(lowerLimits);
          },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getVelocityLowerLimits",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<3>>* self) -> Eigen::VectorXd {
            return self->getVelocityLowerLimits();
          })
      .def(
          "setVelocityUpperLimit",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              size_t index,
              double velocity) {
            self->setVelocityUpperLimit(index, velocity);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocity"))
      .def(
          "getVelocityUpperLimit",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              std::size_t index) -> double {
            return self->getVelocityUpperLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setVelocityUpperLimits",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              const Eigen::VectorXd& upperLimits) {
            self->setVelocityUpperLimits(upperLimits);
          },
          ::pybind11::arg("upperLimits"))
      .def(
          "getVelocityUpperLimits",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<3>>* self) -> Eigen::VectorXd {
            return self->getVelocityUpperLimits();
          })
      .def(
          "resetVelocity",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              std::size_t index) { self->resetVelocity(index); },
          ::pybind11::arg("index"))
      .def(
          "resetVelocities",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self) { self->resetVelocities(); })
      .def(
          "setInitialVelocity",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              size_t index,
              double initial) { self->setInitialVelocity(index, initial); },
          ::pybind11::arg("index"),
          ::pybind11::arg("initial"))
      .def(
          "getInitialVelocity",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              std::size_t index) -> double {
            return self->getInitialVelocity(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setInitialVelocities",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              const Eigen::VectorXd& initial) {
            self->setInitialVelocities(initial);
          },
          ::pybind11::arg("initial"))
      .def(
          "getInitialVelocities",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<3>>* self) -> Eigen::VectorXd {
            return self->getInitialVelocities();
          })
      .def(
          "setAcceleration",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              std::size_t index,
              double acceleration) {
            self->setAcceleration(index, acceleration);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("acceleration"))
      .def(
          "getAcceleration",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              std::size_t index) -> double {
            return self->getAcceleration(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setAccelerations",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              const Eigen::VectorXd& accelerations) {
            self->setAccelerations(accelerations);
          },
          ::pybind11::arg("accelerations"))
      .def(
          "getAccelerations",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<3>>* self) -> Eigen::VectorXd {
            return self->getAccelerations();
          })
      .def(
          "setAccelerationLowerLimit",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              size_t index,
              double acceleration) {
            self->setAccelerationLowerLimit(index, acceleration);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("acceleration"))
      .def(
          "getAccelerationLowerLimit",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              std::size_t index) -> double {
            return self->getAccelerationLowerLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setAccelerationLowerLimits",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              const Eigen::VectorXd& lowerLimits) {
            self->setAccelerationLowerLimits(lowerLimits);
          },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getAccelerationLowerLimits",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<3>>* self) -> Eigen::VectorXd {
            return self->getAccelerationLowerLimits();
          })
      .def(
          "setAccelerationUpperLimit",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              size_t index,
              double acceleration) {
            self->setAccelerationUpperLimit(index, acceleration);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("acceleration"))
      .def(
          "getAccelerationUpperLimit",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              std::size_t index) -> double {
            return self->getAccelerationUpperLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setAccelerationUpperLimits",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              const Eigen::VectorXd& upperLimits) {
            self->setAccelerationUpperLimits(upperLimits);
          },
          ::pybind11::arg("upperLimits"))
      .def(
          "getAccelerationUpperLimits",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<3>>* self) -> Eigen::VectorXd {
            return self->getAccelerationUpperLimits();
          })
      .def(
          "resetAccelerations",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self) { self->resetAccelerations(); })
      .def(
          "setForce",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              std::size_t index,
              double force) { self->setForce(index, force); },
          ::pybind11::arg("index"),
          ::pybind11::arg("force"))
      .def(
          "getForce",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              std::size_t index) -> double { return self->getForce(index); },
          ::pybind11::arg("index"))
      .def(
          "setForces",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              const Eigen::VectorXd& forces) { self->setForces(forces); },
          ::pybind11::arg("forces"))
      .def(
          "getForces",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<3>>* self) -> Eigen::VectorXd {
            return self->getForces();
          })
      .def(
          "setForceLowerLimit",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              size_t index,
              double force) { self->setForceLowerLimit(index, force); },
          ::pybind11::arg("index"),
          ::pybind11::arg("force"))
      .def(
          "getForceLowerLimit",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              std::size_t index) -> double {
            return self->getForceLowerLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setForceLowerLimits",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              const Eigen::VectorXd& lowerLimits) {
            self->setForceLowerLimits(lowerLimits);
          },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getForceLowerLimits",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<3>>* self) -> Eigen::VectorXd {
            return self->getForceLowerLimits();
          })
      .def(
          "setForceUpperLimit",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              size_t index,
              double force) { self->setForceUpperLimit(index, force); },
          ::pybind11::arg("index"),
          ::pybind11::arg("force"))
      .def(
          "getForceUpperLimit",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              size_t index) -> double {
            return self->getForceUpperLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setForceUpperLimits",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              const Eigen::VectorXd& upperLimits) {
            self->setForceUpperLimits(upperLimits);
          },
          ::pybind11::arg("upperLimits"))
      .def(
          "getForceUpperLimits",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<3>>* self) -> Eigen::VectorXd {
            return self->getForceUpperLimits();
          })
      .def(
          "resetForces",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self) { self->resetForces(); })
      .def(
          "setVelocityChange",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              std::size_t index,
              double velocityChange) {
            self->setVelocityChange(index, velocityChange);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocityChange"))
      .def(
          "getVelocityChange",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              std::size_t index) -> double {
            return self->getVelocityChange(index);
          },
          ::pybind11::arg("index"))
      .def(
          "resetVelocityChanges",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self) { self->resetVelocityChanges(); })
      .def(
          "setConstraintImpulse",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              std::size_t index,
              double impulse) { self->setConstraintImpulse(index, impulse); },
          ::pybind11::arg("index"),
          ::pybind11::arg("impulse"))
      .def(
          "getConstraintImpulse",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              std::size_t index) -> double {
            return self->getConstraintImpulse(index);
          },
          ::pybind11::arg("index"))
      .def(
          "resetConstraintImpulses",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self) { self->resetConstraintImpulses(); })
      .def(
          "integratePositions",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              double dt) { self->integratePositions(dt); },
          ::pybind11::arg("dt"))
      .def(
          "integrateVelocities",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              double dt) { self->integrateVelocities(dt); },
          ::pybind11::arg("dt"))
      .def(
          "getPositionDifferences",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              const Eigen::VectorXd& q2,
              const Eigen::VectorXd& q1) -> Eigen::VectorXd {
            return self->getPositionDifferences(q2, q1);
          },
          ::pybind11::arg("q2"),
          ::pybind11::arg("q1"))
      .def(
          "getPositionDifferencesStatic",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>::Vector& q2,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>::Vector& q1)
              -> dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>::Vector {
            return self->getPositionDifferencesStatic(q2, q1);
          },
          ::pybind11::arg("q2"),
          ::pybind11::arg("q1"))
      .def(
          "setSpringStiffness",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              size_t index,
              double k) { self->setSpringStiffness(index, k); },
          ::pybind11::arg("index"),
          ::pybind11::arg("k"))
      .def(
          "getSpringStiffness",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              std::size_t index) -> double {
            return self->getSpringStiffness(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setRestPosition",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              size_t index,
              double q0) { self->setRestPosition(index, q0); },
          ::pybind11::arg("index"),
          ::pybind11::arg("q0"))
      .def(
          "getRestPosition",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              std::size_t index) -> double {
            return self->getRestPosition(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setDampingCoefficient",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              size_t index,
              double d) { self->setDampingCoefficient(index, d); },
          ::pybind11::arg("index"),
          ::pybind11::arg("d"))
      .def(
          "getDampingCoefficient",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              std::size_t index) -> double {
            return self->getDampingCoefficient(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setCoulombFriction",
          +[](dart::dynamics::GenericJoint<dart::math::RealVectorSpace<3>>*
                  self,
              size_t index,
              double friction) { self->setCoulombFriction(index, friction); },
          ::pybind11::arg("index"),
          ::pybind11::arg("friction"))
      .def(
          "getCoulombFriction",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              std::size_t index) -> double {
            return self->getCoulombFriction(index);
          },
          ::pybind11::arg("index"))
      .def(
          "computePotentialEnergy",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<3>>* self) -> double {
            return self->computePotentialEnergy();
          })
      .def(
          "getBodyConstraintWrench",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<3>>* self) -> Eigen::Vector6d {
            return self->getBodyConstraintWrench();
          })
      .def(
          "getRelativeJacobian",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<3>>* self)
              -> const dart::math::Jacobian {
            return self->getRelativeJacobian();
          })
      .def(
          "getRelativeJacobian",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              const Eigen::VectorXd& _positions) -> dart::math::Jacobian {
            return self->getRelativeJacobian(_positions);
          },
          ::pybind11::arg("_positions"))
      .def(
          "getRelativeJacobianStatic",
          +[](const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>* self,
              const dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>::Vector& positions)
              -> dart::dynamics::GenericJoint<
                  dart::math::RealVectorSpace<3>>::JacobianMatrix {
            return self->getRelativeJacobianStatic(positions);
          },
          ::pybind11::arg("positions"))
      .def(
          "getRelativeJacobianTimeDeriv",
          +[](const dart::dynamics::GenericJoint<
               dart::math::RealVectorSpace<3>>* self)
              -> const dart::math::Jacobian {
            return self->getRelativeJacobianTimeDeriv();
          })
      .def_readonly_static(
          "NumDofs",
          &dart::dynamics::GenericJoint<
              dart::math::RealVectorSpace<3>>::NumDofs);

  ::pybind11::class_<
      dart::dynamics::GenericJoint<dart::math::SO3Space>
      // dart::common::EmbedStateAndPropertiesOnTopOf<
      //     dart::dynamics::GenericJoint<dart::math::SO3Space>,
      //     dart::dynamics::detail::GenericJointState<dart::math::SO3Space>,
      //     dart::dynamics::detail::GenericJointUniqueProperties<
      //         dart::math::SO3Space>,
      //     dart::dynamics::Joint>,
      >(m, "GenericJoint_SO3Space")
      .def(
          "hasGenericJointAspect",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self)
              -> bool { return self->hasGenericJointAspect(); })
      .def(
          "setGenericJointAspect",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const dart::dynamics::GenericJoint<dart::math::SO3Space>::Aspect*
                  aspect) { self->setGenericJointAspect(aspect); },
          ::pybind11::arg("aspect"))
      .def(
          "removeGenericJointAspect",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self) {
            self->removeGenericJointAspect();
          })
      .def(
          "releaseGenericJointAspect",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self)
              -> std::unique_ptr<
                  dart::dynamics::GenericJoint<dart::math::SO3Space>::Aspect> {
            return self->releaseGenericJointAspect();
          })
      .def(
          "setProperties",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const dart::dynamics::GenericJoint<
                  dart::math::SO3Space>::Properties& properties) {
            self->setProperties(properties);
          },
          ::pybind11::arg("properties"))
      .def(
          "setProperties",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const dart::dynamics::GenericJoint<
                  dart::math::SO3Space>::UniqueProperties& properties) {
            self->setProperties(properties);
          },
          ::pybind11::arg("properties"))
      .def(
          "setAspectState",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const dart::dynamics::GenericJoint<
                  dart::math::SO3Space>::AspectState& state) {
            self->setAspectState(state);
          },
          ::pybind11::arg("state"))
      .def(
          "setAspectProperties",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const dart::dynamics::GenericJoint<
                  dart::math::SO3Space>::AspectProperties& properties) {
            self->setAspectProperties(properties);
          },
          ::pybind11::arg("properties"))
      .def(
          "getGenericJointProperties",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self)
              -> dart::dynamics::GenericJoint<
                  dart::math::SO3Space>::Properties {
            return self->getGenericJointProperties();
          })
      .def(
          "copy",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const dart::dynamics::GenericJoint<
                  dart::math::SO3Space>::ThisClass& otherJoint) {
            self->copy(otherJoint);
          },
          ::pybind11::arg("otherJoint"))
      .def(
          "copy",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const dart::dynamics::GenericJoint<
                  dart::math::SO3Space>::ThisClass* otherJoint) {
            self->copy(otherJoint);
          },
          ::pybind11::arg("otherJoint"))
      .def(
          "getNumDofs",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self)
              -> std::size_t { return self->getNumDofs(); })
      .def(
          "setDofName",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              size_t index,
              const std::string& name) -> const std::string& {
            return self->setDofName(index, name);
          },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("index"),
          ::pybind11::arg("name"))
      .def(
          "setDofName",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              size_t index,
              const std::string& name,
              bool preserveName) -> const std::string& {
            return self->setDofName(index, name, preserveName);
          },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("index"),
          ::pybind11::arg("name"),
          ::pybind11::arg("preserveName"))
      .def(
          "preserveDofName",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              size_t index,
              bool preserve) { self->preserveDofName(index, preserve); },
          ::pybind11::arg("index"),
          ::pybind11::arg("preserve"))
      .def(
          "isDofNamePreserved",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              size_t index) -> bool { return self->isDofNamePreserved(index); },
          ::pybind11::arg("index"))
      .def(
          "getDofName",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              size_t index) -> const std::string& {
            return self->getDofName(index);
          },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("index"))
      .def(
          "getIndexInSkeleton",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              size_t index) -> size_t {
            return self->getIndexInSkeleton(index);
          },
          ::pybind11::arg("index"))
      .def(
          "getIndexInTree",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              size_t index) -> size_t { return self->getIndexInTree(index); },
          ::pybind11::arg("index"))
      .def(
          "setCommand",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index,
              double command) { self->setCommand(index, command); },
          ::pybind11::arg("index"),
          ::pybind11::arg("command"))
      .def(
          "getCommand",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index) -> double { return self->getCommand(index); },
          ::pybind11::arg("index"))
      .def(
          "setCommands",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const Eigen::VectorXd& commands) { self->setCommands(commands); },
          ::pybind11::arg("commands"))
      .def(
          "getCommands",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self)
              -> Eigen::VectorXd { return self->getCommands(); })
      .def(
          "resetCommands",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self) {
            self->resetCommands();
          })
      .def(
          "setPosition",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index,
              double position) { self->setPosition(index, position); },
          ::pybind11::arg("index"),
          ::pybind11::arg("position"))
      .def(
          "getPosition",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index) -> double { return self->getPosition(index); },
          ::pybind11::arg("index"))
      .def(
          "setPositions",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const Eigen::VectorXd& positions) {
            self->setPositions(positions);
          },
          ::pybind11::arg("positions"))
      .def(
          "getPositions",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self)
              -> Eigen::VectorXd { return self->getPositions(); })
      .def(
          "setPositionLowerLimit",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              size_t index,
              double position) {
            self->setPositionLowerLimit(index, position);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("position"))
      .def(
          "getPositionLowerLimit",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index) -> double {
            return self->getPositionLowerLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setPositionLowerLimits",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const Eigen::VectorXd& lowerLimits) {
            self->setPositionLowerLimits(lowerLimits);
          },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getPositionLowerLimits",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self)
              -> Eigen::VectorXd { return self->getPositionLowerLimits(); })
      .def(
          "setPositionUpperLimit",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              size_t index,
              double position) {
            self->setPositionUpperLimit(index, position);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("position"))
      .def(
          "getPositionUpperLimit",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index) -> double {
            return self->getPositionUpperLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setPositionUpperLimits",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const Eigen::VectorXd& upperLimits) {
            self->setPositionUpperLimits(upperLimits);
          },
          ::pybind11::arg("upperLimits"))
      .def(
          "getPositionUpperLimits",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self)
              -> Eigen::VectorXd { return self->getPositionUpperLimits(); })
      .def(
          "hasPositionLimit",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index) -> bool {
            return self->hasPositionLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "resetPosition",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index) { self->resetPosition(index); },
          ::pybind11::arg("index"))
      .def(
          "resetPositions",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self) {
            self->resetPositions();
          })
      .def(
          "setInitialPosition",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              size_t index,
              double initial) { self->setInitialPosition(index, initial); },
          ::pybind11::arg("index"),
          ::pybind11::arg("initial"))
      .def(
          "getInitialPosition",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index) -> double {
            return self->getInitialPosition(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setInitialPositions",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const Eigen::VectorXd& initial) {
            self->setInitialPositions(initial);
          },
          ::pybind11::arg("initial"))
      .def(
          "getInitialPositions",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self)
              -> Eigen::VectorXd { return self->getInitialPositions(); })
      .def(
          "setPositionsStatic",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const dart::dynamics::GenericJoint<dart::math::SO3Space>::Vector&
                  positions) { self->setPositionsStatic(positions); },
          ::pybind11::arg("positions"))
      .def(
          "getPositionsStatic",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self)
              -> const dart::dynamics::GenericJoint<
                  dart::math::SO3Space>::Vector& {
            return self->getPositionsStatic();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "setVelocitiesStatic",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const dart::dynamics::GenericJoint<dart::math::SO3Space>::Vector&
                  velocities) { self->setVelocitiesStatic(velocities); },
          ::pybind11::arg("velocities"))
      .def(
          "getVelocitiesStatic",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self)
              -> const dart::dynamics::GenericJoint<
                  dart::math::SO3Space>::Vector& {
            return self->getVelocitiesStatic();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "setAccelerationsStatic",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const dart::dynamics::GenericJoint<dart::math::SO3Space>::Vector&
                  accels) { self->setAccelerationsStatic(accels); },
          ::pybind11::arg("accels"))
      .def(
          "getAccelerationsStatic",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self)
              -> const dart::dynamics::GenericJoint<
                  dart::math::SO3Space>::Vector& {
            return self->getAccelerationsStatic();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "setVelocity",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index,
              double velocity) { self->setVelocity(index, velocity); },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocity"))
      .def(
          "getVelocity",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index) -> double { return self->getVelocity(index); },
          ::pybind11::arg("index"))
      .def(
          "setVelocities",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const Eigen::VectorXd& velocities) {
            self->setVelocities(velocities);
          },
          ::pybind11::arg("velocities"))
      .def(
          "getVelocities",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self)
              -> Eigen::VectorXd { return self->getVelocities(); })
      .def(
          "setVelocityLowerLimit",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              size_t index,
              double velocity) {
            self->setVelocityLowerLimit(index, velocity);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocity"))
      .def(
          "getVelocityLowerLimit",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index) -> double {
            return self->getVelocityLowerLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setVelocityLowerLimits",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const Eigen::VectorXd& lowerLimits) {
            self->setVelocityLowerLimits(lowerLimits);
          },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getVelocityLowerLimits",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self)
              -> Eigen::VectorXd { return self->getVelocityLowerLimits(); })
      .def(
          "setVelocityUpperLimit",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              size_t index,
              double velocity) {
            self->setVelocityUpperLimit(index, velocity);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocity"))
      .def(
          "getVelocityUpperLimit",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index) -> double {
            return self->getVelocityUpperLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setVelocityUpperLimits",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const Eigen::VectorXd& upperLimits) {
            self->setVelocityUpperLimits(upperLimits);
          },
          ::pybind11::arg("upperLimits"))
      .def(
          "getVelocityUpperLimits",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self)
              -> Eigen::VectorXd { return self->getVelocityUpperLimits(); })
      .def(
          "resetVelocity",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index) { self->resetVelocity(index); },
          ::pybind11::arg("index"))
      .def(
          "resetVelocities",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self) {
            self->resetVelocities();
          })
      .def(
          "setInitialVelocity",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              size_t index,
              double initial) { self->setInitialVelocity(index, initial); },
          ::pybind11::arg("index"),
          ::pybind11::arg("initial"))
      .def(
          "getInitialVelocity",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index) -> double {
            return self->getInitialVelocity(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setInitialVelocities",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const Eigen::VectorXd& initial) {
            self->setInitialVelocities(initial);
          },
          ::pybind11::arg("initial"))
      .def(
          "getInitialVelocities",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self)
              -> Eigen::VectorXd { return self->getInitialVelocities(); })
      .def(
          "setAcceleration",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index,
              double acceleration) {
            self->setAcceleration(index, acceleration);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("acceleration"))
      .def(
          "getAcceleration",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index) -> double {
            return self->getAcceleration(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setAccelerations",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const Eigen::VectorXd& accelerations) {
            self->setAccelerations(accelerations);
          },
          ::pybind11::arg("accelerations"))
      .def(
          "getAccelerations",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self)
              -> Eigen::VectorXd { return self->getAccelerations(); })
      .def(
          "setAccelerationLowerLimit",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              size_t index,
              double acceleration) {
            self->setAccelerationLowerLimit(index, acceleration);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("acceleration"))
      .def(
          "getAccelerationLowerLimit",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index) -> double {
            return self->getAccelerationLowerLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setAccelerationLowerLimits",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const Eigen::VectorXd& lowerLimits) {
            self->setAccelerationLowerLimits(lowerLimits);
          },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getAccelerationLowerLimits",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self)
              -> Eigen::VectorXd { return self->getAccelerationLowerLimits(); })
      .def(
          "setAccelerationUpperLimit",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              size_t index,
              double acceleration) {
            self->setAccelerationUpperLimit(index, acceleration);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("acceleration"))
      .def(
          "getAccelerationUpperLimit",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index) -> double {
            return self->getAccelerationUpperLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setAccelerationUpperLimits",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const Eigen::VectorXd& upperLimits) {
            self->setAccelerationUpperLimits(upperLimits);
          },
          ::pybind11::arg("upperLimits"))
      .def(
          "getAccelerationUpperLimits",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self)
              -> Eigen::VectorXd { return self->getAccelerationUpperLimits(); })
      .def(
          "resetAccelerations",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self) {
            self->resetAccelerations();
          })
      .def(
          "setForce",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index,
              double force) { self->setForce(index, force); },
          ::pybind11::arg("index"),
          ::pybind11::arg("force"))
      .def(
          "getForce",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index) -> double { return self->getForce(index); },
          ::pybind11::arg("index"))
      .def(
          "setForces",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const Eigen::VectorXd& forces) { self->setForces(forces); },
          ::pybind11::arg("forces"))
      .def(
          "getForces",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self)
              -> Eigen::VectorXd { return self->getForces(); })
      .def(
          "setForceLowerLimit",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              size_t index,
              double force) { self->setForceLowerLimit(index, force); },
          ::pybind11::arg("index"),
          ::pybind11::arg("force"))
      .def(
          "getForceLowerLimit",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index) -> double {
            return self->getForceLowerLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setForceLowerLimits",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const Eigen::VectorXd& lowerLimits) {
            self->setForceLowerLimits(lowerLimits);
          },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getForceLowerLimits",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self)
              -> Eigen::VectorXd { return self->getForceLowerLimits(); })
      .def(
          "setForceUpperLimit",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              size_t index,
              double force) { self->setForceUpperLimit(index, force); },
          ::pybind11::arg("index"),
          ::pybind11::arg("force"))
      .def(
          "getForceUpperLimit",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              size_t index) -> double {
            return self->getForceUpperLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setForceUpperLimits",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const Eigen::VectorXd& upperLimits) {
            self->setForceUpperLimits(upperLimits);
          },
          ::pybind11::arg("upperLimits"))
      .def(
          "getForceUpperLimits",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self)
              -> Eigen::VectorXd { return self->getForceUpperLimits(); })
      .def(
          "resetForces",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self) {
            self->resetForces();
          })
      .def(
          "setVelocityChange",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index,
              double velocityChange) {
            self->setVelocityChange(index, velocityChange);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocityChange"))
      .def(
          "getVelocityChange",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index) -> double {
            return self->getVelocityChange(index);
          },
          ::pybind11::arg("index"))
      .def(
          "resetVelocityChanges",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self) {
            self->resetVelocityChanges();
          })
      .def(
          "setConstraintImpulse",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index,
              double impulse) { self->setConstraintImpulse(index, impulse); },
          ::pybind11::arg("index"),
          ::pybind11::arg("impulse"))
      .def(
          "getConstraintImpulse",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index) -> double {
            return self->getConstraintImpulse(index);
          },
          ::pybind11::arg("index"))
      .def(
          "resetConstraintImpulses",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self) {
            self->resetConstraintImpulses();
          })
      .def(
          "integratePositions",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              double dt) { self->integratePositions(dt); },
          ::pybind11::arg("dt"))
      .def(
          "integrateVelocities",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              double dt) { self->integrateVelocities(dt); },
          ::pybind11::arg("dt"))
      .def(
          "getPositionDifferences",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const Eigen::VectorXd& q2,
              const Eigen::VectorXd& q1) -> Eigen::VectorXd {
            return self->getPositionDifferences(q2, q1);
          },
          ::pybind11::arg("q2"),
          ::pybind11::arg("q1"))
      .def(
          "getPositionDifferencesStatic",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const dart::dynamics::GenericJoint<dart::math::SO3Space>::Vector&
                  q2,
              const dart::dynamics::GenericJoint<dart::math::SO3Space>::Vector&
                  q1)
              -> dart::dynamics::GenericJoint<dart::math::SO3Space>::Vector {
            return self->getPositionDifferencesStatic(q2, q1);
          },
          ::pybind11::arg("q2"),
          ::pybind11::arg("q1"))
      .def(
          "setSpringStiffness",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              size_t index,
              double k) { self->setSpringStiffness(index, k); },
          ::pybind11::arg("index"),
          ::pybind11::arg("k"))
      .def(
          "getSpringStiffness",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index) -> double {
            return self->getSpringStiffness(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setRestPosition",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              size_t index,
              double q0) { self->setRestPosition(index, q0); },
          ::pybind11::arg("index"),
          ::pybind11::arg("q0"))
      .def(
          "getRestPosition",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index) -> double {
            return self->getRestPosition(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setDampingCoefficient",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              size_t index,
              double d) { self->setDampingCoefficient(index, d); },
          ::pybind11::arg("index"),
          ::pybind11::arg("d"))
      .def(
          "getDampingCoefficient",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index) -> double {
            return self->getDampingCoefficient(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setCoulombFriction",
          +[](dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              size_t index,
              double friction) { self->setCoulombFriction(index, friction); },
          ::pybind11::arg("index"),
          ::pybind11::arg("friction"))
      .def(
          "getCoulombFriction",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              std::size_t index) -> double {
            return self->getCoulombFriction(index);
          },
          ::pybind11::arg("index"))
      .def(
          "computePotentialEnergy",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self)
              -> double { return self->computePotentialEnergy(); })
      .def(
          "getBodyConstraintWrench",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self)
              -> Eigen::Vector6d { return self->getBodyConstraintWrench(); })
      .def(
          "getRelativeJacobian",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self)
              -> const dart::math::Jacobian {
            return self->getRelativeJacobian();
          })
      .def(
          "getRelativeJacobian",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const Eigen::VectorXd& _positions) -> dart::math::Jacobian {
            return self->getRelativeJacobian(_positions);
          },
          ::pybind11::arg("_positions"))
      .def(
          "getRelativeJacobianStatic",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self,
              const dart::dynamics::GenericJoint<dart::math::SO3Space>::Vector&
                  positions)
              -> dart::dynamics::GenericJoint<
                  dart::math::SO3Space>::JacobianMatrix {
            return self->getRelativeJacobianStatic(positions);
          },
          ::pybind11::arg("positions"))
      .def(
          "getRelativeJacobianTimeDeriv",
          +[](const dart::dynamics::GenericJoint<dart::math::SO3Space>* self)
              -> const dart::math::Jacobian {
            return self->getRelativeJacobianTimeDeriv();
          })
      .def_readonly_static(
          "NumDofs",
          &dart::dynamics::GenericJoint<dart::math::SO3Space>::NumDofs);

  ::pybind11::class_<
      dart::dynamics::GenericJoint<dart::math::SE3Space>
      // dart::common::EmbedStateAndPropertiesOnTopOf<
      //     dart::dynamics::GenericJoint<dart::math::SE3Space>,
      //     dart::dynamics::detail::GenericJointState<dart::math::SE3Space>,
      //     dart::dynamics::detail::GenericJointUniqueProperties<
      //         dart::math::SE3Space>,
      //  dart::dynamics::Joint>,
      >(m, "GenericJoint_SE3Space")
      .def(
          "hasGenericJointAspect",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self)
              -> bool { return self->hasGenericJointAspect(); })
      .def(
          "setGenericJointAspect",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const dart::dynamics::GenericJoint<dart::math::SE3Space>::Aspect*
                  aspect) { self->setGenericJointAspect(aspect); },
          ::pybind11::arg("aspect"))
      .def(
          "removeGenericJointAspect",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self) {
            self->removeGenericJointAspect();
          })
      .def(
          "releaseGenericJointAspect",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self)
              -> std::unique_ptr<
                  dart::dynamics::GenericJoint<dart::math::SE3Space>::Aspect> {
            return self->releaseGenericJointAspect();
          })
      .def(
          "setProperties",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const dart::dynamics::GenericJoint<
                  dart::math::SE3Space>::Properties& properties) {
            self->setProperties(properties);
          },
          ::pybind11::arg("properties"))
      .def(
          "setProperties",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const dart::dynamics::GenericJoint<
                  dart::math::SE3Space>::UniqueProperties& properties) {
            self->setProperties(properties);
          },
          ::pybind11::arg("properties"))
      .def(
          "setAspectState",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const dart::dynamics::GenericJoint<
                  dart::math::SE3Space>::AspectState& state) {
            self->setAspectState(state);
          },
          ::pybind11::arg("state"))
      .def(
          "setAspectProperties",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const dart::dynamics::GenericJoint<
                  dart::math::SE3Space>::AspectProperties& properties) {
            self->setAspectProperties(properties);
          },
          ::pybind11::arg("properties"))
      .def(
          "getGenericJointProperties",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self)
              -> dart::dynamics::GenericJoint<
                  dart::math::SE3Space>::Properties {
            return self->getGenericJointProperties();
          })
      .def(
          "copy",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const dart::dynamics::GenericJoint<
                  dart::math::SE3Space>::ThisClass& otherJoint) {
            self->copy(otherJoint);
          },
          ::pybind11::arg("otherJoint"))
      .def(
          "copy",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const dart::dynamics::GenericJoint<
                  dart::math::SE3Space>::ThisClass* otherJoint) {
            self->copy(otherJoint);
          },
          ::pybind11::arg("otherJoint"))
      .def(
          "getNumDofs",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self)
              -> std::size_t { return self->getNumDofs(); })
      .def(
          "setDofName",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              size_t index,
              const std::string& name) -> const std::string& {
            return self->setDofName(index, name);
          },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("index"),
          ::pybind11::arg("name"))
      .def(
          "setDofName",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              size_t index,
              const std::string& name,
              bool preserveName) -> const std::string& {
            return self->setDofName(index, name, preserveName);
          },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("index"),
          ::pybind11::arg("name"),
          ::pybind11::arg("preserveName"))
      .def(
          "preserveDofName",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              size_t index,
              bool preserve) { self->preserveDofName(index, preserve); },
          ::pybind11::arg("index"),
          ::pybind11::arg("preserve"))
      .def(
          "isDofNamePreserved",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              size_t index) -> bool { return self->isDofNamePreserved(index); },
          ::pybind11::arg("index"))
      .def(
          "getDofName",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              size_t index) -> const std::string& {
            return self->getDofName(index);
          },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("index"))
      .def(
          "getIndexInSkeleton",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              size_t index) -> size_t {
            return self->getIndexInSkeleton(index);
          },
          ::pybind11::arg("index"))
      .def(
          "getIndexInTree",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              size_t index) -> size_t { return self->getIndexInTree(index); },
          ::pybind11::arg("index"))
      .def(
          "setCommand",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index,
              double command) { self->setCommand(index, command); },
          ::pybind11::arg("index"),
          ::pybind11::arg("command"))
      .def(
          "getCommand",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index) -> double { return self->getCommand(index); },
          ::pybind11::arg("index"))
      .def(
          "setCommands",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const Eigen::VectorXd& commands) { self->setCommands(commands); },
          ::pybind11::arg("commands"))
      .def(
          "getCommands",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self)
              -> Eigen::VectorXd { return self->getCommands(); })
      .def(
          "resetCommands",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self) {
            self->resetCommands();
          })
      .def(
          "setPosition",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index,
              double position) { self->setPosition(index, position); },
          ::pybind11::arg("index"),
          ::pybind11::arg("position"))
      .def(
          "getPosition",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index) -> double { return self->getPosition(index); },
          ::pybind11::arg("index"))
      .def(
          "setPositions",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const Eigen::VectorXd& positions) {
            self->setPositions(positions);
          },
          ::pybind11::arg("positions"))
      .def(
          "getPositions",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self)
              -> Eigen::VectorXd { return self->getPositions(); })
      .def(
          "setPositionLowerLimit",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              size_t index,
              double position) {
            self->setPositionLowerLimit(index, position);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("position"))
      .def(
          "getPositionLowerLimit",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index) -> double {
            return self->getPositionLowerLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setPositionLowerLimits",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const Eigen::VectorXd& lowerLimits) {
            self->setPositionLowerLimits(lowerLimits);
          },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getPositionLowerLimits",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self)
              -> Eigen::VectorXd { return self->getPositionLowerLimits(); })
      .def(
          "setPositionUpperLimit",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              size_t index,
              double position) {
            self->setPositionUpperLimit(index, position);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("position"))
      .def(
          "getPositionUpperLimit",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index) -> double {
            return self->getPositionUpperLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setPositionUpperLimits",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const Eigen::VectorXd& upperLimits) {
            self->setPositionUpperLimits(upperLimits);
          },
          ::pybind11::arg("upperLimits"))
      .def(
          "getPositionUpperLimits",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self)
              -> Eigen::VectorXd { return self->getPositionUpperLimits(); })
      .def(
          "hasPositionLimit",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index) -> bool {
            return self->hasPositionLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "resetPosition",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index) { self->resetPosition(index); },
          ::pybind11::arg("index"))
      .def(
          "resetPositions",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self) {
            self->resetPositions();
          })
      .def(
          "setInitialPosition",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              size_t index,
              double initial) { self->setInitialPosition(index, initial); },
          ::pybind11::arg("index"),
          ::pybind11::arg("initial"))
      .def(
          "getInitialPosition",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index) -> double {
            return self->getInitialPosition(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setInitialPositions",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const Eigen::VectorXd& initial) {
            self->setInitialPositions(initial);
          },
          ::pybind11::arg("initial"))
      .def(
          "getInitialPositions",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self)
              -> Eigen::VectorXd { return self->getInitialPositions(); })
      .def(
          "setPositionsStatic",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const dart::dynamics::GenericJoint<dart::math::SE3Space>::Vector&
                  positions) { self->setPositionsStatic(positions); },
          ::pybind11::arg("positions"))
      .def(
          "setVelocitiesStatic",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const dart::dynamics::GenericJoint<dart::math::SE3Space>::Vector&
                  velocities) { self->setVelocitiesStatic(velocities); },
          ::pybind11::arg("velocities"))
      .def(
          "setAccelerationsStatic",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const dart::dynamics::GenericJoint<dart::math::SE3Space>::Vector&
                  accels) { self->setAccelerationsStatic(accels); },
          ::pybind11::arg("accels"))
      .def(
          "setVelocity",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index,
              double velocity) { self->setVelocity(index, velocity); },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocity"))
      .def(
          "getVelocity",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index) -> double { return self->getVelocity(index); },
          ::pybind11::arg("index"))
      .def(
          "setVelocities",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const Eigen::VectorXd& velocities) {
            self->setVelocities(velocities);
          },
          ::pybind11::arg("velocities"))
      .def(
          "getVelocities",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self)
              -> Eigen::VectorXd { return self->getVelocities(); })
      .def(
          "setVelocityLowerLimit",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              size_t index,
              double velocity) {
            self->setVelocityLowerLimit(index, velocity);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocity"))
      .def(
          "getVelocityLowerLimit",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index) -> double {
            return self->getVelocityLowerLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setVelocityLowerLimits",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const Eigen::VectorXd& lowerLimits) {
            self->setVelocityLowerLimits(lowerLimits);
          },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getVelocityLowerLimits",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self)
              -> Eigen::VectorXd { return self->getVelocityLowerLimits(); })
      .def(
          "setVelocityUpperLimit",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              size_t index,
              double velocity) {
            self->setVelocityUpperLimit(index, velocity);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocity"))
      .def(
          "getVelocityUpperLimit",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index) -> double {
            return self->getVelocityUpperLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setVelocityUpperLimits",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const Eigen::VectorXd& upperLimits) {
            self->setVelocityUpperLimits(upperLimits);
          },
          ::pybind11::arg("upperLimits"))
      .def(
          "getVelocityUpperLimits",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self)
              -> Eigen::VectorXd { return self->getVelocityUpperLimits(); })
      .def(
          "resetVelocity",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index) { self->resetVelocity(index); },
          ::pybind11::arg("index"))
      .def(
          "resetVelocities",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self) {
            self->resetVelocities();
          })
      .def(
          "setInitialVelocity",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              size_t index,
              double initial) { self->setInitialVelocity(index, initial); },
          ::pybind11::arg("index"),
          ::pybind11::arg("initial"))
      .def(
          "getInitialVelocity",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index) -> double {
            return self->getInitialVelocity(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setInitialVelocities",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const Eigen::VectorXd& initial) {
            self->setInitialVelocities(initial);
          },
          ::pybind11::arg("initial"))
      .def(
          "getInitialVelocities",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self)
              -> Eigen::VectorXd { return self->getInitialVelocities(); })
      .def(
          "setAcceleration",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index,
              double acceleration) {
            self->setAcceleration(index, acceleration);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("acceleration"))
      .def(
          "getAcceleration",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index) -> double {
            return self->getAcceleration(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setAccelerations",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const Eigen::VectorXd& accelerations) {
            self->setAccelerations(accelerations);
          },
          ::pybind11::arg("accelerations"))
      .def(
          "getAccelerations",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self)
              -> Eigen::VectorXd { return self->getAccelerations(); })
      .def(
          "setAccelerationLowerLimit",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              size_t index,
              double acceleration) {
            self->setAccelerationLowerLimit(index, acceleration);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("acceleration"))
      .def(
          "getAccelerationLowerLimit",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index) -> double {
            return self->getAccelerationLowerLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setAccelerationLowerLimits",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const Eigen::VectorXd& lowerLimits) {
            self->setAccelerationLowerLimits(lowerLimits);
          },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getAccelerationLowerLimits",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self)
              -> Eigen::VectorXd { return self->getAccelerationLowerLimits(); })
      .def(
          "setAccelerationUpperLimit",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              size_t index,
              double acceleration) {
            self->setAccelerationUpperLimit(index, acceleration);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("acceleration"))
      .def(
          "getAccelerationUpperLimit",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index) -> double {
            return self->getAccelerationUpperLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setAccelerationUpperLimits",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const Eigen::VectorXd& upperLimits) {
            self->setAccelerationUpperLimits(upperLimits);
          },
          ::pybind11::arg("upperLimits"))
      .def(
          "getAccelerationUpperLimits",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self)
              -> Eigen::VectorXd { return self->getAccelerationUpperLimits(); })
      .def(
          "resetAccelerations",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self) {
            self->resetAccelerations();
          })
      .def(
          "setForce",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index,
              double force) { self->setForce(index, force); },
          ::pybind11::arg("index"),
          ::pybind11::arg("force"))
      .def(
          "getForce",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index) -> double { return self->getForce(index); },
          ::pybind11::arg("index"))
      .def(
          "setForces",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const Eigen::VectorXd& forces) { self->setForces(forces); },
          ::pybind11::arg("forces"))
      .def(
          "getForces",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self)
              -> Eigen::VectorXd { return self->getForces(); })
      .def(
          "setForceLowerLimit",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              size_t index,
              double force) { self->setForceLowerLimit(index, force); },
          ::pybind11::arg("index"),
          ::pybind11::arg("force"))
      .def(
          "getForceLowerLimit",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index) -> double {
            return self->getForceLowerLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setForceLowerLimits",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const Eigen::VectorXd& lowerLimits) {
            self->setForceLowerLimits(lowerLimits);
          },
          ::pybind11::arg("lowerLimits"))
      .def(
          "getForceLowerLimits",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self)
              -> Eigen::VectorXd { return self->getForceLowerLimits(); })
      .def(
          "setForceUpperLimit",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              size_t index,
              double force) { self->setForceUpperLimit(index, force); },
          ::pybind11::arg("index"),
          ::pybind11::arg("force"))
      .def(
          "getForceUpperLimit",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              size_t index) -> double {
            return self->getForceUpperLimit(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setForceUpperLimits",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const Eigen::VectorXd& upperLimits) {
            self->setForceUpperLimits(upperLimits);
          },
          ::pybind11::arg("upperLimits"))
      .def(
          "getForceUpperLimits",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self)
              -> Eigen::VectorXd { return self->getForceUpperLimits(); })
      .def(
          "resetForces",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self) {
            self->resetForces();
          })
      .def(
          "setVelocityChange",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index,
              double velocityChange) {
            self->setVelocityChange(index, velocityChange);
          },
          ::pybind11::arg("index"),
          ::pybind11::arg("velocityChange"))
      .def(
          "getVelocityChange",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index) -> double {
            return self->getVelocityChange(index);
          },
          ::pybind11::arg("index"))
      .def(
          "resetVelocityChanges",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self) {
            self->resetVelocityChanges();
          })
      .def(
          "setConstraintImpulse",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index,
              double impulse) { self->setConstraintImpulse(index, impulse); },
          ::pybind11::arg("index"),
          ::pybind11::arg("impulse"))
      .def(
          "getConstraintImpulse",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index) -> double {
            return self->getConstraintImpulse(index);
          },
          ::pybind11::arg("index"))
      .def(
          "resetConstraintImpulses",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self) {
            self->resetConstraintImpulses();
          })
      .def(
          "integratePositions",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              double dt) { self->integratePositions(dt); },
          ::pybind11::arg("dt"))
      .def(
          "integrateVelocities",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              double dt) { self->integrateVelocities(dt); },
          ::pybind11::arg("dt"))
      .def(
          "getPositionDifferences",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const Eigen::VectorXd& q2,
              const Eigen::VectorXd& q1) -> Eigen::VectorXd {
            return self->getPositionDifferences(q2, q1);
          },
          ::pybind11::arg("q2"),
          ::pybind11::arg("q1"))
      .def(
          "getPositionDifferencesStatic",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const dart::dynamics::GenericJoint<dart::math::SE3Space>::Vector&
                  q2,
              const dart::dynamics::GenericJoint<dart::math::SE3Space>::Vector&
                  q1)
              -> dart::dynamics::GenericJoint<dart::math::SE3Space>::Vector {
            return self->getPositionDifferencesStatic(q2, q1);
          },
          ::pybind11::arg("q2"),
          ::pybind11::arg("q1"))
      .def(
          "setSpringStiffness",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              size_t index,
              double k) { self->setSpringStiffness(index, k); },
          ::pybind11::arg("index"),
          ::pybind11::arg("k"))
      .def(
          "getSpringStiffness",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index) -> double {
            return self->getSpringStiffness(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setRestPosition",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              size_t index,
              double q0) { self->setRestPosition(index, q0); },
          ::pybind11::arg("index"),
          ::pybind11::arg("q0"))
      .def(
          "getRestPosition",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index) -> double {
            return self->getRestPosition(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setDampingCoefficient",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              size_t index,
              double d) { self->setDampingCoefficient(index, d); },
          ::pybind11::arg("index"),
          ::pybind11::arg("d"))
      .def(
          "getDampingCoefficient",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index) -> double {
            return self->getDampingCoefficient(index);
          },
          ::pybind11::arg("index"))
      .def(
          "setCoulombFriction",
          +[](dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              size_t index,
              double friction) { self->setCoulombFriction(index, friction); },
          ::pybind11::arg("index"),
          ::pybind11::arg("friction"))
      .def(
          "getCoulombFriction",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              std::size_t index) -> double {
            return self->getCoulombFriction(index);
          },
          ::pybind11::arg("index"))
      .def(
          "computePotentialEnergy",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self)
              -> double { return self->computePotentialEnergy(); })
      .def(
          "getBodyConstraintWrench",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self)
              -> Eigen::Vector6d { return self->getBodyConstraintWrench(); })
      .def(
          "getRelativeJacobian",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self)
              -> const dart::math::Jacobian {
            return self->getRelativeJacobian();
          })
      .def(
          "getRelativeJacobian",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const Eigen::VectorXd& _positions) -> dart::math::Jacobian {
            return self->getRelativeJacobian(_positions);
          },
          ::pybind11::arg("_positions"))
      .def(
          "getRelativeJacobianStatic",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self,
              const dart::dynamics::GenericJoint<dart::math::SE3Space>::Vector&
                  positions)
              -> dart::dynamics::GenericJoint<
                  dart::math::SE3Space>::JacobianMatrix {
            return self->getRelativeJacobianStatic(positions);
          },
          ::pybind11::arg("positions"))
      .def(
          "getRelativeJacobianTimeDeriv",
          +[](const dart::dynamics::GenericJoint<dart::math::SE3Space>* self)
              -> const dart::math::Jacobian {
            return self->getRelativeJacobianTimeDeriv();
          })
      .def_readonly_static(
          "NumDofs",
          &dart::dynamics::GenericJoint<dart::math::SE3Space>::NumDofs);
}

} // namespace python
} // namespace dart
