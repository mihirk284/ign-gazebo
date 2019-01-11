/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef MOVE3D_MOVE3D_HH_
#define MOVE3D_MOVE3D_HH_

#include <ignition/gazebo/System.hh>
#include <memory>

namespace ignition {
namespace gazebo {
namespace systems {

// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
// Forward declaration
class Move3dSystemPrivate;

/// \brief A system for controlling the position of objects in 3d.
class IGNITION_GAZEBO_VISIBLE Move3dSystem
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate
{
  public: Move3dSystem();

  public: ~Move3dSystem();

  /// Documentation inherited
  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &_eventMgr) final;
  public: void PreUpdate(
              const ignition::gazebo::UpdateInfo &_info,
                    ignition::gazebo::EntityComponentManager &_ecm) override;

  private: std::unique_ptr<Move3dSystemPrivate> dataPtr;
};
}  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace systems
}  // namespace gazebo
}  // namespace ignition

#endif