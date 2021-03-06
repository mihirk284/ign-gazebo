/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_COMPONENTS_AxisAlignedBox_HH_
#define IGNITION_GAZEBO_COMPONENTS_AxisAlignedBox_HH_

#include <ignition/msgs/axis_aligned_box.pb.h>
#include <ignition/math/AxisAlignedBox.hh>
#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/components/Serialization.hh>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Conversions.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace serializers
{
  using AxisAlignedBoxSerializer =
      serializers::ComponentToMsgSerializer<math::AxisAlignedBox,
      msgs::AxisAlignedBox>;
}

namespace components
{
  /// \brief A component type that contains axis aligned box,
  /// ignition::math::AxisAlignedBox, information.
  /// The axis aligned box is created from collisions in the entity
  using AxisAlignedBox = Component<ignition::math::AxisAlignedBox,
      class AxisAlignedBoxTag, serializers::AxisAlignedBoxSerializer>;
  IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.AxisAlignedBox",
      AxisAlignedBox)
}
}
}
}

#endif
