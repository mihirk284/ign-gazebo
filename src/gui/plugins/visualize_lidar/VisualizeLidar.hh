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

#ifndef IGNITION_GAZEBO_GUI_VISUALIZELIDAR_HH_
#define IGNITION_GAZEBO_GUI_VISUALIZELIDAR_HH_

#include <memory>

#include <ignition/gazebo/gui/GuiSystem.hh>
#include <ignition/msgs/laserscan.pb.h>
#include "ignition/gui/qt.h"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE
{
  class VisualizeLidarPrivate;

  /// \brief Visualize the contacts returned by the Physics plugin. Use the
  /// checkbox to turn visualization on or off and spin boxes to change
  /// the size of the markers.
  class VisualizeLidar : public ignition::gazebo::GuiSystem
  {
    Q_OBJECT

    /// \brief Constructor
    public: VisualizeLidar();

    /// \brief Destructor
    public: ~VisualizeLidar() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation Inherited
    public: bool eventFilter(QObject *_obj, QEvent *_event);

    // Documentation inherited
    public: void Update(const UpdateInfo &_info,
        EntityComponentManager &_ecm) override;
    
    /// \brief Callback function to get data from the message
    /// \param[in]_msg LidarSensor message
    public: void OnScan(const msgs::LaserScan &_msg);

    /// \brief Reset and clear visual
    void ResetLidarVisual();

    /// \brief Load the scene and attach LidarVisual to the scene
    public: void LoadLidar();

    /// \brief Callback when value is changed
    /// \param[in] _minRange indicates the minimum range for the visual
    public slots: void UpdateMinRange(double _minRange);

    /// \brief Callback when value is changed
    /// \param[in] _minRange indicates the maximum range for the visual
    public slots: void UpdateMaxRange(double _maxRange);

    /// \brief Set visual type of LidarVisual
    /// \param[in] _type Index of selected visual type
    public slots: void UpdateType(int _type);

    /// \brief Set topic to subscribe for LidarSensor data
    /// \param[in] _topic_name Name of selected topic
    public slots: void UpdateTopicName();

    /// \brief Set whether to display non-hitting rays
    /// \param[in] _value Boolean value for displaying non hitting rays
    public slots: void UpdateNonHitting(bool _value);

    /// \internal
    /// \brief Pointer to private data
    private: std::unique_ptr<VisualizeLidarPrivate> dataPtr;
  };
}
}
}

#endif