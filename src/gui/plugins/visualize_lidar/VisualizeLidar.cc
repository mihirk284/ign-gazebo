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

#include <ignition/msgs/lidar_sensor.pb.h>

#include <string>
#include <vector>

#include <sdf/Link.hh>
#include <sdf/Model.hh>

#include <ignition/common/Animation.hh>
#include <ignition/common/Console.hh>
#include <ignition/common/KeyFrame.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/common/Uuid.hh>
#include <ignition/common/VideoEncoder.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include <ignition/transport/Node.hh>

#include <ignition/gui/Conversions.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/gui/GuiEvents.hh"
#include "ignition/gazebo/rendering/RenderUtil.hh"


#include <ignition/rendering/RenderTypes.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/LidarVisual.hh>

#include "VisualizeLidar.hh"

namespace ignition
{
namespace gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE
{
  /// \brief Private data class for VisualizeLidar
  class VisualizeLidarPrivate
  {
    /// \brief Transport node
    public: transport::Node node;

    /// \brief Scene Pointer
    public: rendering::ScenePtr scene;

    /// \brief Pointer to LidarVisual
    public: rendering::LidarVisualPtr lidar;

    /// \brief Visual type for lidar visual
    public: rendering::LidarVisualType visualType;

    /// \brief LidarSensor message from sensor
    public: msgs::LidarSensor msg;

    /// \brief Current state of the checkbox
    public: bool checkboxState{false};

    /// \brief Topic name to subscribe
    public: std::string topic_name;

    /// \brief Message for visualizing contact positions
    public: ignition::msgs::Marker positionMarkerMsg;

    /// \brief Minimum range for the visual
    public: double minVisualRange;

    /// \brief Maximum range for the visual
    /// This range does not override the range set for the sensor
    /// it only determines the range for the visual
    public: double maxVisualRange;

    /// \brief Mutex for variable mutated by the checkbox and spinboxes
    /// callbacks.
    /// The variables are: msg, visualType, minVisualRange and
    /// maxVisualRange
    public: std::mutex serviceMutex;

    /// \brief Initialization flag
    public: bool initialized{false};

    /// \brief Name of the world
    public: std::string worldName;
  };
}
}
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
VisualizeLidar::VisualizeLidar()
  : GuiSystem(), dataPtr(new VisualizeLidarPrivate)
{
}

/////////////////////////////////////////////////
VisualizeLidar::~VisualizeLidar() = default;

/////////////////////////////////////////////////
void VisualizeLidar::LoadLidar()
{
  auto loadedEngNames = rendering::loadedEngines();
  if (loadedEngNames.empty())
    return;

  // assume there is only one engine loaded
  auto engineName = loadedEngNames[0];
  if (loadedEngNames.size() > 1)
  {
    igndbg << "More than one engine is available. "
      << "Grid config plugin will use engine ["
        << engineName << "]" << std::endl;
  }
  auto engine = rendering::engine(engineName);
  if (!engine)
  {
    ignerr << "Internal error: failed to load engine [" << engineName
      << "]. Grid plugin won't work." << std::endl;
    return;
  }

  if (engine->SceneCount() == 0)
    return;

  // assume there is only one scene
  // load scene
  auto scene = engine->SceneByIndex(0);
  if (!scene)
  {
    ignerr << "Internal error: scene is null." << std::endl;
    return;
  }

  if (!scene->IsInitialized() || scene->VisualCount() == 0)
  {
    return;
  }

  // Create grid
  igndbg << "Creating lidar visual" << std::endl;

  auto root = scene->RootVisual();
  this->dataPtr->lidar = scene->CreateLidarVisual();
  if (!this->dataPtr->lidar)
  {
    ignwarn << "Failed to create lidar, lidar visual plugin won't work."
            << std::endl;

    // If we get here, most likely the render engine and scene are fully loaded,
    // but they don't support grids. So stop trying.
    ignition::gui::App()->findChild<
        ignition::gui::MainWindow *>()->removeEventFilter(this);
    return;
  }
  root->AddChild(this->dataPtr->lidar);

  this->dataPtr->initialized = true;
}

/////////////////////////////////////////////////
void VisualizeLidar::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Visualize lidar";

  ignition::gui::App()->findChild<
    ignition::gui::MainWindow *>()->installEventFilter(this);
}

//////////////////////////////////////////////////
void VisualizeLidar::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("VisualizeLidar::Update");

  if (!this->dataPtr->initialized)
  {
    this->LoadLidar();
  }

  this->dataPtr->lidar->SetMinRange(this->dataPtr->minVisualRange);
  this->dataPtr->lidar->SetMinRange(this->dataPtr->maxVisualRange);
  this->dataPtr->lidar->SetVerticalRayCount(this->dataPtr->msg.vertical_samples());
  this->dataPtr->lidar->SetHorizontalRayCount(
                this->dataPtr->msg.horizontal_samples());
  this->dataPtr->lidar->SetMinHorizontalAngle(
            this->dataPtr->msg.horizontal_min_angle());
  this->dataPtr->lidar->SetMaxHorizontalAngle(
            this->dataPtr->msg.horizontal_max_angle());
  this->dataPtr->lidar->SetMinVerticalAngle(
            this->dataPtr->msg.vertical_min_angle());
  this->dataPtr->lidar->SetMaxVerticalAngle(
            this->dataPtr->msg.vertical_max_angle());
  this->dataPtr->lidar->SetType(this->dataPtr->visualType);

  // {
  //   std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  //   if (this->dataPtr->checkboxPrevState && !this->dataPtr->checkboxState)
  //   {
  //     // Remove the markers
  //     this->dataPtr->positionMarkerMsg.set_action(
  //       ignition::msgs::Marker::DELETE_ALL);
  //     this->dataPtr->forceMarkerMsg.set_action(
  //       ignition::msgs::Marker::DELETE_ALL);

  //     igndbg << "Removing markers..." << std::endl;
  //     this->dataPtr->node.Request(
  //       "/marker", this->dataPtr->positionMarkerMsg);
  //     this->dataPtr->node.Request(
  //       "/marker", this->dataPtr->forceMarkerMsg);

  //     // Change action in case checkbox is checked again
  //     this->dataPtr->positionMarkerMsg.set_action(
  //       ignition::msgs::Marker::ADD_MODIFY);
  //     this->dataPtr->forceMarkerMsg.set_action(
  //       ignition::msgs::Marker::ADD_MODIFY);
  //   }

  //   this->dataPtr->checkboxPrevState = this->dataPtr->checkboxState;
  //   if (!this->dataPtr->checkboxState)
  //     return;
  // }

  // // Only publish markers if enough time has passed
  // auto timeDiff =
  //   std::chrono::duration_cast<std::chrono::milliseconds>(_info.simTime -
  //   //this->dataPtr->lastMarkersUpdateTime);

  // if (timeDiff.count() < this->dataPtr->markerLifetime)
  //   return;

  // // Store simulation time
  // //this->dataPtr->lastMarkersUpdateTime = _info.simTime;

  // // todo(anyone) Get the contacts of the links that don't have a
  // // contact sensor

  // // Get the contacts and publish them
  // // Since we are setting a lifetime for the markers, we get all the
  // // contacts instead of getting news and removed ones

  // // Variable for setting the markers id through the iteration
  // int markerID = 1;
  // _ecm.Each<components::ContactSensorData>(
  //   [&](const Entity &,
  //       const components::ContactSensorData *_contacts) -> bool
  //   {
  //     for (const auto &contact : _contacts->Data().contact())
  //     {
  //       // todo(anyone) add information about contact normal, depth
  //       // and wrench
  //       for (int i = 0; i < contact.position_size(); ++i)
  //       {
  //         // Skip dummy data set by physics
  //         bool dataIsDummy = std::fabs(
  //           contact.position(i).x() -
  //           static_cast<float>(ignition::math::NAN_I)) < 0.001;
  //         if (dataIsDummy)
  //           return true;

  //         // Set marker id, poses and request service
  //         this->dataPtr->positionMarkerMsg.set_id(markerID);
  //         ignition::msgs::Set(this->dataPtr->positionMarkerMsg.mutable_pose(),
  //           ignition::math::Pose3d(contact.position(i).x(),
  //             contact.position(i).y(), contact.position(i).z(),
  //             0, 0, 0));

  //         // Placeholder for the force value (see todo comment above)
  //         double force = 1;

  //         double forceLength = this->dataPtr->forceScale * force;
  //         ignition::msgs::Set(this->dataPtr->forceMarkerMsg.mutable_scale(),
  //           ignition::math::Vector3d(0.02, 0.02, forceLength));

  //         // The position of the force marker is modified in order to place the
  //         // end of the cylinder in the contact point, not its middle point
  //         this->dataPtr->forceMarkerMsg.set_id(markerID++);
  //         ignition::msgs::Set(this->dataPtr->forceMarkerMsg.mutable_pose(),
  //           ignition::math::Pose3d(contact.position(i).x(),
  //             contact.position(i).y(),
  //             contact.position(i).z() + forceLength/2,
  //             0, 0, 0));

  //         this->dataPtr->node.Request(
  //           "/marker", this->dataPtr->positionMarkerMsg);
  //         this->dataPtr->node.Request(
  //           "/marker", this->dataPtr->forceMarkerMsg);
  //       }
  //     }
  //     return true;
  //   });
}

//////////////////////////////////////////////////
void VisualizeLidar::UpdateMinRange(double _minRange)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->minVisualRange = _minRange;
}

//////////////////////////////////////////////////
void VisualizeLidar::UpdateMaxRange(double _maxRange)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->maxVisualRange = _maxRange;
}

//////////////////////////////////////////////////
void VisualizeLidar::UpdateType(int _type)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  switch (_type) {
    case 0: this->dataPtr->visualType = 
                      rendering::LidarVisualType::LVT_NONE;
            break;
    case 1: this->dataPtr->visualType =
                      rendering::LidarVisualType::LVT_RAY_LINES;
            break;
    case 2: this->dataPtr->visualType =
                      rendering::LidarVisualType::LVT_POINTS;
            break;
    case 3: this->dataPtr->visualType =
                      rendering::LidarVisualType::LVT_TRIANGLE_STRIPS;
            break;
    default: this->dataPtr->visualType =
                      rendering::LidarVisualType::LVT_TRIANGLE_STRIPS;
            break;
  }
}

////////////////////////////////////////////////////////////////////////////////
void VisualizeLidar::UpdateTopicName(QString &_topic_name)
{
  std::lock_guard<std::mutex>(this->dataPtr->serviceMutex);
  this->dataPtr->topic_name = _topic_name.toStdString();

  // Reset visualization
  this->ResetLidarVisual();
  // Create new subscription
  this->dataPtr->node.Subscribe(this->dataPtr->topic_name,
                            &VisualizeLidar::OnScan, this);
}

////////////////////////////////////////////////////////////////////////////////
void VisualizeLidar::OnScan(const msgs::LidarSensor &_msg)
{
  std::lock_guard<std::mutex>(this->dataPtr->serviceMutex);
  this->dataPtr->msg = std::move(_msg);
}

////////////////////////////////////////////////////////////////////////////////
void VisualizeLidar::ResetLidarVisual()
{
  std::lock_guard<std::mutex>(this->dataPtr->serviceMutex);
  this->dataPtr->lidar->ClearPoints();
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::VisualizeLidar,
                    ignition::gui::Plugin)