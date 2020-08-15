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

#include <ignition/msgs/laserscan.pb.h>

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

    /// \brief LaserScan message from sensor
    public: msgs::LaserScan msg;

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

    // \brief lidar visual display dirty flag
    public: bool visualDirty{false};

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
  this->dataPtr->node.Subscribe("/lidar",
                            &VisualizeLidar::OnScan, this);
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

/////////////////////////////////////////////////
bool VisualizeLidar::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == ignition::gazebo::gui::events::Render::kType)
  {
    // This event is called in Scene3d's RenderThread, so it's safe to make
    // rendering calls here

    if (!this->dataPtr->initialized)
    {
      this->LoadLidar();
    }

    this->dataPtr->lidar->Update();
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

//////////////////////////////////////////////////
void VisualizeLidar::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("VisualizeLidar::Update");

}

//////////////////////////////////////////////////
void VisualizeLidar::UpdateMinRange(double _minRange)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->minVisualRange = _minRange;
  this->dataPtr->lidar->SetMinRange(this->dataPtr->minVisualRange);
}

//////////////////////////////////////////////////
void VisualizeLidar::UpdateMaxRange(double _maxRange)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->serviceMutex);
  this->dataPtr->maxVisualRange = _maxRange;
  this->dataPtr->lidar->SetMaxRange(this->dataPtr->maxVisualRange);
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
  this->dataPtr->lidar->SetType(this->dataPtr->visualType);
}

//////////////////////////////////////////////////
void VisualizeLidar::UpdateTopicName()
{
  std::lock_guard<std::mutex>(this->dataPtr->serviceMutex);
  this->dataPtr->node.Unsubscribe(this->dataPtr->topic_name);

  this->dataPtr->topic_name = "/lidar"; //_topic_name.toStdString();
  // Reset visualization
  this->ResetLidarVisual();
  // Create new subscription
  this->dataPtr->node.Subscribe(this->dataPtr->topic_name,
                            &VisualizeLidar::OnScan, this);
  
  std::cout << "SUBSCRIBED TO TOPIC " << this->dataPtr->topic_name << std::endl;
}

//////////////////////////////////////////////////
void VisualizeLidar::UpdateNonHitting(bool _value)
{
  std::lock_guard<std::mutex>(this->dataPtr->serviceMutex);
  this->dataPtr->lidar->SetDisplayNonHitting(_value);
}

//////////////////////////////////////////////////
void VisualizeLidar::OnScan(const msgs::LaserScan &_msg)
{
  std::lock_guard<std::mutex>(this->dataPtr->serviceMutex);
  this->dataPtr->msg = std::move(_msg);
  ignition::math::Pose3d testPose = msgs::Convert(this->dataPtr->msg.world_pose());
  this->dataPtr->lidar->SetWorldPosition(testPose.Pos());
  this->dataPtr->lidar->SetWorldRotation(testPose.Rot());
  this->dataPtr->lidar->SetVerticalRayCount(this->dataPtr->msg.vertical_count());
  this->dataPtr->lidar->SetHorizontalRayCount(this->dataPtr->msg.count());
  this->dataPtr->lidar->SetMinHorizontalAngle(this->dataPtr->msg.angle_min());
  this->dataPtr->lidar->SetMaxHorizontalAngle(this->dataPtr->msg.angle_max());
  this->dataPtr->lidar->SetMinVerticalAngle(this->dataPtr->msg.vertical_angle_min());
  this->dataPtr->lidar->SetMaxVerticalAngle(this->dataPtr->msg.vertical_angle_max());
  this->dataPtr->lidar->SetPoints(std::vector<double>(
            this->dataPtr->msg.ranges().begin(),
            this->dataPtr->msg.ranges().end()));

  this->dataPtr->visualDirty = true;
}

//////////////////////////////////////////////////
void VisualizeLidar::ResetLidarVisual()
{
  std::lock_guard<std::mutex>(this->dataPtr->serviceMutex);
  this->dataPtr->lidar->ClearPoints();
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::VisualizeLidar,
                    ignition::gui::Plugin)