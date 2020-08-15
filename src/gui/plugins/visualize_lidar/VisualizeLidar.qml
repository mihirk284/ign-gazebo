// /*
//  * Copyright (C) 2020 Open Source Robotics Foundation
//  *
//  * Licensed under the Apache License, Version 2.0 (the "License");
//  * you may not use this file except in compliance with the License.
//  * You may obtain a copy of the License at
//  *
//  *     http://www.apache.org/licenses/LICENSE-2.0
//  *
//  * Unless required by applicable law or agreed to in writing, software
//  * distributed under the License is distributed on an "AS IS" BASIS,
//  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  * See the License for the specific language governing permissions and
//  * limitations under the License.
//  *
// */
// import QtQuick 2.9
// import QtQuick.Controls 2.1
// import QtQuick.Dialogs 1.0
// import QtQuick.Layouts 1.3
// import "qrc:/qml"

// VisualizeLidar {
//   columns: 6
//   columnSpacing: 10
//   Layout.minimumWidth: 250
//   Layout.minimumHeight: 200
//   anchors.fill: parent
//   anchors.leftMargin: 10
//   anchors.rightMargin: 10

//   // Left spacer
//   Item {
//     Layout.columnSpan: 1
//     Layout.rowSpan: 15
//     Layout.fillWidth: true
//   }

  // ComboBox {
  //   id: typeCombo
  //   Layout.fillWidth: true
  //   currentIndex: 3
  //   model: ["None", "Rays", "Points", "Triangles" ]
  //   onCurrentIndexChanged: {
  //     if (currentIndex < 0) {
  //       return;
  //     }
  //     VisualizeLidar.UpdateType(currentIndex);
  //   }
  // }

//   // Right spacer
//   Item {
//     Layout.columnSpan: 1
//     Layout.rowSpan: 15
//     Layout.fillWidth: true
//   }

//   Text {
//     Layout.columnSpan: 2
//     id: minRangeText
//     color: "dimgrey"
//     text: "Minimum Range (m)"
//   }

//   IgnSpinBox {
//     Layout.columnSpan: 2
//     Layout.fillWidth: true
//     id: maxRange
//     maximumValue: 100
//     minimumValue: 0.1
//     value: 10
//     decimals: 2
//     stepSize: 0.50
//     onEditingFinished: VisualizeLidar.UpdateMinRange(maxRange.value)
//   }

//   Text {
//     Layout.columnSpan: 2
//     id: maxRangeText
//     color: "dimgrey"
//     text: "Maximum Range (m)"
//   }

//   IgnSpinBox {
//     Layout.columnSpan: 2
//     Layout.fillWidth: true
//     id: minRange
//     maximumValue: 100
//     minimumValue: 0.1
//     value: 10
//     decimals: 2
//     stepSize: 0.50
//     onEditingFinished: VisualizeLidar.UpdateMaxRange(minRange.value)
//   }

  // ComboBox {
  //   id: combo
  //   Layout.fillWidth: true
  //   model: LaserScanDisplay.topicList
  //   currentIndex: 0
  //   editable: true
  //   editText: currentText
  //   displayText: currentText
  //   onCurrentIndexChanged: {
  //     if (currentIndex < 0) {
  //       return;
  //     }

  //     VisualizeLidar.UpdateTopicName(textAt(currentIndex));
  //   }

  //   Component.onCompleted: {
  //     combo.editText = "/scan"
  //     combo.displayText = "/scan"
  //   }

  //   Connections {
  //     target: LaserScanDisplay
  //     onSetCurrentIndex: {
  //       combo.currentIndex = index
  //     }
  //   }
  // }
// }

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
import QtQuick 2.9
import QtQuick.Controls 2.1
import QtQuick.Dialogs 1.0
import QtQuick.Layouts 1.3
import "qrc:/qml"

GridLayout {
  columns: 6
  columnSpacing: 10
  Layout.minimumWidth: 250
  Layout.minimumHeight: 200
  anchors.fill: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10

  // Left spacer
  Item {
    Layout.columnSpan: 1
    Layout.rowSpan: 15
    Layout.fillWidth: true
  }

  CheckBox {
    Layout.alignment: Qt.AlignHCenter
    id: visualizeContacts
    Layout.columnSpan: 4
    text: qsTr("Show Non Hitting Rays")
    checked: false
    onClicked: {
      VisualizeLidar.UpdateNonHitting(checked)
    }
  }

  // Right spacer
  Item {
    Layout.columnSpan: 1
    Layout.rowSpan: 15
    Layout.fillWidth: true
  }

  Text {
    Layout.columnSpan: 2
    id: minRangeText
    color: "dimgrey"
    text: "Minimum Range (m)"
  }

  IgnSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: minrange
    maximumValue: 10.0
    minimumValue: 0.01
    value: 0.10
    decimals: 2
    stepSize: 0.1
    onEditingFinished: VisualizeLidar.UpdateMinRange(minrange.value)
  }

  Text {
    Layout.columnSpan: 2
    id: maxRangeText
    color: "dimgrey"
    text: "Maximum Range (m)"
  }

  IgnSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: maxrange
    maximumValue: 100.0
    minimumValue: 1.0
    value: 10.0
    decimals: 2
    stepSize: 0.1
    onEditingFinished: VisualizeLidar.UpdateMaxRange(maxrange.value)
  }

  Text {
    Layout.columnSpan: 2
    id: updatePeriodText
    color: "dimgrey"
    text: "Visual Type"
  }

  ComboBox {
    id: typeCombo
    Layout.fillWidth: true
    currentIndex: 3
    model: ["None", "Rays", "Points", "Triangles" ]
    onCurrentIndexChanged: {
      if (currentIndex < 0) {
        return;
      }
      VisualizeLidar.UpdateType(typeCombo.currentIndex);
      // VisualizeLidar.UpdateTopicName();
    }
  }

  Text {
    Layout.columnSpan: 2
    id: lidarMsgText
    color: "dimgrey"
    text: "Lidar Message"
  }

  // ComboBox {
  //   id: msgText
  //   Layout.fillWidth: true
  //   currentIndex: 3
  //   model: ["None", "Rays", "Points", "/lidar" ]
  //   onCurrentIndexChanged: {
  //     if (currentIndex < 0) {
  //       return;
  //     }
  //     VisualizeLidar.UpdateTopicName(textAt(currentIndex));
  //   }
  // }

  ComboBox {
    id: combo
    Layout.fillWidth: true
    model: LaserScanDisplay.topicList
    currentIndex: 0
    editable: true
    editText: currentText
    displayText: currentText
    onCurrentIndexChanged: {
      if (currentIndex < 0) {
        return;
      }

      VisualizeLidar.UpdateTopicName(textAt(currentText));
    }
  }
}
