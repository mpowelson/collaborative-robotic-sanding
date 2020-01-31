/*
 * Copyright 2020 Southwest Research Institute
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
 */

#ifndef CRS_GUI_WIDGETS_PART_SELECTION_WIDGET_H
#define CRS_GUI_WIDGETS_PART_SELECTION_WIDGET_H


#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <string>

class QListWidgetItem;

namespace Ui
{
class PartSelection;
}

namespace crs_gui
{

class PartSelectionWidget : public QWidget
{
  Q_OBJECT
public:
  PartSelectionWidget(rclcpp::Node::SharedPtr node,
                       QWidget* parent = nullptr,
                       std::string database_directory = std::string(std::getenv("HOME")) + "/.local/share/offline_generated_paths");

protected Q_SLOTS:

  void setVisualizationFrame(const QString& text);

  // Parts Page
  void browseForMeshResource();
  void loadMeshFromResource();

  void refreshPartsList();
  void onModelSelectionChanged(QListWidgetItem* current, QListWidgetItem* previous);
  void loadSelectedPart();
  void saveModel();


private:
  void clear();

  bool loadMesh();

  void setModelTabsEnabled(bool enabled);

  void setJobTabsEnabled(bool enabled, bool first_enabled = true);

  Ui::PartSelection* ui_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::string marker_frame_;

  std::string mesh_resource_;
  uint32_t generated_model_id_;

  std::string database_directory_;

};

}  // namespace opp_gui

#endif  // OPP_GUI_WIDGETS_TOOL_PATH_PLANNER_WIDGET_H
