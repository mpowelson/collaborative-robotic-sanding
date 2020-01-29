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

#ifndef CRS_GUI_WIDGETS_CRS_APPLICATION_WIDGET_H
#define CRS_GUI_WIDGETS_CRS_APPLICATION_WIDGET_H

//#include <QSqlTableModel>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <string>
//#include <ros/node_handle.h>
//#include <ros/publisher.h>
//#include <ros/service.h>

//#include <opp_database/opp_database_interface_cpp.h>
//#include <opp_msgs/Part.h>

class QListWidgetItem;

namespace Ui
{
class CRSApplication;
}

namespace crs_gui
{

class CRSApplicationWidget : public QWidget
{
  Q_OBJECT
public:
  CRSApplicationWidget(rclcpp::Node::SharedPtr node,
                       QWidget* parent = nullptr,
                       std::string database_directory = std::string(std::getenv("HOME")) + "/.local/share/offline_generated_paths");

protected Q_SLOTS:

  void setVisualizationFrame(const QString& text);

  // Parts Page
  void browseForMeshResource();
  void loadMeshFromResource();

  void refreshPartsList();
  void onModelSelectionChanged(QListWidgetItem* current, QListWidgetItem* previous);
  void loadSelectedModel();
  void saveModel();

  // Jobs Page
  void newJob();
  void loadJobsFromDatabase();
  void onJobSelectionChanged(QListWidgetItem* current, QListWidgetItem* previous);
  void loadSelectedJob();
  void saveJob();

  // Database Management Page
  void showPartFromDatabase();
  void deletePart();
  void deleteJob();
  void refresh();

private:
  void clear();

  bool loadMesh();

  void setModelTabsEnabled(bool enabled);

  void setJobTabsEnabled(bool enabled, bool first_enabled = true);

  Ui::CRSApplication* ui_;

//  TouchPointEditorWidget* touch_point_editor_;
//  TouchPointEditorWidget* verification_point_editor_;
//  ToolPathEditorWidget* tool_path_editor_;

  rclcpp::Node::SharedPtr node_;
//  ros::Publisher pub_;
//  ros::ServiceClient save_model_client_;
//  ros::ServiceClient save_job_client_;
//  ros::ServiceClient get_all_models_client_;

  std::string marker_frame_;

//  std::vector<opp_msgs::Part> existing_parts_;
//  std::vector<opp_msgs::Job> existing_jobs_;

  std::string mesh_resource_;
  uint32_t generated_model_id_;

  std::string database_directory_;

};

}  // namespace opp_gui

#endif  // OPP_GUI_WIDGETS_TOOL_PATH_PLANNER_WIDGET_H
