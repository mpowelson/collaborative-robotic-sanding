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

#include "crs_gui/widgets/part_selection_widget.h"

#include <map>
#include <regex>
#include <string>

#include <QFileDialog>
#include <QMessageBox>
#include <QTableView>

#include <boost/filesystem.hpp>

#include "ui_part_selection.h"

const static std::string MESH_MARKER_TOPIC = "mesh_marker";
const bool USE_DATABASE=false;

namespace crs_gui
{
PartSelectionWidget::PartSelectionWidget(rclcpp::Node::SharedPtr node,
                                           QWidget* parent,
                                           std::string database_directory)
  : QWidget(parent), ui_(new Ui::PartSelection), node_(node), database_directory_(database_directory)
{
  ui_->setupUi(this);

 // Connect the signals and slots
  connect(ui_->push_button_refresh_parts_list, &QPushButton::clicked, this, &PartSelectionWidget::refreshPartsList);
  refreshPartsList();
  connect(
      ui_->list_widget_parts, &QListWidget::currentItemChanged, this, &PartSelectionWidget::onModelSelectionChanged);
  connect(ui_->push_button_load_selected_part, &QPushButton::clicked, this, &PartSelectionWidget::loadSelectedPart);
  //  connect(ui_->push_button_save_entry, &QPushButton::clicked, this, &ToolPathPlannerWidget::saveModel);

  //  // Signals & slots for the buttons on job definition page
  //  connect(ui_->push_button_new_job, &QPushButton::clicked, this, &ToolPathPlannerWidget::newJob);
  //  connect(ui_->push_button_load_jobs_from_database,
  //          &QPushButton::clicked,
  //          this,
  //          &ToolPathPlannerWidget::loadJobsFromDatabase);
  //  connect(ui_->list_widget_jobs, &QListWidget::currentItemChanged, this,
  //  &ToolPathPlannerWidget::onJobSelectionChanged); connect(ui_->push_button_load_selected_job, &QPushButton::clicked,
  //  this, &ToolPathPlannerWidget::loadSelectedJob); connect(ui_->push_button_save_job, &QPushButton::clicked, this,
  //  &ToolPathPlannerWidget::saveJob);

  //  // Signals & Slots for the Buttons on database management page
  //  connect(ui_->push_button_show_part, &QPushButton::clicked, this, &ToolPathPlannerWidget::showPartFromDatabase);
  //  connect(ui_->push_button_suppress_part, &QPushButton::clicked, this, &ToolPathPlannerWidget::deletePart);
  //  connect(ui_->push_button_suppress_job, &QPushButton::clicked, this, &ToolPathPlannerWidget::deleteJob);
  //  connect(ui_->push_button_refresh_parts, &QPushButton::clicked, this, &ToolPathPlannerWidget::refresh);
  //  connect(ui_->push_button_refresh_jobs, &QPushButton::clicked, this, &ToolPathPlannerWidget::refresh);

    // Add a publisher for the mesh marker
  marker_pub_.reset();
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(MESH_MARKER_TOPIC, 1);
}

void PartSelectionWidget::setVisualizationFrame(const QString& text)
{
//  marker_frame_ = text.toStdString();
//  touch_point_editor_->setMarkerFrame(marker_frame_);
//  verification_point_editor_->setMarkerFrame(marker_frame_);
//  tool_path_editor_->setMarkerFrame(marker_frame_);
}

// Parts Page
void PartSelectionWidget::browseForMeshResource()
{
//  QString filename = QFileDialog::getOpenFileName(this, "Load Model", "", "Mesh Files (*.stl *.ply *.obj)");
//  if (filename.isEmpty())
//  {
//    ROS_WARN_STREAM(__func__ << ": Empty filename");
//    return;
//  }

//  ui_->line_edit_model_filename->setText(filename);
//  loadMeshFromResource();
//  return;
}

void PartSelectionWidget::loadMeshFromResource()
{
//  // Get the filename and package of the model
//  std::string filename = ui_->line_edit_model_filename->text().toStdString();
//  if (filename.empty())
//  {
//    QMessageBox::warning(this, "Input Error", "Model filename or package name not specified");
//    return;
//  }

//  // Construct the mesh resource name using the package and filename
//  std::vector<std::string> file_extensions = { ".stl", ".ply", ".obj" };

//  mesh_resource_.clear();
//  for (const std::string& ext : file_extensions)
//  {
//    std::regex rgx(".*" + ext + "$");
//    std::smatch match;
//    if (std::regex_search(filename, match, rgx))
//    {
//      mesh_resource_ = "file://" + filename;
//      break;
//    }
//  }

//  if (mesh_resource_.empty())
//  {
//    std::string message = "Invalid mesh resource file extension. Acceptable inputs are: ";
//    for (const std::string& ext : file_extensions)
//      message += ext + " ";

//    QMessageBox::warning(this, "Input Error", QString(message.c_str()));
//    return;
//  }
//  ROS_INFO_STREAM("Attempting to load mesh from resource: '" << mesh_resource_ << "'");

//  if (!loadMesh())
//    return;
}

void PartSelectionWidget::refreshPartsList()
{
  using namespace boost::filesystem;
  std::vector<path> part_dirs;
  for ( directory_iterator itr(database_directory_); itr!=directory_iterator(); itr++)
  {
    if (is_directory(itr->path()))
      part_dirs.push_back(itr->path());
  }

  // Retrieve part info from the database
  if (!part_dirs.size())
  {
    // If the function failed, create a warning pop-up box.
    std::string message = "Found no parts in " + database_directory_;
    QMessageBox::warning(this, "Database Communication Error", QString::fromStdString(message));
  }
  else
  {
    ui_->list_widget_parts->clear();
    for (auto& part : part_dirs)
    {
      // Gui display listing parts to user
      QListWidgetItem* item = new QListWidgetItem(QString::fromStdString(part.stem().string()));
      item->setData(Qt::ItemDataRole::UserRole, QVariant(QString::fromStdString(part.stem().string())));
      ui_->list_widget_parts->addItem(item);
    }
  }
  return;
}

void PartSelectionWidget::onModelSelectionChanged(QListWidgetItem* current, QListWidgetItem*)
{
  // Change the description display based on which part is selected
  if (current != nullptr)
  {
    ui_->text_edit_part_description->setText(current->data(Qt::ItemDataRole::UserRole).toString());
  }
}

void PartSelectionWidget::loadSelectedPart()
{
//  int row = ui_->list_widget_parts->currentRow();
//  if (row >= 0 && row < static_cast<int>(existing_parts_.size()))
//  {
//    // Get the selected part information
//    const opp_msgs::Part& part = existing_parts_[static_cast<std::size_t>(row)];
//    generated_model_id_ = part.id;
//    mesh_resource_ = part.mesh_resource;

//    if (!loadMesh())
//      return;

//    // Update the UI and widgets with all part information
//    ui_->line_edit_model_name->setText(QString::fromStdString(part.name));
//    ui_->plain_text_edit_model_description->setPlainText(QString::fromStdString(part.description));

//    touch_point_editor_->setPoints(part.touch_points);
//    verification_point_editor_->setPoints(part.verification_points);

//    setJobTabsEnabled(false, true);
//    loadJobsFromDatabase();
//  }
//  else
//  {
//    QMessageBox::warning(this, "Input Error", "Make a selection in the parts list");
//  }
}

void PartSelectionWidget::saveModel()
{
//  // Verify that the user intended to save the part
//  QMessageBox::StandardButton button =
//      QMessageBox::question(this, "Save Part to Database", "Proceed with adding the defined part to the database?");
//  if (button == QMessageBox::StandardButton::No)
//    return;

//  // Get the relevant model information to save and verify it exists
//  std::string model_name = ui_->line_edit_model_name->text().toStdString();
//  std::string model_description = ui_->plain_text_edit_model_description->toPlainText().toStdString();
//  if (model_name.empty() || model_description.empty())
//  {
//    QMessageBox::warning(this, "Input Error", "Model ID or description field(s) is empty");
//    return;
//  }

//  // Get the touch points and verification points, and make sure there are
//  // at least 3 of each.  (This requirement could probably be relaxed.)
//  using TouchPointMap = std::map<std::string, opp_msgs::TouchPoint>;
//  TouchPointMap touch_points = touch_point_editor_->getPoints();
//  TouchPointMap verification_points = verification_point_editor_->getPoints();
//  if (touch_points.size() < 3 || verification_points.size() < 3)
//  {
//    QMessageBox::warning(
//        this, "Invalid Model Definition", "Ensure at least 3 touch points and 3 verification points have been defined");
//    return;
//  }

//  // Fill out the part struct with input information
//  opp_msgs::Part part;
//  part.name = model_name;
//  part.description = model_description;
//  part.mesh_resource = mesh_resource_;

//  // Copy the touch points
//  part.touch_points.reserve(touch_points.size());
//  for (const std::pair<const std::string, opp_msgs::TouchPoint>& pair : touch_points)
//  {
//    part.touch_points.push_back(pair.second);
//  }

//  // Copy the verification points
//  part.verification_points.reserve(verification_points.size());
//  for (const std::pair<const std::string, opp_msgs::TouchPoint>& pair : verification_points)
//  {
//    part.verification_points.push_back(pair.second);
//  }

//  // Save the model to the database
//  std::string error_msg;
//  long int key = database_.addPartToDatabase(part, error_msg);
//  if (key < 0)
//  {
//    // If the function failed, warn the user.
//    std::string message = "Failed to add part to database, received error: " + error_msg;
//    QMessageBox::warning(this, "Database Communication Error", QString(message.c_str()));
//  }
//  else
//  {
//    // Save the auto generated model id
//    generated_model_id_ = key;

//    // If the save is successful, allow the user to add job data for the part
//    setJobTabsEnabled(true, true);

//    // Make sure the database lists are updated
//    refresh();
//    loadModelsFromDatabase();
//    loadJobsFromDatabase();
//  }
//  return;
}


// Private functions
void PartSelectionWidget::clear()
{
//  // Clear the data in the list editor widgets
//  touch_point_editor_->clear();
//  verification_point_editor_->clear();
//  tool_path_editor_->clear();

//  // Clear the text input data about the model
//  ui_->line_edit_model_name->clear();
//  ui_->plain_text_edit_model_description->clear();

//  // Clear the text input data about the job
//  ui_->line_edit_job_name->clear();
//  ui_->plain_text_edit_job_description->clear();
}

bool PartSelectionWidget::loadMesh()
{
//  // Attempt to load this file into a shape_msgs/Mesh type
//  shape_msgs::Mesh mesh;
//  if (!utils::getMeshMsgFromResource(mesh_resource_, mesh))
//  {
//    std::string message = "Failed to load mesh from resource: '" + mesh_resource_ + "'";
//    QMessageBox::warning(this, "Input Error", message.c_str());
//    return false;
//  }

//  // Clear the touch point and tool path editors' data before continuing
//  clear();

//  // Initialize the tool path editor with the mesh
//  tool_path_editor_->init(mesh);

//  // Enable the models tabs but not the jobs tabs
//  setModelTabsEnabled(true);
//  setJobTabsEnabled(false, false);

//  // Publish the mesh marker
//  visualization_msgs::Marker mesh_marker =
//      utils::createMeshMarker(0, "mesh", Eigen::Isometry3d::Identity(), marker_frame_, mesh_resource_);

//  pub_.publish(mesh_marker);

//  return true;
}

void PartSelectionWidget::setModelTabsEnabled(bool enabled)
{
//  for (int i = 1; i < ui_->tool_box_model_editor->count(); ++i)
//  {
//    ui_->tool_box_model_editor->setItemEnabled(i, enabled);
//  }

//  ui_->frame_define_touch_off_points->setEnabled(enabled);
//  ui_->frame_define_verification_points->setEnabled(enabled);
}

void PartSelectionWidget::setJobTabsEnabled(bool enabled, bool first_enabled)
{
//  for (int i = 1; i < ui_->tool_box_job_editor->count(); ++i)
//  {
//    ui_->tool_box_job_editor->setItemEnabled(i, enabled);
//  }
//  if (ui_->tool_box_job_editor->count() > 0)
//  {
//    ui_->tool_box_job_editor->setItemEnabled(0, first_enabled);
//    ui_->push_button_new_job->setEnabled(first_enabled);
//    ui_->push_button_load_jobs_from_database->setEnabled(first_enabled);
//    ui_->push_button_load_selected_job->setEnabled(first_enabled);
//  }
//  ui_->frame_define_toolpaths->setEnabled(enabled);
}

}  // namespace opp_gui
