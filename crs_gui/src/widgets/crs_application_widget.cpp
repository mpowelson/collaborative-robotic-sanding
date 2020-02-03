/*
 * Copyright 2019 Southwest Research Institute
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

#include "ui_crs_application.h"

#include <atomic>
#include <QMessageBox>
#include <QStateMachine>
#include <QProgressBar>
#include <QProgressDialog>

#include <chrono>

#include <crs_gui/widgets/crs_application_widget.h>

#include <crs_gui/widgets/part_selection_widget.h>
#include <crs_gui/widgets/polygon_area_selection_widget.h>

const static std::string CURRENT_STATE_TOPIC = "current_state";
const static std::string GET_AVAILABLE_ACTIONS = "get_available_actions";
const static std::string EXECUTE_ACTION = "execute_action";
const static double WAIT_FOR_SERVICE_PERIOD = 1.0;

namespace crs_gui
{
CRSApplicationWidget::CRSApplicationWidget(rclcpp::Node::SharedPtr node,
                                           QWidget* parent,
                                           std::string database_directory)
  : QWidget(parent)
  , ui_(new Ui::CRSApplication)
  , part_selector_widget_(new PartSelectionWidget())
  , area_selection_widget_(new PolygonAreaSelectionWidget(node, "", ""))
  , node_(node)

{
  ui_->setupUi(this);

  // Initialize state machine interfaces
  auto current_state_cb = std::bind(&CRSApplicationWidget::currentStateCB, this, std::placeholders::_1);
  current_state_sub_ = node_->create_subscription<std_msgs::msg::String>(CURRENT_STATE_TOPIC, 1, current_state_cb);
  get_available_actions_client_ = node_->create_client<crs_msgs::srv::GetAvailableActions>(GET_AVAILABLE_ACTIONS);
  execute_action_client_ = node_->create_client<crs_msgs::srv::ExecuteAction>(EXECUTE_ACTION);


  //  // Add the widgets to the UI
  ui_->vertical_layout_part_selector->addWidget(part_selector_widget_);
  ui_->vertical_layout_area_selection->addWidget(area_selection_widget_);

  connect(part_selector_widget_, &PartSelectionWidget::partSelected, this, &CRSApplicationWidget::onPartSelected);

  // TODO: Make state machine interaction a standalone widget
  connect(ui_->push_button_sm_apply, &QPushButton::clicked, this, &CRSApplicationWidget::onSMApply);
  connect(ui_->push_button_sm_query, &QPushButton::clicked, this, &CRSApplicationWidget::onSMQuery);
  connect(ui_->push_button_sm_cancel, &QPushButton::clicked, this, &CRSApplicationWidget::onSMCancel);
  connect(ui_->push_button_sm_approve, &QPushButton::clicked, this, &CRSApplicationWidget::onSMApprove);
}

void CRSApplicationWidget::currentStateCB(const std_msgs::msg::String::ConstSharedPtr current_state)
{
  std::cout << current_state << std::endl;
}

void CRSApplicationWidget::onPartSelected(const std::string selected_part) { std::cout << selected_part << std::endl; }

// State machine button callbacks
void CRSApplicationWidget::onSMApply()
{
  // Specify service request to get all user actions in current state
  auto request = std::make_shared<crs_msgs::srv::ExecuteAction::Request>();
  request->action_id = ui_->combo_box_sm_available_actions->currentText().toStdString();

  // Wait if service is not available
  if (!execute_action_client_->wait_for_service(std::chrono::duration<double>(WAIT_FOR_SERVICE_PERIOD)))
  {
    RCLCPP_ERROR(node_->get_logger(), "service not available");
    return;
  }
  // Send request and wait for result
  auto result = execute_action_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call service %s", GET_AVAILABLE_ACTIONS.c_str());
  }
}

void CRSApplicationWidget::onSMQuery()
{
  // Specify service request to get all user actions in current state
  auto request = std::make_shared<crs_msgs::srv::GetAvailableActions::Request>();
  request->search_pattern = request->PATTERN_USER_ACTIONS;

  // Wait if service is not available
  if (!get_available_actions_client_->wait_for_service(std::chrono::duration<double>(WAIT_FOR_SERVICE_PERIOD)))
  {
    RCLCPP_ERROR(node_->get_logger(), "service not available");
    return;
  }
  // Send request and wait for result
  auto result = get_available_actions_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    // Convert ROS msg to QStringList and update combo box
    QStringList available_actions;
    for (std::string action : result.get()->action_ids)
      available_actions.push_back(QString::fromUtf8(action.c_str()));
    ui_->combo_box_sm_available_actions->addItems(available_actions);
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call service %s", GET_AVAILABLE_ACTIONS.c_str());
  }
}

void CRSApplicationWidget::onSMCancel()
{
  // Specify service request to get all user actions in current state
  auto request = std::make_shared<crs_msgs::srv::ExecuteAction::Request>();
  request->action_id = "user_rejects";

  // Wait if service is not available
  if (!execute_action_client_->wait_for_service(std::chrono::duration<double>(WAIT_FOR_SERVICE_PERIOD)))
  {
    RCLCPP_ERROR(node_->get_logger(), "service not available");
    return;
  }
  // Send request and wait for result
  auto result = execute_action_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call service %s", GET_AVAILABLE_ACTIONS.c_str());
  }
}

void CRSApplicationWidget:: onSMApprove()
{
  // Specify service request to get all user actions in current state
  auto request = std::make_shared<crs_msgs::srv::ExecuteAction::Request>();
  request->action_id = "user_approves";

  // Wait if service is not available
  if (!execute_action_client_->wait_for_service(std::chrono::duration<double>(WAIT_FOR_SERVICE_PERIOD)))
  {
    RCLCPP_ERROR(node_->get_logger(), "service not available");
    return;
  }
  // Send request and wait for result
  auto result = execute_action_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call service %s", GET_AVAILABLE_ACTIONS.c_str());
  }
}

}  // namespace crs_gui
