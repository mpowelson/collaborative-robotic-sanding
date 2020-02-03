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

#ifndef CRS_GUI_WIDGETS_STATE_MACHINE_INTERFACE_WIDGET_H
#define CRS_GUI_WIDGETS_STATE_MACHINE_INTERFACE_WIDGET_H

#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <crs_msgs/srv/get_available_actions.hpp>
#include <crs_msgs/srv/execute_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

namespace Ui
{
class StateMachineInterface;
}

namespace crs_gui
{


class StateMachineInterfaceWidget : public QWidget
{
  Q_OBJECT
public:
  StateMachineInterfaceWidget(rclcpp::Node::SharedPtr node,
                       QWidget* parent = nullptr);

protected Q_SLOTS:

  void onSMQuery();
  void onSMApply();
  void onSMApprove();
  void onSMCancel();

protected:
  Ui::StateMachineInterface* ui_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_state_sub_;
  void currentStateCB(const std_msgs::msg::String::ConstSharedPtr current_state);

  rclcpp::Client<crs_msgs::srv::GetAvailableActions>::SharedPtr get_available_actions_client_;
  rclcpp::Client<crs_msgs::srv::ExecuteAction>::SharedPtr execute_action_client_;
};

}  // namespace crs_gui

#endif  // crs_GUI_WIDGETS_APPLICATION_WIDGET_BASE_H
