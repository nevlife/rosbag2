// Copyright 2026 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROSBAG2_TRANSPORT__DESCRIBED_TYPE_SUPPORT_HPP_
#define ROSBAG2_TRANSPORT__DESCRIBED_TYPE_SUPPORT_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/message_info.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/publisher_base.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "type_description_interfaces/msg/type_description.hpp"
#include "type_description_interfaces/msg/type_source.hpp"

#include "rosbag2_transport/visibility_control.hpp"

namespace rosbag2_transport
{

/// Type support for a message type known only from a ros2msg definition, such as one stored in a
/// bag. It carries the type name, hash and description that an rmw needs to publish and take
/// serialized messages, without the type's generated packages. It cannot convert messages.
class ROSBAG2_TRANSPORT_PUBLIC DescribedTypeSupport
{
public:
  /// \param type Type name, e.g. "sensor_msgs/msg/Image".
  /// \param definition ros2msg text of the type followed by its dependencies, as rosbag2 stores
  /// it.
  /// \param type_hash Hash recorded with the definition ("RIHS01_..."), or empty to use the hash
  /// computed from the definition.
  /// \throws std::runtime_error if the definition cannot be parsed or a dependency is missing.
  DescribedTypeSupport(
    const std::string & type, const std::string & definition, const std::string & type_hash = "");
  ~DescribedTypeSupport();

  DescribedTypeSupport(const DescribedTypeSupport &) = delete;
  DescribedTypeSupport & operator=(const DescribedTypeSupport &) = delete;

  const rosidl_message_type_support_t & type_support() const;

  /// Hash computed from the definition, which may differ from the recorded one.
  const rosidl_type_hash_t & computed_type_hash() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/// Publisher of serialized messages of a DescribedTypeSupport type.
class ROSBAG2_TRANSPORT_PUBLIC DescribedTypePublisher : public rclcpp::PublisherBase
{
public:
  DescribedTypePublisher(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    std::shared_ptr<const DescribedTypeSupport> type_support,
    const std::string & topic_name,
    const rclcpp::QoS & qos);

  void publish(const rclcpp::SerializedMessage & message);

private:
  std::shared_ptr<const DescribedTypeSupport> type_support_;
};

/// Subscription to serialized messages of a DescribedTypeSupport type.
class ROSBAG2_TRANSPORT_PUBLIC DescribedTypeSubscription : public rclcpp::SubscriptionBase
{
public:
  using Callback = std::function<void (
        std::shared_ptr<const rclcpp::SerializedMessage>, const rclcpp::MessageInfo &)>;

  DescribedTypeSubscription(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    std::shared_ptr<const DescribedTypeSupport> type_support,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    Callback callback,
    const rclcpp::SubscriptionOptions & options);

  std::shared_ptr<void> create_message() override;
  std::shared_ptr<rclcpp::SerializedMessage> create_serialized_message() override;
  void handle_message(std::shared_ptr<void> &, const rclcpp::MessageInfo &) override;
  void handle_serialized_message(
    const std::shared_ptr<rclcpp::SerializedMessage> & message,
    const rclcpp::MessageInfo & message_info) override;
  void handle_loaned_message(void *, const rclcpp::MessageInfo &) override;
  void return_message(std::shared_ptr<void> & message) override;
  void return_serialized_message(std::shared_ptr<rclcpp::SerializedMessage> & message) override;
  rclcpp::dynamic_typesupport::DynamicMessageType::SharedPtr
  get_shared_dynamic_message_type() override;
  rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr get_shared_dynamic_message() override;
  rclcpp::dynamic_typesupport::DynamicSerializationSupport::SharedPtr
  get_shared_dynamic_serialization_support() override;
  rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr create_dynamic_message() override;
  void return_dynamic_message(rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr &) override;
  void handle_dynamic_message(
    const rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr &,
    const rclcpp::MessageInfo &) override;

private:
  std::shared_ptr<const DescribedTypeSupport> type_support_;
  Callback callback_;
};

/// The ros2msg definition, as rosbag2 stores it, of a type from a type description service.
/// It uses the .msg sources when every type has one, and otherwise writes out the fields.
/// \throws std::runtime_error for a field type that .msg cannot express.
ROSBAG2_TRANSPORT_PUBLIC
std::string ros2msg_definition(
  const type_description_interfaces::msg::TypeDescription & description,
  const std::vector<type_description_interfaces::msg::TypeSource> & sources);

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__DESCRIBED_TYPE_SUPPORT_HPP_
