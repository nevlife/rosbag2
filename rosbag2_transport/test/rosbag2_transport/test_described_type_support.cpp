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

#include <gmock/gmock.h>

#include <chrono>
#include <cstring>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rmw/rmw.h"
#include "rclcpp/typesupport_helpers.hpp"
#include "rosbag2_cpp/message_definitions/local_message_definition_source.hpp"
#include "rosidl_runtime_c/type_hash.h"
#include "test_msgs/message_fixtures.hpp"
#include "test_msgs/msg/strings.hpp"

#include "described_type_support.hpp"

using namespace ::testing;  // NOLINT

namespace
{
std::string stored_definition(const std::string & type)
{
  rosbag2_cpp::LocalMessageDefinitionSource source;
  auto definition = source.get_full_text(type);
  EXPECT_EQ(definition.encoding, "ros2msg");
  return definition.encoded_message_definition;
}

std::string hash_string(const rosidl_type_hash_t & hash)
{
  char * text = nullptr;
  EXPECT_EQ(
    rosidl_stringify_type_hash(&hash, rcutils_get_default_allocator(), &text), RCUTILS_RET_OK);
  std::string result(text);
  rcutils_get_default_allocator().deallocate(text, nullptr);
  return result;
}
}  // namespace

class DescribedTypeSupportTest : public TestWithParam<std::string> {};

TEST_P(DescribedTypeSupportTest, hash_matches_generated_type_support)
{
  const auto type = GetParam();
  auto library = rclcpp::get_typesupport_library(type, "rosidl_typesupport_cpp");
  auto generated = rclcpp::get_message_typesupport_handle(type, "rosidl_typesupport_cpp", *library);
  rosbag2_transport::DescribedTypeSupport described(type, stored_definition(type));
  EXPECT_EQ(
    hash_string(described.computed_type_hash()),
    hash_string(*generated->get_type_hash_func(generated)));
  const auto & handle = described.type_support();
  EXPECT_EQ(
    hash_string(*handle.get_type_hash_func(&handle)),
    hash_string(*generated->get_type_hash_func(generated)));
}

INSTANTIATE_TEST_SUITE_P(
  TestMsgs, DescribedTypeSupportTest,
  Values(
    "test_msgs/msg/Arrays", "test_msgs/msg/BasicTypes", "test_msgs/msg/BoundedPlainSequences",
    "test_msgs/msg/BoundedSequences", "test_msgs/msg/Builtins", "test_msgs/msg/Constants",
    "test_msgs/msg/Defaults", "test_msgs/msg/Empty", "test_msgs/msg/MultiNested",
    "test_msgs/msg/Nested", "test_msgs/msg/Strings", "test_msgs/msg/UnboundedSequences",
    "rosbag2_interfaces/msg/WriteSplitEvent"));

TEST(DescribedTypeSupport, recorded_hash_takes_precedence)
{
  const std::string recorded =
    "RIHS01_0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
  rosbag2_transport::DescribedTypeSupport described(
    "test_msgs/msg/Nested", stored_definition("test_msgs/msg/Nested"), recorded);
  const auto & handle = described.type_support();
  EXPECT_EQ(hash_string(*handle.get_type_hash_func(&handle)), recorded);
  EXPECT_NE(hash_string(described.computed_type_hash()), recorded);
}

TEST(DescribedTypeSupport, rejects_missing_dependencies_and_bad_lines)
{
  EXPECT_THROW(
    rosbag2_transport::DescribedTypeSupport("pkg/msg/A", "pkg/B b\n"), std::runtime_error);
  EXPECT_THROW(
    rosbag2_transport::DescribedTypeSupport("pkg/msg/A", "int32\n"), std::runtime_error);
  EXPECT_THROW(
    rosbag2_transport::DescribedTypeSupport("pkg/msg/A", "int32<=3 a\n"), std::runtime_error);
  EXPECT_THROW(
    rosbag2_transport::DescribedTypeSupport("not a type", "int32 a\n"), std::runtime_error);
}

TEST(DescribedTypeSupport, publishes_to_a_subscription_of_the_generated_type)
{
  if (std::string(rmw_get_implementation_identifier()) != "rmw_zenoh_cpp") {
    GTEST_SKIP() << "Only rmw_zenoh_cpp takes a type support without generated code";
  }
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<rclcpp::Node>("described_type_support_test");
    auto type_support = std::make_shared<const rosbag2_transport::DescribedTypeSupport>(
      "test_msgs/msg/Strings", stored_definition("test_msgs/msg/Strings"));
    auto publisher = std::make_shared<rosbag2_transport::DescribedTypePublisher>(
      node->get_node_base_interface().get(), type_support, "/described", rclcpp::QoS(10));
    std::shared_ptr<const test_msgs::msg::Strings> received;
    auto subscription = node->create_subscription<test_msgs::msg::Strings>(
      "/described", 10, [&received](std::shared_ptr<const test_msgs::msg::Strings> message) {
        received = message;
      });

    auto sent = get_messages_strings()[1];
    rclcpp::Serialization<test_msgs::msg::Strings> serialization;
    rclcpp::SerializedMessage serialized;
    serialization.serialize_message(sent.get(), &serialized);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!received && std::chrono::steady_clock::now() < deadline) {
      if (publisher->get_subscription_count() > 0) {
        publisher->publish(serialized);
      }
      executor.spin_some(std::chrono::milliseconds(100));
    }
    ASSERT_NE(received, nullptr);
    EXPECT_EQ(*received, *sent);
  }
  rclcpp::shutdown();
}
