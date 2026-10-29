// Copyright 2026 nev
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

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include "mock_player.hpp"
#include "mock_sequential_reader.hpp"
#include "rosbag2_play_test_fixture.hpp"
#include "std_msgs/msg/header.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_transport;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

namespace
{
/// Subscribes like a localization node: short queues, and a long callback for each trigger.
class NodeUnderTest
{
public:
  NodeUnderTest(std::chrono::milliseconds work, bool answer)
  : node_(std::make_shared<rclcpp::Node>("node_under_test"))
  {
    done_pub_ = node_->create_publisher<std_msgs::msg::Header>("/done", 10);
    trigger_sub_ = node_->create_subscription<std_msgs::msg::Header>(
      "/trigger", 10, [this, work, answer](std_msgs::msg::Header::ConstSharedPtr msg) {
        std::this_thread::sleep_for(work);
        ++triggers;
        if (answer) {
          done_pub_->publish(*msg);
        }
      });
    data_sub_ = node_->create_subscription<std_msgs::msg::Header>(
      "/data", 10, [this](std_msgs::msg::Header::ConstSharedPtr) {++data;});
    executor_.add_node(node_);
    spin_thread_ = std::thread([this]() {executor_.spin();});
  }

  ~NodeUnderTest()
  {
    executor_.cancel();
    spin_thread_.join();
  }

  std::atomic_size_t triggers{0};
  std::atomic_size_t data{0};

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr done_pub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr trigger_sub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr data_sub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
};

std::shared_ptr<std_msgs::msg::Header> header(int32_t sec, uint32_t nanosec)
{
  auto msg = std::make_shared<std_msgs::msg::Header>();
  msg->stamp.sec = sec;
  msg->stamp.nanosec = nanosec;
  return msg;
}
}  // namespace

class LockstepPlayTest : public RosBag2PlayTestFixture
{
public:
  static constexpr size_t kTriggers = 5;
  static constexpr size_t kDataPerTrigger = 15;

  /// Triggers 2 s apart, each followed by more data messages than the node's queue holds.
  std::unique_ptr<rosbag2_cpp::Reader> make_reader()
  {
    std::vector<rosbag2_storage::TopicMetadata> topics{
      {1u, "/trigger", "std_msgs/msg/Header", "", {}, ""},
      {2u, "/data", "std_msgs/msg/Header", "", {}, ""}};
    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages;
    for (size_t k = 0; k < kTriggers; ++k) {
      const int64_t ms = 2000 * static_cast<int64_t>(k);
      messages.push_back(
        serialize_test_message("/trigger", ms, header(static_cast<int32_t>(k), 0)));
      for (size_t i = 0; i < kDataPerTrigger; ++i) {
        messages.push_back(
          serialize_test_message(
            "/data", ms + 100 + static_cast<int64_t>(i),
            header(static_cast<int32_t>(k), static_cast<uint32_t>(100000000 + i))));
      }
    }
    auto mock_reader = std::make_unique<MockSequentialReader>();
    mock_reader->prepare(messages, topics);
    return std::make_unique<rosbag2_cpp::Reader>(std::move(mock_reader));
  }
};

TEST_F(LockstepPlayTest, waits_for_done_and_keeps_short_queues_from_overflowing)
{
  NodeUnderTest node(50ms, true);
  play_options_.lockstep_trigger_topic = "/trigger";
  play_options_.lockstep_done_topic = "/done";
  auto player = std::make_shared<MockPlayer>(make_reader(), storage_options_, play_options_);

  const auto start = std::chrono::steady_clock::now();
  ASSERT_TRUE(player->play());
  ASSERT_TRUE(player->wait_for_playback_to_finish(30s));
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_EQ(node.triggers.load(), kTriggers);
  EXPECT_EQ(node.data.load(), kTriggers * kDataPerTrigger);
  // Played at its replay time, the bag would take 8 s.
  EXPECT_LT(elapsed, 4s);
}

TEST_F(LockstepPlayTest, continues_after_the_timeout_without_done)
{
  NodeUnderTest node(10ms, false);
  play_options_.lockstep_trigger_topic = "/trigger";
  play_options_.lockstep_done_topic = "/done";
  play_options_.lockstep_timeout = rclcpp::Duration(0, 100000000);
  auto player = std::make_shared<MockPlayer>(make_reader(), storage_options_, play_options_);

  const auto start = std::chrono::steady_clock::now();
  ASSERT_TRUE(player->play());
  ASSERT_TRUE(player->wait_for_playback_to_finish(30s));
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_EQ(node.triggers.load(), kTriggers);
  EXPECT_EQ(node.data.load(), kTriggers * kDataPerTrigger);
  EXPECT_LT(elapsed, 6s);
}
