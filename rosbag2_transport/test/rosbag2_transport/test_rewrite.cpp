// Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include <filesystem>
#include <string>
#include <vector>
#include <utility>

#include "rosbag2_test_common/temporary_directory_fixture.hpp"
#include "rosbag2_test_common/tested_storage_ids.hpp"
#include "rosbag2_transport/bag_rewrite.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

namespace fs = std::filesystem;

/*
Builtin knowledge about the bags under test:
rewriter_a:
  - a_empty:
    - test_msgs/Empty
    - 100 messages
    - 2 offered QoS profiles
  - b_basictypes:
    - test_msgs/BasicTypes
    - 50 messages
    - 1 offered QoS profile
rewriter_b:
  - a_empty:
    - test_msgs/Empty
    - 25 messages
    - 1 offered Qos Profile
  - c_strings:
    - test_msgs/Strings
    - 50 messages
    - 1 offered QoS Profile
*/
class TestRewrite : public ParametrizedTemporaryDirectoryFixture
{
public:
  TestRewrite()
  {
    output_dir_ = fs::path(temporary_dir_path_);
    storage_id_ = GetParam();
    bags_path_ = fs::path(_SRC_RESOURCES_DIR_PATH) / storage_id_;
  }

  void use_input_a()
  {
    rosbag2_storage::StorageOptions storage;
    storage.uri = (bags_path_ / "rewriter_a").string();
    storage.storage_id = storage_id_;
    input_bags_.push_back(storage);
  }

  void use_input_b()
  {
    rosbag2_storage::StorageOptions storage;
    storage.uri = (bags_path_ / "rewriter_b").string();
    storage.storage_id = storage_id_;
    input_bags_.push_back(storage);
  }

  ~TestRewrite() override = default;

  fs::path output_dir_;
  fs::path bags_path_{_SRC_RESOURCES_DIR_PATH};
  std::string storage_id_;
  std::vector<rosbag2_storage::StorageOptions> input_bags_;
  std::vector<std::pair<rosbag2_storage::StorageOptions, rosbag2_transport::RecordOptions>>
  output_bags_;
};

TEST_P(TestRewrite, test_noop_rewrite) {
  use_input_a();

  rosbag2_storage::StorageOptions output_storage;
  output_storage.uri = (output_dir_ / "unchanged").string();
  output_storage.storage_id = storage_id_;
  rosbag2_transport::RecordOptions output_record;
  output_record.all_topics = true;
  output_bags_.push_back({output_storage, output_record});

  rosbag2_transport::bag_rewrite(input_bags_, output_bags_);

  auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(output_storage);
  reader->open(output_storage);
  const auto metadata = reader->get_metadata();
  EXPECT_EQ(metadata.message_count, 100u + 50u);
  EXPECT_THAT(metadata.topics_with_message_count, SizeIs(2));
  EXPECT_EQ(metadata.topics_with_message_count[0].topic_metadata.serialization_format, "cdr");
}

TEST_P(TestRewrite, test_merge) {
  use_input_a();
  use_input_b();

  rosbag2_storage::StorageOptions output_storage;
  output_storage.uri = (output_dir_ / "merged").string();
  output_storage.storage_id = storage_id_;
  rosbag2_transport::RecordOptions output_record;
  output_record.all_topics = true;
  output_bags_.push_back({output_storage, output_record});

  rosbag2_transport::bag_rewrite(input_bags_, output_bags_);

  auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(output_storage);
  reader->open(output_storage);
  const auto metadata = reader->get_metadata();
  EXPECT_EQ(metadata.message_count, 100u + 50u + 50u + 25u);

  // Check that all 3 topics are present (a_empty merged from both)
  EXPECT_THAT(metadata.topics_with_message_count, SizeIs(3));

  // Check that offered QoS profiles got concatenated
  for (const auto & topic_info : metadata.topics_with_message_count) {
    const auto topic = topic_info.topic_metadata;
    if (topic.name == "a_empty") {
      EXPECT_EQ(topic.offered_qos_profiles.size(), 3u);
    }
  }
}

TEST_P(TestRewrite, test_message_definitions_stored_with_merge) {
  use_input_a();
  use_input_b();

  rosbag2_storage::StorageOptions output_storage;
  output_storage.uri = (output_dir_ / "merged").string();
  output_storage.storage_id = storage_id_;
  rosbag2_transport::RecordOptions output_record;
  output_record.all_topics = true;
  output_bags_.push_back({output_storage, output_record});

  rosbag2_transport::bag_rewrite(input_bags_, output_bags_);

  auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(output_storage);
  reader->open(output_storage);

  // Fill message_definitions_map
  std::vector<rosbag2_storage::MessageDefinition> msg_definitions;
  reader->get_all_message_definitions(msg_definitions);
  // Check that all 3 message definitions are present (a_empty merged from both)
  EXPECT_THAT(msg_definitions, SizeIs(3));
  bool found_strings_definition = false;
  bool found_empty_definition = false;
  bool found_basic_types_definition = false;
  for (const auto & msg_definition : msg_definitions) {
    if (msg_definition.topic_type == "test_msgs/msg/Empty") {
      EXPECT_TRUE(msg_definition.encoded_message_definition.empty());
      found_empty_definition = true;
    }
    if (msg_definition.topic_type == "test_msgs/msg/BasicTypes") {
      EXPECT_FALSE(msg_definition.encoded_message_definition.empty());
      found_basic_types_definition = true;
    }
    if (msg_definition.topic_type == "test_msgs/msg/Strings") {
      EXPECT_FALSE(msg_definition.encoded_message_definition.empty());
      found_strings_definition = true;
    }
  }
  EXPECT_TRUE(found_strings_definition);
  EXPECT_TRUE(found_empty_definition);
  EXPECT_TRUE(found_basic_types_definition);
}


TEST_P(TestRewrite, test_filter_split) {
  use_input_a();

  {
    rosbag2_storage::StorageOptions storage_opts;
    storage_opts.uri = (output_dir_ / "split1").string();
    storage_opts.storage_id = storage_id_;
    rosbag2_transport::RecordOptions rec_opts;
    rec_opts.all_topics = true;
    rec_opts.exclude_regex = "basic";
    output_bags_.push_back({storage_opts, rec_opts});
  }
  {
    rosbag2_storage::StorageOptions storage_opts;
    storage_opts.uri = (output_dir_ / "split2").string();
    storage_opts.storage_id = storage_id_;
    rosbag2_transport::RecordOptions rec_opts;
    rec_opts.all_topics = false;
    rec_opts.topics = {"b_basictypes"};
    output_bags_.push_back({storage_opts, rec_opts});
  }

  rosbag2_transport::bag_rewrite(input_bags_, output_bags_);

  {
    auto opts = output_bags_[0].first;
    auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(opts);
    reader->open(opts);
    const auto metadata = reader->get_metadata();
    EXPECT_THAT(metadata.topics_with_message_count, SizeIs(1));
    EXPECT_EQ(metadata.topics_with_message_count[0].topic_metadata.name, "a_empty");
    EXPECT_EQ(metadata.message_count, 100u);
  }
  {
    auto opts = output_bags_[1].first;
    auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(opts);
    reader->open(opts);
    const auto metadata = reader->get_metadata();
    EXPECT_THAT(metadata.topics_with_message_count, SizeIs(1));
    EXPECT_EQ(metadata.topics_with_message_count[0].topic_metadata.name, "b_basictypes");
    EXPECT_EQ(metadata.message_count, 50u);
  }
}

TEST_P(TestRewrite, test_compress) {
  use_input_a();

  rosbag2_storage::StorageOptions output_storage;
  auto out_bag = output_dir_ / "compressed";
  output_storage.uri = out_bag.string();
  output_storage.storage_id = storage_id_;
  rosbag2_transport::RecordOptions output_record;
  output_record.all_topics = true;
  output_record.compression_mode = "file";
  output_record.compression_format = "zstd";
  output_bags_.push_back({output_storage, output_record});

  rosbag2_transport::bag_rewrite(input_bags_, output_bags_);

  rosbag2_storage::MetadataIo metadata_io;
  auto metadata = metadata_io.read_metadata(out_bag.string());
  auto first_storage = out_bag / metadata.relative_file_paths[0];

  EXPECT_EQ(first_storage.extension().string(), ".zstd");
  EXPECT_TRUE(fs::exists(first_storage));
  EXPECT_TRUE(fs::is_regular_file(first_storage));
}

TEST_P(TestRewrite, test_message_range_rewrite) {
  use_input_a();

  rosbag2_storage::StorageOptions output_storage;
  output_storage.uri = (output_dir_ / "ranged").string();
  output_storage.storage_id = storage_id_;
  rosbag2_transport::RecordOptions record_options;
  record_options.all_topics = true;
  rosbag2_transport::TopicMessageRanges ranges = {
    {"a_empty", {10, 20}},
    {"b_basictypes", {5, 10}}};
  output_bags_.push_back({output_storage, record_options});

  rosbag2_transport::bag_rewrite(input_bags_, output_bags_, {ranges});

  auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(output_storage);
  reader->open(output_storage);
  const auto metadata = reader->get_metadata();
  EXPECT_EQ(metadata.message_count, 11u + 6u);
  EXPECT_THAT(metadata.topics_with_message_count, SizeIs(2));
  for (const auto & topic_info : metadata.topics_with_message_count) {
    if (topic_info.topic_metadata.name == "a_empty") {
      EXPECT_EQ(topic_info.message_count, 11u);
    } else if (topic_info.topic_metadata.name == "b_basictypes") {
      EXPECT_EQ(topic_info.message_count, 6u);
    } else {
      ADD_FAILURE() << "Unexpected topic in output: " << topic_info.topic_metadata.name;
    }
  }
}

TEST_P(TestRewrite, test_message_range_merged_inputs) {
  // a_empty merged from rewriter_a (100 messages) and rewriter_b (25 messages) leads to
  // 125 messages, take 95 messages which are from both bag files
  use_input_a();
  use_input_b();

  rosbag2_storage::StorageOptions output_storage;
  output_storage.uri = (output_dir_ / "merged_ranged").string();
  output_storage.storage_id = storage_id_;
  rosbag2_transport::RecordOptions record_options;
  record_options.all_topics = true;
  rosbag2_transport::TopicMessageRanges ranges = {{"a_empty", {10, 104}}};
  output_bags_.push_back({output_storage, record_options});

  rosbag2_transport::bag_rewrite(input_bags_, output_bags_, {ranges});

  auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(output_storage);
  reader->open(output_storage);
  const auto metadata = reader->get_metadata();
  EXPECT_THAT(metadata.topics_with_message_count, SizeIs(3));
  for (const auto & topic_info : metadata.topics_with_message_count) {
    const std::string & topic_name = topic_info.topic_metadata.name;
    if (topic_name == "a_empty") {
      EXPECT_EQ(topic_info.message_count, 95u);
    } else if (topic_name != "b_basictypes" && topic_name != "c_strings") {
      ADD_FAILURE() << "Unexpected topic in output: " << topic_name;
    }
  }
}

TEST_P(TestRewrite, test_message_range_out_of_bounds_throws) {
  use_input_a();

  std::vector<std::pair<rosbag2_storage::StorageOptions, rosbag2_transport::RecordOptions>>
  output_bags;
  rosbag2_storage::StorageOptions output_storage;
  output_storage.uri = (output_dir_ / "end_out_of_bounds").string();
  output_storage.storage_id = storage_id_;
  rosbag2_transport::RecordOptions record_options;
  record_options.all_topics = true;
  // a_empty has 100 messages, valid indices are [0, 99]
  rosbag2_transport::TopicMessageRanges ranges = {{"a_empty", {0, 100}}};
  output_bags.push_back({output_storage, record_options});

  EXPECT_THROW(
    rosbag2_transport::bag_rewrite(input_bags_, output_bags, {ranges}),
    std::invalid_argument);
  EXPECT_FALSE(fs::exists(output_storage.uri));
}

TEST_P(TestRewrite, test_message_range_start_greater_than_end_throws) {
  use_input_a();

  rosbag2_storage::StorageOptions output_storage;
  output_storage.uri = (output_dir_ / "start_gt_end").string();
  output_storage.storage_id = storage_id_;
  rosbag2_transport::RecordOptions record_options;
  record_options.all_topics = true;
  rosbag2_transport::TopicMessageRanges ranges = {{"a_empty", {50, 10}}};
  output_bags_.push_back({output_storage, record_options});

  EXPECT_THROW(
    rosbag2_transport::bag_rewrite(input_bags_, output_bags_, {ranges}),
    std::invalid_argument);
  EXPECT_FALSE(fs::exists(output_storage.uri));
}

TEST_P(TestRewrite, test_message_range_unknown_topic_throws) {
  use_input_a();

  rosbag2_storage::StorageOptions output_storage;
  output_storage.uri = (output_dir_ / "unknown_topic").string();
  output_storage.storage_id = storage_id_;
  rosbag2_transport::RecordOptions record_options;
  record_options.all_topics = true;
  rosbag2_transport::TopicMessageRanges ranges = {{"does_not_exist", {0, 5}}};
  output_bags_.push_back({output_storage, record_options});

  EXPECT_THROW(
    rosbag2_transport::bag_rewrite(input_bags_, output_bags_, {ranges}),
    std::invalid_argument);
  EXPECT_FALSE(fs::exists(output_storage.uri));
}

TEST_P(TestRewrite, message_ranges_are_independent_per_output) {
  use_input_a();
  rosbag2_transport::RecordOptions options;
  options.topics = {"a_empty"};
  for (const auto & name : {"first", "last"}) {
    rosbag2_storage::StorageOptions storage;
    storage.uri = (output_dir_ / name).string();
    storage.storage_id = storage_id_;
    output_bags_.push_back({storage, options});
  }
  rosbag2_transport::bag_rewrite(input_bags_, output_bags_, {
    {{"a_empty", {0, 0}}}, {{"a_empty", {99, 99}}}});
  for (const auto & output : output_bags_) {
    auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(output.first);
    reader->open(output.first);
    EXPECT_EQ(reader->get_metadata().message_count, 1u);
  }
}

INSTANTIATE_TEST_SUITE_P(
  ParametrizedRewriteTests,
  TestRewrite,
  ValuesIn(rosbag2_test_common::kTestedStorageIDs)
);
