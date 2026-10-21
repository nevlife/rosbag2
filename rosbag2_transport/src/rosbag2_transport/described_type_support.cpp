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

#include "described_type_support.hpp"

#include <map>
#include <memory>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rcl/intra_process.h"
#include "rcl/publisher.h"
#include "rcl/type_description_conversions.h"
#include "rcl/type_hash.h"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rcutils/error_handling.h"
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/type_description/field__functions.h"
#include "rosidl_runtime_c/type_description/field_type__struct.h"
#include "rosidl_runtime_c/type_description/individual_type_description__functions.h"
#include "rosidl_runtime_c/type_description/type_description__functions.h"
#include "rosidl_runtime_c/type_description/type_source__functions.h"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "type_description_interfaces/msg/field_type.hpp"
#include "type_description_interfaces/msg/type_description.h"

namespace rosbag2_transport
{
namespace
{

struct Field
{
  std::string name;
  uint8_t type_id;
  uint64_t capacity;
  uint64_t string_capacity;
  std::string nested_type;
};

std::string canonical_type(const std::string & name, const std::string & package)
{
  const auto first = name.find('/');
  if (first == std::string::npos) {
    return package + "/msg/" + name;
  }
  const auto last = name.rfind('/');
  return name.substr(0, first) + "/msg/" + name.substr(last + 1);
}

std::string package_of(const std::string & type)
{
  return type.substr(0, type.find('/'));
}

std::map<std::string, std::string> split_definitions(
  const std::string & type, const std::string & text)
{
  static const std::regex delimiter("^={10,}\\s*$");
  static const std::regex header("^MSG:\\s*(\\S+)\\s*$");
  std::map<std::string, std::string> definitions;
  std::string current = type;
  std::string body;
  std::istringstream lines(text);
  std::string line;
  std::smatch match;
  while (std::getline(lines, line)) {
    if (std::regex_match(line, delimiter)) {
      std::string next;
      if (std::getline(lines, next) && std::regex_match(next, match, header)) {
        definitions.emplace(current, std::move(body));
        current = canonical_type(match[1], "");
        body.clear();
        continue;
      }
      line += "\n" + next;
    }
    body += line + "\n";
  }
  definitions.emplace(current, std::move(body));
  return definitions;
}

// Field type ids of type_description_interfaces/msg/FieldType for .msg primitives, after the
// .msg to IDL mapping (char is uint8, byte is octet).
const std::map<std::string, uint8_t> & primitive_ids()
{
  static const std::map<std::string, uint8_t> ids = {
    {"bool", rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN},
    {"byte", rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BYTE},
    {"char", rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8},
    {"int8", rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8},
    {"uint8", rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8},
    {"int16", rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16},
    {"uint16", rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16},
    {"int32", rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32},
    {"uint32", rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32},
    {"int64", rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64},
    {"uint64", rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64},
    {"float32", rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT},
    {"float64", rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE},
    {"string", rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING},
    {"wstring", rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_WSTRING},
  };
  return ids;
}

std::vector<Field> parse_fields(const std::string & type, const std::string & text)
{
  static const std::regex field_line("^(\\S+)\\s+([A-Za-z_][A-Za-z0-9_]*)(.*)$");
  static const std::regex type_token(
    "^([A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*){0,2})(?:<=([0-9]+))?"
    "(?:\\[([0-9]*|<=[0-9]+)\\])?$");
  constexpr uint8_t array_offset = 48;
  constexpr uint8_t bounded_sequence_offset = 96;
  constexpr uint8_t unbounded_sequence_offset = 144;
  std::vector<Field> fields;
  std::istringstream lines(text);
  std::string line;
  std::smatch match;
  while (std::getline(lines, line)) {
    line = line.substr(0, line.find('#'));
    const auto begin = line.find_first_not_of(" \t\r");
    if (begin == std::string::npos) {
      continue;
    }
    line = line.substr(begin, line.find_last_not_of(" \t\r") + 1 - begin);
    if (!std::regex_match(line, match, field_line)) {
      throw std::runtime_error("invalid line in the definition of " + type + ": " + line);
    }
    const std::string token = match[1];
    Field field{match[2], 0, 0, 0, ""};
    const std::string rest = match[3];
    const auto rest_begin = rest.find_first_not_of(" \t");
    if (rest_begin != std::string::npos && rest[rest_begin] == '=') {
      continue;  // A constant, which is not part of the type description.
    }
    if (!std::regex_match(token, match, type_token)) {
      throw std::runtime_error("unsupported field type in the definition of " + type + ": " +
            token);
    }
    const std::string base = match[1];
    const auto primitive = primitive_ids().find(base);
    if (primitive != primitive_ids().end()) {
      field.type_id = primitive->second;
    } else {
      field.type_id = rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE;
      field.nested_type = canonical_type(base, package_of(type));
    }
    if (match[2].matched) {
      if (base == "string") {
        field.type_id = rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOUNDED_STRING;
      } else if (base == "wstring") {
        field.type_id = rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOUNDED_WSTRING;
      } else {
        throw std::runtime_error("only strings can be bounded: " + token);
      }
      field.string_capacity = std::stoull(match[2]);
    }
    if (match[3].matched) {
      const std::string array = match[3];
      if (array.empty()) {
        field.type_id += unbounded_sequence_offset;
      } else if (array.rfind("<=", 0) == 0) {
        field.type_id += bounded_sequence_offset;
        field.capacity = std::stoull(array.substr(2));
      } else {
        field.type_id += array_offset;
        field.capacity = std::stoull(array);
      }
    }
    fields.push_back(std::move(field));
  }
  if (fields.empty()) {
    // The .msg to IDL conversion gives empty structures one member.
    fields.push_back(
      {"structure_needs_at_least_one_member",
        rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8, 0, 0, ""});
  }
  return fields;
}

void assign(rosidl_runtime_c__String & target, const std::string & value)
{
  if (!rosidl_runtime_c__String__assignn(&target, value.data(), value.size())) {
    throw std::bad_alloc();
  }
}

void fill(
  rosidl_runtime_c__type_description__IndividualTypeDescription & description,
  const std::string & type, const std::vector<Field> & fields)
{
  assign(description.type_name, type);
  if (!rosidl_runtime_c__type_description__Field__Sequence__init(
      &description.fields, fields.size()))
  {
    throw std::bad_alloc();
  }
  for (size_t i = 0; i < fields.size(); ++i) {
    auto & target = description.fields.data[i];
    assign(target.name, fields[i].name);
    target.type.type_id = fields[i].type_id;
    target.type.capacity = fields[i].capacity;
    target.type.string_capacity = fields[i].string_capacity;
    assign(target.type.nested_type_name, fields[i].nested_type);
  }
}

std::string msg_type_name(const std::string & type)
{
  return type.substr(0, type.find('/')) + "/" + type.substr(type.rfind('/') + 1);
}

std::string msg_fields(const type_description_interfaces::msg::IndividualTypeDescription & type)
{
  using FieldType = type_description_interfaces::msg::FieldType;
  static const std::map<uint8_t, std::string> names = {
    {FieldType::FIELD_TYPE_INT8, "int8"}, {FieldType::FIELD_TYPE_UINT8, "uint8"},
    {FieldType::FIELD_TYPE_INT16, "int16"}, {FieldType::FIELD_TYPE_UINT16, "uint16"},
    {FieldType::FIELD_TYPE_INT32, "int32"}, {FieldType::FIELD_TYPE_UINT32, "uint32"},
    {FieldType::FIELD_TYPE_INT64, "int64"}, {FieldType::FIELD_TYPE_UINT64, "uint64"},
    {FieldType::FIELD_TYPE_FLOAT, "float32"}, {FieldType::FIELD_TYPE_DOUBLE, "float64"},
    {FieldType::FIELD_TYPE_BOOLEAN, "bool"}, {FieldType::FIELD_TYPE_BYTE, "byte"},
    {FieldType::FIELD_TYPE_STRING, "string"}, {FieldType::FIELD_TYPE_WSTRING, "wstring"},
    {FieldType::FIELD_TYPE_BOUNDED_STRING, "string"},
    {FieldType::FIELD_TYPE_BOUNDED_WSTRING, "wstring"},
  };
  constexpr uint8_t kinds = 48;
  std::string text;
  for (const auto & field : type.fields) {
    const uint8_t base = field.type.type_id % kinds;
    std::string type_name;
    if (base == FieldType::FIELD_TYPE_NESTED_TYPE) {
      type_name = msg_type_name(field.type.nested_type_name);
    } else {
      const auto name = names.find(base);
      if (name == names.end()) {
        throw std::runtime_error(
                "field " + field.name + " of " + type.type_name + " has type id " +
                std::to_string(field.type.type_id) + ", which .msg cannot express");
      }
      type_name = name->second;
    }
    if (base == FieldType::FIELD_TYPE_BOUNDED_STRING ||
      base == FieldType::FIELD_TYPE_BOUNDED_WSTRING)
    {
      type_name += "<=" + std::to_string(field.type.string_capacity);
    }
    switch (field.type.type_id / kinds) {
      case 1:
        type_name += "[" + std::to_string(field.type.capacity) + "]";
        break;
      case 2:
        type_name += "[<=" + std::to_string(field.type.capacity) + "]";
        break;
      case 3:
        type_name += "[]";
        break;
    }
    text += type_name + " " + field.name + "\n";
  }
  return text;
}

}  // namespace

std::string ros2msg_definition(
  const type_description_interfaces::msg::TypeDescription & description,
  const std::vector<type_description_interfaces::msg::TypeSource> & sources)
{
  std::map<std::string, std::string> msg_sources;
  for (const auto & source : sources) {
    if (source.encoding == "msg" && !source.raw_file_contents.empty()) {
      msg_sources.emplace(source.type_name, source.raw_file_contents);
    }
  }
  auto text_of = [&msg_sources](const auto & type) {
      const auto source = msg_sources.find(type.type_name);
      if (source == msg_sources.end()) {
        return msg_fields(type);
      }
      auto text = source->second;
      if (text.back() != '\n') {
        text += '\n';
      }
      return text;
    };
  std::string text = text_of(description.type_description);
  for (const auto & referenced : description.referenced_type_descriptions) {
    text += "\n" + std::string(80, '=') + "\nMSG: " + msg_type_name(referenced.type_name) + "\n" +
      text_of(referenced);
  }
  return text;
}

struct DescribedTypeSupport::Impl
{
  // Standard layout, so that the rmw's view of type_support.data as callbacks reaches owner.
  struct Callbacks
  {
    message_type_support_callbacks_t callbacks;
    const Impl * owner;
  };

  std::string message_namespace;
  std::string message_name;
  Callbacks callbacks{};
  rosidl_message_type_support_t type_support{};
  rosidl_type_hash_t type_hash{};
  rosidl_type_hash_t computed_type_hash{};
  rosidl_runtime_c__type_description__TypeDescription description{};
  rosidl_runtime_c__type_description__TypeSource__Sequence sources{};

  static const Impl & of(const rosidl_message_type_support_t * type_support)
  {
    return *reinterpret_cast<const Callbacks *>(type_support->data)->owner;
  }
};

DescribedTypeSupport::DescribedTypeSupport(
  const std::string & type, const std::string & definition, const std::string & type_hash)
: impl_(std::make_unique<Impl>())
{
  static const std::regex type_name("^([A-Za-z_][A-Za-z0-9_]*)/msg/([A-Za-z_][A-Za-z0-9_]*)$");
  std::smatch match;
  if (!std::regex_match(type, match, type_name)) {
    throw std::runtime_error("unsupported message type name: " + type);
  }
  impl_->message_namespace = match[1].str() + "::msg";
  impl_->message_name = match[2];

  if (!rosidl_runtime_c__type_description__TypeDescription__init(&impl_->description) ||
    !rosidl_runtime_c__type_description__TypeSource__Sequence__init(&impl_->sources, 0))
  {
    throw std::bad_alloc();
  }
  const auto definitions = split_definitions(type, definition);
  std::map<std::string, std::vector<Field>> referenced;
  std::vector<std::string> pending{type};
  std::set<std::string> seen{type};
  std::vector<Field> fields;
  while (!pending.empty()) {
    const auto current = pending.back();
    pending.pop_back();
    const auto text = definitions.find(current);
    if (text == definitions.end()) {
      throw std::runtime_error("the definition of " + type + " lacks its dependency " + current);
    }
    auto current_fields = parse_fields(current, text->second);
    for (const auto & field : current_fields) {
      if (!field.nested_type.empty() && seen.insert(field.nested_type).second) {
        pending.push_back(field.nested_type);
      }
    }
    if (current == type) {
      fields = std::move(current_fields);
    } else {
      referenced.emplace(current, std::move(current_fields));
    }
  }
  fill(impl_->description.type_description, type, fields);
  if (!rosidl_runtime_c__type_description__IndividualTypeDescription__Sequence__init(
      &impl_->description.referenced_type_descriptions, referenced.size()))
  {
    throw std::bad_alloc();
  }
  size_t index = 0;
  for (const auto & [name, nested_fields] : referenced) {
    fill(impl_->description.referenced_type_descriptions.data[index++], name, nested_fields);
  }

  auto * message = rcl_convert_type_description_runtime_to_msg(&impl_->description);
  if (message == nullptr) {
    rcutils_reset_error();
    throw std::runtime_error("failed to convert the type description of " + type);
  }
  const auto hash_ret = rcl_calculate_type_hash(message, &impl_->computed_type_hash);
  type_description_interfaces__msg__TypeDescription__destroy(message);
  if (hash_ret != RCL_RET_OK) {
    rcutils_reset_error();
    throw std::runtime_error("failed to hash the type description of " + type);
  }
  impl_->type_hash = impl_->computed_type_hash;
  if (!type_hash.empty() &&
    rosidl_parse_type_hash_string(type_hash.c_str(), &impl_->type_hash) != RCUTILS_RET_OK)
  {
    rcutils_reset_error();
    throw std::runtime_error("invalid type hash for " + type + ": " + type_hash);
  }

  auto & callbacks = impl_->callbacks.callbacks;
  callbacks.message_namespace_ = impl_->message_namespace.c_str();
  callbacks.message_name_ = impl_->message_name.c_str();
  callbacks.cdr_serialize = [](const void *, eprosima::fastcdr::Cdr &) {return false;};
  callbacks.cdr_deserialize = [](eprosima::fastcdr::Cdr &, void *) {return false;};
  callbacks.get_serialized_size = [](const void *) -> uint32_t {return 0;};
  callbacks.max_serialized_size = [](char & bounds_info) -> size_t {
      bounds_info = ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
      return 0;
    };
  callbacks.key_callbacks = nullptr;
  impl_->callbacks.owner = impl_.get();

  auto & handle = impl_->type_support;
  handle.typesupport_identifier = rosidl_typesupport_fastrtps_cpp::typesupport_identifier;
  handle.data = &impl_->callbacks;
  handle.func = get_message_typesupport_handle_function;
  handle.get_type_hash_func = [](const rosidl_message_type_support_t * type_support) {
      return &Impl::of(type_support).type_hash;
    };
  handle.get_type_description_func = [](const rosidl_message_type_support_t * type_support) {
      return &Impl::of(type_support).description;
    };
  handle.get_type_description_sources_func =
    [](const rosidl_message_type_support_t * type_support) {
      return &Impl::of(type_support).sources;
    };
}

DescribedTypeSupport::~DescribedTypeSupport()
{
  rosidl_runtime_c__type_description__TypeDescription__fini(&impl_->description);
  rosidl_runtime_c__type_description__TypeSource__Sequence__fini(&impl_->sources);
}

const rosidl_message_type_support_t & DescribedTypeSupport::type_support() const
{
  return impl_->type_support;
}

const rosidl_type_hash_t & DescribedTypeSupport::computed_type_hash() const
{
  return impl_->computed_type_hash;
}

DescribedTypePublisher::DescribedTypePublisher(
  rclcpp::node_interfaces::NodeBaseInterface * node_base,
  std::shared_ptr<const DescribedTypeSupport> type_support,
  const std::string & topic_name,
  const rclcpp::QoS & qos)
: rclcpp::PublisherBase(
    node_base, topic_name, type_support->type_support(),
    rclcpp::PublisherOptions().to_rcl_publisher_options<rclcpp::SerializedMessage>(qos),
    rclcpp::PublisherEventCallbacks(), true),
  type_support_(std::move(type_support))
{}

void DescribedTypePublisher::publish(const rclcpp::SerializedMessage & message)
{
  const auto ret = rcl_publish_serialized_message(
    get_publisher_handle().get(), &message.get_rcl_serialized_message(), nullptr);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to publish serialized message");
  }
}

DescribedTypeSubscription::DescribedTypeSubscription(
  rclcpp::node_interfaces::NodeBaseInterface * node_base,
  std::shared_ptr<const DescribedTypeSupport> type_support,
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  Callback callback,
  const rclcpp::SubscriptionOptions & options)
: rclcpp::SubscriptionBase(
    node_base, type_support->type_support(), topic_name,
    options.to_rcl_subscription_options(qos), options.event_callbacks,
    options.use_default_callbacks, rclcpp::DeliveredMessageKind::SERIALIZED_MESSAGE),
  type_support_(std::move(type_support)),
  callback_(std::move(callback))
{
  if (options.use_intra_process_comm == rclcpp::IntraProcessSetting::Disable) {
    rcl_subscription_disable_ipc(get_subscription_handle().get());
  } else {
    rcl_subscription_set_serialized_ipc(get_subscription_handle().get());
  }
}

std::shared_ptr<void> DescribedTypeSubscription::create_message()
{
  return create_serialized_message();
}

std::shared_ptr<rclcpp::SerializedMessage> DescribedTypeSubscription::create_serialized_message()
{
  return std::make_shared<rclcpp::SerializedMessage>(0);
}

void DescribedTypeSubscription::handle_message(std::shared_ptr<void> &, const rclcpp::MessageInfo &)
{
  throw rclcpp::exceptions::UnimplementedError("only serialized messages are taken");
}

void DescribedTypeSubscription::handle_serialized_message(
  const std::shared_ptr<rclcpp::SerializedMessage> & message,
  const rclcpp::MessageInfo & message_info)
{
  callback_(message, message_info);
}

void DescribedTypeSubscription::handle_loaned_message(void *, const rclcpp::MessageInfo &)
{
  throw rclcpp::exceptions::UnimplementedError("only serialized messages are taken");
}

void DescribedTypeSubscription::return_message(std::shared_ptr<void> & message)
{
  message.reset();
}

void DescribedTypeSubscription::return_serialized_message(
  std::shared_ptr<rclcpp::SerializedMessage> & message)
{
  message.reset();
}

rclcpp::dynamic_typesupport::DynamicMessageType::SharedPtr
DescribedTypeSubscription::get_shared_dynamic_message_type()
{
  throw rclcpp::exceptions::UnimplementedError("only serialized messages are taken");
}

rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr
DescribedTypeSubscription::get_shared_dynamic_message()
{
  throw rclcpp::exceptions::UnimplementedError("only serialized messages are taken");
}

rclcpp::dynamic_typesupport::DynamicSerializationSupport::SharedPtr
DescribedTypeSubscription::get_shared_dynamic_serialization_support()
{
  throw rclcpp::exceptions::UnimplementedError("only serialized messages are taken");
}

rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr
DescribedTypeSubscription::create_dynamic_message()
{
  throw rclcpp::exceptions::UnimplementedError("only serialized messages are taken");
}

void DescribedTypeSubscription::return_dynamic_message(
  rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr &)
{
  throw rclcpp::exceptions::UnimplementedError("only serialized messages are taken");
}

void DescribedTypeSubscription::handle_dynamic_message(
  const rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr &, const rclcpp::MessageInfo &)
{
  throw rclcpp::exceptions::UnimplementedError("only serialized messages are taken");
}

}  // namespace rosbag2_transport
