// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <deque>

#include "record_playback_plugin/topic_meta.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::plugins::record_playback_plugin {

struct MetaData {
  uint32_t version = 0;

  std::vector<TopicMeta> topics;
  YAML::Node extra_attributes;

  struct FileMeta {
    std::string path;
    uint64_t start_timestamp;
  };
  std::deque<FileMeta> files;
};

}  // namespace aimrt::plugins::record_playback_plugin

namespace YAML {
template <>
struct convert<aimrt::plugins::record_playback_plugin::MetaData> {
  using Obj = aimrt::plugins::record_playback_plugin::MetaData;

  static Node encode(const Obj& rhs) {
    Node node;

    node["version"] = rhs.version;

    if (rhs.extra_attributes && !rhs.extra_attributes.IsNull()) {
      node["extra_attributes"] = rhs.extra_attributes;
    } else {
      node["extra_attributes"] = YAML::Node(YAML::NodeType::Map);
    }

    node["topics"] = YAML::Node();
    for (const auto& topic : rhs.topics) {
      Node topic_node;
      topic_node["id"] = topic.id;
      topic_node["topic_name"] = topic.topic_name;
      topic_node["msg_type"] = topic.msg_type;
      topic_node["serialization_type"] = topic.serialization_type;
      node["topics"].push_back(topic_node);
    }

    node["files"] = YAML::Node();
    for (const auto& file : rhs.files) {
      Node file_node;
      file_node["path"] = file.path;
      file_node["start_timestamp"] = file.start_timestamp;
      node["files"].push_back(file_node);
    }
    return node;
  }

  static bool decode(const Node& node, Obj& rhs) {
    if (!node.IsMap()) return false;

    rhs.version = node["version"].as<uint32_t>();

    if (node["extra_attributes"]) {
      rhs.extra_attributes = node["extra_attributes"];
    } else {
      rhs.extra_attributes = YAML::Node(YAML::NodeType::Null);
    }

    if (node["topics"] && node["topics"].IsSequence()) {
      for (const auto& topic_node : node["topics"]) {
        aimrt::plugins::record_playback_plugin::TopicMeta topic{
            .id = topic_node["id"].as<uint64_t>(),
            .topic_name = topic_node["topic_name"].as<std::string>(),
            .msg_type = topic_node["msg_type"].as<std::string>(),
            .serialization_type = topic_node["serialization_type"].as<std::string>()};

        rhs.topics.emplace_back(std::move(topic));
      }
    }

    if (node["files"] && node["files"].IsSequence()) {
      for (const auto& file_node : node["files"]) {
        auto file = Obj::FileMeta{
            .path = file_node["path"].as<std::string>(),
            .start_timestamp = file_node["start_timestamp"].as<uint64_t>()};

        rhs.files.emplace_back(std::move(file));
      }
    }

    return true;
  }
};
}  // namespace YAML
