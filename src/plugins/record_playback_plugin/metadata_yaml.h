// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <deque>

#include "record_playback_plugin/topic_meta.h"

#include "yaml-cpp/yaml.h"

namespace aimrt::plugins::record_playback_plugin {

struct MetaData {
  uint32_t version = 0;

  struct ExtData {
    std::string key;
    std::string value;
  };

  std::vector<TopicMeta> topics;
  std::vector<ExtData> ext_data;

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

    node["ext_data"] = YAML::Node();
    for (const auto& [key, value] : rhs.ext_data) {
      Node ext_node;
      ext_node["key"] = key;
      ext_node["value"] = value;
      node["ext_data"].push_back(ext_node);
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

    if (node["ext_data"] && node["ext_data"].IsSequence()) {
      for (auto& ext_data_node : node["ext_data"]) {
        auto ext_data = Obj::ExtData{
            .key = ext_data_node["key"].as<std::string>(),
            .value = ext_data_node["value"].as<std::string>()};
        rhs.ext_data.emplace_back(ext_data);
      }
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
