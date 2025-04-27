// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/channel/channel_manager.h"
#include "core/channel/channel_backend_tools.h"
#include "core/channel/local_channel_backend.h"
#include "core/util/topic_meta_key.h"

namespace YAML {
template <>
struct convert<aimrt::runtime::core::channel::ChannelManager::Options> {
  using Options = aimrt::runtime::core::channel::ChannelManager::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["backends"] = YAML::Node();
    for (const auto& backend : rhs.backends_options) {
      Node backend_options_node;
      backend_options_node["type"] = backend.type;
      backend_options_node["options"] = backend.options;
      node["backends"].push_back(backend_options_node);
    }

    node["pub_topics_options"] = YAML::Node();
    for (const auto& pub_topic_options : rhs.pub_topics_options) {
      Node pub_topic_options_node;
      pub_topic_options_node["topic_name"] = pub_topic_options.topic_name;
      pub_topic_options_node["enable_backends"] = pub_topic_options.enable_backends;
      pub_topic_options_node["enable_filters"] = pub_topic_options.enable_filters;
      node["pub_topics_options"].push_back(pub_topic_options_node);
    }

    node["sub_topics_options"] = YAML::Node();
    for (const auto& sub_topic_options : rhs.sub_topics_options) {
      Node sub_topic_options_node;
      sub_topic_options_node["topic_name"] = sub_topic_options.topic_name;
      sub_topic_options_node["enable_backends"] = sub_topic_options.enable_backends;
      sub_topic_options_node["enable_filters"] = sub_topic_options.enable_filters;
      node["sub_topics_options"].push_back(sub_topic_options_node);
    }

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (!node.IsMap()) return false;

    if (node["backends"] && node["backends"].IsSequence()) {
      for (const auto& backend_options_node : node["backends"]) {
        auto backend_options = Options::BackendOptions{
            .type = backend_options_node["type"].as<std::string>()};

        if (backend_options_node["options"])
          backend_options.options = backend_options_node["options"];
        else
          backend_options.options = YAML::Node(YAML::NodeType::Null);

        rhs.backends_options.emplace_back(std::move(backend_options));
      }
    }

    if (node["pub_topics_options"] && node["pub_topics_options"].IsSequence()) {
      for (const auto& pub_topic_options_node : node["pub_topics_options"]) {
        auto pub_topic_options = Options::PubTopicOptions{
            .topic_name = pub_topic_options_node["topic_name"].as<std::string>(),
            .enable_backends = pub_topic_options_node["enable_backends"].as<std::vector<std::string>>()};

        if (pub_topic_options_node["enable_filters"])
          pub_topic_options.enable_filters = pub_topic_options_node["enable_filters"].as<std::vector<std::string>>();

        rhs.pub_topics_options.emplace_back(std::move(pub_topic_options));
      }
    }

    if (node["sub_topics_options"] && node["sub_topics_options"].IsSequence()) {
      for (const auto& sub_topic_options_node : node["sub_topics_options"]) {
        auto sub_topic_options = Options::SubTopicOptions{
            .topic_name = sub_topic_options_node["topic_name"].as<std::string>(),
            .enable_backends = sub_topic_options_node["enable_backends"].as<std::vector<std::string>>()};

        if (sub_topic_options_node["enable_filters"])
          sub_topic_options.enable_filters = sub_topic_options_node["enable_filters"].as<std::vector<std::string>>();

        rhs.sub_topics_options.emplace_back(std::move(sub_topic_options));
      }
    }

    return true;
  }
};
}  // namespace YAML

namespace aimrt::runtime::core::channel {

void ChannelManager::Initialize(YAML::Node options_node) {
  RegisterLocalChannelBackend();
  RegisterDebugLogFilter();

  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kInit) == State::kPreInit,
      "Channel manager can only be initialized once.");

  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  channel_registry_ptr_ = std::make_unique<ChannelRegistry>();
  channel_registry_ptr_->SetLogger(logger_ptr_);

  channel_backend_manager_.SetLogger(logger_ptr_);
  channel_backend_manager_.SetChannelRegistry(channel_registry_ptr_.get());
  channel_backend_manager_.SetPublishFrameworkAsyncChannelFilterManager(&publish_filter_manager_);
  channel_backend_manager_.SetSubscribeFrameworkAsyncChannelFilterManager(&subscribe_filter_manager_);

  std::vector<std::string> channel_backend_name_vec;

  // Initialize the specified backend according to the configuration
  for (auto& backend_options : options_.backends_options) {
    auto finditr = std::find_if(
        channel_backend_vec_.begin(), channel_backend_vec_.end(),
        [&backend_options](const auto& ptr) {
          return ptr->Name() == backend_options.type;
        });

    AIMRT_CHECK_ERROR_THROW(finditr != channel_backend_vec_.end(),
                            "Invalid channel backend type '{}'",
                            backend_options.type);

    (*finditr)->SetChannelRegistry(channel_registry_ptr_.get());
    (*finditr)->Initialize(backend_options.options);

    channel_backend_manager_.RegisterChannelBackend(finditr->get());

    used_channel_backend_vec_.emplace_back(finditr->get());
    channel_backend_name_vec.emplace_back((*finditr)->Name());
  }

  // Set rules
  std::vector<std::pair<std::string, std::vector<std::string>>> pub_backends_rules;
  std::vector<std::pair<std::string, std::vector<std::string>>> pub_filters_rules;
  for (const auto& item : options_.pub_topics_options) {
    for (const auto& backend_name : item.enable_backends) {
      AIMRT_CHECK_ERROR_THROW(
          std::find(channel_backend_name_vec.begin(), channel_backend_name_vec.end(), backend_name) != channel_backend_name_vec.end(),
          "Invalid channel backend type '{}' for pub topic '{}'",
          backend_name, item.topic_name);
    }

    pub_backends_rules.emplace_back(item.topic_name, item.enable_backends);
    pub_filters_rules.emplace_back(item.topic_name, item.enable_filters);
  }
  channel_backend_manager_.SetPubTopicsBackendsRules(pub_backends_rules);
  channel_backend_manager_.SetPublishFiltersRules(pub_filters_rules);

  std::vector<std::pair<std::string, std::vector<std::string>>> sub_backends_rules;
  std::vector<std::pair<std::string, std::vector<std::string>>> sub_filters_rules;
  for (const auto& item : options_.sub_topics_options) {
    for (const auto& backend_name : item.enable_backends) {
      AIMRT_CHECK_ERROR_THROW(
          std::find(channel_backend_name_vec.begin(), channel_backend_name_vec.end(), backend_name) != channel_backend_name_vec.end(),
          "Invalid channel backend type '{}' for sub topic '{}'",
          backend_name, item.topic_name);
    }

    sub_backends_rules.emplace_back(item.topic_name, item.enable_backends);
    sub_filters_rules.emplace_back(item.topic_name, item.enable_filters);
  }
  channel_backend_manager_.SetSubTopicsBackendsRules(sub_backends_rules);
  channel_backend_manager_.SetSubscribeFiltersRules(sub_filters_rules);

  // Initialize backend manager
  channel_backend_manager_.Initialize();

  options_node = options_;

  AIMRT_INFO("Channel manager init complete");
}

void ChannelManager::Start() {
  AIMRT_CHECK_ERROR_THROW(
      std::atomic_exchange(&state_, State::kStart) == State::kInit,
      "Method can only be called when state is 'Init'.");

  channel_backend_manager_.Start();
  channel_handle_proxy_start_flag_.store(true);

  AIMRT_INFO("Channel manager start completed.");
}

void ChannelManager::Shutdown() {
  if (std::atomic_exchange(&state_, State::kShutdown) == State::kShutdown)
    return;

  AIMRT_INFO("Channel manager shutdown.");

  channel_handle_proxy_wrap_map_.clear();

  channel_backend_manager_.Shutdown();

  channel_backend_vec_.clear();

  channel_registry_ptr_.reset();

  subscribe_filter_manager_.Clear();
  publish_filter_manager_.Clear();

  get_executor_func_ = nullptr;
}

std::list<std::pair<std::string, std::string>> ChannelManager::GenInitializationReport() const {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kInit,
      "Method can only be called when state is 'Init'.");

  std::vector<std::vector<std::string>> pub_topic_info_table =
      {{"topic", "msg type", "module", "backends", "filters"}};

  const auto& pub_topic_backend_info = channel_backend_manager_.GetPubTopicBackendInfo();
  const auto& pub_topic_index_map = channel_registry_ptr_->GetPubTopicIndexMap();

  // Used to track entries with the same topic and msg_type
  std::unordered_map<util::TopicMetaKey, size_t, util::TopicMetaKey::Hash> pub_topic_msg_indices;

  for (const auto& pub_topic_index_itr : pub_topic_index_map) {
    auto topic_name = pub_topic_index_itr.first;
    auto pub_topic_backend_itr = pub_topic_backend_info.find(topic_name);
    AIMRT_CHECK_ERROR_THROW(pub_topic_backend_itr != pub_topic_backend_info.end(),
                            "Invalid channel registry info.");

    auto filter_name_vec = publish_filter_manager_.GetFilterNameVec(topic_name);

    for (const auto& item : pub_topic_index_itr.second) {
      auto key = util::TopicMetaKey{
          .topic_name = std::string(topic_name),
          .msg_type = std::string(item->info.msg_type),
      };
      auto it = pub_topic_msg_indices.find(key);

      if (it != pub_topic_msg_indices.end()) {
        pub_topic_info_table[it->second][2] += "\n" + item->info.module_name;
      } else {
        std::vector<std::string> cur_topic_info(5);
        cur_topic_info[0] = topic_name;
        cur_topic_info[1] = item->info.msg_type;
        cur_topic_info[2] = item->info.module_name;
        cur_topic_info[3] = aimrt::common::util::JoinVec(pub_topic_backend_itr->second, ",");
        cur_topic_info[4] = aimrt::common::util::JoinVec(filter_name_vec, ",");
        pub_topic_info_table.emplace_back(std::move(cur_topic_info));
        pub_topic_msg_indices.emplace(key, pub_topic_info_table.size() - 1);
      }
    }
  }

  std::vector<std::vector<std::string>> sub_topic_info_table =
      {{"topic", "msg type", "module", "backends", "filters"}};

  const auto& sub_topic_backend_info = channel_backend_manager_.GetSubTopicBackendInfo();
  const auto& sub_topic_index_map = channel_registry_ptr_->GetSubTopicIndexMap();

  std::unordered_map<util::TopicMetaKey, size_t, util::TopicMetaKey::Hash> sub_topic_msg_indices;

  for (const auto& sub_topic_index_itr : sub_topic_index_map) {
    auto topic_name = sub_topic_index_itr.first;
    auto sub_topic_backend_itr = sub_topic_backend_info.find(topic_name);
    AIMRT_CHECK_ERROR_THROW(sub_topic_backend_itr != sub_topic_backend_info.end(),
                            "Invalid channel registry info.");

    auto filter_name_vec = subscribe_filter_manager_.GetFilterNameVec(topic_name);

    for (const auto& item : sub_topic_index_itr.second) {
      auto key = util::TopicMetaKey{
          .topic_name = std::string(topic_name),
          .msg_type = std::string(item->info.msg_type),
      };
      auto it = sub_topic_msg_indices.find(key);

      if (it != sub_topic_msg_indices.end()) {
        sub_topic_info_table[it->second][2] += "\n" + item->info.module_name;
      } else {
        std::vector<std::string> cur_topic_info(5);
        cur_topic_info[0] = topic_name;
        cur_topic_info[1] = item->info.msg_type;
        cur_topic_info[2] = item->info.module_name;
        cur_topic_info[3] = aimrt::common::util::JoinVec(sub_topic_backend_itr->second, ",");
        cur_topic_info[4] = aimrt::common::util::JoinVec(filter_name_vec, ",");
        sub_topic_info_table.emplace_back(std::move(cur_topic_info));
        sub_topic_msg_indices.emplace(key, sub_topic_info_table.size() - 1);
      }
    }
  }

  std::vector<std::string> channel_backend_name_vec;
  channel_backend_name_vec.reserve(channel_backend_vec_.size());
  for (const auto& item : channel_backend_vec_)
    channel_backend_name_vec.emplace_back(item->Name());

  std::string channel_backend_name_vec_str;
  if (channel_backend_name_vec.empty()) {
    channel_backend_name_vec_str = "<empty>";
  } else {
    channel_backend_name_vec_str = "[ " + aimrt::common::util::JoinVec(channel_backend_name_vec, " , ") + " ]";
  }

  auto channel_pub_filter_name_vec = publish_filter_manager_.GetAllFiltersName();
  std::string channel_pub_filter_name_vec_str;
  if (channel_pub_filter_name_vec.empty()) {
    channel_pub_filter_name_vec_str = "<empty>";
  } else {
    channel_pub_filter_name_vec_str = "[ " + aimrt::common::util::JoinVec(channel_pub_filter_name_vec, " , ") + " ]";
  }

  auto channel_sub_filter_name_vec = subscribe_filter_manager_.GetAllFiltersName();
  std::string channel_sub_filter_name_vec_str;
  if (channel_sub_filter_name_vec.empty()) {
    channel_sub_filter_name_vec_str = "<empty>";
  } else {
    channel_sub_filter_name_vec_str = "[ " + aimrt::common::util::JoinVec(channel_sub_filter_name_vec, " , ") + " ]";
  }

  std::list<std::pair<std::string, std::string>> report{
      {"Channel Backend List", channel_backend_name_vec_str},
      {"Channel Pub Filter List", channel_pub_filter_name_vec_str},
      {"Channel Sub Filter List", channel_sub_filter_name_vec_str},
      {"Channel Pub Topic Info", aimrt::common::util::DrawTable(pub_topic_info_table)},
      {"Channel Sub Topic Info", aimrt::common::util::DrawTable(sub_topic_info_table)}};

  for (const auto& backend_ptr : used_channel_backend_vec_) {
    report.splice(report.end(), backend_ptr->GenInitializationReport());
  }

  return report;
}

void ChannelManager::RegisterChannelBackend(
    std::unique_ptr<ChannelBackendBase>&& channel_backend_ptr) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");

  channel_backend_vec_.emplace_back(std::move(channel_backend_ptr));
}

void ChannelManager::RegisterGetExecutorFunc(
    const std::function<executor::ExecutorRef(std::string_view)>& get_executor_func) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");

  get_executor_func_ = get_executor_func;
}

const ChannelHandleProxy& ChannelManager::GetChannelHandleProxy(
    const util::ModuleDetailInfo& module_info) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kInit,
      "Method can only be called when state is 'Init'.");

  auto itr = channel_handle_proxy_wrap_map_.find(module_info.name);
  if (itr != channel_handle_proxy_wrap_map_.end())
    return itr->second->channel_handle_proxy;

  auto emplace_ret = channel_handle_proxy_wrap_map_.emplace(
      module_info.name,
      std::make_unique<ChannelHandleProxyWrap>(
          module_info.pkg_path,
          module_info.name,
          *logger_ptr_,
          channel_backend_manager_,
          passed_context_meta_keys_,
          channel_handle_proxy_start_flag_));

  return emplace_ret.first->second->channel_handle_proxy;
}

void ChannelManager::RegisterPublishFilter(std::string_view name, FrameworkAsyncChannelFilter&& filter) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");
  publish_filter_manager_.RegisterFilter(name, std::move(filter));
}

void ChannelManager::RegisterSubscribeFilter(std::string_view name, FrameworkAsyncChannelFilter&& filter) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");
  subscribe_filter_manager_.RegisterFilter(name, std::move(filter));
}

void ChannelManager::AddPassedContextMetaKeys(const std::unordered_set<std::string>& keys) {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kPreInit,
      "Method can only be called when state is 'PreInit'.");
  passed_context_meta_keys_.insert(keys.begin(), keys.end());
}

const ChannelRegistry* ChannelManager::GetChannelRegistry() const {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kInit,
      "Method can only be called when state is 'Init'.");
  return channel_registry_ptr_.get();
}

const std::vector<ChannelBackendBase*>& ChannelManager::GetUsedChannelBackend() const {
  AIMRT_CHECK_ERROR_THROW(
      state_.load() == State::kInit,
      "Method can only be called when state is 'Init'.");
  return used_channel_backend_vec_;
}

void ChannelManager::RegisterLocalChannelBackend() {
  std::unique_ptr<ChannelBackendBase> local_channel_backend_ptr =
      std::make_unique<LocalChannelBackend>();

  static_cast<LocalChannelBackend*>(local_channel_backend_ptr.get())
      ->RegisterGetExecutorFunc(get_executor_func_);

  static_cast<LocalChannelBackend*>(local_channel_backend_ptr.get())
      ->SetLogger(logger_ptr_);

  RegisterChannelBackend(std::move(local_channel_backend_ptr));
}

void ChannelManager::RegisterDebugLogFilter() {
  RegisterPublishFilter(
      "debug_log",
      [this](MsgWrapper& msg_wrapper, FrameworkAsyncChannelHandle&& h) {
        auto buf_ptr = TrySerializeMsgWithCache(msg_wrapper, "json");

        if (buf_ptr) {
          auto msg_str = buf_ptr->JoinToString();

          AIMRT_INFO("Channel publish a new msg. msg type: {}, topic name: {}, context: {}, msg: {}",
                     msg_wrapper.info.msg_type,
                     msg_wrapper.info.topic_name,
                     msg_wrapper.ctx_ref.ToString(),
                     msg_str);
        } else {
          AIMRT_INFO("Channel publish a new msg. msg type: {}, topic name: {}, context: {}",
                     msg_wrapper.info.msg_type,
                     msg_wrapper.info.topic_name,
                     msg_wrapper.ctx_ref.ToString());
        }

        h(msg_wrapper);
      });

  RegisterSubscribeFilter(
      "debug_log",
      [this](MsgWrapper& msg_wrapper, FrameworkAsyncChannelHandle&& h) {
        auto buf_ptr = TrySerializeMsgWithCache(msg_wrapper, "json");

        if (buf_ptr) {
          auto msg_str = buf_ptr->JoinToString();

          AIMRT_INFO("Channel subscriber handle a new msg. msg type: {}, topic name: {}, context: {}, msg: {}",
                     msg_wrapper.info.msg_type,
                     msg_wrapper.info.topic_name,
                     msg_wrapper.ctx_ref.ToString(),
                     msg_str);
        } else {
          AIMRT_INFO("Channel subscriber handle a new msg. msg type: {}, topic name: {}, context: {}",
                     msg_wrapper.info.msg_type,
                     msg_wrapper.info.topic_name,
                     msg_wrapper.ctx_ref.ToString());
        }

        h(msg_wrapper);
      });
}

}  // namespace aimrt::runtime::core::channel
