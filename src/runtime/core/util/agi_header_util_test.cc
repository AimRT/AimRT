// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "core/util/agi_header_util.h"
#include <gtest/gtest.h>

#ifdef AIMRT_BUILD_WITH_PROTOBUF
  #include <google/protobuf/descriptor.h>
  #include <google/protobuf/descriptor.pb.h>
  #include <google/protobuf/dynamic_message.h>
#endif

namespace aimrt::runtime::core::util {

TEST(AgiHeaderUtilTest, NanosToAgiTime_Zero) {
  AgiTime t = NanosToAgiTime(0);
  EXPECT_EQ(t.seconds, 0);
  EXPECT_EQ(t.fraction, 0);
}

TEST(AgiHeaderUtilTest, NanosToAgiTime_WholeSeconds) {
  AgiTime t = NanosToAgiTime(3'000'000'000ULL);
  EXPECT_EQ(t.seconds, 3);
  EXPECT_EQ(t.fraction, 0u);
}

TEST(AgiHeaderUtilTest, NanosToAgiTime_FractionalPart) {
  // 1.5 seconds = 1'500'000'000 ns
  // nanos = 500'000'000
  // fraction = (500'000'000 << 32) / 1'000'000'000 = 2'147'483'648
  AgiTime t = NanosToAgiTime(1'500'000'000ULL);
  EXPECT_EQ(t.seconds, 1);
  EXPECT_EQ(t.fraction, 2'147'483'648u);
}

TEST(AgiHeaderUtilTest, NanosToAgiTime_RoundTrip) {
  uint64_t original_ns = 1'234'567'890'123ULL;
  AgiTime t = NanosToAgiTime(original_ns);

  // Reconstruct nanos: nanos = (fraction * 10^9) >> 32
  uint64_t reconstructed_nanos =
      (static_cast<uint64_t>(t.fraction) * 1'000'000'000ULL) >> 32;
  uint64_t reconstructed_ns =
      static_cast<uint64_t>(t.seconds) * 1'000'000'000ULL + reconstructed_nanos;

  // Allow 1 nanosecond rounding error due to integer truncation
  EXPECT_NEAR(static_cast<double>(reconstructed_ns),
              static_cast<double>(original_ns), 1.0);
}

TEST(AgiHeaderUtilTest, DetectAgiHeader_NullPtr) {
  AgiHeaderInfo info = DetectAgiHeader("pb:some.Type", nullptr);
  EXPECT_FALSE(info.has_header);
}

TEST(AgiHeaderUtilTest, DetectAgiRequestHeader_NullPtr) {
  AgiHeaderInfo info = DetectAgiRequestHeader("pb:some.Type", nullptr);
  EXPECT_FALSE(info.has_header);
}

TEST(AgiHeaderUtilTest, DetectAgiHeader_UnknownPrefix) {
  AgiHeaderInfo info = DetectAgiHeader("unknown:some.Type", nullptr);
  EXPECT_FALSE(info.has_header);
}

TEST(AgiHeaderUtilTest, FillAgiHeader_NoHeader) {
  AgiHeaderInfo info;
  info.has_header = false;
  int dummy = 0;
  FillAgiHeader(info, &dummy, 1, "test_module", 1000000000ULL);
  // Should not crash
}

TEST(AgiHeaderUtilTest, FillAgiRequestHeader_NoHeader) {
  AgiHeaderInfo info;
  info.has_header = false;
  int dummy = 0;
  FillAgiRequestHeader(info, &dummy, 1, "test_module", 1000000000ULL);
  // Should not crash
}

TEST(AgiHeaderUtilTest, FillAgiHeader_NullMsg) {
  AgiHeaderInfo info;
  info.has_header = true;
  info.type = AgiHeaderType::kProtobuf;
  FillAgiHeader(info, nullptr, 1, "test_module", 1000000000ULL);
  // Should not crash
}

#ifdef AIMRT_BUILD_WITH_PROTOBUF

class AgiHeaderPbTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Build a dynamic descriptor pool with Time, AgiHeader, AgiRequestHeader,
    // TestMsg, and TestRpcReq
    google::protobuf::FileDescriptorProto file_proto;
    file_proto.set_name("test.proto");
    file_proto.set_package("test");
    file_proto.set_syntax("proto3");

    // Time message
    auto* time_msg = file_proto.add_message_type();
    time_msg->set_name("Time");
    auto* seconds_field = time_msg->add_field();
    seconds_field->set_name("seconds");
    seconds_field->set_number(1);
    seconds_field->set_type(google::protobuf::FieldDescriptorProto::TYPE_INT32);
    seconds_field->set_label(google::protobuf::FieldDescriptorProto::LABEL_OPTIONAL);
    auto* fraction_field = time_msg->add_field();
    fraction_field->set_name("fraction");
    fraction_field->set_number(2);
    fraction_field->set_type(google::protobuf::FieldDescriptorProto::TYPE_UINT32);
    fraction_field->set_label(google::protobuf::FieldDescriptorProto::LABEL_OPTIONAL);

    // AgiHeader message
    auto* agi_header_msg = file_proto.add_message_type();
    agi_header_msg->set_name("AgiHeader");
    auto* seq_field = agi_header_msg->add_field();
    seq_field->set_name("seq_num");
    seq_field->set_number(1);
    seq_field->set_type(google::protobuf::FieldDescriptorProto::TYPE_UINT32);
    seq_field->set_label(google::protobuf::FieldDescriptorProto::LABEL_OPTIONAL);
    auto* pub_name_field = agi_header_msg->add_field();
    pub_name_field->set_name("publisher_name");
    pub_name_field->set_number(2);
    pub_name_field->set_type(google::protobuf::FieldDescriptorProto::TYPE_STRING);
    pub_name_field->set_label(google::protobuf::FieldDescriptorProto::LABEL_OPTIONAL);
    auto* pub_time_field = agi_header_msg->add_field();
    pub_time_field->set_name("publish_time");
    pub_time_field->set_number(3);
    pub_time_field->set_type(google::protobuf::FieldDescriptorProto::TYPE_MESSAGE);
    pub_time_field->set_type_name(".test.Time");
    pub_time_field->set_label(google::protobuf::FieldDescriptorProto::LABEL_OPTIONAL);

    // AgiRequestHeader message
    auto* agi_req_header_msg = file_proto.add_message_type();
    agi_req_header_msg->set_name("AgiRequestHeader");
    auto* req_id_field = agi_req_header_msg->add_field();
    req_id_field->set_name("request_id");
    req_id_field->set_number(1);
    req_id_field->set_type(google::protobuf::FieldDescriptorProto::TYPE_UINT32);
    req_id_field->set_label(google::protobuf::FieldDescriptorProto::LABEL_OPTIONAL);
    auto* client_name_field = agi_req_header_msg->add_field();
    client_name_field->set_name("client_name");
    client_name_field->set_number(2);
    client_name_field->set_type(google::protobuf::FieldDescriptorProto::TYPE_STRING);
    client_name_field->set_label(google::protobuf::FieldDescriptorProto::LABEL_OPTIONAL);
    auto* req_time_field = agi_req_header_msg->add_field();
    req_time_field->set_name("request_time");
    req_time_field->set_number(3);
    req_time_field->set_type(google::protobuf::FieldDescriptorProto::TYPE_MESSAGE);
    req_time_field->set_type_name(".test.Time");
    req_time_field->set_label(google::protobuf::FieldDescriptorProto::LABEL_OPTIONAL);

    // TestMsg with AgiHeader field
    auto* test_msg = file_proto.add_message_type();
    test_msg->set_name("TestMsg");
    auto* header_field = test_msg->add_field();
    header_field->set_name("header");
    header_field->set_number(1);
    header_field->set_type(google::protobuf::FieldDescriptorProto::TYPE_MESSAGE);
    header_field->set_type_name(".test.AgiHeader");
    header_field->set_label(google::protobuf::FieldDescriptorProto::LABEL_OPTIONAL);
    auto* data_field = test_msg->add_field();
    data_field->set_name("data");
    data_field->set_number(2);
    data_field->set_type(google::protobuf::FieldDescriptorProto::TYPE_STRING);
    data_field->set_label(google::protobuf::FieldDescriptorProto::LABEL_OPTIONAL);

    // TestRpcReq with AgiRequestHeader field
    auto* test_rpc_req = file_proto.add_message_type();
    test_rpc_req->set_name("TestRpcReq");
    auto* rpc_header_field = test_rpc_req->add_field();
    rpc_header_field->set_name("header");
    rpc_header_field->set_number(1);
    rpc_header_field->set_type(google::protobuf::FieldDescriptorProto::TYPE_MESSAGE);
    rpc_header_field->set_type_name(".test.AgiRequestHeader");
    rpc_header_field->set_label(google::protobuf::FieldDescriptorProto::LABEL_OPTIONAL);

    // TestMsgNoHeader without AgiHeader field
    auto* test_msg_no_header = file_proto.add_message_type();
    test_msg_no_header->set_name("TestMsgNoHeader");
    auto* plain_field = test_msg_no_header->add_field();
    plain_field->set_name("value");
    plain_field->set_number(1);
    plain_field->set_type(google::protobuf::FieldDescriptorProto::TYPE_INT32);
    plain_field->set_label(google::protobuf::FieldDescriptorProto::LABEL_OPTIONAL);

    const google::protobuf::FileDescriptor* file_desc =
        pool_.BuildFile(file_proto);
    ASSERT_NE(file_desc, nullptr);

    test_msg_desc_ = file_desc->FindMessageTypeByName("TestMsg");
    test_rpc_req_desc_ = file_desc->FindMessageTypeByName("TestRpcReq");
    test_no_header_desc_ = file_desc->FindMessageTypeByName("TestMsgNoHeader");
    ASSERT_NE(test_msg_desc_, nullptr);
    ASSERT_NE(test_rpc_req_desc_, nullptr);
    ASSERT_NE(test_no_header_desc_, nullptr);
  }

  google::protobuf::DescriptorPool pool_;
  google::protobuf::DynamicMessageFactory factory_;
  const google::protobuf::Descriptor* test_msg_desc_ = nullptr;
  const google::protobuf::Descriptor* test_rpc_req_desc_ = nullptr;
  const google::protobuf::Descriptor* test_no_header_desc_ = nullptr;
};

TEST_F(AgiHeaderPbTest, DetectAgiHeader_Found) {
  AgiHeaderInfo info = DetectAgiHeader("pb:test.TestMsg", test_msg_desc_);
  EXPECT_TRUE(info.has_header);
  EXPECT_EQ(info.type, AgiHeaderType::kProtobuf);
  EXPECT_NE(info.pb_header_field, nullptr);
}

TEST_F(AgiHeaderPbTest, DetectAgiHeader_NotFound) {
  AgiHeaderInfo info = DetectAgiHeader("pb:test.TestMsgNoHeader", test_no_header_desc_);
  EXPECT_FALSE(info.has_header);
}

TEST_F(AgiHeaderPbTest, DetectAgiRequestHeader_Found) {
  AgiHeaderInfo info = DetectAgiRequestHeader("pb:test.TestRpcReq", test_rpc_req_desc_);
  EXPECT_TRUE(info.has_header);
  EXPECT_EQ(info.type, AgiHeaderType::kProtobuf);
  EXPECT_NE(info.pb_header_field, nullptr);
}

TEST_F(AgiHeaderPbTest, FillAgiHeader_Protobuf) {
  AgiHeaderInfo info = DetectAgiHeader("pb:test.TestMsg", test_msg_desc_);
  ASSERT_TRUE(info.has_header);

  std::unique_ptr<google::protobuf::Message> msg(
      factory_.GetPrototype(test_msg_desc_)->New());

  uint32_t seq = 42;
  std::string_view module_name = "my_module";
  uint64_t timestamp_ns = 1'500'000'000ULL;  // 1.5 seconds

  FillAgiHeader(info, msg.get(), seq, module_name, timestamp_ns);

  const auto* reflection = msg->GetReflection();
  const auto* header_field = test_msg_desc_->FindFieldByName("header");
  ASSERT_NE(header_field, nullptr);

  const auto& header = reflection->GetMessage(*msg, header_field);
  const auto* header_desc = header.GetDescriptor();
  const auto* header_refl = header.GetReflection();

  EXPECT_EQ(header_refl->GetUInt32(header, header_desc->FindFieldByName("seq_num")), 42u);
  EXPECT_EQ(header_refl->GetString(header, header_desc->FindFieldByName("publisher_name")),
            "my_module");

  const auto* time_field = header_desc->FindFieldByName("publish_time");
  ASSERT_NE(time_field, nullptr);
  const auto& time_msg = header_refl->GetMessage(header, time_field);
  const auto* time_desc = time_msg.GetDescriptor();
  const auto* time_refl = time_msg.GetReflection();

  EXPECT_EQ(time_refl->GetInt32(time_msg, time_desc->FindFieldByName("seconds")), 1);
  EXPECT_EQ(time_refl->GetUInt32(time_msg, time_desc->FindFieldByName("fraction")),
            2'147'483'648u);
}

TEST_F(AgiHeaderPbTest, FillAgiRequestHeader_Protobuf) {
  AgiHeaderInfo info = DetectAgiRequestHeader("pb:test.TestRpcReq", test_rpc_req_desc_);
  ASSERT_TRUE(info.has_header);

  std::unique_ptr<google::protobuf::Message> msg(
      factory_.GetPrototype(test_rpc_req_desc_)->New());

  uint32_t request_id = 99;
  std::string_view client_name = "rpc_client";
  uint64_t timestamp_ns = 2'000'000'000ULL;  // 2.0 seconds

  FillAgiRequestHeader(info, msg.get(), request_id, client_name, timestamp_ns);

  const auto* reflection = msg->GetReflection();
  const auto* header_field = test_rpc_req_desc_->FindFieldByName("header");
  ASSERT_NE(header_field, nullptr);

  const auto& header = reflection->GetMessage(*msg, header_field);
  const auto* header_desc = header.GetDescriptor();
  const auto* header_refl = header.GetReflection();

  EXPECT_EQ(header_refl->GetUInt32(header, header_desc->FindFieldByName("request_id")), 99u);
  EXPECT_EQ(header_refl->GetString(header, header_desc->FindFieldByName("client_name")),
            "rpc_client");

  const auto* time_field = header_desc->FindFieldByName("request_time");
  ASSERT_NE(time_field, nullptr);
  const auto& time_msg = header_refl->GetMessage(header, time_field);
  const auto* time_desc = time_msg.GetDescriptor();
  const auto* time_refl = time_msg.GetReflection();

  EXPECT_EQ(time_refl->GetInt32(time_msg, time_desc->FindFieldByName("seconds")), 2);
  EXPECT_EQ(time_refl->GetUInt32(time_msg, time_desc->FindFieldByName("fraction")), 0u);
}

#endif  // AIMRT_BUILD_WITH_PROTOBUF

}  // namespace aimrt::runtime::core::util
