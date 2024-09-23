// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include "util/block_queue.h"

namespace aimrt::common::util {

TEST(BlockQueueTest, EnqueueDequeue) {
  BlockQueue<int> queue;
  queue.Enqueue(1);
  ASSERT_EQ(1, queue.Dequeue());
}

TEST(BlockQueueTest, TryDequeueEmpty) {
  BlockQueue<int> queue;
  auto result = queue.TryDequeue();
  ASSERT_EQ(std::nullopt, result);
}

TEST(BlockQueueTest, TryDequeueNonEmpty) {
  BlockQueue<int> queue;
  queue.Enqueue(1);
  auto result = queue.TryDequeue();
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(1, result.value());
}

TEST(BlockQueueTest, MultipleThreads) {
  BlockQueue<int> queue;
  std::vector<std::thread> threads;

  threads.emplace_back([&]() {
    for (int i = 0; i < 5; ++i) {
      queue.Enqueue(i);
    }
  });

  threads.emplace_back([&]() {
    for (int i = 0; i < 5; ++i) {
      auto item = queue.Dequeue();
      ASSERT_GE(item, 0);
    }
  });

  for (auto& t : threads) {
    t.join();
  }
}

TEST(BlockQueueTest, Stop) {
  BlockQueue<int> queue;

  queue.Stop();

  ASSERT_THROW(queue.Enqueue(1), BlockQueueStoppedException);
  ASSERT_THROW(queue.Dequeue(), BlockQueueStoppedException);
}

}  // namespace aimrt::common::util
