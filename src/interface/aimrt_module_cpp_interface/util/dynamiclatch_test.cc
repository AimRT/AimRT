// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include <gtest/gtest.h>

#include <atomic>
#include <cassert>
#include <chrono>
#include <random>
#include <thread>
#include <vector>

#include "aimrt_module_cpp_interface/util/dynamiclatch.h"

namespace aimrt::util {

TEST(DYNAMICLATCH_TEST, Base_ZeroThenWaitImmediate) {
  DynamicLatch latch;
  latch.Wait();
}

TEST(DYNAMICLATCH_TEST, Base_AddCountDown_Simple) {
  DynamicLatch latch;
  latch.TryAdd();

  std::atomic<bool> done{false};
  std::thread t([&] {
    latch.Wait();
    done.store(true, std::memory_order_release);
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_FALSE(done.load(std::memory_order_acquire));

  latch.CountDown();
  t.join();
  EXPECT_TRUE(done.load(std::memory_order_acquire));
}

TEST(DYNAMICLATCH_TEST, CountDownToZero_NotifiesAllWaiters) {
  DynamicLatch latch;
  latch.TryAdd(2);

  std::atomic<int> woken{0};
  std::thread t1([&] {
    latch.Wait();
    woken.fetch_add(1, std::memory_order_acq_rel);
  });
  std::thread t2([&] {
    latch.Wait();
    woken.fetch_add(1, std::memory_order_acq_rel);
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(woken.load(std::memory_order_acquire), 0);

  latch.CountDown();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(woken.load(std::memory_order_acquire), 0);

  latch.CountDown();
  t1.join();
  t2.join();
  EXPECT_EQ(woken.load(std::memory_order_acquire), 2);
}

TEST(DYNAMICLATCH_TEST, Close_StopsFurtherAdd_AndWakeIfZero) {
  DynamicLatch latch;
  latch.Close();
  latch.Wait();

  latch.TryAdd();
  latch.Wait();
}

TEST(DYNAMICLATCH_TEST, Close_WithOutstandingCount_WakesOnZero) {
  DynamicLatch latch;
  latch.TryAdd(3);

  std::atomic<bool> woke{false};
  std::thread waiter([&] {
    latch.Wait();
    woke.store(true, std::memory_order_release);
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_FALSE(woke.load(std::memory_order_acquire));

  latch.Close();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_FALSE(woke.load(std::memory_order_acquire));

  latch.CountDown(2);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_FALSE(woke.load(std::memory_order_acquire));

  latch.CountDown(1);
  waiter.join();
  EXPECT_TRUE(woke.load(std::memory_order_acquire));
}

TEST(DYNAMICLATCH_TEST, Concurrency_AddAndCountDown) {
  DynamicLatch latch;

  const int producer_threads = 12;
  const int increments_per_thread = 10000;

  std::vector<std::thread> producers;
  std::vector<std::thread> consumers;
  producers.reserve(producer_threads);
  consumers.reserve(producer_threads);

  for (int i = 0; i < producer_threads; ++i) {
    producers.emplace_back([&] {
      for (int j = 0; j < increments_per_thread; ++j) {
        latch.TryAdd();
      }
    });
  }

  for (auto& t : producers) t.join();

  for (int i = 0; i < producer_threads; ++i) {
    consumers.emplace_back([&] {
      for (int j = 0; j < increments_per_thread; ++j)
        latch.CountDown();
    });
  }

  for (auto& t : consumers) t.join();

  latch.Wait();
}

TEST(DYNAMICLATCH_TEST, CountDown_Underflow_Throws) {
  DynamicLatch latch;
  latch.TryAdd(1);
  latch.CountDown(1);
  EXPECT_THROW({ latch.CountDown(1); }, aimrt::common::util::AimRTException);
}

TEST(DYNAMICLATCH_TEST, Stress_MultiRounds_WorkersAndWaiter) {
  DynamicLatch latch;
  const int rounds = 500;
  const int workers = 16;

  for (int r = 0; r < rounds; ++r) {
    std::vector<std::thread> ts;
    ts.reserve(workers);
    for (int i = 0; i < workers; ++i) {
      ts.emplace_back([&] {
        latch.TryAdd();
        std::this_thread::sleep_for(std::chrono::microseconds(50));
        latch.CountDown();
      });
    }

    std::atomic<bool> done{false};
    std::thread waiter([&] {
      latch.Wait();
      done.store(true, std::memory_order_release);
    });

    for (auto& t : ts) t.join();
    waiter.join();
    EXPECT_TRUE(done.load(std::memory_order_acquire));
  }
}

TEST(DYNAMICLATCH_TEST, Stress_Interleaved_ProducersConsumers) {
  DynamicLatch latch;
  const int producers = 8;
  const int consumers = 8;
  const int ops_per_producer = 2000;
  const int total_ops = producers * ops_per_producer;

  std::atomic<int> outstanding{0};
  std::atomic<int> produced{0};
  std::atomic<int> consumed{0};

  std::mt19937 rng(12345);  // NOSONAR
  std::uniform_int_distribution<int> us(0, 30);

  std::vector<std::thread> ps;
  std::vector<std::thread> cs;
  ps.reserve(producers);
  cs.reserve(consumers);

  for (int i = 0; i < producers; ++i) {
    ps.emplace_back([&] {
      for (int j = 0; j < ops_per_producer; ++j) {
        latch.TryAdd();
        outstanding.fetch_add(1, std::memory_order_acq_rel);
        produced.fetch_add(1, std::memory_order_acq_rel);
        std::this_thread::sleep_for(std::chrono::microseconds(us(rng)));
      }
    });
  }

  for (int i = 0; i < consumers; ++i) {
    cs.emplace_back([&] {
      while (consumed.load(std::memory_order_acquire) < total_ops) {
        int before = outstanding.load(std::memory_order_acquire);
        if (before > 0 && outstanding.compare_exchange_strong(before, before - 1, std::memory_order_acq_rel)) {
          latch.CountDown();
          consumed.fetch_add(1, std::memory_order_acq_rel);
          std::this_thread::sleep_for(std::chrono::microseconds(us(rng)));
        } else {
          std::this_thread::yield();
        }
      }
    });
  }

  for (auto& t : ps) t.join();
  for (auto& t : cs) t.join();

  EXPECT_EQ(produced.load(std::memory_order_acquire), total_ops);
  EXPECT_EQ(consumed.load(std::memory_order_acquire), total_ops);

  latch.Wait();
}

TEST(DYNAMICLATCH_TEST, AddLargeDelta_ThenCountDown) {
  DynamicLatch latch;
  const std::ptrdiff_t big = 1'000'000;
  latch.TryAdd(big);

  std::atomic<bool> released{false};
  std::thread waiter([&] {
    latch.Wait();
    released.store(true, std::memory_order_release);
  });

  for (std::ptrdiff_t i = 0; i < big; ++i) {
    latch.CountDown();
  }

  waiter.join();
  EXPECT_TRUE(released.load(std::memory_order_acquire));
}

TEST(DYNAMICLATCH_TEST, ManyWaiters_WakeOnZero) {
  DynamicLatch latch;
  const int waiters = 64;

  latch.TryAdd(1);

  std::atomic<int> woke{0};
  std::vector<std::thread> ws;
  ws.reserve(waiters);
  for (int i = 0; i < waiters; ++i) {
    ws.emplace_back([&] {
      latch.Wait();
      woke.fetch_add(1, std::memory_order_acq_rel);
    });
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_EQ(woke.load(std::memory_order_acquire), 0);

  latch.CountDown(1);
  for (auto& w : ws) w.join();
  EXPECT_EQ(woke.load(std::memory_order_acquire), waiters);
}

TEST(DYNAMICLATCH_TEST, CloseThenAddIgnored_ManyTimes) {
  DynamicLatch latch;
  latch.Close();

  for (int i = 0; i < 100000; ++i) {
    latch.TryAdd();
  }

  latch.Wait();
}

TEST(DYNAMICLATCH_TEST, Close_AddIgnored_Race_Ordering) {
  DynamicLatch latch;

  const int threads = 16;
  std::atomic<int> ready{0};
  std::atomic<bool> go{false};
  std::vector<std::thread> ts;
  ts.reserve(threads);

  for (int i = 0; i < threads; ++i) {
    ts.emplace_back([&] {
      ready.fetch_add(1, std::memory_order_acq_rel);
      while (!go.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      latch.TryAdd();
    });
  }

  while (ready.load(std::memory_order_acquire) < threads) {
    std::this_thread::yield();
  }

  latch.Close();
  go.store(true, std::memory_order_release);

  for (auto& t : ts) t.join();

  latch.Wait();
}

TEST(DYNAMICLATCH_TEST, Close_ThenCountDown_WakesAll_Waiters_Race) {
  DynamicLatch latch;
  const int initial = 1000;
  latch.TryAdd(initial);

  const int waiters = 32;
  std::atomic<int> woke{0};
  std::vector<std::thread> ws;
  ws.reserve(waiters);
  for (int i = 0; i < waiters; ++i) {
    ws.emplace_back([&] {
      latch.Wait();
      woke.fetch_add(1, std::memory_order_acq_rel);
    });
  }

  const int consumers = 8;
  std::atomic<int> remaining{initial};
  std::atomic<bool> go{false};
  std::vector<std::thread> cs;
  cs.reserve(consumers);
  for (int i = 0; i < consumers; ++i) {
    cs.emplace_back([&] {
      while (!go.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      while (true) {
        int before = remaining.load(std::memory_order_acquire);
        if (before <= 0) break;
        if (remaining.compare_exchange_strong(before, before - 1, std::memory_order_acq_rel)) {
          latch.CountDown();
        }
      }
    });
  }

  latch.Close();
  go.store(true, std::memory_order_release);

  for (auto& c : cs) c.join();
  for (auto& w : ws) w.join();

  EXPECT_EQ(woke.load(std::memory_order_acquire), waiters);
}

TEST(DYNAMICLATCH_TEST, Reuse_AfterZero_Works) {
  DynamicLatch latch;
  latch.TryAdd(2);
  latch.CountDown(2);
  latch.Wait();

  std::atomic<bool> done{false};
  std::thread waiter([&] {
    latch.Wait();
    done.store(true, std::memory_order_release);
  });

  latch.TryAdd(1);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  EXPECT_FALSE(done.load(std::memory_order_acquire));
  latch.CountDown(1);
  waiter.join();
  EXPECT_TRUE(done.load(std::memory_order_acquire));
}

TEST(DYNAMICLATCH_TEST, ManyWaiters_Close_Then_LastCountDown_Race) {
  DynamicLatch latch;
  latch.TryAdd(1);

  const int waiters = 64;
  std::atomic<int> woke{0};
  std::vector<std::thread> ws;
  ws.reserve(waiters);
  for (int i = 0; i < waiters; ++i) {
    ws.emplace_back([&] {
      latch.Wait();
      woke.fetch_add(1, std::memory_order_acq_rel);
    });
  }

  std::thread closer([&] { latch.Close(); });
  std::thread last([&] { latch.CountDown(1); });
  closer.join();
  last.join();

  for (auto& w : ws) w.join();
  EXPECT_EQ(woke.load(std::memory_order_acquire), waiters);
}

TEST(DYNAMICLATCH_TEST, Race_AddSucceedsAfterClose) {
  const int iterations = 1000;
  const int adder_threads_count = 40;

  for (int i = 0; i < iterations; ++i) {
    DynamicLatch latch;
    std::atomic<bool> go{false};
    std::atomic<int> ready_threads{0};

    std::vector<std::thread> adder_threads;
    adder_threads.reserve(adder_threads_count);

    for (int j = 0; j < adder_threads_count; ++j) {
      adder_threads.emplace_back([&] {
        ready_threads++;
        while (!go.load(std::memory_order_acquire)) {
        }
        if (latch.TryAdd()) {
          std::this_thread::sleep_for(std::chrono::microseconds(1));
          latch.CountDown();
        } else {
          std::this_thread::yield();
        }
      });
    }

    std::thread closer_thread([&] {
      ready_threads++;
      while (!go.load(std::memory_order_acquire)) {
      }
      latch.Close();
    });

    while (ready_threads.load() < (adder_threads_count + 1)) {
      std::this_thread::yield();
    }

    go.store(true, std::memory_order_release);

    closer_thread.join();
    for (auto& t : adder_threads) {
      t.join();
    }

    ASSERT_EQ(latch.GetCurrentCount(), 0)
        << "Race condition detected in iteration " << i
        << ". Count should be 0 after close, but it is not.";

    latch.Wait();
  }
}

}  // namespace aimrt::util
