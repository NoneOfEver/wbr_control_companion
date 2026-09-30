#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace wbr::realtime {

std::uint64_t MonotonicTimeNs();
bool ConfigureCurrentThread(int cpu, int priority, std::string& error);
bool SleepUntilNs(std::uint64_t absolute_time_ns, std::string& error);

struct SampleSummary {
  std::uint64_t count = 0U;
  std::int64_t minimum_ns = 0;
  double mean_ns = 0.0;
  std::int64_t p50_ns = 0;
  std::int64_t p99_ns = 0;
  std::int64_t p999_ns = 0;
  std::int64_t p9999_ns = 0;
  std::int64_t maximum_ns = 0;
};

SampleSummary Summarize(std::vector<std::int64_t> samples);

}  // namespace wbr::realtime
