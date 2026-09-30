#include "wbr/realtime_loop.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <numeric>

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>

namespace wbr::realtime {

std::uint64_t MonotonicTimeNs() {
  timespec now{};
  clock_gettime(CLOCK_MONOTONIC, &now);
  return static_cast<std::uint64_t>(now.tv_sec) * 1'000'000'000ULL +
         static_cast<std::uint64_t>(now.tv_nsec);
}

bool ConfigureCurrentThread(int cpu, int priority, std::string& error) {
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    error = std::string("mlockall: ") + std::strerror(errno);
    return false;
  }
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(cpu, &cpuset);
  int result = pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
  if (result != 0) {
    error = std::string("pthread_setaffinity_np: ") + std::strerror(result);
    return false;
  }
  sched_param parameters{};
  parameters.sched_priority = priority;
  result = pthread_setschedparam(pthread_self(), SCHED_FIFO, &parameters);
  if (result != 0) {
    error = std::string("pthread_setschedparam: ") + std::strerror(result);
    return false;
  }
  return true;
}

bool SleepUntilNs(std::uint64_t absolute_time_ns, std::string& error) {
  timespec deadline{};
  deadline.tv_sec = static_cast<time_t>(absolute_time_ns / 1'000'000'000ULL);
  deadline.tv_nsec = static_cast<long>(absolute_time_ns % 1'000'000'000ULL);
  int result;
  do {
    result = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
  } while (result == EINTR);
  if (result != 0) {
    error = std::string("clock_nanosleep: ") + std::strerror(result);
    return false;
  }
  return true;
}

SampleSummary Summarize(std::vector<std::int64_t> samples) {
  SampleSummary result{};
  if (samples.empty()) return result;
  std::sort(samples.begin(), samples.end());
  auto percentile = [&samples](double quantile) {
    const double position = quantile * static_cast<double>(samples.size() - 1U);
    return samples[static_cast<std::size_t>(position)];
  };
  result.count = samples.size();
  result.minimum_ns = samples.front();
  result.maximum_ns = samples.back();
  result.mean_ns = static_cast<double>(
      std::accumulate(samples.begin(), samples.end(), std::int64_t{0})) /
      static_cast<double>(samples.size());
  result.p50_ns = percentile(0.50);
  result.p99_ns = percentile(0.99);
  result.p999_ns = percentile(0.999);
  result.p9999_ns = percentile(0.9999);
  return result;
}

}  // namespace wbr::realtime
