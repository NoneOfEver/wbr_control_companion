#include <algorithm>
#include <charconv>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "wbr/ethercat_protocol.hpp"
#include "wbr/ethercat_transport.hpp"
#include "wbr/realtime_loop.hpp"

namespace {

struct Options {
  std::string backend = "mock";
  std::string path_label = "unspecified";
  std::string load_label = "unspecified";
  std::string interface;
  std::string csv_path = "ethercat-benchmark.csv";
  std::string json_path = "ethercat-benchmark.json";
  std::uint32_t vendor_id = 0U;
  std::uint32_t product_code = 1U;
  std::uint32_t period_us = 1000U;
  std::uint32_t duration_s = 10U;
  std::uint32_t warmup_cycles = 100U;
  int cpu = 2;
  int priority = 80;
  bool distributed_clocks = false;
  bool realtime = true;
};

struct CycleRecord {
  std::uint32_t sequence;
  std::int64_t wakeup_jitter_ns;
  std::uint64_t exchange_time_ns;
  std::uint32_t echoed_sequence;
  std::int64_t round_trip_ns;
  std::uint32_t working_counter;
  std::uint32_t status;
};

bool ParseUnsigned(std::string_view text, std::uint32_t& output) {
  int base = 10;
  if (text.size() > 2U && text[0] == '0' &&
      (text[1] == 'x' || text[1] == 'X')) {
    text.remove_prefix(2U);
    base = 16;
  }
  const auto result =
      std::from_chars(text.data(), text.data() + text.size(), output, base);
  return result.ec == std::errc{} && result.ptr == text.data() + text.size();
}

void Usage(const char* program) {
  std::cerr
      << "Usage: " << program
      << " [--backend mock|igh|soem] [--interface IFACE]"
         " [--vendor ID] [--product ID] [--period-us N] [--duration-s N]"
         " [--warmup N] [--cpu N] [--priority N] [--csv PATH]"
         " [--json PATH] [--path-label LABEL] [--load-label LABEL] [--dc]"
         " [--no-rt]\n";
}

bool ParseOptions(int argc, char** argv, Options& options) {
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--no-rt") {
      options.realtime = false;
      continue;
    }
    if (argument == "--dc") {
      options.distributed_clocks = true;
      continue;
    }
    if (argument == "--help") {
      Usage(argv[0]);
      std::exit(0);
    }
    if (i + 1 >= argc) return false;
    const std::string value = argv[++i];
    std::uint32_t number = 0U;
    if (argument == "--backend") options.backend = value;
    else if (argument == "--path-label") options.path_label = value;
    else if (argument == "--load-label") options.load_label = value;
    else if (argument == "--interface") options.interface = value;
    else if (argument == "--csv") options.csv_path = value;
    else if (argument == "--json") options.json_path = value;
    else if (!ParseUnsigned(value, number)) return false;
    else if (argument == "--vendor") options.vendor_id = number;
    else if (argument == "--product") options.product_code = number;
    else if (argument == "--period-us") options.period_us = number;
    else if (argument == "--duration-s") options.duration_s = number;
    else if (argument == "--warmup") options.warmup_cycles = number;
    else if (argument == "--cpu") options.cpu = static_cast<int>(number);
    else if (argument == "--priority") options.priority = static_cast<int>(number);
    else return false;
  }
  return options.period_us > 0U && options.duration_s > 0U;
}

std::unique_ptr<wbr::ethercat::EthercatTransport> MakeTransport(
    const std::string& backend) {
  if (backend == "mock") return wbr::ethercat::CreateMockTransport();
  if (backend == "igh") return wbr::ethercat::CreateIghTransport();
  if (backend == "soem") return wbr::ethercat::CreateSoemTransport();
  return nullptr;
}

void WriteSummary(std::ostream& output, const char* name,
                  const wbr::realtime::SampleSummary& summary,
                  bool trailing_comma) {
  output << "  \"" << name << "\": {\n"
         << "    \"count\": " << summary.count << ",\n"
         << "    \"min_ns\": " << summary.minimum_ns << ",\n"
         << "    \"mean_ns\": " << std::fixed << std::setprecision(3)
         << summary.mean_ns << ",\n"
         << "    \"p50_ns\": " << summary.p50_ns << ",\n"
         << "    \"p99_ns\": " << summary.p99_ns << ",\n"
         << "    \"p999_ns\": " << summary.p999_ns << ",\n"
         << "    \"p9999_ns\": " << summary.p9999_ns << ",\n"
         << "    \"max_ns\": " << summary.maximum_ns << "\n"
         << "  }" << (trailing_comma ? "," : "") << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!ParseOptions(argc, argv, options)) {
    Usage(argv[0]);
    return 2;
  }
  auto transport = MakeTransport(options.backend);
  if (!transport) {
    std::cerr << "Unsupported backend: " << options.backend << "\n";
    return 2;
  }

  wbr::ethercat::TransportConfig config;
  config.interface = options.interface;
  config.vendor_id = options.vendor_id;
  config.product_code = options.product_code;
  config.cycle_time_ns = options.period_us * 1000U;
  config.enable_distributed_clocks = options.distributed_clocks;
  std::string error;
  if (!transport->Open(config, error)) {
    std::cerr << "Failed to open " << options.backend << ": " << error << "\n";
    return 1;
  }

  const std::uint64_t measured_cycles =
      static_cast<std::uint64_t>(options.duration_s) * 1'000'000U /
      options.period_us;
  const std::uint64_t total_cycles = measured_cycles + options.warmup_cycles;
  std::vector<CycleRecord> records;
  records.reserve(static_cast<std::size_t>(measured_cycles));
  std::vector<std::int64_t> wakeup_samples;
  std::vector<std::int64_t> exchange_samples;
  std::vector<std::int64_t> round_trip_samples;
  wakeup_samples.reserve(records.capacity());
  exchange_samples.reserve(records.capacity());
  round_trip_samples.reserve(records.capacity());
  constexpr std::size_t kHistorySize = 65536U;
  std::vector<std::uint64_t> send_times(kHistorySize, 0U);
  std::vector<std::uint32_t> sent_sequences(kHistorySize, 0U);

  if (options.realtime &&
      !wbr::realtime::ConfigureCurrentThread(options.cpu, options.priority,
                                             error)) {
    std::cerr << "Realtime setup failed: " << error << "\n";
    return 1;
  }

  wbr::ethercat::MasterProcessImage output{};
  wbr::ethercat::SlaveProcessImage input{};
  output.benchmark.command = wbr::ethercat::kBenchmarkEcho;
  const std::uint64_t period_ns = config.cycle_time_ns;
  std::uint64_t deadline = wbr::realtime::MonotonicTimeNs() + 10U * period_ns;
  std::uint64_t bad_echoes = 0U;
  std::uint64_t bad_payloads = 0U;
  std::uint64_t exchange_failures = 0U;
  std::uint64_t wkc_failures = 0U;

  for (std::uint64_t cycle = 0U; cycle < total_cycles; ++cycle) {
    if (!wbr::realtime::SleepUntilNs(deadline, error)) {
      std::cerr << error << "\n";
      return 1;
    }
    const std::uint64_t wake_time = wbr::realtime::MonotonicTimeNs();
    const std::uint32_t sequence = static_cast<std::uint32_t>(cycle + 1U);
    output.benchmark.sequence = sequence;
    output.benchmark.host_send_time_ns = wake_time;
    output.benchmark.payload0 = sequence ^ 0xa5a55a5aU;
    output.benchmark.payload1 = sequence * 2654435761U;
    output.benchmark.payload_crc32 = wbr::ethercat::PayloadCrc32(
        sequence, output.benchmark.payload0, output.benchmark.payload1);
    const std::size_t slot = sequence % kHistorySize;
    send_times[slot] = wake_time;
    sent_sequences[slot] = sequence;

    const std::uint64_t exchange_begin = wbr::realtime::MonotonicTimeNs();
    const bool exchanged =
        transport->Exchange(output, input, deadline, error);
    const std::uint64_t exchange_end = wbr::realtime::MonotonicTimeNs();
    const auto status = transport->status();
    const bool measured = cycle >= options.warmup_cycles;
    if (measured && !exchanged) ++exchange_failures;
    if (measured && status.expected_working_counter != 0U &&
        status.working_counter != status.expected_working_counter)
      ++wkc_failures;

    std::int64_t round_trip_ns = -1;
    const std::uint32_t echoed = input.benchmark.echoed_sequence;
    const std::size_t echo_slot = echoed % kHistorySize;
    if (echoed != 0U && sent_sequences[echo_slot] == echoed) {
      round_trip_ns = static_cast<std::int64_t>(exchange_end - send_times[echo_slot]);
    } else if (measured) {
      ++bad_echoes;
    }
    if (measured && echoed != 0U &&
        (input.benchmark.status & wbr::ethercat::kBenchmarkPayloadValid) == 0U) {
      ++bad_payloads;
    }

    if (measured) {
      const std::int64_t wakeup_jitter =
          static_cast<std::int64_t>(wake_time - deadline);
      const std::int64_t exchange_time =
          static_cast<std::int64_t>(exchange_end - exchange_begin);
      records.push_back({sequence, wakeup_jitter,
                         static_cast<std::uint64_t>(exchange_time), echoed,
                         round_trip_ns, status.working_counter,
                         input.benchmark.status});
      wakeup_samples.push_back(wakeup_jitter);
      exchange_samples.push_back(exchange_time);
      if (round_trip_ns >= 0) round_trip_samples.push_back(round_trip_ns);
    }
    deadline += period_ns;
  }

  std::ofstream csv(options.csv_path);
  csv << "sequence,wakeup_jitter_ns,exchange_time_ns,echoed_sequence,"
         "round_trip_ns,working_counter,slave_status\n";
  for (const auto& record : records) {
    csv << record.sequence << ',' << record.wakeup_jitter_ns << ','
        << record.exchange_time_ns << ',' << record.echoed_sequence << ','
        << record.round_trip_ns << ',' << record.working_counter << ','
        << record.status << '\n';
  }

  const auto wakeup_summary = wbr::realtime::Summarize(wakeup_samples);
  const auto exchange_summary = wbr::realtime::Summarize(exchange_samples);
  const auto round_trip_summary = wbr::realtime::Summarize(round_trip_samples);
  std::ofstream json(options.json_path);
  json << "{\n"
       << "  \"backend\": \"" << transport->name() << "\",\n"
       << "  \"path\": \"" << options.path_label << "\",\n"
       << "  \"load\": \"" << options.load_label << "\",\n"
       << "  \"distributed_clocks\": "
       << (options.distributed_clocks ? "true" : "false") << ",\n"
       << "  \"period_us\": " << options.period_us << ",\n"
       << "  \"duration_s\": " << options.duration_s << ",\n"
       << "  \"cpu\": " << options.cpu << ",\n"
       << "  \"priority\": " << options.priority << ",\n"
       << "  \"bad_echoes\": " << bad_echoes << ",\n"
       << "  \"bad_payloads\": " << bad_payloads << ",\n"
       << "  \"exchange_failures\": " << exchange_failures << ",\n"
       << "  \"wkc_failures\": " << wkc_failures << ",\n";
  WriteSummary(json, "wakeup_jitter", wakeup_summary, true);
  WriteSummary(json, "exchange_time", exchange_summary, true);
  WriteSummary(json, "round_trip", round_trip_summary, false);
  json << "}\n";

  std::cout << "backend=" << transport->name() << " samples=" << records.size()
            << " bad_echoes=" << bad_echoes << " bad_payloads=" << bad_payloads
            << " wkc_failures=" << wkc_failures << "\n"
            << "wakeup p99/max [ns]: " << wakeup_summary.p99_ns << '/'
            << wakeup_summary.maximum_ns << "\n"
            << "round-trip p99/max [ns]: " << round_trip_summary.p99_ns << '/'
            << round_trip_summary.maximum_ns << "\n";
  return exchange_failures == 0U && wkc_failures == 0U && bad_echoes == 0U &&
                 bad_payloads == 0U
             ? 0
             : 1;
}
