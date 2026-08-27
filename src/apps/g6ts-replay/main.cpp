// SPDX-License-Identifier: GPL-2.0-or-later

#include <common/types.hpp>
#include <core/generic/application.hpp>
#include <core/generic/config.hpp>
#include <core/generic/device.hpp>
#include <core/generic/g6ts.hpp>
#include <ipts/samples/stylus.hpp>

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

#include <cinttypes>
#include <fstream>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace iptsd::apps::g6ts_replay {
namespace {

/*
 * Offline replay of G6 HEAT corpora (G6T1 text form) through the G6 -> DFT
 * bridge. Every complete HEAT cycle is serialized into one DftWindow; every
 * stylus update is written to stdout as one JSON line, keyed by cycle index.
 * The output can be scored against the Windows processor reports of the same
 * capture (processor-pen-reports-P4-P8.csv in ooaklee/sp11-windows-capture).
 */
class ReplayApp : public core::Application {
public:
	ReplayApp(const core::Config &config, const core::DeviceInfo &info)
		: core::Application {config, info}
	{
	}

	/*
	 * Bridges a serialized HEAT cycle into the protected parser entry.
	 */
	void ingest(std::vector<u8> &frames)
	{
		this->on_data(frames);
	}

	void on_stylus(const ipts::samples::Stylus &stylus) override
	{
		std::printf("{\"cycle\":%" PRIu64 ",\"t_ms\":%.3f,\"pe\":%u,"
			    "\"det11\":%d,"
			    "\"x\":%.9f,\"y\":%.9f,\"proximity\":%s,"
			    "\"contact\":%s,\"pressure\":%.6f,\"button\":%s,"
			    "\"rubber\":%s}\n",
			    cycle, cycle_ms, pressure_energy, det11, stylus.x, stylus.y,
			    stylus.proximity ? "true" : "false",
			    stylus.contact ? "true" : "false", stylus.pressure,
			    stylus.button ? "true" : "false",
			    stylus.rubber ? "true" : "false");
		updates++;
	}

	u64 cycle = 0;
	f64 cycle_ms = 0;
	u32 pressure_energy = 0;
	int det11 = -1;
	usize updates = 0;
	g6ts::ContactDetector contact {};
};

u32 parse_u32(const std::string &text)
{
	return static_cast<u32>(std::stoul(text, nullptr, 0));
}

u64 parse_u64(const std::string &text)
{
	return std::stoull(text, nullptr, 0);
}

u8 parse_u8(const std::string &text)
{
	return static_cast<u8>(std::stoul(text, nullptr, 0));
}

std::vector<u8> parse_hex(const std::string &hex)
{
	std::vector<u8> bytes {};
	bytes.reserve(hex.size() / 2);

	for (usize i = 0; i + 1 < hex.size(); i += 2)
		bytes.push_back(static_cast<u8>(
			std::stoul(hex.substr(i, 2), nullptr, 16)));

	return bytes;
}

int replay(const std::string &path, g6ts::Bundler &bundler)
{
	std::ifstream file {path};
	if (!file.is_open()) {
		spdlog::error("Cannot open {}", path);
		return EXIT_FAILURE;
	}

	std::string line;
	bool have_header = false;
	usize line_number = 0;

	while (std::getline(file, line)) {
		line_number++;

		const usize start = line.find_first_not_of(" \t\r");
		if (start == std::string::npos)
			continue;
		const std::string trimmed = line.substr(start);
		if (trimmed.empty() || trimmed[0] == '#')
			continue;

		if (!have_header) {
			if (trimmed != "G6T1") {
				spdlog::error("{}: not a G6T1 corpus", path);
				return EXIT_FAILURE;
			}
			have_header = true;
			continue;
		}

		std::istringstream stream {trimmed};
		std::string generation_str;
		std::string timestamp_str;
		std::string sequence_str;
		std::string report_id_str;
		std::string flags_str;
		std::string content_str;

		if (!(stream >> generation_str >> timestamp_str >>
		      sequence_str >> report_id_str >> flags_str >>
		      content_str)) {
			spdlog::error("{}:{}: invalid record", path,
				      line_number);
			return EXIT_FAILURE;
		}

		g6ts::Record record {};
		record.generation = parse_u32(generation_str);
		record.timestamp_ns = parse_u64(timestamp_str);
		record.sequence = parse_u32(sequence_str);
		record.report_id = parse_u8(report_id_str);
		record.flags = parse_u8(flags_str);

		const std::vector<u8> content = parse_hex(content_str);
		record.content_len = static_cast<u16>(content.size());
		record.content = content;

		bundler.feed(record);
	}

	return EXIT_SUCCESS;
}

int run(const int argc, const char **argv)
{
	std::vector<std::string> paths {};

	CLI::App app {"Replay G6 HEAT corpora through the IPTS DFT bridge"};
	app.add_option("CORPUS", paths)
		->description("G6T1 corpus files to replay")
		->type_name("FILE")
		->required();
	CLI11_PARSE(app, argc, argv);

	core::Config config {};
	config.width = 27.39;
	config.height = 18.26;
	// G6 HEAT DFT component amplitudes are far smaller than the Intel
	// IPTS rows this default was tuned for; a 50-floor caused spurious
	// stylus lifts on valid cycles.
	config.dft_position_min_amp = 0;

	core::DeviceInfo info {};
	info.vendor = 0x045E;
	info.product = 0x0C83;
	info.type = ipts::Device::Type::Touchscreen;
	info.meta = ipts::Metadata {};
	info.meta->rows = 46;
	info.meta->columns = 68;
	info.meta->width = config.width;
	info.meta->height = config.height;

	ReplayApp replay_app {config, info};
	g6ts::Bundler bundler {};

	u32 group_counter = 0;

	usize cycles_with_pressure = 0;
	u32 pressure_energy_max = 0;

	bundler.on_cycle = [&](const g6ts::Cycle &cycle) {
		replay_app.cycle++;
		replay_app.cycle_ms =
			static_cast<f64>(cycle.last_timestamp_ns) / 1000000.0;

		const auto pb = g6ts::pressure_banks(cycle);
		if (pb.has_value()) {
			cycles_with_pressure++;
			pressure_energy_max =
				std::max(pressure_energy_max, pb->banks.max_energy());
		} else if (replay_app.cycle < 6) {
			const g6ts::Part &second =
				cycle.parts[static_cast<usize>(
					g6ts::PartIndex::HeatSecond)];
			spdlog::info("cycle {}: second 0x0B present={} len={}",
				     replay_app.cycle, second.present,
				     second.content_len);
		}

		const auto pbank = g6ts::pressure_banks(cycle);
		replay_app.pressure_energy =
			pbank.has_value() ? pbank->banks.max_energy() : 0;
		replay_app.det11 =
			pbank.has_value() && pbank->has_detection ?
				pbank->detection[11] : -1;

		auto frames = g6ts::serialize_cycle(cycle, group_counter++,
						    replay_app.contact);
		if (frames.has_value())
			replay_app.ingest(*frames);
	};

	spdlog::set_level(spdlog::level::debug);

	for (const std::string &path : paths)
		if (replay(path, bundler) != EXIT_SUCCESS)
			return EXIT_FAILURE;

	spdlog::info("cycles={} stylus_updates={}", replay_app.cycle,
		     replay_app.updates);

	return EXIT_SUCCESS;
}

	} // namespace
} // namespace iptsd::apps::g6ts_replay

int main(const int argc, const char **argv)
{
	spdlog::set_pattern("[%X.%e] [%^%l%$] %v");

	try {
		return iptsd::apps::g6ts_replay::run(argc, argv);
	} catch (const std::exception &e) {
		spdlog::error(e.what());
		return EXIT_FAILURE;
	}
}
