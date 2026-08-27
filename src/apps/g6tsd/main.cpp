// SPDX-License-Identifier: GPL-2.0-or-later

#include <apps/daemon/daemon.hpp>

#include <common/types.hpp>
#include <core/generic/config.hpp>
#include <core/generic/device.hpp>
#include <core/generic/g6ts.hpp>
#include <core/linux/signal-handler.hpp>
#include <ipts/metadata.hpp>

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <atomic>
#include <chrono>
#include <vector>

namespace iptsd::apps::g6tsd {
namespace {

/*
 * GET_INFO payload of the /dev/g6ts-heat ABI (48 bytes). Mirrors the layout
 * validated by g6-pen: ABI version, info size, record header length, maximum
 * content length, queue capacity and the set of supported boundary flags.
 */
struct [[gnu::packed]] HeatInfo {
	u16 abi_version = 0;
	u16 struct_size = 0;
	u16 record_header_size = 0;
	u16 reserved0 = 0;
	u32 max_content_size = 0;
	u32 queue_capacity = 0;
	u64 supported_record_flags = 0;
	u64 reserved[3] = {};
};

static_assert(sizeof(HeatInfo) == 48);

constexpr u8 HEAT_IOC_GET_INFO = 0; // _IOR('G', 0x00, HeatInfo)

constexpr u64 READ_TIMEOUT_MS = 100;
constexpr u64 STALE_NS = 1000000000; // lift the stylus after 1 s of silence

int validate_info(const int fd)
{
	HeatInfo info {};

	if (ioctl(fd, _IOR('G', HEAT_IOC_GET_INFO, HeatInfo), &info) < 0)
		return -errno;

	if (info.abi_version != g6ts::ABI_VERSION ||
	    info.struct_size != sizeof(HeatInfo) ||
	    info.record_header_size != g6ts::HEADER_LEN ||
	    info.max_content_size != g6ts::MAX_CONTENT ||
	    info.queue_capacity == 0 ||
	    (info.supported_record_flags & g6ts::FLAG_BOUNDARY) !=
		g6ts::FLAG_BOUNDARY) {
		spdlog::error("The device exposes an incompatible G6 HEAT ABI");
		return -EPROTO;
	}

	return 0;
}

int run(const int argc, const char **argv)
{
	std::filesystem::path path {"/dev/g6ts-heat"};
	u64 stale_ms = STALE_NS / 1000000;

	CLI::App app {"Daemon to translate Surface Pro 11 G6 HEAT pen data "
		      "into Linux input events"};
	app.add_option("-d,--device", path)
		->description("The G6 HEAT device node")
		->type_name("FILE");
	app.add_option("--stale-ms", stale_ms)
		->description("Lift the stylus after this much silence");
	CLI11_PARSE(app, argc, argv);

	const int fd = ::open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
	if (fd < 0) {
		spdlog::error("Cannot open {}: {}", path.c_str(), errno);
		return EXIT_FAILURE;
	}

	if (validate_info(fd) != 0)
		return EXIT_FAILURE;

	core::Config config {};
	config.width = 27.39;
	config.height = 18.26;
	// G6 HEAT DFT component amplitudes are far smaller than the Intel
	// IPTS rows this default was tuned for; a 50-floor caused spurious
	// stylus lifts on valid cycles.
	config.dft_position_min_amp = 0;
	config.touchscreen_disable = true; // stylus only: HEAT carries pen data

	core::DeviceInfo info {};
	info.vendor = 0x045E;
	info.product = 0x0C83;
	info.type = ipts::Device::Type::Touchscreen;
	info.meta = ipts::Metadata {};
	info.meta->rows = 46;
	info.meta->columns = 68;
	info.meta->width = config.width;
	info.meta->height = config.height;

	class G6tsDaemon final : public iptsd::apps::daemon::Daemon {
	public:
		using Daemon::Daemon;

		void ingest(std::vector<u8> &frames)
		{
			this->on_data(frames);
		}
	};

	G6tsDaemon daemon {config, info};

	g6ts::Bundler bundler {};
	g6ts::ContactDetector contact {};
	u32 group_counter = 0;
	u64 last_cycle_ns = 0;
	bool stylus_active = false;
	std::atomic_bool should_stop {false};

	bundler.on_cycle = [&](const g6ts::Cycle &cycle) {
		last_cycle_ns = cycle.last_timestamp_ns;

		auto frames = g6ts::serialize_cycle(cycle, group_counter++,
						    contact);
		if (frames.has_value()) {
			daemon.ingest(*frames);
			stylus_active = true;
		}
	};

	const auto _sigterm =
		core::linux::signal<SIGTERM>([&](int) { should_stop = true; });
	const auto _sigint =
		core::linux::signal<SIGINT>([&](int) { should_stop = true; });

	daemon.on_start();

	std::vector<u8> wire(g6ts::HEADER_LEN + g6ts::MAX_CONTENT);

	while (!should_stop.load()) {
		struct pollfd pfd {fd, POLLIN, 0};
		const int polled = ::poll(&pfd, 1, static_cast<int>(READ_TIMEOUT_MS));
		if (polled < 0) {
			if (errno == EINTR)
				continue;
			spdlog::error("poll failed: {}", errno);
			return EXIT_FAILURE;
		}

		if (polled > 0 && (pfd.revents & POLLIN)) {
			const ssize_t size = ::read(fd, wire.data(), wire.size());
			if (size < 0) {
				if (errno == EAGAIN)
					continue;
				spdlog::error("read failed: {}", errno);
				return EXIT_FAILURE;
			}

			const auto record =
				g6ts::decode_record(gsl::span<const u8> {
					wire.data(), static_cast<usize>(size)});
			if (record.has_value())
				bundler.feed(*record);
		}

		// HEAT cycles stop arriving when the pen leaves range. Feed a
		// zero-magnitude window so the DFT solver lifts the stylus.
		// Record timestamps are CLOCK_MONOTONIC.
		if (stylus_active) {
			struct timespec now {};
			clock_gettime(CLOCK_MONOTONIC, &now);
			const u64 now_ns = static_cast<u64>(now.tv_sec) *
						   1000000000ull +
					   static_cast<u64>(now.tv_nsec);

			if (last_cycle_ns &&
			    now_ns > last_cycle_ns + stale_ms * 1000000ull) {
				std::vector<u8> frames = g6ts::serialize_lift(
					static_cast<u32>(now_ns / 1000000));
				daemon.ingest(frames);
				stylus_active = false;
			}
		}
	}

	daemon.on_stop();
	::close(fd);
	return EXIT_SUCCESS;
}

} // namespace
} // namespace iptsd::apps::g6tsd

int main(const int argc, const char **argv)
{
	spdlog::set_pattern("[%X.%e] [%^%l%$] %v");

	try {
		return iptsd::apps::g6tsd::run(argc, argv);
	} catch (const std::exception &e) {
		spdlog::error(e.what());
		return EXIT_FAILURE;
	}
}
