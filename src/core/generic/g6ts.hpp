// SPDX-License-Identifier: GPL-2.0-or-later

#ifndef IPTSD_CORE_GENERIC_G6TS_HPP
#define IPTSD_CORE_GENERIC_G6TS_HPP

#include <common/types.hpp>

#include <ipts/protocol/dft.hpp>
#include <ipts/protocol/hid.hpp>
#include <ipts/protocol/report.hpp>

#include <gsl/gsl>

#include <algorithm>
#include <array>
#include <cstring>
#include <functional>
#include <optional>
#include <vector>

namespace iptsd::g6ts {

/*
 * Bridge between the Surface Pro 11 G6 digitizer (/dev/g6ts-heat) and the
 * IPTS DFT-based stylus pipeline. The G6 HEAT antenna vectors share the
 * 48-byte layout of ipts::protocol::dft::Row (frequency, magnitude, 9x real,
 * 9x imag, window tuple), so a complete HEAT cycle can be serialized into
 * DftMetadata + DftWindow frames and processed by the stock DftStylus solver.
 *
 * The bundler state machine is a port of g6-pen's g6_processor.c (see
 * ooaklee/linux-surface-pro-11-oe, ADR0059/ADR0060): cycles anchor on report
 * 0x0C, span at most 30 ms, tolerate six observed report orderings, and treat
 * resets, suspends, transport faults, generation changes and sequence gaps as
 * authoritative boundaries.
 */

constexpr u32 MAGIC = 0x31483647; // 'G6H1'
constexpr u16 ABI_VERSION = 1;
constexpr u16 HEADER_LEN = 32;
constexpr u16 MAX_CONTENT = 4349;

constexpr u8 FLAG_RESET = 0x01;
constexpr u8 FLAG_SUSPEND = 0x02;
constexpr u8 FLAG_TRANSPORT_FAULT = 0x04;
constexpr u8 FLAG_BOUNDARY = FLAG_RESET | FLAG_SUSPEND | FLAG_TRANSPORT_FAULT;

constexpr u8 REPORT_HEAT = 0x0C;
constexpr u8 REPORT_HEAT_EARLY = 0x0B;
constexpr u8 REPORT_HEAT_LATE = 0x0D;
constexpr u8 REPORT_GROUP = 0x1A;
constexpr u8 REPORT_SIDEBAND_A = 0x07;
constexpr u8 REPORT_SIDEBAND_B = 0x6E;

constexpr u64 CYCLE_WINDOW_NS = 30000000;

constexpr usize BANKS = 2;
constexpr usize MAX_VECTORS = 16;
constexpr usize ROW_SIZE = 48;

constexpr auto FRAME_REPORTS = ipts::protocol::hid::FrameType::Reports;
constexpr auto TYPE_DFT_METADATA = ipts::protocol::report::Type::DftMetadata;
constexpr auto TYPE_DFT_WINDOW = ipts::protocol::report::Type::DftWindow;
constexpr auto TYPE_DFT_POSITION = ipts::protocol::dft::Type::Position;
constexpr auto TYPE_DFT_PRESSURE = ipts::protocol::dft::Type::Pressure;

/*
 * Contact detector thresholds, derived from the P4-P8 Windows captures
 * (ooaklee/sp11-windows-capture, heat-5c-features-P4-P8.csv): the region-4
 * pressure antennas of the second 0x0B record carry a median 5.8M maximum
 * energy during contact versus 0.22M during hover. Frame-level error rates at
 * the midpoint threshold are ~2% false / ~12% missed before debouncing; the
 * two-cycle hysteresis below reduces both in practice. These are evidence-
 * gated starting points, not final values.
 */
constexpr u32 CONTACT_ON_ENERGY = 2528523;
constexpr u32 CONTACT_OFF_ENERGY = 1200000;
constexpr u8 CONTACT_DEBOUNCE_FRAMES = 2;

constexpr usize PRESSURE_ROWS = 6;

struct Record {
	u32 generation = 0;
	u64 timestamp_ns = 0;
	u32 sequence = 0;
	u16 content_len = 0;
	u8 report_id = 0;
	u8 flags = 0;
	gsl::span<const u8> content {};
};

inline u16 read_le16(const u8 *data)
{
	return static_cast<u16>(data[0] | (data[1] << 8));
}

inline u32 read_le32(const u8 *data)
{
	return static_cast<u32>(data[0]) | (static_cast<u32>(data[1]) << 8) |
	       (static_cast<u32>(data[2]) << 16) |
	       (static_cast<u32>(data[3]) << 24);
}

inline u64 read_le64(const u8 *data)
{
	return static_cast<u64>(read_le32(data)) |
	       (static_cast<u64>(read_le32(data + 4)) << 32);
}

/*!
 * Validates and decodes one G6H1 record from its wire form.
 *
 * @param[in] wire The raw record bytes.
 * @return The decoded record, or nullopt if the record is malformed.
 */
inline std::optional<Record> decode_record(const gsl::span<const u8> wire)
{
	if (static_cast<usize>(wire.size()) < HEADER_LEN)
		return std::nullopt;

	const auto *data = wire.data();
	const u16 version = read_le16(data + 4);
	const u16 header_len = read_le16(data + 6);
	const u32 record_len = read_le32(data + 8);
	const u16 content_len = read_le16(data + 28);

	if (read_le32(data) != MAGIC)
		return std::nullopt;
	if (version != ABI_VERSION || header_len != HEADER_LEN)
		return std::nullopt;
	if (content_len > MAX_CONTENT ||
	    record_len != static_cast<u32>(header_len + content_len) ||
	    record_len != static_cast<u32>(wire.size()))
		return std::nullopt;
	if ((data[31] & FLAG_BOUNDARY) && (data[30] || content_len))
		return std::nullopt;

	Record record {};
	record.generation = read_le32(data + 12);
	record.timestamp_ns = read_le64(data + 16);
	record.sequence = read_le32(data + 24);
	record.content_len = content_len;
	record.report_id = data[30];
	record.flags = data[31];
	record.content = wire.subspan(header_len, content_len);
	return record;
}

enum class PartIndex : u8 {
	Heat = 0,
	HeatEarly,
	HeatLate,
	Group,
	HeatSecond,
};

struct Part {
	bool present = false;
	u64 timestamp_ns = 0;
	u16 content_len = 0;
	std::array<u8, MAX_CONTENT> content {};
};

struct Cycle {
	std::array<Part, 5> parts {};
	u32 generation = 0;
	u64 first_timestamp_ns = 0;
	u64 last_timestamp_ns = 0;
};

/*!
 * One G6 HEAT antenna vector, laid out identically to
 * ipts::protocol::dft::Row.
 */
struct Vector {
	u32 frequency = 0;
	u32 magnitude = 0;
	std::array<i16, 9> real {};
	std::array<i16, 9> imag {};
	i8 first = 0;
	i8 last = 0;
	i8 mid = 0;
	i8 zero = 0;
	bool trailer_valid = false;
};

struct BankSet {
	u8 region = 0;
	u8 channel = 0;
	u8 count = 0;
	std::vector<Vector> x {};
	std::vector<Vector> y {};

	[[nodiscard]] u32 max_energy() const
	{
		u32 energy = 0;
		for (const Vector &v : this->x)
			energy = std::max(energy, v.magnitude);
		for (const Vector &v : this->y)
			energy = std::max(energy, v.magnitude);
		return energy;
	}
};

/*!
 * Assembles HEAT records into complete five-part cycles.
 *
 * The port keeps the boundary semantics of g6-pen: sequence gaps, generation
 * changes and boundary-flagged records clear pending state, report 0x0C
 * anchors a new cycle, non-anchored records are dropped, and records that do
 * not arrive within 30 ms of their anchor are discarded.
 */
class Bundler {
public:
	/*!
	 * The callback invoked for every complete cycle.
	 */
	std::function<void(const Cycle &)> on_cycle;

	usize incomplete_cycles = 0;
	usize unanchored_records = 0;
	usize sequence_gaps = 0;
	usize generation_boundaries = 0;
	usize sideband_records = 0;

	/*!
	 * Feeds one record into the bundler.
	 *
	 * @param[in] record The decoded record.
	 */
	void feed(const Record &record)
	{
		tick(record.timestamp_ns);

		if (sequence_valid && record.sequence != last_sequence + 1)
			clear(true);

		last_sequence = record.sequence;
		sequence_valid = true;

		if (!generation_valid) {
			generation = record.generation;
			generation_valid = true;
		} else if (record.generation != generation) {
			clear(true);
			generation_boundaries++;
			generation = record.generation;
			generation_valid = true;
		}

		if (record.flags & FLAG_BOUNDARY) {
			clear(true);
			return;
		}

		if (record.report_id == REPORT_SIDEBAND_A ||
		    record.report_id == REPORT_SIDEBAND_B) {
			sideband_records++;
			return;
		}

		const auto index = part_index(cycle, record.report_id);
		if (!index.has_value()) {
			clear(false);
			return;
		}

		if (record.report_id == REPORT_HEAT) {
			if (cycle_count(cycle))
				clear(true);
		} else if (!cycle.parts[static_cast<usize>(PartIndex::Heat)]
				    .present) {
			unanchored_records++;
			return;
		}

		const u64 anchor = cycle.first_timestamp_ns;
		if (record.report_id != REPORT_HEAT &&
		    record.timestamp_ns > anchor &&
		    record.timestamp_ns - anchor > CYCLE_WINDOW_NS) {
			clear(true);
			unanchored_records++;
			return;
		}

		Part &part = cycle.parts[static_cast<usize>(*index)];
		if (part.present) {
			clear(true);
			unanchored_records++;
			return;
		}

		part.present = true;
		part.timestamp_ns = record.timestamp_ns;
		part.content_len = record.content_len;
		if (record.content_len)
			std::memcpy(part.content.data(), record.content.data(),
				    record.content_len);

		if (record.report_id == REPORT_HEAT)
			cycle.first_timestamp_ns = record.timestamp_ns;
		cycle.last_timestamp_ns = record.timestamp_ns;
		cycle.generation = record.generation;

		if (cycle_complete(cycle) && on_cycle) {
			on_cycle(cycle);
			clear(false);
		}
	}

private:
	static u8 cycle_count(const Cycle &c)
	{
		u8 count = 0;
		for (const Part &part : c.parts)
			count += part.present;
		return count;
	}

	static bool cycle_complete(const Cycle &c)
	{
		for (const Part &part : c.parts) {
			if (!part.present)
				return false;
		}
		return true;
	}

	static std::optional<PartIndex> part_index(const Cycle &c, u8 report_id)
	{
		switch (report_id) {
		case REPORT_HEAT:
			return PartIndex::Heat;
		case REPORT_GROUP:
			return PartIndex::Group;
		case REPORT_HEAT_LATE:
			return PartIndex::HeatLate;
		case REPORT_HEAT_EARLY:
			if (!c.parts[static_cast<usize>(PartIndex::HeatEarly)]
				     .present)
				return PartIndex::HeatEarly;
			return PartIndex::HeatSecond;
		default:
			return std::nullopt;
		}
	}

	void clear(bool incomplete)
	{
		if (incomplete && cycle_count(cycle))
			incomplete_cycles++;
		cycle = Cycle {};
	}

	/*!
	 * Emulates g6-pen's stale watchdog at the bundler layer: cycles that
	 * could not complete before the 30 ms window expired are dropped.
	 */
	void tick(u64 now_ns)
	{
		const bool active = cycle_count(cycle) > 0;
		if (!active)
			return;

		const u64 anchor = cycle.first_timestamp_ns;
		if (now_ns > anchor && now_ns - anchor > CYCLE_WINDOW_NS)
			clear(true);
	}

	Cycle cycle {};
	u32 generation = 0;
	bool generation_valid = false;
	u32 last_sequence = 0;
	bool sequence_valid = false;
};

inline Vector parse_vector(const u8 *entry)
{
	Vector row {};
	std::memcpy(&row.frequency, entry + 0, 4);
	std::memcpy(&row.magnitude, entry + 4, 4);
	for (usize i = 0; i < 9; i++) {
		std::memcpy(&row.real[i], entry + 8 + i * 2, 2);
		std::memcpy(&row.imag[i], entry + 26 + i * 2, 2);
	}
	row.first = static_cast<i8>(entry[44]);
	row.last = static_cast<i8>(entry[45]);
	row.mid = static_cast<i8>(entry[46]);
	row.zero = static_cast<i8>(entry[47]);

	const u8 center = entry[46];
	row.trailer_valid = center >= 4 && center + 4 < 68 &&
			    entry[44] == center - 4 && entry[45] == center + 4 &&
			    entry[47] == 0;
	return row;
}

/*!
 * Extracts every kind-0x5C nested record of a HEAT part, in order of
 * appearance (the "region" ordinal). Each yields an antenna bank set with a
 * per-record channel code and vector count.
 *
 * @param[in] part The HEAT part to scan.
 * @return The bank sets of all structurally valid 0x5C records.
 */
inline std::vector<BankSet> extract_regions(const Part &part)
{
	std::vector<BankSet> regions {};

	if (!part.present || part.content_len < 17)
		return regions;

	const u8 *data = part.content.data();
	const u32 section_length = read_le32(data + 9);
	if (section_length > part.content_len - 9u - 4u)
		return regions;

	const usize section_end = 9 + 4 + section_length;
	if (read_le16(data + 13) != 0xFF00 || data[15] != 0)
		return regions;

	usize position = 16; // section + 7: first nested record header

	while (position + 4 <= section_end) {
		const u8 kind = data[position];
		const u16 payload_len = read_le16(data + position + 2);
		const usize next = position + 4 + payload_len;
		if (next > section_end)
			break;

		if (kind == 0x5C && payload_len >= 12 && data[position + 1] == 0) {
			const u8 *payload = data + position + 4;
			const u8 count = payload[4];

			if (count > 0 && count <= MAX_VECTORS &&
			    payload[6] == 1 && payload[7] == 1 &&
			    payload[9] <= 15 &&
			    payload_len == 12 + static_cast<u16>(count) * 2 * ROW_SIZE) {
				BankSet banks {};
				banks.region = payload[5];
				banks.channel = payload[9];
				banks.count = count;

				for (usize bank = 0; bank < BANKS; bank++) {
					for (usize vector = 0; vector < count;
					     vector++) {
						const u8 *entry =
							payload + 12 +
							(bank * count + vector) *
								ROW_SIZE;
						const Vector row =
							parse_vector(entry);
						if (bank == 0)
							banks.x.push_back(row);
						else
							banks.y.push_back(row);
					}
				}

				regions.push_back(std::move(banks));
			}
		}

		position = next;
	}

	return regions;
}

/*!
 * The position banks of a cycle: the first 0x5C record of the 0x0C part
 * (region 1, channel 6, 8 vectors per bank), with rows ordered
 * primary-first — valid-trailer vectors before invalid ones, then by
 * descending magnitude — so index 0 is the position transmitter and index 1
 * the tilt transmitter expected by DftStylus.
 *
 * @param[in] cycle The complete cycle.
 * @return The position banks, or nullopt if absent or malformed.
 */
inline std::optional<BankSet> position_banks(const Cycle &cycle)
{
	const Part &part = cycle.parts[static_cast<usize>(PartIndex::Heat)];
	const std::vector<BankSet> regions = extract_regions(part);

	for (BankSet banks : regions) {
		if (banks.region != 1 || banks.channel != 6 || banks.count != 8)
			continue;
		if (banks.x.empty())
			continue;

		const auto primary_first = [](const Vector &a, const Vector &b) {
			if (a.trailer_valid != b.trailer_valid)
				return a.trailer_valid > b.trailer_valid;
			return a.magnitude > b.magnitude;
		};
		std::stable_sort(banks.x.begin(), banks.x.end(), primary_first);
		std::stable_sort(banks.y.begin(), banks.y.end(), primary_first);
		return banks;
	}

	return std::nullopt;
}

/*!
 * The pressure banks of a cycle: region 4 (channel 7) of the first 0x0B
 * part. These antennas resonate in a different frequency band and react to
 * tip force; iptsd's pressure estimator reads the first
 * ipts::protocol::dft::PRESSURE_ROWS rows as frequency bins.
 *
 * @param[in] cycle The complete cycle.
 * @return The pressure banks with rows in frequency-bin order, or nullopt if
 *         absent.
 */
inline std::optional<BankSet> pressure_banks(const Cycle &cycle)
{
	const Part &part =
		cycle.parts[static_cast<usize>(PartIndex::HeatEarly)];
	const std::vector<BankSet> regions = extract_regions(part);

	for (const BankSet &banks : regions) {
		if (banks.region != 4 || banks.channel != 7 ||
		    banks.count < PRESSURE_ROWS)
			continue;

		BankSet ordered = banks;
		const auto by_frequency = [](const Vector &a, const Vector &b) {
			return a.frequency < b.frequency;
		};
		std::stable_sort(ordered.x.begin(), ordered.x.end(),
				 by_frequency);
		std::stable_sort(ordered.y.begin(), ordered.y.end(),
				 by_frequency);
		return ordered;
	}

	return std::nullopt;
}

/*!
 * Two-cycle hysteresis contact detector on the pressure antenna energy.
 * Run once per cycle with the maximum vector magnitude of the pressure
 * banks; the result gates Pressure window emission.
 */
class ContactDetector {
public:
	/*!
	 * Updates the detector with one cycle of pressure antenna energy.
	 *
	 * @param[in] energy The maximum vector magnitude of the pressure
	 *                   banks.
	 * @return Whether the stylus should currently be treated as in
	 *         contact.
	 */
	bool update(u32 energy)
	{
		if (!this->contact) {
			if (energy >= CONTACT_ON_ENERGY) {
				this->run++;
				if (this->run >= CONTACT_DEBOUNCE_FRAMES) {
					this->contact = true;
					this->run = 0;
				}
			} else {
				this->run = 0;
			}
		} else {
			if (energy <= CONTACT_OFF_ENERGY) {
				this->run++;
				if (this->run >= CONTACT_DEBOUNCE_FRAMES) {
					this->contact = false;
					this->run = 0;
				}
			} else {
				this->run = 0;
			}
		}
		return this->contact;
	}

private:
	bool contact = false;
	u8 run = 0;
};

/*!
 * Appends one serialized DFT window report to the frame buffer.
 *
 * @param[in] out The target frame bytes.
 * @param[in] type The DFT window type.
 * @param[in] timestamp_ms The cycle timestamp in milliseconds.
 * @param[in] banks The antenna banks to serialize. Rows beyond
 *                  ipts::protocol::dft::MAX_ROWS are ignored.
 * @param[in] rows The number of rows per bank to serialize.
 * @param[in] zero_magnitude If true, all vector magnitudes are zeroed. The
 *                          DFT solver treats a zero-magnitude position window
 *                          as a stylus lift and ignores zero-magnitude
 *                          pressure windows.
 */
inline void append_window(std::vector<u8> &out,
			  const ipts::protocol::dft::Type type,
			  const u32 timestamp_ms, const BankSet &banks,
			  usize rows, const bool zero_magnitude = false)
{
	rows = std::min(rows, static_cast<usize>(ipts::protocol::dft::MAX_ROWS));

	const usize window_size_offset = out.size();
	out.insert(out.end(), {static_cast<u8>(TYPE_DFT_WINDOW), 0x00, 0x00,
			       0x00});

	const usize payload_offset = out.size();
	out.insert(out.end(), {static_cast<u8>(timestamp_ms & 0xFF),
			       static_cast<u8>((timestamp_ms >> 8) & 0xFF),
			       static_cast<u8>((timestamp_ms >> 16) & 0xFF),
			       static_cast<u8>((timestamp_ms >> 24) & 0xFF),
			       static_cast<u8>(rows), 1, 0x00, 0x00, 0x00,
			       static_cast<u8>(type), 0x00, 0x00});

	for (usize bank = 0; bank < BANKS; bank++) {
		const std::vector<Vector> &vectors =
			bank == 0 ? banks.x : banks.y;
		for (usize i = 0; i < rows; i++) {
			std::array<u8, ROW_SIZE> row {};

			if (i < vectors.size()) {
				const Vector &vector = vectors[i];
				const u32 magnitude =
					zero_magnitude ? 0 : vector.magnitude;
				std::memcpy(row.data() + 0, &vector.frequency,
					    4);
				std::memcpy(row.data() + 4, &magnitude, 4);
				for (usize c = 0; c < 9; c++) {
					std::memcpy(row.data() + 8 + c * 2,
						    &vector.real[c], 2);
					std::memcpy(row.data() + 26 + c * 2,
						    &vector.imag[c], 2);
				}
				row[44] = static_cast<u8>(vector.first);
				row[45] = static_cast<u8>(vector.last);
				row[46] = static_cast<u8>(vector.mid);
				row[47] = static_cast<u8>(vector.zero);
			}

			out.insert(out.end(), row.begin(), row.end());
		}
	}

	const u16 payload_size = static_cast<u16>(out.size() - payload_offset);
	out[window_size_offset + 2] = static_cast<u8>(payload_size & 0xFF);
	out[window_size_offset + 3] =
		static_cast<u8>((payload_size >> 8) & 0xFF);
}

/*!
 * Serializes one cycle into the IPTS HID report framing expected by
 * ipts::Parser: a 3-byte report header, one HID frame of type Reports
 * containing a DftMetadata report, a DftWindow position report with the two
 * position banks and — when the pressure banks are present — a DftWindow
 * pressure report whose magnitudes are gated by the contact detector.
 *
 * @param[in] cycle The complete cycle.
 * @param[in] group_counter The DFT group counter for this cycle.
 * @param[in,out] contact The contact detector state (updated per cycle).
 * @return The framed report bytes, ready to be passed to
 *         Application::on_data(), or nullopt if the cycle has no position
 *         banks.
 */
inline std::optional<std::vector<u8>>
serialize_cycle(const Cycle &cycle, const u32 group_counter,
		ContactDetector &contact)
{
	const auto position = position_banks(cycle);
	if (!position.has_value())
		return std::nullopt;

	const auto pressure = pressure_banks(cycle);

	u32 pressure_energy = 0;
	if (pressure.has_value())
		pressure_energy = pressure->max_energy();
	const bool in_contact = contact.update(pressure_energy);

	const u32 timestamp_ms =
		static_cast<u32>(std::min<u64>(cycle.last_timestamp_ns /
						   1000000,
					       UINT32_MAX));

	std::vector<u8> out {};
	out.reserve(3 + 7 + 2 * (4 + 16) + 780 + 600);

	// ReportHeader: id + timestamp, skipped by Parser::parse().
	out.insert(out.end(), {0x00, 0x00, 0x00});

	// HID frame of type Reports. Size is filled in once the payload is
	// known.
	const usize frame_size_offset = out.size();
	out.insert(out.end(), {0x00, 0x00, 0x00, 0x00, 0x00,
			       static_cast<u8>(FRAME_REPORTS), 0x00});

	// DftMetadata report: binds the group to the window sequence number.
	{
		const u8 metadata[16] = {
			static_cast<u8>(group_counter & 0xFF),
			static_cast<u8>((group_counter >> 8) & 0xFF),
			static_cast<u8>((group_counter >> 16) & 0xFF),
			static_cast<u8>((group_counter >> 24) & 0xFF),
			1,
			static_cast<u8>(TYPE_DFT_POSITION),
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		};
		out.insert(out.end(),
			   {static_cast<u8>(TYPE_DFT_METADATA), 0x00, 0x10,
			    0x00});
		out.insert(out.end(), std::begin(metadata), std::end(metadata));
	}

	// Pressure window first: updating contact before the position window
	// makes the contact state observable in this cycle's emitted stylus
	// sample. Zero magnitudes keep the contact state released while the
	// window stays present.
	if (pressure.has_value()) {
		BankSet gated = *pressure;
		if (!in_contact) {
			for (Vector &v : gated.x)
				v.magnitude = 0;
			for (Vector &v : gated.y)
				v.magnitude = 0;
		}
		append_window(out, TYPE_DFT_PRESSURE, timestamp_ms, gated,
			      PRESSURE_ROWS);
	}

	// Position window.
	append_window(out, TYPE_DFT_POSITION, timestamp_ms, *position,
		      position->count);

	const u32 frame_size = static_cast<u32>(out.size() - frame_size_offset);
	out[frame_size_offset + 0] = static_cast<u8>(frame_size & 0xFF);
	out[frame_size_offset + 1] = static_cast<u8>((frame_size >> 8) & 0xFF);
	out[frame_size_offset + 2] = static_cast<u8>((frame_size >> 16) & 0xFF);
	out[frame_size_offset + 3] = static_cast<u8>((frame_size >> 24) & 0xFF);

	return out;
}

/*!
 * Serializes a synthetic zero-magnitude position window, forcing the DFT
 * solver to lift the stylus. Used when HEAT cycles stop arriving while the
 * stylus is still reported as in proximity.
 *
 * @param[in] timestamp_ms The current timestamp in milliseconds.
 * @return The framed report bytes.
 */
inline std::vector<u8> serialize_lift(const u32 timestamp_ms)
{
	std::vector<u8> out {};
	out.reserve(3 + 7 + 4 + 16 + 4 + 12 + 2 * 8 * ROW_SIZE);

	out.insert(out.end(), {0x00, 0x00, 0x00});

	const usize frame_size_offset = out.size();
	out.insert(out.end(), {0x00, 0x00, 0x00, 0x00, 0x00,
			       static_cast<u8>(FRAME_REPORTS), 0x00});

	const u8 metadata[16] = {0, 0, 0, 0, 1,
				 static_cast<u8>(TYPE_DFT_POSITION),
				 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	out.insert(out.end(), {static_cast<u8>(TYPE_DFT_METADATA), 0x00, 0x10,
			       0x00});
	out.insert(out.end(), std::begin(metadata), std::end(metadata));

	BankSet empty {};
	append_window(out, TYPE_DFT_POSITION, timestamp_ms, empty, 8, true);

	const u32 frame_size = static_cast<u32>(out.size() - frame_size_offset);
	out[frame_size_offset + 0] = static_cast<u8>(frame_size & 0xFF);
	out[frame_size_offset + 1] = static_cast<u8>((frame_size >> 8) & 0xFF);
	out[frame_size_offset + 2] = static_cast<u8>((frame_size >> 16) & 0xFF);
	out[frame_size_offset + 3] = static_cast<u8>((frame_size >> 24) & 0xFF);

	return out;
}

} // namespace iptsd::g6ts

#endif // IPTSD_CORE_GENERIC_G6TS_HPP
