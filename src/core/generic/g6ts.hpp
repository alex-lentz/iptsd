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

constexpr auto FRAME_REPORTS = ipts::protocol::hid::FrameType::Reports;
constexpr auto TYPE_DFT_METADATA = ipts::protocol::report::Type::DftMetadata;
constexpr auto TYPE_DFT_WINDOW = ipts::protocol::report::Type::DftWindow;
constexpr auto TYPE_DFT_POSITION = ipts::protocol::dft::Type::Position;

constexpr usize BANKS = 2;
constexpr usize VECTORS_PER_BANK = 8;
constexpr usize ROW_SIZE = 48;

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

struct CycleBanks {
	std::array<Vector, VECTORS_PER_BANK> x {};
	std::array<Vector, VECTORS_PER_BANK> y {};
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
		const bool hovering = cycle_count(cycle) > 0;
		if (!hovering)
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

/*!
 * Extracts the two antenna banks from the region-1 kind-0x5C record of a
 * complete cycle, following the bounded parser of g6-pen's FF00 decoder.
 *
 * Rows are ordered primary-first: valid-trailer vectors before invalid ones,
 * then by descending magnitude. This puts the position transmitter at index 0
 * and the secondary transmitter at index 1, as expected by DftStylus.
 *
 * @param[in] cycle The complete cycle.
 * @return The two banks, or nullopt if the cycle contains no valid 0x5C
 *         record.
 */
inline std::optional<CycleBanks> extract_banks(const Cycle &cycle)
{
	const Part &part = cycle.parts[static_cast<usize>(PartIndex::Heat)];
	if (!part.present || part.content_len < 17)
		return std::nullopt;

	const u8 *data = part.content.data();
	const u32 section_length = read_le32(data + 9);
	if (section_length > part.content_len - 9u - 4u)
		return std::nullopt;

	const usize section_end = 9 + 4 + section_length;
	if (read_le16(data + 13) != 0xFF00 || data[15] != 0)
		return std::nullopt;

	usize position = 16; // section + 7: first nested record header
	std::optional<CycleBanks> banks;
	bool found[BANKS] = {false, false};

	while (position + 4 <= section_end) {
		const u8 kind = data[position];
		const u16 payload_len = read_le16(data + position + 2);
		const usize next = position + 4 + payload_len;
		if (next > section_end)
			return std::nullopt;

		if (kind == 0x5C && payload_len >= 12 && !banks.has_value()) {
			const u8 *payload = data + position + 4;
			const u8 count = payload[4];

			if (data[position + 1] == 0 && count == VECTORS_PER_BANK &&
			    payload[5] == 1 && payload[6] == 1 && payload[7] == 1 &&
			    payload[9] == 6 && read_le16(payload + 10) == 0xFFFF &&
			    payload_len == 12 + count * 2 * ROW_SIZE) {
				banks = CycleBanks {};

				for (usize bank = 0; bank < BANKS; bank++) {
					std::array<Vector, VECTORS_PER_BANK> rows {};
					for (usize vector = 0; vector < count;
					     vector++) {
						const u8 *entry =
							payload + 12 +
							(bank * count + vector) *
								ROW_SIZE;
						Vector &row = rows[vector];
						std::memcpy(&row.frequency,
							    entry + 0, 4);
						std::memcpy(&row.magnitude,
							    entry + 4, 4);
						for (usize i = 0; i < 9; i++) {
							std::memcpy(&row.real[i],
								    entry + 8 + i * 2,
								    2);
							std::memcpy(&row.imag[i],
								    entry + 26 + i * 2,
								    2);
						}
						row.first = static_cast<i8>(entry[44]);
						row.last = static_cast<i8>(entry[45]);
						row.mid = static_cast<i8>(entry[46]);
						row.zero = static_cast<i8>(entry[47]);

						const u8 center =
							static_cast<u8>(entry[46]);
						row.trailer_valid =
							center >= 4 &&
							center + 4 < 68 &&
							entry[44] == center - 4 &&
							entry[45] == center + 4 &&
							entry[47] == 0;
					}

					// Primary transmitter first: valid
					// trailers, then descending magnitude.
					std::stable_sort(rows.begin(), rows.end(),
							 [](const Vector &a,
							    const Vector &b) {
								 if (a.trailer_valid !=
								     b.trailer_valid)
									 return a.trailer_valid >
									        b.trailer_valid;
								 return a.magnitude >
								        b.magnitude;
							 });

					if (bank == 0)
						banks->x = rows;
					else
						banks->y = rows;
					found[bank] = true;
				}
			}
		}

		position = next;
	}

	if (!found[0] || !found[1])
		return std::nullopt;
	return banks;
}

/*!
 * Serializes one cycle into the IPTS HID report framing expected by
 * ipts::Parser: a 3-byte report header, one HID frame of type Reports
 * containing a DftMetadata report followed by a DftWindow report with the two
 * antenna banks.
 *
 * @param[in] banks The antenna banks of the cycle.
 * @param[in] timestamp_ms The cycle timestamp in milliseconds.
 * @param[in] group_counter The group counter for this cycle.
 * @param[in] zero_magnitude If true, all vector magnitudes are zeroed. The
 *                          DFT solver treats this as a stylus lift.
 * @return The framed report bytes, ready to be passed to
 *         Application::on_data().
 */
inline std::vector<u8> serialize_dft(const CycleBanks &banks, u32 timestamp_ms,
				     u32 group_counter, bool zero_magnitude = false)
{
	constexpr u8 seq = 1;

	std::vector<u8> out {};
	out.reserve(3 + 7 + 4 + 16 + 4 + 12 + 2 * VECTORS_PER_BANK * ROW_SIZE);

	// ReportHeader: id + timestamp, skipped by Parser::parse().
	out.insert(out.end(), {0x00, 0x00, 0x00});

	// HID frame of type Reports. Size is filled in once the payload is
	// known.
	const usize frame_size_offset = out.size();
	out.insert(out.end(), {0x00, 0x00, 0x00, 0x00, 0x00,
			       static_cast<u8>(FRAME_REPORTS), 0x00});

	// DftMetadata report.
	{
		const u8 metadata[16] = {
			static_cast<u8>(group_counter & 0xFF),
			static_cast<u8>((group_counter >> 8) & 0xFF),
			static_cast<u8>((group_counter >> 16) & 0xFF),
			static_cast<u8>((group_counter >> 24) & 0xFF),
			seq,
			static_cast<u8>(TYPE_DFT_POSITION),
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		};
		out.insert(out.end(),
			   {static_cast<u8>(TYPE_DFT_METADATA), 0x00,
			    0x10, 0x00});
		out.insert(out.end(), std::begin(metadata), std::end(metadata));
	}

	// DftWindow report.
	{
		const usize window_size_offset = out.size();
		out.insert(out.end(), {static_cast<u8>(TYPE_DFT_WINDOW),
				       0x00, 0x00, 0x00});

		const usize payload_offset = out.size();
		out.insert(out.end(), {static_cast<u8>(timestamp_ms & 0xFF),
				       static_cast<u8>((timestamp_ms >> 8) & 0xFF),
				       static_cast<u8>((timestamp_ms >> 16) & 0xFF),
				       static_cast<u8>((timestamp_ms >> 24) & 0xFF),
				       VECTORS_PER_BANK,
				       seq,
				       0x00, 0x00, 0x00,
				       static_cast<u8>(TYPE_DFT_POSITION),
				       0x00, 0x00});

		for (usize bank = 0; bank < BANKS; bank++) {
			const auto &rows =
				bank == 0 ? banks.x : banks.y;
			for (const Vector &vector : rows) {
				const u32 magnitude =
					zero_magnitude ? 0 : vector.magnitude;

				std::array<u8, ROW_SIZE> row {};
				std::memcpy(row.data() + 0, &vector.frequency, 4);
				std::memcpy(row.data() + 4, &magnitude, 4);
				for (usize i = 0; i < 9; i++) {
					std::memcpy(row.data() + 8 + i * 2,
						    &vector.real[i], 2);
					std::memcpy(row.data() + 26 + i * 2,
						    &vector.imag[i], 2);
				}
				row[44] = static_cast<u8>(vector.first);
				row[45] = static_cast<u8>(vector.last);
				row[46] = static_cast<u8>(vector.mid);
				row[47] = static_cast<u8>(vector.zero);

				out.insert(out.end(), row.begin(), row.end());
			}
		}

		const u16 payload_size =
			static_cast<u16>(out.size() - payload_offset);
		out[window_size_offset + 2] =
			static_cast<u8>(payload_size & 0xFF);
		out[window_size_offset + 3] =
			static_cast<u8>((payload_size >> 8) & 0xFF);
	}

	const u32 frame_size = static_cast<u32>(out.size() - frame_size_offset);
	out[frame_size_offset + 0] = static_cast<u8>(frame_size & 0xFF);
	out[frame_size_offset + 1] = static_cast<u8>((frame_size >> 8) & 0xFF);
	out[frame_size_offset + 2] = static_cast<u8>((frame_size >> 16) & 0xFF);
	out[frame_size_offset + 3] = static_cast<u8>((frame_size >> 24) & 0xFF);

	return out;
}

} // namespace iptsd::g6ts

#endif // IPTSD_CORE_GENERIC_G6TS_HPP
