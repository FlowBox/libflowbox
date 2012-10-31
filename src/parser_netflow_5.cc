// Copyright (C) 2011-12 Dominik Schatzmann <schadomi@gmail.com>
// This file is part of FlowBox. FlowBox is free software: you can redistribute
// it and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version 3
// of the License, or (at your option) any later version.
//
// FlowBox is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with FlowBox. If not, see <http://www.gnu.org/licenses/>.

/**
 * @file   parser_netflow_5.cc
 * @author Dominik Schatzmann <schadomi@gmail.com>
 *
 * @date   October, 2012
 * @brief  Extract NetFlow Version 5 data from a Packet.
 *
 * The ParserNetflow5 is a simple NetFlow parser. It  analyzes the NetFlow
 * information contained by the Packet object and stores the information
 * in our internal Flow data structure.
 *
 */

#include "parser_netflow_5.h"
//------------------------------------------------------------------------------
ParserNetflow5::ParserNetflow5(int export_device_id)
    : export_device_id_(export_device_id),
      loss_packet_sequence_last_(0),
      loss_packet_sequence_lost_(0) {
  statistics_reset();
  assert(export_device_id_ != 0);
}

void ParserNetflow5::parse(const Packet& packet,
                           FlowContainer* flow_container) {
  stat_packets_++;

  // HEADER --------------------------------------------------------------------
  // kHeaderVersion: Already checked, its Netflow 5
  // HeaderCount: already checked, container has enough space
  uint16_t count =
      ntohs(*(reinterpret_cast<const uint16_t*>(packet.buffer_ + kHeaderCount)));
  // kHeaderSystemUp: milliseconds since router was booted
  uint32_t system_up_ms =
      ntohl(*(reinterpret_cast<const uint32_t*>(packet.buffer_ + kHeaderSystemUp)));
  // kHeaderUnixUp: Seconds since 0000 Coordinated Universal Time (UTC) 1970
  uint32_t unix_sec =
      ntohl(*(reinterpret_cast<const uint32_t*>(packet.buffer_ + kHeaderUnix_s)));
  // kHeaderPackageSequence: Sequence number of total flows seen
  // uint32_t sequence_nr =
  // ntohl(*(reinterpret_cast<uint32_t*>(packet.buffer_ + kHeaderSequenceNbr)));
  // kHeaderSourceId: This number is already parsed and stored in source_id_;

  // IV: Packet length is big enough?
  if ((packet.buffer_length_ - kHeaderLength) > kRecordLength * count) {
    // FIXME (asdaniel): throw error!
    std::cout << "IV: illegal packet length" << std::endl;
    return;
  }

#ifdef PARSER_NETFLOW_5_VVERBOSE
  int version = get_version(packet);
  std::cout << "-- HEADER -- "<< std::endl;
  std::cout << "Version:   " << version << std::endl;
  std::cout << "Count:     " << count << std::endl;
  std::cout << "Unix Sec:  " << unix_sec << std::endl;
  std::cout << "System UP: " << system_up_ms << std::endl;
  std::cout << "Boot Time: " << boot_time << std::endl;
#endif

  // TODO(schadomi): Loss estimation. sequence_n contains the total number of
  //                 flows exported since reboot. NOT a header sequence number.
  // RECORD --------------------------------------------------------------------
  FlowContainer::iterator flow(flow_container->end_used());
  FlowContainer::iterator end(flow_container->end());

  const char* records_p = packet.buffer_ + kHeaderLength;
  for (int i = 0; i < count; i++) {
    // capacity left?
    if (flow == end) {
      throw FlowBoxE("No capacity left -- should never happen",
                     __FILE__, __LINE__);
    }

    // current record
    const char* record_p = records_p + i * kRecordLength;

    // parse the record
    flow->addr_length_ = Flow::kAddressLengthIPv4;
    Flow::addr_import_ipv4(record_p + kRecordAddrSrc, flow->addr_src_);
    Flow::addr_import_ipv4(record_p + kRecordAddrDst, flow->addr_dst_);
    Flow::addr_import_ipv4(record_p + kRecordAddrNext, flow->addr_next_);

    // get protocol specific information
    // get the protocol version
    flow->protocol_ = *(reinterpret_cast<const uint8_t *>(record_p
        + kRecordProtocol));
    // get the port src
    flow->port_src_ =
        ntohs(*(reinterpret_cast<const uint16_t *>(record_p+kRecordPortSrc)));
    // get the port dst
    flow->port_dst_ =
        ntohs(*(reinterpret_cast<const uint16_t *>(record_p+kRecordPortDst)));

    if (!(flow->protocol_ == 6 or flow->protocol_ == 17)) {
      flow->port_src_ = 0;
      flow->port_dst_ = 0;
    }

    // relative timing information
    // FIXME(schadomi): long long is not standard strict c++ int64..
    // fix it!
    long long first =
        ntohl(*(reinterpret_cast<const uint32_t *>(record_p+kRecordFirst)));
    long long last =
        ntohl(*(reinterpret_cast<const uint32_t *>(record_p+kRecordLast)));

    flow->sys_start_r_ = first;
    flow->sys_stop_r_ = last;
    flow->sys_up_r_ = system_up_ms;  // sys_up_r_ miliseconds since router reboot

    // absolute timing
    flow->export_s_ = unix_sec;  // export_time_s unix seconds of now

    // --- absolute timing
    uint32_t start, end;
    long long start2, end2;
    start = system_up_ms - first;
    end = system_up_ms - last;

    start2 = start;  /// get more than 32 bit and negative range
    end2 = end;

    if (start2 >= 0x80000000LL) {
      /// Wrap-around: Do correction
      start2 = start2 - 0x100000000LL;
    }
    if (end2 >= 0x80000000LL) {
      /// Wrap-around - do correction
      end2 = end2 - 0x100000000LL;
    }
    // Plausibility checks: We check whether the sysup time
    // of the exporter is newer (works also in case of
    // any wrap-arounds if no data gap larger than
    // 2^31 ms exists!) than the flow start/stop sysup
    // times. Including an Error constant to account for
    // possible delays when writing the header sysuptime and
    // the flow sysuptimes (this can happen but is rarely seen!)
    if ((int64_t) start2 + kSysupTimestampMaxDelayInMs <= 0) {
      std::stringstream err_msg;
      err_msg
          << "Netflow_V5_Parser::parse_data: sysuptime of flow start"
          << std::endl
          << " is newer than the sysuptime in the packet header by more than "
          << kSysupTimestampMaxDelayInMs << " ms " << std::endl
          << " (Note: this error is NOT caused by a wrap-around!)"
          << flow->to_s() << std::endl;
      throw FlowBoxE(err_msg.str(), __FILE__, __LINE__);
    }

    if ((int64_t) end2 + kSysupTimestampMaxDelayInMs <= 0) {
      std::stringstream err_msg;
      err_msg
          << "Netflow_V5_Parser::parse_data: sysuptime of flow stop"
          << std::endl
          << " is newer than the sysuptime in the packet header by more than "
          << kSysupTimestampMaxDelayInMs << " ms " << std::endl
          << " (Note: this error is NOT caused by a wrap-around!)"
          << flow->to_s() << std::endl;
      throw FlowBoxE(err_msg.str(), __FILE__, __LINE__);
    }

    flow->start_s_ = unix_sec - start2 / 1000;
    flow->stop_s_ = unix_sec - end2 / 1000;

    // router information
    flow->export_device_id_ = export_device_id_;
    // get the in interface
    flow->if_in_ =
        ntohs(*(reinterpret_cast<const uint16_t *>(record_p+kRecordSnmpInput)));
    // get the out interface
    flow->if_out_ =
        ntohs(*(reinterpret_cast<const uint16_t *>(record_p+kRecordSnmpOutput)));

    // network information
    flow->direction_ = Flow::kDirectionUnknown;
    flow->border_ = Flow::kBorderUnknown;

    // parse content information
    // packet amount
    flow->packets_ =
        ntohl(*(reinterpret_cast<const uint32_t *>(record_p+kRecordPackets)));
    // packet bytes
    flow->bytes_ =
        ntohl(*(reinterpret_cast<const uint32_t *>(record_p+kRecordBytes)));

    // mark it as valid
    flow->valid_ = true;

    // next
    flow++;
  }
  stat_records_ += count;
  flow_container->update_used_by(count);
}

void ParserNetflow5::reset() {
  loss_packet_sequence_last_ = 0;
  statistics_reset();
}

// STATISTICS ------------------------------------------------------------------
std::map<std::string, uint64_t> ParserNetflow5::statistics_get(void) {
  std::map<std::string, uint64_t> exporter_stat;
  exporter_stat.insert(
      std::pair<std::string, uint64_t>("export_id", export_device_id_));
  exporter_stat.insert(
      std::pair<std::string, uint64_t>("packets", stat_packets_));
  exporter_stat.insert(
      std::pair<std::string, uint64_t>("packets_error", stat_packets_error_));
  exporter_stat.insert(
      std::pair<std::string, uint64_t>("flows", stat_records_));
  return (std::map<std::string, uint64_t>(exporter_stat));
}

void ParserNetflow5::statistics_reset() {
  stat_packets_ = 0;
  stat_records_ = 0;
  loss_packet_sequence_lost_ = 0;
}

//------------------------------------------------------------------------------
// The class helper functions (STATIC)
//------------------------------------------------------------------------------
const uint16_t ParserNetflow5::get_version(const Packet& packet) {
  return (packet.buffer_[1]);
}

const uint16_t ParserNetflow5::get_engine_id(const Packet& packet) {
  return (ntohs( *(reinterpret_cast<const uint16_t*>(packet.buffer_ + kHeaderEngineType))));
}

const uint32_t ParserNetflow5::get_export_time_s(const Packet& packet) {
  return (
      ntohl( *(reinterpret_cast<const uint32_t*>(packet.buffer_ + kHeaderUnix_s))));
}

const uint16_t ParserNetflow5::get_estimated_flow_count(const Packet& packet) {
  return (ntohs( *(reinterpret_cast<const uint32_t*>(packet.buffer_ + kHeaderCount))));
}
