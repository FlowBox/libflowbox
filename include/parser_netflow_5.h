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
 * @file   parser_netflow_5.h
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

#ifndef FLOW_BOX_INCLUDE_PARSER_NETFLOW_5_H_
#define FLOW_BOX_INCLUDE_PARSER_NETFLOW_5_H_

// #define PARSER_NETFLOW_5_VVERBOSE

#include <iostream>
#include <string>
#include <map>

#include "common.h"
#include "packet.h"
#include "flow.h"
#include "flow_container.h"

class ParserNetflow5 {
//------------------------------------------------------------------------------
// The class constants
//------------------------------------------------------------------------------
 public:
  // http://www.cisco.com/en/US/docs/ios/solutions_docs/netflow/nfwhite.html#wp1032612
  // HEADER
  /// Lengths in bytes of a single header
  static const int kHeaderLength = 24;
  /// The version of NetFlow records exported in this packet; for Version 5, this value is 0x0005
  static const int kHeaderVersion = 0;
  /// Number of FlowSet records contained within this packet (normally < 30)
  static const int kHeaderCount = 2;
  /// Time in milliseconds since this device was first booted
  static const int kHeaderSystemUp = 4;
  /// Seconds since 0000 Coordinated Universal Time (UTC) 1970 (when the UDP packet was sent)
  static const int kHeaderUnix_s = 8;
  /// Residual nanoseconds since 0000 UTC 1970 (when the UDP packet was sent)
  static const int kHeaderUnix_ns = 12;
  /// Sequence number of total flows seen
  static const int kHeaderSequenceNbr = 16;
  /// Type of flow switching engine, 0 for RP, 1 for VIP/LC
  static const int kHeaderEngineType = 20;
  /// VIP or LC slot number of the flow switching engine
  static const int kHeaderEngineId = 21;
  /// VIP or LC slot number of the flow switching engine
  static const int kHeaderReserved = 22;

  // RECORD
  /// Lengths in bytes of a single record
  static const int kRecordLength = 48;
  /// Source IP address
  static const int kRecordAddrSrc = 0;
  /// Destination IP address
  static const int kRecordAddrDst = 4;
  /// Next hop router's IP address
  static const int kRecordAddrNext = 8;
  /// Ingress interface SNMP ifIndex
  static const int kRecordSnmpInput = 12;
  /// Egress interface SNMP ifIndex
  static const int kRecordSnmpOutput = 14;
  /// Packets in the flow
  static const int kRecordPackets = 16;
  /// Octets (bytes) in the flow
  static const int kRecordBytes = 20;
  /// SysUptime at start of the flow (milliseconds)
  static const int kRecordFirst = 24;
  /// SysUptime at the time the last packet of the flow was received (milliseconds)
  static const int kRecordLast = 28;
  /// Layer 4 source port number or equivalent
  static const int kRecordPortSrc = 32;
  /// Layer 4 destination port number or equivalent
  static const int kRecordPortDst = 34;
  /// Unused (zero) byte
  static const int kRecordPad1 = 36;
  /// Cumulative OR of TCP flags
  static const int kRecordTCPFlags = 37;
  /// Layer 4 protocol (for example, 6=TCP, 17=UDP)
  static const int kRecordProtocol = 38;
  /// IP type-of-service byte
  static const int kRecordTOS = 39;
  /// Autonomous system number of the source, either origin or peer
  static const int kRecordASrc = 40;
  /// Autonomous system number of the destination, either origin or peer
  static const int kRecordASDst = 42;
  /// Source address prefix mask bits
  static const int kRecordMaskSrc = 44;
  /// Destination address prefix mask bits
  static const int kRecordMaskDst = 46;
  /// Pad 2 is unused (zero) bytes
  static const int kRecordPad2 = 48;

  // ERRORS
  static const int kErrorNo = 0;
  /// received buffer is smaller than a v5 header
  static const int kErrorHeaderMinLengh = 1;
  /// packet lengths != header + count*records
  static const int KErrorPacketLength = 2;
  /// the connection container was full;
  static const int KErrorConnectionsOverfull = 3;

  // FORMAT
  static const int kFormatMinLength = kHeaderLength;
  static const int kFormatVersion = 1280;  ///< 5 << 8 = 1280

  // TIMING
  static const int kSysupTimestampMaxDelayInMs = 30000;

//------------------------------------------------------------------------------
// The class member variables
//------------------------------------------------------------------------------
 private:
  // PARSER
  int export_device_id_;  ///< identifies the export_device_id_

  // LOSS ESTIMATION
  int64_t loss_packet_sequence_last_;
  int64_t loss_packet_sequence_lost_;

  // STATISTICS
  uint64_t stat_packets_;  ///< how many packets are parsed
  uint64_t stat_packets_error_;  ///< how many packets are ignored
  uint64_t stat_records_;  ///< flow records

//------------------------------------------------------------------------------
// The class helper functions
//------------------------------------------------------------------------------
 public:
  static const uint16_t get_version(const Packet& packet);
  static const uint16_t get_engine_id(const Packet& packet);
  static const uint32_t get_export_time_s(const Packet& packet);
  static const uint16_t get_estimated_flow_count(const Packet& packet);

//------------------------------------------------------------------------------
// The class methods
//------------------------------------------------------------------------------
 public:
  explicit ParserNetflow5(int export_device_id);
  void parse(const Packet& packet, FlowContainer* flow_container);
  void reset(void);
  std::map<std::string, uint64_t> statistics_get(void);
  void statistics_reset(void);
};

#endif  // FLOW_BOX_INCLUDE_PARSER_NETFLOW_5_H_
