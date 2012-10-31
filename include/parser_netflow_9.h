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
 * @file   parser_netflow_9.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 *
 * @date   October, 2012
 * @brief  Extract NetFlow Version 9 data from a Packet.
 *
 * The ParserNetflow9 is a simple NetFlow parser. It analyzes the NetFlow
 * information contained by the Packet object and stores the information
 * in our internal Flow data structure.
 *
 */

#ifndef FLOW_BOX_INCLUDE_PARSER_NETFLOW_9_H_
#define FLOW_BOX_INCLUDE_PARSER_NETFLOW_9_H_

#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <algorithm>

#include "common.h"
#include "packet.h"
#include "flow.h"
#include "flow_container.h"

class ParserNetflow9 {
  //----------------------------------------------------------------------------
  // TEMPLATE OBJECT
  //----------------------------------------------------------------------------
  struct TemplateToFlow {
    //--------------------------------------------------------------------------
   public:
    static const int kRawLengthMax = 1024;
    static const int kOffsetInvalid = -1;
    static const int kFieldLength16 = 2;
    static const int kFieldLength32 = 4;
    static const int kFieldLength64 = 8;

    //--------------------------------------------------------------------------
    int template_id_;  ///< template ID
    int length_;  ///< length of this template
    bool valid_;  ///< everything included to build a valid Flow?

    char raw_[kRawLengthMax];  ///< holds raw template data
    int raw_length_;  ///< length of the raw data

    // MAPPING
    // addr
    int addr_length_;  ///< the length of the IP address
    int addr_src_o_;  ///< the source IP
    int addr_dst_o_;  ///< the destination IP

    // protocol
    int protocol_o_;  ///< protocol version

    // ports
    int port_src_o_;  ///< source port
    int port_dst_o_;  ///< destination port

    // timing
    int first_o_;  ///< time of the first packet
    int last_o_;   ///< time of the last packet

    // router
    int addr_next_o_;  ///< the next hop address

    // interfaces
    int if_in_o_;  ///< in interface
    int if_in_l_;
    int if_out_o_;  ///< out interface
    int if_out_l_;

    // content
    int packets_o_;  ///< packets
    int packets_l_;

    int bytes_o_;  ///</ bytes
    int bytes_l_;

    //--------------------------------------------------------------------------
    TemplateToFlow();
    void reset(void);
    void reset_data(void);
    bool validate(void);
    std::string to_s(void) const;
  };

//------------------------------------------------------------------------------
// The class constants
//------------------------------------------------------------------------------
 public:
  // HEADER
  /// Lengths in bytes of a single header
  static const int kHeaderLength = 20;
  /// The version of NetFlow records exported in this packet;
  static const int kHeaderVersion = 0;
  /// Number of FlowSet records contained within this packet (normally < 30)
  static const int kHeaderCount = 2;
  /// Time in milliseconds since this device was first booted
  static const int kHeaderSystemUp = 4;
  /// Seconds since 0000 Coordinated Universal Time (UTC) 1970 (when the packet was sent)
  static const int kHeaderUnixUp = 8;
  static const int kHeaderPackageSequence = 12;
  static const int kHeaderSourceId = 16;

  // TEMPLATE
  static const int kTemplateFlowsetId = 0;
  static const int kTemplateflowsetIdMax = 255;

  // FIELD TYPE DEFINITIONS
  static const int kFieldInBytes = 1;        // N (4)
  static const int kFieldInPackets = 2;      // N (4)
  static const int kFieldProtocol = 4;       // 1
  static const int kFieldL4SrcPort = 7;      // 2
  static const int kFieldIPv4SrcAddr = 8;    // 4
  static const int kFieldInputSNMP = 10;     // N (2)
  static const int kFieldL4DstPort = 11;     // 2
  static const int kFieldIPv4DstAddr = 12;   // 4
  static const int kFieldOutputSNMP = 14;    // N (2)
  static const int kFieldIPv4NextHop = 15;   // 4
  static const int kFieldLastSwitched = 21;  // 4
  static const int kFieldFirstSwitched = 22;  // 4
  static const int kFieldIPv6SrcAddr = 27;   // 16
  static const int kFieldIPv6DstAddr = 28;   // 16
  static const int kFieldIPv6NextHop = 62;   // 16

  // OPTION TEMPLATE
  static const int kOptionsFlowsetId = 1;

  // FORMAT
  static const int kFormatMinLength = 20;
  static const int kFormatVersion = 2304;  ///< 9 << 8 = 2304

  // TIMING
  static const int kSysupTimestampMaxDelayInMs = 2000;

//------------------------------------------------------------------------------
// The class member variables
//------------------------------------------------------------------------------
 private:
  // PARSER
  int export_device_id_;  ///< identifies the export_device_id_

  // LOSS ESTIMATION
  int64_t loss_packet_sequence_last_;
  int64_t loss_packet_sequence_lost_;

  // TEMPLATES
  // <template_id, template>
  std::map<int, TemplateToFlow*> template_map_;

  // STATISTICS
  uint64_t stat_packets_;  ///< how many packets are parsed
  uint64_t stat_packets_error_;  ///< how many packets are ignored
  uint64_t stat_packets_no_template_;  ///< ignored: no template found
  uint64_t stat_packets_no_valid_template_;  ///< ignored: no valid template found
  uint64_t stat_records_;  ///< flow records

//------------------------------------------------------------------------------
// The class helper functions
//------------------------------------------------------------------------------
 private:
  // Template
  TemplateToFlow* create_template(const int template_id);
  TemplateToFlow* get_template(const int template_id);
  void parse_template(const char* flowset_p, const int flowset_length);

  // Flows
  void parse_data(const char* flowset_p, const int flowset_length,
                  const uint64_t export_time, const uint64_t boot_time,
                  const TemplateToFlow* template_p,
                  FlowContainer* flow_container);
  int stat_template_count();
  uint64_t stat_flow_lost();
  int64_t lost();

//------------------------------------------------------------------------------
// API
//------------------------------------------------------------------------------
 public:
  static const uint16_t get_version(const Packet& packet);
  static const uint32_t get_engine_id(const Packet& packet);
  static const uint32_t get_export_time_s(const Packet& packet);
  static const uint16_t get_estimated_flow_count(const Packet& packet);

  explicit ParserNetflow9(int export_device_id);
  ~ParserNetflow9();
  void parse(const Packet& packet, FlowContainer* flow_container);
  void reset(void);
  // STATISTICS
  void statistics_reset();
  std::map<std::string, uint64_t> statatistics_get(void);
};

#endif  // FLOW_BOX_INCLUDE_PARSER_NETFLOW_9_H_
