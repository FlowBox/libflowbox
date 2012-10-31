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
 * @file   flow.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @author Daniel Aschwanden  <nimdanitro@gmail.com>
 * @date   October, 2012
 * @brief  A data structure representing a NetFlow or IPFix record.
 *
 * A 'Flow' data structure contains all required flow information.
 * It's like a abstraction layer for the different flow formats like
 * Netflow V5, Netflow V9, IPFix ...
 *
 */

#ifndef FLOW_BOX_INCLUDE_FLOW_H_
#define FLOW_BOX_INCLUDE_FLOW_H_

#include <stdint.h>
#include <arpa/inet.h>
#include <cstdio>
#include <cstring>
#include <string>

#include "common.h"

class Flow {
//------------------------------------------------------------------------------
// The class constants
//------------------------------------------------------------------------------
 public:
  // DIRECTION: The direction of the Flow
  // Unknown  = Not Set = Unknown
  // Internal = In (Specified by prefix range, e.g. AS559)
  // External = Out (Rest, e.g. Internet)
  static const uint8_t kDirectionUnknown;
  static const uint8_t kDirectionInIn;
  static const uint8_t kDirectionInOut;
  static const uint8_t kDirectionOutIn;
  static const uint8_t kDirectionOutOut;

  // BORDER: The flow crosses a border interface of the Network
  static const uint8_t kBorderUnknown;
  // The input interface is a border if
  static const uint8_t kBorderIfIn;
  // The output interface is a border if
  static const uint8_t kBorderIfOut;
  // The input and output interfaces are a border if
  static const uint8_t kBorderIfInOut;

  // All IP addresses are stored as char arrays using network byte order
  // ADDRESS LENGTH: Since IPv6 will be the future we reserve 16 Bytes
  static const unsigned int kAddressLengthMax = 16;
  // SPECIFIC ADDRESS LENGTHS
  static const int kAddressLengthUnknown;
  static const int kAddressLengthIPv4;
  static const int kAddressLengthIPv6;
  static const std::string kAddressUnknown;

  // ERRORS
  enum Errors {
    kOK = 0,
    kEAddressLengthUnknown = 1,
    kEAddressIPv4Format = 2,
    kEAddressIPv6Format = 3
  };

//------------------------------------------------------------------------------
// The class member variables
//------------------------------------------------------------------------------
// --> ordered by the memory footprint size
 public:
  // NETWORK ADDRESSES: (network byte order)
  // ADDR_SRC: The IP address of the source of this Flow
  char addr_src_[kAddressLengthMax];
  // ADDR_DST: The IP address of the destination of this Flow
  char addr_dst_[kAddressLengthMax];
  // ADDR_NEXT: The IP address of the next hope (e.g. next router)
  char addr_next_[kAddressLengthMax];

  // PACKETS: How many packets were recorded for this Flow
  uint64_t packets_;
  // BYTES: How many bytes were recorded for this Flow
  uint64_t bytes_;

  // TIMING:
  // Absolut timing (we keep only second precision, not very accurate)
  // START_S: The start time of the flow in s
  uint32_t start_s_;
  // STOP_S: The end time of the flow in s
  uint32_t stop_s_;

  // relative timing
  // EXPORT_S: The time when this flow was framed by the exporter
  uint32_t export_s_;
  // SYS_UP_R: Time in milliseconds since this device was first booted
  uint32_t sys_up_r_;
  // FIRST_R: SysUptime at start of flow (ms)
  uint32_t sys_start_r_;
  // FIRST_R: SysUptime at stop of flow (ms)
  uint32_t sys_stop_r_;

  // PORT SRC: The L4 source port of this Flow.
  // This value is only meaningful specific protocols
  // OK: TCP, UDP
  // BAD: rest
  uint16_t port_src_;

  // PORT DST: The L4 destination port of this Flow.
  // see above
  uint16_t port_dst_;

  // EXPORT DEVICE ID: The id of the flow meter that created this flow
  uint16_t export_device_id_;

  // INTERFACE INCOMING: The Interface where the flow entered the router
  uint32_t if_in_;

  // INTERFACE LEAVING: The Interface where the flow left the router
  uint32_t if_out_;

  // DIRECTION: The direction of the Flow
  uint8_t direction_;

  // BORDER: Does the flow cross the network border?
  uint8_t border_;

  // ADDR LENGTH: We process IPv4 and IPv6 traffic. This flag is used to
  // indicate what kind of address is stored inside the char array.
  uint8_t addr_length_;

  // PROTOCOL: The protocol above IPv4/IPv6
  // Examples: ICMP = 1, TCP = 6, UDP = 17, ICMPv6 = 58
  uint8_t protocol_;

  // VALID: Should we interpret this data as a Flow ?
  // True: Yes, this data represents a valid flow
  // False: No, this data is probably incomplete, outdated, or filtered
  bool valid_;
//------------------------------------------------------------------------------
// The class methods
//------------------------------------------------------------------------------
 public:
  Flow();
  void clear(void);
  void zero(void);
  std::string to_s(void) const;

//------------------------------------------------------------------------------
// The class helper functions
//------------------------------------------------------------------------------
 public:
  // use this functions to import address from some other char arrays.
  // IMPORTANT: Unused bits are set to zero
  //    => required to build correct hash keys
  inline static void addr_import_ipv4(const char* from, char* to) {
    memcpy(to, from, 4);
    memset(to + 4, 0, 12);
    return;
  }
  inline static void addr_import_ipv6(const char* from, char* to) {
    memcpy(to, from, 16);
    return;
  }
  static int addr_import_s(const char* ip, char* to, int addr_length);
  static int guess_family(const char* ip);
  static int addr_import_ipv4_s(const char* ip, char* to);
  static int addr_import_ipv6_s(const char* ip, char* to);
  static int addr_import_s(const char* ip, char* to);
  static void addr_to_s(std::string& buf, const char* ip_,
                        uint8_t addr_length_);
  inline static void addr_copy(const char* from, char* to) {
    memcpy(to, from, 16);
    return;
  }
  // return the number of bytes used by a single 'Flow' data structure
  static int memory_footprint(void);
};

#endif  // FLOW_BOX_INCLUDE_FLOW_H_
