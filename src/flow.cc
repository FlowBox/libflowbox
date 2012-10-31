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
 * @file   flow.cc
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

#include "flow.h"

const uint8_t Flow::kDirectionUnknown = 0;
const uint8_t Flow::kDirectionInIn = 1;
const uint8_t Flow::kDirectionInOut = 2;
const uint8_t Flow::kDirectionOutIn = 3;
const uint8_t Flow::kDirectionOutOut = 4;
const uint8_t Flow::kBorderUnknown = 0;
const uint8_t Flow::kBorderIfIn = 1;
const uint8_t Flow::kBorderIfOut = 2;
const uint8_t Flow::kBorderIfInOut = 3;
const int Flow::kAddressLengthUnknown = 0;
const int Flow::kAddressLengthIPv4 = 4;
const int Flow::kAddressLengthIPv6 = 16;
const std::string Flow::kAddressUnknown("NA");

int Flow::addr_import_s(const char* ip, char* to, int addr_length) {
  if (addr_length == kAddressLengthIPv4) {
    if (inet_pton(AF_INET, ip, to) != 1)
      return (kEAddressIPv4Format);
    memset(to + 4, 0, 12);
  } else if (addr_length == kAddressLengthIPv6) {
    if (inet_pton(AF_INET6, ip, to) != 1)
      return (kEAddressIPv6Format);
  } else {
    return (kEAddressLengthUnknown);
  }
  return (kOK);
}

int Flow::guess_family(const char* ip) {
  if (strchr(ip, ':'))
    return (kAddressLengthIPv6);
  else if (strchr(ip, '.'))
    return (kAddressLengthIPv4);
  else
    return (kAddressLengthUnknown);
}

int Flow::addr_import_ipv4_s(const char* ip, char* to) {
  return (addr_import_s(ip, to, kAddressLengthIPv4));
}

int Flow::addr_import_ipv6_s(const char* ip, char* to) {
  return (addr_import_s(ip, to, kAddressLengthIPv6));
}

int Flow::addr_import_s(const char* ip, char* to) {
  return (addr_import_s(ip, to, guess_family(ip)));
}

void Flow::addr_to_s(std::string& buf, const char* ip_, uint8_t addr_length_) {
  char tmp_ip[INET6_ADDRSTRLEN];
  if (addr_length_ == 4) {
    inet_ntop(AF_INET, ip_, tmp_ip, INET6_ADDRSTRLEN);
  } else if (addr_length_ == 16) {
    inet_ntop(AF_INET6, ip_, tmp_ip, INET6_ADDRSTRLEN);
  } else {
    std::stringstream err_msg;
    err_msg << "Flow: Unknown Address! (" << tmp_ip << ")" << std::endl;
    throw FlowBoxE(err_msg.str(), __FILE__, __LINE__);
  }

  buf = tmp_ip;
  return;
}

//------------------------------------------------------------------------------
// The class methods
//------------------------------------------------------------------------------
Flow::Flow() {
  valid_ = false;
}

void Flow::clear(void) {
  valid_ = false;
}

void Flow::zero(void) {
  memset(addr_src_, 0, kAddressLengthMax);
  memset(addr_dst_, 0, kAddressLengthMax);
  memset(addr_next_, 0, kAddressLengthMax);
  start_s_ = 0;
  stop_s_ = 0;
  export_s_ = 0;
  sys_up_r_ = 0;
  sys_start_r_ = 0;
  sys_stop_r_ = 0;
  packets_ = 0;
  bytes_ = 0;
  valid_ = false;
  addr_length_ = 0;
  protocol_ = 0;
  port_src_ = 0;
  port_dst_ = 0;
  export_device_id_ = 0;
  if_in_ = 0;
  if_out_ = 0;
  direction_ = 0;
  border_ = 0;
}

int Flow::memory_footprint(void) {
  return (sizeof(Flow));
}

std::string Flow::to_s(void) const {
  std::stringstream tmp;
  std::string addr_buff;

  Flow::addr_to_s(addr_buff, addr_src_, addr_length_);
  tmp << addr_buff << ", ";
  Flow::addr_to_s(addr_buff, addr_dst_, addr_length_);
  tmp << addr_buff << ", ";
  Flow::addr_to_s(addr_buff, addr_next_, addr_length_);
  tmp << addr_buff << ", ";
  tmp << start_s_ << ", ";
  tmp << stop_s_ << ", ";
  tmp << export_s_ << ", ";
  tmp << sys_up_r_ << ", ";
  tmp << sys_start_r_ << ", ";
  tmp << sys_stop_r_ << ", ";
  tmp << packets_ << ", ";
  tmp << bytes_ << ", ";
  tmp << valid_ << ", ";
  tmp << static_cast<int>(addr_length_) << ", ";
  tmp << static_cast<int>(protocol_) << ", ";
  tmp << static_cast<int>(port_src_) << ", ";
  tmp << static_cast<int>(port_dst_) << ", ";
  tmp << static_cast<int>(export_device_id_) << ", ";
  tmp << static_cast<int>(if_in_) << ", ";
  tmp << static_cast<int>(if_out_) << ", ";
  tmp << static_cast<int>(direction_) << ", ";
  tmp << static_cast<int>(border_) << ", ";

  return (std::string(tmp.str()));
}
