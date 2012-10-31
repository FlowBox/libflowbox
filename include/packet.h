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
 * @file   packet.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @date   October, 2012
 * @brief  A network packet that possible contains NetFlow or IPFix data.
 *
 * The Packet data structure represents a NetFlow or IPFix packet that is
 * captured by the host and should be parsed by FlowBox.
 *
 */


#ifndef FLOW_BOX_INCLUDE_PACKET_H_
#define FLOW_BOX_INCLUDE_PACKET_H_

#include <sys/types.h>
#include <sys/socket.h>

// #define CSG_FILE_VALIDATION

struct Packet {
  // MAX PAYLOAD
  static const int kBufferLength = 3000;

  // IP SRC
  struct sockaddr addr_;
  socklen_t addr_length_;

  // TIME OF CAPTURING
  int time_s_;

  // PAYLOAD
  char buffer_[kBufferLength];
  int buffer_length_;

#ifdef CSG_FILE_VALIDATION
  int validation_;  // used only for debugging
#endif
};

#endif  //  FLOW_BOX_INCLUDE_PACKET_H_
