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
 * @file   packet_source_csg_file.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 *
 * @date   October, 2012
 * @brief  Process incoming CSG NetFlow format files
 *
 * The PacketSourceCSGFile rebuilds packet level input data from
 * the CSG NetFlow files.
 *
 */


#ifndef FLOW_BOX_INCLUDE_PACKET_SOURCE_CSG_FILE_H_
#define FLOW_BOX_INCLUDE_PACKET_SOURCE_CSG_FILE_H_

#include <string.h>
#include <bzlib.h>
#include <arpa/inet.h>

#include <vector>
#include <string>
#include <iostream>
#include <cassert>

#include "common.h"
#include "packet.h"
#include "csg_file.h"

class PacketSourceCSGFile {
//------------------------------------------------------------------------------
// The class constants
//------------------------------------------------------------------------------
 public:
  // ERROR IDS
  static const int kFileSetOK = 0;
  static const int kFileSetRetry = -1;
  static const int kFileSetEOF = -2;
  static const int kFileSetInvalid = -3;

  // STREAM SWITCH DEFAULT
  static const int kStreamSwitchStepSizeDefault = 5;

  enum StreamState {
    kStreamOK = 0,
    kStreamEOF = 1,
    kStreamFailed = 2
  };

  enum LastPacket {
    kLastPacketInit = 0,
    kLastPacketNull = 1,
    kLastPacket19991 = 2,
    kLastPacket19993 = 3
  };

//------------------------------------------------------------------------------
// The class member variables
//------------------------------------------------------------------------------
 private:
  int error_;
  CSGFile stream_19991_;
  CSGFile stream_19993_;
  int last_packet_;

//------------------------------------------------------------------------------
// The class methods
//------------------------------------------------------------------------------
 public:
  PacketSourceCSGFile();
  int set_file_set(const std::string& file_19991_dat,
                   const std::string& file_19991_stat,
                   const std::string& file_19993_dat,
                   const std::string& file_19993_stat);
  int get(Packet** packet);
};

#endif  // FLOW_BOX_INCLUDE_PACKET_SOURCE_CSG_FILE_H_
