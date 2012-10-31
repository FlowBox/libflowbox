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
 * @file   packet_source_csg_file.cc
 * @author Dominik Schatzmann <schadomi@gmail.com>
 *
 * @date   October, 2012
 * @brief  Process incoming CSG NetFlow format files
 *
 * The PacketSourceCSGFile rebuilds packet level input data from
 * the CSG NetFlow files.
 *
 */

#include "packet_source_csg_file.h"

//------------------------------------------------------------------------------
// PacketSourceCSGFile
//------------------------------------------------------------------------------

PacketSourceCSGFile::PacketSourceCSGFile()
    : error_(kFileSetInvalid),
      last_packet_(kLastPacketInit) {
}

int PacketSourceCSGFile::set_file_set(const std::string& file_19991_dat,
                                      const std::string& file_19991_stat,
                                      const std::string& file_19993_dat,
                                      const std::string& file_19993_stat) {
  last_packet_ = kLastPacketInit;

  int error_19991 = stream_19991_.process_file(file_19991_dat, file_19991_stat);
  int error_19993 = stream_19993_.process_file(file_19993_dat, file_19993_stat);

  if (error_19991 == kStreamOK and error_19993 == kStreamOK) {
    error_ = kFileSetOK;
  } else {
    error_ = kFileSetInvalid;
  }
  return (error_);
}

int PacketSourceCSGFile::get(Packet** packet) {
  // move packet point to the next packet
  if (last_packet_ == kLastPacket19991) {
    stream_19991_.next();
  } else if (last_packet_ == kLastPacket19993) {
    stream_19993_.next();
  } else if (last_packet_ == kLastPacketInit) {
    // init --> move both
    stream_19991_.next();
    stream_19993_.next();
  } else if (last_packet_ == kLastPacketNull) {
    // nothing to do
  } else {
    throw FlowBoxE("broken application logic", __FILE__, __LINE__);
  }

  // which stream should be used?
  int error_19991 = stream_19991_.get_stream_state();
  int error_19993 = stream_19993_.get_stream_state();

  if (error_19991 == CSGFile::kStreamOK && error_19993 == CSGFile::kStreamOK) {
    // both streams are ready -- select the older packet
    if (stream_19991_.get_time_r() <= stream_19993_.get_time_r()) {
      (*packet) = stream_19991_.get_packet();
      last_packet_ = kLastPacket19991;
    } else {
      (*packet) = stream_19993_.get_packet();
      last_packet_ = kLastPacket19993;
    }
    return (kFileSetOK);
  } else if (error_19991 == CSGFile::kStreamOK
      && error_19993 == CSGFile::kStreamEOF) {
    (*packet) = stream_19991_.get_packet();
    last_packet_ = kLastPacket19991;
    return (kFileSetOK);
  } else if (error_19991 == CSGFile::kStreamEOF
      && error_19993 == CSGFile::kStreamOK) {
    (*packet) = stream_19993_.get_packet();
    last_packet_ = kLastPacket19993;
    return (kFileSetOK);
  } else if (error_19991 == CSGFile::kStreamEOF
      && error_19993 == CSGFile::kStreamEOF) {
    last_packet_ = kLastPacketNull;
    return (kFileSetEOF);
  } else {
    // sloppy error handling -- at least one stream is in a error state
    // DEBUG: Haven't seen this error yet -- so stop processing
    //  ... to let me analyze this case
    throw FlowBoxE("DEBUG", __FILE__, __LINE__);
    return (kFileSetInvalid);
  }
}
