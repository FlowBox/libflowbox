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
 * @file   csg_file.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @date   October, 2012
 * @brief  Storage structure used by the CSG Group to store NetFlow on disk
 *
 * Defines the storage structure used by the CSG group of ETH to store NetFlow
 */

#ifndef FLOW_BOX_INCLUDE_CSG_FILE_H_
#define FLOW_BOX_INCLUDE_CSG_FILE_H_

#include <bzlib.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <cstring>
#include <cassert>
#include <cstdlib>
#include <exception>
#include <vector>
#include <list>
#include <string>
#include <sstream>
#include <iostream>

#include "common.h"
#include "packet.h"

// enable this to produce a lot of debug output
// #define CSG_FILE_VERBOSEN
class CSGFile {
  // Use a classical producer consumer schema
  // Producer:
  //  - decompression of bz2 data
  //  - parse meta data from stat file
  //  - rebuild packets
  // Consumer:
  // - serves packets
  // Shared memory:
  // - slotted ring buffer
  // - a slot contains multiple packets
  // - each slot is protected by a mutex
  //
  //----------------------------------------------------------------------------
  // Slotted Ring Buffer of Packets
  //----------------------------------------------------------------------------
  //
  // slot_packets_: stores the packets ( X = slot_packets[slot_idx][packet_idx])
  //             slot 0     slot 1     slot 3     slot 4     slot 5
  // packet 0 |    o     |    o     |    X     |    X     |    o     |
  // packet 1 |    o     |    o     |    X     |    X     |    o     |
  // packet 2 |    o     |   [X]    |    X     |   (X)    |    o     |
  // packet 3 |    o     |    X     |    X     |    o     |    o     |
  // packet 4 |    o     |    X     |    X     |    o     |    o     |
  //
  //
  // slot_state_: what the state of the slot
  //           kSlotEmpty            kSlotFilled            kSlotEmpty
  //                     kSlotConsuming         kSlotFilling
  //
  // slot_packets_used:
  // how many packets are filled inside this slot by the producer
  // packet   |    -     |   4      |    2     |    -     |    0     |
  //
  // o empty
  // X packet
  // [] consumer
  // () producer
  //
  //----------------------------------------------------------------------------
  // Usage
  //----------------------------------------------------------------------------
  //
  // 1. Set file set that should be processed. This will will trigger the
  //    producer this.process_file(my_dat_file, my_stat_file);
  //
  // 2. move the packet pointer forward
  //     this.next();
  //
  // 3. check stream
  //    a) OK --> probably access data with 4)
  //    a) EOF reached --> probably load a new file with 1)
  //    b) Failed --> notify user and load new file with 1)
  //
  // 4. get a pointer to the current packet
  //
  // 5. go to 2)

 public:
  // stat file consts
  // fixed buffer length for stat decompression
  static const int kStatBufLength = 1048576;
  // fixed number of bytes that are required in minimum to have a valid statline
  static const int kStatMinData = 60;
  // we use a cache with the ip addresses of the router to minimize
  static const int kRouterMax = 7;

  // ring buffer const
  static const int kSlots = 10;            // number of slots
  static const int kPacketsPerSlot = 1000;  // number of packets per slot

  // Stream state: Whats the current state of the stream
  enum StreamState {
    kStreamOK = 0,
    kStreamEOF = 1,
    kStreamFailed = 2
  };

  // Slot state: Used to communicate between the consumer/producer
  enum SlotState {
    kSlotEmpty = 0,     // empty packets
    kSlotFilling = 1,   // the producer is filling up this slot
    kSlotFilled = 2,    // the packets are ready to be consumed
    kSlotConsuming = 3,  // the consumer is serving this slot to API
    kSlotEOF = 4,      // produce would fill this slot -- but EOF
    kSlotFailed = 5     // produce would fill this slot -- but Failed
  };

  // Parsing: We are parsing STAT and DAT files and use this
  // error codes to report problem
  enum Parsing {
    kParsingUnknown = -1,  // unknown state
    kParsingOK = 0,  // fine no problem
    kParsingStatPacketMaxLength = 1,  // packet length should < buffer_size.
    kParsingStatPacketMinLength = 2,  // packet length should be > 0
    kParsingStatEOF = 3,  // EOF of the STAT file
    kParsingStatFailed = 4,  // problem related to the parsing STAT
    kParsingDatEOF = 5,  // EOF of the DAT file
    kParsingDatFailed = 6  // problem related to the parsing DAT
  };

  // Time can be undefined (no packet available)
  static const int kTimeUnknown = -1;

  // The Packet idx starts negative. First iteration brings it to 0
  static const int kConsumerPacketIndexInit = -1;

 private:
  // STAT --------------------------------------------------------------------
  std::string stat_path_s_;
  FILE* stat_fp_;
  BZFILE* stat_bz_;
  int stat_bz_err_;

  char* stat_buf_;   // buffer to store decompressed stat
  int stat_index_;
  int stat_filled_;
  uint64_t stat_packet_id_;
  uint64_t stat_packet_id_last_;

  // data to resolve the router addr
  struct sockaddr router_addr_[kRouterMax];

  // DAT --------------------------------------------------------------------
  std::string dat_path_s_;
  FILE* dat_fp_;
  BZFILE* dat_bz_;
  int dat_bz_err_;

  // RING BUFFER  ------------------------------------------------------------
  // each slot is in a unique state:
  std::vector<int> slot_state_;

  // use some mutex for signaling
  std::vector<pthread_mutex_t*> slot_producer_mutex_;
  std::vector<pthread_mutex_t*> slot_consumer_mutex_;

  // the packets
  std::vector<std::vector<Packet> > slot_packets_;
  std::vector<int> slot_packets_used_;

  // Producer ----------------------------------------------------------------
  pthread_t* producer_;
  int producer_running_;
  int producer_slot_idx_;

  // Consumer ----------------------------------------------------------------
  int consumer_slot_idx_;
  int consumer_packet_idx_;
  int consumer_packet_max_;
  int consumer_packet_time_rel_s_;
  int consumer_stream_state_;

 public:
  void reset(void);
  int get_slot_stat(int i);

  void refill_stat_buffer(void);
  int parse_stat_line(char* buffer, int buffer_length, Packet* packet);

  void router_map_init(struct sockaddr * router_addr);
  int parser_router_id(int ip_d);

  int packet_fill_stat(Packet* packet);
  int packet_fill_dat(Packet* packet);
  int producer_parser(void);

  void producer_check_slots(void);
  void producer_main(void);
  void producer_start(void);

  void consumer_update(void);
  void consumer_init(void);
  void consumer_switch(void);

 public:
  CSGFile();
  ~CSGFile();

  // API
  // load a file set
  int process_file(const std::string& file_path_dat,
                   const std::string& file_path_stat);

  // access data
  Packet* get_packet(void);
  inline int get_stream_state() {
    return (consumer_stream_state_);
  }

  inline int get_time_r(void) {
    return (consumer_packet_time_rel_s_);
  }

  // move to the next packet
  void next(void);
};

#endif  // FLOW_BOX_INCLUDE_CSG_FILE_H_
