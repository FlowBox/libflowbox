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

#include "csg_file.h"
//------------------------------------------------------------------------------
// CSGFile
//------------------------------------------------------------------------------

/// We store the IP address of the routers in network byte order in
/// return the index of cache (router_addr_). This function returns
/// the index of the cache to use based on the last part of the
/// IP address.
int CSGFile::parser_router_id(int ip_d) {
  // NOTE: We were requested to remove this code segment
  // due to privacy concerns. Please contact
  // schadomi directly to help out. Sorry ...
  throw FlowBoxE("Please update router IP to router object map first!",
                 __FILE__, __LINE__);
  return (0);
}

/// Initialize the router cache.
void CSGFile::router_map_init(struct sockaddr * router_addr) {
  ((struct sockaddr_in*) (&router_addr[0]))->sin_family = AF_INET;
  ((struct sockaddr_in*) (&router_addr[0]))->sin_port = 0;
  inet_pton(AF_INET, "127.0.0.1",
            &(((struct sockaddr_in*) (&router_addr[0]))->sin_addr));

  // NOTE: We were requested to remove this code segment
  // due to privacy concerns. Please contact
  // schadomi directly to help out. Sorry ...
  throw FlowBoxE("Please update router IP to router object map first)",
                 __FILE__, __LINE__);
}

/// Reset the data structure -- to be ready to process a new file set
/// - flush all error states
/// - reset all locks
/// - allocate buffers if required.
void CSGFile::reset(void) {
  // no edit, as long as thread is working !
  if (producer_running_) {
    throw FlowBoxE("Producer is still running", __FILE__, __LINE__);
  }

  // STAT ----------------------------------------------------------------------
  if (stat_bz_ != NULL)
    BZ2_bzReadClose(&stat_bz_err_, stat_bz_);
  stat_bz_ = NULL;

  if (stat_fp_ != NULL)
    fclose(stat_fp_);
  stat_fp_ = NULL;

  // allocate buffers if required
  if (stat_buf_ == NULL) {
    stat_buf_ = new char[kStatBufLength];
  }

  stat_path_s_ = "";
  stat_index_ = 0;
  stat_filled_ = 0;
  stat_packet_id_ = 0;
  stat_packet_id_last_ = 0;

  // update router mapping
  router_map_init(router_addr_);

  // DAT -----------------------------------------------------------------------
  if (dat_bz_ != NULL)
    BZ2_bzReadClose(&dat_bz_err_, stat_bz_);
  dat_bz_ = NULL;

  if (dat_fp_ != NULL)
    fclose(dat_fp_);
  dat_fp_ = NULL;
  dat_path_s_ = "";

  // RING BUFFER ---------------------------------------------------------------
  //  -- state
  slot_state_.resize(kSlots);
  for (int i = 0; i < kSlots; i++) {
    slot_state_[i] = kSlotEmpty;
  }

  // -- packets
  slot_packets_.resize(kSlots);
  slot_packets_used_.resize(kSlots);

  for (int i = 0; i < kSlots; i++) {
    slot_packets_[i].resize(kPacketsPerSlot);
    slot_packets_used_[i] = 0;

#ifdef CSG_FILE_DEBUG
    std::vector<Packet>::iterator begin, end, iter;
    begin = slot_packets_[i].begin();
    end = slot_packets_[i].end();
    iter = begin;
    while (iter != end) {
      iter->validation_ = -1;
      iter++;
    }
#endif
  }

  // -- mutex
  // delete old ones
  for (unsigned int i = 0; i < slot_producer_mutex_.size(); i++) {
    delete slot_producer_mutex_[i];
    delete slot_consumer_mutex_[i];
  }
  // add new ones
  slot_producer_mutex_.resize(kSlots);
  slot_consumer_mutex_.resize(kSlots);
  for (int i = 0; i < kSlots; i++) {
    slot_producer_mutex_[i] = new pthread_mutex_t();
    pthread_mutex_init(slot_producer_mutex_[i], NULL);

    slot_consumer_mutex_[i] = new pthread_mutex_t();
    pthread_mutex_init(slot_consumer_mutex_[i], NULL);
    pthread_mutex_lock(slot_consumer_mutex_[i]);
  }

  // -- producer
  if (producer_ == NULL) {
    producer_ = new pthread_t();
  }
  producer_slot_idx_ = 0;

  // -- consumer
  consumer_slot_idx_ = 0;
  consumer_packet_idx_ = kConsumerPacketIndexInit;
  consumer_packet_time_rel_s_ = kTimeUnknown;
  consumer_stream_state_ = kStreamOK;
}

// STAT ------------------------------------------------------------------------
/// decompress data from file into the 'stat_buf_' for further analysis.
void CSGFile::refill_stat_buffer(void) {
  // move the n 'unused' data to begin
  int n = stat_filled_ - stat_index_;
  memcpy(stat_buf_, stat_buf_ + stat_index_, n);
  stat_index_ = 0;
  stat_filled_ = n;

  // fill up the rest
  n = BZ2_bzRead(&stat_bz_err_, stat_bz_, stat_buf_ + stat_filled_,
                 kStatBufLength - stat_filled_);
  if (stat_bz_err_ == BZ_OK || (stat_bz_err_ == BZ_STREAM_END && n > 0)) {
    // sucess or sucess and end of file reached
    stat_filled_ = stat_filled_ + n;
  } else {
    std::stringstream err_msg;
    // never saw this case ... let us debug this case first
    err_msg << "CSGFile::refill_stat_buffer " << std::endl;
    err_msg << "Warning:: BZ ERROR CODE: " << stat_bz_err_ << std::endl;
    err_msg << "Warning:: BZ READ N: " << n << std::endl;
    throw FlowBoxE(err_msg.str(), __FILE__, __LINE__);
  }
}

/// analyse the this stat line and store the inforamtion in the packet
/// informe the caller about the sucess or error using.
int CSGFile::parse_stat_line(char* buffer, int buffer_length, Packet* packet) {
  // parse the stat line
  // return the real length, this number is used to jump to the next P

#ifdef CSG_FILE_DEBUG
  char tmp[50];
  memcpy(tmp, buffer, 48);
  tmp[49] = 0;
  std::cout << tmp << std::endl;
#endif

  char* nws = NULL;

  // PACKET ID -------------------------------------------------------
  // find the start of the PACKET ID
  buffer += 1;
  while (*buffer == ' ') {
    buffer++;
  }
  nws = buffer;  // this is the start of the PACKET ID
  // find the end of the PACKET ID
  while (*nws != ' ') {
    nws++;
  }
  (*nws) = 0;  // this is the end of the PACKET ID
  stat_packet_id_ = strtoull(buffer, NULL, 10);  // get numerical value

  // Sanity Check :: packet id
  // this allows us to detect parsing errors --- never comment it out !!
  if (stat_packet_id_ - stat_packet_id_last_ == 1 || stat_packet_id_ == 0) {
    stat_packet_id_last_ = stat_packet_id_;
  } else if (stat_packet_id_last_ == 0) {
    stat_packet_id_last_ = stat_packet_id_;
  } else {
    std::stringstream err_msg;
    err_msg << "CSGFile::parse_stat_line: Missing Packet ID"
            << stat_packet_id_last_ << " -> " << stat_packet_id_ << std::endl;
    err_msg << "Did somebody changed the syntax of the stat file?" << std::endl;
    throw FlowBoxE(err_msg.str(), __FILE__, __LINE__);
  }

  // RELATIVE ARRIVAL TIME -------------------------------------------
  buffer = nws + 1;

  // find the start of the RELATIVE ARRIVAL TIME
  while (*buffer == ' ') {
    buffer++;
  }
  nws = buffer;  // this is the start of the RELATIVE ARRIVAL TIME

  // find the end of the RELATIVE ARRIVAL TIME
  while (*nws != ' ') {
    nws++;
  }
  (*nws) = 0;  // this is the end of the RELATIVE ARRIVAL TIME

  packet->time_s_ = atof(buffer);  // get numerical value

  // PACKET LENGTH ---------------------------------------------------
  buffer = nws + 1;
  // find the start of the PACKET LENGTH
  while (*buffer == ' ') {
    buffer++;
  }
  nws = buffer;  // this is the start of the PACKET LENGTH

  // find the end of the PACKET LENGTH
  while (*nws != ' ') {
    nws++;
  }
  (*nws) = 0;  // this is the end of the PACKET LENGTH
  packet->buffer_length_ = atoi(buffer);  // get numerical value

  // ROUTER ID (and PORT) --------------------------------------------
  // router ID is the last part of the IP
  buffer = nws + 1;
  // find the start of the router address
  while (*buffer == ' ') {
    buffer++;
  }
  buffer += 10;  // -> jump to last IP part: here 30
  nws = buffer;

  // find the start of the router address
  while (*nws != ':') {
    nws++;
  }
  (*nws) = 0;

  int packet_router_id_ = parser_router_id(atoi(buffer));
  memcpy(&packet->addr_, &router_addr_[packet_router_id_],
         sizeof(struct sockaddr));
  packet->addr_length_ = 4;

  // update buffer index
  stat_index_ = nws - stat_buf_;

  // validation ...
  if (packet->buffer_length_ >= Packet::kBufferLength) {
    return (kParsingStatPacketMaxLength);
  } else if (packet->buffer_length_ < 0) {
    return (kParsingStatPacketMinLength);
  } else {
    return (kParsingOK);
  }
}

int CSGFile::packet_fill_stat(Packet* packet) {
  while (true) {
    // find and parse the next 'P  ' entry
    // refill ...
    if (stat_index_ + kStatMinData > stat_filled_) {
      // ... end of the stat file reached, no 'P ' entry found
      if (stat_bz_err_ == BZ_STREAM_END)
        return (kParsingStatEOF);

      // ... get new data
      refill_stat_buffer();

      // ... without successful?
      if (stat_index_ + kStatMinData > stat_filled_)
        return (kParsingStatFailed);
    }

    // parsing ...
    // ... search for the next P line in the buffer and parse it
    char* buffer = stat_buf_ + stat_index_;
    int buffer_length = stat_filled_ - stat_index_ - 1;
    char* pch = reinterpret_cast<char*>(memchr(buffer, 'P', buffer_length));

    if (pch != NULL) {  // 'P' found
      // check if it a 'P  ' that indicates
      if (*(pch + 1) == ' ' and *(pch + 2) == ' ') {  // its a 'P  '
        return (parse_stat_line(pch, buffer_length - (pch - buffer), packet));
      } else {  // its not a 'P  ', step forward one char and search again
        stat_index_ = pch - stat_buf_ + 1;
      }
    } else {  // no 'P' found
      // no P found, step forward to the next buffer
      stat_filled_ = stat_index_;
    }
  }  // while, try again
}

int CSGFile::packet_fill_dat(Packet* packet) {
  // Copy the packet payload form the compressed stream
  // into to'packet->buffer_'

  int n = BZ2_bzRead(&dat_bz_err_, dat_bz_, packet->buffer_,
                     packet->buffer_length_);
  if (n == packet->buffer_length_) {
    return (kParsingOK);
  }
  // ERROR HANDLING: DAT failed, but WHY ? -------------------------------------
  // map 'why' to 'ParsingError'
  if (dat_bz_err_ == BZ_STREAM_END) {
    return (kParsingDatEOF);
  } else if (dat_bz_err_ == BZ_STREAM_END) {
    return (kParsingDatFailed);
  } else {
    throw FlowBoxE("ApplicationLogicBrocken", __FILE__, __LINE__);
  }
}

// fill the slot 'producer_slot_idx_' with new packets
int CSGFile::producer_parser(void) {
  std::vector<Packet>::iterator begin, end, iter;
  int error = kParsingUnknown;

  begin = slot_packets_[producer_slot_idx_].begin();
  end = slot_packets_[producer_slot_idx_].end();

  iter = begin;
  while (iter != end) {
#ifdef CSG_FILE_DEBUG
    if (iter->validation_ != -1) {
      printf(
          "CSGFile[%p]::producer parser VALIDATION ERROR SlotIdx: %i PacketIdx %ti\n",
          this,
          producer_slot_idx_,
          std::distance(begin, iter));
      throw FlowBoxE("CSGFile::producer_parser VALIDATION ERROR",
                     __FILE__, __LINE__);
    };
#endif

    if ((error = packet_fill_stat(&(*iter))) != kParsingOK)
      break;
    if ((error = packet_fill_dat(&(*iter))) != kParsingOK)
      break;

    // parsing OK -- go one
#ifdef CSG_FILE_DEBUG
    iter->validation_ = 1;
#endif
    iter++;
  }

#ifdef CSG_FILE_DEBUG
  std::cout << "Producer filled " << std::distance(begin, iter);
  std::cout << " packets into slot " << producer_slot_idx_;
  std::cout << " [Error =  "<< error << "]" << std::endl;
  std::cout.flush();
#endif

  slot_packets_used_[producer_slot_idx_] = std::distance(begin, iter);

  // ERROR HANDLING: Either DAT or STAT parsing failed ------------------------
  if (error == kParsingOK) {
    return (kParsingOK);
  } else if (error == kParsingStatEOF) {
    return (kStreamEOF);

    // TODO(schadomi): Check if DAT file is EOF as well, otherwise raise exception.
    // Haven't seen this cases in real data ... let me debug it first, thanks.
  } else if (error == kParsingStatFailed) {
    throw FlowBoxE("DEBUG FIRST: ParsingStatFailed", __FILE__, __LINE__);
    return (kStreamFailed);
  } else if (error == kParsingStatPacketMinLength) {
    throw FlowBoxE("DEBUG FIRST: kParsingStatPacketMinLength", __FILE__,
                   __LINE__);
    return (kStreamFailed);
  } else if (error == kParsingStatPacketMaxLength) {
    throw FlowBoxE("DEBUG FIRST: kParsingStatPacketMaxLength", __FILE__,
                   __LINE__);
    return (kStreamFailed);
  } else if (error == kParsingDatEOF) {
    throw FlowBoxE("DEBUG FIRST: kParsingDatEOF", __FILE__, __LINE__);
    return (kStreamFailed);
  } else if (error == kParsingDatFailed) {
    throw FlowBoxE("DEBUG FIRST: kParsingDatFailed", __FILE__, __LINE__);
    return (kStreamFailed);
  } else {
    throw FlowBoxE("Aplication Logic Brocken", __FILE__, __LINE__);
    return (kStreamFailed);
  }
}

void* producer_main_wrapper(void *ptr) {
  CSGFile* obj = reinterpret_cast<CSGFile*>(ptr);
  obj->producer_main();
  return (NULL);
}

void CSGFile::producer_check_slots(void) {
  // ... slots ready?
  for (int i = 0; i < kSlots; i++) {
    // ... can we access the slot i?
    int err = pthread_mutex_trylock(slot_producer_mutex_[i]);
    if (err == 0)
      pthread_mutex_unlock(slot_producer_mutex_[i]);
    else
      throw FlowBoxE("Can't lock mutex", __FILE__, __LINE__);

    // ... is slots i empty?
    if (slot_state_[i] != kSlotEmpty)
      throw FlowBoxE("Slot not empty", __FILE__, __LINE__);
  }
}

void CSGFile::producer_main(void) {
#ifdef CSG_FILE_DEBUG
  printf("CSGFile[%p]::producer main -- start\n", this);
#endif
  producer_running_ = true;
  producer_check_slots();

  int err = kStreamOK;
  while (err == kStreamOK) {
#ifdef CSG_FILE_DEBUG
    printf("CSGFile[%p]::producer lock %i\n", this, producer_slot_idx_);
#endif

    // wait on next slot
    pthread_mutex_lock(slot_producer_mutex_[producer_slot_idx_]);
    assert(slot_state_[producer_slot_idx_] == kSlotEmpty);

    // parse the dat & stat file to packets
    slot_state_[producer_slot_idx_] = kSlotFilling;
    err = producer_parser();
    slot_state_[producer_slot_idx_] = kSlotFilled;

#ifdef CSG_FILE_DEBUG
    printf("CSGFile[%p]::producer filled %i (P: %i S: %i E: %i)\n",
        this,
        producer_slot_idx_,
        slot_packets_used_[producer_slot_idx_],
        slot_state_[producer_slot_idx_],
        err);
#endif

    // signal consumer
#ifdef CSG_FILE_DEBUG
    printf("CSGFile[%p]::producer unlock %i\n", this, producer_slot_idx_);
#endif
    pthread_mutex_unlock(slot_consumer_mutex_[producer_slot_idx_]);

    producer_slot_idx_++;
    producer_slot_idx_ = producer_slot_idx_ % kSlots;
  }

#ifdef CSG_FILE_DEBUG
  printf("CSGFile[%p]::producer finishing\n", this);
#endif

  // set next slot state
  int slot_state;
  if (err == kStreamEOF) {
    slot_state = kSlotEOF;
  } else {
    slot_state = kSlotFailed;
  }

#ifdef CSG_FILE_DEBUG
  printf("CSGFile[%p]::producer lock %i\n", this, producer_slot_idx_);
#endif
  pthread_mutex_lock(slot_producer_mutex_[producer_slot_idx_]);

  slot_state_[producer_slot_idx_] = slot_state;
  slot_packets_used_[producer_slot_idx_] = 0;

#ifdef CSG_FILE_DEBUG
  printf("CSGFile[%p]::producer finished %i (P: %i S: %i E: %i)\n",
      this,
      producer_slot_idx_,
      slot_packets_used_[producer_slot_idx_],
      slot_state_[producer_slot_idx_],
      err);
#endif

  // signal end of processing, then unlock the slot
  producer_running_ = false;

#ifdef CSG_FILE_DEBUG
  printf("CSGFile[%p]::producer unlock %i\n", this, producer_slot_idx_);
#endif
  pthread_mutex_unlock(slot_consumer_mutex_[producer_slot_idx_]);

#ifdef CSG_FILE_DEBUG
  printf("CSGFile[%p]::producer main -- end\n", this);
#endif
}

void CSGFile::producer_start(void) {
  // TODO(schadomi): Protect producer_running_ with synchronized

  if (producer_running_ == false) {
    // producer_main_wrapper(this);
    pthread_create(producer_, NULL, producer_main_wrapper,
                   reinterpret_cast<void*>(this));
  } else {
    throw FlowBoxE("Producer still running?", __FILE__, __LINE__);
  }
}

void CSGFile::consumer_update(void) {
#ifdef CSG_FILE_DEBUG
  printf("CSGFile[%p]::consumer consume %i\n", this, consumer_slot_idx_);
#endif

  int state = slot_state_[consumer_slot_idx_];
  if (state == kSlotFilled) {
    slot_state_[consumer_slot_idx_] = kSlotConsuming;
    consumer_packet_max_ = slot_packets_used_[consumer_slot_idx_];
    consumer_stream_state_ = kStreamOK;
  } else if (state == kSlotEOF) {
    consumer_stream_state_ = kStreamEOF;
  } else {
    consumer_stream_state_ = kStreamFailed;
    // please, let me debug this case first.
    std::stringstream err_msg;
    err_msg << "DEBUG FIRST: Consumer found slot with invalid state! "
            << "Consumer found slot :" << consumer_slot_idx_
            << " State:" << state << std::endl;
    throw FlowBoxE(err_msg.str(), __FILE__, __LINE__);
  }
}

/// wait on a valid entry point for the consumer.
void CSGFile::consumer_init(void) {
  // we have to wait until slot = 0 is ready
  consumer_slot_idx_ = 0;

  // wait on data in slot 0
#ifdef CSG_FILE_DEBUG
  printf("CSGFile[%p]::consumer lock %i\n", this, consumer_slot_idx_);
#endif
  pthread_mutex_lock(slot_consumer_mutex_[consumer_slot_idx_]);
  consumer_update();

  // set pointer to before valid packet
  assert(consumer_packet_idx_ == kConsumerPacketIndexInit);

  // is this slot is marked as filled but is, in fact, empty?
  if (consumer_stream_state_ == kStreamOK and consumer_packet_max_ == 0) {
    // ... a rare case but a valid case
    // ... just move to the next slot
    consumer_switch();
  }
}

void CSGFile::consumer_switch(void) {
  while (true) {
    // clean up
#ifdef CSG_FILE_DEBUG
    printf("CSGFile[%p]::consumer unlock %i\n", this, consumer_slot_idx_);
#endif
    slot_state_[consumer_slot_idx_] = kSlotEmpty;
    slot_packets_used_[consumer_slot_idx_] = 0;
    pthread_mutex_unlock(slot_producer_mutex_[consumer_slot_idx_]);

    // next bucket
    consumer_slot_idx_++;
    consumer_slot_idx_ = consumer_slot_idx_ % kSlots;

    // wait on data
#ifdef CSG_FILE_DEBUG
    printf("CSGFile[%p]::consumer lock %i\n", this, consumer_slot_idx_);
#endif
    pthread_mutex_lock(slot_consumer_mutex_[consumer_slot_idx_]);
    consumer_update();
    consumer_packet_idx_ = 0;

    // is this slot is marked as filled but is, in fact, empty?
    if (consumer_stream_state_ == kStreamOK and consumer_packet_max_ == 0) {
      // ... a rare case but a valid case
      // ... just move to the next slot
#ifdef CSG_FILE_DEBUG
      std::cout << "CSGFile::consumer_switch -- double switch" << std::endl;
      std::cout.flush();
#endif
      continue;
    } else if (consumer_stream_state_ == kStreamOK
        and consumer_packet_max_ > 0) {
      // normal case
      return;
    } else if (consumer_stream_state_ == kStreamEOF) {
      // normal case
#ifdef CSG_FILE_DEBUG
      std::cout << "CSGFile::consumer_switch -- EOF " << std::endl;
      std::cout.flush();
#endif
      return;
    } else {
      // should never happen
      std::stringstream err_msg;
      err_msg << "Application Logic?!? "
              << "Consumer Stream State: " << consumer_stream_state_
              << std::endl << "Consumer Packet Max: " << consumer_packet_max_
              << std::endl;
      throw FlowBoxE(err_msg.str(), __FILE__, __LINE__);
      // 127.0.0.1
    }
  }
}

//------------------------------------------------------------------------------
// API
//------------------------------------------------------------------------------
CSGFile::CSGFile()
    : stat_fp_(NULL),
      stat_bz_(NULL),
      stat_buf_(NULL),
      dat_fp_(NULL),
      dat_bz_(NULL),
      producer_(NULL),
      producer_running_(false) {
  reset();
}

CSGFile::~CSGFile() {
  // we have to wait on the thread...
  if (producer_running_) {
    pthread_cancel(*producer_);
  }

  if (stat_buf_ != NULL) {
    delete[] stat_buf_;
    stat_buf_ = NULL;
  }
  if (producer_ != NULL) {
    delete producer_;
    producer_ = NULL;
  }
  // TODO(schadomi): delete Mutex
}

int CSGFile::process_file(const std::string& file_path_dat,
                          const std::string& file_path_stat) {
  // 0. reset & update path information
  // 1. open file hander
  // 2. init decompression
  // 3. start the producer

  // 0. reset
  reset();
  stat_path_s_ = file_path_stat;
  dat_path_s_ = file_path_dat;

  // 1. open file hander
  stat_fp_ = fopen(stat_path_s_.c_str(), "r");
  dat_fp_ = fopen(dat_path_s_.c_str(), "r");
  if (dat_fp_ == NULL or ferror(dat_fp_) or stat_fp_ == NULL
      or ferror(stat_fp_)) {
    std::stringstream err_msg;
    err_msg << "CSGFile::process_file: Can't open these files:'" << std::endl
            << "STAT:'" << stat_path_s_ << "'" << std::endl
            << "DAT: '" << dat_path_s_ << "'" << std::endl;
    reset();
    consumer_stream_state_ = kStreamFailed;
    throw FlowBoxE(err_msg.str(), __FILE__, __LINE__);
  }

  // 2. init decompression
  const int verbosity = 0;  // no output required
  const int small = 0;      // we have enough memory
  void* unused = NULL;      // we don't care about what's "after" the stream
  const int unused_n = 0;   // we don't care about what's "after" the stream

  stat_bz_ = BZ2_bzReadOpen(&stat_bz_err_, stat_fp_, verbosity, small, unused,
                            unused_n);
  dat_bz_ = BZ2_bzReadOpen(&dat_bz_err_, dat_fp_, verbosity, small, unused,
                           unused_n);

  if (stat_bz_err_ != BZ_OK or dat_bz_err_ != BZ_OK) {
    std::stringstream err_msg;
    err_msg << "CSGFile::process_file: Can't open BZ2 Streams for reading'"
            << std::endl
            << "'" << stat_path_s_ << "'" << std::endl
            << "BZERROR STAT: '" << stat_bz_err_ << "'" << std::endl
            << "'" << dat_path_s_ << "'" << std::endl
            << "BZERROR DAT: '" << dat_bz_err_ << "'" << std::endl;
    reset();
    consumer_stream_state_ = kStreamFailed;
    throw FlowBoxE(err_msg.str(), __FILE__, __LINE__);
    return (consumer_stream_state_);
  }

  // we opened both stream successfully
  consumer_stream_state_ = kStreamOK;

  // 3. start the producer
  producer_start();

  // 4. wait on a vaild entry point for the consumer
  consumer_init();

  return (consumer_stream_state_);
}

Packet* CSGFile::get_packet(void) {
#ifdef CSG_FILE_DEBUG
  printf("CSGFile[%p]::get_packet SlotIdx: %i PacketIdx %i\n",
      this,
      consumer_slot_idx_,
      consumer_packet_idx_);
#endif

#ifdef CSG_FILE_DEBUG
  if (consumer_slot_idx_ < 0 or consumer_packet_idx_ < 0
      or consumer_packet_idx_ >= consumer_packet_max_) {
    throw FlowBoxE("CSGFile::producer_parser VALIDATION ERROR",
                   __FILE__, __LINE__);
  }
#endif

  if (consumer_stream_state_ != kStreamOK)
    throw FlowBoxE("API Abuse Detected -- READ DOC FIRST", __FILE__, __LINE__);

  return (&slot_packets_[consumer_slot_idx_][consumer_packet_idx_]);
}

void CSGFile::next(void) {
#ifdef CSG_FILE_DEBUG
  if (consumer_packet_idx_ >= 0) {
    if (slot_packets_[consumer_slot_idx_][consumer_packet_idx_].validation_
        != 1) {
      throw FlowBoxE("CSGFileThread::get VALIDATION ERROR", __FILE__, __LINE__);
    }
    slot_packets_[consumer_slot_idx_][consumer_packet_idx_].validation_ = -1;
  }
#endif

  consumer_packet_idx_++;

  // switch to the next slot?
  if (consumer_packet_idx_ >= consumer_packet_max_) {
    consumer_switch();
  }

  // update time, if stream is valid
  if (consumer_stream_state_ == kStreamOK) {
    assert(consumer_packet_idx_ >= 0);
    consumer_packet_time_rel_s_ =
        slot_packets_[consumer_slot_idx_][consumer_packet_idx_].time_s_;
  } else {
    consumer_packet_time_rel_s_ = kTimeUnknown;
  }

  // application logic sanity check
  if (consumer_stream_state_ == kStreamOK) {
    assert(consumer_packet_idx_ < consumer_packet_max_);
    assert(consumer_packet_max_ > 0);
    assert(consumer_packet_time_rel_s_ >= 0);
  }
}

