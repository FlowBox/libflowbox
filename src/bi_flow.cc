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
 * @file   bi_flow.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @author Daniel Aschwanden  <nimdanitro@gmail.com>
 * @date   October, 2012
 * @brief  A bidirectional flow record.
 *
 * This class implements an bidirectional flow record, further called BiFlows.
 * A BiFlow is produced by adding both corresponding unidirectional flows to
 * this data structure using from(flow) and merge(flow).
 *
 */

#include "bi_flow.h"

// Define the Version and Version number

const uint32_t BiFlow::kSeed = 0xdeadbeef;

//------------------------------------------------------------------------------
// The biflow class methods
//------------------------------------------------------------------------------
void BiFlow::reset(void) {
  memset(in_addr_, 0, Flow::kAddressLengthMax);
  memset(out_addr_, 0, Flow::kAddressLengthMax);
  memset(next_in_addr_, 0, Flow::kAddressLengthMax);
  memset(next_out_addr_, 0, Flow::kAddressLengthMax);

  addr_length_ = 0;
  in_port_ = 0;
  out_port_ = 0;
  protocol_ = 0;

  out_in_start_s_ = 0;
  out_in_stop_s_ = 0;
  in_out_start_s_ = 0;
  in_out_stop_s_ = 0;

  in_out_packets_ = 0;
  out_in_packets_ = 0;
  in_out_bytes_ = 0;
  out_in_bytes_ = 0;

  in_out_if_ = -1;
  out_in_if_ = -1;
  in_out_export_device_id_ = -1;
  out_in_export_device_id_ = -1;

  hkey5_ = 0;
  hkey3_in_ = 0;
  hkey3_out_ = 0;
  hkey1_in_ = 0;
  hkey1_out_ = 0;

  valid_ = false;
  next_ = NULL;
}

void BiFlow::from(const Flow& flow) {
  if (flow.direction_ == Flow::kDirectionInOut) {
    memcpy(in_addr_, flow.addr_src_, Flow::kAddressLengthMax);
    memcpy(out_addr_, flow.addr_dst_, Flow::kAddressLengthMax);
    memset(next_in_addr_, 0, Flow::kAddressLengthMax);
    memcpy(next_out_addr_, flow.addr_next_, Flow::kAddressLengthMax);

    in_port_ = flow.port_src_;
    out_port_ = flow.port_dst_;
    in_out_start_s_ = flow.start_s_;
    in_out_stop_s_ = flow.stop_s_;
    out_in_start_s_ = 0;
    out_in_stop_s_ = 0;
    in_out_packets_ = flow.packets_;
    in_out_bytes_ = flow.bytes_;
    out_in_packets_ = 0;
    out_in_bytes_ = 0;
    in_out_if_ = flow.if_out_;
    in_out_export_device_id_ = flow.export_device_id_;
    out_in_if_ = 0;
    out_in_export_device_id_ = 0;
  } else if (flow.direction_ == Flow::kDirectionOutIn) {
    memcpy(in_addr_, flow.addr_dst_, Flow::kAddressLengthMax);
    memcpy(out_addr_, flow.addr_src_, Flow::kAddressLengthMax);
    memcpy(next_in_addr_, flow.addr_next_, Flow::kAddressLengthMax);
    memset(next_out_addr_, 0, Flow::kAddressLengthMax);
    in_port_ = flow.port_dst_;
    out_port_ = flow.port_src_;
    out_in_start_s_ = flow.start_s_;
    out_in_start_s_ = flow.stop_s_;
    in_out_start_s_ = 0;
    in_out_stop_s_ = 0;
    out_in_packets_ = flow.packets_;
    out_in_bytes_ = flow.bytes_;
    in_out_packets_ = 0;
    in_out_bytes_ = 0;
    out_in_if_ = flow.if_in_;
    out_in_export_device_id_ = flow.export_device_id_;
    in_out_if_ = 0;
    in_out_export_device_id_ = 0;
  } else {
    std::cout << "BiFlow::from(Flow) ERROR:" << errno << std::endl;
    throw FlowBoxE("BiFlow::from(Flow) only accepts IN->OUT or OUT->IN traffic",
                   __FILE__, __LINE__);
  }
  // the Protocol and the address length are direction independents
  protocol_ = flow.protocol_;
  addr_length_ = flow.addr_length_;
  // create the hashes of the biflow
  prepare_hashes();
  // now, we have a valid bi_flow
  valid_ = true;
  next_ = NULL;
}

void BiFlow::from(const BiFlow& biflow) {
  // get all the variables..
  valid_ = biflow.valid_;
  // copy the addresses!
  memcpy(in_addr_, biflow.in_addr_, Flow::kAddressLengthMax);
  memcpy(out_addr_, biflow.out_addr_, Flow::kAddressLengthMax);
  memcpy(next_in_addr_, biflow.next_in_addr_, Flow::kAddressLengthMax);
  memcpy(next_out_addr_, biflow.next_out_addr_, Flow::kAddressLengthMax);
  // port and protocol information
  in_port_ = biflow.in_port_;
  out_port_ = biflow.out_port_;
  protocol_ = biflow.protocol_;
  addr_length_ = biflow.addr_length_;
  // flow timing information of in and out flows
  out_in_start_s_ = biflow.out_in_start_s_;
  out_in_stop_s_ = biflow.out_in_stop_s_;
  in_out_start_s_ = biflow.in_out_start_s_;
  in_out_stop_s_ = biflow.in_out_stop_s_;
  in_out_packets_ = biflow.in_out_packets_;
  out_in_packets_ = biflow.out_in_packets_;
  in_out_bytes_ = biflow.in_out_bytes_;
  out_in_bytes_ = biflow.out_in_bytes_;
  in_out_if_ = biflow.in_out_if_;
  out_in_if_ = biflow.out_in_if_;
  in_out_export_device_id_ = biflow.in_out_export_device_id_;
  out_in_export_device_id_ = biflow.out_in_export_device_id_;
  next_ = biflow.next_;
  hkey5_ = biflow.hkey5_;
  hkey3_in_ = biflow.hkey3_in_;
  hkey3_out_ = biflow.hkey3_out_;
  hkey1_in_ = biflow.hkey1_in_;
  hkey1_out_ = biflow.hkey1_out_;
}

void BiFlow::merge(const Flow& flow) {

  if (flow.direction_ == Flow::kDirectionInOut) {
    // update next external hop address
    memcpy(next_out_addr_, flow.addr_next_, Flow::kAddressLengthMax);
    // update the start time only if not set or older
    if (in_out_start_s_ == 0 or flow.start_s_ < in_out_start_s_) {
      in_out_start_s_ = flow.start_s_;
    }
    // update the stop time only if the new is newer
    if (in_out_stop_s_ < flow.stop_s_) {
      in_out_stop_s_ = flow.stop_s_;
    }
    // add the packets and byte counters
    in_out_packets_ += flow.packets_;
    in_out_bytes_ += flow.bytes_;
    // overwrite the interface (always external) and router id..
    in_out_if_ = flow.if_out_;
    in_out_export_device_id_ = flow.export_device_id_;
  } else if (flow.direction_ == Flow::kDirectionOutIn) {
    // update next internal hop address
    memcpy(next_in_addr_, flow.addr_next_, Flow::kAddressLengthMax);
    // update the start time only if not set or older
    if (out_in_start_s_ == 0 or flow.start_s_ < out_in_start_s_) {
      out_in_start_s_ = flow.start_s_;
    }
    // update the stop time only if the new is newer
    if (out_in_stop_s_ < flow.stop_s_) {
      out_in_stop_s_ = flow.stop_s_;
    }
    // add the packets and byte counters
    out_in_packets_ += flow.packets_;
    out_in_bytes_ += flow.bytes_;
    // overwrite the interface(always external) and router id..
    out_in_if_ = flow.if_in_;
    out_in_export_device_id_ = flow.export_device_id_;
  } else {
    std::cout << "BiFlow::from(Flow) ERROR:" << errno << std::endl;
    throw FlowBoxE("BiFlow::from(Flow) only accepts IN->OUT or OUT->IN traffic",
                   __FILE__, __LINE__);
  }
}

// this function creates the different types of hashkeys of a biflow
// the hash function is similar to the hashlittle, but does not need to setup
// a key with memcpy...
void BiFlow::prepare_hashes(void) {
  uint32_t a, b, c;

  // HKey5: <IP in, IP out, Protocol, Port in Port out>
  a = b = c = kSeed;
  biflow_5tp_hash();
  hkey5_ = c;

  // HKey3: <IP, Protocol, Port in>
  a = b = c = kSeed;
  biflow_3tp_in_hash();
  hkey3_in_ = c;

  a = b = c = kSeed;
  biflow_3tp_out_hash();
  hkey3_out_ = c;

  // HKey1: <IP>
  a = b = c = kSeed;
  biflow_1tp_in_hash();
  hkey1_in_ = c;

  a = b = c = kSeed;
  biflow_1tp_out_hash();
  hkey1_out_ = c;
}

// outputs the biflow to a string
std::string BiFlow::to_s(void) const {
  std::stringstream tmp;
  std::string addr_buff;

  tmp << valid_ << ", ";

  Flow::addr_to_s(addr_buff, in_addr_, addr_length_);
  tmp << addr_buff << ", ";
  Flow::addr_to_s(addr_buff, out_addr_, addr_length_);
  tmp << addr_buff << ", ";
  Flow::addr_to_s(addr_buff, next_in_addr_, addr_length_);
  tmp << addr_buff << ", ";
  Flow::addr_to_s(addr_buff, next_out_addr_, addr_length_);
  tmp << addr_buff << ", ";

  tmp << static_cast<int>(protocol_) << ", ";
  tmp << static_cast<int>(addr_length_) << ", ";
  tmp << static_cast<int>(in_port_) << ", ";
  tmp << static_cast<int>(out_port_) << ", ";

  // flow timing information of in and out flows
    tmp << in_out_start_s_ << ", ";
  tmp << in_out_stop_s_ << ", ";
  tmp << out_in_start_s_ << ", ";
  tmp << out_in_stop_s_ << ", ";

  // Packet and byte information of in and out flows
  tmp << in_out_packets_ << ", ";
  tmp << out_in_packets_ << ", ";
  tmp << in_out_bytes_ << ", ";
  tmp << out_in_bytes_ << ", ";

  // interface and exporting device id information of
  // in and out flows
  tmp << in_out_if_ << ", ";
  tmp << out_in_if_ << ", ";
  tmp << in_out_export_device_id_ << ", ";
  tmp << out_in_export_device_id_ << ", ";

  // Hash values
  tmp << hkey5_ << ", ";
  tmp << hkey3_in_ << ", ";
  tmp << hkey3_out_ << ", ";
  tmp << hkey1_in_ << ", ";
  tmp << hkey1_out_ << ", ";

  return (std::string(tmp.str()));
}

// this functions just says if two biflows are equal with respect to connection
// timing and splitted routing are not considered by purpose!
bool BiFlow::compare_key5(const BiFlow* other) const {
  // valid input?
  if (other == NULL)
    return false;
  // same object
  if (this == other)
    return true;
  // check cheapest first
  // in port
  if (in_port_ != other->in_port_)
    return false;
  // out port
  if (out_port_ != other->out_port_)
    return false;
  // protocol
  if (protocol_ != other->protocol_)
    return false;
  // address length
  if (addr_length_ != other->addr_length_)
    return false;
  // in addr
  if (memcmp(in_addr_, other->in_addr_, Flow::kAddressLengthMax) != 0)
    return false;
  // out addr
  if (memcmp(out_addr_, other->out_addr_, Flow::kAddressLengthMax) != 0)
    return false;
  // hups ... its seems to be the same :-)
  return true;
}

bool BiFlow::compare_key3(const BiFlow* other, bool in, bool other_in) const {
  // valid input?
  if (other == NULL)
    return false;
  // same object
  if (this == other)
    return true;
  // protocol
  if (protocol_ != other->protocol_)
    return false;
  // address length
  if (addr_length_ != other->addr_length_)
    return false;
  // compare ports & protocol
  if (in) {
    if (other_in) {
      if (in_port_ != other->in_port_)
        return false;
      if (memcmp(in_addr_, other->in_addr_, Flow::kAddressLengthMax) != 0)
        return false;
    } else {
      if (in_port_ != other->out_port_)
        return false;
      if (memcmp(in_addr_, other->out_addr_, Flow::kAddressLengthMax) != 0)
        return false;
    };
  } else {
    if (other_in) {
      if (out_port_ != other->in_port_)
        return false;
      if (memcmp(out_addr_, other->in_addr_, Flow::kAddressLengthMax) != 0)
        return false;
    } else {
      if (out_port_ != other->out_port_)
        return false;
      if (memcmp(out_addr_, other->out_addr_, Flow::kAddressLengthMax) != 0)
        return false;
    };
  };
  return true;
}

bool BiFlow::compare_key1(const BiFlow* other, bool in, bool other_in) const {
  // valid input?
  if (other == NULL)
    return false;
  // same object
  if (this == other)
    return true;
  // address length
  if (addr_length_ != other->addr_length_)
    return false;
  // compare ports & protocol
  if (in) {
    if (other_in) {
      if (memcmp(in_addr_, other->in_addr_, Flow::kAddressLengthMax) != 0)
        return false;
    } else {
      if (memcmp(in_addr_, other->out_addr_, Flow::kAddressLengthMax) != 0)
        return false;
    };
  } else {
    if (other_in) {
      if (memcmp(out_addr_, other->in_addr_, Flow::kAddressLengthMax) != 0)
        return false;
    } else {
      if (memcmp(out_addr_, other->out_addr_, Flow::kAddressLengthMax) != 0)
        return false;
    };
  };
  return true;
}

// Default Constructor of BiFlow
BiFlow::BiFlow()
    : valid_(false) {
}

BiFlow::BiFlow(const BiFlow& other) {
  from(other);
}

BiFlow::BiFlow(const Flow& flow) {
  from(flow);
}

BiFlow::~BiFlow(void) {
}

//------------------------------------------------------------------------------
//  Key5c: BiFlow Hash Storage of bidirectional 5-Tuple of BiFlow
//------------------------------------------------------------------------------
void BiFlow::Key5c::calc_hash(void) {
  uint32_t a, b, c;
  a = b = c = kSeed;
  biflow_5tp_hash();
  hkey_ = c;
}

void BiFlow::Key5c::from(const Key5c& other) {
  memcpy(in_addr_, other.in_addr_, Flow::kAddressLengthMax);
  memcpy(out_addr_, other.out_addr_, Flow::kAddressLengthMax);
  in_port_ = other.in_port_;
  out_port_ = other.out_port_;
  addr_length_ = other.addr_length_;
  protocol_ = other.protocol_;
  hkey_ = other.hkey_;
}

void BiFlow::Key5c::from(const BiFlow& biflow) {
  memcpy(in_addr_, biflow.in_addr_, Flow::kAddressLengthMax);
  memcpy(out_addr_, biflow.out_addr_, Flow::kAddressLengthMax);
  in_port_ = biflow.in_port_;
  out_port_ = biflow.out_port_;
  addr_length_ = biflow.addr_length_;
  protocol_ = biflow.protocol_;
  hkey_ = biflow.hkey5_;
}

void BiFlow::Key5c::from_s(std::string str) {
  std::istringstream stream(str);
  std::string column;
  // IP address IN
  std::getline(stream, column, ',');
  std::string addr_in(column);
  // IP address OUT
  std::getline(stream, column, ',');
  std::string addr_out(column);
  // addr_length
  std::getline(stream, column, ',');
  addr_length_ = atoi(column.c_str());
  // convert the address and store it in the data_ array
  Flow::addr_import_s(addr_in.c_str(), in_addr_, addr_length_);
  // convert the address and store it in the data_ array
  Flow::addr_import_s(addr_out.c_str(), out_addr_, addr_length_);
  // protocol
  std::getline(stream, column, ',');
  protocol_ = atoi(column.c_str());
  // port IN
  std::getline(stream, column, ',');
  in_port_ = atoi(column.c_str());
  // port OUT
  std::getline(stream, column, ',');
  out_port_ = atoi(column.c_str());
  // update hkey
  calc_hash();
  return;
}

std::string BiFlow::Key5c::head_to_s() {
  std::stringstream head;
  head << "IP Address IN,";
  head << "IP Address OUT,";
  head << "Address Length,";
  head << "Protocol,";
  head << "Port IN";
  head << "Port OUT";
  return (head.str());
}

std::string BiFlow::Key5c::to_s() const {
  std::stringstream tmp;
  std::string buf;
  Flow::addr_to_s(buf, in_addr_, addr_length_);
  tmp << buf << ",";
  Flow::addr_to_s(buf, out_addr_, addr_length_);
  tmp << buf << ",";
  tmp << static_cast<int>(addr_length_) << ",";
  tmp << static_cast<int>(in_port_) << ",";
  tmp << static_cast<int>(out_port_);
  return (tmp.str());
}

BiFlow::Key5c::Key5c() {
}

BiFlow::Key5c::~Key5c() {
}

BiFlow::Key5c::Key5c(const BiFlow::Key5c& other) {
  from(other);
}

BiFlow::Key5c::Key5c(const BiFlow& biflow) {
  from(biflow);
}

const BiFlow::Key5c& BiFlow::Key5c::operator=(const BiFlow::Key5c& other) {
  from(other);
  return (*this);
}

bool BiFlow::Key5c::operator()(const Key5c& k1, const Key5c& k2) const {
  if (k1.hkey_ != k2.hkey_) {
    return (false);
  } else {
    if ((memcmp(k1.in_addr_, k2.in_addr_, Flow::kAddressLengthMax) == 0)
        and (memcmp(k1.out_addr_, k2.out_addr_, Flow::kAddressLengthMax) == 0)
        and k1.in_port_ == k2.in_port_ and k1.out_port_ == k2.out_port_
        and k1.addr_length_ == k2.addr_length_
        and k1.protocol_ == k2.protocol_) {
      return (true);
    } else {
      return (false);
    }
  }
}

size_t BiFlow::Key5c::operator()(const Key5c& key) const {
  return (key.hkey_);
}

//------------------------------------------------------------------------------
//  Key5l: BiFlow Hash Storage of bidirectional 5-Tuple of BiFlow (LINKED!)
//------------------------------------------------------------------------------
void BiFlow::Key5l::from(const Key5l& other) {
  data_ = other.data_;
  hkey_ = other.hkey_;
}

void BiFlow::Key5l::from(const BiFlow* biflow) {
  if (biflow != NULL) {
    data_ = biflow;
    hkey_ = biflow->hkey5_;
  } else {
    data_ = NULL;
    hkey_ = 0;
  }
}

BiFlow::Key5l::Key5l(void) {
  data_ = NULL;
  hkey_ = 0;
}

BiFlow::Key5l::Key5l(const Key5l& other) {
  from(other);
}

BiFlow::Key5l::Key5l(const BiFlow* biflow) {
  from(biflow);
}

BiFlow::Key5l::~Key5l(void) {
}

const BiFlow::Key5l& BiFlow::Key5l::operator=(const BiFlow::Key5l& other) {
  from(other);
  return (*this);
}

bool BiFlow::Key5l::operator()(const Key5l& k1, const Key5l& k2) const {
  // sanity check
  if (k1.data_ == NULL)
    return false;

  // chached hash values?
  if (k1.hkey_ != k2.hkey_)
    return (false);
  // same data pointer?
  if (k1.data_ == k2.data_)
    return (true);
  // same flow content?
  return (k1.data_->compare_key5(k2.data_));
}

size_t BiFlow::Key5l::operator()(const Key5l& key) const {
  if (key.data_ != NULL) {
    return key.data_->hkey5_;
  } else {
    return 0;
  }
}

//----------------------------------------------------------------------------
//  Key3c: BiFlow Hash Storage of bidirectional 3-Tuple of BiFlow
//----------------------------------------------------------------------------
void BiFlow::Key3c::calc_hash(void) {
  uint32_t a, b, c;
  a = b = c = BiFlow::kSeed;
  biflow_3tp_hash();
  hkey_ = c;
}

void BiFlow::Key3c::from(const BiFlow::Key3c& other) {
  memcpy(addr_, other.addr_, Flow::kAddressLengthMax);
  port_ = other.port_;
  addr_length_ = other.addr_length_;
  protocol_ = other.protocol_;
  hkey_ = other.hkey_;
}

void BiFlow::Key3c::from(const BiFlow& biflow,
                         BiFlow::Key3c::Direction direction) {
  if (direction == BiFlow::Key3c::kIn) {
    memcpy(addr_, biflow.in_addr_, Flow::kAddressLengthMax);
    port_ = biflow.in_port_;
    addr_length_ = biflow.addr_length_;
    protocol_ = biflow.protocol_;
    hkey_ = biflow.hkey3_in_;
  } else if (direction == BiFlow::Key3c::kOut) {
    memcpy(addr_, biflow.out_addr_, Flow::kAddressLengthMax);
    port_ = biflow.out_port_;
    addr_length_ = biflow.addr_length_;
    protocol_ = biflow.protocol_;
    hkey_ = biflow.hkey3_out_;
  } else {
    throw FlowBoxE("BiFlow::Key3c::from Unknown Direction", __FILE__, __LINE__);
  };
}

void BiFlow::Key3c::from_s(std::string str) {
  std::istringstream stream(str);
  std::string column;
  // IP address
  std::getline(stream, column, ',');
  std::string addr(column);
  // addr_length
  std::getline(stream, column, ',');
  addr_length_ = atoi(column.c_str());
  // convert the address and store it in the data_ array
  Flow::addr_import_s(addr.c_str(), addr_, addr_length_);
  // protocol
  std::getline(stream, column, ',');
  protocol_ = atoi(column.c_str());
  // port
  std::getline(stream, column, ',');
  port_ = atoi(column.c_str());
  // update hkey
  calc_hash();
  return;
}

std::string BiFlow::Key3c::head_to_s() {
  std::stringstream head;
  head << "IP Address,";
  head << "Address Length,";
  head << "Protocol,";
  head << "Port";
  return (head.str());
}

std::string BiFlow::Key3c::to_s() const {
  std::stringstream tmp;
  std::string buf;
  Flow::addr_to_s(buf, addr_, addr_length_);
  tmp << buf << ",";
  tmp << static_cast<int>(addr_length_) << ",";
  tmp << static_cast<int>(protocol_) << ",";
  tmp << static_cast<int>(port_);
  return (tmp.str());
}

BiFlow::Key3c::Key3c(void) {
  addr_length_ = Flow::kAddressLengthUnknown;
  port_ = 0;
  protocol_ = 0;
}

BiFlow::Key3c::Key3c(const Key3c& other) {
  addr_length_ = Flow::kAddressLengthUnknown;
  port_ = 0;
  protocol_ = 0;
  from(other);
}

BiFlow::Key3c::Key3c(const BiFlow& biflow, BiFlow::Key3c::Direction direction) {
  addr_length_ = Flow::kAddressLengthUnknown;
  port_ = 0;
  protocol_ = 0;
  from(biflow, direction);
}

BiFlow::Key3c::~Key3c(void) {
}

const BiFlow::Key3c& BiFlow::Key3c::operator=(const BiFlow::Key3c& other) {
  from(other);
  return (*this);
}

bool BiFlow::Key3c::operator()(const Key3c& k1, const Key3c& k2) const {
  if (k1.hkey_ != k2.hkey_) {
    return (false);
  } else {
    if ((memcmp(k1.addr_, k2.addr_, Flow::kAddressLengthMax) == 0)
        and k1.port_ == k2.port_ and k1.addr_length_ == k2.addr_length_
        and k1.protocol_ == k2.protocol_) {
      return (true);
    } else {
      return (false);
    }
  }
}

size_t BiFlow::Key3c::operator()(const Key3c& key) const {
  return key.hkey_;
}

//------------------------------------------------------------------------------
//  Key3l: BiFlow Hash Storage of bidirectional 5-Tuple of BiFlow (LINKED!)
//------------------------------------------------------------------------------
void BiFlow::Key3l::from(const Key3l& other) {
  data_ = other.data_;
  dir_ = other.dir_;
  hkey_ = other.hkey_;
  // if (dir_ == kUnknown) {
  //    throw FlowBoxE("BiFlow::Key3l::from Wrong direction", __FILE__, __LINE__);
  //  }
}

void BiFlow::Key3l::from(const BiFlow* biflow, Direction direction) {
  // Sanity check of biflow
  if (biflow == NULL) {
    data_ = NULL;
    dir_ = kUnknown;
    hkey_ = 0;
    return;
  }

  data_ = biflow;
  dir_ = direction;
  if (direction == BiFlow::Key3l::kIn)
    hkey_ = biflow->hkey3_in_;
  else if (direction == BiFlow::Key3l::kOut)
    hkey_ = biflow->hkey3_out_;
  else
    throw FlowBoxE("BiFlow::Key3l::from Wrong direction", __FILE__, __LINE__);
}

BiFlow::Key3l::Key3l(void) {
  data_ = NULL;
  dir_ = kUnknown;
  hkey_ = 0;
}

BiFlow::Key3l::Key3l(const Key3l& other) {
  from(other);
}

BiFlow::Key3l::Key3l(const BiFlow* biflow, Direction direction) {
  from(biflow, direction);
}

BiFlow::Key3l::~Key3l(void) {
}

const BiFlow::Key3l& BiFlow::Key3l::operator=(const BiFlow::Key3l& other) {
  data_ = other.data_;
  hkey_ = other.hkey_;
  return (*this);
}

bool BiFlow::Key3l::operator()(const Key3l& k1, const Key3l& k2) const {
  // cached values
  if (k1.hkey_ != k2.hkey_)
    return (false);
  // same data pointer --> same key, right?
  if (k1.data_ == k2.data_)
    return (true);
  // compare the payload.
  return (k1.data_->compare_key3(k2.data_, k1.dir_ == BiFlow::Key3l::kIn,
                                 k2.dir_ == BiFlow::Key3l::kIn));
}

size_t BiFlow::Key3l::operator()(const Key3l& key) const {
  if (key.data_ != NULL) {
    return key.hkey_;
  } else {
    return 0;
  }
}

//------------------------------------------------------------------------------
//  Key1c: BiFlow Hash Storage of bidirectional 1-Tuple of BiFlow
//------------------------------------------------------------------------------
void BiFlow::Key1c::calc_hash(void) {
  uint32_t a, b, c;
  a = b = c = BiFlow::kSeed;
  biflow_1tp_hash();
  hkey_ = c;
}

void BiFlow::Key1c::from(const BiFlow::Key1c& other) {
  memcpy(addr_, other.addr_, Flow::kAddressLengthMax);
  addr_length_ = other.addr_length_;
  hkey_ = other.hkey_;
}

void BiFlow::Key1c::from(const BiFlow& biflow, Direction direction) {
  if (direction == Key1c::kIn) {
    memcpy(addr_, biflow.in_addr_, Flow::kAddressLengthMax);
    addr_length_ = biflow.addr_length_;
    hkey_ = biflow.hkey1_in_;
  } else if (direction == Key1c::kOut) {
    memcpy(addr_, biflow.out_addr_, Flow::kAddressLengthMax);
    addr_length_ = biflow.addr_length_;
    hkey_ = biflow.hkey1_out_;
  } else {
    throw FlowBoxE("BiFlow::Key1c::from Unknown Direction", __FILE__, __LINE__);
  };
}

void BiFlow::Key1c::from_s(std::string str) {
  std::istringstream stream(str);
  std::string column;
  // IP address
  std::getline(stream, column, ',');
  std::string addr(column);
  // addr_length
  std::getline(stream, column, ',');
  addr_length_ = atoi(column.c_str());
  // convert the address and store it in the data_ array
  Flow::addr_import_s(addr.c_str(), addr_, (uint8_t) addr_length_);
  // update hkey
  calc_hash();
  return;
}

std::string BiFlow::Key1c::head_to_s() {
  std::stringstream head;
  head << "IP Address,";
  head << "Address Length,";
  return (head.str());
}

std::string BiFlow::Key1c::to_s() const {
  std::stringstream tmp;
  std::string buf;
  Flow::addr_to_s(buf, addr_, addr_length_);
  tmp << buf << ",";
  tmp << static_cast<int>(addr_length_) << ",";
  return (tmp.str());
}

BiFlow::Key1c::Key1c(void) {
  addr_length_ = Flow::kAddressLengthUnknown;
}

BiFlow::Key1c::Key1c(const BiFlow::Key1c& other) {
  addr_length_ = Flow::kAddressLengthUnknown;
  from(other);
}

BiFlow::Key1c::Key1c(const BiFlow& biflow, Direction direction) {
  addr_length_ = Flow::kAddressLengthUnknown;
  from(biflow, direction);
}

BiFlow::Key1c::~Key1c(void) {
}

const BiFlow::Key1c& BiFlow::Key1c::operator=(const BiFlow::Key1c& other) {
  from(other);
  return (*this);
}

bool BiFlow::Key1c::operator()(const Key1c& k1, const Key1c& k2) const {
  if (k1.hkey_ != k2.hkey_) {
    return (false);
  } else {
    if ((memcmp(k1.addr_, k2.addr_, Flow::kAddressLengthMax) == 0)
        and k1.addr_length_ == k2.addr_length_) {
      return (true);
    } else {
      return (false);
    }
  }
}

size_t BiFlow::Key1c::operator()(const Key1c& key) const {
  return key.hkey_;
}

//----------------------------------------------------------------------------
//  Key1l: BiFlow Hash Storage of bidirectional 1-Tuple of BiFlow (LINKED!)
//----------------------------------------------------------------------------
void BiFlow::Key1l::from(const Key1l& other) {
  data_ = other.data_;
  dir_ = other.dir_;
  hkey_ = other.hkey_;

  // if (dir_ == kUnknown) {
  //   throw FlowBoxE("BiFlow::Key1l::from Wrong direction", __FILE__, __LINE__);
  // }
}

void BiFlow::Key1l::from(const BiFlow* biflow, Direction direction) {
  if (biflow == NULL) {
    data_ = NULL;
    dir_ = kUnknown;
    hkey_ = 0;
    return;
  }

  data_ = biflow;
  dir_ = direction;
  if (direction == BiFlow::Key1l::kIn)
    hkey_ = biflow->hkey1_in_;
  else if (direction == BiFlow::Key1l::kOut)
    hkey_ = biflow->hkey1_out_;
  else
    throw FlowBoxE("BiFlow::Key1l::from Wrong direction", __FILE__, __LINE__);
}

BiFlow::Key1l::Key1l(void) {
  data_ = NULL;
  dir_ = kUnknown;
  hkey_ = 0;
}

BiFlow::Key1l::Key1l(const Key1l& other) {
  from(other);
}

BiFlow::Key1l::Key1l(const BiFlow* biflow, Direction direction) {
  from(biflow, direction);
}

BiFlow::Key1l::~Key1l(void) {
}

const BiFlow::Key1l& BiFlow::Key1l::operator=(const BiFlow::Key1l& other) {
  data_ = other.data_;
  hkey_ = other.hkey_;
  return (*this);
}

bool BiFlow::Key1l::operator()(const Key1l& k1, const Key1l& k2) const {
  // cached values
  if (k1.hkey_ != k2.hkey_)
    return (false);
  // same data pointer --> same key, right?
  if (k1.data_ == k2.data_)
    return (true);
  // compare the payload.
  return (k1.data_->compare_key1(k2.data_, k1.dir_ == BiFlow::Key1l::kIn,
                                 k2.dir_ == BiFlow::Key1l::kIn));
}

size_t BiFlow::Key1l::operator()(const Key1l& key) const {
  if (key.data_ != NULL) {
    return key.hkey_;
  } else {
    return 0;
  }
}
