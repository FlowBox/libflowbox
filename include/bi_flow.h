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

#ifndef FLOW_BOX_INCLUDE_BI_FLOW_H_
#define FLOW_BOX_INCLUDE_BI_FLOW_H_

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <cstring>

#include "common.h"
#include "flow.h"

#define biflow_5tp_hash() \
  a += *reinterpret_cast<uint32_t *>(&in_addr_[0]); \
  b += *reinterpret_cast<uint32_t *>(&in_addr_[4]); \
  c += *reinterpret_cast<uint32_t *>(&in_addr_[8]); \
  my_mix(a, b, c); \
\
  a += *reinterpret_cast<uint32_t *>(&in_addr_[12]); \
  b += *reinterpret_cast<uint32_t *>(&out_addr_[0]); \
  c += *reinterpret_cast<uint32_t *>(&out_addr_[4]); \
  my_mix(a, b, c); \
\
  a += *reinterpret_cast<uint32_t *>(&out_addr_[8]); \
  b += *reinterpret_cast<uint32_t *>(&out_addr_[12]); \
  c += addr_length_; \
  my_mix(a, b, c); \
\
  a += in_port_; \
  b += out_port_; \
  c += protocol_; \
  my_final(a, b, c);

#define biflow_3tp_in_hash() \
  a += *reinterpret_cast<uint32_t *>(&in_addr_[0]); \
  b += *reinterpret_cast<uint32_t *>(&in_addr_[4]); \
  c += *reinterpret_cast<uint32_t *>(&in_addr_[8]); \
  my_mix(a, b, c); \
  a += *reinterpret_cast<uint32_t *>(&in_addr_[12]); \
  c += addr_length_; \
  b += in_port_; \
  c += protocol_; \
  my_final(a, b, c);

#define biflow_3tp_out_hash() \
  a += *reinterpret_cast<uint32_t *>(&out_addr_[0]); \
  b += *reinterpret_cast<uint32_t *>(&out_addr_[4]); \
  c += *reinterpret_cast<uint32_t *>(&out_addr_[8]); \
  my_mix(a, b, c); \
  a += *reinterpret_cast<uint32_t *>(&out_addr_[12]); \
  c += addr_length_; \
  b += out_port_; \
  c += protocol_; \
  my_final(a, b, c);

#define biflow_3tp_hash() \
  a += *reinterpret_cast<uint32_t *>(&addr_[0]); \
  b += *reinterpret_cast<uint32_t *>(&addr_[4]); \
  c += *reinterpret_cast<uint32_t *>(&addr_[8]); \
  my_mix(a, b, c); \
  a += *reinterpret_cast<uint32_t *>(&addr_[12]); \
  c += addr_length_; \
  b += port_; \
  c += protocol_; \
  my_final(a, b, c);

#define biflow_1tp_in_hash() \
  a += *reinterpret_cast<uint32_t *>(&in_addr_[0]); \
  b += *reinterpret_cast<uint32_t *>(&in_addr_[4]); \
  c += *reinterpret_cast<uint32_t *>(&in_addr_[8]); \
  my_mix(a, b, c); \
  a += *reinterpret_cast<uint32_t *>(&in_addr_[12]); \
  b += addr_length_; \
  my_final(a, b, c);

#define biflow_1tp_out_hash() \
  a += *reinterpret_cast<uint32_t *>(&out_addr_[0]); \
  b += *reinterpret_cast<uint32_t *>(&out_addr_[4]); \
  c += *reinterpret_cast<uint32_t *>(&out_addr_[8]); \
  my_mix(a, b, c); \
  a += *reinterpret_cast<uint32_t *>(&out_addr_[12]); \
  b += addr_length_; \
  my_final(a, b, c);

#define biflow_1tp_hash() \
  a += *reinterpret_cast<uint32_t *>(&addr_[0]); \
  b += *reinterpret_cast<uint32_t *>(&addr_[4]); \
  c += *reinterpret_cast<uint32_t *>(&addr_[8]); \
  my_mix(a, b, c); \
  a += *reinterpret_cast<uint32_t *>(&addr_[12]); \
  b += addr_length_; \
  my_final(a, b, c);

//------------------------------------------------------------------------------
//  BiFlow Class: represents a bidirection flow in a Hash
//------------------------------------------------------------------------------
class BiFlow {
 public:
  //--------------------------------------------------------------------------
  // The class constants
  //--------------------------------------------------------------------------

  static const uint32_t kSeed;

  //--------------------------------------------------------------------------
  // The class data
  //--------------------------------------------------------------------------

  // Flag indicating if it is really a bidirectional flow.
  bool valid_;

  // NETWORK ADDRESSES: (network byte order)
  // IN_ADDR: internal ip of biflow
  char in_addr_[Flow::kAddressLengthMax];
  // OUT_ADDR: external ip of biflow
  char out_addr_[Flow::kAddressLengthMax];

  // NEXT_IN_ADDR: ip of next internal hop
  char next_in_addr_[Flow::kAddressLengthMax];
  // NEXT_OUT_ADDR: ip of next external hop
  char next_out_addr_[Flow::kAddressLengthMax];

  // Since biflows can be IPv4 or IPv6, this field indicates
  // the address length and thus the ip version of the flow
  uint8_t addr_length_;

  // PORT information of internal and external flows
  uint16_t in_port_;
  uint16_t out_port_;

  // Protocol information
  uint8_t protocol_;

  // flow timing information of in and out flows
  uint64_t in_out_start_s_;
  uint64_t in_out_stop_s_;
  uint64_t out_in_start_s_;
  uint64_t out_in_stop_s_;

  // Packet and byte information of in and out flows
  uint64_t in_out_packets_;
  uint64_t out_in_packets_;
  uint64_t in_out_bytes_;
  uint64_t out_in_bytes_;

  // interface and exporting device id information of
  // in and out flows
  int in_out_if_;
  int out_in_if_;
  int in_out_export_device_id_;
  int out_in_export_device_id_;

  // Pointer to allow chains of biflows
  BiFlow* next_;

  // Hash keys are directly stored in the biflow to increase performance!
  size_t hkey5_;
  size_t hkey3_in_;
  size_t hkey3_out_;
  size_t hkey1_in_;
  size_t hkey1_out_;

  //--------------------------------------------------------------------------
  // The class methods
  //--------------------------------------------------------------------------
  void reset(void);
  void from(const Flow& flow);
  void from(const BiFlow& biflow);
  void merge(const Flow& flow);
  void prepare_hashes(void);
  std::string to_s(void) const;
  bool compare_key5(const BiFlow* other) const;
  bool compare_key3(const BiFlow* other, bool in, bool other_in) const;
  bool compare_key1(const BiFlow* other, bool in, bool other_in) const;
  BiFlow();
  explicit BiFlow(const BiFlow& other);
  explicit BiFlow(const Flow& flow);
  ~BiFlow(void);

  //----------------------------------------------------------------------------
  //  Key5c: BiFlow Hash Storage Copy Object
  //----------------------------------------------------------------------------
  class Key5c {
   private:
    // 5TP
    char in_addr_[Flow::kAddressLengthMax];
    char out_addr_[Flow::kAddressLengthMax];
    uint8_t addr_length_;
    uint16_t in_port_;
    uint16_t out_port_;
    uint8_t protocol_;
    // Hash(5TP)
    size_t hkey_;

    void calc_hash(void);

   public:
    void from(const Key5c& other);
    void from(const BiFlow& biflow);
    void from_s(std::string str);

    static std::string head_to_s();
    std::string to_s() const;

    const Key5c& operator=(const Key5c& other);
    bool operator()(const Key5c& k1, const Key5c& k2) const;
    size_t operator()(const Key5c& key) const;

    Key5c(void);
    explicit Key5c(const Key5c& other);
    explicit Key5c(const BiFlow& biflow);

    ~Key5c(void);
  };

  //----------------------------------------------------------------------------
  //  Key5l: BiFlow Hash Storage Linked Object
  //----------------------------------------------------------------------------
  class Key5l {
   private:
    const BiFlow* data_;
    size_t hkey_;

   public:
    void from(const Key5l& other);
    void from(const BiFlow* biflow);

    Key5l(void);
    Key5l(const Key5l& other);
    explicit Key5l(const BiFlow* biflow);
    ~Key5l(void);

    // Operators required for hash_map
    const Key5l& operator=(const Key5l& other);
    bool operator()(const Key5l& k1, const Key5l& k2) const;
    size_t operator()(const Key5l& key) const;
  };

  //----------------------------------------------------------------------------
  //  Key3c: BiFlow Hash Storage Copy Object
  //----------------------------------------------------------------------------
  class Key3c {
   public:
    enum Direction {
      kIn,
      kOut
    };

   private:
    // 3TP
    char addr_[Flow::kAddressLengthMax];
    uint8_t addr_length_;
    uint16_t port_;
    uint8_t protocol_;

    // Hash(3TP)
    size_t hkey_;
    void calc_hash(void);

   public:
    void from(const Key3c& other);
    void from(const BiFlow& biflow, Direction direction);
    void from_s(std::string str);

    static std::string head_to_s();
    std::string to_s() const;

    Key3c(void);
    Key3c(const Key3c& other);
    Key3c(const BiFlow& biflow, Direction direction);
    ~Key3c(void);

    const Key3c& operator=(const Key3c& other);
    bool operator()(const Key3c& k1, const Key3c& k2) const;
    size_t operator()(const Key3c& key) const;
  };

  //----------------------------------------------------------------------------
  //  Key3l: BiFlow Hash Storage Linked Object
  //----------------------------------------------------------------------------
  class Key3l {
   public:
    enum Direction {
      kIn,
      kOut,
      kUnknown
    };

   private:
    const BiFlow* data_;
    Direction dir_;
    size_t hkey_;

   public:
    void from(const Key3l& other);
    void from(const BiFlow* biflow, Direction direction);

    Key3l(void);
    Key3l(const Key3l& other);
    Key3l(const BiFlow* biflow, Direction direction);
    ~Key3l(void);

    // Operators required for hash_map
    const Key3l& operator=(const Key3l& other);
    bool operator()(const Key3l& k1, const Key3l& k2) const;
    size_t operator()(const Key3l& key) const;
  };

  //----------------------------------------------------------------------------
  //  Key1c: BiFlow Hash Storage of Host Copy Object
  //----------------------------------------------------------------------------
  class Key1c {
   public:
    enum Direction {
      kIn,
      kOut
    };

   private:
    // 1TP
    char addr_[Flow::kAddressLengthMax];
    uint8_t addr_length_;

    // Hash(3TP)
    size_t hkey_;
    void calc_hash(void);

   public:
    void from(const Key1c& other);
    void from(const BiFlow& biflow, Direction direction);
    void from_s(std::string str);

    static std::string head_to_s();
    std::string to_s() const;

    Key1c(void);
    Key1c(const Key1c& other);
    Key1c(const BiFlow& biflow, Direction direction);
    ~Key1c(void);

    const Key1c& operator=(const Key1c& other);
    bool operator()(const Key1c& k1, const Key1c& k2) const;
    size_t operator()(const Key1c& key) const;
  };

  //----------------------------------------------------------------------------
  //  Key1l: BiFlow Hash Storage of Host Linked Object
  //----------------------------------------------------------------------------
  class Key1l {
   public:
    enum Direction {
      kIn,
      kOut,
      kUnknown
    };

   private:
    const BiFlow* data_;
    Direction dir_;
    size_t hkey_;

   public:
    void from(const Key1l& other);
    void from(const BiFlow* biflow, Direction direction);

    Key1l(void);
    Key1l(const Key1l& other);
    Key1l(const BiFlow* biflow, Direction direction);
    ~Key1l(void);

    // Operators required for hash_map
    const Key1l& operator=(const Key1l& other);
    bool operator()(const Key1l& k1, const Key1l& k2) const;
    size_t operator()(const Key1l& key) const;
  };
};
#endif  // FLOW_BOX_INCLUDE_BI_FLOW_H_
