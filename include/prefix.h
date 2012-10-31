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
 * @file   prefix.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @date   October, 2012
 * @brief  A network prefix data type.
 *
 * This implements a general network prefix such as 192.168.0.0/24.
 *
 */

#ifndef FLOW_BOX_INCLUDE_PREFIX_H__
#define FLOW_BOX_INCLUDE_PREFIX_H__

#include <arpa/inet.h>
#include <cstdlib>
#include <cstring>
#include <cassert>

#include <iostream>
#include <string>
#include <sstream>

#include "common.h"

class Prefix {
  // ## CLASS CONST ############################################################ 
 public:
  static const int kFamilyUnkown;
  static const int kFamilyIPv4;
  static const int kFamilyIPv6;

  // bytes
  static const int kSizeIPv4;
  static const int kSizeIPv6;
  static const int kSizeMax = 16;

  // bits
  static const int kLengthUnknown;
  static const int kLengthIPv4;
  static const int kLengthIPv6;

  // ## CLASS VARIABLES ########################################################
 private:
  int family_;  ///< address family (kFamilyUnkown | kFamilyIPv4 | kFamilyIPv6)
  int length_;  ///< prefix length in bits (0-32; 0-128)
  char addr_[kSizeMax];  ///< IP address

  // ## CLASS FUNCTIONS ########################################################
 private:
  // HELPER
  int addr_get_family(const char* prefix_txt) const;
  void addr_set_unused_bits_to_zero(void);

  // ## CLASS API ##############################################################
 public:
  Prefix(void);
  Prefix(const Prefix& other);
  ~Prefix(void);

  // ACCESSORS
  bool get_valid(void) const;
  int get_family(void) const;
  int get_length(void) const;
  int get_max_length(void) const;
  int get_bit_at(int position) const;

  // FROM
  void from(const std::string& prefix_txt);
  void from(const char* prefix_txt);
  void from_nb(const char* prefix_bin, int size_, int family_);
  void from_nb_zero(const char* prefix_bin, int length, int family);
  // TO
  const std::string to_s(void) const;

  // MODIFYER
  void clear(void);
  void cut(int bits);

  // COMPARE
  int common_bits(const Prefix& other) const;
  bool includes(const Prefix& other) const;
  bool eql(const Prefix& other) const;
};

#endif // FLOW_BOX_INCLUDE_PREFIX
