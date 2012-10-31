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
 * @file   prefix.cc
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @date   October, 2012
 * @brief  A network prefix data type.
 *
 * This implements a general network prefix such as 192.168.0.0/24.
 *
 */

#include "prefix.h"
//------------------------------------------------------------------------------
// class Prefix::Const
//------------------------------------------------------------------------------
const int Prefix::kFamilyUnkown = -1;
const int Prefix::kFamilyIPv4 = 4;
const int Prefix::kFamilyIPv6 = 16;

const int Prefix::kSizeIPv4 = 4;
const int Prefix::kSizeIPv6 = 16;
// const int Prefix::kSizeMax = 16;

const int Prefix::kLengthUnknown = 0;
const int Prefix::kLengthIPv4 = 32;
const int Prefix::kLengthIPv6 = 128;

//-----------------------------------------------------------------------------
// HELPER
//-----------------------------------------------------------------------------

int Prefix::addr_get_family(const char* prefix_txt) const {
  // guess prefix family from string

  if (strchr(prefix_txt, ':'))  // : or mixed notation, but both include :
    return (kFamilyIPv6);
  else if (strchr(prefix_txt, '.'))
    return (kFamilyIPv4);
  else
    return (kFamilyUnkown);
}

void Prefix::addr_set_unused_bits_to_zero() {
  // correction of malformed prefixes by
  // set all bits above the prefix length_ to zero
  if (length_ < kSizeMax * 8) {
    int byte = length_ >> 3;   // / 8
    int bits = length_ & 0x07;  // % 8
    uint8_t bit_mask = 255 << (8 - bits);
    addr_[byte] = addr_[byte] & bit_mask;
    for (int i = byte + 1; i < kSizeMax; i++)
      addr_[i] = 0;
  }
}

//------------------------------------------------------------------------------
// class Prefix CLASS API
//------------------------------------------------------------------------------
Prefix::Prefix(void) {
  family_ = kFamilyUnkown;
  length_ = 0;
}

Prefix::Prefix(const Prefix& other) {
  family_ = other.family_;
  length_ = other.length_;
  memcpy(addr_, other.addr_, kSizeMax);
}

Prefix::~Prefix(void) {
}

//------------------------------------------------------------------------------
// ACCESSORS
//------------------------------------------------------------------------------
bool Prefix::get_valid(void) const {
  if (family_ == kFamilyIPv4 or family_ == kFamilyIPv6)
    return (true);
  else
    return (false);
}

int Prefix::get_family(void) const {
  return (family_);
}

int Prefix::get_length(void) const {
  return (length_);
}

int Prefix::get_max_length(void) const {
  if (family_ == kFamilyIPv4)
    return (kLengthIPv4);
  else if (family_ == kFamilyIPv6)
    return (kLengthIPv6);
  else if (family_ == kFamilyUnkown)
    return (kLengthUnknown);
  else
    throw FlowBoxE("FamilyNotDef", __FILE__, __LINE__);
}

int Prefix::get_bit_at(int position) const {
  if (position > length_ or position <= 0) {
    std::stringstream err_msg;
    err_msg <<  "BitOutOfRange! Prefix::get_bit_at :: invalid position :: "
            << position << std::endl;
    throw FlowBoxE(err_msg.str(), __FILE__, __LINE__);
  }
  int byte = addr_[(position - 1) >> 3];
  int bit = (byte >> (7 - ((position - 1) & 0x07))) & 0x01;
  return (bit);
}

//-----------------------------------------------------------------------------
// FROM
//-----------------------------------------------------------------------------
void Prefix::from(const std::string& prefix_txt) {
  from(prefix_txt.c_str());
}

void Prefix::from(const char* prefix_txt) {

  char network_s[INET6_ADDRSTRLEN];
  const char* slash;
  // extract the 'addr family' and allocate the required memory
  family_ = addr_get_family(prefix_txt);

  // extract the '/'
  slash = strchr(prefix_txt, '/');
  if (slash == NULL) {
    throw FlowBoxE("InvalidPrefix!'/' not found!", __FILE__, __LINE__);
  }

  // extract prefix length_
  length_ = atoi(slash + 1);

  // make some sanity checks and extract the network ordered addr
  if (family_ == kFamilyIPv4) {

    if ((slash - prefix_txt) > INET_ADDRSTRLEN or length_ > 32 or length_ < 0) {
      throw FlowBoxE("InvalidPrefix", __FILE__, __LINE__);
    }

    strncpy(network_s, prefix_txt, slash - prefix_txt);
    network_s[slash - prefix_txt] = '\0';

    if (inet_pton(AF_INET, network_s, addr_) != 1)
      throw FlowBoxE("InvalidPrefix", __FILE__, __LINE__);

    addr_set_unused_bits_to_zero();
  } else if (family_ == kFamilyIPv6) {

    if ((slash - prefix_txt > INET6_ADDRSTRLEN) or length_ > 128 or length_ < 0)
      throw FlowBoxE("InvalidPrefix", __FILE__, __LINE__);

    strncpy(network_s, prefix_txt, slash - prefix_txt);
    network_s[slash - prefix_txt] = '\0';

    if (inet_pton(AF_INET6, network_s, addr_) != 1)
      throw FlowBoxE("InvalidPrefix", __FILE__, __LINE__);

    addr_set_unused_bits_to_zero();
  } else {
    throw FlowBoxE("InvalidPrefix", __FILE__, __LINE__);
  }
  return;
}

void Prefix::from_nb(const char* prefix_bin, int length, int family) {
  length_ = length;
  family_ = family;
  if (family_ == kFamilyIPv4) {
    memcpy(addr_, prefix_bin, kSizeIPv4);
    addr_set_unused_bits_to_zero();
  } else if (family_ == kFamilyIPv6) {
    memcpy(addr_, prefix_bin, kSizeIPv6);
    addr_set_unused_bits_to_zero();
  } else {
#ifdef PREFIX_DEBUG
    std::cout << "Prefix::from_nb(const Prefix& other) UNKNOWN ADDRESS"
        << std::endl;
#endif
    throw FlowBoxE("InvalidPrefix", __FILE__, __LINE__);
  }
}

// assumes a 16 byte address with zero padding --
// use this function only if you know what your are doing ...
void Prefix::from_nb_zero(const char* prefix_bin, int length, int family) {
  length_ = length;
  family_ = family;
  memcpy(addr_, prefix_bin, kSizeMax);
}

//-----------------------------------------------------------------------------
// TO
//-----------------------------------------------------------------------------
const std::string Prefix::to_s(void) const {
  std::stringstream tmp;
  char addr_s[INET6_ADDRSTRLEN];
  if (family_ == kFamilyIPv4) {
    if (inet_ntop(AF_INET, addr_, addr_s, INET6_ADDRSTRLEN) == NULL)
      throw FlowBoxE("InvalidPrefix", __FILE__, __LINE__);
  } else if (family_ == kFamilyIPv6) {
    if (inet_ntop(AF_INET6, addr_, addr_s, INET6_ADDRSTRLEN) == NULL)
      throw FlowBoxE("InvalidPrefix", __FILE__, __LINE__);
  } else {
    // FIXME(schadomi/asdaniel): Almost always, snprintf is better than strcpy
    strcpy(addr_s, "NOTDEF");
  }

  tmp << addr_s << "/" << length_;
  return (std::string(tmp.str()));
}

//------------------------------------------------------------------------------
// MODIFYER
//------------------------------------------------------------------------------
void Prefix::clear(void) {
  family_ = kFamilyUnkown;
  length_ = 0;
}

void Prefix::cut(int bits) {
  // cut the prefix to n bits
  if (bits > length_ or bits <= 0) {
    throw FlowBoxE("BitOutOfRange", __FILE__, __LINE__);
  }

  length_ = bits;
  addr_set_unused_bits_to_zero();
}

//------------------------------------------------------------------------------
// COMPARE
//------------------------------------------------------------------------------
int Prefix::common_bits(const Prefix& other) const {
  // How many bits have these prefixes in common?
  // THIS  [1000101000|111111#******************]
  // OTHER [1000101000|111010#100101************]

  // # = common length_ [0,32/128] --> how long is the shorter prefix?
  // | = common bits [0,32/128] --> how many bits have both prefixes in common?

  // we can only compare prefixes from the same family
  if (family_ != other.family_)
    throw FlowBoxE("DifferentFamily", __FILE__, __LINE__);

  int bits = 0;
  int common_length = (length_ <= other.length_) ? length_ : other.length_;
  assert(common_length >= 0 && common_length <= 128);

  // at least one of the prefixes has zero length (nothing in common)
  if (common_length == 0)
    return (0);

  // find the byte that differs
  int byte_idx_stop = (common_length - 1) >> 3;
  int byte_idx = 0;
  while (byte_idx <= byte_idx_stop && addr_[byte_idx] == other.addr_[byte_idx])
    byte_idx++;

  // no byte differs
  if (byte_idx > byte_idx_stop)
    return (common_length);

  // at least one bit is different ... but witch one is it?
  unsigned char difference = addr_[byte_idx] ^ other.addr_[byte_idx];
  int bit_idx = 0;

  while (bit_idx < 8 && (difference & (0x80 >> bit_idx)) == 0)
    bit_idx++;

  // common bits = index of changed bit
  bits = bit_idx + (byte_idx << 3);

  if (bits > common_length)
    return (common_length);

  return (bits);
}

bool Prefix::includes(const Prefix& other) const {
#ifdef PREFIX_DEBUG
  std::cout << "INCLUDES -- is the OTHER prefix included in THIS prefix? "
            << std::endl;
  std::cout << "INCLUDES -- THIS : " << to_s() << std::endl;
  std::cout << "INCLUDES -- OTHER: " << other.to_s() << std::endl;
#endif
  // Is the OTHER prefix included in THIS prefix ?
  // THIS  [1000101000111010100101************]
  // OTHER [1000101000111*********************]
  // --> true

  // 0. Family
  // we can only compare prefixes from the same family
  if (family_ != other.family_)
    throw FlowBoxE("DifferentFamily", __FILE__, __LINE__);

  // 1. Size
  // The OTHER prefix is longer than THIS prefix.
  // ... so it cant be a subpart of THIS prefix
  if (other.length_ > length_) {
#ifdef PREFIX_DEBUG
    std::cout << "INCLUDES -- FALSE (length_) " << std::endl;
#endif
    return (false);
  }

  // 2. Content
  // byte per byte compare
  // THIS  [1000101000111010000'**************]
  // OTHER [10001010001110111*****************]
  // --> false
  // THIS  [1000101000111011100'**************]
  // OTHER [10001010001110111*****************]
  // --> true

  // 2.a compare the bytes

  int byte_idx_stop = (other.length_ - 1) >> 3;
  int byte_idx = 0;

#ifdef PREFIX_DEBUG
  std::cout << "INCLUDES -- COMPARE 2.a first  FROM "<< byte_idx << " TO "
      << byte_idx_stop << " bytes" << std::endl;
#endif

  while (byte_idx <= byte_idx_stop && addr_[byte_idx] == other.addr_[byte_idx])
    byte_idx++;

  // some bytes are different
  if (byte_idx < byte_idx_stop) {
#ifdef PREFIX_DEBUG
    std::cout << "INCLUDES -- FALSE (Byte "<< byte_idx << " differes)"
        << std::endl;
#endif
    return (false);
  }

  // all bytes are similar
  if (byte_idx > byte_idx_stop) {
#ifdef PREFIX_DEBUG
    std::cout << "INCLUDES -- TRUE (all bytes)" << std::endl;
#endif
    return (true);
  }

  // 2.b compare the last byte
  // ... mask all bits that are irrelevant for the comparison
  unsigned char difference = addr_[byte_idx] ^ other.addr_[byte_idx];
  int bit_idx = 0;
  int bit_idx_stop = (other.length_ - 1) & 0x07;  // % 8;
  while (bit_idx <= bit_idx_stop && (difference & (0x80 >> bit_idx)) == 0)
    bit_idx++;

  if (bit_idx > bit_idx_stop) {
#ifdef PREFIX_DEBUG
    std::cout << "INCLUDES -- TRUE (some bytes + some bits)" << std::endl;
#endif
    return (true);
  }

#ifdef PREFIX_DEBUG
  std::cout << "INCLUDES -- FALSE (some equal bytes ... but some bits not)"
      << std::endl;
#endif
  return (false);
}

bool Prefix::eql(const Prefix& other) const {
  if (family_ != other.family_) {
    return (false);
  }
  if (length_ != other.length_) {
    return (false);
  }
  if (memcmp(addr_, other.addr_, kSizeMax) != 0) {
    return (false);
  }
  return (true);
}
