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
 * @file   common.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @author Daniel Aschwanden  <nimdanitro@gmail.com>
 * @date   October, 2012
 * @brief  Contains some shared data structures or methods
 *
 * The common.h file contains shared data strucures or methedos such as
 * functions for common exception handling are hash functions.
 */

#ifndef FLOW_BOX_INCLUDE_COMMON_H_
#define FLOW_BOX_INCLUDE_COMMON_H_

#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include <stdint.h>
#include <tr1/unordered_map>

#include <queue>
#include <cerrno>
#include <cassert>
#include <exception>
#include <string>
#include <sstream>
#include <iostream>
#include <limits>

#ifndef GCC_VERSION
#define GCC_VERSION
#endif

#define hash_map std::tr1::unordered_map

// hide copy and assign operators
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&);               \
  void operator=(const TypeName&)

/// 64-Bit network to host byte order macro
#ifdef WORDS_BIGENDIAN
  #define ntohll(n) ((n))
#else
  #define ntohll(n) (((uint64_t)ntohl(n)) << 32) + ntohl((n) >> 32)
#endif

// 64-Bit host to network byte order macro
#ifdef WORDS_BIGENDIAN
  #define htonll(n) (n)
#else
  #define htonll(n) (((uint64_t)htonl(n)) << 32) + htonl((n) >> 32)
#endif

// HASH FUNCTIONS
#define my_hashmask(n) ( ( (uint32_t)1 << (n) ) - 1)
#define my_hashkey(h, n) ( h & my_hashmask(n) )
#define my_rot(x, k) ( ( (x) << (k) ) | ( (x) >> (32-(k)) ) )
#define my_mix(a, b, c) \
{ \
  a -= c;  a ^= my_rot(c,  4);  c += b; \
  b -= a;  b ^= my_rot(a,  6);  a += c; \
  c -= b;  c ^= my_rot(b,  8);  b += a; \
  a -= c;  a ^= my_rot(c, 16);  c += b; \
  b -= a;  b ^= my_rot(a, 19);  a += c; \
  c -= b;  c ^= my_rot(b,  4);  b += a; \
}
#define my_final(a, b, c) \
{ \
  c ^= b; c -= my_rot(b, 14); \
  a ^= c; a -= my_rot(c, 11); \
  b ^= a; b -= my_rot(a, 25); \
  c ^= b; c -= my_rot(b, 16); \
  a ^= c; a -= my_rot(c,  4); \
  b ^= a; b -= my_rot(a, 14); \
  c ^= b; c -= my_rot(b, 24); \
}

//------------------------------------------------------------------------------
// COMMON EXCEPTION
//------------------------------------------------------------------------------
// Usage:  throw FlowBoxE("TEST",__FILE__, __LINE__);
class FlowBoxE : public std::exception {
 private:
  std::string what_;
 public:
  FlowBoxE(const std::string& what, const std::string& file, int line) {
    std::stringstream tmp;
    tmp << what << " in '" << file << "' line :'" << line << "'";
    what_ = (tmp.str()).c_str();
  }

  virtual const char* what() const throw() {
    return what_.c_str();
  }

  ~FlowBoxE() throw() {
  }
};

inline int64_t get_difference(uint64_t first, uint64_t second) {
  uint64_t abs_diff = (first > second) ? (first - second) : (second - first);
  assert(abs_diff <= (uint64_t) std::numeric_limits<int64_t>::max());
  return (first > second) ? (int64_t) abs_diff : -((int64_t) abs_diff);
}

inline uint64_t get_abs_difference(uint64_t first, uint64_t second) {
  uint64_t abs_diff = (first > second) ? (first - second) : (second - first);
//    assert(abs_diff<=std::numeric_limits<int64_t>::max();
  return (abs_diff);
}


//------------------------------------------------------------------------------
// SEMAPHORE METHODS WITH ERROR HANDLING
//------------------------------------------------------------------------------
// Use this wrappers -- we are doing a lot in parallel and we need error msgs

#define fb_sem_wait(sem_p) \
{ \
  if ( sem_wait(sem_p) != 0 ) { \
    int n = errno; \
    printf("error code #%d: %s, (wait 5s) \n", n, strerror(n)); \
    sleep(5); \
    throw FlowBoxE("sem_wait failed", __FILE__, __LINE__); \
  }\
}

#define fb_sem_post(sem_p) \
{ \
  if (sem_post(sem_p) != 0) { \
    int n = errno; \
    printf("error code #%d: %s, (wait 5s) \n", n, strerror(n)); \
    sleep(5); \
    throw FlowBoxE("sem_post failed", __FILE__, __LINE__); \
  }\
}

#endif  // COMMON_H_
