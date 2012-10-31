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
 * @file   prefix_to_int_map.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 *
 * @date   October, 2012
 * @brief  A longest common prefix lookup table storing a single integer.
 *
 * A longest common prefix lookup table storing a single integer. This kind of
 * lookup table is often required, for example to store the AS number for a
 * network prefix.
 *
 */

#ifndef FLOW_BOX_INCLUDE_PREFIX_TO_INT_H_
#define FLOW_BOX_INCLUDE_PREFIX_TO_INT_H_

// own stuff
#include "common.h"
#include "prefix_mapping.h"

class PrefixToIntMap {
  // adapter pattern
  // --> delegation ...
 private:
  PrefixMapping<int> map;
  int value_not_found;

 public:
  PrefixToIntMap(void);
  void insert(const Prefix& prefix, int value);
  int lookup(const Prefix& prefix);
  void set_value_not_found(int not_found);
  int get_value_not_found(void);
  inline void clear() {
    map.clear();
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(PrefixToIntMap);
};

#endif  // FLOW_BOX_INCLUDE_PREFIX_TO_INT_H_
