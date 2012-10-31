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
 * @file   prefix_to_int_map.cc
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

#include "prefix_to_int_map.h"
PrefixToIntMap::PrefixToIntMap(void) {
  value_not_found = 0;
}

/**
 * Inserts the prefix to the prefix mapping
 * @param prefix the prefix to add
 * @param value TODO (asdaniel)
 */
void PrefixToIntMap::insert(const Prefix& prefix, int value) {
  map.insert(prefix, value);
}

/**
 * find the best match prefix
 * @param prefix
 * @return (not_found or best match)
 */
int PrefixToIntMap::lookup(const Prefix& prefix) {
  const int* best_match = map.lookup(prefix);
  if (best_match == NULL)
    return (value_not_found);
  else
    return (*best_match);
}

void PrefixToIntMap::set_value_not_found(int not_found) {
  value_not_found = not_found;
}

int PrefixToIntMap::get_value_not_found(void) {
  return (value_not_found);
}

