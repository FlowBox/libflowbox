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
 * @file   flow_container.cc
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @date   October, 2012
 * @brief  A data structure containing multiple Flows
 *
 * This class implements a container to forward Flows from units to units.
 */

#include "flow_container.h"

FlowContainer::FlowContainer()
    : flows_(kCapacityDefault) {
  capacity_ = kCapacityDefault;
  used_ = 0;
}

void FlowContainer::resize(int size) {
  flows_.resize(size);
  capacity_ = flows_.capacity();
  used_ = 0;
}
void FlowContainer::reset(void) {
  used_ = 0;
}

int FlowContainer::capacity(void) {
  return (capacity_);
}

int FlowContainer::used(void) {
  return (used_);
}

int FlowContainer::available(void) {
  return (capacity_ - used_);
}

void FlowContainer::update_used_by(int flows_to_add) {
  assert(used_ + flows_to_add >= 0);
  assert(used_ + flows_to_add <= capacity_);
  used_ += flows_to_add;
}

// const iterator interface
FlowContainer::const_iterator FlowContainer::begin() const {
  return (flows_.begin());
}

FlowContainer::const_iterator FlowContainer::end_used() const {
  return (flows_.begin() + used_);
}

FlowContainer::const_iterator FlowContainer::end() const {
  return (flows_.end());
}

// iterator interface
FlowContainer::iterator FlowContainer::begin() {
  return (flows_.begin());
}

FlowContainer::iterator FlowContainer::end_used() {
  return (flows_.begin() + used_);
}

FlowContainer::iterator FlowContainer::end() {
  return (flows_.end());
}
