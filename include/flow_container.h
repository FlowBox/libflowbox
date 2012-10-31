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
 * @file   flow_container.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @date   October, 2012
 * @brief  A data structure containing multiple Flows
 *
 * This class implements a container to forward Flows from units to units.
 */


#ifndef FLOW_BOX_INCLUDE_FLOW_CONTAINER_H_
#define FLOW_BOX_INCLUDE_FLOW_CONTAINER_H_
#include <cassert>
#include <vector>
#include <string>

#include "common.h"
#include "flow.h"

class FlowContainer {
//------------------------------------------------------------------------------
// The class constants
//------------------------------------------------------------------------------
 public:
  // CAPACITY: How many flows should be bounded into a container by default
  static const int kCapacityDefault = 30000;

 private:
  int capacity_;  // the max number of flows that can be stored by the container
  int used_;     // the number of 'used' flows data structure the container

  // the flows
  std::vector<Flow> flows_;

 public:
  typedef std::vector<Flow>::const_iterator const_iterator;
  typedef std::vector<Flow>::iterator iterator;

//------------------------------------------------------------------------------
 public:
  FlowContainer();

  // Getters and setters
  void resize(int size);
  void reset(void);
  int capacity(void);
  int used(void);
  int available(void);
  void update_used_by(int flows_to_add);

  // const iterator interface
  const_iterator begin() const;
  const_iterator end_used() const;
  const_iterator end() const;

  // iterator interface
  iterator begin();
  iterator end_used();
  iterator end();

 private:
  DISALLOW_COPY_AND_ASSIGN(FlowContainer);
};
#endif // FLOW_BOX_INCLUDE_FLOW_CONTAINER_H_
