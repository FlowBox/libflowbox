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
 * @file   flow_container_pool.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @date   October, 2012
 * @brief  A resource pool of FlowsContainers
 *
 * By recycle unused FlowContainers we can reduced the amount of alloc and
 * free call. Further we can control the amount of memory that is consumed by
 * the processing chain.
 *
 */

#ifndef FLOW_BOX_INCLUDE_FLOW_CONTAINER_POOL_H_
#define FLOW_BOX_INCLUDE_FLOW_CONTAINER_POOL_H_

#include <semaphore.h>
#include <stack>
#include <string>
#include <iostream>
#include <cassert>

#include "flow_container.h"

class FlowContainerPool {
//------------------------------------------------------------------------------
// The class constants
//------------------------------------------------------------------------------
 public:
  // CAPACITY: How many flow containers should be added by default to the stack
  static const int kCapacityDefault = 100;

  // STATISTIC:
  static const int kStatPop = 0;
  static const int kStatPush = 1;
  static const int kStatMin = 2;
  static const int kStatMax = 3;
  static const int kStatUsed = 4;
  static const int kStatCapacity = 5;

//------------------------------------------------------------------------------
// The class member variables
//------------------------------------------------------------------------------
 private:
  static FlowContainerPool* instance_;  // the addr of the singleton
  std::stack<FlowContainer*> pool_;  // the available FlowContainers
  sem_t pool_sem_;  // a semaphore to protection the pool
  sem_t pool_available_sem_;  // a semaphore to implement a blocking wait
  int capacity_;  // the max numbers of FlowContainers (limit)

  // STATISTICS:
  int stat_pop_;  // number of pops calls
  int stat_push_;  // number of push calls
  int stat_min_;  // min number of FlowContainers stored in the stack
  int stat_max_;  // max number of FlowContainers stored in the stack

//------------------------------------------------------------------------------
// The class helper functions
//------------------------------------------------------------------------------
 public:
  static FlowContainerPool* instance();  // get the single instance

//------------------------------------------------------------------------------
// The class methods
//------------------------------------------------------------------------------
 public:
  // GET: returns a pointer to a FlowContainer (blocking call)
  FlowContainer* pop(void);

  // PUSH: add the pointer to the resource pool
  void push(FlowContainer* p);

  void set_capacity(int capacity);

  // STATISTICS:
  void stat_reset(void);
  int stat_pop(void) {
    return (stat_pop_);
  }

  int stat_push(void) {
    return (stat_push_);
  }

  int stat_min(void) {
    return (stat_min_);
  }

  int stat_max(void) {
    return (stat_max_);
  }

  int stat_used(void) {
    int available;
    sem_getvalue(&pool_available_sem_, &available);
    return (capacity_ - available);
  }

  int stat_capacity(void) {
    return (capacity_);
  }


 private:
  // keep Constructors private
  FlowContainerPool();
  FlowContainerPool(const FlowContainerPool& other);
  ~FlowContainerPool();

  class Guard {
   public:
    ~Guard();
  };
  friend class Guard;
};
#endif  // FLOW_BOX_INCLUDE_FLOW_CONTAINER_POOL_H_
