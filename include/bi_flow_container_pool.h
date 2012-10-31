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
 * @file   bi_flow_container_pool.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @author Daniel Aschwanden  <nimdanitro@gmail.com>
 * @date   October, 2012
 * @brief  A resource pool of 'BiFlows Containers' adopted from flow
 *         container pool.
 *
 * This class implements a resource pool using a singleton pattern to
 * recycle unused bi_flow_containers. This allows us to reduce the number of
 * alloc and free operations.
 *
 */

#ifndef FLOW_BOX_INCLUDE_BI_FLOW_CONTAINER_POOL_H_
#define FLOW_BOX_INCLUDE_BI_FLOW_CONTAINER_POOL_H_

#include <semaphore.h>
#include <stack>
#include <string>
#include <cassert>

#include "bi_flow_pool.h"
#include "bi_flow_container.h"

class BiFlowContainerPool {
  //----------------------------------------------------------------------------
  // The class constants
  //----------------------------------------------------------------------------
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

  //----------------------------------------------------------------------------
  // The class member variables
  //----------------------------------------------------------------------------
 private:
  static BiFlowContainerPool* instance_;  // the addr of the singleton
  std::stack<BiFlowContainer*> pool_;  // the available BiFlowContainers
  sem_t pool_sem_;  // a semaphore to protection the pool
  sem_t pool_available_sem_;  // a semaphore to implement a blocking wait
  int capacity_;  // the max numbers of BiFlowContainers (limit)

  BiFlowPool& biflow_pool_;

  // STATISTICS:
  int stat_pop_;  // number of pops calls
  int stat_push_;  // number of push calls
  int stat_min_;  // min number of FlowContainers stored in the stack
  int stat_max_;  // max number of FlowContainers stored in the stack

  //----------------------------------------------------------------------------
  // The class helper functions
  //----------------------------------------------------------------------------
 public:
  static BiFlowContainerPool* instance();  // get the single instance

  //----------------------------------------------------------------------------
  // The class methods
  //----------------------------------------------------------------------------
 public:
  // GET: returns a pointer to a FlowContainer (blocking call)
  BiFlowContainer* pop(void);

  // PUSH: add the pointer to the resource pool
  void push(BiFlowContainer* p);

  void set_capacity(int capacity);

  // STATISTICS:
  void stat_reset(void);
  int stat_pop(void) const;
  int stat_push(void) const;
  int stat_min(void) const;
  int stat_max(void) const;
  int stat_used(void) const;
  int stat_capacity(void) const;

 private:
  // keep Constructors private
  BiFlowContainerPool();
  BiFlowContainerPool(const BiFlowContainerPool& other);
  ~BiFlowContainerPool();

  // Since this class is a singleton we have to make sure that it will get
  // deleted so use a guard class to delete the reference and free memory!
  class Guard {
   public:
    ~Guard();
  };
  friend class Guard;
};
#endif  // FLOW_BOX_INCLUDE_BI_FLOW_CONTAINER_POOL_H_
