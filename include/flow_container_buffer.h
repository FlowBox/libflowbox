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
 * @file   flow_container_buffer.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @date   October, 2012
 * @brief  A buffer containing multiple FlowContainers.
 *
 * The FlowContainerBuffer is used to interconnect to processing units.
 *
 */

#ifndef FLOW_BOX_INCLUDE_FLOW_CONTAINER_BUFFER_H_
#define FLOW_BOX_INCLUDE_FLOW_CONTAINER_BUFFER_H_

#include <semaphore.h>

#include <queue>
#include <iostream>
#include <ctime>
#include <cerrno>
#include <string>

#include "common.h"
#include "flow_container.h"

class FlowContainerBuffer {
//------------------------------------------------------------------------------
// The class constants
//------------------------------------------------------------------------------
 public:
  // CAPACITY:
  static const int kCapacity = 10;

  // STATE:
  static const int kStateRun = 0;
  static const int kStateFin = 1;

  // SEMAPHOREN
  static const int kWaitTimeoutS = 10;

//------------------------------------------------------------------------------
// The class member variables
//------------------------------------------------------------------------------
 private:
  std::queue<FlowContainer*> buffer_;
  sem_t sem_unused_;   // how many buffer slots are unused
  sem_t sem_used_;     // how many buffer slots are used
  sem_t sem_critical_section_;  // critical section
  int state_;

//------------------------------------------------------------------------------
// The class helper functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// The class methods
//------------------------------------------------------------------------------

 public:
  FlowContainerBuffer();

  // POP: get a FlowContainer from the buffer (blocking call)
  FlowContainer* pop(void);

  // PUSH: add the pointer to the resource pool (blocking call)
  void push(FlowContainer* p);

  // SIGNAL: send a signal to the buffer
  void signal_fin(void) {
    state_ = kStateFin;
  }

  bool is_state_fin(void) {
    return (state_ == kStateFin);
  }

  // STAT:
  int stat_queue_size(void) {
    int queue_size;
    sem_getvalue(&sem_used_, &queue_size);
    return (queue_size);
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(FlowContainerBuffer);
};
#endif  // FLOW_BOX_INCLUDE_FLOW_CONTAINER_BUFFER_H_
