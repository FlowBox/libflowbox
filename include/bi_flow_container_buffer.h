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
 * @file   bi_flow_container_buffer.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @author Daniel Aschwanden  <nimdanitro@gmail.com>
 * @date   October, 2012
 * @brief  A buffer containing multiple BiFlowContainers.
 *
 * This class implements a container that can be used to forward biflow pointer
 * lists to other processing units.
 *
 */

#ifndef FLOW_BOX_INCLUDE_BI_FLOW_CONTAINER_BUFFER_H_
#define FLOW_BOX_INCLUDE_BI_FLOW_CONTAINER_BUFFER_H_

#include <semaphore.h>
#include <stdint.h>

#include <queue>
#include <ctime>
#include <string>
#include <cstring>
#include <cerrno>

#include "common.h"
#include "bi_flow.h"
#include "bi_flow_container.h"


class BiFlowContainerBuffer {
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
// The class member variables---------------------------------------------------
 private:
  std::queue<BiFlowContainer*> buffer_;
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
  BiFlowContainerBuffer();

  // POP: get a FlowContainer from the buffer (blocking call)
  BiFlowContainer* pop(void);

  // PUSH: add the pointer to the resource pool (blocking call)
  void push(BiFlowContainer* p);

  // SIGNAL: send a signal to the buffer
  void signal_fin(void);
  bool is_state_fin(void);
  int stat_queue_size(void);

 private:
  DISALLOW_COPY_AND_ASSIGN(BiFlowContainerBuffer);
};
#endif  // FLOW_BOX_INCLUDE_BI_FLOW_CONTAINER_BUFFER_H_
