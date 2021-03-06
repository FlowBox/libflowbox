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
 * @file   flow_container_buffer.cc
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @date   October, 2012
 * @brief  A buffer containing multiple FlowContainers.
 *
 * The FlowContainerBuffer is used to interconnect to processing units.
 *
 */

#include "flow_container_buffer.h"

FlowContainerBuffer::FlowContainerBuffer()
    : state_(kStateRun) {
  sem_init(&sem_unused_, 0, kCapacity);
  sem_init(&sem_used_, 0, 0);
  sem_init(&sem_critical_section_, 0, 1);
}


// POP: get a FlowContainer from the buffer (blocking call)
FlowContainer* FlowContainerBuffer::pop(void) {
  // check if it's FIN STATE time..
  if (state_ == kStateFin and stat_queue_size() == 0) {
    return (NULL);
  }

  FlowContainer* p = NULL;

  while (true) {
    // wait kWaitTimeout for new data
    struct timespec wait_time;
    wait_time.tv_sec = time(NULL) + kWaitTimeoutS;
    wait_time.tv_nsec = 0;

    // lets wait ...
    int err = sem_timedwait(&sem_used_, &wait_time);
    if (err == 0) {
      break;  // we have the token ... move on
    } else if (errno == ETIMEDOUT) {
      if (state_ == kStateFin and stat_queue_size() == 0) {
        // its over
        return (NULL);
      }
    } else {
      throw FlowBoxE("Unknown Error state??", __FILE__, __LINE__);
    }
  }

  sem_wait(&sem_critical_section_);

  p = buffer_.front();
  buffer_.pop();

  sem_post(&sem_critical_section_);
  sem_post(&sem_unused_);
  return (p);
}

// PUSH: add the pointer to the resource pool (blocking call)
void FlowContainerBuffer::push(FlowContainer* p) {
  assert(p != NULL);
  // we will refuse to work on NULL
  if (p == NULL)  // we don't store trash;
    return;

  sem_wait(&sem_unused_);
  sem_wait(&sem_critical_section_);

  buffer_.push(p);

  sem_post(&sem_critical_section_);
  sem_post(&sem_used_);
}

