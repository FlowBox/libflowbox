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

#include "bi_flow_container_pool.h"

BiFlowContainerPool* BiFlowContainerPool::instance_ = NULL;

BiFlowContainerPool::~BiFlowContainerPool() {
  fb_sem_wait(&pool_sem_);
  // START critical section
  sem_init(&pool_available_sem_, 0, 0);
  while (!pool_.empty()) {
    BiFlowContainer* p = pool_.top();
    if (p != NULL) {
      delete p;
    }
    pool_.pop();
  }
  fb_sem_post(&pool_sem_);
  // END critical section
}

BiFlowContainerPool* BiFlowContainerPool::instance() {
  static Guard g;
  if (!instance_)
    instance_ = new BiFlowContainerPool();
  return instance_;
}

BiFlowContainerPool::BiFlowContainerPool()
    : biflow_pool_(*BiFlowPool::instance()) {
  sem_init(&pool_sem_, 0, 1);
  sem_init(&pool_available_sem_, 0, 0);
  capacity_ = kCapacityDefault;

  // update the pool
  fb_sem_wait(&pool_sem_);
  // START critical section

  for (int i = 0; i < capacity_; i++) {
    BiFlowContainer* p = new BiFlowContainer();
    pool_.push(p);
    fb_sem_post(&pool_available_sem_);
    // SIGNAL CONTAINER
  }

  fb_sem_post(&pool_sem_);
  // END critical section
  stat_reset();
}

void BiFlowContainerPool::set_capacity(int capacity) {
  while (capacity_ != capacity) {
    int n = capacity - capacity_;
    if (n < 0) {
      // decrement pool -- one by one ...
      BiFlowContainer* p = pop();
      delete p;
      // START critical section
      fb_sem_wait(&pool_sem_);
      capacity_--;
      // END critical section
      fb_sem_post(&pool_sem_);
    } else {
      // increment pool by n, in a single step
      // START critical section
      fb_sem_wait(&pool_sem_);
      for (int i = 0; i < n; i++) {
        BiFlowContainer* p = new BiFlowContainer();
        pool_.push(p);
        sem_post(&pool_available_sem_);
      }

      capacity_++;
      // END critical section
      fb_sem_post(&pool_sem_);
    }
  }
}

BiFlowContainer* BiFlowContainerPool::pop(void) {
  BiFlowContainer* p;

  fb_sem_wait(&pool_available_sem_);
  // WAIT on CONTAINER
  fb_sem_wait(&pool_sem_);
  // START critical section

  if (!pool_.empty()) {
    p = pool_.top();
    pool_.pop();
  } else {
    // This should never happen ...
    assert(false);
    // we saved the pool capacity with a semaphore
    throw FlowBoxE("BiFlowContainerPool empty?!?!", __FILE__, __LINE__);
  }

  // update min size
  if (static_cast<int>(pool_.size()) < stat_min_)
    stat_min_ = pool_.size();

  fb_sem_post(&pool_sem_);
  // END critical section

  stat_pop_++;

  return (p);
}

void BiFlowContainerPool::push(BiFlowContainer* p) {
  // ignore NULL pointers
  if (p == NULL) {
    throw FlowBoxE("BiFlowContainerPool::push received NULL container!",
                   __FILE__, __LINE__);
    return;
  }

  // reset the BiFlowContainer...
  p->time_ = 0;
  // ..and make sure the containing biflows are deleted as well
  biflow_pool_.delete_biflows(p->biflows_);
  p->biflows_ = NULL;

  fb_sem_wait(&pool_sem_);
  // START critical section
  pool_.push(p);
  fb_sem_post(&pool_available_sem_);
  // SIGNAL CONTAINER

  // update max size
  if (static_cast<int>(pool_.size()) > stat_max_)
    stat_max_ = pool_.size();

  fb_sem_post(&pool_sem_);
  // END critical section
  stat_push_++;

  return;
}

void BiFlowContainerPool::stat_reset(void) {
  stat_pop_ = 0;
  stat_push_ = 0;
  stat_min_ = capacity_;
  stat_max_ = 0;
}

// guard of the singleton pattern!
BiFlowContainerPool::Guard::~Guard() {
  if ( NULL != BiFlowContainerPool::instance_ ) {
    delete BiFlowContainerPool::instance_;
    BiFlowContainerPool::instance_ = NULL;
  }
}
