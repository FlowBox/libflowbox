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
 * @file   flow_container_pool.cc
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @date   October, 2012
 * @brief  A resource pool of FlowsContainers
 *
 * By recycle unused FlowContainers we can reduced the amount of alloc and
 * free call. Further we can control the amount of memory that is consumed by
 * the processing chain.
 *
 */

#include "flow_container_pool.h"

FlowContainerPool* FlowContainerPool::instance_ = NULL;
FlowContainerPool::~FlowContainerPool() {
  fb_sem_wait(&pool_sem_);
  // START critical section
  sem_init(&pool_available_sem_, 0, 0);

  while (!pool_.empty()) {
    FlowContainer* p = pool_.top();
    if (p != NULL) {
      delete p;
    }
    pool_.pop();
  }

  fb_sem_post(&pool_sem_);
  // END critical section
}

FlowContainerPool* FlowContainerPool::instance() {
  static Guard g;
  if (!instance_)
    instance_ = new FlowContainerPool();
  return instance_;
}

FlowContainerPool::FlowContainerPool() {
  sem_init(&pool_sem_, 0, 1);
  sem_init(&pool_available_sem_, 0, 0);
  capacity_ = kCapacityDefault;

  // update the pool
  fb_sem_wait(&pool_sem_);
  // START critical section

  for (int i = 0; i < capacity_; i++) {
    FlowContainer* p = new FlowContainer();
    pool_.push(p);
    fb_sem_post(&pool_available_sem_);
    // SIGNAL CONTAINER
  }
  fb_sem_post(&pool_sem_);
  // END critical section
  stat_reset();
}

void FlowContainerPool::set_capacity(int capacity) {
  while (capacity_ != capacity) {
    int n = capacity - capacity_;
    if (n < 0) {
      // decrement pool -- one by one ...
      FlowContainer* p = pop();
      delete p;
      fb_sem_wait(&pool_sem_);
      // START critical section
      capacity_--;
      fb_sem_post(&pool_sem_);
      // END critical section
    } else {
      // increment pool by n, in a single step
      fb_sem_wait(&pool_sem_);
      // START critical section
      for (int i = 0; i < n; i++) {
        FlowContainer* p = new FlowContainer();
        pool_.push(p);
        sem_post(&pool_available_sem_);
      };
      capacity_++;
      fb_sem_post(&pool_sem_);
      // END critical section
    }
  }
}

FlowContainer* FlowContainerPool::pop(void) {
  FlowContainer* p;

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
    throw FlowBoxE("logic broken", __FILE__, __LINE__);
  }

  // update min size
  if (static_cast<int>(pool_.size()) < stat_min_)
    stat_min_ = pool_.size();

  fb_sem_post(&pool_sem_);
  // END critical section

  stat_pop_++;

  return (p);
}

void FlowContainerPool::push(FlowContainer* p) {
  // ignore NULL pointers
  if (p == NULL) {
    // std::cout << "WARNING:: POOL PUSH receives NULL Pointer" << std::endl;
    return;
  }

  // reset
  p->reset();

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

void FlowContainerPool::stat_reset(void) {
  stat_pop_ = 0;
  stat_push_ = 0;
  stat_min_ = capacity_;
  stat_max_ = 0;
}

FlowContainerPool::Guard::~Guard() {
  if (NULL != FlowContainerPool::instance_) {
    delete FlowContainerPool::instance_;
    FlowContainerPool::instance_ = NULL;
  }
}

