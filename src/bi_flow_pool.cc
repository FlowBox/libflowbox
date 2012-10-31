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
 * @file   bi_flow_pool.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @author Daniel Aschwanden  <nimdanitro@gmail.com>
 * @date   October, 2012
 * @brief  A resource pool for BiFlows
 *
 * This class implements a resource pool using a singleton pattern to
 * recycle unused BiFlows. This allows us to reduce the number of
 * alloc and free operations.
 *
 */

#include "bi_flow_pool.h"

//----------------------------------------------------
// BiFlowPool::Statistics class
//----------------------------------------------------
BiFlowPool::Statistics::Statistics() {
  biflows_new_ = 0;
  biflows_deleted_ = 0;
}

BiFlowPool::Statistics::~Statistics() {
}

uint64_t BiFlowPool::Statistics::biflows_available_ = 0;
uint64_t BiFlowPool::Statistics::biflows_total_ = 0;
uint64_t BiFlowPool::Statistics::biflows_deleted_counter_ = 0;
uint64_t BiFlowPool::Statistics::biflows_new_counter_ = 0;

void BiFlowPool::Statistics::reset(void) {
  // don't zero the total and the available statistics!
  // set the new and deleted to the current counter values..
  biflows_new_ = biflows_new_counter_;
  biflows_deleted_ = biflows_deleted_counter_;
}

uint64_t BiFlowPool::Statistics::biflows_available() const {
  return biflows_available_;
}

uint64_t BiFlowPool::Statistics::biflows_new() const {
  return biflows_new_counter_ - biflows_new_;
}

uint64_t BiFlowPool::Statistics::biflows_total() const {
  return biflows_total_;
}

uint64_t BiFlowPool::Statistics::biflows_deleted() const {
  return biflows_deleted_counter_ - biflows_deleted_;
}

std::string BiFlowPool::Statistics::to_s(void) const {
  std::stringstream stat;
  stat << biflows_total_ << ", ";
  stat << biflows_available_ << ", ";
  stat << biflows_new_counter_ - biflows_new_ << ", ";
  stat << biflows_deleted_counter_ - biflows_deleted_;
  return stat.str();
}

std::string BiFlowPool::Statistics::head_to_s(void) const {
  std::stringstream head;
  head << "pool biflows total_" << ", ";
  head << "pool biflows available" << ", ";
  head << "pool biflows new" << ", ";
  head << "pool biflows deleted";
  return head.str();
}

//----------------------------------------------------
// BiFlowPool class
//----------------------------------------------------
BiFlowPool* BiFlowPool::instance_ = NULL;

BiFlowPool* BiFlowPool::instance() {
  static Guard g;
  if (!instance_)
    instance_ = new BiFlowPool();
  return instance_;
}

BiFlowPool::BiFlowPool() {
  sem_init(&pool_sem_, 0, 1);
  fb_sem_wait(&pool_sem_);
  // START critical section
  pool_ = NULL;
  BiFlowPool::Statistics stats_current_;
  fb_sem_post(&pool_sem_);
  // END critical section
}

// Destructor: clean up all flows in pool!
BiFlowPool::~BiFlowPool() {
  BiFlow* next = NULL;
  fb_sem_wait(&pool_sem_);
  // START critical section
  while (pool_ != NULL) {
    next = pool_->next_;
    delete pool_;
    pool_ = next;
  }
  fb_sem_post(&pool_sem_);
  // END critical section
}

BiFlow* BiFlowPool::new_biflow(void) {
  BiFlow* biflow;
  // check if we have some biflow containers in the pool

  fb_sem_wait(&pool_sem_);
  // START critical section
  if (pool_ == NULL) {
    // we don't have any biflow left, so create one!
    biflow = new BiFlow();
    Statistics::biflows_total_++;
    Statistics::biflows_new_counter_++;
  } else {
    // return the top pool biflow..
    biflow = pool_;
    pool_ = biflow->next_;
    biflow->next_ = NULL;
    Statistics::biflows_available_--;
    Statistics::biflows_new_counter_++;
  }
  fb_sem_post(&pool_sem_);
  // END critical section
  return biflow;
}

void BiFlowPool::delete_biflows(BiFlow* biflow) {
  if (biflow == NULL)
    return;

  fb_sem_wait(&pool_sem_);
  // START critical section
  uint64_t available = 0;
  uint64_t deleted = 0;
  BiFlow* start = biflow;

  while (biflow->next_ != NULL) {
    biflow = biflow->next_;
    // increment the statistics..
    available++;
    deleted++;
  }
  biflow->next_ = pool_;
  pool_ = start;
  Statistics::biflows_available_ += available;
  Statistics::biflows_deleted_counter_ += deleted;
  fb_sem_post(&pool_sem_);
  // END critical section
  return;
}

// deletes just one element, and puts it back to pool
void BiFlowPool::delete_biflow(BiFlow* biflow) {
  fb_sem_wait(&pool_sem_);
  // START critical section
  if (biflow == NULL)
    return;
  biflow->next_ = pool_;
  pool_ = biflow;
  Statistics::biflows_available_++;
  Statistics::biflows_deleted_counter_++;
  fb_sem_post(&pool_sem_);
  // END critical section
  return;
}

// Guard of Singleton pattern
BiFlowPool::Guard::~Guard() {
  if (BiFlowPool::instance_ != NULL) {
    delete BiFlowPool::instance_;
    BiFlowPool::instance_ = NULL;
  }
}

