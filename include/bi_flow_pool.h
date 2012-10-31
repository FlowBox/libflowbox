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

#ifndef FLOW_BOX_INCLUDE_BI_FLOW_POOL_H_
#define FLOW_BOX_INCLUDE_BI_FLOW_POOL_H_

#include <semaphore.h>
#include <stdint.h>
#include <string>
#include <cstring>

#include "bi_flow.h"

class BiFlowPool {
 public:
  //----------------------------------------------------
  // BiFlowPool::Statistics class
  //----------------------------------------------------
  class Statistics {
   public:
    // Getters Methods are public
    uint64_t biflows_available() const;
    uint64_t biflows_total() const;
    uint64_t biflows_deleted() const;
    uint64_t biflows_new() const;

	// Resets the statistics
	void reset(void);

	// output to string
    std::string to_s(void) const;
    std::string head_to_s(void) const;

    Statistics();
    ~Statistics();

   private:
    // biflows total and available have to be static in
    // since they are unique per pool
    static uint64_t biflows_total_;
    static uint64_t biflows_available_;

    // the new and deleted are counters which have to be subtracted
    static uint64_t biflows_new_counter_;
    static uint64_t biflows_deleted_counter_;

    // the last value of new and deleted... to calculate the relative values!
    uint64_t biflows_new_;
    uint64_t biflows_deleted_;

    // only the Pool is able to set the statistics!
    friend class BiFlowPool;
  };

 public:
  //----------------------------------------------------
  // BiFlowPool class
  //----------------------------------------------------
  static BiFlowPool* instance();
  BiFlow* new_biflow(void);
  void delete_biflows(BiFlow* biflow);
  void delete_biflow(BiFlow* biflow);

 private:
  sem_t pool_sem_;  // a semaphore to protection the pool
  static BiFlowPool* instance_;
  BiFlow* pool_;

  BiFlowPool();
  BiFlowPool(const BiFlowPool&);
  ~BiFlowPool();

  class Guard {
   public:
    ~Guard();
  };
  friend class Guard;
};

#endif  // FLOW_BOX_INCLUDE_BI_FLOW_POOL_H_
