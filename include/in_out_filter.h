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
 * @file   in_out_filter.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @date   October, 2012
 * @brief  Filter transit or internal traffic.
 *
 * Depending on the measurement setup the flow sensors record traffic crossing
 * the network or stays within the network. For many measurement application
 * such traffic is not of interest. This module can be used to focus on in-out
 * and out-in Flows and filter the reset.
 */

#ifndef FLOW_BOX_INCLUDE_IN_OUT_FILTER_H_
#define FLOW_BOX_INCLUDE_IN_OUT_FILTER_H_

#include <semaphore.h>

#include <cassert>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <map>
#include <queue>
#include <vector>

#include "common.h"
#include "flow_container_pool.h"
#include "flow_container_buffer.h"
#include "prefix_to_int_map.h"

class InOutFilter {
 public:
  static const int kOhter;
  static const int kInside;
  static const int kUnknown;

  enum Policy {
    kTag = 0,
    kFilter = 1
  };

  class Configuration {
   public:
    static const int kStatisticDefaultInterval;
    static const Policy kPolicyDefault;

   private:
    FlowContainerBuffer* input_;
    FlowContainerBuffer* output_;
    uint64_t stat_interval_;
    std::vector<std::string> prefix_;
    Policy policy_;

   public:
    Configuration();
    void reset(void);

    FlowContainerBuffer* get_input(void) const;
    FlowContainerBuffer* get_output(void) const;
    uint64_t get_stat_interval(void) const;
    std::vector<std::string>::const_iterator get_prefix_begin(void) const;
    std::vector<std::string>::const_iterator get_prefix_end(void) const;
    Policy get_policy(void) const;

    void set_input(FlowContainerBuffer* input);
    void set_output(FlowContainerBuffer* output);
    void set_stat_interval(uint64_t interval);
    void add_prefix(std::string prefix);
    void set_policy(Policy policy);
  };

  class Observation {
   private:
    bool valid_;
    uint64_t time_s_;
    std::string message_;

   public:
    Observation();
    void reset();

    bool get_valid(void) const;
    uint64_t get_time_s(void) const;
    std::string get_message(void) const;

    void set_valid(bool valid);
    void set_time_s(uint64_t time_s);
    void set_message(const std::string& message);
  };

  class Statistics {
   private:
    bool valid_;
    uint64_t time_s_;
    uint64_t duration_s_;
    uint64_t flows_;
    uint64_t in_in_;
    uint64_t in_out_;
    uint64_t out_in_;
    uint64_t out_out_;

   public:
    Statistics();
    void reset(void);

    bool get_valid(void) const;
    uint64_t get_time_s(void) const;
    uint64_t get_duration_s(void) const;
    uint64_t get_flows(void) const;
    uint64_t get_in_in(void) const;
    uint64_t get_in_out(void) const;
    uint64_t get_out_in(void) const;
    uint64_t get_out_out(void) const;

    void set_valid(bool valid);
    void set_time_s(uint64_t time_s);
    void set_duration_s(uint64_t duration_s);

    void inc_flows(void);
    void inc_in_in(void);
    void inc_in_out(void);
    void inc_out_in(void);
    void inc_out_out(void);
  };

  // ## CLASS VARIABLES #######################################################
 private:
  // -- CONFIGURATION: INCOMING ----------------------------------------------
  sem_t conf_critical_section_sem_;
  std::queue<Configuration> conf_in_;
  bool conf_available_;
  PrefixToIntMap conf_prefix_inside_;

  // -- OBSERVATION: OUTGOING ------------------------------------------------
  sem_t obs_critical_section_sem_;
  sem_t obs_available_sem_;
  std::queue<Observation> obs_out_;

  // -- STATISTICS: OUTGOING--------------------------------------------------
  sem_t stat_critical_section_sem_;
  sem_t stat_available_sem_;
  std::queue<Statistics> stat_out_;

  Statistics stat_current_;
  uint64_t stat_interval_s_;
  uint64_t stat_export_next_s_;

  // -- DATA STREAM  ---------------------------------------------------------
  FlowContainerBuffer* data_input_;
  FlowContainerBuffer* data_output_;
  FlowContainerPool* data_container_pool_;

  // OTHERS ------------------------------------------------------------------
  Prefix addr4_, addr6_;
  Policy policy_;

  // ## CLASS FUNCTIONS ########################################################
 private:
  // -- CONFIGURATION: INCOMING ----------------------------------------------
  void conf_pop(void);
  void conf_add_prefix(std::string prefix);
  void conf_migrate(const Configuration& migrate);

  // -- OBSERVATION: OUTGOING ------------------------------------------------
  void obs_push(uint64_t now_s, const std::string& message);

  // -- STATISTICS: OUTGOING--------------------------------------------------
  void stat_export(uint64_t now_s);

  // -- DATA STREAM  ---------------------------------------------------------
  void data_process(FlowContainer::iterator flow, FlowContainer::iterator end);
  void process(FlowContainer::iterator& flow);

  // ## CLASS API ##############################################################
 public:
  InOutFilter();
  ~InOutFilter();

  // -- CONFIGURATION: INCOMING ----------------------------------------------
  void conf_push(Configuration);

  // -- OBSERVATION: OUTGOING ------------------------------------------------
  Observation obs_get(void);

  // -- STATISTICS: OUTGOING--------------------------------------------------
  Statistics stat_get(void);

  // -- DATA STREAM  ---------------------------------------------------------
  void data_process(void);
};

#endif  // FLOW_BOX_INCLUDE_IN_OUT_FILTER_H_
