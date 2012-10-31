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
 * @file   ip_filter.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @date   October, 2012
 * @brief  Filter out certain IP addresses ranges.
 *
 * The IPFilter can be used to filter out certain IP ranges such as bogus IP
 * space or PlanetLab host, that would affect the quality of the measurement.
 */

#ifndef FLOW_BOX_INCLUDE_IP_FILTER_H_
#define FLOW_BOX_INCLUDE_IP_FILTER_H_

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

class IPFilter {
 public:
  enum Policy {
    kNoMatch = 0,
    kKeep = 1,
    kFilter = 2
  };

  class Configuration {
   public:
    static const int kStatisticDefaultInterval;
    static const Policy KDefaultPolicy;

   private:
    FlowContainerBuffer* input_;
    FlowContainerBuffer* output_;
    uint64_t stat_interval_;
    std::vector<std::string> prefix_;
    IPFilter::Policy default_policy_;

   public:
    Configuration();
    void reset(void);

    FlowContainerBuffer* get_input(void) const;
    FlowContainerBuffer* get_output(void) const;
    uint64_t get_stat_interval(void) const;
    std::vector<std::string>::const_iterator get_prefix_begin(void) const;
    std::vector<std::string>::const_iterator get_prefix_end(void) const;
    Policy get_default_policy(void) const;

    void set_input(FlowContainerBuffer* input);
    void set_output(FlowContainerBuffer* output);
    void set_stat_interval(uint64_t interval);
    void set_default_policy(Policy policy);
    void add_prefix(std::string prefix);
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
    uint64_t flows_filtered_;

   public:
    Statistics();
    void reset(void);

    bool get_valid(void) const;
    uint64_t get_time_s(void) const;
    uint64_t get_duration_s(void) const;
    uint64_t get_flows(void) const;
    uint64_t get_flows_filtered(void) const;

    void set_valid(bool valid);
    void set_time_s(uint64_t time_s);
    void set_duration_s(uint64_t duration_s);

    void inc_flows(void);
    void inc_flows_filtered(void);
  };

  // ## CLASS VARIABLES #######################################################
 private:
  // -- CONFIGURATION: INCOMING ----------------------------------------------
  sem_t conf_critical_section_sem_;
  std::queue<Configuration> conf_in_;
  bool conf_available_;
  PrefixToIntMap prefixes_;

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
  Policy default_policy_;

  // ## CLASS FUNCTIONS ########################################################
 private:
  // -- CONFIGURATION: INCOMING ----------------------------------------------
  void conf_pop(void);
  void conf_add_prefix(std::string prefix);
  void conf_migrate(const Configuration& migrate);

  // -- OBSERVATION: OUTGOING ------------------------------------------------
  void obs_push(const uint64_t now_s, const std::string& message);

  // -- STATISTICS: OUTGOING--------------------------------------------------
  void stat_export(uint64_t now_s);

  // -- DATA STREAM  ---------------------------------------------------------
  void data_process(FlowContainer::iterator flow, FlowContainer::iterator end);
  void process(const FlowContainer::iterator& flow);

  // ## CLASS API ##############################################################
 public:
  IPFilter();
  ~IPFilter();

  // -- CONFIGURATION: INCOMING ----------------------------------------------
  void conf_push(Configuration);

  // -- OBSERVATION: OUTGOING ------------------------------------------------
  Observation obs_get(void);

  // -- STATISTICS: OUTGOING--------------------------------------------------
  Statistics stat_get(void);

  // -- DATA STREAM  ---------------------------------------------------------
  void data_process(void);
};

#endif  // FLOW_BOX_INCLUDE_IP_FILTER_H_
