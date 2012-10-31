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
 * @file   border_filter.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @date   October, 2012
 * @brief  Tag flows that cross the border of the network
 *
 * The BorderFilter class can be used to tag or filter all flows that are not
 * crossing a border interface of a network. This is useful to eliminate
 * redundant flows and focus on that actual analysis
 *
 */

#ifndef FLOW_BOX_INCLUDE_BORDER_FILTER_H__
#define FLOW_BOX_INCLUDE_BORDER_FILTER_H__

#include <semaphore.h>
#include <cassert>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <map>
#include <queue>
#include <vector>

#include "flow_container_pool.h"
#include "flow_container_buffer.h"

class BorderFilter {
 public:
  enum Purpose {
    kOther = 0,
    kBorder = 1
  };

  enum Policy {
    kTag = 0,
    kFilter = 1
  };

  struct Interface {
    uint16_t export_device_id_;
    uint16_t interface_id_;
    Purpose purpose_;
  };

  class Configuration {
   public:
    static const int kStatisticDefaultInterval;
    static const Policy kPolicyDefault;

   private:
    FlowContainerBuffer* input_;
    FlowContainerBuffer* output_;
    uint64_t stat_interval_;
    std::vector<Interface> interfaces_;
    Policy policy_;

   public:
    Configuration();
    void reset(void);

    FlowContainerBuffer* get_input(void) const;
    FlowContainerBuffer* get_output(void) const;
    uint64_t get_stat_interval(void) const;
    std::vector<Interface>::const_iterator get_interface_begin(void) const;
    std::vector<Interface>::const_iterator get_interface_end(void) const;
    Policy get_policy(void) const;

    void set_input(FlowContainerBuffer* input);
    void set_output(FlowContainerBuffer* output);
    void set_stat_interval(uint64_t interval);
    void add_interface(const Interface& interface);
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
    uint64_t other_;
    uint64_t border_in_;
    uint64_t border_out_;

   public:
    Statistics();
    void reset(void);

    bool get_valid(void) const;
    uint64_t get_time_s(void) const;
    uint64_t get_duration_s(void) const;
    uint64_t get_flows(void) const;
    uint64_t get_other(void) const;
    uint64_t get_border_in(void) const;
    uint64_t get_border_out(void) const;

    void set_valid(bool valid);
    void set_time_s(uint64_t time_s);
    void set_duration_s(uint64_t duration_s);

    void inc_flows(void);
    void inc_other(void);
    void inc_border_in(void);
    void inc_border_out(void);
  };

  // ## CLASS VARIABLES ########################################################
 private:
  // -- CONFIGURATION: INCOMING ----------------------------------------------
  sem_t conf_critical_section_sem_;
  std::queue<Configuration> conf_in_;
  bool conf_available_;

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
  // [export_device_id][interface_id] = purpose
  std::vector<std::vector<Purpose> > map_;
  Policy policy_;

  // ## CLASS FUNCTIONS ########################################################
 private:
  // -- CONFIGURATION: INCOMING ----------------------------------------------
  void conf_pop(void);
  void conf_add_interface(const Interface& interface);
  void conf_migrate(const Configuration& migrate);

  // -- OBSERVATION: OUTGOING ------------------------------------------------
  void obs_push(uint64_t now_s, const std::string& message);

  // -- STATISTICS: OUTGOING--------------------------------------------------
  void stat_export(uint64_t now_s);

  // -- DATA STREAM  ---------------------------------------------------------
  void data_process(FlowContainer::iterator flow, FlowContainer::iterator end);
  void process(const FlowContainer::iterator& flow);

  // ## CLASS API ##############################################################
 public:
  BorderFilter();
  ~BorderFilter();

  // -- CONFIGURATION: INCOMING ----------------------------------------------
  void conf_push(Configuration);

  // -- OBSERVATION: OUTGOING ------------------------------------------------
  Observation obs_get(void);

  // -- STATISTICS: OUTGOING--------------------------------------------------
  Statistics stat_get(void);

  // -- DATA STREAM  ---------------------------------------------------------
  void data_process(void);
};

#endif  // FLOW_BOX_INCLUDE_BORDER_FILTER_H__
