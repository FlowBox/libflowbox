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
 * @file   noise_filter.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @author Daniel Aschwanden  <nimdanitro@gmail.com>
 *
 * @date   October, 2012
 * @brief  Simple Noise Filter based on packet counters
 *
 * The NoiseFilter filters flows with packets below a certain threshold
 * to reduce for example the impact of scanning or unsuccessful connections.
 * Currently, this filter works with Flows and BiFlows and is restricted
 * to TCP and UDP flows.
 *
 */

#ifndef FLOW_BOX_INCLUDE_NOISE_FILTER_H_
#define FLOW_BOX_INCLUDE_NOISE_FILTER_H_

#include <stdint.h>
#include <time.h>
#include <string>
#include <cstring>
#include <queue>

#include "common.h"
#include "flow.h"
#include "flow_container_pool.h"
#include "flow_container_buffer.h"
#include "bi_flow.h"
#include "bi_flow_pool.h"
#include "bi_flow_container.h"
#include "bi_flow_container_pool.h"
#include "bi_flow_container_buffer.h"

class NoiseFilter {
 public:
  //----------------------------------------------------
  // NoiseFilter::Statistics class
  //----------------------------------------------------
  class Statistics {
   private:
    bool valid_;
    // all the statistics are private
    // some stats of times
    uint64_t time_s_;
    uint64_t duration_s_;
    uint64_t flows_in_;
    uint64_t flows_valid_;
    uint64_t flows_kept_;
    uint64_t flows_filtered_;

   public:
    static const int kStatisticDefaultInterval;

    // we need getters and setters for all stats:
    bool valid(void) const;
    void set_valid(bool valid);

    uint64_t time_s(void) const;
    void set_time_s(uint64_t timeS);

    uint64_t duration_s(void) const;
    void set_duration_s(uint64_t duration);

    uint64_t flows_in(void) const;
    void set_flows_in(uint64_t flowsIn);
    void inc_flows_in(void);

    uint64_t flows_valid(void) const;
    void set_flows_valid(uint64_t flowsValid);
    void inc_flows_valid(void);

    uint64_t flows_kept(void) const;
    void set_flows_kept(uint64_t flowsKept);
    void inc_flows_kept(void);

    uint64_t flows_filtered(void) const;
    void set_flows_filtered(uint64_t flowsFiltered);
    void inc_flows_filtered(void);

    std::string head_to_s(void) const;
    std::string to_s(void) const;

    // Methods
    void reset(void);

    Statistics();
    ~Statistics();
  };

  //----------------------------------------------------
  // NoiseFilter::Configuration class
  //----------------------------------------------------
  class Configuration {
   public:
    static const int kTcpMinPacketsDefault;
    static const int kUdpMinPacketsDefault;

   private:
    FlowContainerBuffer* input_flow_;
    FlowContainerBuffer* output_flow_;

    BiFlowContainerBuffer* input_biflow_;
    BiFlowContainerBuffer* output_biflow_;
    BiFlowContainerBuffer* output_filtered_biflow_;

    int udp_packets_min_;
    int tcp_packets_min_;
    uint64_t stat_interval_;

   public:
    // Methods
    Configuration();
    ~Configuration();

    FlowContainerBuffer* flow_input(void) const;
    FlowContainerBuffer* flow_output(void) const;

    BiFlowContainerBuffer* biflow_input(void) const;
    BiFlowContainerBuffer* biflow_output(void) const;
    BiFlowContainerBuffer* biflow_output_filtered(void) const;

    void set_flow_input(FlowContainerBuffer* input);
    void set_flow_output(FlowContainerBuffer* output);

    void set_biflow_input(BiFlowContainerBuffer* input);
    void set_biflow_output(BiFlowContainerBuffer* output);
    void set_biflow_output_filtered(BiFlowContainerBuffer* output);

    int udp_packets_min(void) const;
    int tcp_packets_min(void) const;
    uint64_t stat_interval(void) const;
    void set_udp_packets_min(int num);
    void set_tcp_packets_min(int num);
    void set_stat_interval(uint64_t interval);

    void reset();
  };

  //----------------------------------------------------
  // NoiseFilter::Observation class
  //----------------------------------------------------
  class Observation {
   private:
    bool valid_;
    uint64_t time_s_;
    std::string message_;

   public:
    Observation();
    ~Observation();
    void reset();

    bool valid(void) const;
    uint64_t time_s(void) const;
    std::string message(void) const;

    void set_valid(bool valid);
    void set_time_s(uint64_t time_s);
    void set_message(const std::string& message);
  };

  //----------------------------------------------------
  // NoiseFilter Class
  //----------------------------------------------------
 private:
  // -- CONFIGURATION: INCOMING ------------------------
  sem_t conf_critical_section_sem_;
  std::queue<Configuration> conf_in_;
  bool conf_available_;

  // -- OBSERVATION: OUTGOING ---------------------------
  sem_t obs_critical_section_sem_;
  sem_t obs_available_sem_;
  std::queue<Observation> obs_out_;

  // -- STATISTICS: OUTGOING-----------------------------
  sem_t stat_critical_section_sem_;
  sem_t stat_available_sem_;
  std::queue<Statistics> stat_out_;

  Statistics stat_current_;
  uint64_t stat_interval_s_;
  uint64_t stat_export_next_s_;

  //----------------------------------------------------
  // Input and Output Buffers and Pool for flow containers
  FlowContainerBuffer* data_input_flow_;
  FlowContainerBuffer* data_output_flow_;

  FlowContainerPool* flow_container_pool_;

  BiFlowContainerBuffer* data_input_biflow_;
  BiFlowContainerBuffer* data_output_biflow_;
  BiFlowContainerBuffer* data_output_biflow_filtered_;

  BiFlowContainerPool* biflow_container_pool_;
  BiFlowPool& biflow_pool_;

  int udp_packets_min_;
  int tcp_packets_min_;

  // ## CLASS FUNCTIONS ##################################
  // -- CONFIGURATION: INCOMING --------------------------
  void conf_pop(void);
  void conf_migrate(const Configuration& migrate);

  // -- OBSERVATION: OUTGOING ----------------------------
  void obs_push(uint64_t now_s, const std::string& message);

  // -- STATISTICS: OUTGOING------------------------------
  void stat_export(uint64_t now_s);

  void process_flow_container(FlowContainer::iterator flow,
                              FlowContainer::iterator end);

  void process_biflow_container(BiFlowContainer* bfc, BiFlow* & kept,
                                BiFlow* & filtered);

 public:
  NoiseFilter(void);
  NoiseFilter(const NoiseFilter& other);
  NoiseFilter& operator=(const NoiseFilter& other);
  ~NoiseFilter(void);

  // -- CONFIGURATION: INCOMING --------------------------
  void conf_push(Configuration);

  // -- OBSERVATION: OUTGOING ----------------------------
  Observation obs_get(void);

  // -- STATISTICS: OUTGOING------------------------------
  Statistics stat_get(void);

  void data_process(void);
};

#endif  // FLOW_BOX_INCLUDE_NOISE_FILTER_H_
