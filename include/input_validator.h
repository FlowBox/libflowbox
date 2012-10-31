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
 * @file   input_validator.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @date   October, 2012
 * @brief  Check if the data contains unrealistic or insane Flow Records.
 *
 * The world is not perfect. This unit tries to spot flows that are caused by
 * the imperfection of the measurement system such as zero byte flows.
 */

#ifndef FLOW_BOX_INCLUDE_INPUT_VALIDATOR_H_
#define FLOW_BOX_INCLUDE_INPUT_VALIDATOR_H_

#include <semaphore.h>
#include <cassert>
#include <cstring>
#include <string>
#include <queue>

#include "common.h"
#include "flow_container_pool.h"
#include "flow_container_buffer.h"

class InputValidator {
 public:
  class Configuration {
   public:
    static const int kStatIntervalDefault;
    static const int kSlidingWindowDefault;

   private:
    FlowContainerBuffer* input_;
    FlowContainerBuffer* output_;
    uint64_t stat_interval_s_;
    uint64_t sliding_window_s_;

   public:
    Configuration();
    void reset(void);

    FlowContainerBuffer* get_input(void) const;
    FlowContainerBuffer* get_output(void) const;
    uint64_t get_stat_interval_s() const;
    uint64_t get_sliding_window_s() const;

    void set_input(FlowContainerBuffer* input);
    void set_output(FlowContainerBuffer* output);
    void set_stat_interval(uint64_t stat_interval_s);
    void set_sliding_window_s(uint64_t sliding_window_s);
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
    bool valid_;

    uint64_t time_s_;
    uint64_t duration_s_;
    uint64_t flows_;

    uint64_t iv_time_freshness_;
    uint64_t iv_time_causality_;
    uint64_t iv_bytes_;
    uint64_t iv_packets_;
    uint64_t iv_mtu_;

   public:
    Statistics();
    void reset(void);

    bool get_valid(void) const;
    uint64_t get_time_s(void) const;
    uint64_t get_duration_s(void) const;
    uint64_t get_flows(void) const;
    uint64_t get_iv_time_freshness(void) const;
    uint64_t get_iv_time_causality(void) const;
    uint64_t get_iv_bytes(void) const;
    uint64_t get_iv_packets(void) const;
    uint64_t get_iv_mtu(void) const;

    void set_valid(bool valid);
    void set_time_s(uint64_t time_s);
    void set_duration_s(uint64_t duration_s);

    void inc_flows(void);
    void inc_iv_time_freshness(void);
    void inc_iv_time_causality(void);
    void inc_iv_bytes(void);
    void inc_iv_packets(void);
    void inc_iv_mtu(void);

    std::string head_to_s(void) const;
    std::string to_s(void) const;
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

  // IV STATE
  uint64_t flow_time_s_;
  uint64_t flow_time_last_jump_s_;
  int sliding_window_s_;

  // ## CLASS FUNCTIONS ########################################################
 private:
  // -- CONFIGURATION: INCOMING ----------------------------------------------
  void conf_pop(void);
  void conf_migrate(const Configuration& migrate);

  // -- OBSERVATION: OUTGOING ------------------------------------------------
  void obs_push(const std::string& message);

  // -- STATISTICS: OUTGOING--------------------------------------------------
  void stat_export(uint64_t now_s);

  // -- DATA STREAM  ---------------------------------------------------------
  void data_process(FlowContainer::iterator flow, FlowContainer::iterator end);
  void data_process(const FlowContainer::iterator& flow);

  // -- OTHER ----------------------------------------------------------------
  void report(std::string reason, const FlowContainer::iterator& flow);

  // ## CLASS API ##############################################################
 public:
  InputValidator();
  ~InputValidator();

  // -- CONFIGURATION: INCOMING ----------------------------------------------
  void conf_push(Configuration);

  // -- OBSERVATION: OUTGOING ------------------------------------------------
  Observation obs_get(void);

  // -- STATISTICS: OUTGOING--------------------------------------------------
  Statistics stat_get(void);

  // -- DATA STREAM  ---------------------------------------------------------
  void data_process(void);

 private:
  DISALLOW_COPY_AND_ASSIGN(InputValidator);
};

#endif  // FLOW_BOX_INCLUDE_INPUT_VALIDATOR_H_
