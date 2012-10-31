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
 * @file   fan_out_filter.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @author Daniel Aschwanden  <nimdanitro@gmail.com>
 *
 * @date   October, 2012
 * @brief  Eliminate noise caused by hosts scanning the network.
 *
 * The NoiseFilter identifies flows that are caused by hosts scanning the
 * and removes does from the data stream. To do so, we relay on the fan out
 * property of network scanners.
 *
 */

#ifndef FLOW_BOX_INCLUDE_FAN_OUT_FILTER_H_
#define FLOW_BOX_INCLUDE_FAN_OUT_FILTER_H_

#include <semaphore.h>
#include <cassert>
#include <string>
#include <cstring>
#include <queue>

#include "common.h"
#include "bi_flow.h"
#include "bi_flow_pool.h"
#include "bi_flow_container.h"
#include "bi_flow_container_buffer.h"
#include "bi_flow_container_pool.h"


class FanOutFilter {
 public:
  class Configuration {
   public:
    static const int kStatIntervalDefault;
    static const int kInBadMinDefault;
    static const int kInRatioMinDefault;
    static const int kOutBadMinDefault;
    static const int kOutRatioMinDefault;
    static const int kAllInMinHTDefault;
    static const int kAllOutMinHTDefault;
    static const int kSelectionInMinHTDefault;
    static const int kSelectionOutMinHTDefault;

   private:
    BiFlowContainerBuffer* input_;
    BiFlowContainerBuffer* output_good_;
    BiFlowContainerBuffer* output_bad_;

    uint64_t stat_interval_s_;

    int in_bad_min_;
    int in_ratio_min_;
    int out_bad_min_;
    int out_ratio_min_;

    int all_in_ht_min_;
    int all_out_ht_min_;
    int selection_in_ht_min_;
    int selection_out_ht_min_;

   public:
    Configuration();
    void reset(void);

    BiFlowContainerBuffer* input(void) const;
    BiFlowContainerBuffer* output_good(void) const;
    BiFlowContainerBuffer* output_bad(void) const;

    uint64_t stat_interval_s() const;
    int in_bad_min() const;
    int in_ratio_min() const;
    int out_bad_min() const;
    int out_ratio_min() const;

    void set_input(BiFlowContainerBuffer* input);
    void set_output_good(BiFlowContainerBuffer* output);
    void set_output_bad(BiFlowContainerBuffer* output);

    void set_stat_interval(uint64_t stat_interval_s);

    void set_in_bad_min(int in_bad_min);
    void set_in_ratio_min(int in_ratio_min);
    void set_out_bad_min(int out_bad_min);
    void set_out_ratio_min(int out_ratio_min);
  };

  class Observation {
   private:
    bool valid_;
    uint64_t time_s_;
    std::string message_;

   public:
    Observation();
    void reset();

    bool valid(void) const;
    uint64_t time_s(void) const;
    std::string message(void) const;

    void set_valid(bool valid);
    void set_time_s(uint64_t time_s);
    void set_message(const std::string& message);
  };

  class Statistics {
   private:
    bool valid_;

    uint64_t time_s_;
    uint64_t duration_s_;
    uint64_t duration_all_build_;
    uint64_t duration_selection_build_;
    uint64_t duration_split_;

    uint64_t biflows_in_;
    uint64_t biflows_kept_;
    uint64_t biflows_filtered_;
    uint64_t all_in_;
    uint64_t all_out_;
    uint64_t selection_in_;
    uint64_t selection_out_;
    uint64_t hosts_in_;
    uint64_t hosts_out_;

    uint64_t buckets_all_in_;
    uint64_t buckets_all_out_;
    uint64_t buckets_selection_in_;
    uint64_t buckets_selection_out_;

   public:
    Statistics();
    void reset(void);

    // getters!
    bool valid(void) const;
    uint64_t time_s(void) const;
    uint64_t duration_s(void) const;
    uint64_t duration_all_build(void) const;
    uint64_t duration_selection_build(void) const;
    uint64_t duration_split(void) const;
    uint64_t biflows_in(void) const;
    uint64_t biflows_kept(void) const;
    uint64_t biflows_filtered(void) const;
    uint64_t all_in(void) const;
    uint64_t all_out(void) const;
    uint64_t selection_in(void) const;
    uint64_t selection_out(void) const;
    uint64_t hosts_in(void) const;
    uint64_t hosts_out(void) const;

    uint64_t buckets_all_in(void) const;
    uint64_t buckets_all_out(void) const;
    uint64_t buckets_selection_in(void) const;
    uint64_t buckets_selection_out(void) const;

    // Setters
    void set_valid(bool valid);
    void set_time_s(uint64_t time_s);
    void set_duration_s(uint64_t duration_s);
    void set_duration_all_build(uint64_t duration);
    void set_duration_selection_build(uint64_t duration);
    void set_duration_split(uint64_t duration);
    void set_biflows_in(uint64_t biflows);
    void set_biflows_kept(uint64_t biflows);
    void set_biflows_filtered(uint64_t biflows);
    void set_all_in(uint64_t all_in);
    void set_all_out(uint64_t all_out);
    void set_selection_out(uint64_t selection_out);
    void set_selection_in(uint64_t selection_in);
    void set_hosts_in(uint64_t hosts_in);
    void set_hosts_out(uint64_t hosts_out);
    void set_buckets_all_in(uint64_t buckets);
    void set_buckets_all_out(uint64_t buckets);
    void set_buckets_selection_in(uint64_t buckets);
    void set_buckets_selection_out(uint64_t buckets);

    // Statistic to string function
    std::string head_to_s(void) const;
    std::string to_s(void) const;
  };

  // ## CLASS VARIABLES ########################################################
 private:
  // a light data structure, used over all traffic
  class All {
   public:
    int out_;
    int bi_;
    All();
  };
  typedef hash_map<BiFlow::Key1l, All, BiFlow::Key1l, BiFlow::Key1l> AllHT;

  // a fat data structure, used over a part of the traffic
  typedef hash_map<BiFlow::Key3l, int, BiFlow::Key3l, BiFlow::Key3l> DegreeHT;

  // Selection data Structure
  class Selection {
  public:
    int out_, bi_;
    DegreeHT degree_out_, degree_bi_;
    Selection();
    ~Selection();
    void add(const BiFlow& biflow, BiFlow::Key1l::Direction direction);
    std::string to_s() const;
  };
  typedef hash_map<BiFlow::Key1l, Selection, BiFlow::Key1l, BiFlow::Key1l> SelectionHT;

  int in_bad_min_;
  int in_ratio_min_;
  int out_bad_min_;
  int out_ratio_min_;

  // keep the hash tables (so we don't have to shrink them
  AllHT all_in_ht_, all_out_ht_;
  SelectionHT selection_in_ht_, selection_out_ht_;

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
  BiFlowContainerBuffer* data_input_;
  BiFlowContainerBuffer* data_output_good_;
  BiFlowContainerBuffer* data_output_bad_;

  BiFlowPool* biflow_pool_;
  BiFlowContainerPool* biflow_container_pool_;

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
  void data_process(BiFlow* p_in, BiFlow* & p_out_good,
      BiFlow* & p_out_bad, uint64_t time);

  void all_build(BiFlow* p);
  void all_shrink(AllHT* AllHT, int bad_min, int ratio_min);
  void selection_build(BiFlow* p, BiFlow* & good, BiFlow* & bad);
  void selection_shrink(SelectionHT* SelectionHT, int bad_min, int ratio_min);
  void split(BiFlow* p, BiFlow* & p_good, BiFlow* & p_bad);

  // ## CLASS API ##############################################################
 public:
  FanOutFilter();
  ~FanOutFilter();

  // -- CONFIGURATION: INCOMING ----------------------------------------------
  void conf_push(Configuration);

  // -- OBSERVATION: OUTGOING ------------------------------------------------
  Observation obs_get(void);

  // -- STATISTICS: OUTGOING--------------------------------------------------
  Statistics stat_get(void);

  // -- DATA STREAM  ---------------------------------------------------------
  void data_process(void);

 private:
  DISALLOW_COPY_AND_ASSIGN(FanOutFilter);
};
#endif  // FLOW_BOX_INCLUDE_FAN_OUT_FILTER_H_
