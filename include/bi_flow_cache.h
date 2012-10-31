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
 * @file   bi_flow_cache.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @author Daniel Aschwanden  <nimdanitro@gmail.com>
 * @date   October, 2012
 * @brief  A hashmap based cache used to merge flows into biflow.
 *
 * This class implements a hashmap based cache that can be used to merge
 * unidirectional flows into biflows. Be aware of the fact that the BiFlowCache
 * requires quite some local memory.
 *
 */

#ifndef FLOW_BOX_INCLUDE_BI_FLOW_CACHE_H_
#define FLOW_BOX_INCLUDE_BI_FLOW_CACHE_H_

#include <stdint.h>
#include <time.h>
#include <string>
#include <cstring>
#include <queue>

#include "common.h"
#include "bi_flow.h"
#include "bi_flow_pool.h"
#include "bi_flow_container_pool.h"
#include "bi_flow_container_buffer.h"
#include "flow_container_pool.h"
#include "flow_container_buffer.h"

class BiFlowCache {
 public:
  enum ImportPolicy {
    kImportPolicyAddAll = 0,
    kImportPolicyAddBorderOnly = 1,
    kImportPolicyAddOutBorderInAll = 2
  };

  // Definition of the BiFlowCache HashTable:
  typedef hash_map<BiFlow::Key5l, BiFlow*, BiFlow::Key5l, BiFlow::Key5l>BiFlowHT;

  //----------------------------------------------------
  // BiFlowCache::Statistics class
  //----------------------------------------------------
  class Statistics {
  private:
    BiFlowPool::Statistics pool_stats_;

    bool valid_;
    // all the statistics are private
    // some stats of times
    uint64_t time_s_;
    uint64_t duration_add_;
    uint64_t duration_prune_;

    // some stats of the flows and biflows cached
    uint64_t flows_in_;
    uint64_t flows_valid_;
    uint64_t biflows_new_;
    uint64_t biflows_cached_;

    uint64_t prune_too_young_;
    uint64_t prune_exported_;

    uint64_t elements_;
    uint64_t buckets_;

  public:
    // we need getters and setters for all stats:
    bool valid(void) const;
    void set_valid(bool valid);

    uint64_t biflows_cached(void) const;
    void set_biflows_cached(uint64_t biflowsCached);
    void inc_biflows_cached(void);

    uint64_t biflows_new(void) const;
    void set_biflows_new(uint64_t biflowsMerged);
    void inc_biflows_new(void);

    uint64_t buckets(void) const;
    void set_buckets(uint64_t buckets);
    void inc_buckets(void);

    uint64_t duration_add(void) const;
    void set_duration_add(uint64_t durationAdd);

    uint64_t duration_prune(void) const;
    void set_duration_prune(uint64_t durationPrune);

    uint64_t elements(void) const;
    void set_elements(uint64_t elements);
    void inc_elements(void);

    uint64_t flows_in(void) const;
    void set_flows_in(uint64_t flowsIn);
    void inc_flows_In(void);

    uint64_t flows_valid(void) const;
    void set_flows_valid(uint64_t flowsValid);
    void inc_flows_valid(void);

    uint64_t prune_exported(void) const;
    void set_prune_exported(uint64_t pruneExported);
    void inc_prune_exported(void);
    void inc_prune_exported(uint64_t inc);

    uint64_t prune_too_young(void) const;
    void set_prune_too_young(uint64_t pruneTooYoung);
    void inc_prune_too_young(void);
    void inc_prune_too_young(uint64_t inc);

    uint64_t time_s(void) const;
    void set_time_s(uint64_t timeS);

    std::string head_to_s(void) const;
    std::string to_s(void) const;

    // Methods
    void reset(void);

    Statistics();
    ~Statistics();
  };

  //----------------------------------------------------
  // BiFlowCache::Configuration class
  //----------------------------------------------------
  class Configuration {
  public:
    static const int kStatisticDefaultInterval;
    static const int kPruneIntervalDefault;
    static const int kExportTimeoutDefault;
    static const ImportPolicy kImportPolicyDefault;

  private:
    FlowContainerBuffer* input_;
    FlowContainerBuffer* output_;
    BiFlowContainerBuffer* output_biflow_;
    uint64_t stat_interval_;
    uint64_t prune_interval_s_;
    uint64_t export_timeout_s_;
    ImportPolicy import_policy_;

  public:
    // Methods
    Configuration();
    ~Configuration();
    FlowContainerBuffer* get_input(void) const;
    FlowContainerBuffer* get_output(void) const;
    BiFlowContainerBuffer* get_biflow_output(void) const;

    uint64_t stat_interval(void) const;
    uint64_t prune_interval(void) const;
    uint64_t export_timeout(void) const;
    ImportPolicy import_policy(void) const;

    void set_input(FlowContainerBuffer* input);
    void set_output(FlowContainerBuffer* output);
    void set_biflow_output(BiFlowContainerBuffer* output);
    void set_stat_interval(uint64_t interval);
    void set_prune_interval(uint64_t interval);
    void set_export_timeout(uint64_t timeout);
    void set_import_policy(ImportPolicy import_policy);

    void reset();
  };

  //----------------------------------------------------------------------------
  // BiFlowCache::Observation class
  //----------------------------------------------------------------------------
  class Observation {
  private:
    bool valid_;
    uint64_t time_s_;
    std::string message_;

  public:
    Observation();
    ~Observation();
    void reset();

    bool get_valid(void) const;
    uint64_t get_time_s(void) const;
    std::string get_message(void) const;

    void set_valid(bool valid);
    void set_time_s(uint64_t time_s);
    void set_message(const std::string& message);
  };

  //----------------------------------------------------
  // BiFlowCache Class
  //----------------------------------------------------

 private:
  // -- CONFIGURATION: INCOMING ------------------------
  sem_t conf_critical_section_sem_;
  std::queue<Configuration> conf_in_;
  bool conf_available_;
  ImportPolicy import_policy_;

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

  // --- CACHE STUFF ------------------------------------
  BiFlowPool& biflow_pool_;
  BiFlowContainerPool* biflow_container_pool_;
  uint64_t prune_interval_s_;
  uint64_t prune_last_s_;
  uint64_t export_timeout_s_;
  uint64_t next_export_s_;

  // the cache
  BiFlowHT biflow_hashtable_;

  // Input and Output Buffers and Pool for flow containers
  FlowContainerBuffer* data_input_;
  FlowContainerBuffer* data_output_;
  FlowContainerPool* data_container_pool_;

  BiFlowContainerBuffer* data_output_biflow_;

  // ## CLASS FUNCTIONS ##################################
  // -- CONFIGURATION: INCOMING --------------------------
  void conf_pop(void);
  void conf_migrate(const Configuration& migrate);

  // -- OBSERVATION: OUTGOING ----------------------------
  void obs_push(uint64_t now_s, const std::string& message);

  // -- STATISTICS: OUTGOING------------------------------
  void stat_export(uint64_t now_s);

  void process_flow_container(
      FlowContainer::iterator flow,
      FlowContainer::iterator end);

  void prune(uint64_t time);
  void prune_all(void);

 public:
  BiFlowCache(void);
  BiFlowCache(const BiFlowCache& other);
  BiFlowCache& operator=(const BiFlowCache& other);
  ~BiFlowCache(void);

  // -- CONFIGURATION: INCOMING --------------------------
  void conf_push(Configuration);

  // -- OBSERVATION: OUTGOING ----------------------------
  Observation obs_get(void);

  // -- STATISTICS: OUTGOING------------------------------
  Statistics stat_get(void);

  void add_flow(const FlowContainer::iterator& flow);
  void data_process(void);
};

#endif  // FLOW_BOX_INCLUDE_BI_FLOW_CACHE_H_
