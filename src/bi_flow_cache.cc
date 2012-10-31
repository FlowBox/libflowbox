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

#include "bi_flow_cache.h"

//----------------------------------------------------
// BiFlowCache::Statistics class
//----------------------------------------------------
BiFlowCache::Statistics::Statistics() {
  reset();
}

BiFlowCache::Statistics::~Statistics() {
}

void BiFlowCache::Statistics::reset(void) {
  time_s_ = 0;
  duration_add_ = 0;
  duration_prune_ = 0;

  flows_in_ = 0;
  flows_valid_ = 0;
  biflows_new_ = 0;
  biflows_cached_ = 0;

  prune_too_young_ = 0;
  prune_exported_ = 0;

  elements_ = 0;
  buckets_ = 0;

  // also reset the pool statistics
  pool_stats_.reset();
}

void BiFlowCache::Statistics::set_valid(bool valid) {
  valid_ = valid;
}

bool BiFlowCache::Statistics::valid(void) const {
  return valid_;
}

uint64_t BiFlowCache::Statistics::biflows_cached(void) const {
  return biflows_cached_;
}

void BiFlowCache::Statistics::set_biflows_cached(uint64_t biflowsCached) {
  biflows_cached_ = biflowsCached;
}

void BiFlowCache::Statistics::inc_biflows_cached(void) {
  biflows_cached_++;
}

uint64_t BiFlowCache::Statistics::biflows_new(void) const {
  return biflows_new_;
}

void BiFlowCache::Statistics::set_biflows_new(uint64_t biflowsnew) {
  biflows_new_ = biflowsnew;
}

void BiFlowCache::Statistics::inc_biflows_new(void) {
  biflows_new_++;
}

uint64_t BiFlowCache::Statistics::buckets(void) const {
  return buckets_;
}

void BiFlowCache::Statistics::set_buckets(uint64_t buckets) {
  buckets_ = buckets;
}

void BiFlowCache::Statistics::inc_buckets(void) {
  buckets_++;
}

uint64_t BiFlowCache::Statistics::duration_add(void) const {
  return duration_add_;
}

void BiFlowCache::Statistics::set_duration_add(uint64_t durationAdd) {
  duration_add_ = durationAdd;
}

uint64_t BiFlowCache::Statistics::duration_prune(void) const {
  return duration_prune_;
}

void BiFlowCache::Statistics::set_duration_prune(uint64_t durationPrune) {
  duration_prune_ = durationPrune;
}

uint64_t BiFlowCache::Statistics::elements(void) const {
  return elements_;
}

void BiFlowCache::Statistics::set_elements(uint64_t elements) {
  elements_ = elements;
}

void BiFlowCache::Statistics::inc_elements(void) {
  elements_++;
}

uint64_t BiFlowCache::Statistics::flows_in(void) const {
  return flows_in_;
}

void BiFlowCache::Statistics::set_flows_in(uint64_t flowsIn) {
  flows_in_ = flowsIn;
}

void BiFlowCache::Statistics::inc_flows_In(void) {
  flows_in_++;
}

uint64_t BiFlowCache::Statistics::flows_valid(void) const {
  return flows_valid_;
}

void BiFlowCache::Statistics::set_flows_valid(uint64_t flowsValid) {
  flows_valid_ = flowsValid;
}

void BiFlowCache::Statistics::inc_flows_valid(void) {
  flows_valid_++;
}

uint64_t BiFlowCache::Statistics::prune_exported(void) const {
  return prune_exported_;
}

void BiFlowCache::Statistics::set_prune_exported(uint64_t pruneExported) {
  prune_exported_ = pruneExported;
}

void BiFlowCache::Statistics::inc_prune_exported(void) {
  prune_exported_++;
}

void BiFlowCache::Statistics::inc_prune_exported(uint64_t inc) {
  prune_exported_ = prune_exported_ + inc;
}

uint64_t BiFlowCache::Statistics::prune_too_young(void) const {
  return prune_too_young_;
}

void BiFlowCache::Statistics::set_prune_too_young(uint64_t pruneTooYoung) {
  prune_too_young_ = pruneTooYoung;
}

void BiFlowCache::Statistics::inc_prune_too_young(void) {
  prune_too_young_++;
}

void BiFlowCache::Statistics::inc_prune_too_young(uint64_t inc) {
  prune_too_young_ = prune_too_young_ + inc;
}

uint64_t BiFlowCache::Statistics::time_s(void) const {
  return time_s_;
}

void BiFlowCache::Statistics::set_time_s(uint64_t timeS) {
  time_s_ = timeS;
}

std::string BiFlowCache::Statistics::head_to_s(void) const {
  std::stringstream head;
  head << "time_s, ";
  head << "duration_add_, ";
  head << "duration_prune_, ";
  head << "flows_in, ";
  head << "flows_valid, ";
  head << "biflows_new, ";
  head << "biflows_cached, ";
  head << "prune_too_young, ";
  head << "prune_exported, ";
  head << "elements, ";
  head << "buckets, ";

  // also return the pool statistics
  head << pool_stats_.head_to_s();

  return (std::string(head.str()));
}

std::string BiFlowCache::Statistics::to_s(void) const {
  std::stringstream stat;
  stat << time_s_ << ", ";
  stat << duration_add_ << ", ";
  stat << duration_prune_ << ", ";
  stat << flows_in_ << ", ";
  stat << flows_valid_ << ", ";
  stat << biflows_new_ << ", ";
  stat << biflows_cached_ << ", ";
  stat << prune_too_young_ << ", ";
  stat << prune_exported_ << ", ";
  stat << elements_ << ", ";
  stat << buckets_ << ", ";

  // also return the pool statistics
  stat << pool_stats_.to_s();

  return (std::string(stat.str()));
}

//------------------------------------------------------------------------------
// BiFlowCache::Configuration Class
//------------------------------------------------------------------------------
const int BiFlowCache::Configuration::kStatisticDefaultInterval = 300;
const int BiFlowCache::Configuration::kPruneIntervalDefault = 300;
const int BiFlowCache::Configuration::kExportTimeoutDefault = 360;

const BiFlowCache::ImportPolicy BiFlowCache::Configuration::kImportPolicyDefault =
    BiFlowCache::kImportPolicyAddAll;
BiFlowCache::Configuration::Configuration() {
  reset();
}

BiFlowCache::Configuration::~Configuration() {
}

void BiFlowCache::Configuration::reset(void) {
  input_ = NULL;
  output_ = NULL;
  output_biflow_ = NULL;
  stat_interval_ = kStatisticDefaultInterval;
  prune_interval_s_ = kStatisticDefaultInterval;
  import_policy_ = kImportPolicyDefault;
}

FlowContainerBuffer* BiFlowCache::Configuration::get_input(void) const {
  return (input_);
}

FlowContainerBuffer* BiFlowCache::Configuration::get_output(void) const {
  return (output_);
}

BiFlowContainerBuffer* BiFlowCache::Configuration::get_biflow_output(
    void) const {
  return (output_biflow_);
}

uint64_t BiFlowCache::Configuration::stat_interval(void) const {
  return (stat_interval_);
}

uint64_t BiFlowCache::Configuration::prune_interval(void) const {
  return (prune_interval_s_);
}

uint64_t BiFlowCache::Configuration::export_timeout(void) const {
  return (export_timeout_s_);
}
BiFlowCache::ImportPolicy BiFlowCache::Configuration::import_policy(
    void) const {
  return (import_policy_);
}

//------------------------------------------------------------------------------
void BiFlowCache::Configuration::set_input(FlowContainerBuffer* input) {
  input_ = input;
}

void BiFlowCache::Configuration::set_output(FlowContainerBuffer* output) {
  output_ = output;
}

void BiFlowCache::Configuration::set_biflow_output(
    BiFlowContainerBuffer* output) {
  output_biflow_ = output;
}

void BiFlowCache::Configuration::set_stat_interval(uint64_t interval) {
  stat_interval_ = interval;
}

void BiFlowCache::Configuration::set_prune_interval(uint64_t interval) {
  prune_interval_s_ = interval;
}

void BiFlowCache::Configuration::set_export_timeout(uint64_t timeout) {
  export_timeout_s_ = timeout;
}
void BiFlowCache::Configuration::set_import_policy(ImportPolicy import_policy) {
  import_policy_ = import_policy;
}

//------------------------------------------------------------------------------
// BiFlowCache::Observation Class
//------------------------------------------------------------------------------
BiFlowCache::Observation::Observation() {
  reset();
}

BiFlowCache::Observation::~Observation() {
}

void BiFlowCache::Observation::reset(void) {
  valid_ = false;
  time_s_ = 0;
  message_ = "";
}

bool BiFlowCache::Observation::get_valid(void) const {
  return (valid_);
}

uint64_t BiFlowCache::Observation::get_time_s(void) const {
  return (time_s_);
}

std::string BiFlowCache::Observation::get_message(void) const {
  return (std::string(message_));
}

void BiFlowCache::Observation::set_valid(bool valid) {
  valid_ = valid;
}

void BiFlowCache::Observation::set_time_s(uint64_t time_s) {
  time_s_ = time_s;
}

void BiFlowCache::Observation::set_message(const std::string& message) {
  message_ = message;
}

//------------------------------------------------------------------------------
// BiFlowCache class
//------------------------------------------------------------------------------

BiFlowCache::BiFlowCache()
    : biflow_pool_(*BiFlowPool::instance()),
      biflow_container_pool_(BiFlowContainerPool::instance()),
      data_input_(NULL),
      data_output_(NULL),
      data_container_pool_(FlowContainerPool::instance()),
      data_output_biflow_(NULL) {
  // -- CONFIGURATION ----------------------------------------------------------
  sem_init(&conf_critical_section_sem_, 0, 1);
  conf_available_ = false;
  import_policy_ = Configuration::kImportPolicyDefault;

  // -- OBSERVATION ------------------------------------------------------------
  sem_init(&obs_critical_section_sem_, 0, 1);
  sem_init(&obs_available_sem_, 0, 0);

  // -- STATISTICS -------------------------------------------------------------
  sem_init(&stat_critical_section_sem_, 0, 1);
  sem_init(&stat_available_sem_, 0, 0);
  stat_current_.reset();
  stat_interval_s_ = Configuration::kStatisticDefaultInterval;
  stat_export_next_s_ = 0;

  // -- DATA STREAM  -----------------------------------------------------------
  data_input_ = NULL;
  data_output_ = NULL;

  // --- CACHE STUFF ------------------------------------
  prune_interval_s_ = Configuration::kPruneIntervalDefault;
  prune_last_s_ = 0;
  export_timeout_s_ = Configuration::kExportTimeoutDefault;
  next_export_s_ = 0;

  // Clear the HashTable
  biflow_hashtable_.clear();
}

BiFlowCache::~BiFlowCache() {
}

// Configuration Functions
void BiFlowCache::conf_push(Configuration config) {
  fb_sem_wait(&conf_critical_section_sem_);
  // CRITICAL SECTION::START
  conf_in_.push(config);
  conf_available_ = true;
  fb_sem_post(&conf_critical_section_sem_);
  // CRITICAL SECTION::STOP
  return;
}

void BiFlowCache::conf_pop(void) {
  fb_sem_wait(&conf_critical_section_sem_);
  // CRITICAL SECTION::START
  while (conf_in_.size() > 0) {
    // get config from queue
    Configuration migrate(conf_in_.front());
    conf_in_.pop();
    // apply config
    conf_migrate(migrate);
  }
  conf_available_ = false;
  fb_sem_post(&conf_critical_section_sem_);
  // CRITICAL SECTION::STOP
  return;
}

void BiFlowCache::conf_migrate(const Configuration& migrate) {
  data_input_ = migrate.get_input();
  data_output_ = migrate.get_output();
  data_output_biflow_ = migrate.get_biflow_output();
  stat_interval_s_ = migrate.stat_interval();
  prune_interval_s_ = migrate.prune_interval();
  export_timeout_s_ = migrate.export_timeout();
  import_policy_ = migrate.import_policy();
  return;
}

// Observation Function
BiFlowCache::Observation BiFlowCache::obs_get(void) {
  BiFlowCache::Observation obs;
  fb_sem_wait(&obs_available_sem_);
  // WAIT ON DATA
  fb_sem_wait(&obs_critical_section_sem_);
  // CRITICAL SECTION::START
  assert(obs_out_.size() >= 1);
  obs = obs_out_.front();
  obs_out_.pop();
  fb_sem_post(&obs_critical_section_sem_);
  // CRITICAL SECTION::STOP
  obs.set_valid(true);
  return (BiFlowCache::Observation(obs));
}

void BiFlowCache::obs_push(uint64_t now_s, const std::string& message) {
  BiFlowCache::Observation obs;
  obs.set_time_s(now_s);
  obs.set_message(message);
  fb_sem_wait(&obs_critical_section_sem_);

  // CRITICAL SECTION::START
  obs_out_.push(obs);  // push data
  fb_sem_post(&obs_critical_section_sem_);
  // CRITICAL SECTION::STOP

  fb_sem_post(&obs_available_sem_);
}

// Statistics Functions
BiFlowCache::Statistics BiFlowCache::stat_get(void) {
  BiFlowCache::Statistics stat;
  fb_sem_wait(&stat_available_sem_);

  // WAIT ON DATA
  fb_sem_wait(&stat_critical_section_sem_);

  // CRITICAL SECTION::START
  assert(stat_out_.size() >= 1);
  stat = stat_out_.front();
  stat_out_.pop();
  fb_sem_post(&stat_critical_section_sem_);
  // CRITICAL SECTION::STOP

  stat.set_valid(true);
  return (BiFlowCache::Statistics(stat));
}
void BiFlowCache::stat_export(uint64_t now_s) {
  if (stat_current_.time_s() != 0) {
    fb_sem_wait(&stat_critical_section_sem_);

    // CRITICAL SECTION::START
    stat_out_.push(stat_current_);  // push data
    fb_sem_post(&stat_critical_section_sem_);
    // CRITICAL SECTION::STOP
    fb_sem_post(&stat_available_sem_);
    // SIGNAL DATA
  }
  stat_current_.reset();
  stat_current_.set_time_s(now_s);
  stat_export_next_s_ = ((now_s / stat_interval_s_) + 1) * stat_interval_s_;
}

void BiFlowCache::data_process(void) {
  obs_push(0, "ProcessInput -- started");

  FlowContainer* fc;
  while (true) {
    // do we have some a new config?
    if (conf_available_) {
      obs_push(0, "Conf available -- pop");
      conf_pop();
    }

    // ups no input buffer ...
    if (data_input_ == NULL) {
      std::cout << "BiFlowCache -- empty data_input_" << std::endl;
      obs_push(0, "data_thread_main -- empty data_input_");
      sleep(5);
      continue;
    }

    // blocking wait on some input
    fc = data_input_->pop();

    // since the locking wait can be interrupted
    // by system interrupts, timeouts or signals
    // we have to check if there is the pointer
    // is valid.
    if (fc == NULL) {
      // invalid data but WHY?

      // FIN: is the processing over?
      if (data_input_->is_state_fin()) {
        obs_push(0, "Fdata_thread_main-- fin signaled");

        // forward the signal to the next buffer (if possible)
        if (data_output_ != NULL) {
          data_output_->signal_fin();
        }

        // Clean up hashtable and return biflows to pool
        prune_all();

        // ... and say goodbye
        break;

        // unknown reason
      } else {
        // WTF ??
        obs_push(0, "data_thread_main -- empty container?!?");
        continue;
      }
    } else {
      // WORK:
      process_flow_container(fc->begin(), fc->end_used());

      // forward the buffer to the next element
      if (data_output_ != NULL) {
        data_output_->push(fc);

      } else {
        data_container_pool_->push(fc);
      }
    }
  }  // while(true)

  // ... say goodby
  obs_push(0, "data_thread_main -- finished");
  return;
}

void BiFlowCache::process_flow_container(FlowContainer::iterator flow,
                                         FlowContainer::iterator end) {
  time_t begin_s, end_s;
  uint64_t valid = 0;
  uint64_t flow_counter = 0;

  std::time(&begin_s);

  // walk over flows
  while (flow != end) {
    // is it time to export stats?
    if (flow->start_s_ >= stat_export_next_s_) {
      stat_export(flow->start_s_);
    }

    // is it time to prune too old biflows?
    if (flow->start_s_ >= next_export_s_) {
      // update the time trigger for the next exporting
      next_export_s_ = ((flow->start_s_ / export_timeout_s_) + 1)
          * export_timeout_s_;
      // prune all flows which are older than the prune interval
      prune(flow->start_s_ - prune_interval_s_);
    }

    if (flow->valid_ == true) {
      add_flow(flow);
      valid++;
    }
    flow++;
    flow_counter++;
  }

  std::time(&end_s);
  stat_current_.set_duration_add(
      stat_current_.duration_add() + (uint64_t) std::difftime(end_s, begin_s));
  stat_current_.set_flows_valid(stat_current_.flows_valid() + valid);
  stat_current_.set_flows_in(stat_current_.flows_in() + flow_counter);
  stat_current_.set_elements(biflow_hashtable_.size());
  stat_current_.set_buckets(biflow_hashtable_.bucket_count());
}

//
// prune does check for old flows, removes them from the HT and marks them to
// hand them back to the pool
//
void BiFlowCache::prune(uint64_t time) {
  uint64_t exported = 0;
  uint64_t too_young = 0;
  time_t begin, end;

  // set the last prune time to the current time
  prune_last_s_ = time;

  BiFlow* exported_biflows_ = NULL;
  BiFlowHT::iterator iter(biflow_hashtable_.begin());

  // measure the duration time of prune... *tic*
  std::time(&begin);

  while (iter != biflow_hashtable_.end()) {
    BiFlow* p = iter->second;

    // Delete the flow if it has started before the export_time
    if ((p->in_out_start_s_ <= time) && (p->out_in_start_s_ <= time)) {
      biflow_hashtable_.erase(iter++);
      // add the biflow to the purge list
      p->next_ = exported_biflows_;
      exported_biflows_ = p;
      exported++;
    } else {
      iter++;
      too_young++;
    }
  }
  // *toc* now we have the time used to loop check the ht..
  std::time(&end);

  // update the statistics
  stat_current_.set_duration_prune(
      stat_current_.duration_prune() + (uint64_t) std::difftime(end, begin));
  stat_current_.inc_prune_exported(exported);
  stat_current_.set_prune_too_young(too_young);
  stat_current_.set_elements(biflow_hashtable_.size());
  stat_current_.set_buckets(biflow_hashtable_.bucket_count());

  // forward the buffer to the next element or if last element put back to pool
  if (data_output_biflow_ != NULL && exported_biflows_ != NULL) {
    BiFlowContainer* biflow_container = biflow_container_pool_->pop();
    biflow_container->set_time(time);
    biflow_container->set_biflows(exported_biflows_);
    data_output_biflow_->push(biflow_container);

  } else {
    // put them back to the biflow pool!
    biflow_pool_.delete_biflows(exported_biflows_);
  }
  return;
}

void BiFlowCache::prune_all(void) {
  uint64_t exported = 0;
  time_t begin, end;
  BiFlow* exported_biflows_ = NULL;
  BiFlowHT::iterator iter(biflow_hashtable_.begin());

  // measure the duration time of prune... *tic*
  std::time(&begin);
  while (iter != biflow_hashtable_.end()) {
    BiFlow* p = iter->second;
    // delete the biflow entry from the HT..
    biflow_hashtable_.erase(iter++);
    // add the biflow to the purge list
    p->next_ = exported_biflows_;
    exported_biflows_ = p;
    exported++;
  }
  // *toc* now we have the time used to loop check the ht..
  std::time(&end);

  // update the statistics
  stat_current_.set_duration_prune(
      stat_current_.duration_prune() + (uint64_t) std::difftime(end, begin));
  stat_current_.inc_prune_exported(exported);

  // forward the buffer to the next element or if last element put back to pool
  if (data_output_biflow_ != NULL && exported_biflows_ != NULL) {
    BiFlowContainer* biflow_container = biflow_container_pool_->pop();

    // since we remove all flows make sure the time is old..
    // e.g. the last prune_time
    biflow_container->set_time(prune_last_s_);
    biflow_container->set_biflows(exported_biflows_);
    data_output_biflow_->push(biflow_container);
  } else {
    // put them back to the pool!
    biflow_pool_.delete_biflows(exported_biflows_);
  }

  // prune all is called on fin.. so we have to signal fin to output as well..
  if (data_output_biflow_ != NULL) {
    data_output_biflow_->signal_fin();
  }
}

void BiFlowCache::add_flow(const FlowContainer::iterator& flow) {
  // import policy: should this flow be imported?
  if (import_policy_ == kImportPolicyAddBorderOnly) {
    if (flow->direction_ == Flow::kDirectionInIn)
      return;

    if (flow->direction_ == Flow::kDirectionOutOut)
      return;

    if (flow->border_ == Flow::kBorderUnknown)
      return;

  } else if (import_policy_ == kImportPolicyAddOutBorderInAll) {
    if (flow->direction_ == Flow::kDirectionUnknown)
      return;

    if (flow->direction_ == Flow::kDirectionInIn)
      return;

    if (flow->direction_ == Flow::kDirectionOutOut)
      return;

    if (flow->direction_ == Flow::kDirectionInOut
        and flow->border_ == Flow::kBorderUnknown)
      return;
  }

  // this flow passed the impor policy -- add it.

  // build tmp key
  BiFlow biflow(*flow);
  BiFlow::Key5l key(&biflow);
  // check the HT
  BiFlowHT::iterator iter = biflow_hashtable_.find(key);
  if (iter == biflow_hashtable_.end()) {
    // the flow key (5TP) is not in the HT -> add
    BiFlow* p = biflow_pool_.new_biflow();
    p->from(biflow);
    BiFlow::Key5l keyTmp(p);
    biflow_hashtable_[keyTmp] = p;
    stat_current_.inc_biflows_new();
  } else {
    // the flow key (5TP) is in the HT -> merge
    BiFlow* p = iter->second;
    p->merge(*flow);
    stat_current_.inc_biflows_cached();
  }
  return;
}
