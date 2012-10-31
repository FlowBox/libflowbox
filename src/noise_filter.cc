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

#include "noise_filter.h"

//------------------------------------------------------------------------------
// NoiseFilter::Statistics class
//----------------------------------------------------
NoiseFilter::Statistics::Statistics() {
  reset();
}

NoiseFilter::Statistics::~Statistics() {
}

void NoiseFilter::Statistics::reset(void) {
  time_s_ = 0;
  duration_s_ = 0;

  flows_in_ = 0;
  flows_valid_ = 0;
  flows_kept_ = 0;
  flows_filtered_ = 0;
}

const int NoiseFilter::Statistics::kStatisticDefaultInterval = 300;

void NoiseFilter::Statistics::set_valid(bool valid) {
  valid_ = valid;
}

bool NoiseFilter::Statistics::valid(void) const {
  return valid_;
}

uint64_t NoiseFilter::Statistics::time_s(void) const {
  return time_s_;
}

void NoiseFilter::Statistics::set_time_s(uint64_t timeS) {
  time_s_ = timeS;
}

uint64_t NoiseFilter::Statistics::duration_s(void) const {
  return duration_s_;
}

void NoiseFilter::Statistics::set_duration_s(uint64_t timeS) {
  duration_s_ = timeS;
}

uint64_t NoiseFilter::Statistics::flows_in(void) const {
  return flows_in_;
}

void NoiseFilter::Statistics::set_flows_in(uint64_t flows) {
  flows_in_ = flows;
}

void NoiseFilter::Statistics::inc_flows_in(void) {
  flows_in_++;
}

uint64_t NoiseFilter::Statistics::flows_valid(void) const {
  return flows_valid_;
}

void NoiseFilter::Statistics::set_flows_valid(uint64_t flows) {
  flows_valid_ = flows;
}

void NoiseFilter::Statistics::inc_flows_valid(void) {
  flows_valid_++;
}

uint64_t NoiseFilter::Statistics::flows_kept(void) const {
  return flows_kept_;
}

void NoiseFilter::Statistics::set_flows_kept(uint64_t flows) {
  flows_kept_ = flows;
}

void NoiseFilter::Statistics::inc_flows_kept(void) {
  flows_kept_++;
}

uint64_t NoiseFilter::Statistics::flows_filtered(void) const {
  return flows_filtered_;
}

void NoiseFilter::Statistics::set_flows_filtered(uint64_t flows) {
  flows_filtered_ = flows;
}

std::string NoiseFilter::Statistics::head_to_s(void) const {
  std::stringstream head;
  head << "time_s, ";
  head << "duration_s_, ";
  head << "flows_in, ";
  head << "flows_valid, ";
  head << "flows_kept, ";
  head << "flows_filtered";

  return (std::string(head.str()));
}

std::string NoiseFilter::Statistics::to_s(void) const {
  std::stringstream stat;
  stat << time_s_ << ", ";
  stat << duration_s_ << ", ";
  stat << flows_in_ << ", ";
  stat << flows_valid_ << ", ";
  stat << flows_kept_ << ", ";
  stat << flows_filtered_;
  return (std::string(stat.str()));
}

//------------------------------------------------------------------------------
// NoiseFilter::Configuration Class
//------------------------------------------------------------------------------
const int NoiseFilter::Configuration::kTcpMinPacketsDefault = 2;
const int NoiseFilter::Configuration::kUdpMinPacketsDefault = 2;

NoiseFilter::Configuration::Configuration() {
  reset();
}

NoiseFilter::Configuration::~Configuration() {
}

void NoiseFilter::Configuration::reset(void) {
  input_flow_ = NULL;
  output_flow_ = NULL;
  input_biflow_ = NULL;
  output_biflow_ = NULL;
  output_filtered_biflow_ = NULL;
  udp_packets_min_ = kUdpMinPacketsDefault;
  tcp_packets_min_ = kTcpMinPacketsDefault;
  stat_interval_ = Statistics::kStatisticDefaultInterval;
}

FlowContainerBuffer* NoiseFilter::Configuration::flow_input(void) const {
  return (input_flow_);
}

FlowContainerBuffer* NoiseFilter::Configuration::flow_output(void) const {
  return (output_flow_);
}

BiFlowContainerBuffer* NoiseFilter::Configuration::biflow_output(void) const {
  return (output_biflow_);
}

BiFlowContainerBuffer* NoiseFilter::Configuration::biflow_output_filtered(
    void) const {
  return (output_filtered_biflow_);
}

BiFlowContainerBuffer* NoiseFilter::Configuration::biflow_input(void) const {
  return (input_biflow_);
}

int NoiseFilter::Configuration::udp_packets_min(void) const {
  return (udp_packets_min_);
}

int NoiseFilter::Configuration::tcp_packets_min(void) const {
  return (tcp_packets_min_);
}

uint64_t NoiseFilter::Configuration::stat_interval(void) const {
  return (stat_interval_);
}

void NoiseFilter::Configuration::set_flow_input(FlowContainerBuffer* input) {
  input_flow_ = input;
}

void NoiseFilter::Configuration::set_flow_output(FlowContainerBuffer* output) {
  output_flow_ = output;
}

void NoiseFilter::Configuration::set_biflow_input(
    BiFlowContainerBuffer* input) {
  input_biflow_ = input;
}

void NoiseFilter::Configuration::set_biflow_output(
    BiFlowContainerBuffer* output) {
  output_biflow_ = output;
}

void NoiseFilter::Configuration::set_biflow_output_filtered(
    BiFlowContainerBuffer* output) {
  output_filtered_biflow_ = output;
}

void NoiseFilter::Configuration::set_udp_packets_min(int num) {
  udp_packets_min_ = num;
}

void NoiseFilter::Configuration::set_tcp_packets_min(int num) {
  tcp_packets_min_ = num;
}

void NoiseFilter::Configuration::set_stat_interval(uint64_t interval) {
  stat_interval_ = interval;
}

//------------------------------------------------------------------------------
// NoiseFilter::Observation Class
//------------------------------------------------------------------------------
NoiseFilter::Observation::Observation() {
  reset();
}

NoiseFilter::Observation::~Observation() {
}

void NoiseFilter::Observation::reset(void) {
  valid_ = false;
  time_s_ = 0;
  message_ = "";
}

bool NoiseFilter::Observation::valid(void) const {
  return (valid_);
}

uint64_t NoiseFilter::Observation::time_s(void) const {
  return (time_s_);
}

std::string NoiseFilter::Observation::message(void) const {
  return (std::string(message_));
}

void NoiseFilter::Observation::set_valid(bool valid) {
  valid_ = valid;
}

void NoiseFilter::Observation::set_time_s(uint64_t time_s) {
  time_s_ = time_s;
}

void NoiseFilter::Observation::set_message(const std::string& message) {
  message_ = message;
}

//------------------------------------------------------------------------------
// NoiseFilter class
//------------------------------------------------------------------------------

NoiseFilter::NoiseFilter()
    : data_input_flow_(NULL),
      data_output_flow_(NULL),
      flow_container_pool_(FlowContainerPool::instance()),
      data_input_biflow_(NULL),
      data_output_biflow_(NULL),
      data_output_biflow_filtered_(NULL),
      biflow_container_pool_(BiFlowContainerPool::instance()),
      biflow_pool_(*BiFlowPool::instance()) {
  // -- CONFIGURATION ----------------------------------------------------------
  sem_init(&conf_critical_section_sem_, 0, 1);
  conf_available_ = false;

  // -- OBSERVATION ------------------------------------------------------------
  sem_init(&obs_critical_section_sem_, 0, 1);
  sem_init(&obs_available_sem_, 0, 0);

  // -- STATISTICS -------------------------------------------------------------
  sem_init(&stat_critical_section_sem_, 0, 1);
  sem_init(&stat_available_sem_, 0, 0);
  stat_current_.reset();

  udp_packets_min_ = Configuration::kUdpMinPacketsDefault;
  tcp_packets_min_ = Configuration::kTcpMinPacketsDefault;
}

NoiseFilter::~NoiseFilter() {
}

// Configuration Functions
void NoiseFilter::conf_push(Configuration config) {
  fb_sem_wait(&conf_critical_section_sem_);
  // CRITICAL SECTION::START
  conf_in_.push(config);
  conf_available_ = true;
  fb_sem_post(&conf_critical_section_sem_);
  // CRITICAL SECTION::STOP
  return;
}

void NoiseFilter::conf_pop(void) {
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

void NoiseFilter::conf_migrate(const Configuration& migrate) {
  // INPUT configuration
  data_input_flow_ = migrate.flow_input();
  data_input_biflow_ = migrate.biflow_input();

  // OUTPUT configuration
  data_output_flow_ = migrate.flow_output();
  data_output_biflow_ = migrate.biflow_output();
  data_output_biflow_filtered_ = migrate.biflow_output_filtered();

  // FILTER parameters
  udp_packets_min_ = migrate.udp_packets_min();
  tcp_packets_min_ = migrate.tcp_packets_min();

  stat_interval_s_ = migrate.stat_interval();

  return;
}

// Observation Function
NoiseFilter::Observation NoiseFilter::obs_get(void) {
  NoiseFilter::Observation obs;
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
  return (NoiseFilter::Observation(obs));
}

void NoiseFilter::obs_push(uint64_t now_s, const std::string& message) {
  NoiseFilter::Observation obs;
  obs.set_time_s(now_s);
  obs.set_message(message);
  fb_sem_wait(&obs_critical_section_sem_);
  // CRITICAL SECTION::START
  obs_out_.push(obs);  // push data
  fb_sem_post(&obs_critical_section_sem_);
  // CRITICAL SECTION::STOP
  fb_sem_post(&obs_available_sem_);
  // SIGNAL DATA
}

// Statistics Functions
NoiseFilter::Statistics NoiseFilter::stat_get(void) {
  NoiseFilter::Statistics stat;
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
  return (NoiseFilter::Statistics(stat));
}

void NoiseFilter::stat_export(uint64_t now_s) {
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

void NoiseFilter::data_process(void) {
  obs_push(0, "ProcessInput -- started");
  FlowContainer* fc;
  BiFlowContainer* bfc;

  while (true) {
    // do we have some a new config?
    if (conf_available_) {
      obs_push(0, "Conf available -- pop");
      conf_pop();
    }

    // ups no input buffer...
    if (data_input_flow_ == NULL and data_input_biflow_ == NULL) {
      std::cout << "NoiseFilter -- empty input configuration" << std::endl;
      obs_push(0, "data_thread_main -- empty input configuartion");
      sleep(5);
      continue;
    }

    // ups two input buffers...
    if (data_input_flow_ != NULL and data_input_biflow_ != NULL) {
      std::cout
          << "NoiseFilter -- flow and biflow input configuration won't work!"
          << std::endl;
      obs_push(0, "data_thread_main -- two input configuartion");
      sleep(5);
      continue;
    }

    // ups wrong input / output buffers...
    if (data_input_flow_ != NULL and data_output_biflow_ != NULL) {
      std::cout << "NoiseFilter -- "
                << "flow input and biflow output configuration won't work!"
                << std::endl;
      obs_push(0, "data_thread_main -- wrong input/output configuartion");
      sleep(5);
      continue;
    }

    // ups two input buffers...
    if (data_input_biflow_ != NULL and data_output_flow_ != NULL) {
      std::cout << "NoiseFilter -- "
                << "biflow input and flow output configuration won't work!"
                << std::endl;
      obs_push(0, "data_thread_main -- wrong input/output configuartion");
      sleep(5);
      continue;
    }

    // IN_OUT CONFIG IS FINE::

    // FLOW DATA PROCESSING
    if (data_input_flow_ != NULL) {
      fc = data_input_flow_->pop();

      // since the locking wait can be interrupted
      // by system interrupts, timeouts or signals
      // we have to check if there is the pointer
      // is valid.
      if (fc == NULL) {
        // invalid data but WHY?

        // FIN: is the processing over?
        if (data_input_flow_->is_state_fin()) {
          obs_push(0, "data_thread_main-- fin signaled");

          // forward the signal to the next buffer (if possible)
          if (data_output_flow_ != NULL) {
            data_output_flow_->signal_fin();
          }

          // ... and say goodbye
          break;

          // unknown reason
        } else {
          // WTF ??
          obs_push(0, "data_thread_main -- empty flow container?!?");
          continue;
        }
      } else {
        // WORK:
        process_flow_container(fc->begin(), fc->end_used());

        // forward the buffer to the next element
        if (data_output_flow_ != NULL) {
          data_output_flow_->push(fc);

        } else {
          flow_container_pool_->push(fc);
        }
      }
    }

    // BIFLOW DATA PROCESSING...
    if (data_input_biflow_ != NULL) {
      // Get the biflow container from buffer
      bfc = data_input_biflow_->pop();

      if (bfc == NULL) {
        // invalid data but WHY?

        // FIN: is the processing over?
        if (data_input_biflow_->is_state_fin()) {
          obs_push(0, "data_thread_main-- fin signaled");

          // forward the signal to the next buffer (if possible)
          if (data_output_biflow_ != NULL) {
            data_output_biflow_->signal_fin();
          }

          // ... and say goodbye
          break;

          // unknown reason
        } else {
          // WTF ??
          obs_push(0, "data_thread_main -- empty biflow container?!?");
          continue;
        }
      } else {
        // WORK:
        BiFlow* filtered = NULL;
        BiFlow* kept = NULL;
        process_biflow_container(bfc, kept, filtered);
        uint64_t time = bfc->time();

        // set the kept biflows in the biflow container
        bfc->set_biflows(kept);

        // forward the buffer to the next element
        if (data_output_biflow_ != NULL) {
          data_output_biflow_->push(bfc);

        } else {
          // this will implicitly delete the containing flows..
          biflow_container_pool_->push(bfc);
        }

        // do we have some output configured for the filtered flows?
        if (data_output_biflow_filtered_ != NULL) {
          // Grab a new biflow container and put the filtered flows in it!
          BiFlowContainer* bfc_filtered = biflow_container_pool_->pop();
          bfc_filtered->set_time(time);
          bfc_filtered->set_biflows(filtered);

          // finally push the biflow container on the right buffer
          data_output_biflow_filtered_->push(bfc_filtered);

        } else {
          // we don't have an output configured for the filtered biflows,
          // so return all filtered biflows to the pool and delete them!
          biflow_pool_.delete_biflows(filtered);
        }
      }
    }
  }

  // ... say goodby
  obs_push(0, "data_thread_main -- finished");
  return;
}

void NoiseFilter::process_flow_container(FlowContainer::iterator flow,
                                         FlowContainer::iterator end) {
  time_t begin_s, end_s;
  uint64_t valid = 0;
  int flows_kept = 0;
  int flows_filtered = 0;
  int flows_in = 0;

  std::time(&begin_s);
  // walk over flows
  while (flow != end) {
    // export stats?
    if (flow->start_s_ > stat_export_next_s_) {
      stat_export(flow->start_s_);
    }

    if (flow->valid_ == true) {
      valid++;

      // Do the filtering..
      if (flow->protocol_ == 6
          and flow->packets_ < (uint64_t) tcp_packets_min_) {
        // ...in case we have a TCP flow with less packets than our threshold
        // just set the flow as invalid..
        flow->valid_ = false;
        flows_filtered++;
      } else if (flow->protocol_ == 17
          and flow->packets_ < (uint64_t) udp_packets_min_) {
        // ...in case we have a UDP flow with less packets than our threshold
        // just set the flow as invalid..
        flow->valid_ = false;
        flows_filtered++;
      } else {
        // FIXME (asdaniel): should we cover more?!

        // increment the flows kept statistic
        flows_kept++;
      }
    }
    // next flow
    flow++;
    // increment the flow in statistic
    flows_in++;
  }
  std::time(&end_s);
  stat_current_.set_duration_s((uint64_t) std::difftime(end_s, begin_s));
  stat_current_.set_flows_valid(stat_current_.flows_valid() + valid);
  stat_current_.set_flows_filtered(
      stat_current_.flows_filtered() + flows_filtered);
  stat_current_.set_flows_in(stat_current_.flows_in() + flows_in);
  stat_current_.set_flows_kept(stat_current_.flows_kept() + flows_kept);
}

void NoiseFilter::process_biflow_container(BiFlowContainer* bfc, BiFlow* & kept,
                                           BiFlow* & filtered) {
  time_t begin_s, end_s;
  uint64_t valid = 0;
  int flows_kept = 0;
  int flows_filtered = 0;
  int flows_in = 0;
  uint64_t time = 0;

  std::time(&begin_s);

  if (bfc != NULL) {
    time = bfc->time();
    BiFlow* current = bfc->biflows();
    BiFlow* next = NULL;

    // walk over flows
    while (current != NULL) {
      // save the next biflow pointer
      next = current->next_;

      if (current->valid_ == true) {
        valid++;

        // Do the filtering..
        if (current->protocol_ == 6
            and (current->in_out_packets_ < (uint64_t) tcp_packets_min_
                or current->out_in_packets_ < (uint64_t) tcp_packets_min_)) {
          // ...in case we have a TCP flow with less packets than our threshold
          // just set the biflow as invalid and append it to filtered list..
          current->valid_ = false;
          current->next_ = filtered;
          filtered = current;
          flows_filtered++;
        } else if (current->protocol_ == 17
            and (current->in_out_packets_ < (uint64_t) udp_packets_min_
                or current->out_in_packets_ < (uint64_t) udp_packets_min_)) {
          // ...in case we have a UDP flow with less packets than our threshold
          // just set the flow as invalid..
          current->valid_ = false;
          current->next_ = filtered;
          filtered = current;
          flows_filtered++;
        } else {
          // FIXME(asdaniel): should we cover more?!
          current->next_ = kept;
          kept = current;
          // increment the flows kept statistic
          flows_kept++;
        }
      } else {
        // the flow is not valid... delete it!
        biflow_pool_.delete_biflow(current);
      }

      // next flow
      current = next;
      // increment the flow in statistic
      flows_in++;
    }

    std::time(&end_s);
    stat_current_.set_duration_s((uint64_t) std::difftime(end_s, begin_s));
    stat_current_.set_flows_valid(stat_current_.flows_valid() + valid);
    stat_current_.set_flows_filtered(
        stat_current_.flows_filtered() + flows_filtered);
    stat_current_.set_flows_in(stat_current_.flows_in() + flows_in);
    stat_current_.set_flows_kept(stat_current_.flows_kept() + flows_kept);
    stat_export(time);
  }
}
