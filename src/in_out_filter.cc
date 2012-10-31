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

#include "in_out_filter.h"

//------------------------------------------------------------------------------
// class InOutFilter::Const
//------------------------------------------------------------------------------
const int InOutFilter::kOhter = 0;
const int InOutFilter::kInside = 1;
const int InOutFilter::kUnknown = -1;

//------------------------------------------------------------------------------
// class InOutFilter::Configuration
//------------------------------------------------------------------------------
const int InOutFilter::Configuration::kStatisticDefaultInterval = 300;
const InOutFilter::Policy InOutFilter::Configuration::kPolicyDefault =
    InOutFilter::kTag;

InOutFilter::Configuration::Configuration() {
  reset();
}

void InOutFilter::Configuration::reset(void) {
  input_ = NULL;
  output_ = NULL;
  stat_interval_ = kStatisticDefaultInterval;
  prefix_.clear();
  policy_ = kPolicyDefault;
}

//------------------------------------------------------------------------------
FlowContainerBuffer* InOutFilter::Configuration::get_input(void) const {
  return (input_);
}

FlowContainerBuffer* InOutFilter::Configuration::get_output(void) const {
  return (output_);
}

std::vector<std::string>::const_iterator InOutFilter::Configuration::get_prefix_begin(
    void) const {
  return (prefix_.begin());
}

std::vector<std::string>::const_iterator InOutFilter::Configuration::get_prefix_end(
    void) const {
  return (prefix_.end());
}

uint64_t InOutFilter::Configuration::get_stat_interval(void) const {
  return (stat_interval_);
}

InOutFilter::Policy InOutFilter::Configuration::get_policy(void) const {
  return (policy_);
}

//------------------------------------------------------------------------------
void InOutFilter::Configuration::set_input(FlowContainerBuffer* input) {
  input_ = input;
}

void InOutFilter::Configuration::set_output(FlowContainerBuffer* output) {
  output_ = output;
}

void InOutFilter::Configuration::set_stat_interval(uint64_t interval) {
  stat_interval_ = interval;
}

void InOutFilter::Configuration::add_prefix(std::string prefix) {
  prefix_.push_back(prefix);
}

void InOutFilter::Configuration::set_policy(InOutFilter::Policy policy) {
  policy_ = policy;
}

//------------------------------------------------------------------------------
// class InOutFilter::Observation
//------------------------------------------------------------------------------
InOutFilter::Observation::Observation() {
  reset();
}

void InOutFilter::Observation::reset(void) {
  valid_ = false;
  time_s_ = 0;
  message_ = "";
}

//------------------------------------------------------------------------------
bool InOutFilter::Observation::get_valid(void) const {
  return (valid_);
}

uint64_t InOutFilter::Observation::get_time_s(void) const {
  return (time_s_);
}

std::string InOutFilter::Observation::get_message(void) const {
  return (std::string(message_));
}

//------------------------------------------------------------------------------
void InOutFilter::Observation::set_valid(bool valid) {
  valid_ = valid;
}

void InOutFilter::Observation::set_time_s(uint64_t time_s) {
  time_s_ = time_s;
}

void InOutFilter::Observation::set_message(const std::string& message) {
  message_ = message;
}

//------------------------------------------------------------------------------
// class InOutFilter::Statistics
//------------------------------------------------------------------------------
InOutFilter::Statistics::Statistics() {
  reset();
}

void InOutFilter::Statistics::reset(void) {
  valid_ = false;
  time_s_ = 0;
  duration_s_ = 0;
  flows_ = 0;
  in_in_ = 0;
  in_out_ = 0;
  out_in_ = 0;
  out_out_ = 0;
}

//------------------------------------------------------------------------------
bool InOutFilter::Statistics::get_valid(void) const {
  return (valid_);
}

uint64_t InOutFilter::Statistics::get_time_s(void) const {
  return (time_s_);
}

uint64_t InOutFilter::Statistics::get_duration_s(void) const {
  return (duration_s_);
}

uint64_t InOutFilter::Statistics::get_flows(void) const {
  return (flows_);
}

uint64_t InOutFilter::Statistics::get_in_in(void) const {
  return (in_in_);
}

uint64_t InOutFilter::Statistics::get_in_out(void) const {
  return (in_out_);
}

uint64_t InOutFilter::Statistics::get_out_in(void) const {
  return (out_in_);
}

uint64_t InOutFilter::Statistics::get_out_out(void) const {
  return (out_out_);
}

//------------------------------------------------------------------------------
void InOutFilter::Statistics::set_valid(bool valid) {
  valid_ = valid;
}

void InOutFilter::Statistics::set_time_s(uint64_t time_s) {
  time_s_ = time_s;
}

void InOutFilter::Statistics::set_duration_s(uint64_t duration_s) {
  duration_s_ = duration_s;
}

void InOutFilter::Statistics::inc_flows(void) {
  flows_++;
}

void InOutFilter::Statistics::inc_in_in(void) {
  in_in_++;
}

void InOutFilter::Statistics::inc_in_out(void) {
  in_out_++;
}

void InOutFilter::Statistics::inc_out_in(void) {
  out_in_++;
}

void InOutFilter::Statistics::inc_out_out(void) {
  out_out_++;
}


//------------------------------------------------------------------------------
// class InOutFilter
//------------------------------------------------------------------------------
InOutFilter::InOutFilter() {

  // -- CONFIGURATION ----------------------------------------------------------
  sem_init(&conf_critical_section_sem_, 0, 1);

  conf_available_ = false;
  conf_prefix_inside_.set_value_not_found(kOhter);

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
  data_container_pool_ = FlowContainerPool::instance();

  // OTHERS --------------------------------------------------------------------
  addr4_.from("0.0.0.0/32");
  addr6_.from("::1/128");
  policy_ = Configuration::kPolicyDefault;
}

InOutFilter::~InOutFilter() {
}


//------------------------------------------------------------------------------
// class InOutFilter CONFIGURATION
//------------------------------------------------------------------------------
void InOutFilter::conf_push(Configuration config) {
  fb_sem_wait(&conf_critical_section_sem_);
  // CRITICAL SECTION::START
  conf_in_.push(config);
  conf_available_ = true;
  fb_sem_post(&conf_critical_section_sem_);
  // CRITICAL SECTION::STOP
  return;
}

void InOutFilter::conf_pop(void) {
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

void InOutFilter::conf_add_prefix(std::string prefix) {
  if (prefix.size() > 0) {
    // we have at least one character
    // parse the string 
    // X.X.X.X/YY, X:X:X:X/YY 
    std::stringstream lineStream(prefix);
    std::string cell;
    while (std::getline(lineStream, cell, ',')) {
      // std::cout << "ADD: '" << cell << "'" << std::endl;
      // std::cout.flush();
      if (cell.size() < 3)
        continue;

      std::string addr_s, inout_s;

      std::stringstream cellStream(cell);
      std::getline(cellStream, addr_s, '@');
      std::getline(cellStream, inout_s, '@');

      Prefix addr;
      int inout = kUnknown;
      addr.from(addr_s);
      inout = atoi(inout_s.c_str());

      // std::cout << "Prefix: '" << addr_s << "' -> " << addr.to_s() << std::endl;
      // std::cout << "Value: '" << inout_s << "' -> " << inout << "'" << std::endl;

      if (!(inout == kInside or inout == kOhter)) {
        assert(false);
        // input validation failed;
        throw FlowBoxE("input validation failed??", __FILE__, __LINE__);
      }

      int in_range = conf_prefix_inside_.lookup(addr);
      if (in_range != kInside) {
        conf_prefix_inside_.insert(addr, inout);
      } else {
        // std::cout << "Already added " << endl;
      }
    }
  }
}

void InOutFilter::conf_migrate(const Configuration& migrate) {

  data_input_ = migrate.get_input();
  data_output_ = migrate.get_output();

  conf_prefix_inside_.clear();
  std::vector<std::string>::const_iterator prefix;
  std::vector<std::string>::const_iterator end;

  prefix = migrate.get_prefix_begin();
  end = migrate.get_prefix_end();
  while (prefix != end) {
    conf_add_prefix(*prefix);
    prefix++;
  };
  stat_interval_s_ = migrate.get_stat_interval();
  policy_ = migrate.get_policy();
  return;
}

//------------------------------------------------------------------------------
// class InOutFilter OBSERVATION
//------------------------------------------------------------------------------
InOutFilter::Observation InOutFilter::obs_get(void) {
  InOutFilter::Observation obs;
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
  return (InOutFilter::Observation(obs));
}
void InOutFilter::obs_push(uint64_t now_s, const std::string& message) {
  InOutFilter::Observation obs;
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
//------------------------------------------------------------------------------
// class InOutFilter STATISTICS
//------------------------------------------------------------------------------
InOutFilter::Statistics InOutFilter::stat_get(void) {
  InOutFilter::Statistics stat;
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
  return (InOutFilter::Statistics(stat));
}
void InOutFilter::stat_export(uint64_t now_s) {
  if (stat_current_.get_time_s() != 0) {
    stat_current_.set_duration_s(now_s - stat_current_.get_time_s());
    fb_sem_wait(&stat_critical_section_sem_);
    // CRITICAL SECTION::START
    stat_out_.push(stat_current_);  // push data
    fb_sem_post(&stat_critical_section_sem_);
    // CRITICAL SECTION::STOP

    fb_sem_post(&stat_available_sem_);
    // SIGNAL DATA
  };
  stat_current_.reset();
  stat_current_.set_time_s(now_s);
  stat_export_next_s_ = ((now_s / stat_interval_s_) + 1) * stat_interval_s_;
}
//------------------------------------------------------------------------------
// class InOutFilter DATA
//------------------------------------------------------------------------------
void InOutFilter::data_process(FlowContainer::iterator flow,
                               FlowContainer::iterator end) {
  // walk over flows
  while (flow != end) {
    if (flow->valid_ == true) {
      process(flow);
    };
    flow++;
  };
}

void InOutFilter::process(FlowContainer::iterator& flow) {

  // export stats?
  if (flow->start_s_ > stat_export_next_s_) {
    stat_export(flow->start_s_);
  }

  stat_current_.inc_flows();

  int src_is_inside;
  int dst_is_inside;

  if (flow->addr_length_ == (int) Flow::kAddressLengthIPv4) {
    addr4_.from_nb_zero(flow->addr_src_, 32, Prefix::kFamilyIPv4);
    src_is_inside = (conf_prefix_inside_.lookup(addr4_) == kInside);
    addr4_.from_nb_zero(flow->addr_dst_, 32, Prefix::kFamilyIPv4);
    dst_is_inside = (conf_prefix_inside_.lookup(addr4_) == kInside);
  } else if (flow->addr_length_ == (int) Flow::kAddressLengthIPv6) {
    addr6_.from_nb_zero(flow->addr_src_, 128, Prefix::kFamilyIPv6);
    src_is_inside = (conf_prefix_inside_.lookup(addr6_) == kInside);
    addr6_.from_nb_zero(flow->addr_dst_, 128, Prefix::kFamilyIPv6);
    dst_is_inside = (conf_prefix_inside_.lookup(addr6_) == kInside);
  } else {
    std::cout << "TopologyFilter::process unknown address length '"
              << flow->addr_length_ << "'" << std::endl;
    std::cout.flush();
    throw FlowBoxE("unknown address length??", __FILE__, __LINE__);
  }

  if (src_is_inside and !dst_is_inside) {
    flow->direction_ = Flow::kDirectionInOut;
    stat_current_.inc_in_out();
  } else if (!src_is_inside and dst_is_inside) {
    flow->direction_ = Flow::kDirectionOutIn;
    stat_current_.inc_out_in();
  } else {

    if (src_is_inside and dst_is_inside) {
      flow->direction_ = Flow::kDirectionInIn;
      stat_current_.inc_in_in();
    } else if (!src_is_inside and !dst_is_inside) {
      flow->direction_ = Flow::kDirectionOutOut;
      stat_current_.inc_out_out();
    } else {
      flow->direction_ = Flow::kDirectionUnknown;
      assert(false);
      // brocken logic
      throw FlowBoxE("kDirectionUnknown", __FILE__, __LINE__);
    }

    if (policy_ == kFilter)
      flow->valid_ = false;
  }
}


// MAIN LOOP -------------------------------------------------------------------
void InOutFilter::data_process(void) {

  obs_push(0, "data_process -- started");

  FlowContainer* fc;
  while (true) {

    // do we have some a new config?
    if (conf_available_) {
      conf_pop();
    }

    // ups no input buffer ...
    if (data_input_ == NULL) {
      std::cout << "TopologyFilter::data_thread_main -- empty data_input_"
                << std::endl;
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

        // push/flush our internal results
        // NOP

        // forward the signal to the next buffer (if possible)
        if (data_output_ != NULL) {
          data_output_->signal_fin();
        }

        // break the while(true) ...
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
      data_process(fc->begin(), fc->end_used());

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
}
