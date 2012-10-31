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
 * @file   ip_filter.cc
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @date   October, 2012
 * @brief  Filter out certain IP addresses ranges.
 *
 * The IPFilter can be used to filter out certain IP ranges such as bogus IP
 * space or PlanetLab host, that would affect the quality of the measurement.
 */

#include "ip_filter.h"
//------------------------------------------------------------------------------
// class IPFilter::Configuration
//------------------------------------------------------------------------------
const int IPFilter::Configuration::kStatisticDefaultInterval = 300;
const IPFilter::Policy IPFilter::Configuration::KDefaultPolicy = IPFilter::kKeep;
IPFilter::Configuration::Configuration() {
  reset();
}

void IPFilter::Configuration::reset(void) {
  input_ = NULL;
  output_ = NULL;
  stat_interval_ = kStatisticDefaultInterval;
  prefix_.clear();
  default_policy_ = KDefaultPolicy;
}

//------------------------------------------------------------------------------
FlowContainerBuffer* IPFilter::Configuration::get_input(void) const {
  return (input_);
}

FlowContainerBuffer* IPFilter::Configuration::get_output(void) const {
  return (output_);
}

std::vector<std::string>::const_iterator
IPFilter::Configuration::get_prefix_begin(void) const {
  return (prefix_.begin());
}

std::vector<std::string>::const_iterator
IPFilter::Configuration::get_prefix_end(void) const {
  return (prefix_.end());
}

uint64_t IPFilter::Configuration::get_stat_interval(void) const {
  return (stat_interval_);
}

IPFilter::Policy IPFilter::Configuration::get_default_policy(void) const {
  return (default_policy_);
}

//------------------------------------------------------------------------------
void IPFilter::Configuration::set_input(FlowContainerBuffer* input) {
  input_ = input;
}

void IPFilter::Configuration::set_output(FlowContainerBuffer* output) {
  output_ = output;
}

void IPFilter::Configuration::set_stat_interval(uint64_t interval) {
  stat_interval_ = interval;
}

void IPFilter::Configuration::set_default_policy(Policy policy) {
  default_policy_ = policy;
}

void IPFilter::Configuration::add_prefix(std::string prefix) {
  prefix_.push_back(prefix);
}

//------------------------------------------------------------------------------
// class IPFilter::Observation
//------------------------------------------------------------------------------
IPFilter::Observation::Observation() {
  reset();
}

void IPFilter::Observation::reset(void) {
  valid_ = false;
  time_s_ = 0;
  message_ = "";
}

//------------------------------------------------------------------------------
bool IPFilter::Observation::get_valid(void) const {
  return (valid_);
}

uint64_t IPFilter::Observation::get_time_s(void) const {
  return (time_s_);
}

std::string IPFilter::Observation::get_message(void) const {
  return (std::string(message_));
}

//------------------------------------------------------------------------------
void IPFilter::Observation::set_valid(bool valid) {
  valid_ = valid;
}

void IPFilter::Observation::set_time_s(uint64_t time_s) {
  time_s_ = time_s;
}

void IPFilter::Observation::set_message(const std::string& message) {
  message_ = message;
}

//------------------------------------------------------------------------------
// class IPFilter::Statistics
//------------------------------------------------------------------------------
IPFilter::Statistics::Statistics() {
  reset();
}

void IPFilter::Statistics::reset(void) {
  valid_ = false;
  time_s_ = 0;
  duration_s_ = 0;
  flows_ = 0;
  flows_filtered_ = 0;
}

//------------------------------------------------------------------------------
bool IPFilter::Statistics::get_valid(void) const {
  return (valid_);
}

uint64_t IPFilter::Statistics::get_time_s(void) const {
  return (time_s_);
}

uint64_t IPFilter::Statistics::get_duration_s(void) const {
  return (duration_s_);
}

uint64_t IPFilter::Statistics::get_flows(void) const {
  return (flows_);
}

uint64_t IPFilter::Statistics::get_flows_filtered(void) const {
  return (flows_filtered_);
}

//------------------------------------------------------------------------------
void IPFilter::Statistics::set_valid(bool valid) {
  valid_ = valid;
}

void IPFilter::Statistics::set_time_s(uint64_t time_s) {
  time_s_ = time_s;
}

void IPFilter::Statistics::set_duration_s(uint64_t duration_s) {
  duration_s_ = duration_s;
}

void IPFilter::Statistics::inc_flows(void) {
  flows_++;
}

void IPFilter::Statistics::inc_flows_filtered(void) {
  flows_filtered_++;
}

//------------------------------------------------------------------------------
// class IPFilter
//------------------------------------------------------------------------------
IPFilter::IPFilter() {
  // -- CONFIGURATION ----------------------------------------------------------
  sem_init(&conf_critical_section_sem_, 0, 1);

  conf_available_ = false;
  prefixes_.set_value_not_found(kNoMatch);

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
  default_policy_ = Configuration::KDefaultPolicy;
}

IPFilter::~IPFilter() {
}

//------------------------------------------------------------------------------
// class IPFilter CONFIGURATION
//------------------------------------------------------------------------------
void IPFilter::conf_push(Configuration config) {
  fb_sem_wait(&conf_critical_section_sem_);
  // CRITICAL SECTION::START
  conf_in_.push(config);
  conf_available_ = true;
  fb_sem_post(&conf_critical_section_sem_);
  // CRITICAL SECTION::STOP
  return;
}

void IPFilter::conf_pop(void) {
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

void IPFilter::conf_add_prefix(std::string prefix) {
  if (prefix.size() > 0) {
    // we have at least one character
    // parse the string
    // X.X.X.X/YY, X:X:X:X/YY
    std::stringstream lineStream(prefix);
    std::string cell;
    while (std::getline(lineStream, cell, ',')) {
      if (cell.size() < 3)
        continue;

      std::string addr_s, inout_s;
      std::stringstream cellStream(cell);
      std::getline(cellStream, addr_s, '@');
      std::getline(cellStream, inout_s, '@');

      Prefix addr;
      int filter;
      addr.from(addr_s);
      filter = atoi(inout_s.c_str());

      if (!(filter == kKeep or filter == kFilter)) {
        std::stringstream err_msg;
        err_msg   << "input validation failed?? "<<  std::endl
                  << "E::Prefix: '" << addr_s << "' -> " << addr.to_s()
                  << std::endl
                  << "E::Value: '" << inout_s.c_str() << "' -> " << "'"
                  << std::endl;
        throw FlowBoxE(err_msg.str(), __FILE__, __LINE__);
      }

      prefixes_.insert(addr, filter);

      /*
       int in_range = prefixes_.lookup(addr);
       if(in_range != filter) {
       prefixes_.insert(addr, filter);
       } else {
       std::cout << "Already coverd ?!?" << addr.to_s() << std::endl;
       std::cout.flush();
       throw FlowBoxE("input validation failed??",__FILE__, __LINE__);
       }
       */
    }
  }
}

void IPFilter::conf_migrate(const Configuration& migrate) {
  data_input_ = migrate.get_input();
  data_output_ = migrate.get_output();

  prefixes_.clear();
  std::vector<std::string>::const_iterator prefix;
  std::vector<std::string>::const_iterator end;

  prefix = migrate.get_prefix_begin();
  end = migrate.get_prefix_end();
  while (prefix != end) {
    conf_add_prefix(*prefix);
    prefix++;
  };
  stat_interval_s_ = migrate.get_stat_interval();
  default_policy_ = migrate.get_default_policy();
  return;
}

//------------------------------------------------------------------------------
// class IPFilter OBSERVATION
//------------------------------------------------------------------------------
IPFilter::Observation IPFilter::obs_get(void) {
  IPFilter::Observation obs;
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
  return (IPFilter::Observation(obs));
}

void IPFilter::obs_push(const uint64_t now_s, const std::string& message) {
  IPFilter::Observation obs;
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
// class IPFilter STATISTICS
//------------------------------------------------------------------------------
IPFilter::Statistics IPFilter::stat_get(void) {
  IPFilter::Statistics stat;
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
  return (IPFilter::Statistics(stat));
}
void IPFilter::stat_export(uint64_t now_s) {
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
// class IPFilter DATA
//------------------------------------------------------------------------------
void IPFilter::data_process(FlowContainer::iterator flow,
                            FlowContainer::iterator end) {
  // walk over flows
  while (flow != end) {
    if (flow->valid_ == true) {
      process(flow);
    }
    flow++;
  }
}

void IPFilter::process(const FlowContainer::iterator& flow) {
  // export stats?
  if (flow->start_s_ > stat_export_next_s_) {
    stat_export(flow->start_s_);
  }

  stat_current_.inc_flows();

  // check prefixes
  int src_policy = kNoMatch;
  int dst_policy = kNoMatch;

  if (flow->addr_length_ == static_cast<uint8_t>(Flow::kAddressLengthIPv4)) {
    addr4_.from_nb_zero(flow->addr_src_, 32, Prefix::kFamilyIPv4);
    src_policy = prefixes_.lookup(addr4_);
    addr4_.from_nb_zero(flow->addr_dst_, 32, Prefix::kFamilyIPv4);
    dst_policy = prefixes_.lookup(addr4_);
  } else if (flow->addr_length_ == static_cast<uint8_t>(Flow::kAddressLengthIPv6)) {
    addr6_.from_nb_zero(flow->addr_src_, 128, Prefix::kFamilyIPv6);
    src_policy = prefixes_.lookup(addr6_);
    addr6_.from_nb_zero(flow->addr_dst_, 128, Prefix::kFamilyIPv6);
    dst_policy = prefixes_.lookup(addr6_);
  } else {
    std::stringstream err_msg;
    err_msg << "IPFilter::process unknown address length '"
            << flow->addr_length_ << "'" << std::endl;
    throw FlowBoxE(err_msg.str(), __FILE__, __LINE__);
  }

  // apply policy
  if (src_policy == kNoMatch and dst_policy == kNoMatch) {
    // apply default policy
    if (default_policy_ == kFilter) {
      flow->valid_ = false;
      stat_current_.inc_flows_filtered();
    }
    return;
  }

#ifdef IP_FILTER_DEBUG
  std::cout << "IPFilter -- ip match"<< std::endl;
  std::cout << "SRC P: " << src_policy << std::endl;
  std::cout << "DST P: " << dst_policy << std::endl;
  std::cout.flush();
#endif

  if (src_policy == kFilter and dst_policy == kNoMatch) {
    flow->valid_ = false;
    stat_current_.inc_flows_filtered();
  } else if (src_policy == kKeep and dst_policy == kNoMatch) {
    // keep
  } else if (src_policy == kNoMatch and dst_policy == kFilter) {
    flow->valid_ = false;
    stat_current_.inc_flows_filtered();
  } else if (src_policy == kNoMatch and dst_policy == kKeep) {
    // keep
  } else if (src_policy == kFilter and dst_policy == kFilter) {
    flow->valid_ = false;
    stat_current_.inc_flows_filtered();
  } else if (src_policy == kKeep and dst_policy == kKeep) {
    // keep
  } else if (src_policy == kKeep and dst_policy == kFilter) {
    // apply inverse of default
    if (default_policy_ == kKeep) {
      flow->valid_ = false;
      stat_current_.inc_flows_filtered();
    }
  } else if (src_policy == kFilter and dst_policy == kKeep) {
    // apply inverse of default
    if (default_policy_ == kKeep) {
      flow->valid_ = false;
      stat_current_.inc_flows_filtered();
    }
  } else {
    throw FlowBoxE("Broken Logic?", __FILE__, __LINE__);
  }
}

// MAIN LOOP -------------------------------------------------------------------
void IPFilter::data_process(void) {
  obs_push(0, "data_process -- started");

  FlowContainer* fc;
  while (true) {
    // do we have some a new config?
    if (conf_available_) {
      conf_pop();
    }

    // ups no input buffer ...
    if (data_input_ == NULL) {
#ifdef IP_FILTER_DEBUG
      std::cout << "IPFilter::data_thread_main -- empty data_input_"
                << std::endl;
#endif
      obs_push(0, "data_thread_main -- empty data_input_");
      sleep(5);
      continue;
    }

    // blocking wait on some input
    fc = data_input_->pop();

    // since the locking wait can be interrupted
    // by system interrupts, timeouts or signals
    // we have to check if there is the pointer is valid.
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

  // clean up
  // NOP
  obs_push(0, "data_thread_main -- finished");
}

