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

#include "border_filter.h"

//------------------------------------------------------------------------------
// class BorderFilter::Const
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// class BorderFilter::Configuration
//------------------------------------------------------------------------------
const int BorderFilter::Configuration::kStatisticDefaultInterval = 300;
const BorderFilter::Policy BorderFilter::Configuration::kPolicyDefault =
    BorderFilter::kTag;

BorderFilter::Configuration::Configuration() {
  reset();
}

void BorderFilter::Configuration::reset(void) {
  input_ = NULL;
  output_ = NULL;
  stat_interval_ = kStatisticDefaultInterval;
  interfaces_.clear();
  policy_ = kPolicyDefault;
}

//------------------------------------------------------------------------------
FlowContainerBuffer* BorderFilter::Configuration::get_input(void) const {
  return (input_);
}

FlowContainerBuffer* BorderFilter::Configuration::get_output(void) const {
  return (output_);
}

std::vector<BorderFilter::Interface>::const_iterator
BorderFilter::Configuration::get_interface_begin(void) const {
  return (interfaces_.begin());
}

std::vector<BorderFilter::Interface>::const_iterator
BorderFilter::Configuration::get_interface_end(void) const {
  return (interfaces_.end());
}

uint64_t BorderFilter::Configuration::get_stat_interval(void) const {
  return (stat_interval_);
}

BorderFilter::Policy BorderFilter::Configuration::get_policy(void) const {
  return (policy_);
}

//------------------------------------------------------------------------------
void BorderFilter::Configuration::set_input(FlowContainerBuffer* input) {
  input_ = input;
}

void BorderFilter::Configuration::set_output(FlowContainerBuffer* output) {
  output_ = output;
}

void BorderFilter::Configuration::set_stat_interval(uint64_t interval) {
  stat_interval_ = interval;
}

void BorderFilter::Configuration::add_interface(const Interface& interface) {
  interfaces_.push_back(interface);
}

void BorderFilter::Configuration::set_policy(BorderFilter::Policy policy) {
  policy_ = policy;
}

//------------------------------------------------------------------------------
// class BorderFilter::Observation
//------------------------------------------------------------------------------
BorderFilter::Observation::Observation() {
  reset();
}

void BorderFilter::Observation::reset(void) {
  valid_ = false;
  time_s_ = 0;
  message_ = "";
}

//------------------------------------------------------------------------------
bool BorderFilter::Observation::get_valid(void) const {
  return (valid_);
}

uint64_t BorderFilter::Observation::get_time_s(void) const {
  return (time_s_);
}

std::string BorderFilter::Observation::get_message(void) const {
  return (std::string(message_));
}

//------------------------------------------------------------------------------
void BorderFilter::Observation::set_valid(bool valid) {
  valid_ = valid;
}

void BorderFilter::Observation::set_time_s(uint64_t time_s) {
  time_s_ = time_s;
}

void BorderFilter::Observation::set_message(const std::string& message) {
  message_ = message;
}

//------------------------------------------------------------------------------
// class BorderFilter::Statistics
//------------------------------------------------------------------------------
BorderFilter::Statistics::Statistics() {
  reset();
}

void BorderFilter::Statistics::reset(void) {
  valid_ = false;
  time_s_ = 0;
  duration_s_ = 0;
  flows_ = 0;
  other_ = 0;
  border_in_ = 0;
  border_out_ = 0;
}

//------------------------------------------------------------------------------
bool BorderFilter::Statistics::get_valid(void) const {
  return (valid_);
}

uint64_t BorderFilter::Statistics::get_time_s(void) const {
  return (time_s_);
}

uint64_t BorderFilter::Statistics::get_duration_s(void) const {
  return (duration_s_);
}

uint64_t BorderFilter::Statistics::get_flows(void) const {
  return (flows_);
}

uint64_t BorderFilter::Statistics::get_other(void) const {
  return (other_);
}

uint64_t BorderFilter::Statistics::get_border_in(void) const {
  return (border_in_);
}

uint64_t BorderFilter::Statistics::get_border_out(void) const {
  return (border_out_);
}

//------------------------------------------------------------------------------
void BorderFilter::Statistics::set_valid(bool valid) {
  valid_ = valid;
}

void BorderFilter::Statistics::set_time_s(uint64_t time_s) {
  time_s_ = time_s;
}

void BorderFilter::Statistics::set_duration_s(uint64_t duration_s) {
  duration_s_ = duration_s;
}

void BorderFilter::Statistics::inc_flows(void) {
  flows_++;
}

void BorderFilter::Statistics::inc_other(void) {
  other_++;
}

void BorderFilter::Statistics::inc_border_in(void) {
  border_in_++;
}

void BorderFilter::Statistics::inc_border_out(void) {
  border_out_++;
}

//------------------------------------------------------------------------------
// class BorderFilter
//------------------------------------------------------------------------------
BorderFilter::BorderFilter() {
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
  stat_interval_s_ = Configuration::kStatisticDefaultInterval;
  stat_export_next_s_ = 0;

  // -- DATA STREAM  -----------------------------------------------------------
  data_input_ = NULL;
  data_output_ = NULL;
  data_container_pool_ = FlowContainerPool::instance();

  // OTHERS --------------------------------------------------------------------
  policy_ = Configuration::kPolicyDefault;
  map_.clear();
}

BorderFilter::~BorderFilter() {
  map_.clear();
}


//------------------------------------------------------------------------------
// class BorderFilter CONFIGURATION
//------------------------------------------------------------------------------
void BorderFilter::conf_push(Configuration config) {
  fb_sem_wait(&conf_critical_section_sem_);
  // CRITICAL SECTION::START
  conf_in_.push(config);
  conf_available_ = true;
  fb_sem_post(&conf_critical_section_sem_);
  // CRITICAL SECTION::STOP
  return;
}

void BorderFilter::conf_pop(void) {
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

void BorderFilter::conf_migrate(const Configuration& migrate) {
  data_input_ = migrate.get_input();
  data_output_ = migrate.get_output();

  map_.clear();
  std::vector<Interface>::const_iterator interface;
  std::vector<Interface>::const_iterator end;
  interface = migrate.get_interface_begin();
  end = migrate.get_interface_end();
  while (interface != end) {
    conf_add_interface(*interface);
    interface++;
  };

  stat_interval_s_ = migrate.get_stat_interval();
  policy_ = migrate.get_policy();
  return;
}

void BorderFilter::conf_add_interface(const Interface& interface) {
#ifdef BORDER_FILTER_DEBUG
  std::cout <<"ED ID: " << interface.export_device_id_  << std::endl;
  std::cout <<"IF ID: " << interface.interface_id_      << std::endl;
  std::cout <<"P    : " << interface.purpose_           << std::endl;
  std::cout << "c-before: " << map_.capacity() << std::endl;
  std::cout << "s-before: " << map_.size() << std::endl;
#endif

  if (interface.export_device_id_ + 1 > map_.size()) {
    std::vector<BorderFilter::Purpose> empty_v(1, BorderFilter::kOther);
    map_.resize(interface.export_device_id_ + 1, empty_v);
  }
#ifdef BORDER_FILTER_DEBUG
  std::cout << "c-after: " << map_.capacity() << std::endl;
  std::cout << "s-after: " << map_.size() << std::endl;
  std::cout << "c[" << interface.export_device_id_ << "]: "
      << (map_[interface.export_device_id_]).capacity() << std::endl;
  std::cout << "s[" << interface.export_device_id_ << "]: "
      << (map_[interface.export_device_id_]).size() << std::endl;
#endif

  if (interface.interface_id_ + 1
      >= (map_[interface.export_device_id_]).size()) {
    (map_[interface.export_device_id_]).resize(interface.interface_id_ + 1,
                                               BorderFilter::kOther);
  }
#ifdef BORDER_FILTER_DEBUG
  std::cout << "c[" << interface.export_device_id_ << "]: "
      << (map_[interface.export_device_id_]).capacity() << std::endl;
  std::cout << "s[" << interface.export_device_id_ << "]: "
      << (map_[interface.export_device_id_]).size() << std::endl;
#endif
  (map_.at(interface.export_device_id_)).at(interface.interface_id_) =
      interface.purpose_;
}


//------------------------------------------------------------------------------
// class BorderFilter OBSERVATION
//------------------------------------------------------------------------------
BorderFilter::Observation BorderFilter::obs_get(void) {
  BorderFilter::Observation obs;
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
  return (BorderFilter::Observation(obs));
}

void BorderFilter::obs_push(uint64_t now_s, const std::string& message) {
  BorderFilter::Observation obs;
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
// class BorderFilter STATISTICS
//------------------------------------------------------------------------------
BorderFilter::Statistics BorderFilter::stat_get(void) {
  BorderFilter::Statistics stat;
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
  return (BorderFilter::Statistics(stat));
}

void BorderFilter::stat_export(uint64_t now_s) {
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
// class BorderFilter DATA
//------------------------------------------------------------------------------
void BorderFilter::data_process(FlowContainer::iterator flow,
                                FlowContainer::iterator end) {
  // walk over flows
  while (flow != end) {
    if (flow->valid_ == true) {
      process(flow);
    };
    flow++;
  };
}

void BorderFilter::process(const FlowContainer::iterator& flow) {
  // export stats?
  if (flow->start_s_ > stat_export_next_s_) {
    stat_export(flow->start_s_);
  }

  stat_current_.inc_flows();

  // filter the flows
  if (flow->export_device_id_ < map_.size()) {
    // input interface == border?
    if (flow->if_in_ < map_[flow->export_device_id_].size()) {
      if (map_[flow->export_device_id_][flow->if_in_] == kBorder) {
        stat_current_.inc_border_in();
        flow->border_ = Flow::kBorderIfIn;
        return;
      }
    }

    // output interface == border?
    if (flow->if_out_ < map_[flow->export_device_id_].size()) {
      if (map_[flow->export_device_id_][flow->if_out_] == kBorder) {
        stat_current_.inc_border_out();
        flow->border_ = Flow::kBorderIfOut;
        return;
      }
    }
  }

  stat_current_.inc_other();
  flow->border_ = Flow::kBorderUnknown;
  if (policy_ == kFilter)
    flow->valid_ = false;
  return;
}


// MAIN LOOP -------------------------------------------------------------------
void BorderFilter::data_process(void) {
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
  obs_push(0, "data_thread_main -- finished");
}
