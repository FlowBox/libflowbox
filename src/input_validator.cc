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
 * @file   input_validator.cc
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @date   October, 2012
 * @brief  Check if the data contains unrealistic or insane Flow Records.
 *
 * The world is not perfect. This unit tries to spot flows that are caused by
 * the imperfection of the measurement system such as zero byte flows.
 *
 */

#include "input_validator.h"
//------------------------------------------------------------------------------
// class InputValidator::Configuration
//------------------------------------------------------------------------------
const int InputValidator::Configuration::kStatIntervalDefault = 300;
const int InputValidator::Configuration::kSlidingWindowDefault = 600;

InputValidator::Configuration::Configuration() {
  reset();
}

void InputValidator::Configuration::reset(void) {
  input_ = NULL;
  output_ = NULL;
  stat_interval_s_ = kStatIntervalDefault;
  sliding_window_s_ = kSlidingWindowDefault;
}

//------------------------------------------------------------------------------
FlowContainerBuffer* InputValidator::Configuration::get_input(void) const {
  return (input_);
}

FlowContainerBuffer* InputValidator::Configuration::get_output(void) const {
  return (output_);
}

uint64_t InputValidator::Configuration::get_stat_interval_s(void) const {
  return (stat_interval_s_);
}

uint64_t InputValidator::Configuration::get_sliding_window_s(void) const {
  return (sliding_window_s_);
}

//------------------------------------------------------------------------------
void InputValidator::Configuration::set_input(FlowContainerBuffer* input) {
  input_ = input;
}

void InputValidator::Configuration::set_output(FlowContainerBuffer* output) {
  output_ = output;
}

void InputValidator::Configuration::set_stat_interval(
    uint64_t stat_interval_s) {
  stat_interval_s_ = stat_interval_s;
}

void InputValidator::Configuration::set_sliding_window_s(
    uint64_t sliding_window_s) {
  sliding_window_s_ = sliding_window_s;
}

//------------------------------------------------------------------------------
// class InputValidator::Observation
//------------------------------------------------------------------------------
InputValidator::Observation::Observation() {
  reset();
}

void InputValidator::Observation::reset(void) {
  valid_ = false;
  time_s_ = 0;
  message_ = "";
}

//------------------------------------------------------------------------------
bool InputValidator::Observation::get_valid(void) const {
  return (valid_);
}

uint64_t InputValidator::Observation::get_time_s(void) const {
  return (time_s_);
}

std::string InputValidator::Observation::get_message(void) const {
  return (std::string(message_));
}

//------------------------------------------------------------------------------
void InputValidator::Observation::set_valid(bool valid) {
  valid_ = valid;
}

void InputValidator::Observation::set_time_s(uint64_t time_s) {
  time_s_ = time_s;
}

void InputValidator::Observation::set_message(const std::string& message) {
  message_ = message;
}

//------------------------------------------------------------------------------
// class InputValidator::Statistics
//------------------------------------------------------------------------------
InputValidator::Statistics::Statistics() {
  reset();
}

void InputValidator::Statistics::reset(void) {
  valid_ = false;
  time_s_ = 0;
  duration_s_ = 0;
  flows_ = 0;
  iv_time_freshness_ = 0;
  iv_time_causality_ = 0;
  iv_bytes_ = 0;
  iv_packets_ = 0;
  iv_mtu_ = 0;
}

//------------------------------------------------------------------------------
bool InputValidator::Statistics::get_valid(void) const {
  return (valid_);
}

uint64_t InputValidator::Statistics::get_time_s(void) const {
  return (time_s_);
}

uint64_t InputValidator::Statistics::get_duration_s(void) const {
  return (duration_s_);
}

uint64_t InputValidator::Statistics::get_flows(void) const {
  return (flows_);
}

uint64_t InputValidator::Statistics::get_iv_time_freshness(void) const {
  return (iv_time_freshness_);
}

uint64_t InputValidator::Statistics::get_iv_time_causality(void) const {
  return (iv_time_causality_);
}

uint64_t InputValidator::Statistics::get_iv_bytes(void) const {
  return (iv_bytes_);
}

uint64_t InputValidator::Statistics::get_iv_packets(void) const {
  return (iv_packets_);
}

uint64_t InputValidator::Statistics::get_iv_mtu(void) const {
  return (iv_mtu_);
}

//------------------------------------------------------------------------------
void InputValidator::Statistics::set_valid(bool valid) {
  valid_ = valid;
}

void InputValidator::Statistics::set_time_s(uint64_t time_s) {
  time_s_ = time_s;
}

void InputValidator::Statistics::set_duration_s(uint64_t duration_s) {
  duration_s_ = duration_s;
}

void InputValidator::Statistics::inc_flows(void) {
  flows_++;
}

void InputValidator::Statistics::inc_iv_time_freshness(void) {
  iv_time_freshness_++;
}

void InputValidator::Statistics::inc_iv_time_causality(void) {
  iv_time_freshness_++;
}

void InputValidator::Statistics::inc_iv_bytes(void) {
  iv_bytes_++;
}

void InputValidator::Statistics::inc_iv_packets(void) {
  iv_packets_++;
}

void InputValidator::Statistics::inc_iv_mtu(void) {
  iv_mtu_++;
}

//------------------------------------------------------------------------------
std::string InputValidator::Statistics::head_to_s(void) const {
  std::stringstream head;
  head << "time_s, ";
  head << "duration_s, ";
  head << "flows, ";
  head << "iv_time_freshness, ";
  head << "iv_time_causality, ";
  head << "iv_bytes_, ";
  head << "iv_packets_, ";
  head << "iv_mtu";
  return (std::string(head.str()));
}

std::string InputValidator::Statistics::to_s(void) const {
  std::stringstream stat;
  stat << time_s_ << ", ";
  stat << duration_s_ << ", ";
  stat << flows_ << ", ";
  stat << iv_time_freshness_ << ", ";
  stat << iv_time_causality_ << ", ";
  stat << iv_bytes_ << ", ";
  stat << iv_packets_ << ", ";
  stat << iv_mtu_;
  return (std::string(stat.str()));
}

//------------------------------------------------------------------------------
// class InputValidator
//------------------------------------------------------------------------------
InputValidator::InputValidator() {
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
  stat_interval_s_ = Configuration::kStatIntervalDefault;
  stat_export_next_s_ = 0;

  // -- DATA STREAM  -----------------------------------------------------------
  data_input_ = NULL;
  data_output_ = NULL;
  data_container_pool_ = FlowContainerPool::instance();

  // OTHERS --------------------------------------------------------------------
  flow_time_s_ = 0;
  flow_time_last_jump_s_ = 0;
  sliding_window_s_ = Configuration::kSlidingWindowDefault;
}

InputValidator::~InputValidator() {
}

//------------------------------------------------------------------------------
// class InputValidator CONFIGURATION
//------------------------------------------------------------------------------
void InputValidator::conf_push(Configuration config) {
  fb_sem_wait(&conf_critical_section_sem_);
  // CRITICAL SECTION::START
  conf_in_.push(config);
  conf_available_ = true;
  fb_sem_post(&conf_critical_section_sem_);
  // CRITICAL SECTION::STOP
  return;
}

void InputValidator::conf_pop(void) {
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

void InputValidator::conf_migrate(const Configuration& migrate) {
  obs_push("Configuration migration");
  data_input_ = migrate.get_input();
  data_output_ = migrate.get_output();
  stat_interval_s_ = migrate.get_stat_interval_s();
  sliding_window_s_ = migrate.get_sliding_window_s();
  return;
}

//------------------------------------------------------------------------------
// class InputValidator OBSERVATION
//------------------------------------------------------------------------------
InputValidator::Observation InputValidator::obs_get(void) {
  InputValidator::Observation obs;
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
  return (InputValidator::Observation(obs));
}
void InputValidator::obs_push(const std::string& message) {
  InputValidator::Observation obs;
  obs.set_time_s(flow_time_s_);
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
// class InputValidator STATISTICS
//------------------------------------------------------------------------------
InputValidator::Statistics InputValidator::stat_get(void) {
  InputValidator::Statistics stat;
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
  return (InputValidator::Statistics(stat));
}
void InputValidator::stat_export(uint64_t now_s) {
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
// class InputValidator DATA
//------------------------------------------------------------------------------
void InputValidator::data_process(FlowContainer::iterator flow,
                                  FlowContainer::iterator end) {
  // walk over flows
  while (flow != end) {
    if (flow->valid_ == true) {
      data_process(flow);
    }
    flow++;
  }
}

void InputValidator::report(std::string reason,
                            const FlowContainer::iterator& flow) {
  std::stringstream tmp;
  tmp << "reason: " << reason << ", ";
  tmp << "flow_time_s: " << flow_time_s_ << ", ";
  tmp << "flow_time_last_jump_s_: " << flow_time_last_jump_s_ << ", ";
  int64_t diff_s = get_difference(flow->stop_s_, flow->start_s_);
  tmp << "diff_s_: " << diff_s << ", ";
  int64_t step_s = get_difference(flow->stop_s_, flow_time_s_);
  tmp << "step_s_: " << step_s << ", ";
  tmp << "flow: " << flow->to_s();
  obs_push(tmp.str());
}

void InputValidator::data_process(const FlowContainer::iterator& flow) {
  // Time Causality
  int64_t diff_s = get_difference(flow->stop_s_, flow->start_s_);
  if (diff_s < 0 or diff_s > 600) {
    report("time_causality", flow);
    // std::cout << "Time Causality? : " << diff_s << std::endl;
    stat_current_.inc_iv_time_causality();
    flow->valid_ = false;
    return;
  }

  // continues time?
  int64_t step_s = get_difference(flow->stop_s_, flow_time_s_);
  if (step_s < -sliding_window_s_ or step_s > sliding_window_s_) {
    report("not_continues_in_time", flow);

    // not continues -- WHY?
    // 1) init
    if (flow_time_s_ == 0) {
      // std::cout << "Time Freshness :: init flow_time_s_ = "
      // << flow->stop_s_ << std::endl;
      flow_time_s_ = flow->stop_s_;
      flow_time_last_jump_s_ = flow->stop_s_;

      // 2) jump backwards
    } else if (step_s < 0) {
      // stat_current_.inc_iv_export_time_too_small();
      stat_current_.inc_iv_time_freshness();
      flow->valid_ = false;
      return;

      // 3) jump forward?
    } else if (step_s > 0) {
      // how big is the jump from the last jump
      int64_t step_last_s = get_difference(flow->stop_s_,
                                           flow_time_last_jump_s_);
      if (step_last_s < -sliding_window_s_ or step_last_s > sliding_window_s_) {
        // 3a) first first jump, reject flow
        flow_time_last_jump_s_ = flow->stop_s_;
        //stat_current_.inc_iv_export_time_too_big();
        stat_current_.inc_iv_time_freshness();
        flow->valid_ = false;
        return;

      } else {
        // 3b) we jumped again, do the jump! accept flow
        flow_time_s_ = flow->stop_s_;
      }
    } else {
      throw FlowBoxE("Application Logic Broken ?!?", __FILE__, __LINE__);
    }
  }

  // now we can trust the flow->stop_ -- statistic export?
  flow_time_s_ = flow->stop_s_;

  if (((uint64_t) flow_time_s_) >= stat_export_next_s_) {
    stat_export(flow_time_s_);
  }

  // Zero Packets Flows
  // On some plattforms those flows are exported. Exact reason unknown.
  if (flow->packets_ < 1) {
    // std::cout << "Packets? : " << flow->packets_  << std::endl;
    // report("Packets?", flow);
    stat_current_.inc_iv_packets();
    flow->valid_ = false;
    return;
  }

  // Bytes
  if (flow->bytes_ < 20 or (flow->protocol_ == 6 and flow->bytes_ < 40)
      or (flow->protocol_ == 6 and flow->bytes_ < 28)) {
    // std::cout << "Bytes? : " << flow->bytes_
    // << " Bytes with prot = " << (int) flow->protocol_ << std::endl;
    report("Bytes?", flow);
    stat_current_.inc_iv_bytes();
    flow->valid_ = false;
    return;
  }

  // MTU
  int mtu = flow->bytes_ / flow->packets_;
  // http://en.wikipedia.org/wiki/Maximum_transmission_unit
  // http://www.cisco.com/en/US/products/hw/switches/ps700/products_configuration_example09186a008010edab.shtml
  if (mtu > 9216) {  // Ethernet Jumbo Frames
    // std::cout << "MTU? : " << mtu << std::endl;
    report("MTU?", flow);
    stat_current_.inc_iv_mtu();
    flow->valid_ = false;
    return;
  }
  // Bitrate
  int mbps = flow->bytes_ * 8 / 1000000;
  if (diff_s > 0) {
    mbps = mbps / diff_s;
  }
  if (mbps > 10000) {  // < 10Gbits
    // std::cout << "Bitrate? : " << mbps << " Mbit/s" << std::endl;
    report("Bitrate?", flow);
    flow->valid_ = false;
    return;
  }
  // Packet rate
  int kpps = flow->packets_ / 1000;
  if (diff_s > 0) {
    kpps = kpps / diff_s;
  }
  if (kpps > 10000) {  // Ethernet Jumbo Frames
    // std::cout << "Packet Rate? : " << kpps << " kPacket/s" << std::endl;
    report("Packet Rate?", flow);
    flow->valid_ = false;
    return;
  }

  stat_current_.inc_flows();
}

void InputValidator::data_process(void) {
  obs_push("data_process -- started");

  FlowContainer* fc;
  while (true) {
    // do we have some a new config?
    if (conf_available_)
      conf_pop();

    // ups no input buffer ...
    if (data_input_ == NULL) {
      obs_push("data_thread_main -- empty data_input_");
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
        obs_push("data_thread_main -- empty container?!?");
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
  obs_push("data_thread_main -- finished");
}

