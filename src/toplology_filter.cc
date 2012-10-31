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
 * @file   topology_filter.cc
 * @author Dominik Schatzmann <schadomi@gmail.com>
 *
 * @date   October, 2012
 * @brief  Extract the network topology from the flow data.
 *
 * This class performs an analysis of the network topology and filters
 * flows according to the topology.
 *
 */

#include "topology_filter.h"
//------------------------------------------------------------------------------
// class TopologyFilter::Const
//------------------------------------------------------------------------------
const int TopologyFilter::kNoIndex = -1;

//------------------------------------------------------------------------------
// class TopologyFilter::Configuration
//------------------------------------------------------------------------------
const int TopologyFilter::Configuration::kStatisticDefaultInterval = 300;

TopologyFilter::Configuration::Configuration() {
  reset();
}

void TopologyFilter::Configuration::reset(void) {
  input_ = NULL;
  output_ = NULL;
  stat_interval_ = kStatisticDefaultInterval;
  interfaces_.clear();
}

//------------------------------------------------------------------------------
FlowContainerBuffer* TopologyFilter::Configuration::get_input(void) const {
  return (input_);
}

FlowContainerBuffer* TopologyFilter::Configuration::get_output(void) const {
  return (output_);
}

uint64_t TopologyFilter::Configuration::get_stat_interval(void) const {
  return (stat_interval_);
}

std::vector<TopologyFilter::Configuration::Interface>::const_iterator
TopologyFilter::Configuration::get_interfaces_begin(void) const {
  return (std::vector<TopologyFilter::Configuration::Interface>::const_iterator(
      interfaces_.begin()));
}

std::vector<TopologyFilter::Configuration::Interface>::const_iterator
TopologyFilter::Configuration::get_interfaces_end(void) const {
  return (std::vector<TopologyFilter::Configuration::Interface>::const_iterator(
      interfaces_.end()));
}

//------------------------------------------------------------------------------
void TopologyFilter::Configuration::set_input(FlowContainerBuffer* input) {
  input_ = input;
}

void TopologyFilter::Configuration::set_output(FlowContainerBuffer* output) {
  output_ = output;
}

void TopologyFilter::Configuration::set_stat_interval(uint64_t interval) {
  stat_interval_ = interval;
}

void TopologyFilter::Configuration::add_interface(int export_device_id,
                                                  int snmp_if_id, int filter,
                                                  std::string addr_ranges) {
  TopologyFilter::Configuration::Interface interface;
  interface.export_device_id_ = export_device_id;
  interface.snmp_if_id_ = snmp_if_id;
  interface.filter_ = filter;
  interface.addr_ranges_ = addr_ranges;

  interfaces_.push_back(interface);
}

//------------------------------------------------------------------------------
// class TopologyFilter::Observation
//------------------------------------------------------------------------------
TopologyFilter::Observation::Observation() {
  reset();
}

void TopologyFilter::Observation::reset(void) {
  valid_ = false;
  time_s_ = 0;
  export_device_id_ = 0;
  snmp_if_id_ = 0;
  state_ = 0;
  observed_addrs_ = "";
}

//------------------------------------------------------------------------------
bool TopologyFilter::Observation::get_valid(void) const {
  return (valid_);
}

uint64_t TopologyFilter::Observation::get_time_s(void) const {
  return (time_s_);
}

int TopologyFilter::Observation::get_export_device_id(void) const {
  return (export_device_id_);
}

int TopologyFilter::Observation::get_snmp_if_id(void) const {
  return (snmp_if_id_);
}

int TopologyFilter::Observation::get_state(void) const {
  return (state_);
}

std::string TopologyFilter::Observation::get_observed_addrs(void) const {
  return (std::string(observed_addrs_));
}

//------------------------------------------------------------------------------
void TopologyFilter::Observation::set_valid(bool valid) {
  valid_ = valid;
}

void TopologyFilter::Observation::set_time_s(uint64_t time_s) {
  time_s_ = time_s;
}

void TopologyFilter::Observation::set_export_device_id(int export_device_id) {
  export_device_id_ = export_device_id;
}

void TopologyFilter::Observation::set_snmp_if_id(int snmp_if_id) {
  snmp_if_id_ = snmp_if_id;
}

void TopologyFilter::Observation::set_state(int state) {
  state_ = state;
}

void TopologyFilter::Observation::set_observed_addrs(
    std::string observed_addrs) {
  observed_addrs_ = observed_addrs;
}

//------------------------------------------------------------------------------
// class TopologyFilter::Statistics
//------------------------------------------------------------------------------
TopologyFilter::Statistics::Statistics() {
  reset();
}

void TopologyFilter::Statistics::reset(void) {
  valid_ = false;
  time_s_ = 0;
  duration_s_ = 0;
  flows_ = 0;
  flows_filtered_ = 0;
}

//------------------------------------------------------------------------------
bool TopologyFilter::Statistics::get_valid(void) const {
  return (valid_);
}

uint64_t TopologyFilter::Statistics::get_time_s(void) const {
  return (time_s_);
}

uint64_t TopologyFilter::Statistics::get_duration_s(void) const {
  return (duration_s_);
}

uint64_t TopologyFilter::Statistics::get_flows(void) const {
  return (flows_);
}

uint64_t TopologyFilter::Statistics::get_flows_filtered(void) const {
  return (flows_filtered_);
}

//------------------------------------------------------------------------------
void TopologyFilter::Statistics::set_valid(bool valid) {
  valid_ = valid;
}

void TopologyFilter::Statistics::set_time_s(uint64_t time_s) {
  time_s_ = time_s;
}

void TopologyFilter::Statistics::set_duration_s(uint64_t duration_s) {
  duration_s_ = duration_s;
}

void TopologyFilter::Statistics::set_flows(uint64_t flows) {
  flows_ = flows;
}

void TopologyFilter::Statistics::set_flows_filtered(uint64_t flows_filtered) {
  flows_filtered_ = flows_filtered;
}

void TopologyFilter::Statistics::inc_flows(void) {
  flows_++;
}

void TopologyFilter::Statistics::inc_flows_filtered(void) {
  flows_filtered_++;
}

//------------------------------------------------------------------------------
// class NextHopAddr
//------------------------------------------------------------------------------
TopologyFilter::NextHopAddr::NextHopAddr() {
  reset();
}

TopologyFilter::NextHopAddr::NextHopAddr(const char* addr, const int length) {
  reset();
  set(addr, length);
}

void TopologyFilter::NextHopAddr::reset() {
  memset(addr_, 0, Flow::kAddressLengthMax);
  addr_length_ = 0;
}

void TopologyFilter::NextHopAddr::set(const char* addr, const int length) {
  if (length == static_cast<int>(Flow::kAddressLengthIPv4))
    Flow::addr_import_ipv4(addr, addr_);
  else if (length == static_cast<int>(Flow::kAddressLengthIPv6)) {
    Flow::addr_import_ipv6(addr, addr_);
  } else {
    std::stringstream err_msg;
    err_msg << "TopologyFilter::NextHopAddr::set :: Unknown addr length ..."
              << std::endl;
    throw FlowBoxE(err_msg.str(), __FILE__, __LINE__);
  }
  addr_length_ = length;
}

bool TopologyFilter::NextHopAddr::operator()(
    const TopologyFilter::NextHopAddr& nh1,
    const TopologyFilter::NextHopAddr& nh2) const {
  if (nh1.addr_length_ < nh2.addr_length_) {
    return true;
  } else {
    if (nh1.addr_length_ == nh2.addr_length_) {
      return (memcmp(nh1.addr_, nh2.addr_, static_cast<int>(nh1.addr_length_))
              < 0);
    } else {
      return false;
    }
  }
}

std::string TopologyFilter::NextHopAddr::to_s(void) const {
  std::string buf;
  if (addr_length_ == static_cast<int>(Flow::kAddressLengthIPv4)
      || addr_length_ == static_cast<int>(Flow::kAddressLengthIPv6)) {
    Flow::addr_to_s(buf, addr_, addr_length_);
  } else {
    std::stringstream err_msg;
    err_msg << "TopologyFilter::NextHopAddr::set :: Unknown addr length ..."
              << std::endl;
    throw FlowBoxE(err_msg.str(), __FILE__, __LINE__);
  }
  return (std::string(buf));
}

//------------------------------------------------------------------------------
// class Interface
//------------------------------------------------------------------------------
TopologyFilter::Interface::Interface(int export_device_id, int snmp_id)
    : export_device_id_(export_device_id),
      snmp_id_(snmp_id),
      filter_(kStateDefault),
      state_(kStateDefault),
      addr_ranges_(""),
      addr4_(),
      addr6_(),
      send_update_(true) {
  addr4_.from("0.0.0.0/32");
  addr6_.from("::1/128");
  addr_ranges_map_.set_value_not_found(kAddrNotFound);
  stat_reset();
}

std::string TopologyFilter::Interface::to_s(void) {
  std::stringstream tmp;
  tmp << "EID: " << export_device_id_ << " SNMP: " << snmp_id_ << " ";
  tmp << "in: " << stat_incoming_ << " out: " << stat_outgoing_ << std::endl;
  NextHopAddr_map::iterator next_hop(next_hops_.begin());
  NextHopAddr_map::iterator end(next_hops_.end());
  while (next_hop != end) {
    tmp << "   " << (next_hop->first).to_s() << ", " << next_hop->second << " "
        << std::endl;
    next_hop++;
  }
  return (std::string(tmp.str()));
}

bool TopologyFilter::Interface::incoming(const FlowContainer::iterator& flow) {
  assert(flow->if_in_ == snmp_id_);

  // update statistics
  stat_incoming_++;

  // apply filter policy
  if (filter_ == kFilterAll or filter_ == kFilterIn) {
    return (false);
  } else {
    return (true);
  }
}

bool TopologyFilter::Interface::outgoing(const FlowContainer::iterator& flow) {
  assert(flow->if_out_ == snmp_id_);

  // update statistics
  stat_outgoing_++;

  // check if the 'next hop ip address' is included in the configured
  // address range of the interface
  // (longest common prefix matching)
  // ... prepare the prefix
  int in_range;
  if (flow->addr_length_ == static_cast<int>(Flow::kAddressLengthIPv4)) {
    addr4_.from_nb_zero(flow->addr_next_, 32, Prefix::kFamilyIPv4);
    in_range = addr_ranges_map_.lookup(addr4_);
  } else if (flow->addr_length_ == static_cast<int>(Flow::kAddressLengthIPv6)) {
    addr6_.from_nb_zero(flow->addr_next_, 128, Prefix::kFamilyIPv6);
    in_range = addr_ranges_map_.lookup(addr6_);
  } else {
    std::stringstream err_msg;
    err_msg << "TopologyFilter::Interface::outgoing: unknown address length '"
            << flow->addr_length_ << "'" << std::endl;
    assert(false);
    throw FlowBoxE(err_msg.str(), __FILE__, __LINE__);
  }

  if (in_range != kAddrFound) {
    // mark this interface as in alarm state
    state_ = kStateAlarm;

    // remember the observed next hop interface
    TopologyFilter::NextHopAddr next_hop(flow->addr_next_, flow->addr_length_);
    NextHopAddr_map::iterator iter = next_hops_.find(next_hop);
    if (iter == next_hops_.end()) {
      next_hops_[next_hop]++;
      send_update_ = true;
    } else {
      iter->second++;
    }
  }

  // apply filter policy
  if (filter_ == kFilterAll or filter_ == kFilterOut) {
    return (false);
  } else {
    return (true);
  };
}

void TopologyFilter::Interface::stat_reset() {
  stat_incoming_ = 0;
  stat_outgoing_ = 0;
}
void TopologyFilter::Interface::set_filter(const int filter) {
  filter_ = filter;
}

void TopologyFilter::Interface::set_state(const int state) {
  state_ = state;
}

void TopologyFilter::Interface::set_addr_ranges(const std::string addr_ranges) {
  addr_ranges_ = addr_ranges;

#ifdef TOPOLOGY_FILTER_DEBUG
  std::cout << "TopologyFilter::Interface::set_addr_ranges(const std::string addr_ranges)"
            << std::endl;
  std::cout.flush();
#endif

  // do we have any IP?
  if (addr_ranges_.size() == 0)
    return;

  // we have at least one IP
  // parse the string
  // X.X.X.X/YY, X:X:X:X/YY
  std::stringstream lineStream(addr_ranges_);
  std::string cell;
  while (std::getline(lineStream, cell, ',')) {
#ifdef TOPOLOGY_FILTER_DEBUG
    std::cout << "ADD: '" << cell << "'" << std::endl;
    std::cout.flush();
#endif
    if (cell.size() < 3)
      continue;

    Prefix addr_;
    addr_.from(cell);
    int in_range = addr_ranges_map_.lookup(addr_);
    if (in_range != kAddrFound) {
      addr_ranges_map_.insert(addr_, kAddrFound);
    } else {
      // ALREADY ADDED
    }
  }
}

int TopologyFilter::Interface::get_export_device_id(void) {
  return (export_device_id_);
}

int TopologyFilter::Interface::get_snmp_id(void) {
  return (snmp_id_);
}

int TopologyFilter::Interface::get_state(void) {
  return (state_);
}

std::string TopologyFilter::Interface::get_observed_addrs(void) {
  std::stringstream tmp;
  NextHopAddr_map::iterator next_hop(next_hops_.begin());
  NextHopAddr_map::iterator end(next_hops_.end());
  while (next_hop != end) {
    tmp << (next_hop->first).to_s() << "@" << next_hop->second << ", ";
    next_hop++;
  }
  return (std::string(tmp.str()));
}

bool TopologyFilter::Interface::update_required(void) {
  return (state_ == kStateAlarm and send_update_);
}

void TopologyFilter::Interface::update_sent(void) {
  send_update_ = false;
}

//------------------------------------------------------------------------------
// class TopologyFilter
//------------------------------------------------------------------------------
TopologyFilter::TopologyFilter() {
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
  interfaces_.clear();
  map_export_device_id_to_array_index_.clear();
  map_snmp_to_array_index_.clear();
}

TopologyFilter::~TopologyFilter() {
}

//------------------------------------------------------------------------------
// class TopologyFilter CONFIGURATION
//------------------------------------------------------------------------------
void TopologyFilter::conf_push(Configuration config) {
  fb_sem_wait(&conf_critical_section_sem_);
  // CRITICAL SECTION::START
  conf_in_.push(config);
  conf_available_ = true;
  fb_sem_post(&conf_critical_section_sem_);
  // CRITICAL SECTION::STOP
  return;
}

void TopologyFilter::conf_pop(void) {
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

void TopologyFilter::conf_migrate(const Configuration& migrate) {
  data_input_ = migrate.get_input();
  data_output_ = migrate.get_output();
  stat_interval_s_ = migrate.get_stat_interval();

  interfaces_clear();

  std::vector<TopologyFilter::Configuration::Interface>::const_iterator iter =
      migrate.get_interfaces_begin();
  std::vector<TopologyFilter::Configuration::Interface>::const_iterator end =
      migrate.get_interfaces_end();

  while (iter != end) {
    TopologyFilter::Interface* interface = get_interface(
        iter->export_device_id_, iter->snmp_if_id_);
    interface->set_filter(iter->filter_);
    interface->set_addr_ranges(iter->addr_ranges_);
    interface->set_state(TopologyFilter::Interface::kStateNormal);

    iter++;
  }
  return;
}

//------------------------------------------------------------------------------
// class TopologyFilter OBSERVATION
//------------------------------------------------------------------------------
TopologyFilter::Observation TopologyFilter::obs_get(void) {
  TopologyFilter::Observation obs;
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
  return (TopologyFilter::Observation(obs));
}
void TopologyFilter::obs_push(Interface* interface, uint64_t now_s) {
  Observation observation;
  observation.set_time_s(now_s);
  observation.set_export_device_id(interface->get_export_device_id());
  observation.set_snmp_if_id(interface->get_snmp_id());
  observation.set_state(interface->get_state());
  observation.set_observed_addrs(interface->get_observed_addrs());
  fb_sem_wait(&obs_critical_section_sem_);
  // CRITICAL SECTION::START
  obs_out_.push(observation);
  fb_sem_post(&obs_critical_section_sem_);
  // CRITICAL SECTION::STOP
  fb_sem_post(&obs_available_sem_);
  // SIGNAL DATA
}

//------------------------------------------------------------------------------
// class TopologyFilter STATISTICS
//------------------------------------------------------------------------------
TopologyFilter::Statistics TopologyFilter::stat_get(void) {
  TopologyFilter::Statistics stat;
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
  return (TopologyFilter::Statistics(stat));
}
void TopologyFilter::stat_export(uint64_t now_s) {
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
// class TopologyFilter DATA
//------------------------------------------------------------------------------
void TopologyFilter::data_process(FlowContainer::iterator flow,
                                  FlowContainer::iterator end) {
  // walk over flows
  while (flow != end) {
    if (flow->valid_ == true) {
      process(flow);
    };
    flow++;
  };
}

void TopologyFilter::process(const FlowContainer::iterator& flow) {
  // export stats?
  if (flow->stop_s_ > stat_export_next_s_) {
    stat_export(flow->stop_s_);
  }
  stat_current_.inc_flows();

  Interface* interface;

  // Interface: Incoming
  interface = get_interface(flow->export_device_id_, flow->if_in_);
  bool keep_in = interface->incoming(flow);

  if (interface->update_required()) {
    obs_push(interface, flow->stop_s_);
    interface->update_sent();
  }

  // Interface: Outgoing
  interface = get_interface(flow->export_device_id_, flow->if_out_);
  bool keep_out = interface->outgoing(flow);
  if (interface->update_required()) {
    obs_push(interface, flow->stop_s_);
    interface->update_sent();
  }
  if (!(keep_in or keep_out)) {
    flow->valid_ = false;  // filter it
    stat_current_.inc_flows_filtered();
  }
}

//------------------------------------------------------------------------------
// class TopologyFilter OTHER
//------------------------------------------------------------------------------
//
// We keep for each 'interface' of a 'export device' an 'Interface' data structur
void TopologyFilter::interfaces_clear(void) {
  // delete the 'Interface' structures
  std::vector<Interface*>::iterator iter = interfaces_.begin();
  std::vector<Interface*>::iterator end = interfaces_.end();
  while (iter != end) {
    if (*iter != NULL) {
      delete (*iter);
      *iter = NULL;
    }
    iter++;
  }
  interfaces_.clear();

  // clear the mapping
  map_export_device_id_to_array_index_.clear();
  map_snmp_to_array_index_.clear();
}

int TopologyFilter::get_exporter_index(const int export_device_id) {
  if ((static_cast<int>(map_export_device_id_to_array_index_.size()))
      < export_device_id + 1) {
    map_export_device_id_to_array_index_.resize(export_device_id + 1, kNoIndex);
  };
  int exporter_index = map_export_device_id_to_array_index_[export_device_id];
  if (exporter_index == kNoIndex) {
    exporter_index = map_snmp_to_array_index_.size();
    map_snmp_to_array_index_.resize(exporter_index + 1);
    map_export_device_id_to_array_index_[export_device_id] = exporter_index;
  };
  assert(static_cast<int>(map_snmp_to_array_index_.size()) > exporter_index);
  return (exporter_index);
}

TopologyFilter::Interface* TopologyFilter::get_interface(
    const int export_device_id, const int snmp_id) {
  int exporter_index = get_exporter_index(export_device_id);
  std::vector<int>& exporter = map_snmp_to_array_index_[exporter_index];
  if (static_cast<int>(exporter.size()) < snmp_id + 1) {
    // we have to resize the map first
    exporter.resize(snmp_id + 1, kNoIndex);
  }
  assert(static_cast<int>(exporter.size()) > snmp_id);
  int snmp_index = exporter[snmp_id];
  if (snmp_index == kNoIndex) {
    // new interface on exporter device detected, add it
    snmp_index = interfaces_.size();
    interfaces_.push_back(new Interface(export_device_id, snmp_id));
    exporter[snmp_id] = snmp_index;

#ifdef TOPOLOGY_FILTER_DEBUG
    std::cout << "TP::Interface: ";
    std::cout << "EID: " << export_device_id << " ";
    std::cout << "If: "  << snmp_id << " ";
    std::cout << "MAP: [" << exporter_index << "][" << snmp_index << "] ";
    std::cout << "MAP[].size() " <<  map_export_device_id_to_array_index_.size() << " ";
    std::cout << "MAP[][].size() " <<  exporter.size()<< " ";
    std::cout << std::endl;
#endif
  }
  return (interfaces_[snmp_index]);
}

// MAIN LOOP -------------------------------------------------------------------
void TopologyFilter::data_process(void) {
#ifdef TOPOLOGY_FILTER_DEBUG
  std::cout << "TopologyFilter::data_process -- started" << std::endl;
  std::cout.flush();
#endif
  FlowContainer* fc;
  while (true) {
    // do we have some a new config?
    if (conf_available_) {
      conf_pop();
#ifdef TOPOLOGY_FILTER_DEBUG
      std::cout << "TopologyFilter::data_input:" << data_input_<< std::endl;
      std::cout << "TopologyFilter::data_output:" << data_output_<< std::endl;
#endif
    }

    // ups no input buffer ...
    if (data_input_ == NULL) {
      std::cout << "TopologyFilter::data_thread_main -- empty data_input_"
                << std::endl;
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
#ifdef TOPOLOGY_FILTER_DEBUG
        std::cout
            << "I've got an empty container pointer without good reason !!"
            << std::endl;
#endif
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
#ifdef TOPOLOGY_FILTER_DEBUG
  std::cout << "TopologyFilter::worker_thread_loop -- finished" << std::endl;
#endif
}
