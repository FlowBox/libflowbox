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
 * @file   topology_filter.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 *
 * @date   October, 2012
 * @brief  Extract the network topology from the flow data.
 *
 * This class performs an analysis of the network topology and filters
 * flows according to the topology.
 *
 */

#ifndef FLOW_BOX_INCLUDE_TOPOLOGY_FILTER_H_
#define FLOW_BOX_INCLUDE_TOPOLOGY_FILTER_H_

// http://www.gnu.org/s/hello/manual/libc/index.html
#include <semaphore.h>

#include <cassert>
#include <cstring>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include <map>
#include <queue>
#include <vector>

#include "flow_container_pool.h"
#include "flow_container_buffer.h"
#include "prefix_to_int_map.h"

class TopologyFilter {
 public:
  static const int kNoIndex;

  class Configuration {
   public:
    static const int kStatisticDefaultInterval;
    struct Interface {
      int export_device_id_;
      int snmp_if_id_;
      int filter_;
      std::string addr_ranges_;
    };

   private:
    FlowContainerBuffer* input_;
    FlowContainerBuffer* output_;
    uint64_t stat_interval_;
    std::vector<Interface> interfaces_;

   public:
    Configuration();
    void reset(void);

    FlowContainerBuffer* get_input(void) const;
    FlowContainerBuffer* get_output(void) const;
    uint64_t get_stat_interval(void) const;
    std::vector<Interface>::const_iterator get_interfaces_begin(void) const;
    std::vector<Interface>::const_iterator get_interfaces_end(void) const;

    void set_input(FlowContainerBuffer* input);
    void set_output(FlowContainerBuffer* output);
    void set_stat_interval(uint64_t interval);
    void add_interface(int export_device_id, int snmp_if_id, int filter,
                       std::string addr_ranges);
  };

  class Observation {
    bool valid_;
    uint64_t time_s_;
    int export_device_id_;
    int snmp_if_id_;
    int state_;
    std::string observed_addrs_;

   public:
    Observation();
    void reset();

    bool get_valid(void) const;
    uint64_t get_time_s(void) const;
    int get_export_device_id(void) const;
    int get_snmp_if_id(void) const;
    int get_state(void) const;
    std::string get_observed_addrs(void) const;

    void set_valid(bool valid);
    void set_time_s(uint64_t time_s);
    void set_export_device_id(int export_device_id);
    void set_snmp_if_id(int snmp_if_id);
    void set_state(int state);
    void set_observed_addrs(std::string observed_addrs);
  };

  class Statistics {
    bool valid_;
    uint64_t time_s_;
    uint64_t duration_s_;
    uint64_t flows_;
    uint64_t flows_filtered_;

   public:
    Statistics();
    void reset(void);

    bool get_valid(void) const;
    uint64_t get_time_s(void) const;
    uint64_t get_duration_s(void) const;
    uint64_t get_flows(void) const;
    uint64_t get_flows_filtered(void) const;

    void set_valid(bool valid);
    void set_time_s(uint64_t time_s);
    void set_duration_s(uint64_t duration_s);
    void set_flows(uint64_t flows);
    void set_flows_filtered(uint64_t flows_filtered);
    void inc_flows(void);
    void inc_flows_filtered(void);
  };

  /// This class implements a Hash Key object for the std::map
  class NextHopAddr {
   private:
    char addr_[Flow::kAddressLengthMax];  ///< the key storage
    int addr_length_;  ///< defines the length of the address (4 or 6)

    void reset();  ///< set addr and length to zero
    void set(const char* addr, const int length);  ///< set addr and length

   public:
    NextHopAddr();
    NextHopAddr(const char* addr, const int length);
    bool operator()(const NextHopAddr& nh1, const NextHopAddr& nh2) const;
    std::string to_s(void) const;
  };

  typedef std::map<NextHopAddr, uint64_t, NextHopAddr> NextHopAddr_map;

  class Interface {
   public:
    // DEFINE FILTER ACTION
    static const int kFilterAll = 0;
    static const int kFilterIn = 1;
    static const int kFilterOut = 2;
    static const int kFilterNon = 4;

    // define the default behavior
    static const int kFilterDefault = kFilterAll;

    // DEFINE STATE
    static const int kStateNormal = 0;
    static const int kStateAlarm = 1;
    // define the default behavior
    static const int kStateDefault = kStateNormal;

    // DEFINE ADDR MAP
    static const int kAddrNotFound = 0;
    static const int kAddrFound = 1;

   private:
    int export_device_id_;
    int snmp_id_;
    int filter_;
    int state_;
    std::string addr_ranges_;
    PrefixToIntMap addr_ranges_map_;
    Prefix addr4_, addr6_;

    bool send_update_;
    uint64_t stat_incoming_;
    uint64_t stat_outgoing_;
    NextHopAddr_map next_hops_;

   public:
    Interface(int export_device_id, int snmp_id);
    std::string to_s(void);
    bool incoming(const FlowContainer::iterator& flow);
    bool outgoing(const FlowContainer::iterator& flow);

    void stat_reset(void);
    void set_filter(const int filter);
    void set_addr_ranges(const std::string addr_ranges);
    void set_state(const int state);

    int get_export_device_id(void);
    int get_snmp_id(void);
    int get_state(void);
    bool update_required(void);
    void update_sent(void);
    std::string get_observed_addrs(void);

   private:
    DISALLOW_COPY_AND_ASSIGN(Interface);
  };

  // ## CLASS VARIABLES #######################################################
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

  // -- OTHERS ---------------------------------------------------------------
  // List of all interfaces
  std::vector<Interface*> interfaces_;
  // Map export devices to array index
  std::vector<int> map_export_device_id_to_array_index_;
  // Map snmp interface number to a array index
  std::vector<std::vector<int> > map_snmp_to_array_index_;

  // ## CLASS FUNCTIONS ########################################################
 private:
  // -- CONFIGURATION: INCOMING ----------------------------------------------
  void conf_pop(void);
  void conf_add_prefix(std::string prefix);
  void conf_migrate(const Configuration& migrate);

  // -- OBSERVATION: OUTGOING ------------------------------------------------
  void obs_push(Interface* interface, uint64_t now_s);

  // -- STATISTICS: OUTGOING--------------------------------------------------
  void stat_export(uint64_t now_s);

  // -- DATA STREAM  ---------------------------------------------------------
  void data_process(FlowContainer::iterator flow, FlowContainer::iterator end);
  void process(const FlowContainer::iterator& flow);

  // -- OTHERS ---------------------------------------------------------------
  void interfaces_clear(void);
  int get_exporter_index(const int export_device_id);
  Interface* get_interface(const int export_device_id, const int snmp_id);

  // ## CLASS API ##############################################################
 public:
  TopologyFilter();
  ~TopologyFilter();

  // -- CONFIGURATION: INCOMING ----------------------------------------------
  void conf_push(Configuration);

  // -- OBSERVATION: OUTGOING ------------------------------------------------
  Observation obs_get(void);

  // -- STATISTICS: OUTGOING--------------------------------------------------
  Statistics stat_get(void);

  // -- DATA STREAM  ---------------------------------------------------------
  void data_process(void);

 private:
  DISALLOW_COPY_AND_ASSIGN(TopologyFilter);
};
#endif  // FLOW_BOX_INCLUDE_wTOPOLOGY_FILTER_H_
