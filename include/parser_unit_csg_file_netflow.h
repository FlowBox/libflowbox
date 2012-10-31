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
 * @file   parser_unit_csg_file_netflow.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @date   October, 2012
 * @brief  Process NetFlow 5 and 9 records stored in CSG files .
 *
 * The ParserUnitCSGFile is required to process the two CSG data
 * files belonging to the same time interval in parallel.
 *
 */

#ifndef FLOW_BOX_INCLUDE_PARSER_UNIT_CSG_FILE_NETFLOW_H_
#define FLOW_BOX_INCLUDE_PARSER_UNIT_CSG_FILE_NETFLOW_H_

#include <semaphore.h>
#include <sys/types.h>
#include <dirent.h>

#include <ctime>
#include <cassert>
#include <cstring>
#include <string>
#include <sstream>
#include <vector>
#include <set>
#include <list>
#include <queue>
#include <map>

#include "common.h"
#include "packet.h"
#include "packet_source_csg_file.h"
#include "parser_netflow_5.h"
#include "parser_netflow_9.h"
#include "flow_container_pool.h"
#include "flow_container_buffer.h"

// #define PARSER_UNIT_CSG_FILE_NETFLOW_VERBOSEN

class ParserUnitCSGFileNetflow {
 public:
  // SET: Mapping of paths to files
  static const int kFileSet19991Dat;
  static const int kFileSet19991Stat;
  static const int kFileSet19993Dat;
  static const int kFileSet19993Stat;

  static const int kIVMinFlowSize = 20;

  // EXPORT DEVICE: The device that produces the flows
  // We will only accept packets from devices that are explicitly
  // acknowledged by the user. Note that an attacker could still fork
  // the source address of the UDP packet and the Engine ID field.
  class ExportDevice {
   private:
    // identifier
    struct sockaddr packet_src_addr_;
    uint32_t flow_engine_id_;
    uint32_t version_;
    // alias
    uint32_t export_id_;
    // parser
    ParserNetflow5 parser5_;
    ParserNetflow9 parser9_;

   public:
    ExportDevice(const struct sockaddr& packet_src_addr,
                 const uint32_t flow_engine_id, const uint32_t version,
                 const uint32_t export_id);

    struct sockaddr const * get_packet_src_addr(void) const;
    uint32_t get_flow_engine_id(void) const;
    uint32_t get_version(void) const;
    uint32_t get_export_id(void) const;

    std::string to_s(void) const;
    void parse(const Packet& packet, FlowContainer* fc);
    void reset(void);
    std::string get_stat_s(void);
    void reset_stat(void);
  };

  class ExportDeviceKey {
   public:
    static const uint32_t kSeed = 0xdeadbeef;

   private:
    struct sockaddr addr_;
    uint32_t engine_id_;
    uint32_t version_;
    uint32_t hkey_;

    void set_hkey(void);

   public:
    ExportDeviceKey(void);
    ExportDeviceKey(const struct sockaddr& src_addr,
                    const uint32_t flow_engine_id, const uint32_t version);
    bool operator()(const ExportDeviceKey& left,
                    const ExportDeviceKey& right) const;
    size_t operator()(const ExportDeviceKey& key) const;
    uint32_t get_hkey(void) const;
    std::string to_s(void) const;
  };

  typedef hash_map<ExportDeviceKey, ExportDevice*, ExportDeviceKey, ExportDeviceKey> ExportDevice_map;

  class Configuration {
   public:
    static const std::string kPathDefault;
    static const int kScanIntervalDefault;
    static const int kStatisticsIntervalDefault;
    static const bool kRemoveFilesAfterDefault;
    static const int kExportTimeWindowDefault;

   private:
    std::string path_;
    int scan_interval_s_;
    FlowContainerBuffer* output_;
    std::vector<ExportDevice> export_devices_;
    int stat_interval_s_;
    bool remove_files_after_;
    int export_time_window_s_;

   public:
    Configuration();
    ~Configuration();
    void reset(void);

    void set_path(std::string path);
    void set_scan_interval_s(int scan_interval_s);
    void set_output(FlowContainerBuffer* output);
    void add_export_device(ExportDevice& device);
    void set_stat_interval_s(int set_stat_interval_s);
    void set_remove_files_after(bool remove_files_after);
    void set_export_time_window_s(int export_time_window_s);

    std::string get_path(void) const;
    FlowContainerBuffer* get_output(void) const;
    int get_scan_interval_s(void) const;
    std::vector<ExportDevice>::const_iterator get_export_device_begin(void) const;
    std::vector<ExportDevice>::const_iterator get_export_device_end(void) const;
    int get_stat_interval_s(void) const;
    bool get_remove_files_after(void) const;
    int get_export_time_window_s(void) const;
  };

  class Observation {
   private:
    bool valid_;
    uint64_t time_s_;
    std::string message_;

   public:
    Observation();
    void reset();

    bool get_valid(void) const;
    uint64_t get_time_s(void) const;
    std::string get_message(void) const;

    void set_valid(bool valid);
    void set_time_s(uint64_t time_s);
    void set_message(const std::string& message);
  };

  class Statistics {
   private:
    bool valid_;
    // timing
    uint64_t time_s_;
    uint64_t duration_s_;
    uint64_t real_time_s_;
    uint64_t real_duration_s_;
    // load
    uint64_t packets_;
    // input validation;
    uint64_t iv_min_size_;
    uint64_t iv_format_;
    uint64_t iv_exporter_;
    uint64_t iv_export_time_too_small_;
    uint64_t iv_export_time_too_big_;

   public:
    Statistics();
    void reset(void);

    bool get_valid(void);
    uint64_t get_time_s(void);
    uint64_t get_duration_s(void);
    uint64_t get_real_time_s(void);
    uint64_t get_real_duration_s(void);
    uint64_t get_packets(void);
    uint64_t get_iv_min_size(void);
    uint64_t get_iv_format(void);
    uint64_t get_iv_exporter(void);
    uint64_t get_iv_export_time_too_small(void);
    uint64_t get_iv_export_time_too_big(void);

    void set_valid(bool valid);
    void set_time_s(uint64_t time_s);
    void set_duration_s(uint64_t duration_s);
    void set_real_time_s(uint64_t packets);
    void set_real_duration_s(uint64_t duration_s);
    void set_packets(uint64_t packets);
    void set_iv_min_size(uint64_t min_size);
    void set_iv_format(uint64_t iv_format);
    void set_iv_exporter(uint64_t iv_exporter);
    void set_iv_export_time_too_small(uint64_t iv_export_time_too_small);
    void set_iv_export_time_too_big(uint64_t iv_export_time_too_big);

    void inc_packets(void);
    void inc_iv_min_size(void);
    void inc_iv_format(void);
    void inc_iv_exporter(void);
    void inc_iv_export_time_too_small(void);
    void inc_iv_export_time_too_big(void);

    static std::string head_to_s(void);
    std::string to_s(void) const;
  };

  // ## CLASS VARIABLES #######################################################
 private:
  // -- CONFIGURATION: INCOMING ----------------------------------------------
  sem_t conf_critical_section_sem_;
  std::queue<Configuration> conf_in_;
  bool conf_available_;
  Configuration conf_current_;

  std::vector<ExportDevice*> export_devices_;
  ExportDevice_map export_devices_map_;

  // -- OBSERVATION: OUTGOING ------------------------------------------------
  sem_t obs_critical_section_sem_;
  sem_t obs_available_sem_;
  std::queue<Observation> obs_out_;

  // -- STATISTICS: OUTGOING--------------------------------------------------
  sem_t stat_critical_section_sem_;
  sem_t stat_available_sem_;
  std::queue<Statistics> stat_out_;
  Statistics stat_current_;
  uint64_t stat_export_next_s_;

  // -- DATA STREAM  ---------------------------------------------------------
  PacketSourceCSGFile data_source_;
  FlowContainerBuffer* data_output_;
  FlowContainerPool* data_container_pool_;

  // -- OTHERS  --------------------------------------------------------------
  // estimated time (based on flow export timestamp)
  uint64_t time_s_;
  uint64_t iv_export_time_last_jump_;

  // CSG Files Set
  std::set<int>processed_seq_numbers_;
  std::queue<std::vector<std::string> > file_sets_;

  // ## CLASS FUNCTIONS ########################################################
 private:
  // -- CONFIGURATION: INCOMING ----------------------------------------------
  void conf_pop(void);
  void conf_migrate(const Configuration& migrate);

  // -- OBSERVATION: OUTGOING ------------------------------------------------
  void obs_push(const std::string& message);

  // -- STATISTICS: OUTGOING--------------------------------------------------
  void stat_export(void);

  // -- DATA STREAM  ---------------------------------------------------------
  void data_process_file_set(const std::vector<std::string>& set);

  // -- OTHERS ---------------------------------------------------------------
  ExportDevice* get_export_device(
      const struct sockaddr& src_addr,
      const int engine_id,
      const int version);

  void reset_exporters(void);
  void scan_folder(void);
  int grep_seq_number(std::string file_name);
  std::map<int, std::vector<std::string> > grep_files(std::string folder);

  // ## CLASS API ##############################################################
 public:
  ParserUnitCSGFileNetflow();
  ~ParserUnitCSGFileNetflow();
  // -- CONFIGURATION: INCOMING ----------------------------------------------
  void conf_push(const Configuration& config);

  // -- OBSERVATION: OUTGOING ------------------------------------------------
  Observation obs_get(void);

  // -- STATISTICS: OUTGOING--------------------------------------------------
  Statistics stat_get(void);

  // -- DATA STREAM  ---------------------------------------------------------
  void data_process(void);

 private:
  DISALLOW_COPY_AND_ASSIGN(ParserUnitCSGFileNetflow);
};

#endif  // FLOW_BOX_INCLUDE_PARSER_UNIT_CSG_FILE_NETFLOW_H_
