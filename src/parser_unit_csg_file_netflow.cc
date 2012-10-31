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
 * @file   parser_unit_csg_file_netflow.cc
 * @author Dominik Schatzmann <schadomi@gmail.com>
 *
 * @date   October, 2012
 * @brief  Process NetFlow 5 and 9 records stored in CSG files .
 *
 * The ParserUnitCSGFile is required to process the two CSG data
 * files belonging to the same time interval in parallel.
 *
 */

#include "parser_unit_csg_file_netflow.h"

//------------------------------------------------------------------------------
// class ParserUnitCSGFileNetflow::Const
//------------------------------------------------------------------------------
const int ParserUnitCSGFileNetflow::kFileSet19991Dat = 0;
const int ParserUnitCSGFileNetflow::kFileSet19991Stat = 1;
const int ParserUnitCSGFileNetflow::kFileSet19993Dat = 2;
const int ParserUnitCSGFileNetflow::kFileSet19993Stat = 3;

//------------------------------------------------------------------------------
// struct ParserUnitCSGFileNetflow::ExportDevice
//------------------------------------------------------------------------------
ParserUnitCSGFileNetflow::ExportDevice::ExportDevice(
    const struct sockaddr& packet_src_addr, const uint32_t flow_engine_id,
    const uint32_t version, const uint32_t export_id)
    : packet_src_addr_(packet_src_addr),
      flow_engine_id_(flow_engine_id),
      version_(version),
      export_id_(export_id),
      parser5_(export_id),
      parser9_(export_id) {
}

//------------------------------------------------------------------------------
struct sockaddr const * ParserUnitCSGFileNetflow::ExportDevice::get_packet_src_addr(
    void) const {
  return (&packet_src_addr_);
}

uint32_t ParserUnitCSGFileNetflow::ExportDevice::get_flow_engine_id(
    void) const {
  return (flow_engine_id_);
}

uint32_t ParserUnitCSGFileNetflow::ExportDevice::get_version(void) const {
  return (version_);
}

uint32_t ParserUnitCSGFileNetflow::ExportDevice::get_export_id(void) const {
  return (export_id_);
}

//------------------------------------------------------------------------------
std::string ParserUnitCSGFileNetflow::ExportDevice::to_s(void) const {
  std::stringstream tmp;
  char tmp_ip[INET6_ADDRSTRLEN];
  if (packet_src_addr_.sa_family == AF_INET) {
    inet_ntop(AF_INET, &((struct sockaddr_in*) (&packet_src_addr_))->sin_addr,
              tmp_ip, INET6_ADDRSTRLEN);
    tmp << "IP:" << tmp_ip << ", ";
  } else if (packet_src_addr_.sa_family == AF_INET6) {
    inet_ntop(AF_INET6,
              &((struct sockaddr_in6*) (&packet_src_addr_))->sin6_addr, tmp_ip,
              INET6_ADDRSTRLEN);
    tmp << "IP:" << tmp_ip << ", ";
  } else {
    tmp << "IP: unknown addr family, ";
  }
  tmp << "Fid: " << flow_engine_id_ << ", ";
  tmp << "V: " << version_ << ", ";
  tmp << " -> Eid: " << export_id_;
  return (std::string(tmp.str()));
}
void ParserUnitCSGFileNetflow::ExportDevice::parse(const Packet& packet,
                                                   FlowContainer* fc) {
#ifdef PARSER_UNIT_CSG_FILE_NETFLOW_DEBUG
  uint32_t id = ParserNetflow9::get_engine_id(packet);
  if (id != flow_engine_id_) {
    std::cout << "ID (" << id << ") != FLOW ID ("<< flow_engine_id_<< ")" << std::endl;
    assert(false);
  }
#endif

  if (version_ == 5) {
    parser5_.parse(packet, fc);
  } else if (version_ == 9) {
    parser9_.parse(packet, fc);
  }
}

void ParserUnitCSGFileNetflow::ExportDevice::reset(void) {
  parser5_.reset();
  parser9_.reset();
}

std::string ParserUnitCSGFileNetflow::ExportDevice::get_stat_s(void) {
  std::map<std::string, uint64_t> stat;
  if (version_ == 5) {
    stat = parser5_.statistics_get();
    parser5_.statistics_reset();
  } else if (version_ == 9) {
    stat = parser9_.statatistics_get();
    parser9_.statistics_reset();
  }

  std::stringstream tmp;
  std::map<std::string, uint64_t>::iterator iter = stat.begin();
  while (iter != stat.end()) {
    tmp << iter->first << ":" << iter->second << ", ";
    iter++;
  }

  return (std::string(tmp.str()));
}

void ParserUnitCSGFileNetflow::ExportDevice::reset_stat(void) {
  parser5_.statistics_reset();
  parser9_.statistics_reset();
}

//------------------------------------------------------------------------------
// struct ParserUnitCSGFileNetflow::ExportDeviceKey
//------------------------------------------------------------------------------
ParserUnitCSGFileNetflow::ExportDeviceKey::ExportDeviceKey(void) {
  hkey_ = 0;
}

ParserUnitCSGFileNetflow::ExportDeviceKey::ExportDeviceKey(
    const struct sockaddr& src_addr, const uint32_t flow_engine_id,
    const uint32_t version) {
  addr_ = src_addr;
  engine_id_ = flow_engine_id;
  version_ = version;
  set_hkey();
}

void ParserUnitCSGFileNetflow::ExportDeviceKey::set_hkey(void) {
  uint32_t a, b, c;
  if (addr_.sa_family == AF_INET) {
    a = *(reinterpret_cast<uint32_t*>(&((struct sockaddr_in*) (&addr_))->sin_addr));
    b = engine_id_;
    c = version_ & kSeed;
    my_mix(a, b, c);
  } else {
    throw FlowBoxE("NotYetImplemented", __FILE__, __LINE__);
  }
  my_final(a, b, c);
  hkey_ = c;
}

bool ParserUnitCSGFileNetflow::ExportDeviceKey::operator()(
    const ExportDeviceKey& left, const ExportDeviceKey& right) const {

  if (left.hkey_ == right.hkey_ and left.engine_id_ == right.engine_id_
      and left.addr_.sa_family == right.addr_.sa_family) {
    // compare address -- IPV4
    if (left.addr_.sa_family == AF_INET
        and (memcmp(&((struct sockaddr_in*) (&left.addr_))->sin_addr,
                    &((struct sockaddr_in*) (&right.addr_))->sin_addr, 4))
            == 0) {
      return (true);
      // compare address -- IPV6
    } else if (left.addr_.sa_family == AF_INET6
        and (memcmp(&((struct sockaddr_in6*) (&left.addr_))->sin6_addr,
                    &((struct sockaddr_in6*) (&right.addr_))->sin6_addr, 16))
            == 0) {
      return (true);
    } else {
      // WTF? Can't work this address type
      throw FlowBoxE("Can't work this address type", __FILE__, __LINE__);
      return false;
    }
  } else {
    return (false);
  }
  return (false);
}

size_t ParserUnitCSGFileNetflow::ExportDeviceKey::operator()(
    const ExportDeviceKey& key) const {
  return (key.hkey_);
}

uint32_t ParserUnitCSGFileNetflow::ExportDeviceKey::get_hkey(void) const {
  return (hkey_);
}

std::string ParserUnitCSGFileNetflow::ExportDeviceKey::to_s(void) const {
  std::stringstream tmp;
  char tmp_ip[INET6_ADDRSTRLEN];
  if (addr_.sa_family == AF_INET) {
    inet_ntop(AF_INET, &((struct sockaddr_in*) (&addr_))->sin_addr, tmp_ip,
              INET6_ADDRSTRLEN);
    tmp << "IP:" << tmp_ip << ", ";
  } else if (addr_.sa_family == AF_INET6) {
    inet_ntop(AF_INET6, &((struct sockaddr_in6*) (&addr_))->sin6_addr, tmp_ip,
              INET6_ADDRSTRLEN);
    tmp << "IP:" << tmp_ip << ", ";
  } else {
    tmp << "IP: unknown addr family, ";
  }
  tmp << "Fid: " << engine_id_ << ", ";
  tmp << "Version " << version_ << ", ";
  tmp << "HKey: " << hkey_;
  return (std::string(tmp.str()));
}

//------------------------------------------------------------------------------
// class ParserUnitCSGFileNetflow::Configuration
//------------------------------------------------------------------------------
const std::string ParserUnitCSGFileNetflow::Configuration::kPathDefault("/");
const int ParserUnitCSGFileNetflow::Configuration::kScanIntervalDefault = 30;
const int ParserUnitCSGFileNetflow::Configuration::kStatisticsIntervalDefault =
    300;
const bool ParserUnitCSGFileNetflow::Configuration::kRemoveFilesAfterDefault =
    false;
const int ParserUnitCSGFileNetflow::Configuration::kExportTimeWindowDefault = 60;

ParserUnitCSGFileNetflow::Configuration::Configuration() {
  reset();
}

ParserUnitCSGFileNetflow::Configuration::~Configuration() {
  reset();
}

void ParserUnitCSGFileNetflow::Configuration::reset(void) {
  path_ = kPathDefault;
  scan_interval_s_ = kScanIntervalDefault;
  output_ = NULL;
  export_devices_.clear();
  stat_interval_s_ = kStatisticsIntervalDefault;
  remove_files_after_ = kRemoveFilesAfterDefault;
  export_time_window_s_ = kExportTimeWindowDefault;
}

//------------------------------------------------------------------------------
void ParserUnitCSGFileNetflow::Configuration::set_path(std::string path) {
  path_ = path;
}

void ParserUnitCSGFileNetflow::Configuration::set_scan_interval_s(
    int scan_interval_s) {
  scan_interval_s_ = scan_interval_s;
}

void ParserUnitCSGFileNetflow::Configuration::set_output(
    FlowContainerBuffer* output) {
  output_ = output;
}

void ParserUnitCSGFileNetflow::Configuration::add_export_device(
    ExportDevice& device) {
  export_devices_.push_back(device);
}

void ParserUnitCSGFileNetflow::Configuration::set_stat_interval_s(
    int stat_interval_s) {
  stat_interval_s_ = stat_interval_s;
}

void ParserUnitCSGFileNetflow::Configuration::set_remove_files_after(
    bool remove_files_after) {
  remove_files_after_ = remove_files_after;
}

void ParserUnitCSGFileNetflow::Configuration::set_export_time_window_s(
    int export_time_window_s) {
  export_time_window_s_ = export_time_window_s;
}

//------------------------------------------------------------------------------
std::string ParserUnitCSGFileNetflow::Configuration::get_path(void) const {
  return (path_);
}

FlowContainerBuffer* ParserUnitCSGFileNetflow::Configuration::get_output(
    void) const {
  return (output_);
}

int ParserUnitCSGFileNetflow::Configuration::get_scan_interval_s(void) const {
  return (scan_interval_s_);
}

std::vector<ParserUnitCSGFileNetflow::ExportDevice>::const_iterator ParserUnitCSGFileNetflow::Configuration::get_export_device_begin(
    void) const {
  return (std::vector<ParserUnitCSGFileNetflow::ExportDevice>::const_iterator(
      export_devices_.begin()));
}

std::vector<ParserUnitCSGFileNetflow::ExportDevice>::const_iterator ParserUnitCSGFileNetflow::Configuration::get_export_device_end(
    void) const {
  return (std::vector<ParserUnitCSGFileNetflow::ExportDevice>::const_iterator(
      export_devices_.end()));
}

int ParserUnitCSGFileNetflow::Configuration::get_stat_interval_s(void) const {
  return (stat_interval_s_);
}

int ParserUnitCSGFileNetflow::Configuration::get_export_time_window_s(
    void) const {
  return (export_time_window_s_);
}

//------------------------------------------------------------------------------
bool ParserUnitCSGFileNetflow::Configuration::get_remove_files_after(
    void) const {
  return (remove_files_after_);
}

//------------------------------------------------------------------------------
// class ParserUnitCSGFileNetflow::Observation
//------------------------------------------------------------------------------
ParserUnitCSGFileNetflow::Observation::Observation() {
  reset();
}

void ParserUnitCSGFileNetflow::Observation::reset(void) {
  valid_ = false;
  time_s_ = 0;
  message_ = "";
}

//------------------------------------------------------------------------------
bool ParserUnitCSGFileNetflow::Observation::get_valid(void) const {
  return (valid_);
}

uint64_t ParserUnitCSGFileNetflow::Observation::get_time_s(void) const {
  return (time_s_);
}

std::string ParserUnitCSGFileNetflow::Observation::get_message(void) const {
  return (std::string(message_));
}

//------------------------------------------------------------------------------
void ParserUnitCSGFileNetflow::Observation::set_valid(bool valid) {
  valid_ = valid;
}

void ParserUnitCSGFileNetflow::Observation::set_time_s(uint64_t time_s) {
  time_s_ = time_s;
}

void ParserUnitCSGFileNetflow::Observation::set_message(
    const std::string& message) {
  message_ = message;
}

//------------------------------------------------------------------------------
// class ParserUnitCSGFileNetflow::Statistics
//------------------------------------------------------------------------------
ParserUnitCSGFileNetflow::Statistics::Statistics() {
  reset();
}

void ParserUnitCSGFileNetflow::Statistics::reset(void) {
  valid_ = false;
  time_s_ = 0;
  duration_s_ = 0;
  real_time_s_ = 0;
  real_duration_s_ = 0;
  packets_ = 0;
  iv_min_size_ = 0;
  iv_format_ = 0;
  iv_exporter_ = 0;
  iv_export_time_too_small_ = 0;
  iv_export_time_too_big_ = 0;
}

//------------------------------------------------------------------------------
bool ParserUnitCSGFileNetflow::Statistics::get_valid(void) {
  return (valid_);
}

uint64_t ParserUnitCSGFileNetflow::Statistics::get_time_s(void) {
  return (time_s_);
}

uint64_t ParserUnitCSGFileNetflow::Statistics::get_duration_s(void) {
  return (duration_s_);
}

uint64_t ParserUnitCSGFileNetflow::Statistics::get_real_time_s(void) {
  return (real_time_s_);
}

uint64_t ParserUnitCSGFileNetflow::Statistics::get_real_duration_s(void) {
  return (real_duration_s_);
}

uint64_t ParserUnitCSGFileNetflow::Statistics::get_packets(void) {
  return (packets_);
}

uint64_t ParserUnitCSGFileNetflow::Statistics::get_iv_min_size(void) {
  return (iv_min_size_);
}

uint64_t ParserUnitCSGFileNetflow::Statistics::get_iv_format(void) {
  return (iv_format_);
}

uint64_t ParserUnitCSGFileNetflow::Statistics::get_iv_exporter(void) {
  return (iv_exporter_);
}

uint64_t ParserUnitCSGFileNetflow::Statistics::get_iv_export_time_too_small(
    void) {
  return (iv_export_time_too_small_);
}

uint64_t ParserUnitCSGFileNetflow::Statistics::get_iv_export_time_too_big(
    void) {
  return (iv_export_time_too_big_);
}

//------------------------------------------------------------------------------
void ParserUnitCSGFileNetflow::Statistics::set_valid(bool valid) {
  valid_ = valid;
}

void ParserUnitCSGFileNetflow::Statistics::set_time_s(uint64_t time_s) {
  time_s_ = time_s;
}

void ParserUnitCSGFileNetflow::Statistics::set_duration_s(uint64_t duration_s) {
  duration_s_ = duration_s;
}

void ParserUnitCSGFileNetflow::Statistics::set_real_time_s(
    uint64_t real_time_s) {
  real_time_s_ = real_time_s;
}

void ParserUnitCSGFileNetflow::Statistics::set_real_duration_s(
    uint64_t real_duration_s) {
  real_duration_s_ = real_duration_s;
}

void ParserUnitCSGFileNetflow::Statistics::set_packets(uint64_t packets) {
  packets_ = packets;
}

void ParserUnitCSGFileNetflow::Statistics::set_iv_min_size(
    uint64_t iv_min_size) {
  iv_min_size_ = iv_min_size;
}

void ParserUnitCSGFileNetflow::Statistics::set_iv_format(uint64_t iv_format) {
  iv_format_ = iv_format;
}

void ParserUnitCSGFileNetflow::Statistics::set_iv_exporter(
    uint64_t iv_exporter) {
  iv_exporter_ = iv_exporter;
}

void ParserUnitCSGFileNetflow::Statistics::set_iv_export_time_too_small(
    uint64_t iv_export_time_too_small) {
  iv_export_time_too_small_ = iv_export_time_too_small;
}

void ParserUnitCSGFileNetflow::Statistics::set_iv_export_time_too_big(
    uint64_t iv_export_time_too_big) {
  iv_export_time_too_big_ = iv_export_time_too_big;
}

//------------------------------------------------------------------------------
void ParserUnitCSGFileNetflow::Statistics::inc_packets(void) {
  packets_++;
}

void ParserUnitCSGFileNetflow::Statistics::inc_iv_min_size(void) {
  iv_min_size_++;
}

void ParserUnitCSGFileNetflow::Statistics::inc_iv_format(void) {
  iv_format_++;
}

void ParserUnitCSGFileNetflow::Statistics::inc_iv_exporter(void) {
  iv_exporter_++;
}

void ParserUnitCSGFileNetflow::Statistics::inc_iv_export_time_too_small(void) {
  iv_export_time_too_small_++;
}

void ParserUnitCSGFileNetflow::Statistics::inc_iv_export_time_too_big(void) {
  iv_export_time_too_big_++;
}

//------------------------------------------------------------------------------
std::string ParserUnitCSGFileNetflow::Statistics::head_to_s(void) {
  std::stringstream tmp;
  tmp << "time_s, ";
  tmp << "duration_s, ";
  tmp << "real_time_s, ";
  tmp << "real_duration_s, ";
  tmp << "packets, ";
  tmp << "iv_min_size, ";
  tmp << "iv_format, ";
  tmp << "iv_exporter, ";
  tmp << "iv_export_time_too_small, ";
  tmp << "iv_export_time_too_big, ";
  return (std::string(tmp.str()));
}

std::string ParserUnitCSGFileNetflow::Statistics::to_s(void) const {
  std::stringstream tmp;
  tmp << time_s_ << ", ";
  tmp << duration_s_ << ", ";
  tmp << real_time_s_ << ", ";
  tmp << real_duration_s_ << ", ";
  tmp << packets_ << ", ";
  tmp << iv_min_size_ << ", ";
  tmp << iv_format_ << ", ";
  tmp << iv_exporter_ << ", ";
  tmp << iv_export_time_too_small_ << ", ";
  tmp << iv_export_time_too_big_ << ", ";
  return (std::string(tmp.str()));
}

//------------------------------------------------------------------------------
// class ParserUnitCSGFileNetflow
//------------------------------------------------------------------------------
ParserUnitCSGFileNetflow::ParserUnitCSGFileNetflow() {
  // -- CONFIGURATION ----------------------------------------------------------
  sem_init(&conf_critical_section_sem_, 0, 1);
  conf_available_ = false;
  conf_current_.reset();

  export_devices_.clear();

  // -- OBSERVATION ------------------------------------------------------------
  sem_init(&obs_critical_section_sem_, 0, 1);
  sem_init(&obs_available_sem_, 0, 0);

  // -- STATISTICS -------------------------------------------------------------
  sem_init(&stat_critical_section_sem_, 0, 1);
  sem_init(&stat_available_sem_, 0, 0);
  stat_current_.reset();
  stat_export_next_s_ = 0;

  // -- DATA STREAM  -----------------------------------------------------------
  data_output_ = NULL;
  data_container_pool_ = FlowContainerPool::instance();

  // -- OTHERS  ----------------------------------------------------------------
  time_s_ = 0;
  iv_export_time_last_jump_ = 0;
  processed_seq_numbers_.clear();
}

ParserUnitCSGFileNetflow::~ParserUnitCSGFileNetflow() {
  for (unsigned int i = 0; i < export_devices_.size(); i++) {
    if (export_devices_[i] != NULL) {
      delete export_devices_[i];
      export_devices_[i] = NULL;
    }
  }
  export_devices_.clear();
  export_devices_map_.clear();
}
//------------------------------------------------------------------------------
// class ParserUnitCSGFileNetflow CONFIGURATION
//------------------------------------------------------------------------------
void ParserUnitCSGFileNetflow::conf_push(const Configuration& config) {
  fb_sem_wait(&conf_critical_section_sem_);
  // CRITICAL SECTION::START
  conf_in_.push(config);
  conf_available_ = true;
  fb_sem_post(&conf_critical_section_sem_);
  // CRITICAL SECTION::STOP
  return;
}

void ParserUnitCSGFileNetflow::conf_pop(void) {
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

void ParserUnitCSGFileNetflow::conf_migrate(const Configuration& migrate) {
  data_output_ = migrate.get_output();
  for (unsigned int i = 0; i < export_devices_.size(); i++) {
    if (export_devices_[i] != NULL) {
      delete export_devices_[i];
      export_devices_[i] = NULL;
    }
  }
  export_devices_.clear();
  export_devices_map_.clear();

  std::vector<ExportDevice>::const_iterator iter = migrate
      .get_export_device_begin();
  std::vector<ExportDevice>::const_iterator end =
      migrate.get_export_device_end();

  while (iter != end) {
    ExportDeviceKey key(*iter->get_packet_src_addr(),
                        iter->get_flow_engine_id(), iter->get_version());

    ExportDevice* p = new ExportDevice(*iter->get_packet_src_addr(),
                                       iter->get_flow_engine_id(),
                                       iter->get_version(),
                                       iter->get_export_id());

    export_devices_.push_back(p);
    export_devices_map_[key] = p;

    iter++;
  }

  conf_current_ = migrate;
  return;
}

//------------------------------------------------------------------------------
// class ParserUnitCSGFileNetflow OBSERVATION
//------------------------------------------------------------------------------
ParserUnitCSGFileNetflow::Observation ParserUnitCSGFileNetflow::obs_get(void) {
  ParserUnitCSGFileNetflow::Observation obs;
  fb_sem_wait(&obs_available_sem_);
  // WAIT ON DATA
  fb_sem_wait(&obs_critical_section_sem_);
  // CRITICAL SECTION::START
  assert(obs_out_.size() >= 1);
  // we need some data
  obs = obs_out_.front();
  obs_out_.pop();
  fb_sem_post(&obs_critical_section_sem_);
  // CRITICAL SECTION::STOP
  obs.set_valid(true);
  return (ParserUnitCSGFileNetflow::Observation(obs));
}

void ParserUnitCSGFileNetflow::obs_push(const std::string& message) {
  ParserUnitCSGFileNetflow::Observation obs;
  obs.set_time_s(time_s_);
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
// class ParserUnitCSGFileNetflow STATISTICS
//------------------------------------------------------------------------------
ParserUnitCSGFileNetflow::Statistics ParserUnitCSGFileNetflow::stat_get(void) {
  ParserUnitCSGFileNetflow::Statistics stat;
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
  return (ParserUnitCSGFileNetflow::Statistics(stat));
}

void ParserUnitCSGFileNetflow::stat_export(void) {
  // time_s // packet time
  uint64_t real_time_s = time(NULL);  // real time
  if (stat_current_.get_time_s() == 0) {
    // init

  } else {
    // update duration
    stat_current_.set_duration_s(time_s_ - stat_current_.get_time_s());
    stat_current_.set_real_duration_s(
        real_time_s - stat_current_.get_real_time_s());

    // update exporter stats
    std::vector<ExportDevice*>::iterator iter, end;
    iter = export_devices_.begin();
    end = export_devices_.end();

    while (iter != end) {
      // do stuff per exporter
      obs_push((*iter)->get_stat_s());
      iter++;
    }

    fb_sem_wait(&stat_critical_section_sem_);
    // CRITICAL SECTION::START
    stat_out_.push(stat_current_);  // push data
    fb_sem_post(&stat_critical_section_sem_);
    // CRITICAL SECTION::STOP

    fb_sem_post(&stat_available_sem_);
    // SIGNAL DATA

    stat_current_.reset();
  }
  stat_current_.set_time_s(time_s_);
  stat_current_.set_real_time_s(real_time_s);

  int interval_s = conf_current_.get_stat_interval_s();
  stat_export_next_s_ = ((time_s_ / interval_s) + 1) * interval_s;
}

//------------------------------------------------------------------------------
// class ParserUnitCSGFileNetflow DATA
//------------------------------------------------------------------------------
void ParserUnitCSGFileNetflow::data_process_file_set(
    const std::vector<std::string>& set) {

  // --- send an observation that we start with a new file
  std::stringstream msg;
  msg << "New Set: <";
  msg << set[kFileSet19991Dat] << ", ";
  msg << set[kFileSet19991Stat] << ", ";
  msg << set[kFileSet19993Dat] << ", ";
  msg << set[kFileSet19993Stat] << " >";
  obs_push(msg.str());

  // 0. prepare: fc, iv ...
  // 1. open the stream
  // 2. process the packets
  // 3. flush the carrier
  // -- return stats/obs when requested

  // 0. prepare: fc, iv ...
  int err;
  Packet* packet_p;
  FlowContainer* fc = data_container_pool_->pop();
  ExportDevice* export_device = NULL;

  // IV CHECK: Input Validation Check
  int iv_export_time_gap_max = conf_current_.get_export_time_window_s();

  // 1. open the stream
  err = data_source_.set_file_set(set[kFileSet19991Dat], set[kFileSet19991Stat],
                                  set[kFileSet19993Dat],
                                  set[kFileSet19993Stat]);
  if (err != PacketSourceCSGFile::kFileSetOK) {
    obs_push("FileSet not OK");
    std::stringstream err_msg;
    err_msg << "Stream Failed! FileSet not OK" << std::endl
            << set[kFileSet19991Dat] << std::endl
            << set[kFileSet19991Stat] << std::endl
            << set[kFileSet19993Dat] << std::endl
            << set[kFileSet19993Stat] << std::endl;
    throw FlowBoxE(err_msg.str(), __FILE__, __LINE__);
  }

  // 2. process the packets
  while (true) {
    // statistic export?
    if (time_s_ >= stat_export_next_s_) {
      stat_export();
    };

    // get a packet
    err = data_source_.get(&packet_p);
    if (err == PacketSourceCSGFile::kFileSetOK) {
      stat_current_.inc_packets();

#ifdef PARSER_UNIT_CSG_FILE_NETFLOW_DEBUG
      printf("ParserUnitCSGFileNetflow[%p]:: %" PRIu64 ", %i\n",
          this,
          stat_current_.get_packets(),
          packet_p->buffer_length_);

      if (stat_current_.get_packets()% 1000 == 0) {
        std::cout << stat_current_.get_packets() << std::endl;
      }
#endif

      // IV CHECK: MIN SIZE -- Do we have enough data to do something?
      if (packet_p->buffer_length_ < kIVMinFlowSize) {
        stat_current_.inc_iv_min_size();
        continue;
      }

      // IV CHECK: FORMAT -- Is a NetFlow v9 or v5 packet
      uint32_t version = ParserNetflow9::get_version(*packet_p);
      if (version != 5 and version != 9) {
        // NOT Netflow 5 or 9
        stat_current_.inc_iv_format();
        continue;
      }

      uint32_t engine_id;
      if (version == 5) {
        engine_id = ParserNetflow5::get_engine_id(*packet_p);
      } else if (version == 9) {
        engine_id = ParserNetflow9::get_engine_id(*packet_p);
      } else {
        // at this point of code we have only 5 or 9 records
        throw FlowBoxE(
            "We should have only NetFlow 5 or 9 Records, nothing else ... ",
            __FILE__, __LINE__);
      };

      // IV CHECK: EXPORTER Belongs to a registered exporter
      export_device = get_export_device(packet_p->addr_, engine_id, version);
      if (export_device == NULL) {
        stat_current_.inc_iv_exporter();
        continue;
      }

      // TODO(schadomi): Validate correctness of this check
      // IV CHECK: Export Time is continues in time
      // JUMP FORWARD: Jump to the future (gap?)
      // JUMP BACKWARD: Jump to the past (router out of time sync?)
      uint64_t export_time_s = ParserNetflow9::get_export_time_s(*packet_p);
      if (time_s_ == 0) {
#ifdef PARSER_UNIT_CSG_FILE_NETFLOW_DEBUG
        std::cout << "PacketTime INIT to " << export_time_s << std::endl;
#endif
        time_s_ = export_time_s;
      } else if (export_time_s < time_s_ - iv_export_time_gap_max) {
#ifdef PARSER_UNIT_CSG_FILE_NETFLOW_DEBUG
        std::cout << "Jump backward " << export_time_s << std::endl;
#endif
        stat_current_.inc_iv_export_time_too_small();
        continue;
      } else if (export_time_s < time_s_ + iv_export_time_gap_max) {
        time_s_ = export_time_s;
      } else if (export_time_s > time_s_ + iv_export_time_gap_max) {
#ifdef PARSER_UNIT_CSG_FILE_NETFLOW_DEBUG
        std::cout << "Jump forward " << export_time_s << std::endl;
#endif
        // jump detected ... what should we do?
        if ((export_time_s < iv_export_time_last_jump_ - iv_export_time_gap_max)
            or (export_time_s
                  > iv_export_time_last_jump_ + iv_export_time_gap_max)) {
          // reject first jump
          iv_export_time_last_jump_ = export_time_s;
          stat_current_.inc_iv_export_time_too_big();
          continue;
        } else {
          // we jumped again,  do the jump!
          time_s_ = export_time_s;
          // reset reporters -> templates are probably changed
          reset_exporters();
        }
      }

      // Do we have enough space in this flow container?
      if (fc->available()
          < ParserNetflow9::get_estimated_flow_count(*packet_p)) {
        // flush this container first
        if (data_output_ != NULL)
          data_output_->push(fc);
        else
          data_container_pool_->push(fc);

        // get a new container (blocking call)
        fc = data_container_pool_->pop();
      }

      // parse the packet
      export_device->parse(*packet_p, fc);

      // DEBUG ONLY DS 2012-05-08
#ifdef PARSER_UNIT_CSG_FILE_NETFLOW_DEBUG
      fc->update_used_by(ParserNetflow9::get_estimated_flow_count(*packet_p));
#endif

    } else if (err == PacketSourceCSGFile::kFileSetRetry) {
      continue;
    } else if (err == PacketSourceCSGFile::kFileSetEOF) {
#ifdef PARSER_UNIT_CSG_FILE_NETFLOW_DEBUG
      std::cout << "EOF -- get next one " << std::endl;
      reset_exporters();
#endif
      break;
    } else {
      throw FlowBoxE("SteamFailed", __FILE__, __LINE__);
    }
  }  // while:  PROCESS FILE SET

  // 3. flush the carrier
  if (data_output_ != NULL)
    data_output_->push(fc);
  else
    data_container_pool_->push(fc);
}

// MAIN LOOP -------------------------------------------------------------------
void ParserUnitCSGFileNetflow::data_process(void) {
  obs_push("Data processing started");

  // scan and process csg flow files
  while (true) {
    // process configuration updates
    if (conf_available_) {
      conf_pop();
      continue;  // process all configurations first
    }

    // scan for files
    if (file_sets_.size() == 0) {
      obs_push("Scan input folder");
      scan_folder();
    }

    // no (more) files found
    if (file_sets_.size() == 0) {
      // sleep x seconds or abort processing (based on configured value)
      if (conf_current_.get_scan_interval_s() > 0) {
        sleep(conf_current_.get_scan_interval_s());
        continue;
      } else {
        break;  // done
      }
    }

    // get the next file set
    assert(file_sets_.size() > 0);
    std::vector<std::string> set = file_sets_.front();
    file_sets_.pop();

    // process the file set
    data_process_file_set(set);

    // remove the processed files if requeted
    if (conf_current_.get_remove_files_after()) {
      remove(set[kFileSet19991Dat].c_str());
      remove(set[kFileSet19991Stat].c_str());
      remove(set[kFileSet19993Dat].c_str());
      remove(set[kFileSet19993Stat].c_str());
    }
  }  // while -- scan and process

  // send the fin signal
  if (data_output_ != NULL) {
    data_output_->signal_fin();
  }
  obs_push("Data processing finished");
}

// OTHERS ----------------------------------------------------------------------
int ParserUnitCSGFileNetflow::grep_seq_number(std::string file_name) {
  std::string separator("_");
  size_t first = file_name.find(separator);
  first++;
  size_t second = file_name.find(separator, first);
  std::string seq_str = file_name.substr(first, second - first);
  return (atoi(seq_str.c_str()));
}

std::map<int, std::vector<std::string> > ParserUnitCSGFileNetflow::grep_files(
    std::string folder) {

  std::map<int, std::vector<std::string> > files;
  struct dirent **namelist;
  int n;

  n = scandir(folder.c_str(), &namelist, 0, alphasort);
  if (n > 0) {
    while (n--) {
      std::string file_name(namelist[n]->d_name);
      free(namelist[n]);
      // is a 19991 dat file?
      if ((file_name.find("19991_") != std::string::npos)
          and (file_name.find(".dat.bz2") != std::string::npos)) {
        int seq_number = grep_seq_number(file_name);
        files[seq_number].resize(4);
        files[seq_number][kFileSet19991Dat] = file_name;
      } else if ((file_name.find("19993_") != std::string::npos)
          and (file_name.find(".dat.bz2") != std::string::npos)) {
        int seq_number = grep_seq_number(file_name);
        files[seq_number].resize(4);
        files[seq_number][kFileSet19993Dat] = file_name;
      } else if ((file_name.find("19991_") != std::string::npos)
          and (file_name.find(".stat.bz2") != std::string::npos)) {
        int seq_number = grep_seq_number(file_name);
        files[seq_number].resize(4);
        files[seq_number][kFileSet19991Stat] = file_name;
      } else if ((file_name.find("19993_") != std::string::npos)
          and (file_name.find(".stat.bz2") != std::string::npos)) {
        int seq_number = grep_seq_number(file_name);
        files[seq_number].resize(4);
        files[seq_number][kFileSet19993Stat] = file_name;
      }
    }
    free(namelist);
  }
#ifdef PARSER_UNIT_CSG_FILE_NETFLOW_DEBUG
  for (std::map<int, std::vector<std::string> >::iterator file_iter(files.begin());
       file_iter != files.end(); file_iter++) {
    std::cout << (file_iter->first) << std::endl;
    std::cout << (file_iter->second)[kFileSet19991Dat] << std::endl;
    std::cout << (file_iter->second)[kFileSet19991Stat] << std::endl;
    std::cout << (file_iter->second)[kFileSet19993Dat] << std::endl;
    std::cout << (file_iter->second)[kFileSet19991Stat] << std::endl;
  }
#endif

  return (std::map<int, std::vector<std::string> >(files));
}

void ParserUnitCSGFileNetflow::scan_folder(void) {
  std::string path = conf_current_.get_path();
  std::map<int, std::vector<std::string> > files(grep_files(path));

  std::list<int> seq_numbers;

  for (std::map<int, std::vector<std::string> >::iterator file_iter(
      files.begin()); file_iter != files.end(); file_iter++) {
    // sanity check: 4 files required
    if (((file_iter->second)[kFileSet19991Dat].size() > 1)
        and ((file_iter->second)[kFileSet19991Stat].size() > 1)
        and ((file_iter->second)[kFileSet19993Dat].size() > 1)
        and ((file_iter->second)[kFileSet19993Stat].size() > 1)) {
      seq_numbers.push_back(file_iter->first);
    }
  }
  seq_numbers.sort();
  for (std::list<int>::iterator seq(seq_numbers.begin());
       seq != seq_numbers.end(); seq++) {
    if (processed_seq_numbers_.find(*seq) == processed_seq_numbers_.end()) {
      std::stringstream msg;
      msg << "Found new set with seq number " << *seq;
      obs_push(msg.str());

      processed_seq_numbers_.insert(*seq);
      std::vector<std::string> set;
      set.resize(4);
      set[kFileSet19991Dat] = path + "/" + files[*seq][kFileSet19991Dat];
      set[kFileSet19991Stat] = path + "/" + files[*seq][kFileSet19991Stat];
      set[kFileSet19993Dat] = path + "/" + files[*seq][kFileSet19993Dat];
      set[kFileSet19993Stat] = path + "/" + files[*seq][kFileSet19993Stat];
      file_sets_.push(set);
    } else {
#ifdef PARSER_UNIT_CSG_FILE_NETFLOW_DEBUG
      std::cout << "Old Seq: " << *seq << std::endl;
#endif
    }
  }
}

ParserUnitCSGFileNetflow::ExportDevice*
ParserUnitCSGFileNetflow::get_export_device(const struct sockaddr& src_addr,
                                            const int engine_id,
                                            const int version) {
  ExportDeviceKey key(src_addr, engine_id, version);
  ExportDevice_map::iterator iter;

  // hash table lookup
  iter = export_devices_map_.find(key);
#ifdef PARSER_UNIT_CSG_FILE_NETFLOW_DEBUG
  std::cout << key.to_s() << std::endl;
#endif

  if (iter != export_devices_map_.end()) {
#ifdef PARSER_UNIT_CSG_FILE_NETFLOW_DEBUG
    std::cout << (iter->first).to_s() << std::endl;
#endif
    return (iter->second);
  } else {
#ifdef PARSER_UNIT_CSG_FILE_NETFLOW_DEBUG
    std::cout << key.to_s() << std::endl;
    std::cout << "-- NULL --" << std::endl;
#endif
    return (NULL);
  }
}

void ParserUnitCSGFileNetflow::reset_exporters(void) {
  ExportDevice_map::iterator iter = export_devices_map_.begin();
  while (iter != export_devices_map_.end()) {
    iter->second->reset();
    iter++;
  }
}
