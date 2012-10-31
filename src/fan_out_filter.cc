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
 * @file   fan_out_filter.cc
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @author Daniel Aschwanden  <nimdanitro@gmail.com>
 *
 * @date   October, 2012
 * @brief  Eliminate noise caused by hosts scanning the network.
 *
 * The NoiseFilter identifies flows that are caused by hosts scanning the
 * and removes does from the data stream. To do so, we relay on the fan out
 * property of network scanners.
 *
 */

#include "fan_out_filter.h"

//------------------------------------------------------------------------------
// class FanOutFilter::Const
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// class FanOutFilter::Configuration
//------------------------------------------------------------------------------
const int FanOutFilter::Configuration::kStatIntervalDefault = 300;
const int FanOutFilter::Configuration::kInBadMinDefault = 2;
const int FanOutFilter::Configuration::kInRatioMinDefault = 4;
const int FanOutFilter::Configuration::kOutBadMinDefault = 2;
const int FanOutFilter::Configuration::kOutRatioMinDefault = 4;

const int FanOutFilter::Configuration::kAllInMinHTDefault = 1000000;
const int FanOutFilter::Configuration::kAllOutMinHTDefault = 1000000;
const int FanOutFilter::Configuration::kSelectionInMinHTDefault = 1000000;
const int FanOutFilter::Configuration::kSelectionOutMinHTDefault = 1000000;

FanOutFilter::Configuration::Configuration() {
  reset();
}

void FanOutFilter::Configuration::reset(void) {
  input_ = NULL;
  output_good_ = NULL;
  output_bad_ = NULL;

  // Reset the values to the default constants

  // FIXME (asdaniel): get rid of the stat interval!!
  stat_interval_s_ = kStatIntervalDefault;
  in_bad_min_ = kInBadMinDefault;
  in_ratio_min_ = kInRatioMinDefault;
  out_bad_min_ = kOutBadMinDefault;
  out_ratio_min_ = kOutBadMinDefault;
  all_in_ht_min_ = kAllInMinHTDefault;
  all_out_ht_min_ = kAllOutMinHTDefault;
  selection_in_ht_min_ = kSelectionInMinHTDefault;
  selection_out_ht_min_ = kSelectionInMinHTDefault;
}

//------------------------------------------------------------------------------
BiFlowContainerBuffer* FanOutFilter::Configuration::input(void) const {
  return (input_);
}

BiFlowContainerBuffer* FanOutFilter::Configuration::output_good(void) const {
  return (output_good_);
}

BiFlowContainerBuffer* FanOutFilter::Configuration::output_bad(void) const {
  return (output_bad_);
}

uint64_t FanOutFilter::Configuration::stat_interval_s(void) const {
  return (stat_interval_s_);
}

int FanOutFilter::Configuration::in_bad_min() const {
  return (in_bad_min_);
}

int FanOutFilter::Configuration::in_ratio_min() const {
  return (in_ratio_min_);
}

int FanOutFilter::Configuration::out_bad_min() const {
  return (out_bad_min_);
}

int FanOutFilter::Configuration::out_ratio_min() const {
  return (out_ratio_min_);
}

//------------------------------------------------------------------------------
void FanOutFilter::Configuration::set_input(BiFlowContainerBuffer* input) {
  input_ = input;
}

void FanOutFilter::Configuration::set_output_good(
    BiFlowContainerBuffer* output) {
  output_good_ = output;
}

void FanOutFilter::Configuration::set_output_bad(
    BiFlowContainerBuffer* output) {
  output_bad_ = output;
}

void FanOutFilter::Configuration::set_stat_interval(uint64_t stat_interval_s) {
  stat_interval_s_ = stat_interval_s;
}

void FanOutFilter::Configuration::set_in_bad_min(int in_bad_min) {
  in_bad_min_ = in_bad_min;
}

void FanOutFilter::Configuration::set_in_ratio_min(int in_ratio_min) {
  in_ratio_min_ = in_ratio_min;
}

void FanOutFilter::Configuration::set_out_bad_min(int out_bad_min) {
  out_bad_min_ = out_bad_min;
}

void FanOutFilter::Configuration::set_out_ratio_min(int out_ratio_min) {
  out_ratio_min_ = out_ratio_min;
}

//------------------------------------------------------------------------------
// class FanOutFilter::Observation
//------------------------------------------------------------------------------
FanOutFilter::Observation::Observation() {
  reset();
}

void FanOutFilter::Observation::reset(void) {
  valid_ = false;
  time_s_ = 0;
  message_ = "";
}

//------------------------------------------------------------------------------
bool FanOutFilter::Observation::valid(void) const {
  return (valid_);
}

uint64_t FanOutFilter::Observation::time_s(void) const {
  return (time_s_);
}

std::string FanOutFilter::Observation::message(void) const {
  return (std::string(message_));
}

//------------------------------------------------------------------------------
void FanOutFilter::Observation::set_valid(bool valid) {
  valid_ = valid;
}

void FanOutFilter::Observation::set_time_s(uint64_t time_s) {
  time_s_ = time_s;
}

void FanOutFilter::Observation::set_message(const std::string& message) {
  message_ = message;
}

//------------------------------------------------------------------------------
// class FanOutFilter::Statistics
//------------------------------------------------------------------------------
FanOutFilter::Statistics::Statistics() {
  reset();
}

void FanOutFilter::Statistics::reset(void) {
  valid_ = false;
  time_s_ = 0;
  duration_s_ = 0;
  duration_all_build_ = 0;
  duration_selection_build_ = 0;
  duration_split_ = 0;

  biflows_in_ = 0;
  biflows_kept_ = 0;
  biflows_filtered_ = 0;
  all_in_ = 0;
  all_out_ = 0;
  selection_in_ = 0;
  selection_out_ = 0;
  hosts_in_ = 0;
  hosts_out_ = 0;

  buckets_all_in_ = 0;
  buckets_all_out_ = 0;
  buckets_selection_in_ = 0;
  buckets_selection_out_ = 0;
}

//------------------------------------------------------------------------------
uint64_t FanOutFilter::Statistics::all_in() const {
  return all_in_;
}

void FanOutFilter::Statistics::set_all_in(uint64_t allIn) {
  all_in_ = allIn;
}

uint64_t FanOutFilter::Statistics::all_out() const {
  return all_out_;
}

void FanOutFilter::Statistics::set_all_out(uint64_t allOut) {
  all_out_ = allOut;
}

uint64_t FanOutFilter::Statistics::biflows_filtered() const {
  return biflows_filtered_;
}

void FanOutFilter::Statistics::set_biflows_filtered(uint64_t biflowsFiltered) {
  biflows_filtered_ = biflowsFiltered;
}

uint64_t FanOutFilter::Statistics::biflows_in() const {
  return biflows_in_;
}

void FanOutFilter::Statistics::set_biflows_in(uint64_t biflowsIn) {
  biflows_in_ = biflowsIn;
}

uint64_t FanOutFilter::Statistics::biflows_kept() const {
  return biflows_kept_;
}

void FanOutFilter::Statistics::set_biflows_kept(uint64_t biflowsKept) {
  biflows_kept_ = biflowsKept;
}

uint64_t FanOutFilter::Statistics::buckets_all_in() const {
  return buckets_all_in_;
}

void FanOutFilter::Statistics::set_buckets_all_in(uint64_t buckets) {
  buckets_all_in_ = buckets;
}

uint64_t FanOutFilter::Statistics::buckets_all_out() const {
  return buckets_all_out_;
}

void FanOutFilter::Statistics::set_buckets_all_out(uint64_t buckets) {
  buckets_all_out_ = buckets;
}

uint64_t FanOutFilter::Statistics::buckets_selection_in() const {
  return buckets_selection_in_;
}
void FanOutFilter::Statistics::set_buckets_selection_in(uint64_t buckets) {
  buckets_selection_in_ = buckets;
}

uint64_t FanOutFilter::Statistics::buckets_selection_out() const {
  return buckets_selection_out_;
}

void FanOutFilter::Statistics::set_buckets_selection_out(uint64_t buckets) {
  buckets_selection_out_ = buckets;
}

uint64_t FanOutFilter::Statistics::duration_all_build() const {
  return duration_all_build_;
}

void FanOutFilter::Statistics::set_duration_all_build(
    uint64_t durationAllBuild) {
  duration_all_build_ = durationAllBuild;
}

uint64_t FanOutFilter::Statistics::duration_s() const {
  return duration_s_;
}

void FanOutFilter::Statistics::set_duration_s(uint64_t durationS) {
  duration_s_ = durationS;
}

uint64_t FanOutFilter::Statistics::duration_selection_build() const {
  return duration_selection_build_;
}

void FanOutFilter::Statistics::set_duration_selection_build(
    uint64_t durationSelectionBuild) {
  duration_selection_build_ = durationSelectionBuild;
}

uint64_t FanOutFilter::Statistics::duration_split() const {
  return duration_split_;
}

void FanOutFilter::Statistics::set_duration_split(uint64_t durationSplit) {
  duration_split_ = durationSplit;
}

uint64_t FanOutFilter::Statistics::hosts_in() const {
  return hosts_in_;
}

void FanOutFilter::Statistics::set_hosts_in(uint64_t hostsIn) {
  hosts_in_ = hostsIn;
}

uint64_t FanOutFilter::Statistics::hosts_out() const {
  return hosts_out_;
}

void FanOutFilter::Statistics::set_hosts_out(uint64_t hostsOut) {
  hosts_out_ = hostsOut;
}

uint64_t FanOutFilter::Statistics::selection_in() const {
  return selection_in_;
}

void FanOutFilter::Statistics::set_selection_in(uint64_t selectionIn) {
  selection_in_ = selectionIn;
}

uint64_t FanOutFilter::Statistics::selection_out() const {
  return selection_out_;
}

void FanOutFilter::Statistics::set_selection_out(uint64_t selectionOut) {
  selection_out_ = selectionOut;
}

uint64_t FanOutFilter::Statistics::time_s() const {
  return time_s_;
}

void FanOutFilter::Statistics::set_time_s(uint64_t timeS) {
  time_s_ = timeS;
}

bool FanOutFilter::Statistics::valid() const {
  return valid_;
}

void FanOutFilter::Statistics::set_valid(bool valid) {
  valid_ = valid;
}

//------------------------------------------------------------------------------
std::string FanOutFilter::Statistics::head_to_s(void) const {
  std::stringstream head;
  head << "time_s, ";
  head << "duration_s, ";
  head << "duration_all_build_, ";
  head << "duration_selection_build_, ";
  head << "duration_split_, ";
  head << "biflows_in_, ";
  head << "biflows_kept_, ";
  head << "biflows_filtered_, ";
  head << "all_in_, ";
  head << "all_out_, ";
  head << "buckets_all_in_, ";
  head << "buckets_all_out_, ";
  head << "hosts_in_, ";
  head << "hosts_out_, ";
  head << "selection_in_, ";
  head << "selection_out_, ";
  head << "buckets_selection_in_, ";
  head << "buckets_selection_out_";
  return (std::string(head.str()));
}

std::string FanOutFilter::Statistics::to_s(void) const {
  std::stringstream stat;
  stat << time_s_ << ", ";
  stat << duration_s_ << ", ";
  stat << duration_all_build_ << ", ";
  stat << duration_selection_build_ << ", ";
  stat << duration_split_ << ", ";
  stat << biflows_in_ << ", ";
  stat << biflows_kept_ << ", ";
  stat << biflows_filtered_ << ", ";
  stat << all_in_ << ", ";
  stat << all_out_ << ", ";
  stat << buckets_all_in_ << ", ";
  stat << buckets_all_out_ << ", ";
  stat << hosts_in_ << ", ";
  stat << hosts_out_ << ", ";
  stat << selection_in_ << ", ";
  stat << selection_out_ << ", ";
  stat << buckets_selection_in_ << ", ";
  stat << buckets_selection_out_ << ", ";
  return (std::string(stat.str()));
}

//------------------------------------------------------------------------------
// class FanOutFilter ALL data structure
//------------------------------------------------------------------------------
FanOutFilter::All::All()
    : out_(0),
      bi_(0) {
}

//------------------------------------------------------------------------------
// class FanOutFilter Selection data structure
//------------------------------------------------------------------------------
FanOutFilter::Selection::Selection()
    : out_(0),
      bi_(0) {
  degree_out_.clear();
  degree_bi_.clear();
}

FanOutFilter::Selection::~Selection() {
  degree_out_.clear();
  degree_bi_.clear();
}

void FanOutFilter::Selection::add(const BiFlow& biflow,
                                  BiFlow::Key1l::Direction direction) {
  BiFlow::Key3l key3;
  bool in;
  if (direction == BiFlow::Key1l::kIn) {
    key3.from(&biflow, BiFlow::Key3l::kIn);
    in = true;
  } else if (direction == BiFlow::Key1l::kOut) {
    key3.from(&biflow, BiFlow::Key3l::kOut);
    in = false;
  } else {
    std::stringstream err_msg;
    err_msg << "FanOutFilter::Selection::add: "
            << "Wrong direction selection, only in or out!";
    throw FlowBoxE(err_msg.str(), __FILE__, __LINE__);
  }

  int protocol = biflow.protocol_;
  // do we have a udp or tcp biflow?
  if (protocol == 6 or protocol == 17) {
    // in case of TCP we require at least two packets in each direction
    if (protocol == 6 and biflow.in_out_packets_ > 2
        and biflow.out_in_packets_ > 2) {
      bi_++;
      degree_bi_[key3]++;

      // in case of UDP we want just more than one packet in each direction
      // to be bidirectional..
    } else if (protocol == 17 and biflow.in_out_packets_ > 0
        and biflow.out_in_packets_ > 0) {
      bi_++;
      degree_bi_[key3]++;

    } else if (in and biflow.in_out_packets_ > 0
        and biflow.out_in_packets_ == 0) {
      out_++;
      degree_out_[key3]++;

    } else if (!in and biflow.in_out_packets_ == 0
        and biflow.out_in_packets_ > 0) {
      out_++;
      degree_out_[key3]++;
    }
  }
}

std::string FanOutFilter::Selection::to_s() const {
  std::stringstream tmp;
  tmp << out_ << ", ";
  tmp << degree_out_.size() << ", ";
  tmp << bi_ << ", ";
  tmp << degree_bi_.size();
  return (std::string(tmp.str()));
}

//------------------------------------------------------------------------------
// class FanOutFilter
//------------------------------------------------------------------------------
FanOutFilter::FanOutFilter()
    : data_input_(NULL),
      data_output_good_(NULL),
      data_output_bad_(NULL),
      biflow_pool_(BiFlowPool::instance()),
      biflow_container_pool_(BiFlowContainerPool::instance()) {

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

  // OTHER ---------------------------------------------------------------------
  all_in_ht_.clear();
  all_out_ht_.clear();
  selection_in_ht_.clear();
  selection_out_ht_.clear();
}

FanOutFilter::~FanOutFilter() {
}

//------------------------------------------------------------------------------
// class FanOutFilter CONFIGURATION
//------------------------------------------------------------------------------
void FanOutFilter::conf_push(Configuration config) {
  fb_sem_wait(&conf_critical_section_sem_);
  // CRITICAL SECTION::START
  conf_in_.push(config);
  conf_available_ = true;
  fb_sem_post(&conf_critical_section_sem_);
  // CRITICAL SECTION::STOP
  return;
}

void FanOutFilter::conf_pop(void) {
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

void FanOutFilter::conf_migrate(const Configuration& migrate) {
  obs_push("Configuration migration");
  data_input_ = migrate.input();
  data_output_good_ = migrate.output_good();
  data_output_bad_ = migrate.output_bad();

  stat_interval_s_ = migrate.stat_interval_s();
  in_bad_min_ = migrate.in_bad_min();
  in_ratio_min_ = migrate.in_ratio_min();
  out_bad_min_ = migrate.out_bad_min();
  out_ratio_min_ = migrate.out_ratio_min();
  return;
}

//------------------------------------------------------------------------------
// class FanOutFilter OBSERVATION
//------------------------------------------------------------------------------
FanOutFilter::Observation FanOutFilter::obs_get(void) {
  FanOutFilter::Observation obs;
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
  return (FanOutFilter::Observation(obs));
}

void FanOutFilter::obs_push(const std::string& message) {
  FanOutFilter::Observation obs;
  obs.set_time_s(stat_current_.time_s());
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
// class FanOutFilter STATISTICS
//------------------------------------------------------------------------------
FanOutFilter::Statistics FanOutFilter::stat_get(void) {
  FanOutFilter::Statistics stat;
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
  return (FanOutFilter::Statistics(stat));
}

void FanOutFilter::stat_export(uint64_t now_s) {
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
  stat_export_next_s_ = ((now_s / stat_interval_s_) + 1) * stat_interval_s_;
}

//------------------------------------------------------------------------------
// class FanOutFilter DATA
//------------------------------------------------------------------------------
void FanOutFilter::data_process(void) {
  obs_push("data_process -- started");

  BiFlowContainer* bfc_in = NULL;
  uint64_t time = 0;
  while (true) {
    // do we have some a new config?
    if (conf_available_)
      conf_pop();

    // ups no input buffer ...
    if (data_input_ == NULL) {
      std::cout << "FanOutFilter::data_thread_main -- empty data_input_"
                << std::endl;
      obs_push("data_thread_main -- empty data_input_");
      sleep(5);
      continue;
    }

    // blocking wait on some input
    bfc_in = data_input_->pop();

    // since the locking wait can be interrupted
    // by system interrupts, timeouts or signals
    // we have to check if there is the pointer
    // is valid.
    if (bfc_in == NULL) {
      // invalid data but WHY?

      // FIN: is the processing over?
      if (data_input_->is_state_fin()) {
        // push/flush our internal results
        // NOP

        // forward the signal to the next buffer (if possible)
        if (data_output_good_ != NULL) {
          data_output_good_->signal_fin();
        }
        if (data_output_bad_ != NULL) {
          data_output_bad_->signal_fin();
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
      // Split up the incoming biflows into good and bad and forward them
      BiFlow *biflows_in, *biflows_good, *biflows_bad;

      time = bfc_in->time();
      biflows_in = bfc_in->biflows();
      data_process(biflows_in, biflows_good, biflows_bad, time);

      // forward the buffer to the next element
      if (data_output_good_ != NULL and biflows_good != NULL) {
        // recycle the in container
        bfc_in->set_time(time);
        bfc_in->set_biflows(biflows_good);
        data_output_good_->push(bfc_in);
      } else {
        // make sure we set the good biflows in the container if we want to
        // delete the container and the biflows (if we don't set the biflows,
        // it will delete perhaps not all biflows and perhaps also some
        // bad biflows!)
        bfc_in->set_biflows(biflows_good);
        biflow_container_pool_->push(bfc_in);
      }

      if (data_output_bad_ != NULL) {
        // we need another BiFlowContainer
        BiFlowContainer* bfc = biflow_container_pool_->pop();
        // set the timestamp..
        bfc->set_time(time);
        // ...and the bad biflows..
        bfc->set_biflows(biflows_bad);
        // push the container on the buffer...
        data_output_bad_->push(bfc);
      } else {
        // we don't have a second
        biflow_pool_->delete_biflows(biflows_bad);
      }

      // export the statistics
      stat_export(time);
    }
  }  // while(true)
  // ... say goodby
  // clean up
  // NOP
  obs_push("data_thread_main -- finished");
}

void FanOutFilter::data_process(BiFlow* p_in, BiFlow* & p_out_good,
                                BiFlow* & p_out_bad, uint64_t time) {
  time_t begin, end;
  std::time(&begin);
  stat_current_.set_time_s(time);

  BiFlow* p_good = NULL;
  BiFlow* p_possibly_bad = NULL;
  BiFlow* p_bad = NULL;

  // Clear the hash tables
  all_in_ht_.clear();
  all_out_ht_.clear();
  selection_in_ht_.clear();
  selection_out_ht_.clear();

  // Build the all hash table
  all_build(p_in);

  all_shrink(&all_in_ht_, in_bad_min_, in_ratio_min_);
  all_shrink(&all_out_ht_, out_bad_min_, out_ratio_min_);

  // Build the selection hash table and do a first separation
  selection_build(p_in, p_good, p_possibly_bad);

  selection_shrink(&selection_in_ht_, in_bad_min_, in_ratio_min_);
  selection_shrink(&selection_out_ht_, out_bad_min_, out_ratio_min_);

  stat_current_.set_hosts_in(selection_in_ht_.size());
  stat_current_.set_hosts_out(selection_out_ht_.size());

  // split now the possible bad into bad and good ones
  split(p_possibly_bad, p_good, p_bad);
  stat_current_.set_biflows_filtered(
      stat_current_.biflows_in() - stat_current_.biflows_kept());
  stat_current_.set_buckets_all_in(all_in_ht_.bucket_count());
  stat_current_.set_buckets_all_out(all_out_ht_.bucket_count());
  stat_current_.set_buckets_selection_in(selection_in_ht_.bucket_count());
  stat_current_.set_buckets_selection_out(selection_out_ht_.bucket_count());

  // Clean up
  all_in_ht_.clear();
  all_out_ht_.clear();
  selection_in_ht_.clear();
  selection_out_ht_.clear();

  std::time(&end);
  stat_current_.set_duration_s(std::difftime(end, begin));

  // Set the output
  p_out_good = p_good;
  p_out_bad = p_bad;
  return;
}

void FanOutFilter::all_build(BiFlow* p) {
  uint64_t biflow_counter = 0;
  time_t begin, end;
  // tic
  std::time(&begin);
  BiFlow::Key1l key1_in;
  BiFlow::Key1l key1_out;

  while (p != NULL and p->valid_) {
    // get the protocol of the biflow
    int protocol = p->protocol_;

    // Do we have a udp or tcp flow?!
    if (protocol == 6 or protocol == 17) {
      key1_in.from(p, BiFlow::Key1l::kIn);
      key1_out.from(p, BiFlow::Key1l::kOut);

      // TCP Case... we must have at least a handshake..
      if (protocol == 6 and p->in_out_packets_ > 2 and p->out_in_packets_ > 2) {
        // Update the all in HT with the in key..
        all_in_ht_[key1_in].bi_++;
        // and update the all out HT with the out key
        all_out_ht_[key1_out].bi_++;

        // UDP we just need more than one package in each direction
      } else if (protocol == 17 and p->in_out_packets_ > 0
          and p->out_in_packets_ > 0) {
        // Update the all in HT with the in key..
        all_in_ht_[key1_in].bi_++;
        // and update the all out HT with the out key
        all_out_ht_[key1_out].bi_++;

      } else if (p->in_out_packets_ > 0 and p->out_in_packets_ == 0) {
        // it seems that we only have an outflows..
        all_in_ht_[key1_in].out_++;

      } else if (p->in_out_packets_ == 0 and p->out_in_packets_ > 0) {
        // it seems that we only have an in flow..
        all_out_ht_[key1_out].out_++;
      }
    } else {
      // FIXME(asdaniel): what do we do in case of non udp or tcp flow?!
    }
    p = p->next_;
    biflow_counter++;
  }
  // toc..
  std::time(&end);

  // update the statistics
  stat_current_.set_duration_all_build((uint64_t) std::difftime(end, begin));
  stat_current_.set_all_in(all_in_ht_.size());
  stat_current_.set_all_out(all_out_ht_.size());
  stat_current_.set_biflows_in(biflow_counter);
}

void FanOutFilter::all_shrink(AllHT* all_ht, int bad_min, int ratio_min) {
  // A
  // all hosts hitting less than 'bad_min' number of remote
  // sockets are NOT CLASSIFIED as BAD
  //  --> needs at least 'bad_min' number of 'out' edges

  // B
  // all hosts having of a ration of less than 'ratio_min' (bad/good)
  // sockets are NOT CLASSIFIED as BAD
  //  --> does not help us to make a decision now

  // Based on this information we can already CLASSIFY certain hosts
  // as GOOD --> REDUCE the possible CANDIDATES

  AllHT::iterator iter(all_ht->begin());
  while (iter != all_ht->end()) {
    if (iter->second.out_ >= bad_min) {
      // a candidate keep it
      iter++;
    } else {
      // a GOOD host ... remove it
      all_ht->erase(iter++);
    }
  }
}

void FanOutFilter::selection_build(BiFlow* bf, BiFlow* &good_bf,
                                   BiFlow* &possible_bad_bf) {
  uint64_t good_counter = 0;
  time_t begin, end;

  AllHT::iterator iter;
  AllHT::iterator in_end(all_in_ht_.end());
  AllHT::iterator out_end(all_out_ht_.end());

  std::time(&begin);
  BiFlow::Key1l key1;

  while (bf != NULL) {
    key1.from(bf, BiFlow::Key1l::kIn);
    bool good = true;

    // if we can't find the biflows in the HT yet, mark them as "possible bad"
    // in
    if (all_in_ht_.find(key1) != in_end) {
      selection_in_ht_[key1].add(*bf, BiFlow::Key1l::kIn);
      good = false;
    }
    // out
    key1.from(bf, BiFlow::Key1l::kOut);
    if (all_out_ht_.find(key1) != out_end) {
      selection_out_ht_[key1].add(*bf, BiFlow::Key1l::kOut);
      good = false;
    }
    // save the next element of the current biflow
    BiFlow* next = bf->next_;
    if (good) {
      // we have a good biflow, so add it to the good chain
      good_counter++;
      bf->next_ = good_bf;
      // update the head pointer of the good chain to the current biflow..
      good_bf = bf;
    } else {
      // the biflow is a bad biflow so add it to the bad chain
      bf->next_ = possible_bad_bf;
      possible_bad_bf = bf;
    }
    // the new biflow is the next element
    bf = next;
  }

  std::time(&end);
  // statistics..
  stat_current_.set_duration_selection_build(std::difftime(end, begin));
  stat_current_.set_selection_in(selection_in_ht_.size());
  stat_current_.set_selection_out(selection_out_ht_.size());
  stat_current_.set_biflows_kept(good_counter);
}

void FanOutFilter::selection_shrink(SelectionHT* selection_ht, int bad_min,
                                    int ratio_min) {
  // A
  // all hosts hitting less than 'bad_min' number of remote
  // sockets are NOT CLASSIFIED as BAD

  // B
  // all hosts having of a ration of less than 'ratio_min' (bad/good)
  // sockets are NOT CLASSIFIED as BAD

  SelectionHT::iterator iter(selection_ht->begin());
  while (iter != selection_ht->end()) {
    float bad  = static_cast<float>(iter->second.degree_out_.size());
    float good = static_cast<float>(iter->second.degree_bi_.size());
    // A
    if (bad >= static_cast<float>(bad_min)) {
      // B
      if (static_cast<int>(good) == 0
          or bad/good >= static_cast<float>(ratio_min)) {
        iter++;
        continue;
      }
    }
    // NOT A && B
    // --> remove it
    selection_ht->erase(iter++);
  }
}

void FanOutFilter::split(BiFlow* bf, BiFlow* & good_bf, BiFlow* & bad_bf) {
  uint64_t good_counter = 0;
  SelectionHT::iterator iter_fos;
  SelectionHT::iterator in_end(selection_in_ht_.end());
  SelectionHT::iterator out_end(selection_out_ht_.end());

  time_t begin, end;
  std::time(&begin);

  BiFlow::Key1l key1;
  while (bf != NULL) {
    BiFlow* next = bf->next_;

    // bad - in
    key1.from(bf, BiFlow::Key1l::kIn);
    if (selection_in_ht_.find(key1) != in_end) {
      bf->next_ = bad_bf;
      bad_bf = bf;
      bf = next;
      continue;
    }

    // bad - out
    key1.from(bf, BiFlow::Key1l::kOut);
    if (selection_out_ht_.find(key1) != out_end) {
      bf->next_ = bad_bf;
      bad_bf = bf;
      bf = next;
      continue;
    }

    // good
    good_counter++;
    bf->next_ = good_bf;
    good_bf = bf;

    // next
    bf = next;
  }
  std::time(&end);

  // Update the statistics
  stat_current_.set_biflows_kept(stat_current_.biflows_kept() + good_counter);
  stat_current_.set_duration_split(std::difftime(end, begin));
}
