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
 * @file   parser_netflow_9.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 *
 * @date   October, 2012
 * @brief  Extract NetFlow Version 9 data from a Packet.
 *
 * The ParserNetflow9 is a simple NetFlow parser. It analyzes the NetFlow
 * information contained by the Packet object and stores the information
 * in our internal Flow data structure.
 *
 */

#include "parser_netflow_9.h"
//------------------------------------------------------------------------------
//--- TemplateToFlow  ----------------------------------------------------------
//------------------------------------------------------------------------------
ParserNetflow9::TemplateToFlow::TemplateToFlow() {
  reset();
}

/**
 * Sets all template and data values to the initial values
 */
void ParserNetflow9::TemplateToFlow::reset(void) {
  template_id_ = 0;
  raw_length_ = 0;
  reset_data();
}

/**
 * Sets the data to the initial values
 */
void ParserNetflow9::TemplateToFlow::reset_data(void) {
  length_ = 0;
  valid_ = false;
  addr_length_ = 0;
  addr_src_o_ = -1;
  addr_dst_o_ = -1;
  protocol_o_ = -1;
  port_src_o_ = -1;
  port_dst_o_ = -1;
  first_o_ = -1;
  last_o_ = -1;
  addr_next_o_ = -1;
  if_in_o_ = -1;
  if_in_l_ = 0;
  if_out_o_ = -1;
  if_out_l_ = 0;
  packets_o_ = -1;
  packets_l_ = 0;
  bytes_o_ = -1;
  bytes_l_ = 0;
}

/**
 * Checks if the values found for the template make sense in the way that the
 * configurations worked.
 * @return true if no problem found, false otherwise
 */
bool ParserNetflow9::TemplateToFlow::validate(void) {
  valid_ = true;
  if (!(addr_length_ == Flow::kAddressLengthIPv4
      || addr_length_ == Flow::kAddressLengthIPv6)) {
    valid_ = false;
    std::stringstream err_msg;
    err_msg << "ParserNetflow9::TemplateToFlow::validate: "
            << "ADDR LENGTH FAILED";
    throw FlowBoxE(err_msg.str(), __FILE__, __LINE__);
  }
  // check offsets
  if (addr_src_o_ == kOffsetInvalid || addr_dst_o_ == kOffsetInvalid
      || protocol_o_ == kOffsetInvalid || first_o_ == kOffsetInvalid
      || last_o_ == kOffsetInvalid || addr_next_o_ == kOffsetInvalid
      || if_in_o_ == kOffsetInvalid || if_out_o_ == kOffsetInvalid
      || packets_o_ == kOffsetInvalid || bytes_o_ == kOffsetInvalid) {
    valid_ = false;
    std::stringstream err_msg;
    err_msg << "ParserNetflow9::TemplateToFlow::validate: "
            << "At least one data field is missing";
    throw FlowBoxE(err_msg.str(), __FILE__, __LINE__);
  }
  // check field size
  if (!((if_in_l_ == kFieldLength16 or if_in_l_ == kFieldLength32)
      and (if_out_l_ == kFieldLength16 or if_out_l_ == kFieldLength32))) {
    valid_ = false;
    std::stringstream err_msg;
    err_msg << "ParserNetflow9::TemplateToFlow::validate: "
            << "At least one field length is not recognized";
    throw FlowBoxE(err_msg.str(), __FILE__, __LINE__);

    // FIXME (asdaniel/schadomi): Correct handling?! YES => delete cout stuff!
    // std::cout << "ParserNetflow9::TemplateToFlow::validate: ";
    // std::cout << "At least one  interface field length is not recognized!";
    // std::cout << std::endl;
    // std::cout << "if_in_l_ " << if_in_l_ << std::endl;
    // std::cout << "if_out_l_ " << if_out_l_ << std::endl;
    // std::cout.flush();
  }

  if (packets_l_ != kFieldLength32 || bytes_l_ != kFieldLength32) {
    valid_ = false;
    std::stringstream err_msg;
    err_msg << "ParserNetflow9::TemplateToFlow::validate: "
            << "At least one field length is not recognized";
    throw FlowBoxE(err_msg.str(), __FILE__, __LINE__);

    // FIXME (asdaniel/schadomi): Correct handling?! YES => delete cout stuff!
    // std::cout << "ParserNetflow9::TemplateToFlow::validate: ";
    // std::cout << "At least one content field length is not recognized!";
    // std::cout << std::endl;
    // std::cout << "packets_l_ " << packets_l_ << std::endl;
    // std::cout << "bytes_l_ " << bytes_l_ << std::endl;
    // std::cout.flush();
  }
  return (valid_);
}

std::string ParserNetflow9::TemplateToFlow::to_s(void) const {
  std::stringstream tmp;

  tmp << "template_id_ => " << template_id_ << std::endl;
  tmp << "length_ => " << length_ << std::endl;
  tmp << "valid_ => " << valid_ << std::endl;
  tmp << "addr_length_ => " << addr_length_ << std::endl;
  tmp << "addr_src_ => " << addr_src_o_ << std::endl;
  tmp << "addr_dst_ => " << addr_dst_o_ << std::endl;
  tmp << "protocol_o_ => " << protocol_o_ << std::endl;
  tmp << "port_src_o_ => " << port_src_o_ << std::endl;
  tmp << "port_dst_o_ => " << port_dst_o_ << std::endl;
  tmp << "first_o_ => " << first_o_ << std::endl;
  tmp << "last_o_ => " << last_o_ << std::endl;
  tmp << "addr_next_o_ => " << addr_next_o_ << std::endl;
  tmp << "if_in_o_ => " << if_in_o_ << " ( " << if_in_l_ << ")" << std::endl;
  tmp << "if_out_o_ => " << if_out_o_ << " (" << if_out_l_ << ")" << std::endl;
  tmp << "packets_o_ => " << packets_o_ << " (" << packets_l_ << ")"
      << std::endl;
  tmp << "bytes_o_ => " << bytes_o_ << " (" << bytes_l_ << ")" << std::endl;

  return (std::string(tmp.str()));
}

//------------------------------------------------------------------------------
//--- ParserNetflow9  ----------------------------------------------------------
//------------------------------------------------------------------------------
ParserNetflow9::ParserNetflow9(int export_device_id)
    : export_device_id_(export_device_id),
      loss_packet_sequence_last_(0),
      loss_packet_sequence_lost_(0) {
  statistics_reset();
  assert(export_device_id_ != 0);
}

ParserNetflow9::~ParserNetflow9() {
  for (std::map<int, TemplateToFlow*>::iterator iter = template_map_.begin();
      iter != template_map_.end(); iter++) {
    delete iter->second;
  }
  template_map_.clear();
}

void ParserNetflow9::parse_template(const char* flowset_p,
                                    const int flowset_length) {
#ifdef PARSER_NETFLOW_9_DEBUG
  std::cout << "---- Template Flow Set: " << std::endl;
  std::cout << "Length:     " << flowset_length << std::endl;
#endif

  // beginning of the template -> points to template id
  const char* template_p = flowset_p + 4;
  int rest = flowset_length - 4;  // rest to parse
  while (rest > 0) {
    // PARSE TEMPLATE HEADER
    // get template ID
    int template_id = htons(* (reinterpret_cast<const uint16_t*>(template_p)));
    // get the template field count
    int template_fields =
        htons(* (reinterpret_cast<const uint16_t*>(template_p+2)));

    // PARSE TEMPLATE DATA
    // get the map
    TemplateToFlow* current = get_template(template_id);
    if (current == NULL) {
      // first time we see this template
#ifdef PARSER_NETFLOW_9_DEBUG
      std::cout << "EID: "<< export_device_id_ << " WARNING:  -- New Template ";
      std::cout << template_id << std::endl;
#endif
      current = create_template(template_id);
      current->raw_length_ = template_fields * 4 + 4;
      // check if smaller than maximum size
      if (current->raw_length_ >= TemplateToFlow::kRawLengthMax) {
        std::stringstream error_msg;
        error_msg << "EID: " << export_device_id_
                  << " ERROR:  -- increase kRawLengthMax to at least "
                  << template_fields * 4 + 4 << std::endl;
        throw FlowBoxE((error_msg.str()), __FILE__, __LINE__);
      }

      // get the template in raw format
      memcpy(current->raw_, template_p, current->raw_length_);
    } else {
      // template exists, we check if we need to update it
      if ((current->raw_length_ == template_fields * 4 + 4)
          && memcmp(template_p, current->raw_, current->raw_length_) == 0) {
        // UNCHANGED: nothing has to be done
#ifdef PARSER_NETFLOW_9_DEBUG
        std::cout << "EID: "<< export_device_id_ << " Unchanged Template ";
        std::cout << std::endl;
        std::cout.flush();
#endif
        // go to next template
        template_p += (4 + (template_fields * 4));
        rest -= (4 + template_fields * 4);
        continue;
      } else {
        // CHANGED: reset data part and parse it again
#ifdef PARSER_NETFLOW_9_DEBUG
        std::cout << "EID: "<< export_device_id_;
        std::cout << " Existing Template is changing "<< std::endl;
        std::cout.flush();
#endif
        current->reset_data();  // initialize all data to initial values
      }
    }  // get or create template

    // PARSE THE TEMPLATE
    template_p = template_p + 4;  // jump to the first field type
    int offset = 0;

    // do this loop for every template field -> determine type
    for (int i = 0; i < template_fields; i++) {
      // the type definition
      int field_type =
          htons(* (reinterpret_cast<const uint16_t*>(template_p + 4*i)));
      // the length
      int field_length =
          htons(* (reinterpret_cast<const uint16_t*>(template_p + 4*i + 2)));
      // take only what we need ... -> configures the template but takes
      // only what is needed
      switch (field_type) {
        // address related fields
        case kFieldIPv4SrcAddr:
          current->addr_length_ = Flow::kAddressLengthIPv4;
          current->addr_src_o_ = offset;
          if (field_length != 4)
            throw FlowBoxE("Unknown IPV4 IF IN field length, check rfc3954.txt",
                           __FILE__, __LINE__);
          break;
        case kFieldIPv6SrcAddr:
          current->addr_length_ = Flow::kAddressLengthIPv6;
          current->addr_src_o_ = offset;
          if (field_length != 16)
            throw FlowBoxE("Unknown IPV6 IF IN field length, check rfc3954.txt",
                           __FILE__, __LINE__);
          break;
        case kFieldIPv4DstAddr:
          current->addr_dst_o_ = offset;
          if (field_length != 4)
            throw FlowBoxE(
                "Unknown IPV4 IF OUT field length, check rfc3954.txt", __FILE__,
                __LINE__);
          break;
        case kFieldIPv6DstAddr:
          current->addr_dst_o_ = offset;
          if (field_length != 16)
            throw FlowBoxE(
                "Unknown IPV6 IF OUT field length, check rfc3954.txt", __FILE__,
                __LINE__);
          break;

          // protocol related
        case kFieldProtocol:
          current->protocol_o_ = offset;
          if (field_length != 1)
            throw FlowBoxE("Unknown PROTOCOL field length, check rfc3954.txt",
                           __FILE__, __LINE__);
          break;
        case kFieldL4SrcPort:
          current->port_src_o_ = offset;
          if (field_length != 2)
            throw FlowBoxE("Unknown PORT SRC field length, check rfc3954.txt",
                           __FILE__, __LINE__);
          break;
        case kFieldL4DstPort:
          current->port_dst_o_ = offset;
          if (field_length != 2)
            throw FlowBoxE("Unknown PORT DST field length, check rfc3954.txt",
                           __FILE__, __LINE__);
          break;

          // timing
        case kFieldFirstSwitched:
          current->first_o_ = offset;
          if (field_length != 4)
            throw FlowBoxE("Unknown FIRST field length, check rfc3954.txt",
                           __FILE__, __LINE__);
          break;
        case kFieldLastSwitched:
          current->last_o_ = offset;
          if (field_length != 4)
            throw FlowBoxE("Unknown LAST field length, check rfc3954.txt",
                           __FILE__, __LINE__);
          break;
          // router
        case kFieldIPv4NextHop:
          current->addr_next_o_ = offset;
          if (field_length != 4)
            throw FlowBoxE("Unknown NEXT IPV4 field length, check rfc3954.txt",
                           __FILE__, __LINE__);
          break;
        case kFieldIPv6NextHop:
          current->addr_next_o_ = offset;
          if (field_length != 16)
            throw FlowBoxE("Unknown NEXT IPV6 field length, check rfc3954.txt",
                           __FILE__, __LINE__);
          break;
        case kFieldInputSNMP:
#ifdef PARSER_NETFLOW_9_DEBUG
          std::cout << "IF IN  @ " << offset << " L:"
                    << field_length << " idx " << i << std::endl;
#endif
          current->if_in_o_ = offset;
          current->if_in_l_ = field_length;
          break;
        case kFieldOutputSNMP:
#ifdef PARSER_NETFLOW_9_DEBUG
          std::cout << "IF OUT  @ " << offset << " L:" << field_length
                    << " idx " << i << std::endl;
#endif
          current->if_out_o_ = offset;
          current->if_out_l_ = field_length;
          break;

          // content
        case kFieldInBytes:
#ifdef PARSER_NETFLOW_9_DEBUG
          std::cout << "BYTES  @ " << offset << " L:" << field_length
                    << " idx " << i << std::endl;
#endif
          current->bytes_o_ = offset;
          current->bytes_l_ = field_length;
          break;
        case kFieldInPackets:
#ifdef PARSER_NETFLOW_9_DEBUG
          std::cout << "PACKETS  @ " << offset << " L:" << field_length
                    << " idx " << i << std::endl;
#endif
          current->packets_o_ = offset;
          current->packets_l_ = field_length;
          break;
      }  // switch
      // shift the offset by the length of the field
      offset += field_length;
    }  // for
    // the size of the this template is the actual offset
    current->length_ = offset;
    // check if the values are correct
    current->validate();

#ifdef PARSER_NETFLOW_9_DEBUG
    std::cout << "ADD/UPDATE TEMPLATE "<< export_device_id_ << std::endl;
    std::cout << "exporter_id => "<< export_device_id_ << std::endl;
    std::cout << current->to_s() << std::endl;
    std::cout.flush();
#endif

    // move pointer to next template
    template_p = template_p + (template_fields * 4);
    // deduct the already parsed from the rest to parse
    rest = rest - (4 + template_fields * 4);
  };
}

ParserNetflow9::TemplateToFlow* ParserNetflow9::get_template(
    const int template_id) {
  std::map<int, TemplateToFlow*>::iterator iter;
  iter = template_map_.find(template_id);
  if (iter != template_map_.end())
    return (iter->second);
  else
    return (NULL);
}

ParserNetflow9::TemplateToFlow* ParserNetflow9::create_template(
    const int template_id) {
  TemplateToFlow* current = new TemplateToFlow();
  current->reset();
  current->template_id_ = template_id;
  template_map_[template_id] = current;
  return (current);
}

//------------------------------------------------------------------------------
void ParserNetflow9::parse_data(
    const char* flowset_p, const int flowset_length,
    const uint64_t export_time_s, const uint64_t sys_up_r,
    const ParserNetflow9::TemplateToFlow* template_p,
    FlowContainer* flow_container) {

#ifdef PARSER_NETFLOW_9_DEBUG
  std::cout << "ParserNetflow9::parse_data" << std::endl;
  std::cout << "Length: " << flowset_length << std::endl;
#endif

  // check: template is defined
  if (template_p == NULL) {
#ifdef PARSER_NETFLOW_9_DEBUG
    std::cout << "check: template is defined -- failed" << std::endl;
    std::cout.flush();
#endif
    stat_packets_no_template_++;
    return;  // skip processing;
  }
  // check: template is valid
  if (template_p->valid_ == false) {
#ifdef PARSER_NETFLOW_9_DEBUG
    std::cout << "check: template is valid -- failed" << std::endl;
    std::cout.flush();
#endif
    stat_packets_no_valid_template_++;
    return;  // skip processing;
  };

  // try to parse flow set
  int record_count = 0;
  // pointer that points to the record start
  const char* records_p = flowset_p + 4;
  // the rest bytes to parse
  int rest = flowset_length - 4;
  FlowContainer::iterator flow(flow_container->end_used());
  FlowContainer::iterator end(flow_container->end());

  while (rest >= template_p->length_) {
    // TODO (schadomi): CHECK BYTE ALIGNMENT
    // are all uint64_t on %8 == 0 addresses?
    // are all uint32_t on %4 == 0 addresses?
    // are all uint16_t on %2 == 0 addresses?

    // check: do we have engough empty flows?
    if (flow == end) {
      throw FlowBoxE("No space left, but why? we checked it!!", __FILE__,
                     __LINE__);
    }
    // addresses
    if (template_p->addr_length_ == Flow::kAddressLengthIPv4) {
      flow->addr_length_ = Flow::kAddressLengthIPv4;
      Flow::addr_import_ipv4(records_p + template_p->addr_src_o_,
                             flow->addr_src_);
      Flow::addr_import_ipv4(records_p + template_p->addr_dst_o_,
                             flow->addr_dst_);
      Flow::addr_import_ipv4(records_p + template_p->addr_next_o_,
                             flow->addr_next_);
    } else if (template_p->addr_length_ == Flow::kAddressLengthIPv6) {
      flow->addr_length_ = Flow::kAddressLengthIPv6;
      Flow::addr_import_ipv6(records_p + template_p->addr_src_o_,
                             flow->addr_src_);
      Flow::addr_import_ipv6(records_p + template_p->addr_dst_o_,
                             flow->addr_dst_);
      Flow::addr_import_ipv6(records_p + template_p->addr_next_o_,
                             flow->addr_next_);
    } else {
      std::stringstream err_msg;
      err_msg << "Netflow_V9_Parser::parse_data: Unknown Address Format: "
              << static_cast<int>(flow->addr_length_) << std::endl;
      throw FlowBoxE(err_msg.str(), __FILE__, __LINE__);
    }

    // get protocol specific information
    flow->protocol_ = *(reinterpret_cast<const uint8_t *>(records_p + template_p->protocol_o_));
    // get the protocol version
    if (template_p->port_src_o_ != -1)
      // get the port src
      flow->port_src_ =
          ntohs(*(reinterpret_cast<const uint16_t *>(records_p+template_p->port_src_o_)));
    else
      flow->port_src_ = 0;

    if (template_p->port_dst_o_ != -1)
      // get the port dst
      flow->port_dst_ =
          ntohs(*(reinterpret_cast<const uint16_t *>(records_p+template_p->port_dst_o_)));
    else
      flow->port_dst_ = 0;

    if (!(flow->protocol_ == 6 or flow->protocol_ == 17)) {
      flow->port_src_ = 0;
      flow->port_dst_ = 0;
    }

    //--------------------------------------------------------------------------
    // TIMING ------------------------------------------------------------------
    //--------------------------------------------------------------------------

    // relative timing
    // FIXME(schadomi): long long is not standard strict c++ int64..
    // fix it!
    long long first = ntohl(*(reinterpret_cast<const uint32_t *>(records_p+template_p->first_o_)));
    long long last = ntohl(*(reinterpret_cast<const uint32_t *>(records_p+template_p->last_o_)));

    flow->export_s_ = export_time_s;  // export_time_s unix seconds of now
    flow->sys_up_r_ = sys_up_r;  // sys_up_r_ miliseconds since router reboot
    flow->sys_start_r_ = first;
    flow->sys_stop_r_ = last;

    // absolute timing
    // FIXME(schadomi): long long is not standard strict c++ int64..
    // fix it!
    uint32_t start, end;
    long long start2, end2;
    start = sys_up_r - first;
    end = sys_up_r - last;

    start2 = start;  // get more than 32 bit and negative range
    end2 = end;

    // Wrap-around: Do corrections
    // 2**32
    if (start2 >= 0x80000000LL) {
      start2 = start2 - 0x100000000LL;
    }
    if (end2 >= 0x80000000LL) {
      end2 = end2 - 0x100000000LL;
    }

    // Plausibility checks: We check whether the sysup time
    // of the exporter is newer (works also in case of
    // any wrap-arounds if no data gap larger than
    // 2^31 ms exists!) than the flow start/stop sysup
    // times. Including an Error constant to account for
    // possible delays when writing the header sysuptime and
    // the flow sysuptimes (this can happen but is rarely seen!)
    if ((int64_t) start2 + kSysupTimestampMaxDelayInMs <= 0) {
      std::stringstream err_msg;
      err_msg
          << "Netflow_V9_Parser::parse_data: sysuptime of flow start"
          << std::endl
          << " is newer than the sysuptime in the packet header by more than "
          << kSysupTimestampMaxDelayInMs << " ms " << std::endl
          << " (Note: this error is NOT caused by a wrap-around!)"
          << flow->to_s() << std::endl << "exporter_id: " << export_device_id_
          << std::endl << "template: " << std::endl << template_p->to_s()
          << std::endl;
      throw FlowBoxE(err_msg.str(), __FILE__, __LINE__);
    }
    if ((int64_t) end2 + kSysupTimestampMaxDelayInMs <= 0) {
      std::stringstream err_msg;
      err_msg
          << "Netflow_V9_Parser::parse_data: sysuptime of flow stop"
          << std::endl
          << " is newer than the sysuptime in the packet header by more than "
          << kSysupTimestampMaxDelayInMs << " ms " << std::endl
          << " (Note: this error is NOT caused by a wrap-around!)"
          << flow->to_s() << std::endl << "exporter_id: " << export_device_id_
          << std::endl << "template: " << std::endl << template_p->to_s()
          << std::endl;
      throw FlowBoxE(err_msg.str(), __FILE__, __LINE__);
    }
    flow->start_s_ = export_time_s - start2 / 1000;
    flow->stop_s_ = export_time_s - end2 / 1000;

    // 2**22 bug-fix
    if (flow->stop_s_ < flow->start_s_) {
      if ((flow->start_s_ - flow->stop_s_) < 4200
          and (flow->start_s_ - flow->stop_s_) > 100)
        flow->stop_s_ += 4195;
    }

    // router information
    flow->export_device_id_ = export_device_id_;
    // get the in interface
    if (template_p->if_in_l_ == TemplateToFlow::kFieldLength16)
      flow->if_in_ = ntohs(*(reinterpret_cast<const uint16_t *>(records_p+template_p->if_in_o_)));
    else if (template_p->if_in_l_ == TemplateToFlow::kFieldLength32)
      flow->if_in_ = ntohl(*(reinterpret_cast<const uint32_t *>(records_p+template_p->if_in_o_)));
    else
      throw FlowBoxE("Unknown IF IN field length, check rfc3954.txt", __FILE__,
                     __LINE__);
    // get the out interface
    if (template_p->if_out_l_ == TemplateToFlow::kFieldLength16)
      flow->if_out_ = ntohs(*(reinterpret_cast<const uint16_t *>(records_p+template_p->if_out_o_)));
    else if (template_p->if_out_l_ == TemplateToFlow::kFieldLength32)
      flow->if_out_ = ntohl(*(reinterpret_cast<const uint32_t *>(records_p+template_p->if_out_o_)));
    else
      throw FlowBoxE("Unknown IF OUT field length, check rfc3954.txt", __FILE__,
                     __LINE__);

    // network information
    flow->direction_ = Flow::kDirectionUnknown;
    flow->border_ = Flow::kBorderUnknown;

    // parse content information
    // packet amount
    flow->packets_ = ntohl(*(reinterpret_cast<const uint32_t *>(records_p+template_p->packets_o_)));
    // packet bytes
    flow->bytes_ = ntohl(*(reinterpret_cast<const uint32_t *>(records_p+template_p->bytes_o_)));
    // mark it as valid
    flow->valid_ = true;

    // prepare pointers for next iteration
    flow++;
    record_count++;
    rest -= template_p->length_;  // deduct already parsed bytes
    records_p += template_p->length_;  // point to next record
  }

  if (rest > 3) {
    std::stringstream err_msg;
    err_msg << "Netflow_V9_Parser::parse_data: To many left over bytes: "
            << static_cast<int>(rest) << std::endl;
    throw FlowBoxE(err_msg.str(), __FILE__, __LINE__);
    return;
  }
  stat_records_ += record_count;
  flow_container->update_used_by(record_count);
#ifdef PARSER_NETFLOW_9_DEBUG
  std::cout << "Sucessfuly parsed " << record_count << " records" << std::endl;
  std::cout.flush();
#endif
  return;
}

void ParserNetflow9::parse(const Packet& packet,
                           FlowContainer* flow_container) {
  stat_packets_++;

  // HEADER --------------------------------------------------------------------
  // kHeaderVersion: Already checked, its Netflow 9
  // HeaderCount: already checked, container has enough space
  // kHeaderSystemUp: milliseconds since router was booted
  uint32_t system_up_ms =
      ntohl(* (reinterpret_cast<const uint32_t*>(packet.buffer_ + kHeaderSystemUp)));
  // kHeaderUnixUp: Seconds since 0000 Coordinated Universal Time (UTC) 1970
  uint32_t unix_sec = ntohl(* (reinterpret_cast<const uint32_t*>(packet.buffer_ + kHeaderUnixUp)));
  // kHeaderPackageSequence: Packet number
  uint32_t sequence_nr =
      ntohl(*(reinterpret_cast<const uint32_t*>(packet.buffer_ + kHeaderPackageSequence)));
  // kHeaderSourceId: This number is already parsed and stored in source_id_;

  // TODO(schadomi): Ignore UDP Reordering!
  // LOSS Estimation: Try to estimate how many packets are lost in the network
  // By Guido Hungerbuehler
  // TODO(schadomi / guh85): Jump
  if (loss_packet_sequence_last_ + 1 == sequence_nr) {
    // Normal case: no loss or init
    loss_packet_sequence_last_ = sequence_nr;
  } else if (sequence_nr == 0 or loss_packet_sequence_last_ == 0) {
    // uint32_t: frist flow, sequence_nr overflow, or router rebooted
    loss_packet_sequence_last_ = sequence_nr;
  } else if (loss_packet_sequence_last_ + 1 < sequence_nr) {
    // Loss: gap
    int gap = sequence_nr - loss_packet_sequence_last_ - 1;
    loss_packet_sequence_lost_ += gap;
    loss_packet_sequence_last_ = sequence_nr;
  } else if (loss_packet_sequence_last_ > sequence_nr) {
    // Missing packet found: reduce gap
    loss_packet_sequence_lost_--;
  } else if (loss_packet_sequence_last_ == sequence_nr) {
    // This is a very rare case: sequence_nr overflow
    // happening during router reboot
    loss_packet_sequence_last_ = sequence_nr;
  } else {
    // All cases are already handled?? WTF
    throw FlowBoxE("Application Logic broken?!?", __FILE__, __LINE__);
  }

  // PARSE FLOW SETS
  const char* flowset_p = packet.buffer_ + kHeaderLength;
  int rest = packet.buffer_length_ - kHeaderLength;
  while (rest > 4) {
    // ignore padding on 32 bit boundary

    // CURRENT FLOW SET HEADER
    // get the flow set ID
    int flowset_id = htons(*(reinterpret_cast<const uint16_t*>(flowset_p)));
    // get the length (total length of flowsets)
    int flowset_length = htons(*(reinterpret_cast<const uint16_t*>(flowset_p+2)));

    // debug
#ifdef PARSER_NETFLOW_9_DEBUG
    std::cout << "--- Flow Set: " << std::endl;
    std::cout << "Id:         " << flowset_id << std::endl;
    std::cout << "Length:     " << flowset_length << std::endl;
    std::cout << "Rest After: " << rest - flowset_length << " B" << std::endl;
#endif

    // Sanity check if the packet length
    // corresponds to the bytes left to parse
    if (rest < flowset_length) {
      stat_packets_error_++;
#ifdef PARSER_NETFLOW_9_DEBUG
      std::cout << "EID: "<< export_device_id_ << " ERROR FLOW SET LENGTH: "
      << rest << " Next Length: " << flowset_length << std::endl;
#endif
      return;
    }

    if (flowset_id == kTemplateFlowsetId) {  // TEMPLATE
      parse_template(flowset_p, flowset_length);
    } else if (flowset_id == kOptionsFlowsetId) {  // OPTIONS TEMPLATE
      throw FlowBoxE("Options Templates are not yet supported", __FILE__,
                     __LINE__);
    } else if (flowset_id > kTemplateflowsetIdMax) {  // DATA FLOWSET
      parse_data(flowset_p, flowset_length, unix_sec, system_up_ms,
                 get_template(flowset_id), flow_container);
    } else {
      throw FlowBoxE("We don't understand this template id", __FILE__,
                     __LINE__);
    }
    // deduct the parsed part from the rest that has to be parsed
    rest -= flowset_length;
    // move the pointer to the next flow set record
    flowset_p += flowset_length;
  }
}

void ParserNetflow9::reset(void) {
  loss_packet_sequence_last_ = 0;

  // flush tmplates
  std::map<int, TemplateToFlow*>::iterator iter(template_map_.begin());
  while (iter != template_map_.end()) {
    delete iter->second;
    template_map_.erase(iter++);
  }

  statistics_reset();
}

// STATISTICS ------------------------------------------------------------------
void ParserNetflow9::statistics_reset() {
  stat_packets_ = 0;
  stat_packets_error_ = 0;
  stat_packets_no_template_ = 0;
  stat_packets_no_valid_template_ = 0;
  stat_records_ = 0;
  loss_packet_sequence_lost_ = 0;
}

std::map<std::string, uint64_t> ParserNetflow9::statatistics_get(void) {
  std::map<std::string, uint64_t> exporter_stat;
  exporter_stat.insert(
      std::pair<std::string, uint64_t>("export_id", export_device_id_));
  exporter_stat.insert(
      std::pair<std::string, uint64_t>("packets", stat_packets_));
  exporter_stat.insert(
      std::pair<std::string, uint64_t>("packets_error", stat_packets_error_));
  exporter_stat.insert(
      std::pair<std::string, uint64_t>("packets_no_template",
                                       stat_packets_no_template_));
  exporter_stat.insert(
      std::pair<std::string, uint64_t>("packets_no_valid_template",
                                       stat_packets_no_valid_template_));
  exporter_stat.insert(
      std::pair<std::string, uint64_t>("flows", stat_records_));
  exporter_stat.insert(
      std::pair<std::string, uint64_t>("flows_lost", stat_flow_lost()));
  exporter_stat.insert(
      std::pair<std::string, uint64_t>("templates", stat_template_count()));

  return (std::map<std::string, uint64_t>(exporter_stat));
}

//------------------------------------------------------------------------------
// The class helper functions
//------------------------------------------------------------------------------
int ParserNetflow9::stat_template_count() {
  return (template_map_.size());
}

uint64_t ParserNetflow9::stat_flow_lost() {
  if (loss_packet_sequence_lost_ >= 0)
    return (loss_packet_sequence_lost_);
  else
    return (0);
}

int64_t ParserNetflow9::lost() {
  return (loss_packet_sequence_lost_);
}

//------------------------------------------------------------------------------
// API (STATIC)
//------------------------------------------------------------------------------
const uint16_t ParserNetflow9::get_version(const Packet& packet) {
  return (packet.buffer_[1]);
}

const uint32_t ParserNetflow9::get_engine_id(const Packet& packet) {
  return (ntohl( *(reinterpret_cast<const uint32_t*>(packet.buffer_ + kHeaderSourceId))));
}

const uint32_t ParserNetflow9::get_export_time_s(const Packet& packet) {
  return (ntohl( *(reinterpret_cast<const uint32_t*>(packet.buffer_ + kHeaderUnixUp))));
}

const uint16_t ParserNetflow9::get_estimated_flow_count(const Packet& packet) {
  return (ntohs( *(reinterpret_cast<const uint16_t*>(packet.buffer_ + kHeaderCount))));
}

