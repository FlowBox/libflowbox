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
 * @file   prefix_mapping.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 *
 * @date   October, 2012
 * @brief  PrefixMapping implements the prefix search trees.
 *
 * PrefixMapping implements the longest common prefix lookup tree.
 */

#ifndef FLOW_BOX_INCLUDE_PREFIX_MAPPING_H__
#define FLOW_BOX_INCLUDE_PREFIX_MAPPING_H__

// c++ libs
#include <string>
#include <cassert>

// own stuff
#include "prefix.h"
#include "prefix_mapping_node.h"

template<class T>
class PrefixMapping {
 private:
  PrefixMappingNode<T>* head_ipv4_;
  PrefixMappingNode<T>* head_ipv6_;

 public:
  PrefixMapping<T>(void) {
    Prefix p;
    p.from("0.0.0.0/0");
    head_ipv4_ = new PrefixMappingNode<T>(p);

    p.from("0::0/0");
    head_ipv6_ = new PrefixMappingNode<T>(p);
  }

  ~PrefixMapping<T>(void) {
    delete head_ipv4_;
    delete head_ipv6_;
  }

  void clear(void) {
    head_ipv4_->clear();
    head_ipv6_->clear();
  }

  /**
   * insert new node in the prefix tree
   * @param prefix
   * @param value
   */
  void insert(const Prefix& prefix, T& value) {
    // std::cout << "INSERT: STARTED " << prefix.to_s() << std::endl;
    PrefixMappingNode<T>*node, *last;

    // get the correct root node
    int family = prefix.get_family();
    if (family == Prefix::kFamilyIPv4) {
      node = head_ipv4_;
    } else if (family == Prefix::kFamilyIPv6) {
      node = head_ipv6_;
    } else {
      throw FlowBoxE("INSERT: Addr Type is not yet supported", __FILE__,
                     __LINE__);
      node = NULL;
    }

    last = node;

    // walk down
    // int depth = 0;
    // std::cout << "INSERT: WALK DOWN -- START" << std::endl;
    while (node != NULL) {
      // depth += 1;
      // std::cout << "INSERT: WALK DOWN " << depth << std::endl;
      // std::cout << node->to_s();
      // std::cout.flush();

      last = node;
      node = node->get_next(prefix);
      // std::cout << "INSERT: WALK DOWN NEXT: " << (int) node << std::endl;
    }
    // std::cout << "INSERT: WALK DOWN -- STOP" << std::endl;
    // std::cout << "INSERT: Prefix corresponds to this/direct child of this node (" << node << ")" << std::endl;
    // std::cout << "INSERT: ADD (Prefix, Value)" << std::endl;
    last->add(prefix, value);
    // std::cout << "INSERT: FINISHED" << std::endl;
  }

  const T* lookup(const Prefix& prefix) {
    // std::cout << "LOOKUP: STARTED " << prefix.to_s() << std::endl;
    PrefixMappingNode<T>*node_best_match = NULL;
    PrefixMappingNode<T>*node;

    // init
    int family = prefix.get_family();
    if (family == Prefix::kFamilyIPv4) {
      node = head_ipv4_;
    } else if (family == Prefix::kFamilyIPv6) {
      node = head_ipv6_;
    } else {
      std::cout << "LOOKUP: Unknown Addr Type" << std::endl;
      throw FlowBoxE("LOOKUP: Unknown Addr Type", __FILE__, __LINE__);
      node = NULL;
    }

    // walk
    // int depth = 0;
    // std::cout << "LOOKUP: WALK DOWN -- START" << std::endl;
    while (node != NULL) {
      // depth += 1;
      // std::cout << "LOOKUP: WALK DOWN " << depth << std::endl;
      // std::cout << node->to_s() << std::endl;
      // std::cout.flush();

      // provides this node a valid prefix?
      if (node->value_is_set()) {
        node_best_match = node;
      }

      node = node->get_next(prefix);
      // std::cout << "LOOKUP: WALK DOWN NEXT: " << (int) node << endl;
    }
    // std::cout << "LOOKUP: WALK DOWN -- STOP" << endl;
    if (node_best_match != NULL) {
      // std::cout << "LOOKUP: FINISHED -- MATCH" <<  endl;
      return (node_best_match->get_value());
    } else {
      // std::cout << "LOOKUP: FINISHED -- NOMATCH" << endl;
      return (NULL);
    }
  }
};

#endif // FLOW_BOX_INCLUDE_PREFIX_MAPPING_H__
