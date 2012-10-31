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
 * @file   prefix_mapping_node.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 *
 * @date   October, 2012
 * @brief  PrefixMappingNode is used to build prefix search trees
 *
 * The PrefixMappingNode is used to build search trees that can be used
 * to longest common prefix search.
 *
 */

#ifndef FLOW_BOX_INCLUDE_PREFIX_MAPPING_NODE_H__
#define FLOW_BOX_INCLUDE_PREFIX_MAPPING_NODE_H__

#include <string>
#include <cassert>

#include "prefix.h"

template<class T>
class PrefixMappingNode {
  // ## CLASS VARIABLES ########################################################
 private:
  // CONTENT of this node
  Prefix prefix_;  // assigned prefix
  T value_;        // value of this node (if assigened)
  bool value_set_;  // does this node have a value?

  // Next node information to build the TREE
  PrefixMappingNode* left_;  // next bit == 0
  PrefixMappingNode* right_;  // next bit == 1

  // ## CLASS FUNCTIONS ########################################################
 private:
  // HELPER
  const std::string to_s(void) const {
    std::stringstream tmp;
    tmp << "Prefix_Mapping_Node " << static_cast<int>(this) << std::endl;
    tmp << "Prefix:      " << prefix_.to_s() << std::endl;
    tmp << "Value is set:" << value_set_ << std::endl;
    tmp << "LEFT:        " << static_cast<int>(left_) << std::endl;
    tmp << "RIGHT:       " << static_cast<int>(right_);
    return (std::string(tmp.str()));
  }

  // ## CLASS API ##############################################################
 public:
  PrefixMappingNode(void) {
    value_set_ = false;
    left_ = NULL;
    right_ = NULL;
  }

  explicit PrefixMappingNode(const Prefix & p)
      : prefix_(p) {
    value_set_ = false;
    left_ = NULL;
    right_ = NULL;
  }

  PrefixMappingNode(const Prefix & p, const T& v)
      : prefix_(p),
        value_(v) {
    value_set_ = true;
    left_ = NULL;
    right_ = NULL;
  }

  ~PrefixMappingNode(void) {
    if (left_ != NULL) {
      delete left_;
      left_ = NULL;
    }
    if (right_ != NULL) {
      delete right_;
      right_ = NULL;
    }
  }

//------------------------------------------------------------------------------
// ACCESSORS
//------------------------------------------------------------------------------
  bool value_is_set(void) {
    return (value_set_);
  }

  const T* get_value(void) {
    return (&value_);
  }

//------------------------------------------------------------------------------
// TREE FUNCTIONS
//------------------------------------------------------------------------------
  // get next node
  PrefixMappingNode<T>* get_next(const Prefix& other) {
    // std::cout << "GET NEXT: ";

    // 1. this node has no further leafs
    if (left_ == NULL and right_ == NULL) {
      // std::cout << "GET NEXT:  No leafs found, return NULL" << std::endl;
      // std::cout.flush();
      return (NULL);
    }

    // 2. no bits left
    if (other.get_length() <= prefix_.get_length()) {
      // std::cout << "GET NEXT:  No bit left" << endl;
      // std::cout.flush();
      return (NULL);
    }

    // 3. we have at least one leaf
    // ... but which one would be the right one
    int bit = other.get_bit_at(prefix_.get_length() + 1);
    PrefixMappingNode* next = (bit == 0) ? left_ : right_;

    // 3.a sorry the corresponding leaf is NULL = do not exist.
    if (next == NULL)
      return (NULL);

    // 3.b we have a match but ...
    // is searched prefix include by the next node?
    if (other.includes(next->prefix_)) {
      // Yes, so lets ask the next node for further information
      return (next);
    } else {
      // No, so the searched node would be between this and the next one
      return (NULL);
    }
  }

  void add(const Prefix& other, T& v) {
    // std::out << "ADD: STARTED " << other.to_s() << endl;

    // 1. Is this node the traget ?
    if (prefix_.eql(other)) {
      // Yes, we overwrite the old content
      // std::cout << "ADD: EQUAL =  true --> replace or set value of this node " << endl;
      // std::out.flush();
      value_ = v;
      value_set_ = true;
      return;
    }
    // else  {
    // std::cout << "ADD: EQUAL = false " << endl;
    // }

    // 2. We have to add a new node into the chain: Left or Right?
    int bit = other.get_bit_at(prefix_.get_length() + 1);
    // std::cout << "ADD: BIT at position " << prefix_.get_size() + 1 <<  " is "  << bit << endl;
    if (bit == 0) {
      // 2a. add content into the left side
      // std::cout << "ADD: New LEAF on the LEFT " << endl;
      left_ = add(other, v, left_);
      // std::cout << "ADD: FINISHED" << endl;
      return;
    } else if (bit == 1) {
      // 2a. add content into the right side
      // std::cout << "ADD: New LEAF on the RIGHT " << endl;
      right_ = add(other, v, right_);
      // std::cout << "ADD: FINISHED" << endl;
      return;
    } else {
      // std::cout << "ADD: We don't know if we have to add the node on the right or on the left ?!!" << endl;
      throw FlowBoxE("Application Logic Brocken??", __FILE__, __LINE__);
    }
    // std::cout << "ADD: FINISHED" << endl;
  }

  PrefixMappingNode* add(const Prefix& other, T& v, PrefixMappingNode* root) {
    // std::cout << "ADD NEW NODE: STARTED" << endl;
    if (root == NULL) {
      // std::cout << "ADD NEW NODE: No other node ... just add" << endl;
      root = new PrefixMappingNode(other, v);
      // std::cout << "ADD NEW NODE: FINISHED" << endl;
      return (root);
    }

    if (root->prefix_.includes(other)) {
      // CASE ONE:
      // -->[other]-->[root]
      //           -->[NULL]
      // CASE DUE:
      // -->[other]-->[NULL]
      //           -->[root]
      // std::cout << "ADD NEW NODE: New Node includes the old prefix_: Extend the chain" << endl;
      PrefixMappingNode* other_node = new PrefixMappingNode(other, v);
      other_node->attach(root);
      // std::cout << "ADD NEW NODE: FINISHED" << endl;
      return (other_node);
    }
    // else {
    // std::cout << "ADD NEW NODE: New Node is NOT TOTALLY included the old prefix_" << std::endl;
    // }

    int common_bits = other.common_bits(root->prefix_);
    // std::cout << "ADD NEW NODE: Common Bits: " << common_bits << std::endl;
    if (common_bits <= root->prefix_.get_length()) {
      // CASE TRE
      // -->[new]-->[left]
      //         -->[other]
      // CASE QUATRO
      // -->[new]-->[other]
      //         -->[left]
      // std::cout << "ADD NEW NODE: New Node differs from the old prefix_: Make a brunch" << std::endl;
      // A new node having the prefix_ max common prefix_ length
      PrefixMappingNode* new_node = new PrefixMappingNode(other);
      new_node->prefix_.cut(common_bits);
      new_node->left_ = NULL;
      new_node->right_ = NULL;

      PrefixMappingNode* other_node = new PrefixMappingNode(other, v);
      other_node->left_ = NULL;
      other_node->right_ = NULL;

      new_node->attach(root);
      new_node->attach(other_node);

      assert(   (new_node->left_ == other_node and new_node->right_ == root)
             or (new_node->left_ == root and new_node->right_ == other_node));

      // std::cout << "ADD NEW NODE: BRUNCH:   " <<  new_node->to_s() << std::endl;
      // std::cout << "ADD NEW NODE: NEW NODE: " <<  other_node->to_s() << std::endl;
      // std::cout << "ADD NEW NODE: OLD NODE: " <<  root->to_s() << std::endl;

      // std::cout << "ADD NEW NODE: FINISHED" << std::endl;
      return (new_node);
    }
    // std::cout << "ADD NEW NODE: no case matched !??" << std::endl;
    // std::cout.flush();
    throw FlowBoxE("Application Logic Broken??", __FILE__, __LINE__);
  }

  void attach(PrefixMappingNode* leaf) {
    int bit = leaf->prefix_.get_bit_at(prefix_.get_length() + 1);
    if (bit == 0)
      left_ = leaf;
    else if (bit == 1)
      right_ = leaf;
    else
      throw FlowBoxE("Application Logic Broken??", __FILE__, __LINE__);
  }

  void clear(void) {
    if (left_ != NULL) {
      left_->clear();
      delete left_;
      left_ = NULL;
    }
    if (left_ != NULL) {
      left_->clear();
      delete left_;
      left_ = NULL;
    }
  }
};

#endif // FLOW_BOX_INCLUDE_PREFIX_MAPPING_NODE_H__
