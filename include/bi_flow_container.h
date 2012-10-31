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
 * @file   bi_flow_container.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @author Daniel Aschwanden  <nimdanitro@gmail.com>
 * @date   October, 2012
 * @brief  A container containing biflows with timestamps
 *
 * This class implements a container to forward BiFlows from units to units.
 * Beside the actual BiFlows data meta data such as timestamps can be attached
 * to the container.
 *
 */

#ifndef FLOW_BOX_INCLUDE_BI_FLOW_CONTAINER_H_
#define FLOW_BOX_INCLUDE_BI_FLOW_CONTAINER_H_

#include <cassert>
#include <string>

#include "common.h"
#include "bi_flow.h"


class BiFlowContainer {
 private:
  // the time stamp of the container
  uint64_t time_;
  // the BiFlowPointer
  BiFlow* biflows_;
  //----------------------------------------------------------------------------
 public:
  // getters of time and biflow pointer
  uint64_t time() const;
  BiFlow* biflows() const;

  // setters of time and biflow pointer
  void set_time(uint64_t time);
  void set_biflows(BiFlow* biflows);

 private:
  BiFlowContainer();
  friend class BiFlowContainerPool;
  DISALLOW_COPY_AND_ASSIGN(BiFlowContainer);
};
#endif  // FLOW_BOX_INCLUDE_BI_FLOW_CONTAINER_H_
