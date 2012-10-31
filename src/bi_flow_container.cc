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

#include "bi_flow_container.h"

BiFlowContainer::BiFlowContainer()
    : time_(0),
      biflows_(NULL) {
}

uint64_t BiFlowContainer::time(void) const {
  return time_;
}

BiFlow* BiFlowContainer::biflows(void) const {
  return biflows_;
}

void BiFlowContainer::set_time(uint64_t time) {
  time_ = time;
}

void BiFlowContainer::set_biflows(BiFlow* biflows) {
  biflows_ = biflows;
}
