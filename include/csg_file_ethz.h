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
 * @file   csg_file_ethz.h
 * @author Dominik Schatzmann <schadomi@gmail.com>
 * @date   October, 2012
 * @brief  SWITCH specific parameters used by CSG ETH Zurich
 *
 * Defines certain SWTICH specific paramters that can't be published
 * due to privacy concerns.
 */

#ifndef FLOW_BOX_INCLUDE_CSG_FILE_ETHZ_H_
#define FLOW_BOX_INCLUDE_CSG_FILE_ETHZ_H_
/// We store the IP address of the routers in network byte order in
/// return the index of cache (router_addr_). This function returns
/// the index of the cache to use based on the last part of the
/// IP address.
int CSGFile::parser_router_id(int ip_d) {
  // NOTE: We were requested to remove this code segment
  // due to privacy concerns. Please contact
  // schadomi directly to help out. Sorry ...
  throw FlowBoxE("Please update router IP to router object map first!\
 Contact schadomi@gmail.com to receive the mapping",
                 __FILE__, __LINE__);
  return (0);
}

/// Initialize the router cache.
void CSGFile::router_map_init(struct sockaddr * router_addr) {
  ((struct sockaddr_in*) (&router_addr[0]))->sin_family = AF_INET;
  ((struct sockaddr_in*) (&router_addr[0]))->sin_port = 0;
  inet_pton(AF_INET, "127.0.0.1",
            &(((struct sockaddr_in*) (&router_addr[0]))->sin_addr));

  // NOTE: We were requested to remove this code segment
  // due to privacy concerns. Please contact
  // schadomi directly to help out. Sorry ...
  throw FlowBoxE("Please update router IP to router object map first)\
 Contact schadomi@gmail.com to receive the mapping",
                 __FILE__, __LINE__);
}
#endif  // FLOW_BOX_INCLUDE_CSG_FILE_ETHZ_H_
