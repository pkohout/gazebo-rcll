/***************************************************************************
 *  mps_data.h -  OpcUa nodes used to communicate with the mps
 *
 *  Created: Thu 16 Nov 2020 13:29:11 CET 13:29
 *  Copyright  2020 Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#pragma once
#include "opc_utils.h"
#include <exception>
#include <opc/ua/node.h>
#include <opc/ua/protocol/variant.h>

using namespace OpcUa;

namespace mps_comm {

class MpsData {
  friend class CommandHandler;

  // Registers existing in the MPS, to which it is possible to subscribe to
  enum Registers {
    ACTION_ID = 0,
    BARCODE,
    DATA_PAYLOAD1,
    DATA_PAYLOAD2,
    ERROR,
    SLIDECOUNT,
    STATUS_BUSY,
    STATUS_ENABLE,
    STATUS_ERROR,
    STATUS_READY,
    LAST, // must be the last in the list
  };

public:
  MpsData(std::shared_ptr<OpcUa::UaServer> OPCServer,
          std::shared_ptr<spdlog::logger> logger, std::vector<std::string> path)
      : server_(OPCServer), logger_(logger), base_path_(path) {

    // Get parent Object Node descibed by the base path
    Node parent = OPCServer->GetRootNode();
    uint16_t namespaceIdx = parent.GetId().GetNamespaceIndex();
    for (std::string str : path) {
      namespaceIdx = parent.GetId().GetNamespaceIndex();
      try {
        parent = parent.GetChild(str);
      } catch (std::exception &e) {
        // Node does not exist, Create it
        QualifiedName qname = ToQualifiedName(str, namespaceIdx);
        namespaceIdx = qname.NamespaceIndex;
        NodeId nodeid = NumericNodeId(0, namespaceIdx);
        parent = parent.AddObject(nodeid, qname);
        OpcUtils::logNodeInfo(parent, logger_, true, 2);
      }
    }

    parent = parent.AddObject(namespaceIdx, "p");
    reg_nodes_[ACTION_ID] =
        parent.AddVariable(namespaceIdx, "ActionId", Variant((uint16_t)0));
    reg_nodes_[BARCODE] =
        parent.AddVariable(namespaceIdx, "BarCode", Variant((uint32_t)0));
    reg_nodes_[SLIDECOUNT] =
        parent.AddVariable(namespaceIdx, "SlideCnt", Variant((uint16_t)0));
    reg_nodes_[ERROR] =
        parent.AddVariable(namespaceIdx, "Error", Variant((uint8_t)0));
    // Data Object Node
    Node n_data = parent.AddObject(namespaceIdx, "Data");
    reg_nodes_[DATA_PAYLOAD1] =
        n_data.AddVariable(namespaceIdx, "payload1", Variant((uint16_t)0));
    reg_nodes_[DATA_PAYLOAD2] =
        n_data.AddVariable(namespaceIdx, "payload2", Variant((uint16_t)0));
    // Status Object Vars
    Node n_status = parent.AddObject(namespaceIdx, "Status");
    reg_nodes_[STATUS_BUSY] =
        n_status.AddVariable(namespaceIdx, "Busy", Variant(false));
    reg_nodes_[STATUS_ENABLE] =
        n_status.AddVariable(namespaceIdx, "Enable", Variant(false));
    reg_nodes_[STATUS_ERROR] =
        n_status.AddVariable(namespaceIdx, "Error", Variant((uint8_t)0));
    reg_nodes_[STATUS_READY] =
        n_status.AddVariable(namespaceIdx, "Ready", Variant(false));
  }

  ~MpsData(){};

  OpcUa::Node get_node(Registers reg) { return reg_nodes_[reg]; }

  std::vector<Registers> CommandRegisters = {
      ACTION_ID, DATA_PAYLOAD1, DATA_PAYLOAD2, STATUS_ENABLE, ERROR};

private:
  std::shared_ptr<OpcUa::UaServer> server_;
  std::shared_ptr<spdlog::logger> logger_;
  std::vector<std::string> base_path_;

  std::map<Registers, OpcUa::Node> reg_nodes_;
};

} // namespace mps_comm
