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
#include <gazsim_msgs/OpcComm.pb.h>
#include <opc/ua/node.h>
#include <opc/ua/protocol/variant.h>

using namespace OpcUa;
using namespace gazsim_msgs;

namespace mps_comm {

class MpsData {
  friend class CommandHandler;

public:
  MpsData(const OpcUa::UaServer *OPCServer, std::vector<std::string> path)
      : base_path_(path) {

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
        // OpcUtils::logNodeInfo(parent, logger_, true, 2);
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
        n_data.AddVariable(namespaceIdx, "Payload1", Variant((uint16_t)0));
    reg_nodes_[DATA_PAYLOAD2] =
        n_data.AddVariable(namespaceIdx, "Payload2", Variant((uint16_t)0));
    // Status Object Vars
    Node n_status = parent.AddObject(namespaceIdx, "Status");
    reg_nodes_[STATUS_BUSY] =
        n_status.AddVariable(namespaceIdx, "Busy", Variant(false));
    reg_nodes_[STATUS_ENABLED] =
        n_status.AddVariable(namespaceIdx, "Enable", Variant(false));
    reg_nodes_[STATUS_ERR] =
        n_status.AddVariable(namespaceIdx, "Error", Variant((uint8_t)0));
    reg_nodes_[STATUS_READY] =
        n_status.AddVariable(namespaceIdx, "Ready", Variant(false));
  }

  ~MpsData(){};

  std::string type() { return base_path_.back(); }
  OpcUa::Node get_node(Register reg) { return reg_nodes_[reg]; }
  std::string get_value_str(Register reg) {
    return reg_nodes_[reg].GetValue().ToString();
  }
  bool set_value(Register reg, std::string val) {

    if (val.empty())
      return false;
    switch (reg_nodes_[reg].GetValue().Type()) {
    case OpcUa::VariantType::UINT16:
      get_node(reg).SetValue(static_cast<uint16_t>(std::stoi(val)));
      break;
    case OpcUa::VariantType::UINT32:
      get_node(reg).SetValue(static_cast<uint32_t>(std::stoi(val)));
      break;
    case OpcUa::VariantType::UINT64:
      get_node(reg).SetValue(static_cast<uint64_t>(std::stoi(val)));
      break;
    case OpcUa::VariantType::INT16:
      get_node(reg).SetValue(static_cast<int16_t>(std::stoi(val)));
      break;
    case OpcUa::VariantType::INT32:
      get_node(reg).SetValue(static_cast<int32_t>(std::stoi(val)));
      break;
    case OpcUa::VariantType::INT64:
      get_node(reg).SetValue(static_cast<int64_t>(std::stoi(val)));
      break;
    case OpcUa::VariantType::FLOAT:
      get_node(reg).SetValue(static_cast<float>(std::stof(val)));
      break;
    case OpcUa::VariantType::DOUBLE:
      get_node(reg).SetValue(static_cast<double>(std::stod(val)));
      break;
    case OpcUa::VariantType::BOOLEAN:
      get_node(reg).SetValue(val == "true" ? true : false);
      break;
    case OpcUa::VariantType::BYTE:
      get_node(reg).SetValue(static_cast<uint8_t>(std::stoi(val)));
      break;
    default:
      get_node(reg).SetValue(val);
      break;
    }

    printf("OPC server: Register %s set to %s '\n",
           get_node(reg).GetBrowseName().Name.c_str(), val.c_str());
    return true;
  }

  std::vector<Register> CommandRegisters = {
      ACTION_ID, DATA_PAYLOAD1, DATA_PAYLOAD2, STATUS_ENABLED, ERROR};

private:
  std::vector<std::string> base_path_;
  std::map<Register, OpcUa::Node> reg_nodes_;
};

} // namespace mps_comm
