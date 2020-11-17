/***************************************************************************
 *  mps_command.h - Command handler for OpcUa nodes
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

#include <exception>
#include <opc/ua/node.h>
#include <opc/ua/protocol/variant.h>
#include <opc/ua/subscription.h>

#include "mps_data.h"
#include "opc_utils.h"

using namespace OpcUa;

namespace mps_comm {

class CommandHandler : public SubscriptionHandler {
public:
  CommandHandler(std::shared_ptr<MpsData> mps_data)
      : mps_data_(mps_data), server_(mps_data->server_),
        logger_(mps_data->logger_){};

  ~CommandHandler(){};

  void handle_commands(std::vector<MpsData::Registers> command_registers) {
    command_registers_ = command_registers;
    subscription_ = server_->CreateSubscription(100, *this);
    for (auto &i : command_registers) {
      OpcUa::Node node = mps_data_->get_node(i);
      command_node_names_.push_back(node.GetBrowseName().Name);
      subscription_->SubscribeDataChange(node);
    }
  }

  void DataChange(uint32_t handle, const Node &node, const Variant &val,
                  AttributeId attr) override {

    if (node.GetBrowseName().Name == command_node_names_.front())
      if (!mps_command_.empty())
        dispatch_command();

    mps_command_[node.GetBrowseName().Name] = node.GetValue().ToString();

    if (node.GetBrowseName().Name == command_node_names_.back())
      dispatch_command();
  }

  void dispatch_command() {
    for (auto &i : mps_command_)
      std::cout << i.first << " : " << i.second << std::endl;

    std::cout << "======================" << std::endl;
    mps_command_.clear();
  }

private:
  std::shared_ptr<MpsData> mps_data_;
  std::shared_ptr<OpcUa::UaServer> server_;
  std::shared_ptr<spdlog::logger> logger_;

private:
  std::shared_ptr<Subscription> subscription_;

  // Registers which constitutes an MPS Command
  std::vector<MpsData::Registers> command_registers_;
  std::vector<std::string> command_node_names_;
  std::map<std::string, std::string> mps_command_;

  // std::tuple<unsigned short, unsigned short, unsigned short, int, unsigned
  // char,
  //            unsigned char>
  //     instruction_;
};

} // end namespace mps_comm
