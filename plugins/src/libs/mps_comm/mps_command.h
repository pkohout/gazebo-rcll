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

typedef std::map<std::string, std::string> Instruction;

class CommandHandler : public SubscriptionHandler {
public:
  CommandHandler(OpcUa::UaServer *server, std::shared_ptr<MpsData> mps_data,
                 std::shared_ptr<spdlog::logger> logger)
      : server_(server), mps_data_(mps_data), logger_(logger){};

  ~CommandHandler(){};

  void
  handle_instructions(std::vector<MpsData::Registers> instruction_registers) {
    instruction_registers_ = instruction_registers;
    subscription_ = server_->CreateSubscription(100, *this);
    for (auto &i : instruction_registers) {
      OpcUa::Node node = mps_data_->get_node(i);
      instruction_nodes_name_.push_back(node.GetBrowseName().Name);
      subscription_->SubscribeDataChange(node);
    }
  }

  void
  register_instruction_callback(Instruction instruction,
                                std::function<void(Instruction)> callback) {
    callbacks_[instruction] = callback;
  }

  void DataChange(uint32_t handle, const Node &node, const Variant &val,
                  AttributeId attr) override {

    if (node.GetBrowseName().Name == instruction_nodes_name_.front())
      if (!mps_instruction_.empty())
        dispatch_instruction();

    mps_instruction_[node.GetBrowseName().Name] = node.GetValue().ToString();

    if (node.GetBrowseName().Name == instruction_nodes_name_.back())
      dispatch_instruction();
  }

  void dispatch_instruction() {
    for (auto &i : mps_instruction_)
      std::cout << i.first << " : " << i.second << std::endl;

    for (auto &i : callbacks_) {
      bool call = true;
      for (auto &registered_instruction : i.first)
        if (mps_instruction_[registered_instruction.first] !=
            registered_instruction.second) {
          call = false;
          continue;
        }
      if (call)
        i.second(mps_instruction_);
    }
    mps_instruction_.clear();

    std::cout << "--------------------------------" << std::endl;
  }

private:
  OpcUa::UaServer *server_;
  std::shared_ptr<MpsData> mps_data_;
  std::shared_ptr<spdlog::logger> logger_;

private:
  std::shared_ptr<Subscription> subscription_;

  // Registers which constitutes an MPS Command
  std::vector<MpsData::Registers> instruction_registers_;
  std::vector<std::string> instruction_nodes_name_;

  Instruction mps_instruction_;
  std::map<Instruction, std::function<void(Instruction)>> callbacks_;
  // std::tuple<unsigned short, unsigned short, unsigned short, int, unsigned
  // char,
  //            unsigned char>
  //     instruction_;
};

// struct MPSIntruction{
//    std::vector<MpsData::Regitsters> instruction_registers_;
//}

} // end namespace mps_comm
