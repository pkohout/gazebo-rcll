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
      instruction_set_flags_[node.GetBrowseName().Name] = false;
      subscription_->SubscribeDataChange(node);
    }
  }

  void
  register_instruction_callback(Instruction instruction,
                                std::function<void(Instruction)> callback) {
    callbacks_[instruction] = callback;
  }



/* There is no guarantees on the order by which the node values (making up
  an instruction) are received. This is a cheep solution to keep flags for
  each register and drop information which arrives on set flag. Dispatch
  when all flags are set.
  TODO: implement a proper instruction queue
*/
  void DataChange(uint32_t handle, const Node &node, const Variant &val,
                  AttributeId attr) override {

    if (mps_data_->type().find("In") != std::string::npos)
      std::cout << "...Recived :  " << node.GetBrowseName().Name << ":"
                << node.GetValue().ToString() << std::endl;

    if (!instruction_set_flags_[node.GetBrowseName().Name]) {
      mps_instruction_[node.GetBrowseName().Name] = node.GetValue().ToString();
      instruction_set_flags_[node.GetBrowseName().Name] = true;
    } else
      std::cout
          << "...OPC Recived : new instruction wants to used register ["
          << node.GetBrowseName().Name << ":" << node.GetValue().ToString()
          << "] while old instruction still not dispatched...dropping value!"
          << std::endl;

    dispatch_instruction();
  }

  void dispatch_instruction() {

    for (auto &set_flags : instruction_set_flags_)
      if (!set_flags.second)
        return;

    if (mps_data_->type().find("In") != std::string::npos) {
      std::cout << "---------------Dispatched---------------" << std::endl;
      for (auto &i : mps_instruction_)
        std::cout << i.first << " : " << i.second << std::endl;

      std::cout << "--------------------------------" << std::endl;
    }

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
    //  mps_instruction_.clear();
    //  mps_instruction_ = {{"ActionId", "0"},
    //                      {"Enable", "false"},
    //                      {"Payload1", "0"},
    //                      {"Payload2", "0"},
    //                      {"Error", ""}
    //                      };

    for (auto &set_flags : instruction_set_flags_)
      set_flags.second = false;
  }

private:
  OpcUa::UaServer *server_;
  std::shared_ptr<MpsData> mps_data_;
  std::shared_ptr<spdlog::logger> logger_;

private:
  std::shared_ptr<Subscription> subscription_;

  // Registers which constitutes an MPS Command
  // Desciption of an instruction registers
  std::vector<Register> instruction_registers_;
  //Desciption of instruction node_names
  std::vector<std::string> instruction_nodes_name_;
  //Flags to track a single values beloning to the same instruction
  std::map<std::string, bool> instruction_set_flags_;
  //The instructions
  Instruction mps_instruction_;

  std::map<Instruction, std::function<void(Instruction)>> callbacks_;
};
} // end namespace mps_comm
