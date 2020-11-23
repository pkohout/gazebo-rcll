/***************************************************************************
 *  machine.h - OPC-UA communication with an MPS
 *
 *  Created: Thu 20 Nov 2020 13:29:11 CET 13:29
 *  Copyright  2020  Mostafa Gomaa <gomaa@kbsg.rwth-aachen.de>
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

// Abstract base class for the stations

#include "machine.h"
#include <gazsim_msgs/OpcInstruction.pb.h>

using namespace gazebo;
using namespace gazsim_msgs;
using namespace mps_comm;

void something(int i, int r) {}

Machine::Machine(std::string mps_name, std::shared_ptr<OPCServer> server,
                 transport::NodePtr transport_node)
    : server_(server) {

  transport_node_ = transport_node;
  machine_name_ = mps_name;
  if (mps_name.find("BS") != std::string::npos)
    machine_type_ = STATION_BASE;
  if (mps_name.find("RS") != std::string::npos)
    machine_type_ = STATION_RING;
  if (mps_name.find("CS") != std::string::npos)
    machine_type_ = STATION_CAP;
  if (mps_name.find("DS") != std::string::npos)
    machine_type_ = STATION_DELIVERY;
  if (mps_name.find("SS") != std::string::npos)
    machine_type_ = STATION_STORAGE;

  register_instructions();
}

Machine::~Machine(){};

void Machine::register_instructions() {
  std::string mps_name = machine_name_;
  // Instrucion Heartbeat
  Instruction c = {{"ActionId", "0"},
                   {"Enable", "false"},
                   {"Payload1", "0"},
                   {"Payload2", "0"}};
  server_->handle_instruction(c,
                              std::bind(&Machine::publish_instruction_heartbeat,
                                        this, std::placeholders::_1));

  // Instruction Move Conveyor
  c = {{"ActionId", std::to_string(machine_type_ + COMMAND_MOVE_CONVEYOR)}};
  server_->handle_instruction(
      c, std::bind(&Machine::publish_instruction_move_conveyor, this,
                   std::placeholders::_1));

  // Instruction SetLight
  c = {{"ActionId", std::to_string(LIGHT_COLOR_RESET)}};
  server_->handle_instruction(c,
                              std::bind(&Machine::publish_instruction_set_light,
                                        this, std::placeholders::_1));
  c = {{"ActionId", std::to_string(LIGHT_COLOR_RED)}};
  server_->handle_instruction(c,
                              std::bind(&Machine::publish_instruction_set_light,
                                        this, std::placeholders::_1));
  c = {{"ActionId", std::to_string(LIGHT_COLOR_YELLOW)}};
  server_->handle_instruction(c,
                              std::bind(&Machine::publish_instruction_set_light,
                                        this, std::placeholders::_1));
  c = {{"ActionId", std::to_string(LIGHT_COLOR_GREEN)}};
  server_->handle_instruction(c,
                              std::bind(&Machine::publish_instruction_set_light,
                                        this, std::placeholders::_1));

  // Instruction Reset Command
  c = {{"ActionId", std::to_string(machine_type_ | COMMAND_RESET)}};
  server_->handle_instruction(c, std::bind(&Machine::publish_instruction_reset,
                                           this, std::placeholders::_1));
  // Instruction Base Operations
  c = {{"ActionId", std::to_string(machine_type_ + OPERATION_GET_BASE)}};
  server_->handle_instruction(
      c, std::bind(&Machine::publish_instruction_base_operation, this,
                   std::placeholders::_1));
  // Instruction Cap Operations
  c = {{"ActionId", std::to_string(machine_type_ + OPERATION_CAP_ACTION)}};
  server_->handle_instruction(
      c, std::bind(&Machine::publish_instruction_cap_operation, this,
                   std::placeholders::_1));
  // Instruction Ring Operations
  c = {{"ActionId", std::to_string(machine_type_ + OPERATION_MOUNT_RING)}};
  server_->handle_instruction(
      c, std::bind(&Machine::publish_instruction_ring_operation, this,
                   std::placeholders::_1));
  // Instruction Deliver Operations
  c = {{"ActionId", std::to_string(machine_type_ | OPERATION_DELIVER)}};
  server_->handle_instruction(
      c, std::bind(&Machine::publish_instruction_deliver_operation, this,
                   std::placeholders::_1));

  // Publishers for Opc Instructions as gaszim_msgs
  publisher_heartbeat_ =
      transport_node_->Advertise<gazsim_msgs::OpcInstructionHeartbeat>(
          "~/opcua/instruction/heartbeat");
  publisher_reset_ =
      transport_node_->Advertise<gazsim_msgs::OpcInstructionReset>(
          "~/opcua/instruction/reset");
  publisher_move_conveyor_ =
      transport_node_->Advertise<gazsim_msgs::OpcInstructionMoveConveyor>(
          "~/opcua/instruction/move_conveyor");
  publisher_set_light_ =
      transport_node_->Advertise<gazsim_msgs::OpcInstructionSetLight>(
          "~/opcua/instruction/set_light");
  publisher_base_operation_ =
      transport_node_->Advertise<gazsim_msgs::OpcInstructionBaseOperation>(
          "~/opcua/instruction/base_operation");
  publisher_cap_operation_ =
      transport_node_->Advertise<gazsim_msgs::OpcInstructionCapOperation>(
          "~/opcua/instruction/cap_operation");
  publisher_ring_operation_ =
      transport_node_->Advertise<gazsim_msgs::OpcInstructionRingOperation>(
          "~/opcua/instruction/ring_operation");
  publisher_deliver_operation_ =
      transport_node_->Advertise<gazsim_msgs::OpcInstructionDeliverOperation>(
          "~/opcua/instruction/deliver_operation");
};

void Machine::publish_instruction_heartbeat(Instruction instruction) {
  std::cout << "[" << machine_name_ << "] :"
            << "HeartBeat " << std::endl;
  OpcInstructionHeartbeat msg;
  msg.set_mps(machine_name_);
  msg.set_station(machine_type_);
  publisher_heartbeat_->Publish(msg);
}

void Machine::publish_instruction_reset(Instruction instruction) {
  std::cout << "[" << machine_name_ << "] :"
            << " Reset Received " << std::endl;
  OpcInstructionReset msg;
  msg.set_mps(machine_name_);
  msg.set_station(machine_type_);
  publisher_reset_->Publish(msg);
}

void Machine::publish_instruction_move_conveyor(Instruction instruction) {
  MPSSensor sensor = static_cast<MPSSensor>(std::stoi(instruction["Payload1"]));
  ConveyorDirection direction =
      static_cast<ConveyorDirection>(std::stoi(instruction["Payload2"]));
  std::cout << "[" << machine_name_ << "] : "
            << " Move Conveyor " << sensor << " " << direction << std::endl;
  OpcInstructionMoveConveyor msg;
  msg.set_mps(machine_name_);
  msg.set_station(machine_type_);
  msg.set_sensor(sensor);
  msg.set_direction(direction);
  publisher_move_conveyor_->Publish(msg);
}

void Machine::publish_instruction_set_light(Instruction instruction) {
  LightColor l_color =
      static_cast<LightColor>(std::stoi(instruction["ActionId"]));
  LightState l_state =
      static_cast<LightState>(std::stoi(instruction["Payload1"]));
  std::cout << "[" << machine_name_ << "]"
            << "SetLight " << l_color << " " << l_state << std::endl;
  OpcInstructionSetLight msg;
  msg.set_mps(machine_name_);
  msg.set_station(machine_type_);
  msg.set_color(l_color);
  msg.set_state(l_state);
  publisher_set_light_->Publish(msg);
}

void Machine::publish_instruction_base_operation(
    mps_comm::Instruction instruction) {
  BaseColor cl = static_cast<BaseColor>(std::stoi(instruction["Payload1"]));
  std::cout << "[" << machine_name_ << "] : "
            << "Recived Get Base " << cl << std::endl;

  OpcInstructionBaseOperation msg;
  msg.set_mps(machine_name_);
  msg.set_station(machine_type_);
  msg.set_color(cl);
  publisher_base_operation_->Publish(msg);
}

void Machine::publish_instruction_cap_operation(
    mps_comm::Instruction instruction) {
  Operation p1 = static_cast<Operation>(std::stoi(instruction["Payload1"]));
  std::cout << "[" << machine_name_ << "] : "
            << "Recived Cap Action " << p1 << std::endl;

  OpcInstructionCapOperation msg;
  msg.set_mps(machine_name_);
  msg.set_station(machine_type_);
  msg.set_operation(p1);
  publisher_cap_operation_->Publish(msg);
}

void Machine::publish_instruction_ring_operation(
    mps_comm::Instruction instruction) {
  unsigned int feeder =
      static_cast<unsigned int>(std::stoi(instruction["Payload1"]));
  std::cout << "[" << machine_name_ << "] : "
            << "Recived Mount Ring (feeder: " << feeder << ")" << std::endl;

  OpcInstructionRingOperation msg;
  msg.set_mps(machine_name_);
  msg.set_station(machine_type_);
  msg.set_feeder(feeder);
  publisher_ring_operation_->Publish(msg);
}

void Machine::publish_instruction_deliver_operation(
    mps_comm::Instruction instruction) {
  int slot = static_cast<unsigned int>(std::stoi(instruction["Payload1"]));
  std::cout << "[" << machine_name_ << "] : "
            << "Recived Deliver (slot: " << slot << ")" << std::endl;

  OpcInstructionDeliverOperation msg;
  msg.set_mps(machine_name_);
  msg.set_station(machine_type_);
  msg.set_slot(slot);
  publisher_deliver_operation_->Publish(msg);
}

void Machine::on_status_busy(bool busy) {}
void Machine::on_status_ready(bool ready) {}
void Machine::on_barcode(uint16_t barcode) {}
