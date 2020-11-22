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
using namespace std::placeholders;
using namespace mps_comm;

void something(int i, int r) {}

Machine::Machine(std::string mps_name, std::shared_ptr<OPCServer> server)
    : server_(server) {
  machine_name_ = mps_name;
  if (mps_name == "C-BS" || mps_name == "M-BS")
    machine_type_ = STATION_BASE;
  if (mps_name == "C-RS1" || mps_name == "M-RS1" || mps_name == "C-RS2" ||
      mps_name == "M-RS2")
    machine_type_ = STATION_RING;
  if (mps_name == "C-CS1" || mps_name == "M-CS1" || mps_name == "C-CS2" ||
      mps_name == "M-CS2")
    machine_type_ = STATION_CAP;
  if (mps_name == "C-DS" || mps_name == "M-DS")
    machine_type_ = STATION_DELIVERY;
  if (mps_name == "C-SS" || mps_name == "M-SS")
    machine_type_ = STATION_STORAGE;

  register_instructions();
}

Machine::~Machine(){};

void Machine::register_instructions() {
  std::string mps_name = machine_name_;
  Instruction c = {{"ActionId", "0"},
                   {"Enable", "false"},
                   {"Payload1", "0"},
                   {"Payload2", "0"}};
  server_->handle_instruction(c, [mps_name](Instruction incoming) {
    std::cout << "HeartBeat " << mps_name << std::endl;
  });

  // Move Conveyor Commands
  c = {{"ActionId", std::to_string(machine_type_ + COMMAND_MOVE_CONVEYOR)}};
  auto f = std::bind(&Machine::publish_move_conveyor, this, _1);
  server_->handle_instruction(c, f);

  // SetLight Commands
  c = {{"ActionId", std::to_string(LIGHT_COLOR_RESET)}};
  server_->handle_instruction(c,
                              std::bind(&Machine::publish_set_light, this, _1));
  c = {{"ActionId", std::to_string(LIGHT_COLOR_RED)}};
  server_->handle_instruction(c,
                              std::bind(&Machine::publish_set_light, this, _1));
  c = {{"ActionId", std::to_string(LIGHT_COLOR_YELLOW)}};
  server_->handle_instruction(c,
                              std::bind(&Machine::publish_set_light, this, _1));
  c = {{"ActionId", std::to_string(LIGHT_COLOR_GREEN)}};
  server_->handle_instruction(c,
                              std::bind(&Machine::publish_set_light, this, _1));

  // Reset Command
  c = {{"ActionId", std::to_string(machine_type_ | COMMAND_RESET)}};
  server_->handle_instruction(c, std::bind(&Machine::publish_reset, this, _1));

  // Machine Operations
  c = {{"ActionId", std::to_string(machine_type_ + OPERATION_GET_BASE)}};
  server_->handle_instruction(
      c, std::bind(&Machine::publish_operation_base, this, _1));

  c = {{"ActionId", std::to_string(machine_type_ + OPERATION_CAP_ACTION)}};
  server_->handle_instruction(
      c, std::bind(&Machine::publish_operation_cap, this, _1));

  c = {{"ActionId", std::to_string(machine_type_ + OPERATION_MOUNT_RING)}};
  server_->handle_instruction(
      c, std::bind(&Machine::publish_operation_ring, this, _1));

  c = {{"ActionId", std::to_string(machine_type_ | OPERATION_DELIVER)}};
  server_->handle_instruction(
      c, std::bind(&Machine::publish_operation_deliver, this, _1));
};

void Machine::publish_move_conveyor(Instruction instruction) {
  MPSSensor sensor = static_cast<MPSSensor>(std::stoi(instruction["Payload1"]));
  ConveyorDirection direction =
      static_cast<ConveyorDirection>(std::stoi(instruction["Payload2"]));
  std::cout << "[" << machine_name_ << "] : "
            << " Move Conveyor " << sensor << " " << direction << std::endl;
}

void Machine::publish_reset(Instruction instruction) {
  std::cout << "[" << machine_name_ << "] :"
            << " Reset Received " << std::endl;
}
void Machine::publish_set_light(Instruction instruction) {
  LightColor l_color =
      static_cast<LightColor>(std::stoi(instruction["ActionId"]));
  LightState l_state =
      static_cast<LightState>(std::stoi(instruction["Payload1"]));
  std::cout << "[" << machine_name_ << "]"
            << "SetLight " << l_color << " " << l_state << std::endl;
}

void Machine::publish_operation_base(mps_comm::Instruction instruction) {
  BaseColor cl = static_cast<BaseColor>(std::stoi(instruction["Payload1"]));
  std::cout << "[" << machine_name_ << "] : "
            << "Recived Get Base " << cl << std::endl;
}

void Machine::publish_operation_cap(mps_comm::Instruction instruction) {
  Operation p1 = static_cast<Operation>(std::stoi(instruction["Payload1"]));
  std::cout << "[" << machine_name_ << "] : "
            << "Recived Cap Action " << p1 << std::endl;
}

void Machine::publish_operation_ring(mps_comm::Instruction instruction) {
  unsigned int feeder =
      static_cast<unsigned int>(std::stoi(instruction["Payload1"]));
  std::cout << "[" << machine_name_ << "] : "
            << "Recived Mount Ring (feeder: " << feeder << ")" << std::endl;
}

void Machine::publish_operation_deliver(mps_comm::Instruction instruction) {
  int slot = static_cast<unsigned int>(std::stoi(instruction["Payload1"]));
  std::cout << "[" << machine_name_ << "] : "
            << "Recived Deliver (slot: " << slot << ")" << std::endl;
}

void Machine::on_status_busy(bool busy) {}
void Machine::on_status_ready(bool ready) {}
void Machine::on_barcode(uint16_t barcode) {}
