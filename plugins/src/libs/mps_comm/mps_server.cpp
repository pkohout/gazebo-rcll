/***************************************************************************
 *  mps_server.h - OPCUA server for an mps
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

#include "mps_server.h"

using namespace OpcUa;
using namespace mps_comm;
using namespace gazsim_msgs;

OPCServer::OPCServer(const std::string &name, const std::string &type,
                     const std::string &ip, unsigned int port)
    : UaServer(spdlog::stderr_color_mt("OPCserver " + name)) {
  mps_name_ = name;
  std::string endpoint =
      std::string("opc.tcp://localhost:") + std::to_string(port);
  printf(("starting OPC Server: " + endpoint + "\n").c_str());
  SetEndpoint(endpoint);
  SetServerURI(("urn://" + name + ".opcserver").c_str());
}

OPCServer::~OPCServer() {
  if (server_started_)
    Stop();
}

void OPCServer::run_server() {
  try {
    Start();
    // then register our server namespace and get its index in server
    RegisterNamespace("http://" + mps_name_);

    std::vector<std::string> basic_path = {
        "Objects",     "2:DeviceSet",   "4:CPX-E-CEC-C1-PN",
        "4:Resources", "4:Application", "3:GlobalVars",
        "4:G",         "4:Basic"};
    basic_data_ = std::make_shared<MpsData>(this, basic_path);
    basic_commands_handler_ =
        std::make_shared<CommandHandler>(this, basic_data_, Logger);
    basic_commands_handler_->handle_instructions(basic_data_->CommandRegisters);

    std::vector<std::string> in_path = {
        "Objects",     "2:DeviceSet",   "4:CPX-E-CEC-C1-PN",
        "4:Resources", "4:Application", "3:GlobalVars",
        "4:G",         "4:In"};
    in_data_ = std::make_shared<MpsData>(this, in_path);
    in_commands_handler_ =
        std::make_shared<CommandHandler>(this, in_data_, Logger);
    in_commands_handler_->handle_instructions(in_data_->CommandRegisters);

    server_started_ = true;
  } catch (const std::exception &exc) {
    std::cout << "Starting Server Failed: " << exc.what() << std::endl;
  }
}

void OPCServer::handle_instruction(Instruction i,
                                   std::function<void(Instruction)> callback) {
  // Commands with out a station code are handeled by basic nodes
  if (std::stoi(i["ActionId"]) < Station_MIN)
    basic_commands_handler_->register_instruction_callback(i, callback);
  else
    in_commands_handler_->register_instruction_callback(i, callback);
}

bool OPCServer::set_register_value(Register reg, std::string value) {
  return in_data_->set_value(reg, value);
}
