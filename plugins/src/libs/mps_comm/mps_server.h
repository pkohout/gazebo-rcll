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

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>
#include <time.h>

#include <opc/ua/node.h>
#include <opc/ua/protocol/variant.h>
#include <opc/ua/server/server.h>
#include <opc/ua/subscription.h>

#include <gazsim_msgs/OpcComm.pb.h>

#include "mps_command.h"
#include "mps_data.h"
#include "opc_utils.h"

using namespace OpcUa;

namespace mps_comm {
class OPCServer : public OpcUa::UaServer {

public:
  OPCServer(const std::string &name, const std::string &type,
            const std::string &ip, unsigned int port);
  ~OPCServer();

  void run_server();

  void handle_instruction(Instruction instruction,
                          std::function<void(Instruction)> callback);

  bool set_register_value(Register reg, std::string value);

private:
  std::shared_ptr<MpsData> basic_data_;
  std::shared_ptr<MpsData> in_data_;

  std::shared_ptr<mps_comm::CommandHandler> basic_commands_handler_;
  std::shared_ptr<mps_comm::CommandHandler> in_commands_handler_;

private:
  std::string mps_name_;
  bool server_started_ = 0;
};

} // namespace mps_comm
