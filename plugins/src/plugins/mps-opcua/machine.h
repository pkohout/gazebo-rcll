/***************************************************************************
 *  machine.h - OPC-UA communication with an MPS
 *
 *  Created: Thu 20 Noc 2020 13:29:11 CET 13:29
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
#pragma once

#include <mps_comm/mps_io_mapping.h>
#include <mps_comm/mps_server.h>

#include <string>
#include <vector>

namespace gazebo {

enum ConveyorDirection { FORWARD = 1, BACKWARD = 2 };
enum MPSSensor { INPUT = 1, MIDDLE = 2, OUTPUT = 3 };

class Machine {

public:
  Machine(std::string machine_name,
          std::shared_ptr<mps_comm::OPCServer> server);

  ~Machine();

  void register_instructions();
  void on_status_busy(bool busy);
  void on_status_ready(bool ready);
  void on_barcode(uint16_t barcode);

protected:
  void publish_move_conveyor(mps_comm::Instruction instruction);
  // Reset: send the reset command (which is different for each machine type)
  void publish_reset(mps_comm::Instruction instruction);
  // Set & reset the light of specified color to specified state
  // color: 1 - 3, state 0 - 2
  void publish_set_light(mps_comm::Instruction instruction);
  void publish_operation_get_base(mps_comm::Instruction instruction);
  void publish_operation_retrieve_cap(mps_comm::Instruction instruction);
  void publish_operation_mount_cap(mps_comm::Instruction instruction);
  void publish_operation_mount_ring(mps_comm::Instruction instruction);
  void publish_operation_deliver(mps_comm::Instruction instruction);

  mps_comm::Station machine_type_;
  std::string machine_name_;
  // OpcUa Server pointer
  std::shared_ptr<mps_comm::OPCServer> server_;
};

} // namespace gazebo
