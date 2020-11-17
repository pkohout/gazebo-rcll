/// @brief OPC UA Server main.
/// @license GNU LGPL
///
/// Distributed under the GNU LGPL License
/// (See accompanying file LICENSE or copy at
/// http://www.gnu.org/licenses/lgpl.html)
///
#include <algorithm>
#include <iostream>
#include <time.h>

#include <chrono>
#include <thread>

#include <opc/ua/node.h>
#include <opc/ua/protocol/variant.h>
#include <opc/ua/server/server.h>
#include <opc/ua/subscription.h>

#include "mps_command.h"
#include "mps_data.h"
#include "opc_utils.h"

using namespace OpcUa;

namespace mps_comm {
class OPCServer {

public:
  OPCServer(const std::string &name, const std::string &type,
            const std::string &ip, unsigned int port);
  ~OPCServer();

  void run_server();
  void populate_nodes();

private:
  std::shared_ptr<spdlog::logger> logger_;
  std::shared_ptr<OpcUa::UaServer> server_;

  std::shared_ptr<mps_comm::CommandHandler> basic_commands_handler_;
  std::shared_ptr<mps_comm::CommandHandler> in_commands_handler_;

private:
  std::string mps_name_;
  bool server_started_ = 0;
};

} // namespace mps_comm
