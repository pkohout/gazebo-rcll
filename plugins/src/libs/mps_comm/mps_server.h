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
#include <opc/ua/server/server.h>
#include <opc/ua/subscription.h>

#include "opc_utils.h"
using namespace OpcUa;

namespace mps_comm {

class SubClient : public SubscriptionHandler {
public:
  SubClient(std::shared_ptr<spdlog::logger> logger_) : logger(logger_){};
  void DataChange(uint32_t handle, const Node &node, const Variant &val,
                  AttributeId attr) override {
    OpcUtils::logNodeInfo(node, logger, true, 2);
  }

private:
  std::shared_ptr<spdlog::logger> logger;
};

class OPCServer {

public:
  OPCServer(const std::string &name, const std::string &type,
            const std::string &ip, unsigned int port);
  ~OPCServer();

  void run_server();
  void print(OpcUa::Node start_node, std::string level);

private:
  std::shared_ptr<spdlog::logger> logger_;
  std::shared_ptr<OpcUa::UaServer> server_;
  std::shared_ptr<SubClient> subscription_client_;
  std::shared_ptr<Subscription> subscription_;

  std::string mps_name_;
  bool server_started_ = 0;
};

} // namespace mps_comm
