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

#include "opc_utils.h"
using namespace OpcUa;

namespace mps_comm {

class SubClient : public SubscriptionHandler {
public:
  SubClient(std::shared_ptr<spdlog::logger> logger_) : logger(logger_){};
  void DataChange(uint32_t handle, const Node &node, const Variant &val,
                  AttributeId attr) override {
    std::cout << node.GetParent().GetBrowseName().Name << ":"
              << node.GetBrowseName().Name << std::endl;
    OpcUtils::logNodeInfo(node, logger, true, 2);
    if (node.GetBrowseName().Name.find("Error") != std::string::npos &&
        node.GetParent().GetBrowseName().Name.find("p") != std::string::npos) {
      //  OpcUtils::logNodeInfo(node.GetParent(), logger, true, 2);
      uint16_t actionid = static_cast<uint16_t>(
          node.GetParent().GetChild("4:ActionId").GetValue());
      //    uint16_t payload1 =
      //       static_cast<uint16_t>(node.GetParent().GetChild("4:Data").GetChild("4:payload1").GetValue());
      //    uint16_t payload2 =
      //       static_cast<uint16_t>(node.GetParent().GetChild("4:Data").GetChild("4:paylood2").GetValue());
      // unsigned char  error    =
      //  static_cast<char>(node.GetParent().GetChild("4:Error").GetValue());

      std::cout << actionid << std::endl;
      //      std::cout << actionid << " " << payload1  << " " << payload2 << "
      //      " << std::endl;
    }
  }

private:
  std::shared_ptr<spdlog::logger> logger;
  std::tuple<unsigned short, unsigned short, unsigned short, int, unsigned char,
             unsigned char>
      instruction_;
};

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

  std::shared_ptr<SubClient> subscription_client_;
  std::shared_ptr<Subscription> subscription_;

  std::shared_ptr<OpcUa::Node> n_basic_;
  std::shared_ptr<OpcUa::Node> n_in_;

private:
  std::string mps_name_;
  bool server_started_ = 0;
};

} // namespace mps_comm
