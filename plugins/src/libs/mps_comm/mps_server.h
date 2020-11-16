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

class MpsDataNode {
public:
  MpsDataNode(OpcUa::Node parent) {
    Parent = parent;

    Node n_p = parent.AddObject(4, "p");

    ActionId = n_p.AddVariable(4, "ActionId", Variant((uint16_t)0));
    BarCode = n_p.AddVariable(4, "BarCode", Variant((uint32_t)0));
    Error = n_p.AddVariable(4, "Error", Variant((uint8_t)0));
    SlideCount = n_p.AddVariable(4, "SlideCnt", Variant((uint16_t)0));

    Node n_data = n_p.AddObject(4, "Data");
    Data_Payload1 = n_data.AddVariable(4, "payload1", Variant((uint16_t)0));
    Data_Payload2 = n_data.AddVariable(4, "payload2", Variant((uint16_t)0));

    Node n_status = n_p.AddObject(4, "Status");
    Status_Busy = n_status.AddVariable(4, "Busy", Variant(false));
    Status_Enable = n_status.AddVariable(4, "Enable", Variant(false));
    Status_Error = n_status.AddVariable(4, "Error", Variant((uint8_t)0));
    Status_Ready = n_status.AddVariable(4, "Ready", Variant(false));
  }

  ~MpsDataNode(){};

  OpcUa::Node Parent;

  // Commnon
  OpcUa::Node Error;

  // Command Nodes
  OpcUa::Node ActionId;
  OpcUa::Node Data_Payload1;
  OpcUa::Node Data_Payload2;

  // status Nodes
  OpcUa::Node Status_Enable;
  OpcUa::Node Status_Busy;
  OpcUa::Node Status_Ready;
  OpcUa::Node Status_Error;

  // Sensor Nodes
  OpcUa::Node BarCode;
  OpcUa::Node SlideCount;
};

class CommandHandler : public SubscriptionHandler {
public:
  CommandHandler(std::shared_ptr<OpcUa::UaServer> server,
                 std::shared_ptr<spdlog::logger> logger)
      : server_(server), logger_(logger){};
  ~CommandHandler(){};

  void register_handler(std::shared_ptr<MpsDataNode> data_node) {
    data_node_ = data_node;
    subscription_ = server_->CreateSubscription(100, *this);

    subscription_->SubscribeDataChange(data_node->ActionId);
    subscription_->SubscribeDataChange(data_node->Data_Payload1);
    subscription_->SubscribeDataChange(data_node->Data_Payload2);
    subscription_->SubscribeDataChange(data_node->Status_Enable);
    subscription_->SubscribeDataChange(data_node->Error);
  }

  void DataChange(uint32_t handle, const Node &node, const Variant &val,
                  AttributeId attr) override {
    if (node == data_node_->ActionId)
      std::cout << "======================" << std::endl;

    std::cout << node.GetParent().GetBrowseName().Name << ":"
              << node.GetBrowseName().Name << " " << node.GetValue().ToString()
              << std::endl;
    //  OpcUtils::logNodeInfo(node, logger_, true, 2);
    //  if (node.GetBrowseName().Name.find("Error") != std::string::npos &&
    //     node.GetParent().GetBrowseName().Name.find("p") != std::string::npos)
    //     {
    //  OpcUtils::logNodeInfo(node.GetParent(), logger, true, 2);
    //  uint16_t actionid = static_cast<uint16_t>(
    //      node.GetParent().GetChild("4:ActionId").GetValue());
    //    uint16_t payload1 =
    //       static_cast<uint16_t>(node.GetParent().GetChild("4:Data").GetChild("4:payload1").GetValue());
    //    uint16_t payload2 =
    //       static_cast<uint16_t>(node.GetParent().GetChild("4:Data").GetChild("4:paylood2").GetValue());
    // unsigned char  error    =
    //  static_cast<char>(node.GetParent().GetChild("4:Error").GetValue());

    //  std::cout << actionid << std::endl;
    //      std::cout << actionid << " " << payload1  << " " << payload2 << "
    //      " << std::endl;
  }

private:
  std::shared_ptr<OpcUa::UaServer> server_;
  std::shared_ptr<spdlog::logger> logger_;

  std::shared_ptr<MpsDataNode> data_node_;
  std::tuple<unsigned short, unsigned short, unsigned short, int, unsigned char,
             unsigned char>
      instruction_;

  std::shared_ptr<Subscription> subscription_;
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

  std::shared_ptr<mps_comm::CommandHandler> basic_commands_handler_;
  std::shared_ptr<mps_comm::CommandHandler> in_commands_handler_;

private:
  std::string mps_name_;
  bool server_started_ = 0;
};

} // namespace mps_comm
