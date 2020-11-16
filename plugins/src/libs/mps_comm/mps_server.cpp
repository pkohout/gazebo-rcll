/// @brief OPC UA Server main.
/// @license GNU LGPL
///
/// Distributed under the GNU LGPL License
/// (See accompanying file LICENSE or copy at
/// http://www.gnu.org/licenses/lgpl.html)
///
#include "mps_server.h"

using namespace OpcUa;

using namespace mps_comm;

OPCServer::OPCServer(const std::string &name, const std::string &type,
                     const std::string &ip, unsigned int port) {
  mps_name_ = name;
  std::string endpoint =
      std::string("opc.tcp://localhost:") + std::to_string(port);
  printf("starting OPC Server: ");
  printf((endpoint + "\n").c_str());
  logger_ = spdlog::stderr_color_mt("OPCserver " + name);
  server_ = std::make_shared<OpcUa::UaServer>(logger_);
  server_->SetEndpoint(endpoint);
  server_->SetServerURI(("urn://" + name + ".opcserver").c_str());
}

OPCServer::~OPCServer() {
  if (server_started_)
    server_->Stop();
}

void OPCServer::run_server() {
  try {
    server_->Start();
    // then register our server namespace and get its index in server
    server_->RegisterNamespace("http://" + mps_name_);

    Node n_g = server_->GetObjectsNode()
                   .AddObject(2, "DeviceSet")
                   .AddObject(4, "CPX-E-CEC-C1-PN")
                   .AddObject(4, "Resources")
                   .AddObject(4, "Application")
                   .AddObject(3, "GlobalVars")
                   .AddObject(4, "G");

    Node n_basic = n_g.AddObject(4, "Basic");
    Node n_in = n_g.AddObject(4, "In");

    std::shared_ptr<MpsDataNode> basic_data =
        std::make_shared<MpsDataNode>(n_basic);
    std::shared_ptr<MpsDataNode> in_data = std::make_shared<MpsDataNode>(n_in);

    basic_commands_handler_ =
        std::make_shared<CommandHandler>(server_, logger_);
    basic_commands_handler_->register_handler(basic_data);
    in_commands_handler_ = std::make_shared<CommandHandler>(server_, logger_);
    in_commands_handler_->register_handler(basic_data);

    server_started_ = true;

  }

  catch (const std::exception &exc) {
    std::cout << "Starting Server Failed: " << exc.what() << std::endl;
  }
}
