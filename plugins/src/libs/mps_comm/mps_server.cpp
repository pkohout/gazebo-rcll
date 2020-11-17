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

    std::vector<std::string> basic_path = {
        "Objects",     "2:DeviceSet",   "4:CPX-E-CEC-C1-PN",
        "4:Resources", "4:Application", "3:GlobalVars",
        "4:G",         "4:Basic"};
    std::shared_ptr<MpsData> basic_data =
        std::make_shared<MpsData>(server_, logger_, basic_path);
    basic_commands_handler_ = std::make_shared<CommandHandler>(basic_data);
    basic_commands_handler_->handle_commands(basic_data->CommandRegisters);

    std::vector<std::string> in_path = {
        "Objects",     "2:DeviceSet",   "4:CPX-E-CEC-C1-PN",
        "4:Resources", "4:Application", "3:GlobalVars",
        "4:G",         "4:In"};
    std::shared_ptr<MpsData> in_data =
        std::make_shared<MpsData>(server_, logger_, in_path);
    in_commands_handler_ = std::make_shared<CommandHandler>(in_data);
    in_commands_handler_->handle_commands(in_data->CommandRegisters);

    server_started_ = true;

  }

  catch (const std::exception &exc) {
    std::cout << "Starting Server Failed: " << exc.what() << std::endl;
  }
}
