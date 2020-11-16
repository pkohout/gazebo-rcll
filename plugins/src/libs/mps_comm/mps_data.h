
#include "opc_utils.h"
#include <exception>
#include <opc/ua/node.h>
#include <opc/ua/protocol/variant.h>
using namespace OpcUa;

namespace mps_comm {

class MpsData {
  // Registers existing in the MPS, to which it is possible to subscribe to
  enum Registers {
    ACTION_ID = 0,
    BARCODE,
    DATA,
    ERROR,
    SLIDECOUNT,
    STATUS_BUSY,
    STATUS_ENABLE,
    STATUS_ERROR,
    STATUS_READY,
    LAST, // must be the last in the list
  };

public:
  MpsData(std::shared_ptr<OpcUa::UaServer> OPCServer,
          std::shared_ptr<spdlog::logger> logger, std::vector<std::string> path)
      : OPCServer(OPCServer), logger_(logger) {

    // Get parent Object Node descibed by the base path
    Node parent = OPCServer->GetRootNode();
    uint16_t namespaceIdx = parent.GetId().GetNamespaceIndex();
    for (std::string str : path) {
      namespaceIdx = parent.GetId().GetNamespaceIndex();
      try {
        parent = parent.GetChild(str);
      } catch (std::exception &e) {
        // Node does not exist, Create it
        QualifiedName qname = ToQualifiedName(str, namespaceIdx);
        namespaceIdx = qname.NamespaceIndex;
        NodeId nodeid = NumericNodeId(0, namespaceIdx);
        parent = parent.AddObject(nodeid, qname);
        OpcUtils::logNodeInfo(parent, logger_, true, 2);
      }
    }

    parent = parent.AddObject(namespaceIdx, "p");
    reg_nodes_[ACTION_ID] =
        parent.AddVariable(namespaceIdx, "ActionId", Variant((uint16_t)0));
    reg_nodes_[BARCODE] =
        parent.AddVariable(namespaceIdx, "BarCode", Variant((uint32_t)0));
    reg_nodes_[SLIDECOUNT] =
        parent.AddVariable(namespaceIdx, "SlideCnt", Variant((uint16_t)0));
    reg_nodes_[ERROR] =
        parent.AddVariable(namespaceIdx, "Error", Variant((uint8_t)0));
    // Data Object Node
    reg_nodes_[DATA] = parent.AddObject(namespaceIdx, "Data");
    reg_nodes_[DATA].AddVariable(namespaceIdx, "payload1",
                                 Variant((uint16_t)0));
    reg_nodes_[DATA].AddVariable(namespaceIdx, "payload2",
                                 Variant((uint16_t)0));
    // Status Object Vars
    Node n_status = parent.AddObject(namespaceIdx, "Status");
    reg_nodes_[STATUS_BUSY] =
        n_status.AddVariable(namespaceIdx, "Busy", Variant(false));
    reg_nodes_[STATUS_ENABLE] =
        n_status.AddVariable(namespaceIdx, "Enable", Variant(false));
    reg_nodes_[STATUS_ERROR] =
        n_status.AddVariable(namespaceIdx, "Error", Variant((uint8_t)0));
    reg_nodes_[STATUS_READY] =
        n_status.AddVariable(namespaceIdx, "Ready", Variant(false));
  }

  ~MpsData(){};

  std::shared_ptr<OpcUa::UaServer> OPCServer;
  std::shared_ptr<spdlog::logger> logger_;

  // private:
  std::map<Registers, OpcUa::Node> reg_nodes_;
};

} // namespace mps_comm
