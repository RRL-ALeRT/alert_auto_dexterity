#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

using namespace BT;

std::string join(std::vector<std::string>& vec, const char* delim)
{
  std::stringstream res;

  for (size_t i = 0; i < vec.size(); ++i) {
    res << vec[i];
    if (i < vec.size() - 1) {
      res << delim;
    }
  }

  return res.str();
}

class StoreObjectList : public SyncActionNode
{
  public:
  StoreObjectList(const std::string& name, const NodeConfig& config) :
        SyncActionNode(name, config)
  {}

  NodeStatus tick() override
  {
    std::string objects_list;
    std::string pop_object;

    getInput("objects_list", objects_list);
    getInput("pop_object", pop_object);

    auto object_vec = splitString(objects_list, ';');
    std::vector<std::string> string_vec(object_vec.begin(), object_vec.end());

    if (pop_object.size() > 0)
    {
      auto itr = std::find(string_vec.begin(), string_vec.end(), pop_object);
      if (itr != string_vec.end())
      {
        string_vec.erase(itr);
        auto output = join(string_vec, ";");

        setOutput<std::string>("objects_list", output);
      }
    }

    SharedQueue<std::string> shared_queue = std::make_shared<std::deque<std::string>>();
    for (const StringView& part : string_vec)
    {
      shared_queue->push_back(convertFromString<std::string>(part));
    }
    setOutput<SharedQueue<std::string>>("objects_loop", shared_queue);

    return NodeStatus::SUCCESS;
  }

  static PortsList providedPorts()
  {
    return {BidirectionalPort<std::string>("objects_list"), InputPort<std::string>("pop_object"), OutputPort<SharedQueue<std::string>>("objects_loop")};
  }
};

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<StoreObjectList>("StoreObjectList");
}