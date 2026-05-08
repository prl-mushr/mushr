#include "ackermann_cmd_mux/ackermann_cmd_subscribers.hpp"
#include "ackermann_cmd_mux/exceptions.hpp"

namespace ackermann_cmd_mux {

void AckermannCmdSubscribers::configure(const YAML::Node & node)
{
    list.clear();

    if (node.size() == 0) {
        throw EmptyCfgException();
    }

    try {
        for (size_t i = 0; i < node.size(); ++i)
        {
            AckermannCmdSubs sub(i);
            sub << node[i];
            list.push_back(sub);
        }
    }
    catch (const YAML::ParserException & e)
    {
        throw YamlException(e.what());
    }
    catch (const YAML::RepresentationException & e)
    {
        throw YamlException(e.what());
    }
}

} // namespace ackermann_cmd_mux
