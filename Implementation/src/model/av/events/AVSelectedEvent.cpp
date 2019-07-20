#include <src/model/av/events/AVSelectedEvent.hpp>

namespace src
{
    namespace model
    {
       namespace av
       {
           namespace events
           {
               AVSelectedEvent::AVSelectedEvent(const std::string &type)
                   : mType(type)
               {
               }

               std::string AVSelectedEvent::getType() const
               {
                   return mType;
               }

           }
       }
    }
}
