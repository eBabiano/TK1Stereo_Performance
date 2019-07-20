#include <src/model/av/events/AlgorithmParametersModified.hpp>

namespace src
{
    namespace model
    {
       namespace av
       {
           namespace events
           {
               AlgorithmParametersModified::AlgorithmParametersModified(const std::string &type)
                   : mType(type)
               {
               }
               std::string AlgorithmParametersModified::getType() const
               {
                   return mType;
               }

           }
       }
    }
}
