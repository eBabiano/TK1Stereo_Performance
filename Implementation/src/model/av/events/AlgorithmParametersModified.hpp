#ifndef ALGORITHMPARAMETERSMODIFIED_HPP
#define ALGORITHMPARAMETERSMODIFIED_HPP

#include <string>

namespace src
{
    namespace model
    {
       namespace av
       {
           namespace events
           {
               class AlgorithmParametersModified
               {
                   public:
                       AlgorithmParametersModified(const std::string& type);

                       std::string getType() const;

               private:
                       std::string mType;
               };
           }
       }
    }
}

#endif // ALGORITHMPARAMETERSMODIFIED_HPP
