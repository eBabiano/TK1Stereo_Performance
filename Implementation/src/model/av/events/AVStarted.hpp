#ifndef AVSTARTED_HPP
#define AVSTARTED_HPP

#include <string>

namespace src
{
    namespace model
    {
       namespace av
       {
           namespace events
           {
               class AVStarted
               {
                   public:
                       AVStarted(const std::string& type, bool activated);

                       std::string getType() const;
                       bool getIsActivated() const;

                    private:
                       std::string mType;
                       bool mIsActivated;
               };
           }
       }
    }
}

#endif // AVSTARTED_HPP
