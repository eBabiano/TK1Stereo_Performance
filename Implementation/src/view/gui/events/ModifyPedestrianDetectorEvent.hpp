#ifndef MODIFYPEDESTRIANDETECTOREVENT_HPP
#define MODIFYPEDESTRIANDETECTOREVENT_HPP

#include <string>

namespace src
{
    namespace view
    {
        namespace gui
        {
            namespace events
            {
                class ModifyPedestrianDetectorEvent
                {
                    public:
                        ModifyPedestrianDetectorEvent(const std::string& processorType);

                        std::string getProcessorType() const;

                private:
                        std::string mProcessorType;
                };
            }
        }
    }
}

#endif // MODIFYPEDESTRIANDETECTOREVENT_HPP
