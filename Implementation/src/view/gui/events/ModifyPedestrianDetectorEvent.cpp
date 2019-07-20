#include <src/view/gui/events/ModifyPedestrianDetectorEvent.hpp>

namespace src
{
    namespace view
    {
        namespace gui
        {
            namespace events
            {
                ModifyPedestrianDetectorEvent::ModifyPedestrianDetectorEvent(const std::string& processorType)
                    : mProcessorType(processorType)
                {
                }
                std::string ModifyPedestrianDetectorEvent::getProcessorType() const
                {
                    return mProcessorType;
                }

            }
        }
    }
}
