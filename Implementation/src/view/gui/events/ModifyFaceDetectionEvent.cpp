#include <src/view/gui/events/ModifyFaceDetectionEvent.hpp>

namespace src
{
    namespace view
    {
        namespace gui
        {
            namespace events
            {
                ModifyFaceDetectionEvent::ModifyFaceDetectionEvent(const std::string &type)
                    : mProcessorType(type)
                {
                }

                std::string ModifyFaceDetectionEvent::getProcessorType() const
                {
                    return mProcessorType;
                }

            }
        }
    }
}
