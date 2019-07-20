#ifndef MODIFYFACEDETECTIONEVENT_HPP
#define MODIFYFACEDETECTIONEVENT_HPP

#include <string>

namespace src
{
    namespace view
    {
        namespace gui
        {
            namespace events
            {
                class ModifyFaceDetectionEvent
                {
                    public:
                        ModifyFaceDetectionEvent(const std::string& type);

                        std::string getProcessorType() const;

                private:
                        std::string mProcessorType;
                };
            }
        }
    }
}

#endif // MODIFYFACEDETECTIONEVENT_HPP
