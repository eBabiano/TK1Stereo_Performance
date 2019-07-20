#include <src/view/gui/events/ModifyBackgroundSubstractorEvent.hpp>

namespace src
{
    namespace view
    {
        namespace gui
        {
            namespace events
            {
                ModifyBackgroundSubstractorEvent::ModifyBackgroundSubstractorEvent(int learningRate)
                    : mLearningRate(learningRate)
                {
                }

                int ModifyBackgroundSubstractorEvent::getLearningRate() const
                {
                    return mLearningRate;
                }

            }
        }
    }
}
