#ifndef MODIFYBACKGROUNDSUBSTRACTOREVENT_HPP
#define MODIFYBACKGROUNDSUBSTRACTOREVENT_HPP

namespace src
{
    namespace view
    {
        namespace gui
        {
            namespace events
            {
                class ModifyBackgroundSubstractorEvent
                {
                    public:
                        ModifyBackgroundSubstractorEvent(int learningRate);

                        int getLearningRate() const;

                private:
                        int mLearningRate;
                };
            }
        }
    }
}

#endif // MODIFYBACKGROUNDSUBSTRACTOREVENT_HPP
