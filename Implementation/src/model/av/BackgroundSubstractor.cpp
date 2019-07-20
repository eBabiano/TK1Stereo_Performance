#include <src/model/av/BackgroundSubstractor.hpp>
#include <iostream>
#include <src/model/av/AVTypes.hpp>

namespace src
{
    namespace model
    {
        namespace av
        {
            BackgroundSubstractor::BackgroundSubstractor()
                : mLearningRate(0)
            {
            }

            int BackgroundSubstractor::getLearningRate() const
            {
                return mLearningRate;
            }

            void BackgroundSubstractor::setLearningRate(int learningRate)
            {
                mLearningRate = learningRate;
                notify(events::AlgorithmParametersModified(model::av::AVTypes::BACKGROUND_SUBTRACTOR));
            }
        }
    }
}
