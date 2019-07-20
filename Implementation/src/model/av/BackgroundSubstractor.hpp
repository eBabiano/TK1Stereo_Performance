#ifndef BACKGROUNDSUBSTRACTOR_HPP
#define BACKGROUNDSUBSTRACTOR_HPP

#include <src/model/av/AV.hpp>

namespace src
{
    namespace model
    {
        namespace av
        {
            class BackgroundSubstractor
                    : public AV
            {
                public:
                    BackgroundSubstractor();

                    virtual int getLearningRate() const;
                    virtual void setLearningRate(int learningRate);

            private:
                    int mLearningRate;

            };
        }
    }
}

#endif // BACKGROUNDSUBSTRACTOR_HPP
