#ifndef AV_HPP
#define AV_HPP

#include <src/util/Observable.hpp>
#include <src/model/av/events/AlgorithmParametersModified.hpp>
namespace src
{
    namespace model
    {
        namespace av
        {
            class AV
                    : public util::Observable<events::AlgorithmParametersModified>
            {
                public:
                    AV();

                    //BACKGROUND SUBSTRACTOR
                    virtual int getLearningRate() const {}
                    virtual void setLearningRate(int learningRate){}

                    //COMMON
                    virtual std::string getProcessorType() const {}
                    virtual void setProcessorType(const std::string& type) {}

            protected:
                    using util::Observable<events::AlgorithmParametersModified>::notify;


            };
        }
    }
}

#endif // AV_HPP
