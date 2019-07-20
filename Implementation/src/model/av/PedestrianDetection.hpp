#ifndef PEDESTRIANDETECTION_HPP
#define PEDESTRIANDETECTION_HPP

#include <src/model/av/AV.hpp>

namespace src
{
    namespace model
    {
        namespace av
        {
            class PedestrianDetection
                    : public AV
            {
                public:
                    PedestrianDetection();

                    virtual std::string getProcessorType() const;
                    virtual void setProcessorType(const std::string& type);

                private:
                    std::string mProcessorType;
            };
        }
    }
}

#endif // PEDESTRIANDETECTION_HPP
