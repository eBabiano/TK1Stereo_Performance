#ifndef FACEDETECTION_HPP
#define FACEDETECTION_HPP

#include <src/model/av/AV.hpp>

namespace src
{
    namespace model
    {
        namespace av
        {
            class FaceDetection
                    : public AV
            {
                public:
                    FaceDetection();

                    virtual std::string getProcessorType() const;
                    virtual void setProcessorType(const std::string& type);

                private:
                    std::string mProcessorType;
            };
        }
    }
}

#endif // FACEDETECTION_HPP
