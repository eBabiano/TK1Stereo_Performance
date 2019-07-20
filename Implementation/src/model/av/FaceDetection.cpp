#include <src/model/av/FaceDetection.hpp>
#include <src/model/av/AVTypes.hpp>

namespace src
{
    namespace model
    {
        namespace av
        {
            FaceDetection::FaceDetection()
                : mProcessorType(model::av::AVTypes::CPU)
            {
            }

            std::string FaceDetection::getProcessorType() const
            {
                return mProcessorType;
            }

            void FaceDetection::setProcessorType(const std::string &type)
            {
                mProcessorType = type;
                notify(events::AlgorithmParametersModified(model::av::AVTypes::FACE_DETECTION));
            }
        }
    }
}
