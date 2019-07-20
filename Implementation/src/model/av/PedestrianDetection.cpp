#include <src/model/av/PedestrianDetection.hpp>
#include <src/model/av/AVTypes.hpp>

namespace src
{
    namespace model
    {
        namespace av
        {
            PedestrianDetection::PedestrianDetection()
                :mProcessorType(AVTypes::CPU)
            {
            }

            std::string PedestrianDetection::getProcessorType() const
            {
                return mProcessorType;
            }

            void PedestrianDetection::setProcessorType(const std::string &type)
            {
                mProcessorType = type;
                notify(events::AlgorithmParametersModified(model::av::AVTypes::PEDESTRIAN_DETECTOR));
            }
        }
    }
}
