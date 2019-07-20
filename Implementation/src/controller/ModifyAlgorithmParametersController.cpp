#include <src/controller/ModifyAlgorithmParametersController.hpp>
#include <src/model/av/AVTypes.hpp>

namespace src
{
    namespace controller
    {
        ModifyAlgorithmParametersController::ModifyAlgorithmParametersController(
                model::av::AVManager &avManager)
            : mAVManager(&avManager)
        {
        }

        void ModifyAlgorithmParametersController::observableUpdated(const view::gui::events::ModifyBackgroundSubstractorEvent &event)
        {
            mAVManager->getAlgorithm(model::av::AVTypes::BACKGROUND_SUBTRACTOR)->setLearningRate(event.getLearningRate());
        }

        void ModifyAlgorithmParametersController::observableUpdated(const view::gui::events::ModifyFaceDetectionEvent &event)
        {
            mAVManager->getAlgorithm(model::av::AVTypes::FACE_DETECTION)->setProcessorType(event.getProcessorType());
        }

        void ModifyAlgorithmParametersController::observableUpdated(const view::gui::events::ModifyOpticalFlowEvent &event)
        {

        }

        void ModifyAlgorithmParametersController::observableUpdated(const view::gui::events::ModifyPedestrianDetectorEvent &event)
        {
            mAVManager->getAlgorithm(model::av::AVTypes::PEDESTRIAN_DETECTOR)->setProcessorType(event.getProcessorType());
        }
    }
}
