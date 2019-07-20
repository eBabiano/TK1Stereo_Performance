
#include <src/view/av/AVRenderManager.hpp>

#include <src/view/av/FaceDetection.hpp>
#include <src/view/av/BackgroundSubtractor.hpp>
#include <src/view/av/OpticalFlow.hpp>
#include <src/view/av/PedestrianDetection.hpp>
#include <src/view/av/Stereo.hpp>

#include <src/model/av/AVTypes.hpp>

namespace src
{
    namespace view
    {
       namespace av
       {
           AVRenderManager::AVRenderManager(model::av::AVManager &avManager,
                                            controller::UpdateBenchmarkController& updateBenchmarkController)
               : mAVManager(&avManager)
               , mImageForRender(cv::Mat::zeros(cv::Size(640, 480), CV_8UC3))
               , mSelectedAV(avManager.getSelectedType())
           {
               mAVViews[model::av::AVTypes::FACE_DETECTION] = new view::av::FaceDetection();
               mAVViews[model::av::AVTypes::BACKGROUND_SUBTRACTOR] = new view::av::BackgroundSubtractor();
               mAVViews[model::av::AVTypes::OPTICAL_FLOW] = new view::av::OpticalFlow();
               mAVViews[model::av::AVTypes::PEDESTRIAN_DETECTOR] = new view::av::PedestrianDetection();
               mAVViews[model::av::AVTypes::STEREO] = new view::av::Stereo();

               mAVManager->src::util::Observable<model::av::events::AVStarted>::attach(*this);

               mAVManager->getAlgorithm(model::av::AVTypes::FACE_DETECTION)->
                       src::util::Observable<model::av::events::AlgorithmParametersModified>::attach(*this);
               mAVManager->getAlgorithm(model::av::AVTypes::BACKGROUND_SUBTRACTOR)->
                       src::util::Observable<model::av::events::AlgorithmParametersModified>::attach(*this);
               mAVManager->getAlgorithm(model::av::AVTypes::OPTICAL_FLOW)->
                       src::util::Observable<model::av::events::AlgorithmParametersModified>::attach(*this);
               mAVManager->getAlgorithm(model::av::AVTypes::PEDESTRIAN_DETECTOR)->
                       src::util::Observable<model::av::events::AlgorithmParametersModified>::attach(*this);
               mAVManager->getAlgorithm(model::av::AVTypes::STEREO)->
                       src::util::Observable<model::av::events::AlgorithmParametersModified>::attach(*this);

               for (auto& view : mAVViews)
               {
                   view.second->src::util::Observable<view::av::events::UpdateBenchmarkEvent>::attach(updateBenchmarkController);
               }
           }

           AVRenderManager::~AVRenderManager()
           {
               for (auto& av : mAVViews)
               {
                   delete av.second;
               }
           }

           void AVRenderManager::observableUpdated(const model::av::events::AVStarted &event)
           {
               mSelectedAV = event.getType();
               if (event.getIsActivated())
               {
                   mAVViews.at(mSelectedAV)->start();
               }
               else
               {
                   mAVViews.at(mSelectedAV)->stop();
               }
           }

           void AVRenderManager::observableUpdated(const model::av::events::AlgorithmParametersModified &event)
           {
               if (event.getType() == model::av::AVTypes::BACKGROUND_SUBTRACTOR)
               {
                   int learningRate = mAVManager->getAlgorithm(mAVManager->getSelectedType())->getLearningRate();
                   mAVViews.at(mAVManager->getSelectedType())->setLearningRate((double)learningRate / 100.0);
               }
               else if (event.getType() == model::av::AVTypes::PEDESTRIAN_DETECTOR)
               {
                   std::string processorType = mAVManager->getAlgorithm(mAVManager->getSelectedType())->getProcessorType();
                   if (processorType == model::av::AVTypes::CPU)
                   {
                       mAVViews.at(mAVManager->getSelectedType())->activateGPU(false);
                   }
                   else
                   {
                       mAVViews.at(mAVManager->getSelectedType())->activateGPU(true);
                   }
               }
               else if (event.getType() == model::av::AVTypes::FACE_DETECTION)
               {
                   std::string processorType = mAVManager->getAlgorithm(mAVManager->getSelectedType())->getProcessorType();
                   if (processorType == model::av::AVTypes::CPU)
                   {
                       mAVViews.at(mAVManager->getSelectedType())->activateGPU(false);
                   }
                   else
                   {
                       mAVViews.at(mAVManager->getSelectedType())->activateGPU(true);
                   }
               }
               else if (event.getType() == model::av::AVTypes::OPTICAL_FLOW)
               {

               }

           }

           cv::Mat AVRenderManager::getImageForRender()
           {
               cv::Mat image;
               mImageForRender.copyTo(image);
               mImageForRender = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);
               return image;
           }

           void AVRenderManager::sumAllAVImages()
           {
               if (!mAVViews.at(mSelectedAV)->getImage().empty())
               {
                   if (mImageForRender.channels() == mAVViews.at(mSelectedAV)->getImage().channels())
                   {
                       mImageForRender = mImageForRender + mAVViews.at(mSelectedAV)->getImage();
                   }
                   else
                   {
                       mImageForRender = mAVViews.at(mSelectedAV)->getImage();
                   }
               }
           }

           void AVRenderManager::setCaptureImage(cv::Mat mat)
           {
               mAVViews.at(mSelectedAV)->setCapturedImage(mat);
           }

           void AVRenderManager::setCaptureImageStereo(cv::Mat mat)
           {
               mAVViews.at(mSelectedAV)->setCapturedImageStereo(mat);
           }

           void AVRenderManager::setTotalFrames(int value)
           {
               mAVViews.at(mSelectedAV)->setTotalFrames(value);
           }

       }
    }
}
