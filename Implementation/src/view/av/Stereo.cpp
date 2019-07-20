#include <src/view/av/Stereo.hpp>
#include <src/view/av/stereo/Main.hpp>

namespace src
{
    namespace view
    {
       namespace av
       {
           Stereo::Stereo()
           {
           }

           void Stereo::runCPUThread()
           {
               cv::Mat capturedFrameLeft, capturedFrame2Right;
               unsigned t1, t2;
               double time;
               av::stereo::Main* stereoMain = new av::stereo::Main();
               stereoMain->init();

               while (mIsRunningThread)
               {
                   mMutex.lock();

                   getCapturedImage().copyTo(capturedFrameLeft);
                   getCapturedImageStereo().copyTo(capturedFrame2Right);

                   t1 = clock();
                   ///ALGORITHM
                   t2 = clock();

                   capturedFrameLeft.copyTo(mImage);

                   updateBenchmark(t1, t2);
                   mMutex.unlock();
               }
           }

           void Stereo::runGPUThread()
           {

           }
       }
    }
}
