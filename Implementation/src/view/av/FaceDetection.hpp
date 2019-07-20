#ifndef FACEDETECTION_HPP
#define FACEDETECTION_HPP

#include <src/view/av/AVView.hpp>

namespace src
{
    namespace view
    {
       namespace av
       {
           class FaceDetection
                   : public AVView
           {
               public:
                   FaceDetection();

                   virtual void runCPUThread();
                   virtual void runGPUThread();

           };
       }
    }
}

#endif // FACEDETECTION_HPP
