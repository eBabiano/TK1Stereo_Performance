#ifndef PEDESTRIANDETECTION_HPP
#define PEDESTRIANDETECTION_HPP

#include <src/view/av/AVView.hpp>

namespace src
{
    namespace view
    {
       namespace av
       {
           class PedestrianDetection
                   : public AVView
           {
               public:
                   PedestrianDetection();

                   virtual void runCPUThread();
                   virtual void runGPUThread();
           };
       }
    }
}

#endif // PEDESTRIANDETECTION_HPP
