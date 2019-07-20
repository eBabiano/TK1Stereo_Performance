#ifndef OPTICALFLOW_HPP
#define OPTICALFLOW_HPP

#include <src/view/av/AVView.hpp>

namespace src
{
    namespace view
    {
       namespace av
       {
           class OpticalFlow
                   : public AVView
           {
               public:
                   OpticalFlow();

                   virtual void runCPUThread();
                   virtual void runGPUThread();
           };
       }
    }
}

#endif // OPTICALFLOW_HPP
