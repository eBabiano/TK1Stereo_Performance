#ifndef STEREO_HPP
#define STEREO_HPP

#include <src/view/av/AVView.hpp>

namespace src
{
    namespace view
    {
       namespace av
       {
           class Stereo
                   : public AVView
           {
               public:
                   Stereo();

                   virtual void runCPUThread();
                   virtual void runGPUThread();
           };
       }
    }
}

#endif // STEREO_HPP
