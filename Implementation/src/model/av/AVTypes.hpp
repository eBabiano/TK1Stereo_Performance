#ifndef AVTYPES_HPP
#define AVTYPES_HPP

#include <string>

namespace src
{
    namespace model
    {
        namespace av
        {
            class AVTypes
            {
                public:
                    static const std::string FACE_DETECTION;
                    static const std::string BACKGROUND_SUBTRACTOR;
                    static const std::string OPTICAL_FLOW;
                    static const std::string PEDESTRIAN_DETECTOR;
                    static const std::string STEREO;

                    static const std::string GPU;
                    static const std::string CPU;

            };
        }
    }
}

#endif // AVTYPES_HPP
