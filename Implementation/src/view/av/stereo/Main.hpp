
#include <src/view/av/stereo/Structs.hpp>
#include <src/view/av/stereo/Stereo.hpp>
#include <src/view/av/stereo/Config.hpp>

namespace src
{
    namespace view
    {
       namespace av
       {
           namespace stereo
           {
               class Main
               {
                   public:
                        Main();

                        void init(/*int argc, char *argv[]*/);
                        void loopAlgorithm();
                        void end();

                   private:
                        // Variables.
                        Structs::t_Processing processing_data; // Structure with all variables that will be used for the processing.
                        Structs::t_Videos videos; // Structure with video path and pointers.
                        Structs::t_Socket coms_struct;
                        Structs::t_Semaphores sem_struct;
//                        Camera** ppCameras = new Camera*[2];
                        bool retVal;
//                        Error error;
//                        Image rawImage[2];
                        unsigned int rowBytes;
                        cv::Mat aux;
                        bool adjusting = false;
                        bool sh_chng=false;
                        bool shutdown_at_exit = false;
//                        TriggerMode triggerMode;
                        double proj_map_t = 0.0, disp_map_t = 0.0, xz_t = 0.0, cand_t = 0.0,
                        track_t = 0.0, class_t = 0.0, overlay_t = 0.0, show_t = 0.0, loop_t = 0.0;
                        bool shutdown = false;
//                        CpuTime tloop;
//                        CpuTime t;
                        time_t hora;
                        char the_date[50];
                        char nombreImagen[100];
                        int index = 1;
                        int fd, sercmd, serstat;
                        int counter_flag_sign = 0;
                        char key = ' ';
                        int count=0;
//                        TimeStamp timestamp;
                        double t00=0;
                        double t01=0;
                        uint Time_index=0;
                        cv::Mat times = cv::Mat(POINTS_DIRECTION,1,CV_64FC1);
                        char c = 'i', cadena[20];
                        // End variables.

                        Config* mConfig;
                        Stereo* mStereo;

                        void variableInitialization();
                        void readConfigsFiles(int argc, char *argv[]);
                        void stereoInitialization();
                        void mallocMemoryForImages();
                        void videoModeEnabled();
                        void getImagesFromCameraOrVideos();
                        void initBackground_Disparity3D_Extrinsic();
                        void processingAreas();
                        void proximityMatrix_SWTrigger();
                        void closeVideos();
                        void closeAndShutdown();

                        void grabFromCameras();

               };
           }
       }
    }
}
