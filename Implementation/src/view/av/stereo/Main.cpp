/**
 * @file main.cpp
 * @author  Raúl Quintero Mínguez <raul.quintero@aut.uah.es>, David Fernández LLorca <llorca@aut.uah.es>. ISIS Research Group. University of Alcalá.
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * Project: SmartCross
 * 
 * This file contains the main function.
 * 
 */

#include <src/view/av/stereo/Main.hpp>

namespace src
{
    namespace view
    {
       namespace av
       {
           namespace stereo
           {
               Main::Main()
               {

               }

               /**
                    * Main function.
                   @param IN argc: number of arguments from command line.
                   @param IN argv: arguments from command line.

                   @return int value is returned. 0 if the application finish correctly.
                    **/
               void Main::init(/*int argc, char *argv[]*/)
               {

                   mConfig = new Config();
                   mStereo = new Stereo();

                   int argc = 0;
                   char *argv[20];

                   variableInitialization();
                   readConfigsFiles(argc, argv);

                   stereoInitialization();
                   mallocMemoryForImages();

                   videoModeEnabled();
                   getImagesFromCameraOrVideos();

                   initBackground_Disparity3D_Extrinsic();
                   processingAreas();
                   proximityMatrix_SWTrigger();
               }



               void Main::variableInitialization()
               {
                   ///setenv("LC_NUMERIC", "C", 1);

                   // Initialization of object ID, frame and flag_sign.
                   processing_data.nident = 1;
                   processing_data.frame = 0;
                   processing_data.flag_sign = 0;

                   // Initialization of counters.
                   processing_data.num_pedestrians = 0;
                   processing_data.num_vehicles = 0;
                   processing_data.num_pedestrians12 = 0;
                   processing_data.num_pedestrians21 = 0;


                   // Clears vectors.
                   processing_data.objects.clear();
                   processing_data.candidates.clear();
               }

               void Main::readConfigsFiles(int argc, char *argv[])
               {
                   // Reads the configuration file.
                   if (mConfig->ReadConfFile(&processing_data.config_file, &processing_data.limits,
                                            &processing_data.num_waiting_areas, &processing_data.num_crossing_areas) == -1) {
                       printf("The application hasn't read the configuration file.\n");
                       exit(-1);
                   }

                   if (mConfig->cmdOptionExists(argv, argv + argc, "--debug")) {
                       processing_data.config_file.visualization = 1;
                   }
                   if (mConfig->cmdOptionExists(argv, argv + argc, "--save")) {
                       processing_data.config_file.save = 1;
                   }

                   if (mConfig->cmdOptionExists(argv, argv + argc, "--areas")) {
                       processing_data.config_file.areas = 1;
                   }

                   if (mConfig->cmdOptionExists(argv, argv + argc, "--vid")) {
                       processing_data.config_file.mode =0;
                   }
                   if (mConfig->cmdOptionExists(argv, argv + argc, "--v_left")) {
                       char *sh = mConfig->getCmdOption(argv, argv+argc, "--v_left");
                       printf("%s\n",sh);
                       strcpy(processing_data.config_file.video_left,sh);
                       processing_data.config_file.mode =0;
                   }

                   if (mConfig->cmdOptionExists(argv, argv + argc, "--v_right")) {
                       char *sh = mConfig->getCmdOption(argv, argv+argc, "--v_right");
                       printf("%s\n",sh);

                       strcpy(processing_data.config_file.video_right,sh);
                       processing_data.config_file.mode =0;
                   }
               }

               void Main::stereoInitialization()
               {
                   // Stereo initialization. SGBM
                   if (mStereo->InitStereo(processing_data.config_file, &processing_data.map_matrices, &processing_data.calib_params,
                                          &processing_data.sgbm_state, &processing_data.bm_state) == -1) {
                       printf("The application hasn't computed the stereo.\n");
                       exit(-1);
                   }
               }

               void Main::mallocMemoryForImages()
               {
                   // Allocates memory for images.
                   ///@todo Search the dependencie
                   /// @author emilio
                   ///AllocImages(processing_data.calib_params, &processing_data.images_stereo);
               }

               void Main::videoModeEnabled()
               {
                   // If video mode is enable then videos are opened.
                   ///@brief This function enable the video captures and cofigure it
                   /// @author emilio
//                   if (processing_data.config_file.mode == 0) {
//                       // Opens videos.
//                       if (OpenVideos(processing_data.config_file, &videos) == -1) {
//                           printf("The application hasn't opened any video.\n");
//                           PrintLog("The application hasn't opened any video", LOG_ERROR);
//                           exit(-1);
//                       }
//                   }// If camera is selected then the application will try to open the capture.
//                   else {
//                       PrintLog("Configuring cameras.", LOG_INFO);
//                       configureCameras(ppCameras);
//                       // Check that the trigger is ready.
//                       PrintLog("Waiting for cameras.", LOG_INFO);
//                       PollForTriggerReady(ppCameras[IZQ]);
//                       PollForTriggerReady(ppCameras[DER]);
//                       SetCameraBrightness(ppCameras[IZQ], &processing_data.config_file.brightness);
//                       SetCameraBrightness(ppCameras[DER], &processing_data.config_file.brightness);
//                       SetCameraShutter(ppCameras[DER], &processing_data.config_file.shutter);
//                       SetCameraShutter(ppCameras[IZQ], &processing_data.config_file.shutter);
//                       SetCameraExposure(ppCameras[IZQ], &processing_data.config_file.exposure);
//                       SetCameraExposure(ppCameras[DER], &processing_data.config_file.exposure);
//                       SetCameraGain(ppCameras[IZQ], &processing_data.config_file.gain);
//                       SetCameraGain(ppCameras[DER], &processing_data.config_file.gain);
//                   }
               }

               void Main::getImagesFromCameraOrVideos()
               {
                   // Gets images from camera or videos.
                   ///**********************************************************************************

//                   if (processing_data.config_file.mode == 0) {
//                       // Read images from the videos.
//                       if (GrabFromVideos(processing_data.config_file, &videos, &processing_data.images_stereo, &processing_data.diff_time) == -1) {
//                           PrintLog("Error grabbing from video.", LOG_ERROR);
//                           exit(-1);
//                       }
//                   } else {
//                       // Fire software trigger
//                       retVal = FireSoftwareTrigger(ppCameras[DER]);
//                       if (SIN_CABLE_SINC)
//                           retVal = FireSoftwareTrigger(ppCameras[IZQ]);

//                       // Grab image
//                       error = ppCameras[DER]->RetrieveBuffer(&rawImage[DER]);
//                       if (error != PGRERROR_OK) {
//                           PrintError(error);
//                           PrintLog("Right Camera timeout.", LOG_CRITICAL);
//                           exit(-1);
//                       }
//                       error = ppCameras[IZQ]->RetrieveBuffer(&rawImage[IZQ]);
//                       if (error != PGRERROR_OK) {
//                           PrintError(error);
//                           PrintLog("Left Camera timeout.", LOG_CRITICAL);
//                           exit(-1);
//                       }
//                       rowBytes = (double) rawImage[IZQ].GetReceivedDataSize() / (double) rawImage[IZQ].GetRows();
//                       processing_data.images_stereo.left_image_raw.data = rawImage[IZQ].GetData();
//                       cv::cvtColor(processing_data.images_stereo.left_image_raw, aux, CV_BayerBG2RGB);
//                       cv::cvtColor(aux, processing_data.images_stereo.left_image_raw, CV_RGB2GRAY);
//                       cv::resize(processing_data.images_stereo.left_image_raw, processing_data.images_stereo.left_image,
//                                  cv::Size(processing_data.images_stereo.left_image.cols, processing_data.images_stereo.left_image.rows), 0, 0, CV_INTER_LINEAR);

//                       processing_data.images_stereo.right_image_raw.data = rawImage[DER].GetData();
//                       cv::cvtColor(processing_data.images_stereo.right_image_raw, aux, CV_BayerBG2RGB);
//                       cv::cvtColor(aux, processing_data.images_stereo.right_image_raw, CV_RGB2GRAY);
//                       cv::resize(processing_data.images_stereo.right_image_raw, processing_data.images_stereo.right_image,
//                                  cv::Size(processing_data.images_stereo.right_image.cols, processing_data.images_stereo.right_image.rows), 0, 0, CV_INTER_LINEAR);
//                   }
                   ///**********************************************************************************

               }

               void Main::initBackground_Disparity3D_Extrinsic()
               {
                   // Initialization of background subtraction.
                   processing_data.bg = cv::BackgroundSubtractorMOG();

                   // Gets disparity map and 3D reconstruction.
                   mStereo->GetDisparityMapAnd3D(&processing_data.map_matrices, processing_data.config_file,
                                                 &processing_data.sgbm_state, &processing_data.bm_state,
                                                 &processing_data.images_stereo);


                   // Adjusts extrinsic parameters and projects from camera reference to ground reference.
                   mStereo->AdjustExtrinsicParams(processing_data.config_file.computepitch, &processing_data.calib_params, &processing_data.images_stereo);
               }

               void Main::processingAreas()
               {
                   // The pedestrian areas are defined by the user. If previous areas exist the file will be read, else the user will define new areas.
                   ///***************************************************************************
//                   if ((processing_data.pf_areas = fopen(processing_data.config_file.areas_file, "r")) == NULL || processing_data.config_file.areas == 1) {
//                       DefinePedestrianAreasAndLimits(processing_data.num_waiting_areas, processing_data.num_crossing_areas, &processing_data.limits, &processing_data.waiting_areas, &processing_data.crossing_areas, &processing_data.images_stereo);
//                       if (SaveAreas(processing_data.config_file.areas_file, processing_data.num_waiting_areas, processing_data.num_crossing_areas, &processing_data.waiting_areas, &processing_data.crossing_areas, &processing_data.images_stereo.areas) == -1) {
//                           printf("The application hasn't saved the areas file");
//                           PrintLog("The application hasn't saved the areas file", LOG_ERROR);
//                           exit(-1);
//                       }
//                   } else {
//                       if (ReadAreas(processing_data.config_file.areas_file, &processing_data.num_waiting_areas, &processing_data.num_crossing_areas, &processing_data.waiting_areas, &processing_data.crossing_areas, &processing_data.images_stereo.areas, &processing_data.limits) == -1) {
//                           printf("The application hasn't read the areas file");
//                           PrintLog("The application hasn't read the areas file", LOG_ERROR);
//                           exit(-1);
//                       }
//                   }
                   ///***************************************************************************
               }

               void Main::proximityMatrix_SWTrigger()
               {
                   // Get proximity matrices.
                   ///***************************************************************************

//                   PrintLog("Proximity areas.", LOG_INFO);
//                   ProximityToAreasMatrix(&processing_data.waiting_areas);

//                   // Creates the 3D reconstruction of the scene in order to check the stereo information and reference.
//                   if (processing_data.config_file.point_cloud == 1) {
//                       /*      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>); // Point cloud for reconstructing the scene.
//                           boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer")); // Point cloud viewer.
//                           CreatePointCloud(&processing_data.images_stereo, cloud);
//                           ShowPointCloud(cloud, viewer);*/
//                   }

//                   // Fire software trigger
//                   PrintLog("Fire cameras.", LOG_INFO);
//                   if (processing_data.config_file.mode == 1) {
//                       // Check that the trigger is ready
//                       PollForTriggerReady(ppCameras[IZQ]);
//                       PollForTriggerReady(ppCameras[DER]);
//                       retVal = FireSoftwareTrigger(ppCameras[DER]);
//                       if (SIN_CABLE_SINC)
//                           retVal = FireSoftwareTrigger(ppCameras[IZQ]);

//                   }
                   ///***************************************************************************

               }

               void Main::loopAlgorithm()
               {
                   //Processing loop.
                   while (!shutdown && key != 'q') {
                       ///***************************************************************************

//                       // Gets images from camera or videos.
                       if (processing_data.config_file.mode == 0) {
//                           // Read images from the videos.
//                           if (GrabFromVideos(processing_data.config_file, &videos, &processing_data.images_stereo, &processing_data.diff_time) == -1) {
//                               break;
//                           }

                       } else {
                           grabFromCameras();
                       }
                       ///***************************************************************************

                       // Gets disparity map, matrix 3D.
                       ///**********************************************************************************
                       mStereo->GetDisparityMapAnd3D(&processing_data.map_matrices, processing_data.config_file,
                                                     &processing_data.sgbm_state, &processing_data.bm_state,
                                                     &processing_data.images_stereo);
                       ///**********************************************************************************

                       // Processes the background subtraction.
                       ///**********************************************************************************
                       if(processing_data.frame < 100 || sh_chng)
                           count =100;
                       if(count>0)
                       {
                           // boost::thread th(BackgroundSubtractionFilter, &processing_data.bg, processing_data.images_stereo.undist_left_image,
                           // 		 &processing_data.images_stereo.back, &processing_data.images_stereo.vdisp_image,0.01);
                          /// BackgroundSubtractionFilter( &processing_data.bg, processing_data.images_stereo.undist_left_image,
                          ///                              &processing_data.images_stereo.back, &processing_data.images_stereo.vdisp_image,/*0.01*/0.1);
                           count--;

                       }
                       else{
                           // boost::thread th(BackgroundSubtractionFilter, &processing_data.bg, processing_data.images_stereo.undist_left_image,
                           // 		 &processing_data.images_stereo.back, &processing_data.images_stereo.vdisp_image,0.0002);
                           ///BackgroundSubtractionFilter( &processing_data.bg, processing_data.images_stereo.undist_left_image,
                           ///                             &processing_data.images_stereo.back, &processing_data.images_stereo.vdisp_image, /*0.0002*/0.1);

                       }


                       // Projects points 3D from camera reference to ground reference.
                       mStereo->ProjectToGround(&processing_data.calib_params, &processing_data.images_stereo);
                       ///**********************************************************************************

                       ///**********************************************************************************
                       // Gets X-Z map and filter it.
                       mStereo->GetImageXZ(processing_data.limits, &processing_data.images_stereo);
                       ///**********************************************************************************

                       // Gets candidates to be pedestrian or vehicle.
                       ///**********************************************************************************
//                       GetCandidates(processing_data.limits, &processing_data.images_stereo, &processing_data.candidates);
                       ///**********************************************************************************

                       // An object tracking is performed.
                       ///**********************************************************************************
//                       if (ObjectTracking(&processing_data.candidates, &processing_data.objects, &processing_data.nident) == -1) {
//                           printf("Error in tracking function.\n");
//                           PrintLog("Error in tracking function", LOG_ERROR);
//                           exit(-1);
//                       }
                       ///**********************************************************************************

                       // Classifies object by size.
                       ///**********************************************************************************
//                       ClassifyObjects(processing_data.limits, &processing_data.images_stereo, &processing_data.objects, &processing_data.waiting_areas, &processing_data.crossing_areas, processing_data.diff_time);
                       ///**********************************************************************************

                       // Print overlay.
                       ///**********************************************************************************
//                       processing_data.flag_sign = Overlay(&sem_struct, &coms_struct, &processing_data.images_stereo, &processing_data.objects, &processing_data.peatones, &processing_data.waiting_areas, &processing_data.crossing_areas, processing_data.config_file.visualization, processing_data.config_file.save);
//                       if (processing_data.flag_sign == 0 && counter_flag_sign != 0) {
//                           processing_data.flag_sign = 1;
//                           counter_flag_sign--;
//                       } else if (processing_data.flag_sign == 1)
//                           counter_flag_sign = COUNTER_FLAG_SIGN;
                       ///**********************************************************************************

                       processing_data.frame++;

                       // Shows images.
                       ///**********************************************************************************
//                       if (processing_data.config_file.visualization == 1) {
//                           t.start();
//                           ShowImages(processing_data.images_stereo);
//                           t.stop();
//                           printf("Show time %s\n", t.getText().data());
//                           show_t += t.getTotal();

//                           t.start();
//                           key = cv::waitKey(1);
//                           t.stop();
//                           printf("Wait time %s\n", t.getText().data());
//                           if(key=='c')
//                               count=100;
//                       }
//                       if (processing_data.config_file.save == 1) {
//                           cv::Mat vdisp_color;
//                           cv::cvtColor(processing_data.images_stereo.vdisp_image, vdisp_color, CV_GRAY2RGB);

//                           /*	    cv::Size sz1 = processing_data.images_stereo.img_objects_color.size();
//                           cv::Mat imResult(sz1.height, sz1.width*3, CV_8UC3);
//                           cv::Mat left(imResult, cv::Rect(0, 0, sz1.width, sz1.height));
//                           processing_data.images_stereo.img_objects_color.copyTo(left);
//                           cv::Mat center(imResult, cv::Rect(sz1.width, 0, sz1.width, sz1.height));
//                           processing_data.images_stereo.undist_left_image_color.copyTo(center);
//                           cv::Mat right(imResult, cv::Rect(2*sz1.width, 0, sz1.width, sz1.height));
//                           vdisp_color.copyTo(right);

//                               sprintf(nombreImagen, "/media/data1/%08d.png", index++);
//                               cv::imwrite(nombreImagen, imResult);
//                     */

//                           sprintf(nombreImagen, "./cam0/cam0_%08d.png", index);
//                           cv::imwrite(nombreImagen, processing_data.images_stereo.left_image_raw);
//                           sprintf(nombreImagen, "./cam1/cam1_%08d.png", index++);
//                           cv::imwrite(nombreImagen, processing_data.images_stereo.right_image_raw);

//                           /*sprintf(nombreImagen, "/media/data1/objects_%08d.png", index);
//                                 cv::imwrite(nombreImagen, processing_data.images_stereo.img_objects_color);
//                                 sprintf(nombreImagen, "/media/data2/left_%08d.png", index++);
//                                 cv::imwrite(nombreImagen, processing_data.images_stereo.undist_left_image_color);
//                                 sprintf(nombreImagen, "/media/data2/disp_%08d.png", index++);
//                                 cv::imwrite(nombreImagen, processing_data.images_stereo.vdisp_image);*/
//                       }


//                       //	PrintLog("Saving log", LOG_INFO);
//                       SaveLogByHour(&processing_data.objects, &processing_data.waiting_areas, &processing_data.num_pedestrians, &processing_data.num_vehicles, &processing_data.num_pedestrians12, &processing_data.num_pedestrians21);

//                       tloop.stop();
//                       printf("Loop time %s\n\n\n", tloop.getText().data());
//                       loop_t += tloop.getTotal();
                       ///**********************************************************************************

                   }
               }

               void Main::end()
               {
                   closeVideos();
                   closeAndShutdown();
               }

               void Main::closeVideos()
               {
//                   if (processing_data.config_file.mode == 0) { // Closes videos.

//                       PrintLog("Closing videos", LOG_INFO);
//                       CloseVideos(&videos);
//                       printf("---------------------------\n");
//                       printf("End video\n");
//                       printf("---------------------------\n");
//                   } else { // Closes capture
//                       PrintLog("Closing cameras", LOG_INFO);

//                       // Turn trigger mode off.
//                       triggerMode.onOff = false;

//                       error = ppCameras[DER]->SetTriggerMode(&triggerMode);
//                       if (error != PGRERROR_OK) {
//                           PrintError(error);
//                           exit(-1);
//                       }

//                       triggerMode.onOff = false;
//                       error = ppCameras[IZQ]->SetTriggerMode(&triggerMode);
//                       if (error != PGRERROR_OK) {
//                           PrintError(error);
//                           exit(-1);
//                       }
//                       printf("\nFinished grabbing images\n");

//                       // Stop capturing images
//                       error = ppCameras[DER]->StopCapture();
//                       if (error != PGRERROR_OK) {
//                           PrintError(error);
//                           exit(-1);
//                       }
//                       error = ppCameras[IZQ]->StopCapture();
//                       if (error != PGRERROR_OK) {
//                           PrintError(error);
//                           exit(-1);
//                       }


//                       // Disconnect the camera
//                       error = ppCameras[DER]->Disconnect();
//                       if (error != PGRERROR_OK) {
//                           PrintError(error);
//                           exit(-1);
//                       }
//                       error = ppCameras[IZQ]->Disconnect();
//                       if (error != PGRERROR_OK) {
//                           PrintError(error);
//                           exit(-1);
//                       }
//                   }
               }

               void Main::closeAndShutdown()
               {
                   if (ACTIVAR_PANEL) {
                       ioctl(fd, TIOCMBIC, &sercmd); // Reset the RTS pin.
                       close(fd);
                   }

                   // If comunication mode is enable then socket is closed.
                   if(processing_data.config_file.comunication == 1)
                   {
                       close(coms_struct.socket);
                   }

                   // If semaphore mode is enable then semaphores are shut down.
                   if(processing_data.config_file.semaphore == 1){
                       c = 'o';
                       int n0 = sprintf(cadena, "%c\n", c);
                       n0 = write(sem_struct.fd0, cadena, n0);
                       int n1 = sprintf(cadena, "%c\n", c);
                       n1 = write(sem_struct.fd1, cadena, n1);
                   }






//                   PrintLog("Salvando ficheros para el servidor en el programa", LOG_INFO);
                   // Saves the Statistics files for the web server
                   system("cp /tmp/hourly_ped.txt /home/smartcross/Programas/SmartCross/SmartCross2.0/Statistics/hourly_ped_yest.txt ");
                   system("cp /tmp/hourly_veh.txt /home/smartcross/Programas/SmartCross/SmartCross2.0/Statistics/hourly_veh_yest.txt");
                   system("cp /tmp/weekly*.txt /home/smartcross/Programas/SmartCross/SmartCross2.0/Statistics/");

                   //Creates csv files
                   system("/home/smartcross/Programas/SmartCross/SmartCross2.0/Statistics/./CreaCSV.sh");

                   // Shutdown computer
                   if (shutdown_at_exit && shutdown) {
//                       PrintLog("Salida por shutdown ", LOG_INFO);
                       system("sudo poweroff");
                   }

//                   PrintLog("Normal Exit", LOG_INFO);
               }

               void Main::grabFromCameras()
               {
                   // Grab image
//                   error = ppCameras[DER]->RetrieveBuffer(&rawImage[DER]);
//                   if (error != PGRERROR_OK) {
//                       PrintError(error);
//                       PrintLog("Right Camera timeout.", LOG_CRITICAL);
//                       //exit(-1);
//                   }
//                   error = ppCameras[IZQ]->RetrieveBuffer(&rawImage[IZQ]);
//                   if (error != PGRERROR_OK) {
//                       PrintError(error);
//                       PrintLog("Left Camera timeout.", LOG_CRITICAL);
//                       //exit(-1);
//                   }
//                   timestamp = rawImage[DER].GetTimeStamp();
//                   rowBytes = (double) rawImage[IZQ].GetReceivedDataSize() / (double) rawImage[IZQ].GetRows();
//                   processing_data.images_stereo.left_image_raw.data = rawImage[IZQ].GetData();
//                   cv::cvtColor(processing_data.images_stereo.left_image_raw, aux, CV_BayerBG2RGB);
//                   cv::cvtColor(aux, processing_data.images_stereo.left_image_raw, CV_RGB2GRAY);
//                   cv::resize(processing_data.images_stereo.left_image_raw, processing_data.images_stereo.left_image, cv::Size(processing_data.images_stereo.left_image.cols, processing_data.images_stereo.left_image.rows), 0, 0, CV_INTER_LINEAR);

//                   processing_data.images_stereo.right_image_raw.data = rawImage[DER].GetData();
//                   cv::cvtColor(processing_data.images_stereo.right_image_raw, aux, CV_BayerBG2RGB);
//                   cv::cvtColor(aux, processing_data.images_stereo.right_image_raw, CV_RGB2GRAY);
//                   cv::resize(processing_data.images_stereo.right_image_raw, processing_data.images_stereo.right_image, cv::Size(processing_data.images_stereo.right_image.cols, processing_data.images_stereo.right_image.rows), 0, 0, CV_INTER_LINEAR);

//                   ///////////// Cálculo tiempo medio últimas N(POINTS_DIRECTION) capturas /////

//                   t00 = t01;
//                   t01 = (double)timestamp.cycleSeconds + (((double)timestamp.cycleCount+((double)timestamp.cycleOffset/3072.0))/8000.0);

//                   times.at<double>(Time_index%POINTS_DIRECTION)=((double)(t01-t00));
//                   processing_data.diff_time = cv::sum(times).val[0];
//                   Time_index++;

//                   // Check that the trigger is ready
//                   PollForTriggerReady(ppCameras[IZQ]);
//                   PollForTriggerReady(ppCameras[DER]);

//                   // Fire software trigger
//                   retVal = FireSoftwareTrigger(ppCameras[DER]);
//                   if (SIN_CABLE_SINC)
//                       retVal = FireSoftwareTrigger(ppCameras[IZQ]);

//                   // Check shutter adjustment.
//                   if (((processing_data.frame % SHUTTER_ADJ_FRAMES) == 0) || adjusting) {
//                       shutdown = ajusteShutter(processing_data.images_stereo.left_image, ppCameras, &adjusting, &sh_chng);
//                   }
               }

           }
       }
    }
}


