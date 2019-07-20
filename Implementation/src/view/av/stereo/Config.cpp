/**
 * @file config.cpp
 * @author  Raúl Quintero Mínguez <raul.quintero@aut.uah.es>, David Fernández LLorca <llorca@aut.uah.es>. ISIS Research Group. University of Alcalá.
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * Project: SmartCross
 * 
 * This file contains functions to read the configuration file.
 * 
 */

#include <src/view/av/stereo/Config.hpp>
namespace src
{
    namespace view
    {
       namespace av
       {
           namespace stereo
           {

               /**
                * Reads the information from the configuration file to run the application.
                * @param OUT conf_file: Structure with the parameters that have been read from the file.
                * @param OUT limits: Limits for 3D map.
                * @param OUT num_waiting_areas: Number of waiting areas that have been defined by the user.
                * @param OUT num_crossing_areas: Number of crossing areas that have been defined by the user.
                *
                * @return int value is returned. 0 if all parameters have been read correctly, else -1.
                **/
               int Config::ReadConfFile(Structs::t_ConfigFile *conf_file, Structs::t_Limits *limits, int *waiting_areas, int *crossing_areas) {
                   // Variables.
                   FILE *pf_conf = NULL;
                   int fsr = 0;
                   char height[100];
                   char colour[10];
                   char reduction[10];
                   char mode[10];
                   char limit_y_min[100];
                   char limit_y_max[100];
                   char wa[10];
                   char ca[10];
                   char pc[10];
                   char shutter[100];
                   char gain[100];
                   char brightness[100];
                   char exposure[100];
                   char vis[10];
                   char pitch[10];
                   char temp[100];
                   char save[10];
                   char method[10];
                   char comunication[10];
                   char semaphore[10];
                   // End variables.

                   if ((pf_conf = fopen(CONFIG_FILE_NAME, "r")) == NULL) {
                       printf("Error while loading configuration file\n");
                       return (-1);
                   }
                   fgets(temp,100,pf_conf);
                   fsr = sscanf(temp,"Video_left=%[^\n]", conf_file->video_left);
                   if (fsr == 0) {
                       printf("Error: Left video path has not been read");
                       return (-1);
                   }

                   fgets(temp,100,pf_conf);
                   fsr = sscanf(temp,"Video_right=%[^\n]", conf_file->video_right);
                   if (fsr == 0) {
                       printf("Error: Right video path has not been read");
                       return (-1);
                   }
                   fsr = fscanf(pf_conf, "Calib_file=%s\n", conf_file->calib_file);
                   if (fsr == 0) {
                       printf("Error: Calibration file path has not been read");
                       return (-1);
                   }
                   fsr = fscanf(pf_conf, "Areas_file=%s\n", conf_file->areas_file);
                   if (fsr == 0) {
                       printf("Error: Areas file path has not been read");
                       return (-1);
                   }
                   fsr = fscanf(pf_conf, "Height_camera=%s\n", height);
                   if (fsr == 0) {
                       printf("Error: Height camera value has not been read");
                       return (-1);
                   }
                   fsr = fscanf(pf_conf, "Colour=%s\n", colour);
                   if (fsr == 0) {
                       printf("Error: Colour value has not been read");
                       return (-1);
                   }
                   fsr = fscanf(pf_conf, "Reduction=%s\n", reduction);
                   if (fsr == 0) {
                       printf("Error: Reduction factor value has not been read");
                       return (-1);
                   }
                   fsr = fscanf(pf_conf, "Mode=%s\n", mode);
                   if (fsr == 0) {
                       printf("Error: Mode value has not been read");
                       return (-1);
                   }
                   fsr = fscanf(pf_conf, "Limit_Y_Min=%s\n", limit_y_min);
                   if (fsr == 0) {
                       printf("Error: Limit Y Minimum value has not been read");
                       return (-1);
                   }
                   fsr = fscanf(pf_conf, "Limit_Y_Max=%s\n", limit_y_max);
                   if (fsr == 0) {
                       printf("Error: Limit Y Maximum value has not been read");
                       return (-1);
                   }
                   fsr = fscanf(pf_conf, "Waiting_areas=%s\n", wa);
                   if (fsr == 0) {
                       printf("Error: Waiting areas value has not been read");
                       return (-1);
                   }
                   fsr = fscanf(pf_conf, "Crossing_areas=%s\n", ca);
                   if (fsr == 0) {
                       printf("Error: Crossing areas value has not been read");
                       return (-1);
                   }
                   fsr = fscanf(pf_conf, "Point_cloud=%s\n", pc);
                   if (fsr == 0) {
                       printf("Error: Point cloud value has not been read");
                       return (-1);
                   }
                   fsr = fscanf(pf_conf, "Shutter=%s\n", shutter);
                   if (fsr == 0) {
                       printf("Error: Shutter value has not been read");
                       return (-1);
                   }
                   fsr = fscanf(pf_conf, "Gain=%s\n", shutter);
                   if (fsr == 0) {
                       printf("Error: Gain value has not been read");
                       return (-1);
                   }
                   fsr = fscanf(pf_conf, "Brightness=%s\n", brightness);
                   if (fsr == 0) {
                       printf("Error: Brightness value has not been read");
                       return (-1);
                   }
                   fsr = fscanf(pf_conf, "Exposure=%s\n", exposure);
                   if (fsr == 0) {
                       printf("Error: Exposure value has not been read");
                       return (-1);
                   }
                   fsr = fscanf(pf_conf, "Visualization=%s\n", vis);
                   if (fsr == 0) {
                       printf("Error: Visualization value has not been read");
                       return (-1);
                   }
                   fsr = fscanf(pf_conf, "ComputePitch=%s\n", pitch);
                   if (fsr == 0) {
                       printf("Error: Compute value has not been read");
                       return (-1);
                   }
                   fsr = fscanf(pf_conf, "Save=%s\n", save);
                   if (fsr == 0) {
                       printf("Error: Save flag has not been read");
                       return (-1);
                   }
                   fsr = fscanf(pf_conf, "Method_stereo=%s\n", method);
                   if (fsr == 0) {
                       printf("Error: Method stereo has not been read");
                       return (-1);
                   }
                   fsr = fscanf(pf_conf, "Comunication=%s\n", comunication);
                   if (fsr == 0) {
                       printf("Error: Comunication has not been read");
                       return (-1);
                   }
                   fsr = fscanf(pf_conf, "Semaphore=%s\n", semaphore);
                   if (fsr == 0) {
                       printf("Error: Semaphore has not been read");
                       return (-1);
                   }

                   conf_file->height_camera = atof(height);
                   conf_file->colour = atoi(colour);
                   conf_file->reduction = atoi(reduction);
                   conf_file->mode = atoi(mode);
                   limits->min_y = (double) atof(limit_y_min);
                   limits->max_y = (double) atof(limit_y_max);
                   *waiting_areas = atoi(wa);
                   *crossing_areas = atoi(ca);
                   conf_file->point_cloud = atoi(pc);
                   conf_file->shutter = atof(shutter);
                   conf_file->gain = atof(gain);
                   conf_file->brightness = atoi(brightness);
                   conf_file->exposure = atoi(exposure);
                   conf_file->visualization = atoi(vis);
                   conf_file->computepitch = atoi(pitch);
                   conf_file->areas = 0;
                   conf_file->save = atoi(save);
                   conf_file->method_stereo = atoi(method);
                   conf_file->comunication = atoi(comunication);
                   conf_file->semaphore = atoi(semaphore);
                   return (0);
               }

               char* Config::getCmdOption(char ** begin, char ** end, const std::string & option)
               {
                   char ** itr = std::find(begin, end, option);
                   if (itr != end && ++itr != end)
                   {
                       return *itr;
                   }
                   return 0;
               }

               bool Config::cmdOptionExists(char** begin, char** end, const std::string& option)
               {
                   return std::find(begin, end, option) != end;
               }
           }
       }
    }
}
