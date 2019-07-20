/**
 * @file stereo.h
 * @author  Raúl Quintero Mínguez <raul.quintero@aut.uah.es>, David Fernández LLorca <llorca@aut.uah.es>. ISIS Research Group. University of Alcalá.
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * Project: SmartCross
 * 
 * This file contains the headers functions to compute stereo.
 * 
 */

#ifndef STEREO_H_
#define STEREO_H_

#include <src/view/av/stereo/Structs.hpp>

namespace src
{
    namespace view
    {
       namespace av
       {
           namespace stereo
           {
               class Stereo
               {
                   public:
                       int LoadCalibParams(const char* file_name, Structs::t_CalibParams* calib_params);

                       void AllocMapMatrices(Structs::t_CalibParams *calib_params, Structs::t_Map *map_matrices);

                       void ComputeStereo(Structs::t_CalibParams *calib_params, Structs::t_Map *map_matrices, Structs::t_ConfigFile config_file,
                                          cv::StereoSGBM *sgbm_state,cv::StereoBM *bm_state);

                       int InitStereo(Structs::t_ConfigFile config_file, Structs::t_Map *map_matrices, Structs::t_CalibParams *calib_params,
                                      cv::StereoSGBM *sgbm_state, cv::StereoBM *bm_state);

                       void Remap(cv::Mat src, cv::Mat dst, cv::Mat mx, cv::Mat my);


                       void GetDisparityMapAnd3D(Structs::t_Map *map_matrices, Structs::t_ConfigFile config_file,
                                                 cv::StereoSGBM *sgbm_state, cv::StereoBM *bm_state, Structs::t_ImagesStereo *images_stereo);

                       double ComputePitch(Structs::t_ImagesStereo *images_stereo);

                       void ProjectToGround(Structs::t_CalibParams *calib_params, Structs::t_ImagesStereo *images_stereo);

                       void AdjustExtrinsicParams(int computepitch, Structs::t_CalibParams *calib_params, Structs::t_ImagesStereo *images_stereo);

                       //void CreatePointCloud(t_ImagesStereo *images_stereo, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

                       void GetImageXZ(Structs::t_Limits limits, Structs::t_ImagesStereo *images_stereo);
               };
           }
       }
    }
}

#endif /* STEREO_H */
