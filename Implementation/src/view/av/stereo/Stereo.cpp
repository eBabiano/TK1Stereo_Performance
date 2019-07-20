/**
 * @file video.cpp
 * @author  Raúl Quintero Mínguez <raul.quintero@aut.uah.es>, David Fernández LLorca <llorca@aut.uah.es>. ISIS Research Group. University of Alcalá.
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * Project: SmartCross
 * 
 * This file contains functions to compute stereo.
 * 
 */

#include <src/view/av/stereo/Stereo.hpp>
///#include "cputime.h"


namespace src
{
    namespace view
    {
       namespace av
       {
           namespace stereo
           {

               /**
                * Reads the file with the calibration parameters and save the information in the appropriate structure.
                * @param IN file_name: Name of file where the the calibration parameters are saved.
                * @param OUT calib_params: Calibration parameters.
                *
                * @return int value is returned. -1 if any parameters have not been read, 0 if all parameters have been read correctly.
                **/
               int Stereo::LoadCalibParams(const char* file_name, Structs::t_CalibParams* calib_params) {
                   // Variables.
                   int i = 0;
                   int j = 0;
                   int ret_fs = 0;
                   FILE* f = NULL;
                   // End variables.

                   // Opens the file with the information about the stereo and correlation parameters.
                   f = fopen((const char *) file_name, "r");
                   if (!f) {
                       printf("File hasn't been opened.\n");
                       return (-1);
                   } else {
                       // Loads camera parameters.
                       for (i = 0; i < 2; i++) {
                           if (i == 0) {
                               ret_fs = fscanf(f, "LeftCameraParams:");
                           } else {
                               ret_fs = fscanf(f, "RightCameraParams:");
                           }

                           ret_fs = fscanf(f, "\n");

                           // Reads image size.
                           for (j = 0; j < 2; j++) {
                               ret_fs = fscanf(f, "%d ", &(calib_params->camera[i].img_size[j]));
                               if (ret_fs != 1) {
                                   printf("\nERROR in LoadCalibParams: size of image hasn't been read\n");
                                   return (-1);
                               }
                           }

                           ret_fs = fscanf(f, "\n");

                           // Reads the matrix of intrinsic parameters.
                           for (j = 0; j < 9; j++) {
                               ret_fs = fscanf(f, "%lf ", &(calib_params->camera[i].matrix[j]));
                               if (ret_fs != 1) {
                                   printf("\nERROR in LoadCoalibParams: the matrix of intrinsic parameters hasn't been read\n");
                                   return (-1);
                               }
                           }
                           printf("Intrinsic matrix %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n",
                                  calib_params->camera[i].matrix[0], calib_params->camera[i].matrix[1], calib_params->camera[i].matrix[2],
                                   calib_params->camera[i].matrix[3], calib_params->camera[i].matrix[4], calib_params->camera[i].matrix[5],
                                   calib_params->camera[i].matrix[6], calib_params->camera[i].matrix[7], calib_params->camera[i].matrix[8]);

                           ret_fs = fscanf(f, "\n");

                           // Reads distortion parameters.
                           for (j = 0; j < 5; j++) {
                               ret_fs = fscanf(f, "%lf ", &(calib_params->camera[i].distortion[j]));
                               if (ret_fs != 1) {
                                   printf("\nERROR in LoadCalibParams: distortion parameters haven't been read\n");
                                   return (-1);
                               }
                           }
                           printf("Distortion matrix %lf, %lf, %lf, %lf, %lf\n",
                                  calib_params->camera[i].distortion[0], calib_params->camera[i].distortion[1], calib_params->camera[i].distortion[2],
                                   calib_params->camera[i].distortion[3], calib_params->camera[i].distortion[4]);
                           ret_fs = fscanf(f, "\n");
                       }


                       // Loads stereo parameters.
                       ret_fs = fscanf(f, "StereoParams:");

                       ret_fs = fscanf(f, "\n");

                       // Reads rotation matrix.
                       for (i = 0; i < 9; i++) {
                           ret_fs = fscanf(f, "%lf ", &(calib_params->rot_matrix[i]));
                           if (ret_fs != 1) {
                               printf("\nERROR in LoadCalibParams: the rotation matrix hasn't been read\n");
                               return (-1);
                           }
                       }
                       printf("Rotation matrix %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n",
                              calib_params->rot_matrix[0], calib_params->rot_matrix[1], calib_params->rot_matrix[2],
                               calib_params->rot_matrix[3], calib_params->rot_matrix[4], calib_params->rot_matrix[5],
                               calib_params->rot_matrix[6], calib_params->rot_matrix[7], calib_params->rot_matrix[8]);

                       ret_fs = fscanf(f, "\n");

                       // Reads the translation vector.
                       for (i = 0; i < 3; i++) {
                           ret_fs = fscanf(f, "%lf ", &(calib_params->trans_vector[i]));
                           if (ret_fs != 1) {
                               printf("\nERROR in LoadCalibParams: the translation vector hasn't been read\n");
                               return (-1);
                           }
                       }
                       printf("Translation vector %lf, %lf, %lf\n",
                              calib_params->trans_vector[0], calib_params->trans_vector[1], calib_params->trans_vector[2]);

                       ret_fs = fscanf(f, "\n");

                       // Reads the yaw angle.
                       ret_fs = fscanf(f, "%lf ", &(calib_params->pitch));
                       if (ret_fs != 1) {
                           printf("\nERROR in LoadCalibParams: the yaw angle hasn't been read\n");
                           return (-1);
                       }
                       printf("Pitch %lf\n", calib_params->pitch);
                       ret_fs = fscanf(f, "\n");
                       ret_fs = fscanf(f, "\n");

                       // Reads the pitch angle.
                       ret_fs = fscanf(f, "%lf ", &(calib_params->yaw));
                       if (ret_fs != 1) {
                           printf("\nERROR in LoadCalibParams: the pitch angle hasn't been read\n");
                           return (-1);
                       }
                       printf("Yaw %lf\n", calib_params->yaw);
                       ret_fs = fscanf(f, "\n");
                       ret_fs = fscanf(f, "\n");

                       // Reads roll angle.
                       ret_fs = fscanf(f, "%lf ", &(calib_params->roll));
                       if (ret_fs != 1) {
                           printf("\nERROR in LoadCalibParams: the roll angle hasn't been read\n");
                           return (-1);
                       }
                       printf("Roll %lf\n", calib_params->roll);

                       // Closes the file.
                       fclose(f);

                       return (0);
                   }
               }

               /**
                * Allocates the memory for the mapping matrices.
                * @param IN calib_params: Calibration parameters.
                * @param OUT map_matrices: Matrices for computing the distortion and rectification.
                *
                * @return void value is returned.
                **/
               void Stereo::AllocMapMatrices(Structs::t_CalibParams *calib_params, Structs::t_Map *map_matrices) {
                   map_matrices->mrx1 = cv::Mat(cv::Size(calib_params->camera[0].img_size[0] / calib_params->reduction, calib_params->camera[0].img_size[1] / calib_params->reduction), CV_64FC1);
                   map_matrices->mrx2 = cv::Mat(cv::Size(calib_params->camera[0].img_size[0] / calib_params->reduction, calib_params->camera[0].img_size[1] / calib_params->reduction), CV_64FC1);
                   map_matrices->mry1 = cv::Mat(cv::Size(calib_params->camera[0].img_size[0] / calib_params->reduction, calib_params->camera[0].img_size[1] / calib_params->reduction), CV_64FC1);
                   map_matrices->mry2 = cv::Mat(cv::Size(calib_params->camera[0].img_size[0] / calib_params->reduction, calib_params->camera[0].img_size[1] / calib_params->reduction), CV_64FC1);
               }

               /**
                * Computes the stereo information.
                * @param IN calib_params: Calibration parameters.
                * @param OUT map_matrices: Matrices for computing the distortion and rectification.
                * @param OUT sgbm_state: Semi global block matching structure for computing the disparity map.
                *
                * @return void value is returned.
                **/
               void Stereo::ComputeStereo(Structs::t_CalibParams *calib_params, Structs::t_Map *map_matrices, Structs::t_ConfigFile config_file,
                                          cv::StereoSGBM *sgbm_state, cv::StereoBM *bm_state) {

                   int SIZE_MATRIX = 3;
                   int SIZE_VECTOR = 5;
                   int LEFT_CAMERA = 0;
                   int RIGHT_CAMERA = 0;
                   // Variables.
                   double M1[SIZE_MATRIX][SIZE_MATRIX];
                   double M2[SIZE_MATRIX][SIZE_MATRIX];
                   double D1[SIZE_VECTOR];
                   double D2[SIZE_VECTOR];
                   double R[SIZE_MATRIX][SIZE_MATRIX];
                   double T[SIZE_MATRIX];
                   double R1[SIZE_MATRIX][SIZE_MATRIX];
                   double R2[SIZE_MATRIX][SIZE_MATRIX];
                   double P1[SIZE_MATRIX][4];
                   double P2[SIZE_MATRIX][4];
                   int i = 0, j = 0;
                   // End local variables.

                   // Loads matrices.
                   // Rotation Matrix.
                   for (i = 0; i < SIZE_MATRIX; i++) {
                       for (j = 0; j < SIZE_MATRIX; j++)
                           R[i][j] = calib_params->rot_matrix[SIZE_MATRIX * i + j];
                   }

                   // Left and right distortion vectors.
                   for (i = 0; i < SIZE_VECTOR; i++) {
                       D1[i] = calib_params->camera[LEFT_CAMERA].distortion[i];
                   }
                   for (i = 0; i < SIZE_VECTOR; i++) {
                       D2[i] = calib_params->camera[RIGHT_CAMERA].distortion[i];
                   }

                   // Translation vector.
                   for (i = 0; i < SIZE_MATRIX; i++) {
                       T[i] = calib_params->trans_vector[i];
                   }

                   // Matrix of intrinsic parameters.
                   M1[0][0] = calib_params->camera[LEFT_CAMERA].matrix[0];
                   M1[0][1] = calib_params->camera[LEFT_CAMERA].matrix[1];
                   M1[0][2] = calib_params->camera[LEFT_CAMERA].matrix[2];
                   M1[1][0] = calib_params->camera[LEFT_CAMERA].matrix[3];
                   M1[1][1] = calib_params->camera[LEFT_CAMERA].matrix[4];
                   M1[1][2] = calib_params->camera[LEFT_CAMERA].matrix[5];
                   M1[2][0] = calib_params->camera[LEFT_CAMERA].matrix[6];
                   M1[2][1] = calib_params->camera[LEFT_CAMERA].matrix[7];
                   M1[2][2] = calib_params->camera[LEFT_CAMERA].matrix[8];
                   M2[0][0] = calib_params->camera[RIGHT_CAMERA].matrix[0];
                   M2[0][1] = calib_params->camera[RIGHT_CAMERA].matrix[1];
                   M2[0][2] = calib_params->camera[RIGHT_CAMERA].matrix[2];
                   M2[1][0] = calib_params->camera[RIGHT_CAMERA].matrix[3];
                   M2[1][1] = calib_params->camera[RIGHT_CAMERA].matrix[4];
                   M2[1][2] = calib_params->camera[RIGHT_CAMERA].matrix[5];
                   M2[2][0] = calib_params->camera[RIGHT_CAMERA].matrix[6];
                   M2[2][1] = calib_params->camera[RIGHT_CAMERA].matrix[7];
                   M2[2][2] = calib_params->camera[RIGHT_CAMERA].matrix[8];

                   cv::Mat _M1(SIZE_MATRIX, SIZE_MATRIX, CV_64FC1, M1);
                   cv::Mat _M2(SIZE_MATRIX, SIZE_MATRIX, CV_64FC1, M2);
                   cv::Mat _D1(SIZE_VECTOR, 1, CV_64FC1, D1);
                   cv::Mat _D2(SIZE_VECTOR, 1, CV_64FC1, D2);
                   cv::Mat _R(SIZE_MATRIX, SIZE_MATRIX, CV_64FC1, R);
                   cv::Mat _T(SIZE_MATRIX, 1, CV_64FC1, T);
                   cv::Mat _R1(SIZE_MATRIX, SIZE_MATRIX, CV_64FC1, R1);
                   cv::Mat _R2(SIZE_MATRIX, SIZE_MATRIX, CV_64FC1, R2);
                   cv::Mat _P1(SIZE_MATRIX, 4, CV_64FC1, P1);
                   cv::Mat _P2(SIZE_MATRIX, 4, CV_64FC1, P2);

                   cv::stereoRectify(_M1, _D1, _M2, _D2, cv::Size(calib_params->camera[0].img_size[0] / calib_params->reduction,
                           calib_params->camera[0].img_size[1] / calib_params->reduction),
                           _R, _T, _R1, _R2, _P1, _P2, map_matrices->mq, CV_CALIB_ZERO_DISPARITY);
                   cv::initUndistortRectifyMap(_M1, _D1, _R1, _P1,
                                               cv::Size(calib_params->camera[0].img_size[0] / calib_params->reduction,
                                               calib_params->camera[0].img_size[1] / calib_params->reduction), CV_32FC1, map_matrices->mrx1,
                                               map_matrices->mry1);
                   cv::initUndistortRectifyMap(_M2, _D2, _R2, _P2,
                                               cv::Size(calib_params->camera[0].img_size[0] / calib_params->reduction,
                                               calib_params->camera[0].img_size[1] / calib_params->reduction), CV_32FC1,
                                               map_matrices->mrx2, map_matrices->mry2);

                   // Modifies semi global block matching stereo correspondence structure.
                   if (config_file.method_stereo == 1) {
                       sgbm_state->SADWindowSize = 21;
                       sgbm_state->numberOfDisparities = 64;
                       sgbm_state->preFilterCap = 21;
                       sgbm_state->minDisparity = 0;
                       sgbm_state->uniquenessRatio = 11;
                       sgbm_state->speckleWindowSize = 21;
                       sgbm_state->speckleRange = 32;
                       sgbm_state->disp12MaxDiff = 2;
                       sgbm_state->P1 = 2 * sgbm_state->SADWindowSize * sgbm_state->SADWindowSize;
                       sgbm_state->P2 = 5 * sgbm_state->SADWindowSize * sgbm_state->SADWindowSize;
                       sgbm_state->fullDP = false;
                   }
                   else if (config_file.method_stereo == 0) {
                       bm_state->init(cv::StereoBM::BASIC_PRESET, 32, 21);
                   }

               }

               /**
                * Initializes the process for computing stereo.
                * @param IN config_file: Information of configuration.
                * @param OUT map_matrices: Matrices for computing the distortion and rectification.
                * @param OUT calib_params: Calibration parameters.
                * @param OUT sgbm_state: Block matching structure for computing the disparity map.
                *
                * @return int value is returned. -1 if maps are NULL, 0 if maps aren't NULL.
                **/
               int Stereo::InitStereo(Structs::t_ConfigFile config_file, Structs::t_Map *map_matrices, Structs::t_CalibParams *calib_params,
                                      cv::StereoSGBM *sgbm_state, cv::StereoBM *bm_state) {

                   // Loads calibration parameters of the calibration file.
                   if (LoadCalibParams(config_file.calib_file, calib_params) == -1) {
                       printf("\nCalibration parameters have been read incorrectly. \n\n");
                       return (-1);
                   } else
                       printf("\nCalibration parameters have been read correctly. \n\n");

                   // Copy reduction factor value into calibration structure.
                   calib_params->reduction = config_file.reduction;

                   // Creates mapping matrices.
                   AllocMapMatrices(calib_params, map_matrices);

                   // Computes the stereo information.
                   ComputeStereo(calib_params, map_matrices, config_file, sgbm_state, bm_state);

                   // Copies height camera to calibration_parameters.
                   calib_params->height = config_file.height_camera;

                   return 0;
               }

               /**
                * Remapping images to delete distorsion and apply rectification
                * @param IN src: Source image.
                * @param IN/OUT dst: Destination image.
                * @param IN mx: The first map.
                * @param IN my: The second map.
                * @param OUT images_stereo: Images.
                *
                * @return void value is returned.
                **/
               void Stereo::Remap(cv::Mat src, cv::Mat dst, cv::Mat mx, cv::Mat my) {
                   cv::remap(src, dst, mx, my, cv::INTER_CUBIC, cv::BORDER_TRANSPARENT, 0);
               }

               /**
                * Computing stereo correspondence using the semi-global block matching algorithm.
                * @param IN left: Left image.
                * @param IN right: Right image.
                * @param OUT disp: Disparity image.
                * @param IN sgbm_state: semi-global block matching structure.
                *
                * @return void value is returned.
                **/
//               void Stereo::SGBM(cv::Mat left, cv::Mat right, cv::Mat disp, cv::StereoSGBM *sgbm_state) {
//                   sgbm_state->operator()(left, right, disp);
//               }

               /**
                * Gets the disparity map and 3D coordinates using stereo.
                * @param IN map_matrices: Matrices for computing the distortion and rectification.
                * @param IN sgbm_state: Semi global block matching structure for computing the disparity map.
                * @param OUT images_stereo: Images.
                *
                * @return void value is returned.
                **/
               void Stereo::GetDisparityMapAnd3D(Structs::t_Map *map_matrices, Structs::t_ConfigFile config_file,
                                                 cv::StereoSGBM *sgbm_state, cv::StereoBM *bm_state, Structs::t_ImagesStereo *images_stereo) {
                   // Variables.
                   cv::Mat undist_left_image_up, undist_left_image_down, undist_right_image_up,
                           undist_right_image_down, disp_image_16S_up, disp_image_16S_down;
                   cv::StereoSGBM sgbm_state2;
                   // End variables.

                   // Applies a generic geometrical transformation to the images of video.
                   ///boost::thread th(Remap, images_stereo->left_image, images_stereo->undist_left_image,
                   ///                 map_matrices->mrx1, map_matrices->mry1);
                   Remap(images_stereo->left_image, images_stereo->undist_left_image,
                         map_matrices->mrx1, map_matrices->mry1);
                   Remap(images_stereo->right_image, images_stereo->undist_right_image,
                         map_matrices->mrx2, map_matrices->mry2);
                   ///th.join();

                   cv::cvtColor(images_stereo->undist_left_image, images_stereo->undist_left_image_color, CV_GRAY2RGB);
                   cv::cvtColor(images_stereo->undist_right_image, images_stereo->undist_right_image_color, CV_GRAY2RGB);

                   if (config_file.method_stereo == 1) {
                       // Create ROIs.
                       undist_left_image_up = images_stereo->undist_left_image(cv::Rect(0, 0, images_stereo->undist_left_image.cols, images_stereo->undist_left_image.rows / 2 + 11));
                       undist_left_image_down = images_stereo->undist_left_image(cv::Rect(0, images_stereo->undist_left_image.rows / 2 - 11, images_stereo->undist_left_image.cols, images_stereo->undist_left_image.rows / 2));
                       undist_right_image_up = images_stereo->undist_right_image(cv::Rect(0, 0, images_stereo->undist_right_image.cols, images_stereo->undist_right_image.rows / 2 + 11));
                       undist_right_image_down = images_stereo->undist_right_image(cv::Rect(0, images_stereo->undist_right_image.rows / 2 - 11, images_stereo->undist_right_image.cols, images_stereo->undist_right_image.rows / 2));
                       disp_image_16S_up = images_stereo->disp_image_16S(cv::Rect(0, 0, images_stereo->disp_image_16S.cols, images_stereo->disp_image_16S.rows / 2 + 11));
                       disp_image_16S_down = images_stereo->disp_image_16S(cv::Rect(0, images_stereo->disp_image_16S.rows / 2, images_stereo->disp_image_16S.cols, images_stereo->disp_image_16S.rows / 2));

                       // Copy SGBM structure.
                       sgbm_state2.SADWindowSize = sgbm_state->SADWindowSize;
                       sgbm_state2.numberOfDisparities = sgbm_state->numberOfDisparities;
                       sgbm_state2.preFilterCap = sgbm_state->preFilterCap;
                       sgbm_state2.minDisparity = sgbm_state->minDisparity;
                       sgbm_state2.uniquenessRatio = sgbm_state->uniquenessRatio;
                       sgbm_state2.speckleWindowSize = sgbm_state->speckleWindowSize;
                       sgbm_state2.speckleRange = sgbm_state->speckleRange;
                       sgbm_state2.disp12MaxDiff = sgbm_state->disp12MaxDiff;
                       sgbm_state2.P1 = sgbm_state->P1;
                       sgbm_state2.P2 = sgbm_state->P2;
                       sgbm_state2.fullDP = sgbm_state->fullDP;

                       ///boost::thread th2(SGBM, undist_left_image_up, undist_right_image_up, disp_image_16S_up, &sgbm_state2);
                       ///SGBM(undist_left_image_down, undist_right_image_down, disp_image_16S_down, sgbm_state);
                       ///th2.join();
                       //    SGBM(images_stereo->undist_left_image, images_stereo->undist_right_image, images_stereo->disp_image_16S, sgbm_state);

                       images_stereo->disp_image_16S.convertTo(images_stereo->disp_image, CV_32F, 1.0 / 16.0);
                       images_stereo->disp_image_16S.convertTo(images_stereo->vdisp_image, CV_8U, 255 / (sgbm_state->numberOfDisparities * 16.));
                   }

                   else if (config_file.method_stereo == 0)
                   {
                       bm_state->operator()(images_stereo->undist_left_image,
                                            images_stereo->undist_right_image, images_stereo->disp_image_16S);

                       images_stereo->disp_image_16S.convertTo(images_stereo->disp_image, CV_32F, 1.0 / 16.0);
                       images_stereo->disp_image_16S.convertTo(images_stereo->vdisp_image, CV_8U, 255 / (32 * 16.));
                   }

                   // Remove bad matchings
                   cv::threshold(images_stereo->vdisp_image, images_stereo->vdisp_image, 140, 255, CV_THRESH_TOZERO_INV);

                   // Filter
                   cv::line(images_stereo->disp_image, cv::Point(1, 1), cv::Point(images_stereo->disp_image.cols - 1, 1), cv::Scalar(0), 40);
                   cv::line(images_stereo->vdisp_image, cv::Point(1, 1), cv::Point(images_stereo->vdisp_image.cols - 1, 1), cv::Scalar(0), 40);

                   // Gets 3D information.
                   //map_matrices->mq.at<double>(3,2) = -map_matrices->mq.at<double>(3,2);
                   cv::reprojectImageTo3D(images_stereo->disp_image, images_stereo->image3D, map_matrices->mq, false);

               }

               /**
                * Calculates pitch angle applying RANSAC.
                * @param IN images_stereo: Images.
                *
                * @return double value is returned. Pitch value in radians is returned.
                **/
               double Stereo::ComputePitch(Structs::t_ImagesStereo *images_stereo) {
                   // Variables.
//                   std::vector<cv::Mat> coordinates(3);
//                   cv::Mat coord_x, coord_y, coord_z;
//                   t_Plane plane;
//                   cv::Point point;
//                   int i = 0, j = 0;
//                   cv::Point3d point3D;
//                   double a = 0.0, b = 0.0, c = 0.0, d = 0.0;
//                   double plane_d = 0.0;
//                   double distance_point_plane = 0.0;
//                   t_NormalVector normal_vector;
//                   int iters = 0;
//                   int iter_search_points = 0;
//                   std::vector<cv::Point3d> inliers;
//                   std::vector<cv::Point3d> inliers_plane;
//                   cv::Mat plane_matrix(3, 1, CV_64FC1);
//                   cv::Mat a_matrix, a_inv_matrix, a_transpose_matrix, b_matrix;
//                   // End variables.

//                   printf("Computing pitch angle....\n");

//                   // Splits 3D information into X,Y and Z coordinates.
//                   cv::split(images_stereo->image3D, coordinates);
//                   coord_x = coordinates[0];
//                   coord_y = coordinates[1];
//                   coord_z = coordinates[2];

//                   srand(time(NULL));
//                   inliers_plane.clear();

//                   do {

//                       // Selects three 2D points on the image randomly. If points haven't been found, the application will finish after 1000 iterations.
//                       plane.points3D.clear();
//                       inliers.clear();

//                       iter_search_points = 0;
//                       for (i = 0; i < 3; i++) {
//                           point.x = rand() % images_stereo->left_image.cols + 1;
//                           point.y = rand() % images_stereo->left_image.rows + 1;

//                           point3D.x = (double) (coord_x.at<float>(point.y, point.x));
//                           point3D.y = (double) (coord_y.at<float>(point.y, point.x));
//                           point3D.z = (double) (coord_z.at<float>(point.y, point.x));

//                           if (images_stereo->vdisp_image.at<uchar> (point.y, point.x) > 0 && !isnan(point3D.x) && !isnan(point3D.y) && !isnan(point3D.z) && point3D.z > 0.0) {
//                               plane.points3D.push_back(point3D);
//                           } else {
//                               i--;
//                           }

//                           if (iter_search_points > ITERS_SEARCH_POINTS)
//                               break;

//                           iter_search_points++;
//                       }

//                       if (plane.points3D.size() < 3) {
//                           printf("Number of points for computing pitch is insufficient.\n");
//                           exit(-1);
//                       }

//                       // Plane model.
//                       a = (plane.points3D.at(1).y - plane.points3D.at(0).y) * (plane.points3D.at(2).z - plane.points3D.at(0).z) - (plane.points3D.at(1).z - plane.points3D.at(0).z) * (plane.points3D.at(2).y - plane.points3D.at(0).y);
//                       b = -((plane.points3D.at(1).x - plane.points3D.at(0).x) * (plane.points3D.at(2).z - plane.points3D.at(0).z) - (plane.points3D.at(1).z - plane.points3D.at(0).z) * (plane.points3D.at(2).x - plane.points3D.at(0).x));
//                       c = (plane.points3D.at(1).x - plane.points3D.at(0).x) * (plane.points3D.at(2).y - plane.points3D.at(0).y) - (plane.points3D.at(1).y - plane.points3D.at(0).y) * (plane.points3D.at(2).x - plane.points3D.at(0).x);
//                       d = -a * plane.points3D.at(0).x + b * plane.points3D.at(0).y - c * plane.points3D.at(0).z;

//                       // Gets inliers every ten pixels in order to decrease the computation time.
//                       distance_point_plane = 0.0;
//                       for (i = 0; i < images_stereo->vdisp_image.rows; i++) {
//                           for (j = 0; j < images_stereo->vdisp_image.cols; j++) {
//                               point3D.x = (double) (coord_x.at<float>(i, j));
//                               point3D.y = (double) (coord_y.at<float>(i, j));
//                               point3D.z = (double) (coord_z.at<float>(i, j));
//                               if (images_stereo->vdisp_image.at<uchar> (i, j) > 0 && !isnan(point3D.x) && !isnan(point3D.y) && !isnan(point3D.z) && point3D.z > 0.0) {
//                                   distance_point_plane = abs(a * point3D.x + b * point3D.y + c * point3D.z + d) / sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));

//                                   if (distance_point_plane < EPSILON) {
//                                       inliers.push_back(point3D);
//                                   }
//                               }
//                           }
//                       }

//                       // Saves the plane with maximum number of inliers.
//                       if (iters == 1 || inliers_plane.size() < inliers.size()) {
//                           inliers_plane.clear();
//                           inliers_plane = inliers;
//                           plane_d = d;
//                       }

//                       iters++;

//                   } while (iters < 3000);

//                   // Get plane model with inliers.
//                   a_matrix = cv::Mat(inliers_plane.size(), 3, CV_64FC1);
//                   b_matrix = cv::Mat(inliers_plane.size(), 1, CV_64FC1);
//                   for (i = 0; i < inliers_plane.size(); i++) {
//                       a_matrix.at<double>(i, 0) = inliers_plane.at(i).x;
//                       a_matrix.at<double>(i, 1) = inliers_plane.at(i).y;
//                       a_matrix.at<double>(i, 2) = inliers_plane.at(i).z;
//                       b_matrix.at<double>(i) = -plane_d;
//                   }

//                   cv::transpose(a_matrix, a_transpose_matrix);
//                   cv::invert(a_transpose_matrix*a_matrix, a_inv_matrix, CV_SVD);
//                   plane_matrix = a_inv_matrix * a_transpose_matrix * b_matrix;

//                   // Normal vector.
//                   normal_vector.x = plane_matrix.at<double>(0);
//                   normal_vector.y = plane_matrix.at<double>(1);
//                   normal_vector.z = plane_matrix.at<double>(2);

//                   // Normalization.
//                   normal_vector.x = normal_vector.x / sqrt(pow(normal_vector.x, 2) + pow(normal_vector.y, 2) + pow(normal_vector.z, 2));
//                   normal_vector.y = normal_vector.y / sqrt(pow(normal_vector.x, 2) + pow(normal_vector.y, 2) + pow(normal_vector.z, 2));
//                   normal_vector.z = normal_vector.z / sqrt(pow(normal_vector.x, 2) + pow(normal_vector.y, 2) + pow(normal_vector.z, 2));

//                   // Changes the normal direction when component y is negative.
//                   if (normal_vector.y < 0) {
//                       normal_vector.x = -normal_vector.x;
//                       normal_vector.y = -normal_vector.y;
//                       normal_vector.z = -normal_vector.z;
//                   }

//                   //printf("Roll %lf\n", asin(normal_vector.x));
//                   return acos(normal_vector.y) + PI;
               }

               /**
                * Projects 3D coordinates from camera to ground.
                * @param IN calib_params: Calibration parameters.
                * @param OUT images_stereo: Images.
                *
                * @return void value is returned.
                **/
               void Stereo::ProjectToGround(Structs::t_CalibParams *calib_params, Structs::t_ImagesStereo * images_stereo) {

                   // Variables.
                   double sx = 0.0, cx = 0.0, sy = 0.0, cy = 0.0, sz = 0.0, cz = 0.0;
                   std::vector<cv::Mat> coordinates(3);
                   cv::Mat coord_x, coord_y, coord_z;
                   int i = 0, j = 0;
                   cv::Mat translation_vector;
                   cv::Mat rotation_matrix;
                   // End variables.

                   cv::split(images_stereo->image3D, coordinates);
                   coord_x = coordinates[0];
                   coord_y = coordinates[1];
                   coord_z = coordinates[2];

                   // Translation vector.
                   translation_vector = cv::Mat(3, 1, CV_64FC1);
                   translation_vector.at<double>(0) = 0.0;
                   translation_vector.at<double>(1) = calib_params->height;
                   translation_vector.at<double>(2) = 0.0;
                   calib_params->extrinsic_trans_vector = translation_vector;

                   // Sines and cosines of rotation matrix.
                   sx = sin(calib_params->pitch);
                   cx = cos(calib_params->pitch);
                   sy = sin(calib_params->yaw);
                   cy = cos(calib_params->yaw);
                   sz = sin(calib_params->roll);
                   cz = cos(calib_params->roll);

                   // Rotation matrix. Rz * Rx * Ry.
                   rotation_matrix = cv::Mat(3, 3, CV_64FC1);
                   rotation_matrix.at<double>(0, 0) = cy * cz - sx * sy * sz;
                   rotation_matrix.at<double>(0, 1) = -cx * sz;
                   rotation_matrix.at<double>(0, 2) = sy * cz + sx * cy * sz;
                   rotation_matrix.at<double>(1, 0) = cy * sz + sx * sy * cz;
                   rotation_matrix.at<double>(1, 1) = cx * cz;
                   rotation_matrix.at<double>(1, 2) = sy * sz - sx * cy * cz;
                   rotation_matrix.at<double>(2, 0) = -sy * cx;
                   rotation_matrix.at<double>(2, 1) = sx;
                   rotation_matrix.at<double>(2, 2) = cx * cy;
                   calib_params->extrinsic_rot_matrix = rotation_matrix;

                   // Initializes the 3D matrix.
                   images_stereo->image3D_ground.zeros(images_stereo->image3D_ground.rows, images_stereo->image3D_ground.cols, CV_64FC3);

                   // Projects 3D points from camera reference to ground reference.
                   for (i = 0; i < images_stereo->image3D.rows; i++) {
                       for (j = 0; j < images_stereo->image3D.cols; j++) {

                           images_stereo->image3D_ground.at<cv::Vec3d>(i, j)[0] = rotation_matrix.at<double>(0, 0) * (double) coord_x.at<float>(i, j) + rotation_matrix.at<double>(0, 1) * (double) coord_y.at<float>(i, j) + rotation_matrix.at<double>(0, 2) * (double) coord_z.at<float>(i, j) + translation_vector.at<double>(0);
                           images_stereo->image3D_ground.at<cv::Vec3d>(i, j)[1] = rotation_matrix.at<double>(1, 0) * (double) coord_x.at<float>(i, j) + rotation_matrix.at<double>(1, 1) * (double) coord_y.at<float>(i, j) + rotation_matrix.at<double>(1, 2) * (double) coord_z.at<float>(i, j) + translation_vector.at<double>(1);
                           images_stereo->image3D_ground.at<cv::Vec3d>(i, j)[2] = rotation_matrix.at<double>(2, 0) * (double) coord_x.at<float>(i, j) + rotation_matrix.at<double>(2, 1) * (double) coord_y.at<float>(i, j) + rotation_matrix.at<double>(2, 2) * (double) coord_z.at<float>(i, j) + translation_vector.at<double>(2);
                       }
                   }
               }

               /**
                * Adjusts extrinsic parameters in order to project to ground.
                * @param IN-OUT calib_params: Calibration parameters.
                * @param IN-OUT images_stereo: Images.
                *
                * @return void value is returned.
                **/
               void Stereo::AdjustExtrinsicParams(int computepitch, Structs::t_CalibParams *calib_params, Structs::t_ImagesStereo * images_stereo) {

                   if (computepitch)
                       calib_params->pitch = ComputePitch(images_stereo);
                   printf("Pitch = %lf\n", calib_params->pitch);
                   printf("Yaw = %lf\n", calib_params->yaw);
                   printf("Roll = %lf\n", calib_params->roll);

                   ProjectToGround(calib_params, images_stereo);
               }

               /**
                * Gets the X-Z map.
                * @param IN limits: Limits for 3D map.
                * @param IN images_stereo: Structure with the information about images.
                *
                * @return void value is returned.
                **/
               void Stereo::GetImageXZ(Structs::t_Limits limits, Structs::t_ImagesStereo * images_stereo) {
                   // Variables.
                   int i = 0;
                   int j = 0;
                   double x = 0.0;
                   double y = 0.0;
                   double z = 0.0;
                   std::vector<cv::Mat> coordinates(3);
                   cv::Mat coord_x, coord_y, coord_z;
                   int u = 0, v = 0;
                   // End variables.

                   cv::split(images_stereo->image3D_ground, coordinates);
                   coord_x = coordinates[0];
                   coord_y = coordinates[1];
                   coord_z = coordinates[2];

                   images_stereo->imageXZ = cv::Mat::zeros(images_stereo->image3D_ground.rows, images_stereo->image3D_ground.cols, CV_8UC1);

                   for (i = 0; i < images_stereo->image3D_ground.rows; i++) {
                       for (j = 0; j < images_stereo->image3D_ground.cols; j++) {
                           if (images_stereo->vdisp_image.at<uchar> (i, j) > 0) {
                               x = coord_x.at<double>(i, j);
                               y = coord_y.at<double>(i, j);
                               z = coord_z.at<double>(i, j);

                               if (x >= limits.min_x && x <= limits.max_x && y >= limits.min_y && y <= limits.max_y && z >= limits.min_z && z <= limits.max_z) {
                                   u = (int) ((abs(limits.max_x - x) * images_stereo->imageXZ.cols) / abs(limits.min_x - limits.max_x));
                                   v = (int) (images_stereo->imageXZ.rows - (((z - abs(limits.min_z)) * images_stereo->imageXZ.rows) / abs(limits.max_z - limits.min_z)));

                                   images_stereo->imageXZ.at<uchar>(v, u) = 255;
                               }
                           }
                       }
                   }
               }
           }
       }
    }
}
