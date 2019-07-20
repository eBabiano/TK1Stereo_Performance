/**
 * @file structs.h
 * @author  Raúl Quintero Mínguez <raul.quintero@aut.uah.es>, David Fernández LLorca <llorca@aut.uah.es>. ISIS Research Group. University of Alcalá.
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * Project: SmartCross
 * 
 * This file contains all structures that are used by the application.
 * 
 */

#ifndef STRUCTS_H
#define	STRUCTS_H

#include <vector>
#include <string>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <boost/thread/thread.hpp>
/*#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>*/
#include <boost/thread/thread.hpp>
///#include "FlyCapture2.h"
#include <lib/flycapture2/include/FlyCapture2.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>


#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdlib.h>



using namespace std;

//using namespace FlyCapture2;


#define DEVICE_SEM_COCHE "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_85436323431351D03011-if00"
#define DEVICE_SEM_PEATONES "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_75439333235351519102-if00"

#define CONFIG_FILE_NAME "../res/stereo/config.txt" // Path of configure file.
#define PI 3.14159265359
#define ITERS_SEARCH_POINTS 500 // Maximum number of iterations to search 3D points for computing pitch.
#define MIN_AREA_W 300 //Minimum area of candidate. 
#define MIN_AREA_H 300 //Minimum area of candidate. 
#define MAX_AREA_W 4000 //Maximum area of pedestrian. 
#define SIGMA_R 0.01 // Variance of MeasurementNoiseCov matrix in the Kalman filter.
#define SIGMA_Q 0.0001 //Variance of ProcessNoiseCov matrix in the Kalman filter.
#define SIGMA_P 0.001 // Variance of ErrorCovPost matrix in the Kalman filter. 
#define N_STATES 8 // Number of states for Kalman filter.
#define N_MEASUREMENTS 4 // Number of measurements for Kalman filter.
#define N_MEAS2 N_MEASUREMENTS * N_MEASUREMENTS // Number of measurements^2 for Kalman filter.
#define INFINITE 10.0 // Maximum distance for Hungarian Association.
#define MAX_LIFE 10 // Maximum life of the objects when the tracking is working.
#define EPSILON 200.0 // Minimum distance between a point and a plane in RANSAC.
#define OVERLAY_FRAMES 35
#define SOFTWARE_TRIGGER_CAMERA
/*#define CAM_IZQ_SERIAL_NUMBER 14330633
//12191177 camara lab
#define CAM_DER_SERIAL_NUMBER 14292755
//12251186 camara lab
*/
#define CAM_IZQ_SERIAL_NUMBER 12191177
#define CAM_DER_SERIAL_NUMBER 12251186
#define DER 1
#define IZQ 0
#define NUM_BUFFERS 25
#define NUM_IMG_ADJ_SHUTTER 5
#define SHUTTER_ADJ_FRAMES 300
#define LOGGING_FRAMES 10000
#define ACTIVAR_PANEL 0
#define COUNTER_FLAG_SIGN 12*2
#define POINTS_DIRECTION 20

namespace src
{
    namespace view
    {
       namespace av
       {
           namespace stereo
           {
               class Structs
               {
                   public:
                       /**
                        * \struct t_ConfigFile
                        * \brief Contains the configuration parameters.
                        **/
                       typedef struct {
                           char video_left[100]; /**< Path of left video. */
                           char video_right[100]; /**< Path of right video. */
                           char calib_file[100]; /**< Path of calibration file. */
                           char areas_file[100]; /**< Path of areas file. */
                           double height_camera; /**< Height where the camera are situated. */
                           int colour; /**< Flag for indicating if videos and cameras are in colour or not. */
                           int reduction; /**< Reduction factor value for images. */
                           int mode; /**< Running mode. 1 Camera, 0 Videos. */
                           int point_cloud; /**< Flag for visualising the point cloud. */
                           float shutter;
                           float gain;
                           int brightness;
                           int exposure;
                           int visualization; /**< Flag for visualising the images. */
                           int computepitch; /**< Flag for computing the pitch angle. */
                           int areas;
                           int save;
                           int method_stereo; /**< Flag for indicating the method of stereo (0 BM, 1 SGBM) . */
                           int comunication; //Flag for indicating the comunication with the car
                           int semaphore;
                       } t_ConfigFile;

                       /**
                        * \struct t_Limits
                        * \brief Contains the limits of 3D map.
                        **/
                       typedef struct {
                           double min_x; /**< Minimum value in x coordinate for 3D map.*/
                           double max_x; /**< Maximum value in x coordinate for 3D map.*/
                           double min_y; /**< Minimum value in y coordinate for 3D map.*/
                           double max_y; /**< Maximum value in y coordinate for 3D map.*/
                           double min_z; /**< Minimum value in z coordinate for 3D map.*/
                           double max_z; /**< Maximum value in z coordinate for 3D map.*/
                       } t_Limits;

                       /**
                        * \struct t_Camera
                        * \brief Contains the calibration parameters of one camera.
                        **/
                       typedef struct {
                           int img_size[2]; /**< Correspond with the width and height of the camera view. Two elements of the array.  */
                           double matrix[9]; /**< Intrinsic parameters: [fx 0 cx;0 fy cy;0 0 1 ].*/
                           double distortion[4]; /**< Distortion coefficients - two coefficients for radial distortion and another two for tangential: [ k1 k2 p1 p2 ]. */
                       } t_Camera;

                       /**
                        * \struct t_CalibParams
                        * \brief Contains the calibration parameters.
                        **/
                       typedef struct {
                           t_Camera camera[2]; /**< Two individual camera parameters. */
                           double fund_matrix[9]; /**< Fundamental matrix. */
                           double rot_matrix[9]; /**< Rotation matrix between cameras. */
                           double trans_vector[3]; /**< Translation vector between cameras. */
                           double yaw; /**< Yaw angle. */
                           double pitch; /**< Pitch angle. */
                           double roll; /**< Roll angle. */
                           double height; /**< Height of cameras (mm). */
                           cv::Mat extrinsic_rot_matrix; /**< Rotation matrix between camera reference and ground. */
                           cv::Mat extrinsic_trans_vector; /**< Translation vector between camera reference and ground. */
                           int reduction; /**< Reduction factor value for images. */
                       } t_CalibParams;

                       /**
                        * \struct t_Map
                        * \brief Contains matrices for computing the undistortion and rectification.
                        **/
                       typedef struct {
                           cv::Mat mrx1; /**< Mapping matrix. */
                           cv::Mat mrx2; /**< Mapping matrix. */
                           cv::Mat mry1; /**< Mapping matrix. */
                           cv::Mat mry2; /**< Mapping matrix. */
                           cv::Mat mq; /**< Disparity-to-depth mapping matrix. */
                       } t_Map;

                       /**
                        * \struct t_Videos
                        * \brief Contains the paths, names of videos and the pointers.
                        **/
                       typedef struct {
                           char video_file_right[200]; /**< Right camera name of video. */
                           char video_file_left[200]; /**< Left camera name of video.  */
                           FILE *pf_left; /**< Pointer to the left video. */
                           FILE *pf_right; /**< Pointer to the right video. */
                       } t_Videos;

                       /**
                        * \struct t_ImagesStereo
                        * \brief Contains images.
                        **/
                       typedef struct {
                           cv::Mat left_image_color_raw; /**< Left original image RGB . */
                           cv::Mat right_image_color_raw; /**< Right original image RGB . */
                           cv::Mat left_image_raw; /**< Left original image. */
                           cv::Mat right_image_raw; /**< Right original image . */
                           cv::Mat left_image_color; /**< Left original resized image RGB . */
                           cv::Mat right_image_color; /**< Right original resized image RGB . */
                           cv::Mat left_image; /**< Left original resized image. */
                           cv::Mat right_image; /**< Right original resized image. */
                           cv::Mat undist_left_image; /**< Left resized image of video without distortion. */
                           cv::Mat undist_right_image; /**< Right resized image of video without distortion. */
                           cv::Mat undist_left_image_color; /**< Left image of video without distortion (3 channels). */
                           cv::Mat undist_right_image_color; /**< Right image of video without distortion (3 channels). */
                           cv::Mat disp_image_16S; /**< Disparity map image in 16S. */
                           cv::Mat disp_image; /**< Disparity map image. */
                           cv::Mat vdisp_image; /**< Disparity map image in level grey. */
                           cv::Mat image3D; /**< Matrix with 3D. Reference: camera */
                           cv::Mat image3D_ground; /**< Matrix with 3D. Reference: ground. */
                           cv::Mat areas; /**< Image with pedestrian areas. */
                           cv::Mat plane; /**< Image with plane. */
                           cv::Mat back; /**< Background. */
                           cv::Mat imageXZ; /**< Image with X-Z coordinates. */
                           cv::Mat imageXZ_prev1; /**< Image with X-Z coordinates in t-1. */
                           cv::Mat imageXZ_prev2; /**< Image with X-Z coordinates in t-2. */
                           cv::Mat imageXZ_prev3; /**< Image with X-Z coordinates in t-3. */
                           cv::Mat img_candidates; /**< Image with candidates on X-Z coordinates. */
                           cv::Mat img_objects; /**< Image with objects on X-Z coordinates. */
                           cv::Mat img_objects_color; /**< Image with objects on X-Z coordinates. */
                       } t_ImagesStereo;

                       /**
                        * \struct t_NormalVector
                        * \brief Contains information about a normal vector.
                        **/
                       typedef struct {
                           vector<cv::Point3d> points3D; /**< 3D information of the three points used for computing the normal vector. */
                           double x; /**< X component of normal vector. */
                           double y; /**< Y component of normal vector. */
                           double z; /**< Z component of normal vector. */
                       } t_NormalVector;

                       typedef struct {
                           vector<cv::Point> points2D; /**< Points 2D on the ground selected for computing pitch angle. */
                           vector<cv::Point3d> points3D; /**< Points 3D on the ground selected for computing pitch angle. */
                       } t_Plane;

                       /**
                        * \struct t_Candidate
                        * \brief Contains information about the detected objects (candidate).
                        **/
                       typedef struct {
                           cv::Rect rect; /**< Bounding rectangle of a candidate. */
                       } t_Candidate;

                       /**
                        * \struct t_Object
                        * \brief Contains information about the detected objects.
                        **/
                       typedef struct {
                           cv::Rect rect_XZ; /**< Bounding rectangle of an object on X-Z map. */
                           cv::Point mass_centre_XZ; /**< Center point of the bounding rectangle on XZ map in t. */
                           cv::Point mass_centre_XZ_in; /**< Center point of the bounding rectangle on XZ map in input. */
                           cv::KalmanFilter kalman_filter; /**< Structure with the Kalman filter. */
                           int life; /**< Counter of object timelife. */
                           bool tracking; /**< Flag to know if object is in tracking or pre-tracking (0 pre-tracking, 1 tracking). */
                           int ident; /**< Object identification. It's unique for each object. */
                           int type; /**< Type of object. 0 pedestrian, 1 vehicle. */
                           int zone; /**< Zone of the object. 0 crossing area, 1 waiting area. */
                           double direction;
                           int statistics; /**< Flag for indicating that the object have been accounted for the statistics (number of objects). */
                           int statistics2; /**< Flag for indicating that the object have been accounted for the statistics (direction). */
                           int in; /**< Input waiting area. */
                           int out; /**< Output waiting area. */
                           int act_sign;
                           int act_sign_waiting_area_index;
                           int act_sign_crossing_area_index;
                           int act_sign_crossing_area;
                           int act_sign_waiting_area;
                           double occupancy;
                           vector<cv::Point> trajectory;
                           cv::Point first;
                           cv::Point second;
                           double module;
                           int stop_times;
                       } t_Object;

                       /**
                        * \struct t_WaitingArea
                        * \brief Contains information about the waiting areas.
                        **/
                       typedef struct {
                           vector<cv::Point> points2D; /**< Points 2D selected by the user to mark the waiting area. */
                           vector<cv::Point3d> points3D; /**< Points 3D selected by the user to mark the waiting area. */
                           vector<cv::Point> pointsXZ; /**< Points 2D selected by the user to print the waiting area on X-Z map. */
                           cv::Mat mask; /**< Binary mask. */
                           cv::Mat proximity; /**< Proximity to area matrix. */
                       } t_WaitingArea;

                       /**
                        * \struct t_RoadArea
                        * \brief Contains information about the road areas.
                        **/
                       typedef struct {
                           vector<cv::Point> points2D; /**< Points 2D selected by the user to mark the crossing area. */
                           vector<cv::Point3d> points3D; /**< Points 3D selected by the user to mark the crossing area. */
                           vector<cv::Point> pointsXZ; /**< Points 2D selected by the user to print the crossing area on X-Z map. */
                           cv::Mat mask; /**< Binary mask. */
                       } t_CrossingArea;

                       /**
                        * \struct t_Processing
                        * \brief Contains all elements to be process in threads mode
                        **/
                       typedef struct {
                           int nident; /**< Index of detected objects. */
                           long int frame; /**< Index of frames. */
                           t_ConfigFile config_file; /**< Structure for saving configuration file. */
                           t_Limits limits; /**< Structure for saving the limits of the 3D map. */
                           int num_waiting_areas; /**< Number of waiting areas in the scene. */
                           int num_crossing_areas; /**< Number of crossing areas in the scene. */
                           t_CalibParams calib_params;
                           t_Map map_matrices;
                           cv::StereoSGBM sgbm_state;
                           cv::StereoBM bm_state;
                           t_ImagesStereo images_stereo;
                           FILE *pf_areas;
                           std::vector<t_Candidate> candidates;
                           std::vector<t_Object> objects;
                           std::vector<t_Object> peatones;
                         cv::BackgroundSubtractorMOG bg;
                           std::vector<t_WaitingArea> waiting_areas;
                           std::vector<t_CrossingArea> crossing_areas;
                           int flag_sign;
                           int num_pedestrians;
                           int num_vehicles;
                           int num_pedestrians12;
                           int num_pedestrians21;
                         double diff_time;
                       } t_Processing;

                       typedef union _AbsValueConversion {
                           unsigned int ulValue;
                           float fValue;
                       } AbsValueConversion;

                       /**
                        * \struct t_Socket
                        * \brief Contains elements for the comunications with the car
                        **/
                       typedef struct {
                           int socket;
                           char buffer [80];
                           bool socket_open;
                           struct sockaddr_in si_other;
                           int slen;//=sizeof(si_other);
                       } t_Socket;

                       /**
                        * \struct t_Semaphores
                        * \brief Contains elements for the serial port comunication
                        **/
                       typedef struct {
                           int fd0;
                           int fd1;
                       } t_Semaphores;


               };
           }
       }
    }
}
#endif	/* STRUCTS_H */
