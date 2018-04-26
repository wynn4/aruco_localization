#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

#include <ros/ros.h>
#include <aruco/aruco.h>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <aruco_localization/MarkerMeasurement.h>
#include <aruco_localization/MarkerMeasurementArray.h>
#include <aruco_localization/FloatList.h>
#include <std_srvs/Trigger.h>

#include <std_msgs/Float32.h>

#include <experimental/filesystem>

namespace aruco_localizer {

    class ArucoLocalizer
    {
    public:
        ArucoLocalizer();

    private:
        // ROS node handles
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        // state subscriber
        ros::Subscriber state_sub_;

        // image transport pub/sub
        image_transport::ImageTransport it_;
        image_transport::CameraSubscriber image_sub_;
        image_transport::Publisher image_pub_;

        // ROS tf listener and broadcaster
        tf::TransformListener tf_listener_;
        tf::TransformBroadcaster tf_br_;

        // Bias for the roll and pitch components of camera to body
        tf::Quaternion quat_att_bias_;

        // ROS publishers and subscribers
        ros::Publisher estimate_pub_;
        // ros::Publisher meas_pub_;

        // Outer marker of the nested marker
        ros::Publisher center_pix_outer_pub_;
        ros::Publisher corner_pix_outer_pub_;
        ros::Publisher distance_outer_pub_;
        ros::Publisher heading_outer_pub_;

        // Inner marker of the nested marker
        ros::Publisher center_pix_inner_pub_;
        ros::Publisher corner_pix_inner_pub_;
        ros::Publisher distance_inner_pub_;

        ros::ServiceServer calib_attitude_;

        // ArUco Map Detector
        double markerSize_;
        aruco::MarkerMap mmConfig_;
        aruco::MarkerDetector mDetector_;
        aruco::MarkerMapPoseTracker mmPoseTracker_;
        aruco::CameraParameters camParams_;

        bool showOutputVideo_;
        bool debugSaveInputFrames_;
        bool debugSaveOutputFrames_;
        std::string debugImagePath_;

        // Save header time stamp of original image
        std_msgs::Header image_header_;

        // NaN counter
        int nanCount_ = 0;

        // IDs for outer and inner marker
        int id_outer_;
        int id_inner_;

        // Inner marker size (size of the outer marker is markerSize_)
        double markerSize_inner_;

        // Quadcopter Euler angles
        double phi_;
        double theta_;
        double psi_;

        // Marker relative heading
        float rel_heading_;
        Eigen::Matrix<float, 2, 1> heading_vec_;
        Eigen::Matrix2f R_c_v1_;

        // Level-Frame mapping stuff
        std::vector<cv::Point2f> corners_;
        std::vector<cv::Point2f> cornersUndist_;
        std::vector<cv::Point2f> levelCorners_;

        Eigen::Matrix<float, 3, 4> uvf_;
        Eigen::Matrix<float, 3, 4> hom_;

        Eigen::Matrix3f R_vlc_v1_;
        Eigen::Matrix3f R_v1_v2_;
        Eigen::Matrix3f R_v2_b_;
        Eigen::Matrix3f R_v1_b_;
        Eigen::Matrix3f R_b_m_;
        Eigen::Matrix3f R_m_c_;
        Eigen::Matrix3f R_c_vlc_;

        cv::Point2f corner0_;
        cv::Point2f corner1_;
        cv::Point2f corner2_;
        cv::Point2f corner3_;

        cv::Point2f level_corner0_;
        cv::Point2f level_corner1_;
        cv::Point2f level_corner2_;
        cv::Point2f level_corner3_;

        cv::Mat cameraMatrix_;
        cv::Mat distortionCoeff_;

        // Camera Focal length
        float f_;
        float fx_;
        float fy_;

        // Image dimensions
        int im_height_;
        int im_width_;

        // Misc
        bool first_;

        //
        // Methods
        //

        // image_transport camera subscriber
        void cameraCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cinfo);

        // quadcopter state subscriber
        void stateCallback(const nav_msgs::OdometryConstPtr& msg);

        // service handlers
        bool calibrateAttitude(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

        // This is where the real ArUco processing is done
        void processImage(cv::Mat& frame, bool drawDetections);

        // Convert ROS CameraInfo message to ArUco style CameraParameters
        aruco::CameraParameters ros2arucoCamParams(const sensor_msgs::CameraInfoConstPtr& cinfo);

        // functions to convert to tf and then broadcast
        tf::Quaternion rodriguesToTFQuat(const cv::Mat& rvec);
        tf::Transform aruco2tf(const cv::Mat& rvec, const cv::Mat& tvec);
        void sendtf(const cv::Mat& rvec, const cv::Mat& tvec);

        // Save the current frame to file. Useful for debugging
        void saveInputFrame(const cv::Mat& frame);
        void saveOutputFrame(const cv::Mat& frame);
        void saveFrame(const cv::Mat& frame, std::string format_spec, unsigned int img_num);
    };

}