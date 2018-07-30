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

#include <rosflight_msgs/Command.h>
#include <mavros_msgs/PositionTarget.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <aruco_localization/MarkerMeasurement.h>
#include <aruco_localization/MarkerMeasurementArray.h>
#include <aruco_localization/FloatList.h>
#include <std_srvs/Trigger.h>

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

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

        // p_des subscriber
        ros::Subscriber p_des_outer_sub_;
        ros::Subscriber p_des_inner_sub_;

        // state machine status subscriber
        ros::Subscriber status_sub_;

        // current target subscriber
        ros::Subscriber current_target_sub_;

        // IBVS velocity subscriber
        ros::Subscriber ibvs_vel_sub_mavros_;
        ros::Subscriber ibvs_vel_sub_roscopter_;

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
        // ros::Publisher orientation_inner_pub_;
        ros::Publisher k_angle_pub_;

        ros::ServiceServer calib_attitude_;

        // ArUco Map Detector
        double markerSize_;
        aruco::MarkerMap mmConfig_;
        aruco::MarkerDetector mDetector_;
        aruco::MarkerMapPoseTracker mmPoseTracker_;
        aruco::CameraParameters camParams_;

        // Pose tracker for center marker
        aruco::MarkerPoseTracker mPoseTracker_;

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

        Eigen::Matrix<float, 3, 1> p_tilde_;
        Eigen::Matrix<float, 3, 1> zeta_;
        Eigen::Matrix<float, 3, 1> P_c_;
        Eigen::Matrix<float, 4, 4> P_array_c_;
        Eigen::Matrix<float, 4, 4> P_array_cv_;

        Eigen::Matrix3f R_vlc_v1_;
        Eigen::Matrix3f R_v1_v2_;
        Eigen::Matrix3f R_v2_b_;
        Eigen::Matrix3f R_v1_b_;
        Eigen::Matrix3f R_b_m_;
        Eigen::Matrix3f R_m_c_;
        Eigen::Matrix3f R_c_vlc_;

        // Homogeneous Transform Matrices
        Eigen::Matrix4f T_vlc_v1_;
        Eigen::Matrix4f T_v1_b_;
        Eigen::Matrix4f T_b_m_;
        Eigen::Matrix4f T_m_c_;
        Eigen::Matrix4f T_vlc_c_;
        Eigen::Matrix4f T_c_vlc_;

        // Camera mount offset from C.G.
        Eigen::Matrix<float, 3, 1> camera_offset_;

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
        bool resize_;
        bool drawData_;

        float z_c_;
        float z_c_outer_;
        float z_c_inner_;

        float vx_, vy_, vz_, wz_;
        float vel_angle_;

        double q_wx_;
        double q_wy_;
        double q_wz_;
        double q_xx_;
        double q_xy_;
        double q_xz_;
        double q_yy_;
        double q_yz_;
        double q_zz_;

        double q_x_;
        double q_y_;
        double q_z_;
        double q_w_;

        Eigen::Matrix3d mQmatrix_;
        Eigen::Matrix<double, 3, 1> k_axis_;
        Eigen::Matrix<double, 3, 1> k_axis_disp_;
        double k_angle_;


        // Stuff for drawing
        cv::Point p_des_outer_0_;
        cv::Point p_des_outer_1_;
        cv::Point p_des_outer_2_;
        cv::Point p_des_outer_3_;

        cv::Point p_des_inner_0_;
        cv::Point p_des_inner_1_;
        cv::Point p_des_inner_2_;
        cv::Point p_des_inner_3_;

        std::string status_;
        std::string currentTarget_;

        // Colors
        cv::Scalar mRed_ = cv::Scalar(0, 0, 255);
        cv::Scalar mGreen_ = cv::Scalar(0, 255, 0);
        cv::Scalar mBlue_ = cv::Scalar(255, 0, 0);
        cv::Scalar mYellow_ = cv::Scalar(0,255,255);
        cv::Scalar mWhite_ = cv::Scalar(255,255,255);
        cv::Scalar mBlack_ = cv::Scalar(0,0,0);
        cv::Scalar mCyan_ = cv::Scalar(255,255,0);

        //
        // Methods
        //

        // image_transport camera subscriber
        void cameraCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cinfo);

        // quadcopter state subscriber
        void stateCallback(const nav_msgs::OdometryConstPtr& msg);

        // IBVS p_des subscribers
        void pdesOuterCallback(const aruco_localization::FloatList& msg);
        void pdesInnerCallback(const aruco_localization::FloatList& msg);

        // state machine status callback
        void statusCallback(const std_msgs::String& msg);

        // current target callback
        void currentTargetCallback(const std_msgs::String& msg);

        // ibvs velocity callback roscopter
        void roscopterVelCallback(const rosflight_msgs::Command& msg);

        // ibvs velocity callback mavros
        void mavrosVelCallback(const mavros_msgs::PositionTarget& msg);

        // service handlers
        bool calibrateAttitude(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

        // This is where the real ArUco processing is done
        void processImage(cv::Mat& frame, bool drawDetections);

        // Draw Level Corners
        void drawLevelCorners(cv::Mat& frame, std::vector<float>& corners);

        // Convert ROS CameraInfo message to ArUco style CameraParameters
        aruco::CameraParameters ros2arucoCamParams(const sensor_msgs::CameraInfoConstPtr& cinfo);

        // functions to convert to tf and then broadcast
        tf::Quaternion rodriguesToTFQuat(const cv::Mat& rvec);
        tf::Transform aruco2tf(const cv::Mat& rvec, const cv::Mat& tvec);
        void sendtf(const cv::Mat& rvec, const cv::Mat& tvec);

        // function to convert a quaternion to a rotation matrix
        void get_R_from_quat();

        // Save the current frame to file. Useful for debugging
        void saveInputFrame(const cv::Mat& frame);
        void saveOutputFrame(const cv::Mat& frame);
        void saveFrame(const cv::Mat& frame, std::string format_spec, unsigned int img_num);
    };

}