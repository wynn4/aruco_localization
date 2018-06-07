#include "aruco_localization/ArucoLocalizer.h"

namespace aruco_localizer {

// ----------------------------------------------------------------------------

ArucoLocalizer::ArucoLocalizer() :
    nh_(ros::NodeHandle()), nh_private_("~"), it_(nh_)
{

    // Read in ROS params
    std::string mmConfigFile = nh_private_.param<std::string>("markermap_config", "");
    markerSize_ = nh_private_.param<double>("marker_size", 0.0298);
    markerSize_inner_ = nh_private_.param<double>("marker_size_inner", 0.0100);
    id_outer_ = nh_private_.param<int>("id_outer", -1);
    id_inner_ = nh_private_.param<int>("id_inner", -1);
    nh_private_.param<bool>("show_output_video", showOutputVideo_, false);
    nh_private_.param<bool>("debug_save_input_frames", debugSaveInputFrames_, false);
    nh_private_.param<bool>("debug_save_output_frames", debugSaveOutputFrames_, false);
    nh_private_.param<std::string>("debug_image_path", debugImagePath_, "/tmp/arucoimages");
    double camera_offset_x = nh_private_.param<double>("camera_offset_x", 0.0);
    double camera_offset_y = nh_private_.param<double>("camera_offset_y", 0.0);
    double camera_offset_z = nh_private_.param<double>("camera_offset_z", 0.0);

    nh_private_.param<bool>("shrink_image", resize_, false);
    nh_private_.param<bool>("draw_data", drawData_, false);

    // Initialize arrays
    corners_.reserve(4);
    cornersUndist_.reserve(4);
    levelCorners_.reserve(4);
    
    double defaultIntrinsics[9] = {1000.0, 0.0, 320.0, 0.0, 1000.0, 240.0, 0.0, 0.0, 1.0};
    cv::Mat temp = cv::Mat(3,3, CV_64FC1, defaultIntrinsics);
    temp.copyTo(cameraMatrix_);

    double defaultDistortion[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    temp = cv::Mat(5,1, CV_64FC1, defaultDistortion);
    temp.copyTo(distortionCoeff_);

    R_v1_v2_ = Eigen::Matrix3f::Identity();
    R_v2_b_ = Eigen::Matrix3f::Identity();
    R_v1_b_.setZero();
    R_vlc_v1_.setZero();
    R_vlc_v1_(0,1) = -1.0;
    R_vlc_v1_(1,0) = 1.0;
    R_vlc_v1_(2,2) = 1.0;
    // R_vlc_v1 = np.array([[0., -1., 0.],
    //                      [1., 0., 0.],
    //                      [0., 0., 1.]])

    // TODO add the rotations
    // Just set to identity for now (assumes that camera is mounted pointing straight down)
    R_b_m_ = Eigen::Matrix3f::Identity();

    R_m_c_.setZero();
    R_m_c_(0,1) = 1.0;
    R_m_c_(1,0) = -1.0;
    R_m_c_(2,2) = 1.0;
    // R_m_c = np.array([[0., 1., 0.],
    //                   [-1., 0., 0.],
    //                   [0., 0., 1.]])

    R_c_vlc_.setZero();

    heading_vec_.setZero();
    R_c_v1_.setZero();
    R_c_v1_(0,1) = -1.0;
    R_c_v1_(1,0) = 1.0;

    T_vlc_v1_ = Eigen::Matrix4f::Identity();
    T_v1_b_ = Eigen::Matrix4f::Identity();
    T_b_m_ = Eigen::Matrix4f::Identity();
    T_m_c_ = Eigen::Matrix4f::Identity();
    T_vlc_c_ = Eigen::Matrix4f::Identity();
    T_c_vlc_ = Eigen::Matrix4f::Identity();

    camera_offset_.setZero();
    camera_offset_(0,0) = camera_offset_x;
    camera_offset_(1,0) = camera_offset_y;
    camera_offset_(2,0) = camera_offset_z;

    // Add in the camera offset to the T_b_m_ transformation
    T_b_m_.block<3,1>(0,3) = camera_offset_;

    // Add in the rotations to the remaining transformations
    T_vlc_v1_.block<3,3>(0,0) = R_vlc_v1_;
    T_b_m_.block<3,3>(0,0) = R_b_m_;
    T_m_c_.block<3,3>(0,0) = R_m_c_;

    // uvf_ is augmented with a row of ones
    uvf_(3,0) = 1.0;
    uvf_(3,1) = 1.0;
    uvf_(3,2) = 1.0;
    uvf_(3,3) = 1.0;

    k_axis_(0,0) = 0.0;
    k_axis_(1,0) = 0.0;
    k_axis_(2,0) = -1.0;

    k_angle_ = 9999.0;


    // Initialize the attitude bias to zero
    quat_att_bias_.setRPY(0, 0, 0);

    // Initialize focal length to something non-zero
    f_ = 2000.0;
    fx_ = 2000.0;
    fy_ = 2000.0;

    first_ = true;

    status_ = " ";

    // Subscribe to state
    state_sub_ = nh_.subscribe("/quadcopter/ground_truth/odometry/NED", 1, &ArucoLocalizer::stateCallback, this);

    // Subscribe to desired corner locations
    p_des_outer_sub_ = nh_.subscribe("/ibvs/pdes_outer", 1, &ArucoLocalizer::pdesOuterCallback, this);
    p_des_inner_sub_ = nh_.subscribe("/ibvs/pdes_inner", 1, &ArucoLocalizer::pdesInnerCallback, this);
    status_sub_ = nh_.subscribe("/status_flag", 1, &ArucoLocalizer::statusCallback, this);
    current_target_sub_ = nh_.subscribe("/ibvs_status_flag", 1, &ArucoLocalizer::currentTargetCallback, this);

    ibvs_vel_sub_roscopter_ = nh_.subscribe("/quadcopter/high_level_command", 1, &ArucoLocalizer::roscopterVelCallback, this);
    ibvs_vel_sub_mavros_ = nh_.subscribe("/mavros/setpoint_raw/local", 1, &ArucoLocalizer::mavrosVelCallback, this);

    // Subscribe to input video feed and publish output video feed
    it_ = image_transport::ImageTransport(nh_);
    image_sub_ = it_.subscribeCamera("input_image", 1, &ArucoLocalizer::cameraCallback, this);
    image_pub_ = it_.advertise("output_image", 1);

    // Create ROS publishers
    estimate_pub_ = nh_private_.advertise<geometry_msgs::PoseStamped>("estimate", 1);
    // meas_pub_ = nh_private_.advertise<aruco_localization::MarkerMeasurementArray>("measurements", 1);
    center_pix_outer_pub_ = nh_private_.advertise<geometry_msgs::PointStamped>("marker_center_outer", 1);
    corner_pix_outer_pub_ = nh_private_.advertise<aruco_localization::FloatList>("marker_corners_outer", 1);
    distance_outer_pub_ = nh_private_.advertise<std_msgs::Float32>("distance_outer", 1);
    heading_outer_pub_ = nh_private_.advertise<std_msgs::Float32>("heading_outer", 1);

    center_pix_inner_pub_ = nh_private_.advertise<geometry_msgs::PointStamped>("marker_center_inner", 1);
    corner_pix_inner_pub_ = nh_private_.advertise<aruco_localization::FloatList>("marker_corners_inner", 1);
    distance_inner_pub_ = nh_private_.advertise<std_msgs::Float32>("distance_inner", 1);
    orientation_inner_pub_ = nh_private_.advertise<geometry_msgs::Quaternion>("orientation_inner", 1);

    // Create ROS services
    calib_attitude_ = nh_private_.advertiseService("calibrate_attitude", &ArucoLocalizer::calibrateAttitude, this);

    //
    // Set up the ArUco detector
    //

    // Set up the Marker Map dimensions, spacing, dictionary, etc from the YAML
    mmConfig_.readFromFile(mmConfigFile);

    // Prepare the marker detector by:
    // (1) setting the dictionary we are using
    mDetector_.setDictionary(mmConfig_.getDictionary());
    // (2) setting the corner refinement method
    // ... TODO -- make this corner sub pix or something
    // mDetector_.setCornerRefinementMethod(aruco::MarkerDetector::LINES);

    // set markmap size. Convert to meters if necessary
    if (mmConfig_.isExpressedInPixels())
        mmConfig_ = mmConfig_.convertToMeters(markerSize_);

    // Configuring of Pose Tracker is done once a CameraInfo message has been received.

    //
    // Misc
    //

    

    // Create the `debug_image_path` if it doesn't exist
    std::experimental::filesystem::create_directories(debugImagePath_);
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

bool ArucoLocalizer::calibrateAttitude(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

    // Get the latest attitude in the body frame (is this in the body frame?)
    tf::StampedTransform transform;
    tf_listener_.lookupTransform("base", "chiny", ros::Time(0), transform);

    // Store the old bias correction term to correctly capture the original biased attitude
    tf::Quaternion q0(quat_att_bias_.x() ,quat_att_bias_.y(), quat_att_bias_.z(), quat_att_bias_.w());

    // extract the inverse of the current attitude, unbiased using the current quat bias term
    // Get the original biased attitude by multiplying by the old bias correction term
    quat_att_bias_ = transform.getRotation()*q0;

    // Let the caller know what the new zeroed setpoint is
    double r, p, y;
    tf::Matrix3x3(quat_att_bias_).getRPY(r,p,y);
    res.success = true;
    res.message = "Zeroed at: R=" + std::to_string(r*(180.0/M_PI)) + ", P=" + std::to_string(p*(180.0/M_PI));
    return true;
}

// ----------------------------------------------------------------------------

void ArucoLocalizer::sendtf(const cv::Mat& rvec, const cv::Mat& tvec) {

    // We want all transforms to use the same exact time
    // ros::Time now = ros::Time::now();

    // Create the transform from the camera to the ArUco Marker Map
    tf::Transform transform = aruco2tf(rvec, tvec);

    //
    // Link the aruco (parent) to the camera (child) frames
    //

    // Note that `transform` is a measurement of the ArUco map w.r.t the camera,
    // therefore the inverse gives the transform from `aruco` to `camera`.
    // tf_br_.sendTransform(tf::StampedTransform(transform.inverse(), now, "aruco", "camera"));

    //
    // Publish measurement of the pose of the ArUco board w.r.t the camera frame
    //

    geometry_msgs::PoseStamped poseMsg;
    tf::poseTFToMsg(transform, poseMsg.pose);
    poseMsg.header.frame_id = "camera";
    poseMsg.header.stamp = image_header_.stamp;

    // poseMsg.pose.position.x = NAN;
    // poseMsg.pose.position.y = NAN;
    // poseMsg.pose.position.z = NAN;

    // poseMsg.pose.orientation.x = NAN;
    // poseMsg.pose.orientation.y = NAN;
    // poseMsg.pose.orientation.z = NAN;
    // poseMsg.pose.orientation.w = NAN;

    estimate_pub_.publish(poseMsg);

    // Check for NaNs
    double nanCheck = poseMsg.pose.position.x;
    if (std::isnan(nanCheck)) nanCount_++;
}

// ----------------------------------------------------------------------------

void ArucoLocalizer::processImage(cv::Mat& frame, bool drawDetections) {

    // Detection of the board
    std::vector<aruco::Marker> detected_markers = mDetector_.detect(frame);

    // Get the corner and center pixel data
    for (auto idx : mmConfig_.getIndices(detected_markers))
    {
        cv::Point2f center = detected_markers[idx].getCenter();

        // corner pixels

        // top left (NW)
        corner0_.x = detected_markers[idx][0].x;
        corner0_.y = detected_markers[idx][0].y;

        // top right (NE)
        corner1_.x = detected_markers[idx][1].x;
        corner1_.y = detected_markers[idx][1].y;
        // bottom right (SE)
        corner2_.x = detected_markers[idx][2].x;
        corner2_.y = detected_markers[idx][2].y;

        // bottom left (SW)
        corner3_.x = detected_markers[idx][3].x;
        corner3_.y = detected_markers[idx][3].y;

        // add corners to our corners_ vector
        corners_.push_back(corner0_);
        corners_.push_back(corner1_);
        corners_.push_back(corner2_);
        corners_.push_back(corner3_);

        // compute approximate distance to the ArUco
        float Ls_1_2 = sqrt((corner0_.x - corner1_.x)*(corner0_.x - corner1_.x) + (corner0_.y - corner1_.y)*(corner0_.y - corner1_.y));
        float Ls_2_3 = sqrt((corner1_.x - corner2_.x)*(corner1_.x - corner2_.x) + (corner1_.y - corner2_.y)*(corner1_.y - corner2_.y));
        float Ls_3_4 = sqrt((corner2_.x - corner3_.x)*(corner2_.x - corner3_.x) + (corner2_.y - corner3_.y)*(corner2_.y - corner3_.y));
        float Ls_4_1 = sqrt((corner3_.x - corner0_.x)*(corner3_.x - corner0_.x) + (corner3_.y - corner0_.y)*(corner3_.y - corner0_.y));

        // take the average
        float Ls = (Ls_1_2 + Ls_2_3 + Ls_3_4 + Ls_4_1) / 4.0;

        //
        // Get the marker's relative heading
        //

        heading_vec_(0,0) = corner0_.x - corner3_.x;
        heading_vec_(1,0) = corner0_.y - corner3_.y;

        // rotate into the v1 frame
        heading_vec_ = R_c_v1_ * heading_vec_;

        // compute relative heading using atan2
        rel_heading_ = atan2(heading_vec_(1,0), heading_vec_(0,0));

        //
        // Transform Points into the Virtual-Level-Frame
        //

        // Undistort the corners
        cv::undistortPoints(corners_, cornersUndist_, cameraMatrix_, distortionCoeff_);
        corners_.clear();

        // De-normalize (multiply by focal length) but still keep corner locations wrt image center
        for (int i=0; i<4; i++)
        {
            cornersUndist_[i].x = cornersUndist_[i].x * fx_;
            cornersUndist_[i].y = cornersUndist_[i].y * fy_;
        }

        // Throw the undistorted corners and focal length into our Eigen uvf matrix (3x4) uvf_ = [x_pix; y_pix; f]
        uvf_(0,0) = cornersUndist_[0].x;
        uvf_(0,1) = cornersUndist_[1].x;
        uvf_(0,2) = cornersUndist_[2].x;
        uvf_(0,3) = cornersUndist_[3].x;

        uvf_(1,0) = cornersUndist_[0].y;
        uvf_(1,1) = cornersUndist_[1].y;
        uvf_(1,2) = cornersUndist_[2].y;
        uvf_(1,3) = cornersUndist_[3].y;

        uvf_(2,0) = f_;
        uvf_(2,1) = f_;
        uvf_(2,2) = f_;
        uvf_(2,3) = f_;

        // Setup rotations
        // pre-evaluate sines and cosines for rotation matrix
        float sphi = sin(phi_);
        float cphi = cos(phi_);
        float stheta = sin(theta_);
        float ctheta = cos(theta_);

        // Update rotation from v1 to v2 frame with latest pitch angle
        R_v1_v2_(0,0) = ctheta;
        R_v1_v2_(0,2) = -stheta;
        R_v1_v2_(2,0) = stheta;
        R_v1_v2_(2,2) = ctheta;

        // R_v1_v2 = np.array([[ctheta, 0., -stheta],
        //                     [0., 1., 0.],
        //                     [stheta, 0., ctheta]])

        // Update rotation from v2 to body frame with latest roll angle
        R_v2_b_(1,1) = cphi;
        R_v2_b_(1,2) = sphi;
        R_v2_b_(2,1) = -sphi;
        R_v2_b_(2,2) = cphi;

        // R_v2_b = np.array([[1., 0., 0.],
        //                    [0., cphi, sphi],
        //                    [0., -sphi, cphi]])

        // Rotation from v1 to body frame
        R_v1_b_ = R_v2_b_ * R_v1_v2_;

        // Update transformation from vehicle-1 to body frame
        T_v1_b_.block<3,3>(0,0) = R_v1_b_;

        // Compute whole rotation from camera frame to level frame
        // R_c_vlc_ = (R_m_c_ * R_b_m_ * R_v1_b_ * R_vlc_v1_).transpose();  // R_c_vlc = R_vlc_c.transpose()
        // Compose the whole transformation from the vlf to the camera frame
        T_vlc_c_ = T_m_c_ * T_b_m_ * T_v1_b_ * T_vlc_v1_;
        // Compute the inverse transformation T_c_vlc_ (with the Homogeneous transform Tinv != T.transpose)
        T_c_vlc_.block<3,3>(0,0) = T_vlc_c_.block<3,3>(0,0).transpose();
        T_c_vlc_.block<3,1>(0,3) = -T_vlc_c_.block<3,3>(0,0).transpose() * T_vlc_c_.block<3,1>(0,3);
        // T_c_vlc_ = T_vlc_c_.inverse();

        // Use the Homography to get the corner locations in the level frame
        // hom_ = R_c_vlc_ * uvf_;  // 3x4
        hom_ = T_c_vlc_ * uvf_;  // 4x4

        level_corner0_.x = f_ * (hom_(0,0)/hom_(2,0));  // u1
        level_corner0_.y = f_ * (hom_(1,0)/hom_(2,0));  // v1

        level_corner1_.x = f_ * (hom_(0,1)/hom_(2,1));  // u2
        level_corner1_.y = f_ * (hom_(1,1)/hom_(2,1));  // v2

        level_corner2_.x = f_ * (hom_(0,2)/hom_(2,2));  // u3
        level_corner2_.y = f_ * (hom_(1,2)/hom_(2,2));  // v3

        level_corner3_.x = f_ * (hom_(0,3)/hom_(2,3));  // u4
        level_corner3_.y = f_ * (hom_(1,3)/hom_(2,3));  // v4

        // Fill out the corner pixels vector {original_corner_pix, level_corner_pix}
        std::vector<float> corner_pixels = {corner0_.x, corner0_.y, corner1_.x, corner1_.y, corner2_.x, corner2_.y, corner3_.x, corner3_.y
            , level_corner0_.x, level_corner0_.y, level_corner1_.x, level_corner1_.y, level_corner2_.x, level_corner2_.y, level_corner3_.x, level_corner3_.y};


        if (detected_markers[idx].id == id_outer_)
        {
            geometry_msgs::PointStamped pointMsg;
            pointMsg.point.x = center.x;
            pointMsg.point.y = center.y;
            pointMsg.header.frame_id = "aruco_center";
            pointMsg.header.stamp = image_header_.stamp;
            // pointMsg.header = image_header_;
            center_pix_outer_pub_.publish(pointMsg);

            // publish corner pixel data
            aruco_localization::FloatList cornersMsg;
            cornersMsg.header.frame_id = "aruco_corners";
            cornersMsg.header.stamp = image_header_.stamp;
            cornersMsg.data = corner_pixels;
            corner_pix_outer_pub_.publish(cornersMsg);

            // publish distance data
            z_c_outer_ = (markerSize_ * f_) / Ls;
            std_msgs::Float32 distanceMsg;
            distanceMsg.data = z_c_outer_;
            distance_outer_pub_.publish(distanceMsg);

            // publish heading data
            std_msgs::Float32 headingMsg;
            headingMsg.data = rel_heading_;
            heading_outer_pub_.publish(headingMsg);

            // draw
            if (drawDetections)
                drawLevelCorners(frame, corner_pixels);

        }

        if (detected_markers[idx].id == id_inner_)
        {
            geometry_msgs::PointStamped pointMsg;
            pointMsg.point.x = center.x;
            pointMsg.point.y = center.y;
            pointMsg.header.frame_id = "aruco_center";
            pointMsg.header.stamp = image_header_.stamp;
            // pointMsg.header = image_header_;
            center_pix_inner_pub_.publish(pointMsg);

            // publish corner pixel data
            aruco_localization::FloatList cornersMsg;
            cornersMsg.header.frame_id = "aruco_corners";
            cornersMsg.header.stamp = image_header_.stamp;
            cornersMsg.data = corner_pixels;
            corner_pix_inner_pub_.publish(cornersMsg);

            // publish distance data
            z_c_inner_ = (markerSize_inner_ * f_) / Ls;
            std_msgs::Float32 distanceMsg;
            distanceMsg.data = z_c_inner_;
            distance_inner_pub_.publish(distanceMsg);

            // draw
            if (drawDetections)
                drawLevelCorners(frame, corner_pixels);

            //
            // Get the orientation of just the inner marker in the level frame
            //

            // Create a marker from the level-frame corners
            aruco::Marker innerMarkerLF;

            std::vector<cv::Point2f> corners;
            corners.push_back(level_corner0_);
            corners.push_back(level_corner1_);
            corners.push_back(level_corner2_);
            corners.push_back(level_corner3_);

            // Make corner locations w.r.t. top left corner of the image frame
            for (int i=0; i<4; i++)
            {
                corners[i].x = corners[i].x + im_width_/2.0;
                corners[i].y = corners[i].y + im_height_/2.0;
            }

            innerMarkerLF = aruco::Marker(corners, id_inner_);

            // Calculate Rvec & Tvec with the pose tracker
            // innerMarkerLF.calculateExtrinsics(markerSize_inner_, camParams_, false);
            if (mPoseTracker_.estimatePose(innerMarkerLF, camParams_, markerSize_inner_))
            {
                // Create the ROS message
                geometry_msgs::Quaternion quatMsg;
                tf::Quaternion quat = rodriguesToTFQuat(mPoseTracker_.getRvec());
                tf::quaternionTFToMsg(quat, quatMsg);

                // Publish.
                orientation_inner_pub_.publish(quatMsg);

                q_x_ = quatMsg.x;
                q_y_ = quatMsg.y;
                q_z_ = quatMsg.z;
                q_w_ = quatMsg.w;

                // Update our rotation from quat
                get_R_from_quat();

                // Displace the k-axis vector by the rotation
                k_axis_disp_ = mQmatrix_ * k_axis_;

                // Compute the angle between k-axis and displaced k-axis
                k_angle_ = acos(k_axis_disp_.dot(k_axis_));

                k_angle_ = 180.0 - k_angle_ * (180.0/3.14159);

                // std::cout << k_angle_ << std::endl;
            }

            corners.clear();

        }

    }

    if (drawDetections) {
        // print the markers detected that belongs to the markerset
        for (auto idx : mmConfig_.getIndices(detected_markers))
            detected_markers[idx].draw(frame, mRed_, 1);

        // Draw IBVS data on the frame
        if (currentTarget_ == "aruco_outer")
        {
            // Draw p_des_outer
            cv::circle(frame, p_des_outer_0_, 10, mGreen_, 2);
            cv::circle(frame, p_des_outer_1_, 10, mGreen_, 2);
            cv::circle(frame, p_des_outer_2_, 10, mGreen_, 2);
            cv::circle(frame, p_des_outer_3_, 10, mGreen_, 2);
        }
        else
        {
            // Draw p_des_inner
            cv::circle(frame, p_des_inner_0_, 10, mGreen_, 2);
            cv::circle(frame, p_des_inner_1_, 10, mGreen_, 2);
            cv::circle(frame, p_des_inner_2_, 10, mGreen_, 2);
            cv::circle(frame, p_des_inner_3_, 10, mGreen_, 2);
        }

        // Draw IBVS state machine status
        cv::rectangle(frame, cv::Point(0,0), cv::Point(370,40), mBlack_, CV_FILLED);
        cv::putText(frame, "State Machine Status: " + status_, cv::Point(2, 15), CV_FONT_HERSHEY_PLAIN, 1.0, mWhite_);

        if (currentTarget_ == "aruco_outer")
        {
            cv::putText(frame, "Current Target: " + currentTarget_ + " d: " + std::to_string(z_c_outer_), cv::Point(2, 35), CV_FONT_HERSHEY_PLAIN, 1.0, mWhite_);
        }
        else
        {
            cv::putText(frame, "Current Target: " + currentTarget_ + " d: " + std::to_string(z_c_inner_), cv::Point(2, 35), CV_FONT_HERSHEY_PLAIN, 1.0, mWhite_);
        }
        

        // Print Velocities
        cv::rectangle(frame, cv::Point(0,40), cv::Point(125,132), mBlack_, CV_FILLED);
        cv::putText(frame, "vx: " + std::to_string(vx_), cv::Point(2, 55), CV_FONT_HERSHEY_PLAIN, 1.0, mWhite_);
        cv::putText(frame, "vy: " + std::to_string(vy_), cv::Point(2, 75), CV_FONT_HERSHEY_PLAIN, 1.0, mWhite_);
        cv::putText(frame, "vz: " + std::to_string(vz_), cv::Point(2, 95), CV_FONT_HERSHEY_PLAIN, 1.0, mWhite_);

        // Print ArUco Angle
        if (k_angle_ <= 15.0)
        {
            cv::putText(frame, "ArUco Angle:", cv::Point(2, 115), CV_FONT_HERSHEY_PLAIN, 1.0, mWhite_);
            cv::putText(frame, std::to_string(k_angle_), cv::Point(2, 130), CV_FONT_HERSHEY_PLAIN, 1.0, mGreen_);
        }
        else
        {
            cv::putText(frame, "ArUco Angle:", cv::Point(2, 115), CV_FONT_HERSHEY_PLAIN, 1.0, mWhite_);
            cv::putText(frame, std::to_string(k_angle_), cv::Point(2, 130), CV_FONT_HERSHEY_PLAIN, 1.0, mRed_);
        }

        // // Draw velocity line
        // vel_angle_ = atan2(vy_, vx_);
        // float line_end_x = 50.0 * cos(vel_angle_);
        // float line_end_y = 50.0 * sin(vel_angle_);
        // cv::Point start;
        // start.x = 60.0;
        // start.y = 160.0;

        // cv::Point end;
        // end.x = start.x + line_end_y;
        // end.y = start.y - line_end_x;
        // cv::line(frame, start, end, mGreen_);
        // cv::circle(frame, cv::Point(start.x, start.y), 3, mRed_, 2);

        // Legend on bottom of frame
        cv::rectangle(frame, cv::Point(0, im_height_ - 20), cv::Point(im_width_, im_height_), mBlack_, CV_FILLED);
        cv::putText(frame, "Legend: ", cv::Point(2, im_height_ - 7), CV_FONT_HERSHEY_PLAIN, 1.0, mWhite_);
        cv::putText(frame, "Detected Marker", cv::Point(80, im_height_ - 7), CV_FONT_HERSHEY_PLAIN, 1.0, mRed_);
        cv::putText(frame, "Desired Corner Locations", cv::Point(235, im_height_ - 7), CV_FONT_HERSHEY_PLAIN, 1.0, mGreen_);
        cv::putText(frame, "Level-Frame Corner Locations", cv::Point(470, im_height_ - 7), CV_FONT_HERSHEY_PLAIN, 1.0, mYellow_);
    }

    //
    // Calculate pose of each individual marker w.r.t the camera
    //

    // aruco_localization::MarkerMeasurementArray measurement_msg;
    // measurement_msg.header.frame_id = "camera";
    // measurement_msg.header.stamp = ros::Time::now();

    // for (auto marker : detected_markers) {
    //     // Create Tvec, Rvec based on the camera and marker geometry
    //     marker.calculateExtrinsics(markerSize_, camParams_, false);

    //     // Create the ROS pose message and add to the array
    //     aruco_localization::MarkerMeasurement msg;
    //     msg.position.x = marker.Tvec.at<float>(0);
    //     msg.position.y = marker.Tvec.at<float>(1);
    //     msg.position.z = marker.Tvec.at<float>(2);

    //     // Represent Rodrigues parameters as a quaternion
    //     tf::Quaternion quat = rodriguesToTFQuat(marker.Rvec);
    //     tf::quaternionTFToMsg(quat, msg.orientation);

    //     // Extract Euler angles
    //     double r, p, y;
    //     tf::Matrix3x3(quat).getRPY(r,p,y);

    //     msg.euler.x = r*180/M_PI;
    //     msg.euler.y = p*180/M_PI;
    //     msg.euler.z = y*180/M_PI;

    //     // attach the ArUco ID to this measurement
    //     msg.aruco_id = marker.id;

    //     measurement_msg.poses.push_back(msg);
    // }

    // meas_pub_.publish(measurement_msg);

    //
    // Calculate pose of the entire marker map w.r.t the camera
    //

    // If the Pose Tracker was properly initialized, find 3D pose information
    if (mmPoseTracker_.isValid()) {
        if (mmPoseTracker_.estimatePose(detected_markers)) {

            // if (drawDetections)
                // aruco::CvDrawingUtils::draw3dAxis(frame, camParams_, mmPoseTracker_.getRvec(), mmPoseTracker_.getTvec(), mmConfig_[0].getMarkerSize()*2);

            sendtf(mmPoseTracker_.getRvec(), mmPoseTracker_.getTvec());
        }
    }

}

void ArucoLocalizer::drawLevelCorners(cv::Mat& frame, std::vector<float>& corners)
{
    cv::circle(frame, cv::Point(corners[8] + im_width_/2.0, corners[9] + im_height_/2.0), 10, mYellow_, 2);
    cv::circle(frame, cv::Point(corners[10] + im_width_/2.0, corners[11] + im_height_/2.0), 10, mYellow_, 2);
    cv::circle(frame, cv::Point(corners[12] + im_width_/2.0, corners[13] + im_height_/2.0), 10, mYellow_, 2);
    cv::circle(frame, cv::Point(corners[14] + im_width_/2.0, corners[15] + im_height_/2.0), 10, mYellow_, 2);
}

// ----------------------------------------------------------------------------

void ArucoLocalizer::cameraCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cinfo) {
    // Get image header (time)
    image_header_ = image->header;

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // First time this callback fires, populate the cameraMatrix and distortionCoeffs and other fields with cinfo data
    if (first_)
    {
        // Fill out cameraMatrix_ and distortioncoeff_
        for(int i=0; i<9; ++i)
            cameraMatrix_.at<double>(i%3, i-(i%3)*3) = cinfo->K[i];

        for(int i=0; i<5; ++i)
            distortionCoeff_.at<double>(i, 0) = cinfo->D[i];

        // Grab the average focal length for use later
        f_ = (cinfo->K[0] + cinfo->K[4]) / 2.0;

        // Grab x and y focal lengths
        fx_ = cinfo->K[0];
        fy_ = cinfo->K[4];

        // Grab image height and width
        im_height_ = cinfo->height;
        im_width_ = cinfo->width;

        // Done
        first_ = false;
        // std::cout << "K = "<< std::endl << " "  << cameraMatrix_ << std::endl << std::endl;
        // std::cout << "D = "<< std::endl << " "  << distortionCoeff_ << std::endl << std::endl;
    }
    

    // Configure the Pose Tracker if it has not been configured before
    if (!mmPoseTracker_.isValid() && mmConfig_.isExpressedInMeters()) {

        // Extract ROS camera_info (i.e., K and D) for ArUco library
        camParams_ = ros2arucoCamParams(cinfo);

        // Now, if the camera params have been ArUco-ified, set up the tracker
        if (camParams_.isValid())
            mmPoseTracker_.setParams(camParams_, mmConfig_);

    }


    // ==========================================================================
    // Process the incoming video frame

    // Get image as a regular Mat
    // cv::Mat frame = cv_ptr->image;

    // Resize to improve ArUco processing time
    if (resize_)
    {
        cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(962, 720), CV_INTER_LINEAR);
    }
    

    // const int mtype = frame.type();
    // std::cout << std::to_string(mtype) << std::endl;

    // THIS IS A MAJOR HACK.
    // If our pose tracker has been giving us a bunch of NaNs
    // we toss in a blank (black) frame to 'refresh' our view of the marker.
    // Although extremely hacky, this seems to work OK for 'resetting' the pose tracker.

    // TODO: Enhance this hack by only blanking when we have received a certain number
    // of NaNs within a certain amount of time.
    if (nanCount_ >= 10)
    {
        cv_ptr->image = cv::Mat::zeros(im_height_, im_width_, CV_8UC3);
        nanCount_ = 0;
    }


    if (debugSaveInputFrames_) saveInputFrame(cv_ptr->image);

    // Process the image and do ArUco localization on it
    processImage(cv_ptr->image, drawData_);

    if (debugSaveOutputFrames_) saveOutputFrame(cv_ptr->image);

    if (showOutputVideo_) {
        // Update GUI Window
        cv::imshow("detections", cv_ptr->image);
        cv::waitKey(1);
    }

    // ==========================================================================

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
}

void ArucoLocalizer::stateCallback(const nav_msgs::OdometryConstPtr &msg)
{
    // Get the roll and pitch angles for level-frame mapping
    // Convert Quaternion to RPY
    tf::Quaternion tf_quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, tf_quat);
    tf::Matrix3x3(tf_quat).getRPY(phi_, theta_, psi_);
    // phi_ = phi_;
    // theta_ = theta_;
}


void ArucoLocalizer::pdesOuterCallback(const aruco_localization::FloatList& msg)
{
    p_des_outer_0_.x = msg.data[0] + im_width_/2.0;
    p_des_outer_0_.y = msg.data[1] + im_height_/2.0;

    p_des_outer_1_.x = msg.data[2] + im_width_/2.0;
    p_des_outer_1_.y = msg.data[3] + im_height_/2.0;

    p_des_outer_2_.x = msg.data[4] + im_width_/2.0;
    p_des_outer_2_.y = msg.data[5] + im_height_/2.0;

    p_des_outer_3_.x = msg.data[6] + im_width_/2.0;
    p_des_outer_3_.y = msg.data[7] + im_height_/2.0;
}

void ArucoLocalizer::pdesInnerCallback(const aruco_localization::FloatList& msg)
{
    p_des_inner_0_.x = msg.data[0] + im_width_/2.0;
    p_des_inner_0_.y = msg.data[1] + im_height_/2.0;

    p_des_inner_1_.x = msg.data[2] + im_width_/2.0;
    p_des_inner_1_.y = msg.data[3] + im_height_/2.0;

    p_des_inner_2_.x = msg.data[4] + im_width_/2.0;
    p_des_inner_2_.y = msg.data[5] + im_height_/2.0;

    p_des_inner_3_.x = msg.data[6] + im_width_/2.0;
    p_des_inner_3_.y = msg.data[7] + im_height_/2.0;
}

void ArucoLocalizer::statusCallback(const std_msgs::String& msg)
{
    status_ = msg.data;
}

void ArucoLocalizer::currentTargetCallback(const std_msgs::String& msg)
{
    currentTarget_ = msg.data;
}

void ArucoLocalizer::roscopterVelCallback(const rosflight_msgs::Command& msg)
{
    if (status_ == "IBVS")
    {
        vx_ = msg.x;
        vy_ = msg.y;
        vz_ = msg.F;
        wz_ = msg.z;
    }
    else
    {
        vx_ = 0.0;
        vy_ = 0.0;
        vz_ = 0.0;
        wz_ = 0.0;
    }
}


void ArucoLocalizer::mavrosVelCallback(const mavros_msgs::PositionTarget& msg)
{
    if (status_ == "IBVS")
    {
        // what comes in is in ENU but we want it in NED
        vx_ = msg.velocity.y;
        vy_ = msg.velocity.x;
        vz_ = -msg.velocity.z;
        wz_ = -msg.yaw_rate;
    }
    else
    {
        vx_ = 0.0;
        vy_ = 0.0;
        vz_ = 0.0;
        wz_ = 0.0;
    }
}


// ----------------------------------------------------------------------------
void ArucoLocalizer::get_R_from_quat()
{
    q_wx_ = q_w_*q_x_;
    q_wy_ = q_w_*q_y_;
    q_wz_ = q_w_*q_z_;
    q_xx_ = q_x_*q_x_;
    q_xy_ = q_x_*q_y_;
    q_xz_ = q_x_*q_z_;
    q_yy_ = q_y_*q_y_;
    q_yz_ = q_y_*q_z_;
    q_zz_ = q_z_*q_z_;

    mQmatrix_(0,0) = 1.0 - 2.0*q_yy_ - 2.0*q_zz_;
    mQmatrix_(0,1) = 2.0*q_xy_ + 2.0*q_wz_;
    mQmatrix_(0,2) = 2.0*q_xz_ - 2.0*q_wy_;
    mQmatrix_(1,0) = 2.0*q_xy_ - 2.0*q_wz_;
    mQmatrix_(1,1) = 1.0 - 2.0*q_xx_ - 2.0*q_zz_;
    mQmatrix_(1,2) = 2.0*q_yz_ + 2.0*q_wx_;
    mQmatrix_(2,0) = 2.0*q_xz_ + 2.0*q_wy_;
    mQmatrix_(2,1) = 2.0*q_yz_ - 2.0*q_wx_;
    mQmatrix_(2,2) = 1.0 - 2.0*q_xx_ - 2.0*q_yy_;

    // return np.array([[1. - 2.*yy - 2.*zz, 2.*xy + 2.*wz, 2.*xz - 2.*wy],
    //                      [2.*xy - 2.*wz, 1. - 2.*xx - 2.*zz, 2.*yz + 2.*wx],
    //                      [2.*xz + 2.*wy, 2.*yz - 2.*wx, 1. - 2.*xx - 2.*yy]])

    // return mQmatrix_;
}

// ----------------------------------------------------------------------------

aruco::CameraParameters ArucoLocalizer::ros2arucoCamParams(const sensor_msgs::CameraInfoConstPtr& cinfo) {
    cv::Mat cameraMatrix(3, 3, CV_64FC1);
    cv::Mat distortionCoeff(4, 1, CV_64FC1);
    cv::Size size(cinfo->height, cinfo->width);

    // Make a regular 3x3 K matrix from CameraInfo
    for(int i=0; i<9; ++i)
        cameraMatrix.at<double>(i%3, i-(i%3)*3) = cinfo->K[i];

    // The ArUco library requires that there are only 4 distortion params (k1, k2, p1, p2, 0) 
    if (cinfo->D.size() == 4 || cinfo->D.size() == 5) {

        // Make a regular 4x1 D matrix from CameraInfo
        for(int i=0; i<4; ++i)
            distortionCoeff.at<double>(i, 0) = cinfo->D[i];

    } else {

        ROS_WARN("[aruco] Length of distortion matrix is not 4, assuming zero distortion.");
        for(int i=0; i<4; ++i)
            distortionCoeff.at<double>(i, 0) = 0;

    }

    return aruco::CameraParameters(cameraMatrix, distortionCoeff, size);
}

// ----------------------------------------------------------------------------

// From camera frame to ArUco marker
tf::Transform ArucoLocalizer::aruco2tf(const cv::Mat& rvec, const cv::Mat& tvec) {
    // convert tvec to a double
    cv::Mat tvec64; tvec.convertTo(tvec64, CV_64FC1);

    // Convert Rodrigues paramaterization of the rotation to quat
    tf::Quaternion q1 = rodriguesToTFQuat(rvec);

    tf::Vector3 origin(tvec64.at<double>(0), tvec64.at<double>(1), tvec64.at<double>(2));

    // The measurements coming from the ArUco lib are vectors from the
    // camera coordinate system pointing at the center of the ArUco board.
    return tf::Transform(q1, origin);
}

// ----------------------------------------------------------------------------

tf::Quaternion ArucoLocalizer::rodriguesToTFQuat(const cv::Mat& rvec) {
    // convert rvec to double
    cv::Mat rvec64; rvec.convertTo(rvec64, CV_64FC1);

    // Unpack Rodrigues paramaterization of the rotation
    cv::Mat rot(3, 3, CV_64FC1);
    cv::Rodrigues(rvec64, rot);

    // Convert OpenCV to tf matrix
    tf::Matrix3x3 tf_rot(rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
                         rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
                         rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2));

    // convert rotation matrix to an orientation quaternion
    tf::Quaternion quat;
    tf_rot.getRotation(quat);

    // For debugging
    // double r, p, y;
    // tf::Matrix3x3(quat).getRPY(r,p,y);
    // std::cout << "RPY: [ " << r*(180.0/M_PI) << ", " << p*(180.0/M_PI) << ", " << y*(180.0/M_PI) << " ]\t";

    return quat;
}

// ----------------------------------------------------------------------------

void ArucoLocalizer::saveInputFrame(const cv::Mat& frame) {
    static unsigned int counter = 0;
    saveFrame(frame, "aruco%03i_in.png", counter++);
}

// ----------------------------------------------------------------------------

void ArucoLocalizer::saveOutputFrame(const cv::Mat& frame) {
    static unsigned int counter = 0;
    saveFrame(frame, "aruco%03i_out.png", counter++);
}

// ----------------------------------------------------------------------------

void ArucoLocalizer::saveFrame(const cv::Mat& frame, std::string format_spec, unsigned int img_num) {
    // Create a filename
    std::stringstream ss;
    char buffer[100];
    sprintf(buffer, format_spec.c_str(), img_num);
    ss << debugImagePath_ << "/" << buffer;
    std::string filename = ss.str();

    // save the frame
    cv::imwrite(filename, frame);
}
}