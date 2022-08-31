/**
 * @file vinodom.hpp
 * @brief Visual innertial odometry based homography decomposition
 * @author Fernando Caballero, fcaballero@us.es
 * @date June 2022
 */

#ifndef __VINODOM_HPP__
#define __VINODOM_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <rclcpp/qos.hpp>
#include <vector>
#include <algorithm>
#include "robustmatcher.hpp"

#include <limits>
#include <Eigen/Core>
#include <Eigen/Dense>

using std::placeholders::_1;

#define DEBUG_VINODOM 1

#define G_ACC 9.8

/**
 * @brief Keypoint comparison auxiliar function to sort the sets of keypoints
 * according to their score
 * @param p1 The first keypoint
 * @param p2 The second keypoint
 * @return Boolean true if p1 > p2
 */
bool score_comparator(const cv::KeyPoint &p1, const cv::KeyPoint &p2)
{
    return p1.response > p2.response;
}

/**
 * @brief Match comparison auxiliar function to sort the sets of matches
 * @param m1 The first match
 * @param m2 The second match
 * @return Boolean true if m1 < m2
 */
bool match_comparator(const cv::DMatch &m1, const cv::DMatch &m2)
{
    return m1.distance < m2.distance;
}

/**
 * @brief KeyFrame struct for storing image, keypoints, descriptors and other relevant information
 */
struct KeyFrame
{
    cv::Mat img;                    // Image
    std::vector<cv::KeyPoint> kpts; // Detected key-points
    cv::Mat desc;                   // Key-points descriptors
    cv::Mat H;                      // Chained homography to the first keyframe
    double height;                  // Plane distance
    tf2::Transform tf;              // Transform in base frame at this KF
};

/**
 * @class VinOdom
 * @brief Visual innertial odometry based on homography decomposition
 */
class VinOdom : public rclcpp::Node
{
public:
    /**
     * @brief Constructor
     * @param nodeName Node name for publishing topics
     * @param cameraTopic Name of the camera
     */
    VinOdom() : Node("vinodom")
    {
        // Declare node parameters
        this->declare_parameter<std::string>("camera_topic", "/camera");
        this->declare_parameter<std::string>("imu_topic", "/imu");
        this->declare_parameter<std::string>("altimeter_topic", "/altimeter");
        this->declare_parameter<std::string>("lidar3d_altimeter_topic", "/scan");
        this->declare_parameter<std::string>("barometer_topic", "/barometer");
        this->declare_parameter<std::string>("odom_topic", "/odom");
        this->declare_parameter<std::string>("odom_frame", "/odom");
        this->declare_parameter<std::string>("base_frame", "/base_link");
        this->declare_parameter<int>("max_features", 1000);
        this->declare_parameter<int>("min_matches", 20);
        this->declare_parameter<int>("min_score_detector", 1);
        this->declare_parameter<int>("key_frame_th", 100);
        this->declare_parameter<bool>("show_matching", false);
        this->declare_parameter<double>("min_plane_dist", 1.0);
        this->declare_parameter<double>("init_x", 0.0);
        this->declare_parameter<double>("init_y", 0.0);
        this->declare_parameter<double>("init_z", 0.0);
        this->declare_parameter<bool>("override_height_with_bar", true);
        this->declare_parameter<bool>("start_landed", true);
        this->declare_parameter<bool>("pure_innertial", false);

        // Read parameters
        this->get_parameter("camera_topic", camTopic_);
        this->get_parameter("imu_topic", imuTopic_);
        this->get_parameter("altimeter_topic", altTopic_);
        this->get_parameter("lidar3d_altimeter_topic", lidar3dAltTopic_);
        this->get_parameter("barometer_topic", barTopic_);
        this->get_parameter("odom_topic", odomTopic_);
        this->get_parameter("odom_frame", odomFrame_);
        this->get_parameter("base_frame", baseFrame_);
        this->get_parameter("max_features", maxFeatures_);
        this->get_parameter("min_matches", minMatches_);
        this->get_parameter("min_score_detector", minScoreDetector_);
        this->get_parameter("key_frame_th", keyFrameTh_);
        this->get_parameter("show_matching", showMatching_);
        this->get_parameter("min_plane_dist", minPlaneDist_);
        this->get_parameter("init_x", initX_);
        this->get_parameter("init_y", initY_);
        this->get_parameter("init_z", initZ_);
        this->get_parameter("override_height_with_bar", overrideHeighWithBar_);
        this->get_parameter("start_landed", startLanded_);
        this->get_parameter("pure_inertial", pureInertial_);

        // Check topic name format
        if (camTopic_.back() == '/')
            camTopic_.pop_back();
        if (imuTopic_.back() == '/')
            imuTopic_.pop_back();
        if (altTopic_.back() == '/')
            altTopic_.pop_back();
        if (lidar3dAltTopic_.back() == '/')
            lidar3dAltTopic_.pop_back();
        if (odomTopic_.back() == '/')
            odomTopic_.pop_back();
        if (barTopic_.back() == '/')
            barTopic_.pop_back();

        // init variables
        haveCalibration_ = false;
        haveImu_ = false;
        biasComputed_ = false;
        biasCount_ = 0;
        axBias_ = ayBias_ = azBias_ = 0.0;
        haveKFrame_ = false;
        haveAlt_ = false;
        haveBar_ = false;
        haveBarLanded_ = false;

        // Topic subscription
        imgSub_ = this->create_subscription<sensor_msgs::msg::Image>(camTopic_ + "/image_raw", 10, std::bind(&VinOdom::imageCallback, this, _1));
        cInfoSub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(camTopic_ + "/camera_info", rclcpp::SensorDataQoS(), std::bind(&VinOdom::cInfoCallback, this, _1));
        imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>(imuTopic_, rclcpp::SensorDataQoS(), std::bind(&VinOdom::imuCallback, this, _1));
        altSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(altTopic_, rclcpp::SensorDataQoS(), std::bind(&VinOdom::altCallback, this, _1));
        barSub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(barTopic_, rclcpp::SensorDataQoS(), std::bind(&VinOdom::barCallback, this, _1));
        lid3dAltSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(lidar3dAltTopic_, rclcpp::SensorDataQoS(), std::bind(&VinOdom::lidar3dAltCallback, this, _1));

        // Topic publication
        odomPub_ = this->create_publisher<nav_msgs::msg::Odometry>(odomTopic_, 10);

        // TF listening
        tfCamCatched_ = false;
        tfImuCatched_ = false;
        tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

        // Feature detection and extractor instantation
        fDetector_ = cv::FastFeatureDetector::create(minScoreDetector_);
        fExtractor_ = cv::ORB::create();

        // Setup matcher
        matcher_.setRatioTest();
        matcher_.setRansacTest(RobustMatcher::HOMOGRAPHY, 3, 0.99, true);
    }

    /** @brief Destructor */
    ~VinOdom(void)
    {
    }

private:
    /**
     * @brief IMU callback. It performs innertial integration and stores IMU data to be used by odometry
     * @param msg IMU message
     */
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Time between samples
        static rclcpp::Time t = rclcpp::Time(msg->header.stamp);
        uint64_t dT = (rclcpp::Time(msg->header.stamp) - t).nanoseconds();
        t = rclcpp::Time(msg->header.stamp);

        // Pre-catch transform from imu to base frame (this is done just once!)
        if (!tfImuCatched_)
        {
            try
            {
                geometry_msgs::msg::TransformStamped tf;
                tf = tfBuffer_->lookupTransform(baseFrame_, msg->header.frame_id, tf2::TimePointZero);
                tf2::fromMsg(tf, imuBaseTf);
                tfImuCatched_ = true;
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s",
                             baseFrame_.c_str(), msg->header.frame_id.c_str(), ex.what());
                return;
            }
        }

        // Get orientation in base frame
        imuQ_ = imuBaseTf.getRotation() * tf2::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        imuQ_.normalize();
        haveImu_ = true;
#if DEBUG_VINODOM == 1
        RCLCPP_INFO_ONCE(this->get_logger(), "Have imu");
#endif

        // We do not process acc if it is garbage
        if(tf2::Vector3(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z).length() > 2*G_ACC)
            return;

        // Computes acc BIAS, simple way
        if(!biasComputed_)
        {
            axBias_ +=  msg->linear_acceleration.x;
            ayBias_ +=  msg->linear_acceleration.y;
            azBias_ +=  msg->linear_acceleration.z - G_ACC;
            biasCount_++;

            if(biasCount_ == 100)
            {
                axBias_ = axBias_/100;
                ayBias_ = ayBias_/100;
                azBias_ = azBias_/100;
                vx_ = 0.0;
                vy_ = 0.0;
                tx_ = initX_;
                ty_ = initY_;
                biasComputed_ = true;
            }
            else    
                return;
        }

        // Integrate accelerations, removing computed biases
        tf2::Transform innertialTf;
        innertialTf.setRotation(imuQ_);
        tf2::Vector3 acc = innertialTf * tf2::Vector3(msg->linear_acceleration.x-axBias_, msg->linear_acceleration.y-ayBias_, msg->linear_acceleration.z-azBias_);
        //std::cout << "acc: " << acc.getX() << ", " << acc.getY() << ", " << acc.getZ() << std::endl;
        //std::cout << "acc mod: " << acc.length() << std::endl;

        // Double integrates acceleration
        double delta = dT/1000000000.0, mod;
        vx_ = vx_ + delta*acc.getX();
        vy_ = vy_ + delta*acc.getY();
        tx_ = tx_ + delta*vx_;
        ty_ = ty_ + delta*vy_;
        mod = sqrt(vx_*vx_ + vy_*vy_);
        if(mod > 5.0)
        {
            vx_ = 5.0*vx_/mod;
            vy_ = 5.0*vy_/mod;
        }
    }

    /**
     * @brief Laser alt callback.
     * @param msg Laser message with altimeters
     */
    void altCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
    	height_ = std::numeric_limits<double>::max();
    	std::vector<Eigen::Vector3d> points;
    	
    	for (unsigned int i=0; i < msg->ranges.size();i++)
    	{
    		double auxRange = msg->ranges[i];
    		
    		if(auxRange > minPlaneDist_*0.8 && auxRange < msg->range_max)
    		{
    			Eigen::Vector3d p;
    		
    			double angle = msg->angle_min + i* msg->angle_increment;
    		
    			p(0,0) = auxRange*std::cos(angle);
    			p(1,0) = auxRange*std::sin(angle);
    			p(2,0) = 0.0;
    			
    			points.push_back(p);
    		
       		    if(auxRange < height_ && angle > -0.34 && angle < 0.34)
            		height_ = auxRange;
       	    }
	    }
	
        /*
        //Fit 3D line to the points
        std::pair < Eigen::Vector3d, Eigen::Vector3d > r = best_line_from_points(points);
        
        //Compute distance to the line. The sensor is at the origin
        Eigen::Vector3d dx = r.first - (r.first.dot(r.second))*r.second;
        double distance = std::sqrt(dx.dot(dx)) + 0.62;*/
        
        if(height_ < msg->range_max)
            height_ += 0.62;
        
        haveAlt_ = true;
#if DEBUG_VINODOM == 1
        RCLCPP_INFO_ONCE(this->get_logger(), "Have Altimeter by Lidar2D");
#endif
    }

    /**
     * @brief Lidar3d alt callback.
     * @param msg Laser message with altimeters
     */
    void lidar3dAltCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        height_ = std::numeric_limits<double>::max();    

    	for (unsigned int i=0; i < msg->ranges.size();i++)
    	{
    		double range = msg->ranges[i];
            double angle = msg->angle_min + i* msg->angle_increment;
    		
    		if(range > minPlaneDist_*0.8 && range < msg->range_max && range < height_ &&
               angle > -0.34 && angle < 0.34) // angle between -20 and 20 degrees 
    		{
            	height_ = range; 
       	    }
	    }
        
        if(height_ < msg->range_max)
            height_ += 0.62;

        haveAlt_ = true;
#if DEBUG_VINODOM == 1
        RCLCPP_INFO_ONCE(this->get_logger(), "Have Altimeter by Lidar3D");
#endif
    }

    /**
     * @brief Barometer callback.
     * @param msg Laser message with altimeters
     */
    void barCallback(const sensor_msgs::msg::FluidPressure::SharedPtr msg)
    {
        barHeigh_ = 0.3048 * 145366.45 * (1 - pow(msg->fluid_pressure * 0.01 / 1013.25, 0.190284));
        haveBar_ = true;
        if (!haveBarLanded_)
        {
            barHeighLanded_ = barHeigh_;
            haveBarLanded_ = true;
        }
#if DEBUG_VINODOM == 1
        RCLCPP_INFO_ONCE(this->get_logger(), "Have Barometer");
#endif
    }

    /**
     * @brief RGB image callback
     * @param msg RGB image message
     */
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Check pre-conditions
        if (!haveCalibration_ || !haveImu_ || !haveAlt_ || !haveBar_)
            return;

        // Pre-catch transform from camera to base frame (this is done just once!)
        if (!tfCamCatched_)
        {
            try
            {
                geometry_msgs::msg::TransformStamped tf;
                tf = tfBuffer_->lookupTransform(baseFrame_, msg->header.frame_id, tf2::TimePointZero);
                tf2::fromMsg(tf, camBaseTf);
                tfCamCatched_ = true;

                // PATCH: Add camera to camera link because simulator does not include it
                tf2::Matrix3x3 R(0, 0, 1, -1, 0, 0, 0, -1, 0);
                tf2::Transform camLink, aux;
                camLink.setIdentity();
                camLink.setBasis(R);
                aux = camBaseTf * camLink;
                camBaseTf.setData(aux);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s",
                             baseFrame_.c_str(), msg->header.frame_id.c_str(), ex.what());
                return;
            }
        }

        // Get the distance to plane. It will take the shoterst one between altimeter and barometer
        // Altimeter provides inf over the seawater.
        // Barometer should provide altitude over the sea level.
        bool overTheSea  = false;
        double planeDist = height_;
        if (height_ > barHeigh_)
        {
            planeDist = barHeigh_;
            overTheSea = true;
        }

        // If the odom is pure inertial, just set the overTheSea flag 
        if(pureInertial_)
            overTheSea = true;

        // Convert to OpenCV format
        cv_bridge::CvImageConstPtr cvbImg;
        try
        {
            cvbImg = cv_bridge::toCvCopy(msg, "bgr8");
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Detect key-points in the image
        std::vector<cv::KeyPoint> kpts;
        selectKeypoints(cvbImg->image, kpts);

        // Extract feature descritors from image
        cv::Mat desc;
        fExtractor_->compute(cvbImg->image, kpts, desc);

        // Check we have a key frame
        if (!haveKFrame_)
        {
            cvbImg->image.copyTo(kFrame_.img);
            desc.copyTo(kFrame_.desc);
            kFrame_.kpts = kpts;
            kFrame_.H = cv::Mat::eye(3, 3, CV_64FC1);
            kFrame_.height = planeDist;
            kFrame_.tf.setRotation(imuQ_);
            if (overrideHeighWithBar_)
                kFrame_.tf.setOrigin(tf2::Vector3(initX_, initY_, barHeigh_));
            else
                kFrame_.tf.setOrigin(tf2::Vector3(initX_, initY_, initZ_));
            haveKFrame_ = true;

            return;
        }

        // We cannot compute odometry if we are bellow a given distance to the floor
        // due to camera focus blurring. In this case we just publish
        // the orientation from IMU and integrates accelerations for pose update
        if (startLanded_ && barHeigh_ - barHeighLanded_ < minPlaneDist_)
        {
            nav_msgs::msg::Odometry odomMsg;
            odomMsg.header.stamp = msg->header.stamp;
            odomMsg.header.frame_id = odomFrame_;
            odomMsg.child_frame_id = baseFrame_;
            odomMsg.pose.pose.position.x = tx_;
            odomMsg.pose.pose.position.y = ty_;
            if (overrideHeighWithBar_)
                odomMsg.pose.pose.position.z = barHeigh_;
            else
                odomMsg.pose.pose.position.z = kFrame_.tf.getOrigin().getZ();
            odomMsg.pose.pose.orientation.x = imuQ_.getX();
            odomMsg.pose.pose.orientation.y = imuQ_.getY();
            odomMsg.pose.pose.orientation.z = imuQ_.getZ();
            odomMsg.pose.pose.orientation.w = imuQ_.getW();
            odomPub_->publish(odomMsg);

            return;
        }

        // Match with respect to key-frame
        std::vector<cv::DMatch> matches;
        cv::Mat H = matcher_.match(kpts, desc, kFrame_.kpts, kFrame_.desc, matches);
        if (H.empty())
        {
            cvbImg->image.copyTo(kFrame_.img);
            desc.copyTo(kFrame_.desc);
            kFrame_.kpts = kpts;
            kFrame_.height = planeDist;
            kFrame_.tf.setRotation(imuQ_);
            RCLCPP_INFO(this->get_logger(), "Tracking error. Resetting keyframe!");

            return;
        }
        H = H.inv(); // Homography inversion to compute chain to key-frame

        // Homography decomposition.
        std::vector<cv::Mat> Rs_decomp, ts_decomp, normals_decomp;
        int solutions = cv::decomposeHomographyMat(H, K_, Rs_decomp, ts_decomp, normals_decomp);

        // We get the solution with the normal closest to (0,0,-1)
        int idx = -1;
        double min = 100000, d;
        for (int i = 0; i < solutions; i++)
        {
            d = pow(normals_decomp[i].at<double>(0, 0), 2) +
                pow(normals_decomp[i].at<double>(1, 0), 2) +
                pow(normals_decomp[i].at<double>(2, 0) + 1, 2);
            if (d < min)
            {
                idx = i;
                min = d;
            }
        }

        // Compute odometry step with respecto to key-frame on camera frame
        tf2::Transform stepCamTf;
        cv::Mat t = kFrame_.height * ts_decomp[idx];
        openCVToTf(t, Rs_decomp[idx], stepCamTf);

        // Transform odometry step to base frame
        tf2::Transform stepBaseTf;
        stepBaseTf = camBaseTf * stepCamTf * camBaseTf.inverse();

        // Concatenate with odometry stored in key-frame (which is in base frame)
        tf2::Transform odomTf;
        odomTf = kFrame_.tf * stepBaseTf;

        // Loose coupled IMU integration (in base frame) to avoid orientation drifting
        odomTf.setRotation(imuQ_);

        // Overrides Z estimation with barometric altitude
        if (overrideHeighWithBar_)
        {
            tf2::Vector3 v = odomTf.getOrigin();
            v.setZ(barHeigh_);
            odomTf.setOrigin(v);
        }

        // If we are over the sea, we get the position diretly from the IMU integration
        // Otherwise, we keep updated the position integration
        if(overTheSea)
        {
            tf2::Vector3 v = odomTf.getOrigin();
            v.setX(tx_);
            v.setY(ty_);
            odomTf.setOrigin(v);
        }
        else
        {
            tx_ = odomTf.getOrigin().getX();
            ty_ = odomTf.getOrigin().getY();
        }

        // Build odometry message and publish it
        nav_msgs::msg::Odometry odomMsg;
        odomMsg.header.stamp = msg->header.stamp; 
        odomMsg.header.frame_id = odomFrame_;
        odomMsg.child_frame_id = baseFrame_;
        odomMsg.pose.pose.position.x = odomTf.getOrigin().getX();
        odomMsg.pose.pose.position.y = odomTf.getOrigin().getY();
        odomMsg.pose.pose.position.z = odomTf.getOrigin().getZ();
        odomMsg.pose.pose.orientation.x = odomTf.getRotation().getX();
        odomMsg.pose.pose.orientation.y = odomTf.getRotation().getY();
        odomMsg.pose.pose.orientation.z = odomTf.getRotation().getZ();
        odomMsg.pose.pose.orientation.w = odomTf.getRotation().getW();
        odomPub_->publish(odomMsg);

        // Visulize matching
        if (showMatching_)
        {
            cv::Mat imgMatches;
            cv::drawMatches(cvbImg->image, kpts, kFrame_.img, kFrame_.kpts, matches, imgMatches, cv::Scalar::all(-1),
                            cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
            cv::imshow("Matches", imgMatches);
            cv::waitKey(5);
        }

        // Update key frame if matches are bellow a reasonable number
        if (matches.size() < (long unsigned int)keyFrameTh_)
        {
            cvbImg->image.copyTo(kFrame_.img);
            desc.copyTo(kFrame_.desc);
            kFrame_.kpts = kpts;
            kFrame_.H = kFrame_.H * H;
            kFrame_.height = planeDist;
            kFrame_.tf = odomTf;
        }
    }

    /**
     * @brief Camera calibration info callback
     * @param msg Camera calibration message
     */
    void cInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (!haveCalibration_)
        {
            K_ = cv::Mat(3, 3, CV_64FC1, (void *)msg->k.data()).clone();
            D_ = cv::Mat(msg->d.size(), 1, CV_64FC1, (void *)msg->d.data()).clone();
            imgSize_.width = msg->width;
            imgSize_.height = msg->height;
#if DEBUG_VINODOM == 1
            RCLCPP_INFO_ONCE(this->get_logger(), "Have calibration");
#endif
            haveCalibration_ = true;
        }
    }

    /** @brief Converts OpenCV translation and rotation into tf2::Transform
     * @param[in] t OpenCV translation vector
     * @param[in] R OpenCV rotation matrix
     * @param[out] transform ROS tf2::Transform element
     */
    void openCVToTf(const cv::Mat &t, const cv::Mat &R, tf2::Transform &tf)
    {
        tf2::Vector3 translation_tf(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0));

        tf2::Matrix3x3 rotation_tf;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                rotation_tf[i][j] = R.at<double>(i, j);

        tf.setOrigin(translation_tf);
        tf.setBasis(rotation_tf);
    }

    /** @brief Print on scree the content of a tf2 transform
     * @param[in] tf ROS tf2::Transform element
     */
    void printTf(const tf2::Transform &tf)
    {
        tf2::Vector3 t = tf.getOrigin();
        tf2::Matrix3x3 R = tf.getBasis();

        std::cout << R[0][0] << "\t" << R[0][1] << "\t" << R[0][2] << "\t" << t.getX() << std::endl;
        std::cout << R[1][0] << "\t" << R[1][1] << "\t" << R[1][2] << "\t" << t.getY() << std::endl;
        std::cout << R[2][0] << "\t" << R[2][1] << "\t" << R[2][2] << "\t" << t.getZ() << std::endl;
    }

    /**
     * @brief Distribute max_features among all buckets (six buckets, 2rows x 3cols)
     * @param srcKpts Sorted key-points
     * @param dstKpts Output bucketed key-points
     */
    void kptsBucketing(std::vector<cv::KeyPoint> &srcKpts, std::vector<cv::KeyPoint> &dstKpts)
    {
        const int maxFeatBuck = maxFeatures_ / 6;
        int buckets[6] = {maxFeatBuck, maxFeatBuck, maxFeatBuck, maxFeatBuck, maxFeatBuck, maxFeatBuck};
        dstKpts.clear();
        for (size_t i = 0; i < srcKpts.size(); i++)
        {
            int id;
            if (srcKpts[i].pt.y <= imgSize_.height / 2)
                id = 0;
            else
                id = 3;
            if (srcKpts[i].pt.x <= imgSize_.width / 3)
                id += 0;
            else if (srcKpts[i].pt.x <= 2 * imgSize_.width / 3)
                id += 1;
            else
                id += 2;

            if (buckets[id] > 0)
            {
                buckets[id]--;
                dstKpts.push_back(srcKpts[i]);
            }
        }
    }

    /**
     * @brief Detects key-points in the RGB image, applying bucketing
     * @param img Input image
     * @param kpts Key-points extracted from input image
     */
    void selectKeypoints(const cv::Mat &img, std::vector<cv::KeyPoint> &kpts)
    {
        // Detect key-points in the image
        std::vector<cv::KeyPoint> kpts_all;
        fDetector_->detect(img, kpts_all);

        // Sort keypoints according to their score
        std::sort(kpts_all.begin(), kpts_all.end(), score_comparator);

        // Distribute maxFeatures_ among buckets
        kptsBucketing(kpts_all, kpts);
    }

    /** @brief Compute feature flow between matched key-points
     * @param matches List of matches
     * @param trainKpts Key-points from 'train' image
     * @param queryKpts Key-points fraom 'query' image
     * @return Computed feature flow
     */
    double computeFeatureFlow(const std::vector<cv::DMatch> &matches,
                              const std::vector<cv::KeyPoint> &trainKpts,
                              const std::vector<cv::KeyPoint> &queryKpts)
    {
        int idT, idQ;
        double xDiff, yDiff, totalFlow = 0.0;

        for (size_t i = 0; i < matches.size(); i++)
        {
            idT = matches[i].trainIdx;
            idQ = matches[i].queryIdx;
            xDiff = trainKpts[idT].pt.x - queryKpts[idQ].pt.x;
            yDiff = trainKpts[idT].pt.y - queryKpts[idQ].pt.y;
            totalFlow += sqrt(xDiff * xDiff + yDiff * yDiff);
        }

        return totalFlow / matches.size();
    }
    
    
   // template<class Vector3>
	std::pair < Eigen::Vector3d, Eigen::Vector3d > best_line_from_points(const std::vector<Eigen::Vector3d> & c)
	{
		// copy coordinates to  matrix in Eigen format
		size_t num_atoms = c.size();
		Eigen::Matrix< Eigen::Vector3d::Scalar, Eigen::Dynamic, Eigen::Dynamic > centers(num_atoms, 3);
		for (size_t i = 0; i < num_atoms; ++i) centers.row(i) = c[i];

		Eigen::Vector3d origin = centers.colwise().mean();
		Eigen::MatrixXd centered = centers.rowwise() - origin.transpose();
		Eigen::MatrixXd cov = centered.adjoint() * centered;
		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);
		Eigen::Vector3d axis = eig.eigenvectors().col(2).normalized();

		return std::make_pair(origin, axis);
	}

    // Data subscribers
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cInfoSub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgSub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr altSub_;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr barSub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lid3dAltSub_;

    // Camera calibration information
    cv::Size imgSize_;
    cv::Mat K_, D_;
    bool haveCalibration_;

    // IMU data
    bool haveImu_, biasComputed_;
    tf2::Quaternion imuQ_;
    double axBias_, ayBias_, azBias_;
    int biasCount_;
    double vx_, vy_, tx_, ty_;

    // Altimeter data
    bool haveAlt_, haveBar_, haveBarLanded_;
    double height_, barHeigh_, barHeighLanded_;

    // Feature detection, extractor
    cv::Ptr<cv::FastFeatureDetector> fDetector_;
    cv::Ptr<cv::ORB> fExtractor_;
    RobustMatcher matcher_;

    // Key-frame info
    bool haveKFrame_;
    KeyFrame kFrame_;

    // Node parameters
    int maxFeatures_, minMatches_, minScoreDetector_, keyFrameTh_;
    std::string camTopic_, imuTopic_, altTopic_, odomTopic_, odomFrame_, baseFrame_, barTopic_, lidar3dAltTopic_;
    double minPlaneDist_, initX_, initY_, initZ_;
    bool overrideHeighWithBar_, startLanded_, pureInertial_;

    // Current odometry computation
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;

    // Sensor TFs
    bool tfCamCatched_, tfImuCatched_;
    tf2::Stamped<tf2::Transform> camBaseTf, imuBaseTf;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;

    // Debug info
    bool showMatching_;
};

#endif
