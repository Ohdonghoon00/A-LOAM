#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <unistd.h>
#include <stdio.h>
#include <algorithm>
#include <chrono>
#include <sstream>
#include <boost/program_options.hpp>
#include <glog/logging.h>

#include <rosbag/bag.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>


#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "common.h"
#include "aloam/feature_info.h"



ros::Publisher pubLaserCloud;
// ros::Publisher pubfeature;

using namespace std;

namespace po = boost::program_options;
bool publish = true;
int cnt = 0;

// Extrinsic parameter rig - imu / rig - lidar
const Eigen::Matrix4f IMUToRig = To44RT(imu2rig_pose);
const Eigen::Matrix4f LidarToRig_ = To44RT(lidar2rig_pose);
Eigen::Matrix4d LidarToRig = LidarToRig_.cast<double>();


struct LidarData 
{

    std::vector<char> binary_data;
    int64_t timestamp_ns;
    int num_points, num_blocks;
    uint8_t num_channels;

    
    LidarData()
        : num_points(0), num_blocks(0), num_channels(0) {}
    virtual ~LidarData() {}
    
    
    float* points_ptr() const { return (float*) binary_data.data(); }
    uint8_t* intensities_ptr() const { return (uint8_t*) &binary_data[3 * num_points * sizeof(float)]; } // reflectivity
    uint8_t* azimuth_idxs_ptr() const { return intensities_ptr() + num_points; }
    float* azimuth_degs_ptr() const { return (float*) (azimuth_idxs_ptr() + num_points); }
    
     

};




Vector6f to6DOF(Eigen::Matrix4f RT)
{
    float data[] = {    RT(0, 0), RT(0, 1), RT(0, 2),
                        RT(1, 0), RT(1, 1), RT(1, 2),
                        RT(2, 0), RT(2, 1), RT(2, 2)};
    cv::Mat rot(3, 3, CV_32FC1, data);
    cv::Rodrigues(rot, rot);

    Vector6f a;
    a << rot.at<float>(0, 0), rot.at<float>(1, 0), rot.at<float>(2, 0), RT(0, 3), RT(1, 3), RT(2, 3);

    return a;                          
}

Eigen::Matrix4f gyroToRotation(Eigen::Vector3f gyro)
{
    float t = 0.005; // 200Hz
    float angle_x(gyro[0] * t), angle_y(gyro[1] * t), angle_z(gyro[2] * t);  
    
    float data_x[] = {  1.0, 0.0, 0.0,
                        0.0, cos(angle_x), sin(angle_x),
                        0.0, -sin(angle_x), cos(angle_x)};

    float data_y[] = {  cos(angle_y), 0.0, -sin(angle_y),
                        0.0, 1.0, 0.0,
                        sin(angle_y), 0.0, cos(angle_y)};

    float data_z[] = {  cos(angle_z), sin(angle_z), 0.0,
                        -sin(angle_z), cos(angle_z), 0.0,
                        0.0, 0.0, 1.0};

    cv::Mat R_x( 3, 3, CV_32FC1, data_x);
    cv::Mat R_y( 3, 3, CV_32FC1, data_y);
    cv::Mat R_z( 3, 3, CV_32FC1, data_z);
    cv::Mat R = R_x * R_y * R_z;




    Eigen::Matrix4f RT;
    RT << R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2), 0,
                R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2), 0,
                R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2), 0,
                0,                  0,                  0,              1;

    return RT;
}



void MoveDistortionPoints(std::vector<Eigen::Vector3d> &points, const Eigen::Matrix4f LidarRotation, int ScanStepNum, int num_seqs)
{
    int PointNum = points.size();
    Eigen::MatrixXd MatrixPoints(4, PointNum);
    for(int i = 0; i < PointNum; i++){
        MatrixPoints(0, i) = points[i].x();
        MatrixPoints(1, i) = points[i].y();
        MatrixPoints(2, i) = points[i].z();
        MatrixPoints(3, i) = 1.0;
    }
    
    float angle = ToAngle(LidarRotation);
    Eigen::Vector3f Axis = ToAxis(LidarRotation);
    
    float AngleRatio = (((float)(ScanStepNum + 1) / (float)num_seqs) * angle);
    Axis = (Axis * AngleRatio);
    
    double data[] = {(double)Axis(0, 0), (double)Axis(1, 0), (double)Axis(2, 0)};
    cv::Mat R(3, 1, CV_64FC1, data);
    cv::Rodrigues(R, R);

    Eigen::Matrix4d RT;
    RT <<   R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), LidarRotation(0, 3),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), LidarRotation(1, 3),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), LidarRotation(2, 3),
            0,                  0,                  0,              1;


    Eigen::Matrix4Xd MatrixPoints_ = RT.inverse() * MatrixPoints;
    points.clear();
    for(int i = 0; i < PointNum; i++){
        Eigen::Vector3d Point;
        Point.x() = MatrixPoints_(0, i);
        Point.y() = MatrixPoints_(1, i);
        Point.z() = MatrixPoints_(2, i);
        points.push_back(Point);
    }

}

int ReadIMUdata(const std::string Path, std::vector<double> *IMUtimestamps, std::vector<Eigen::Vector3f> *IMUGyros)
{
    std::ifstream IMUcsvFile(Path, std::ifstream::in);

    if(!IMUcsvFile.is_open()){
        std::cout << " IMU csv file failed to open " << std::endl;
        return EXIT_FAILURE;
    }

    std::string IMUcsvline;
    int IMUline_num = 0;
    

    
    // Read IMU Data csv
    while(std::getline(IMUcsvFile, IMUcsvline) && ros::ok())
    {
        if(IMUline_num == 0){
            IMUline_num++;
            continue;
        }
            
        std::string IMUvalue;
        std::vector<std::string> IMUvalues;
            
        // IMUvalues[0] -> Timestamp (ns)
        // IMUvalues[1] -> Gyro_x
        // IMUvalues[2] -> Gyro_y
        // IMUvalues[3] -> Gyro_z
        // IMUvalues[4] -> Acc_x
        // IMUvalues[5] -> Acc_y
        // IMUvalues[6] -> Acc_z

        std::stringstream ss(IMUcsvline);
        while(std::getline(ss, IMUvalue, ','))
            IMUvalues.push_back(IMUvalue);
        // std::cout << " IMUline num : " << IMUline_num << "th    ";
            
        IMUtimestamps->push_back(std::stod(IMUvalues[0]) * 10e9);
        // std::cout.precision(15);
        // std::cout << " IMU timestamp : " << IMUtimestamp << std::endl;

        Eigen::Vector3f Gyro;
        Gyro.resize(3);
        Gyro[0] = std::stof(IMUvalues[1]);
        Gyro[1] = std::stof(IMUvalues[2]);
        Gyro[2] = std::stof(IMUvalues[3]);
        IMUGyros->push_back(Gyro);
         
             
            
        std::cout << std::endl;
        IMUline_num++;
    } 

    IMUcsvFile.close();
}

int ReadVIOdata(const std::string Path, std::vector<double> *VIOtimestamps, std::vector<Vector6d> *VIORidarPoses)
{
    std::ifstream Rovins2PoseFile(Path, std::ifstream::in);

    if(!Rovins2PoseFile.is_open()){
        std::cout << " Rovins2Pose file failed to open " << std::endl;
        return EXIT_FAILURE;
    }

    std::string line;
    int line_num = 0;

    while(std::getline(Rovins2PoseFile, line) && ros::ok()){

        std::string value;
        std::vector<std::string> values;        

        // values[0]        -> camera fidx
        // values[1] ~ [6]  -> rig pose
        // values[7]        -> timestamps 
        
        std::stringstream ss(line);
        while(std::getline(ss, value, ' '))
            values.push_back(value);

        Vector6d pos;
        pos << std::stod(values[1]), std::stod(values[2]), std::stod(values[3]), std::stod(values[4]), std::stod(values[5]), std::stod(values[6]);
        Eigen::Matrix4d Rigpos = To44RT(pos);
        Eigen::Matrix4d Ridarpos = LidarToRig.inverse() * Rigpos * LidarToRig;

        VIOtimestamps->push_back(std::stod(values[7]) * 10e-10 );
        VIORidarPoses->push_back(To6DOF(Ridarpos));

        line_num++;
    }

    Rovins2PoseFile.close();
}

int ReadSlamdata(const std::string Path, std::vector<double> *SlamKFtimestamps, std::vector<Vector6d> *SlamKFPoses, std::vector<int> *Slamcamidxs)
{
    std::ifstream SlamPoseFile(Path, std::ifstream::in);

    if(!SlamPoseFile.is_open()){
        std::cout << " SlamPoseFile failed to open " << std::endl;
        return EXIT_FAILURE;        
    }

    std::string line;
    int line_num = 0;

    while(std::getline(SlamPoseFile, line) && ros::ok()){

        std::string value;
        std::vector<std::string> values;

        // values[0]        -> camera fidx
        // values[1] ~ [6]  -> slam rig? pose
        // values[7]        -> timestamps

        std::stringstream ss(line);
        while(std::getline(ss, value, ' '))
            values.push_back(value);

        Vector6d pos;
        pos << std::stod(values[1]), std::stod(values[2]), std::stod(values[3]), std::stod(values[4]), std::stod(values[5]), std::stod(values[6]);
        Eigen::Matrix4d Rigpos = To44RT(pos);
        Eigen::Matrix4d Ridarpos = LidarToRig.inverse() * Rigpos * LidarToRig;

        SlamKFtimestamps->push_back(std::stod(values[7]) * 10e-10 );
        SlamKFPoses->push_back(To6DOF(Ridarpos));
        Slamcamidxs->push_back(std::stoi(values[0]));

        line_num++;        
    }
    SlamPoseFile.close();
}

int main(int argc, char **argv) 
{
    
    ros::init(argc, argv, "TestPublishData");
    ros::NodeHandle nh("~");
    LidarData lidar_data;
    
    // launch parameter
    rosbag::Bag bag;
    bool to_bag, ToUndistortionPoints;
    std::string data_dir, output_bag_file, yaml_path;
    int publish_delay;
    
    nh.getParam("data_dir", data_dir);
    nh.getParam("to_bag", to_bag);
    nh.getParam("ToUndistortionPoints", ToUndistortionPoints);
    nh.getParam("publish_delay", publish_delay);
    
    // Read Yaml setting file

    
    // bagfile
    if (to_bag){
        nh.getParam("output_bag_file", output_bag_file);
        bag.open(data_dir + output_bag_file, rosbag::bagmode::Write);
    }
    
    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);

    ros::Publisher pubVIOodometry = nh.advertise<nav_msgs::Odometry>("/VIO_odom_to_init", 100);
    
    ros::Publisher pubVIOPath = nh.advertise<nav_msgs::Path>("/VIO_odom_path", 100);
    nav_msgs::Path VIOPath;
    // pubfeature = nh.advertise<sort_lidarpoints::feature_info>("/velodyne_points", 100);

    // publish delay
    ros::Rate r(10.0 / publish_delay);
    
    // Read IMU data
    std::string IMUcsvPath = data_dir + "imu_data.csv";
    std::vector<double> IMUtimestamps;
    std::vector<Eigen::Vector3f> IMUGyros;
    
    ReadIMUdata(IMUcsvPath, &IMUtimestamps, &IMUGyros);
    
    // VIO data
    std::string Rovins2PosePath = data_dir + "rovins2_all_frames.txt";
    std::vector<double> VIOtimestamps;
    std::vector<Vector6d> VIORidarPoses;
    
    Vector6d p;
    p << 0, 0, 0, 0, 0, 0;
    VIORidarPoses.push_back(p);
    VIOtimestamps.push_back(0.0);

    ReadVIOdata(Rovins2PosePath, &VIOtimestamps, &VIORidarPoses);
    
    // SLAM Pose data ( Keyframe )
    std::string SlamPosePath = data_dir + "slam_poses.txt";
    std::vector<double> SlamKFtimestamps;
    std::vector<Vector6d> SlamKFPoses;
    std::vector<int> Slamcamidxs;

    ReadSlamdata(SlamPosePath, &SlamKFtimestamps, &SlamKFPoses, &Slamcamidxs);

    // Lidar timestamp.csv path
    std::string LidarcsvPath = data_dir + "lidar_timestamp.csv";
    std::ifstream LidarcsvFile(LidarcsvPath, std::ifstream::in);

    if(!LidarcsvFile.is_open()){
        std::cout << " Lidar csv file failed to open " << std::endl;
        return EXIT_FAILURE;
    }
    
    // Read Lidar timestamp.csv
    Eigen::Matrix4f LidarRotation;
    std::string Lidarcsvline;
    int Lidarline_num(0), IMUcount(0);
    
    while(std::getline(LidarcsvFile, Lidarcsvline) && ros::ok())
    {
        if(Lidarline_num == 0){
            Lidarline_num++;
            continue;
        }
        
        std::string value;
        std::vector<std::string> values;
        
        // values[0] -> First Seq Timestamp (ns)
        // values[1] -> Last Seq Timestamp (ns)
        // values[2] -> Fidx
        // values[3] -> Num pts
        // values[4] -> Date
        
        std::stringstream ss(Lidarcsvline);
        while(std::getline(ss, value, ','))
            values.push_back(value);
        int fidx = std::stoi(values[2]);
        double LidarScantimestamp = std::stod(values[1]) * 10e9;

        // Binary Data Path
        std::stringstream Lidar_binary_path;
        Lidar_binary_path <<    data_dir + "lidar/" << 
                                std::setfill('0') << 
                                std::setw(5) << fidx << ".xyz";
        
        std::ifstream ifs(Lidar_binary_path.str(), std::ifstream::in);
        
        if (!ifs.is_open()){
            std::cout << "xyz file failed to open: " << std::endl;
            return EXIT_FAILURE;
        }        

        std::vector<Eigen::Vector3d> PublishPoints;
        // const size_t kMaxNumberOfPoints = 1e6; 
        // PublishPoints.clear();
        // PublishPoints.reserve(kMaxNumberOfPoints);
        
        
        std::cout << " File number : " << fidx << "     " << std::endl;
        
        // Read Binary data file
        int num_seqs = 0;
        ifs.read((char*)&num_seqs, sizeof(int));
        // std::cout << " num_seqs : " << num_seqs << std::endl;

         
        // Integral IMU rotation to one lidar scan
        Eigen::Matrix4f IMURotation_integral = Eigen::Matrix4f::Identity();
        while(LidarScantimestamp > IMUtimestamps[IMUcount]){
            
            Eigen::Matrix4f IMURotation = gyroToRotation(IMUGyros[IMUcount]);
            IMURotation_integral = IMURotation * IMURotation_integral;
            IMUcount++;
        }

        Eigen::Matrix4f RT_ = IMUToRig * IMURotation_integral * IMUToRig.inverse();
        Eigen::Matrix4f RT = LidarToRig_.inverse() * RT_ * LidarToRig_;
        LidarRotation = RT;

        for (int j = 0; j < num_seqs; j++){

            Eigen::Vector3d point;
            std::vector<Eigen::Vector3d> Points;
            
            
            int& num_pts = lidar_data.num_points;
            ifs.read((char*)&num_pts, sizeof(int));
            
            lidar_data.binary_data.resize((4 * num_pts) * sizeof(float) + 2 * num_pts);
            

            ifs.read((char*) lidar_data.points_ptr(), num_pts * sizeof(float) * 3);
            ifs.read((char*) lidar_data.intensities_ptr(), num_pts );
            
            ifs.read((char*) lidar_data.azimuth_idxs_ptr(), num_pts);    
            ifs.read((char*) &lidar_data.num_blocks, sizeof(int));
            CHECK_LE(lidar_data.num_blocks, num_pts);  
            ifs.read((char*) lidar_data.azimuth_degs_ptr(),
                    lidar_data.num_blocks * sizeof(float));
            ifs.read((char*) &lidar_data.num_channels, sizeof(uint8_t));
            ifs.read((char*) &lidar_data.timestamp_ns, sizeof(int64_t));


            // save 3D points and intensity 
            for(int k = 0; k < num_pts * 3; k+=3){
                point.x() = (double)*(lidar_data.points_ptr() + k);
                point.y() = (double)*(lidar_data.points_ptr() + k + 1);
                point.z() = (double)*(lidar_data.points_ptr() + k + 2);
                // point.intensity = (((float)*( lidar_data.intensities_ptr() + (k/3) ) ) / 255); // 0 ~ 1 , raw data : 0 ~ 254
                Points.push_back(point);
            }

            // UndistortionPoints
            if(ToUndistortionPoints) MoveDistortionPoints(Points, LidarRotation, j, num_seqs);

            for(size_t i = 0; i < Points.size(); i ++){
                Eigen::Vector3d NoDistortionPoint;
                NoDistortionPoint.x() = Points[i].x();
                NoDistortionPoint.y() = Points[i].y();
                NoDistortionPoint.z() = Points[i].z();
                    
                PublishPoints.push_back(NoDistortionPoint);
            }        

        }
            
        // timestamp
        ros::Time timestamp_ros;
        timestamp_ros.fromNSec(lidar_data.timestamp_ns);
        double LidarTime = timestamp_ros.toSec();
        
        // find lidartimestamp - cam timestamp
        int Minidx = -1;
        double Mindiff = 1000;
        for(size_t i = 0; i < VIOtimestamps.size(); i++){
            double diff = std::fabs(VIOtimestamps[i] - LidarTime);
            if(diff < Mindiff){
                Mindiff = diff;
                Minidx = i;
            }    
            
        }
        // Eigen::Quaterniond q = ToQuaternion(VIORidarPoses[Minidx]);

        int Minvalue = 100;
        int Minslamidx = -1;
        for(int i = 0; i < Slamcamidxs.size(); i++){
            int idx_diff = std::abs(Slamcamidxs[i] - Minidx);
            if(idx_diff < Minvalue){
                Minvalue = idx_diff;
                Minslamidx = i;
            }
        }
        if(Minvalue < 2){
            Eigen::Quaterniond q;
            Eigen::Vector3d p;
            if(Minvalue == 0){
                q = ToQuaternion(SlamKFPoses[Minslamidx]);
                p << SlamKFPoses[Minslamidx][3], SlamKFPoses[Minslamidx][4], SlamKFPoses[Minslamidx][5];
                std::cout << " lidartimestamp and Keyframe are same timestamp !!!  fidx : " << Slamcamidxs[Minslamidx] << std::endl;
                cnt = 1;
            }
            else{
                q = ToQuaternion(SlamKFPoses[Minslamidx]);
                p << SlamKFPoses[Minslamidx][3], SlamKFPoses[Minslamidx][4], SlamKFPoses[Minslamidx][5];
                cnt++;
                std::cout << " different !! all frames fidx : " << Slamcamidxs[Minslamidx] << std::endl;
            }

            if(cnt % 2 == 1){
        // publish odometry
        std::cout << "Publish!" << std::endl;
        nav_msgs::Odometry VIOodometry;
        VIOodometry.header.frame_id = LidarFrame;
        VIOodometry.child_frame_id = "/laser_odom";
        VIOodometry.header.stamp = timestamp_ros;
        VIOodometry.pose.pose.orientation.x = q.x();
        VIOodometry.pose.pose.orientation.y = q.y();
        VIOodometry.pose.pose.orientation.z = q.z();
        VIOodometry.pose.pose.orientation.w = q.w();
        VIOodometry.pose.pose.position.x = p.x();
        VIOodometry.pose.pose.position.y = p.y();
        VIOodometry.pose.pose.position.z = p.z();
        pubVIOodometry.publish(VIOodometry);

        geometry_msgs::PoseStamped VIOPose;
        VIOPose.header = VIOodometry.header;
        VIOPose.pose = VIOodometry.pose.pose;
        VIOPath.header.stamp = VIOodometry.header.stamp;
        VIOPath.poses.push_back(VIOPose);
        VIOPath.header.frame_id = LidarFrame;
        pubVIOPath.publish(VIOPath);


        // publish
        sensor_msgs::PointCloud2 output;
        // pcl::toROSMsg(PublishPoints, output);
        output = ConverToROSmsg(PublishPoints);
        output.header.stamp = timestamp_ros;
        output.header.frame_id = "/camera_init";
        pubLaserCloud.publish(output);
        
        // Publish IMU Gyro Data
        // Vector6f rt = to6DOF(LidarRotation);
        // PublishIMUandcloud(pubfeature, PublishPoints, rt, timestamp_ros, LidarFrame);
        
        // bagfile
        if( to_bag ) bag.write("/velodyne_points", timestamp_ros, output);
            }

        }
        Lidarline_num++;
        r.sleep();
        ifs.close();   
    }

    LidarcsvFile.close();
            
    std::cout << std::endl;
    if (to_bag) bag.close();
    
    std::cout << "Publish Lidar Data Done ..." << std::endl;
    
    return 0;
}


    

    

    
    






       
            
    

       
            

            


    







             

    



