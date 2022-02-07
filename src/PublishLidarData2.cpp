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
#include <glog/logging.h>

#include <rosbag/bag.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "common.h"



ros::Publisher pubLaserCloud;

using namespace std;

bool publish = true;
int cnt = 0;

// Extrinsic parameter rig - imu / rig - lidar
const Eigen::Matrix4d IMUToRig = To44RT(imu2rig_pose);
const Eigen::Matrix4d LidarToRig = To44RT(lidar2rig_pose);

LidarData lidar_data;
DataBase DB;

    
     


Eigen::Matrix4d gyroToRotation(Eigen::Vector3f gyro)
{
    float t = 0.005; // 200Hz
    float angle_x(gyro[0] * t), angle_y(gyro[1] * t), angle_z(gyro[2] * t);  
    
    Eigen::Matrix3f data_x;
    data_x <<   1.0, 0.0, 0.0,
                0.0, cos(angle_x), sin(angle_x),
                0.0, -sin(angle_x), cos(angle_x);

    Eigen::Matrix3f data_y;
    data_y <<   cos(angle_y), 0.0, -sin(angle_y),
                0.0, 1.0, 0.0,
                sin(angle_y), 0.0, cos(angle_y);

    Eigen::Matrix3f data_z;
    data_z <<   cos(angle_z), sin(angle_z), 0.0,
                -sin(angle_z), cos(angle_z), 0.0,
                0.0, 0.0, 1.0;

    Eigen::Matrix3f rot = data_x * data_y * data_z;
    Eigen::Matrix4d RT;
    RT <<   (double)rot(0, 0), (double)rot(0, 1), (double)rot(0, 2), 0,
            (double)rot(1, 0), (double)rot(1, 1), (double)rot(1, 2), 0,
            (double)rot(2, 0), (double)rot(2, 1), (double)rot(2, 2), 0,
            0,         0,         0,         1;

    return RT;
}

void MoveDistortionPoints(Eigen::Matrix3Xd &points, const Eigen::Matrix4d LidarRotation, int ScanStepNum, int num_seqs)
{
    int PointNum = points.cols();
    Eigen::MatrixXd MatrixPoints(4, PointNum);
    for(int i = 0; i < PointNum; i++){
        MatrixPoints(0, i) = points(0, i);
        MatrixPoints(1, i) = points(1, i);
        MatrixPoints(2, i) = points(2, i);
        MatrixPoints(3, i) = 1.0;
    }
    
    double angle = ToAngle(LidarRotation);
    Eigen::Vector3d Axis = ToAxis(LidarRotation);
    
    double AngleRatio = (((double)(ScanStepNum + 1) / (double)num_seqs) * angle);
    Axis = (Axis * AngleRatio);
    
    Eigen::Matrix3d R = ToMat33(Axis);

    Eigen::Matrix4d RT;
    RT <<   R(0, 0), R(0, 1), R(0, 2), LidarRotation(0, 3),
            R(1, 0), R(1, 1), R(1, 2), LidarRotation(1, 3),
            R(2, 0), R(2, 1), R(2, 2), LidarRotation(2, 3),
            0,       0,       0,       1;

    Eigen::Matrix4Xd MatrixPoints_ = RT.inverse() * MatrixPoints;
    // points.clear();
    for(int i = 0; i < PointNum; i++){
        Eigen::Vector3d Point;
        points(0, i) = MatrixPoints_(0, i);
        points(1, i) = MatrixPoints_(1, i);
        points(2, i) = MatrixPoints_(2, i);
        // points.push_back(Point);
    }

}






int ReadIMUdata(const std::string Path, DataBase* db)
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
            
        db->IMUtimestamps.push_back(std::stod(IMUvalues[0]) * 10e-10);

        Eigen::Vector3f Gyro;
        Gyro.resize(3);
        Gyro[0] = std::stof(IMUvalues[1]);
        Gyro[1] = std::stof(IMUvalues[2]);
        Gyro[2] = std::stof(IMUvalues[3]);
        db->IMUGyros.push_back(Gyro);
         
             
            
        std::cout << std::endl;
        IMUline_num++;
    } 

    IMUcsvFile.close();
    return EXIT_SUCCESS;
}

int ReadVIOdata(const std::string Path, DataBase *db)
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
        Eigen::Matrix4d Lidarpos = LidarToRig.inverse() * Rigpos * LidarToRig;

        db->VIOtimestamps.push_back(std::stod(values[7]) * 10e-10 );
        db->VIOLidarPoses.push_back(To6DOF(Lidarpos));

        line_num++;
    }

    Rovins2PoseFile.close();
    return EXIT_SUCCESS;
}

int ReadSlamdata(const std::string Path, DataBase* db)
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
        Eigen::Matrix4d Lidarpos = LidarToRig.inverse() * Rigpos * LidarToRig;

        db->SlamKFtimestamps.push_back(std::stod(values[7]) * 10e-10 );
        db->SlamKFPoses.push_back(To6DOF(Lidarpos));
        db->Slamcamidxs.push_back(std::stoi(values[0]));

        line_num++;        
    }
    SlamPoseFile.close();
    return EXIT_SUCCESS;
}

int ReadLidardata(const std::string Path, const std::string LidarBinaryPath, DataBase* db, bool ToUndistortionPoints)
{
    std::ifstream LidarcsvFile(Path, std::ifstream::in);

    if(!LidarcsvFile.is_open()){
        std::cout << " Lidar csv file failed to open " << std::endl;
        return EXIT_FAILURE;
    }
    
    // Read Lidar timestamp.csv
    Eigen::Matrix4d LidarRotation;
    std::string Lidarcsvline;
    int Lidarline_num(1), IMUcount(0);
    int cnt_75 = 0;
    int cnt_76 = 0;


    while(std::getline(LidarcsvFile, Lidarcsvline) && ros::ok())
    {
        if(Lidarline_num == 1){
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
        double LidarScantimestamp = std::stod(values[1]) * 10e-10;
        
        // Binary Data Path
        std::stringstream Lidar_binary_path;
        Lidar_binary_path <<    LidarBinaryPath << std::setfill('0') << 
                                std::setw(5) << fidx << ".xyz";
        
        std::ifstream ifs(Lidar_binary_path.str(), std::ifstream::in);
        
        if (!ifs.is_open()){
            std::cout << "xyz file failed to open: " << std::endl;
            return EXIT_FAILURE;
        }        
                
        // Read Binary data file
        int num_seqs = 0;
        ifs.read((char*)&num_seqs, sizeof(int));
        
        // Integral IMU rotation to one lidar scan
        // Eigen::Matrix4d IMURotation_integral = Eigen::Matrix4d::Identity();
        // while(LidarScantimestamp > db->IMUtimestamps[IMUcount]){
        //     Eigen::Matrix4d IMURotation = gyroToRotation(db->IMUGyros[IMUcount]);
        //     IMURotation_integral = IMURotation * IMURotation_integral;
        //     IMUcount++;
        //     if(IMUcount == db->IMUtimestamps.size()) break;
        // }

        // Integral IMU rotation to Camera two frames
        Eigen::Matrix4d IMURotation_integral = Eigen::Matrix4d::Identity();
        while(LidarScantimestamp > db->IMUtimestamps[IMUcount]){
            Eigen::Matrix4d IMURotation = gyroToRotation(db->IMUGyros[IMUcount]);
            IMURotation_integral = IMURotation * IMURotation_integral;
            IMUcount++;
            if(IMUcount == db->IMUtimestamps.size()) break;
        }

        Eigen::Matrix4d RT_ = IMUToRig * IMURotation_integral * IMUToRig.inverse();
        Eigen::Matrix4d RT = LidarToRig.inverse() * RT_ * LidarToRig;
        LidarRotation = RT;

        // std::cout << "File num : " << fidx << std::endl;
        // std::cout << "num : " << num_seqs << std::endl;
        // if(num_seqs == 76) cnt_76++;
        // if(num_seqs == 75) cnt_75++;
        for (int j = 0; j < num_seqs; j++){

            
            int& num_pts = lidar_data.num_points;
            ifs.read((char*)&num_pts, sizeof(int));
            Eigen::Matrix3Xd Points(3, num_pts);
            // std::vector<Eigen::Vector3d> Points;
            
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
                // point.x() = (double)*(lidar_data.points_ptr() + k);
                // point.y() = (double)*(lidar_data.points_ptr() + k + 1);
                // point.z() = (double)*(lidar_data.points_ptr() + k + 2);
                // point.intensity = (((float)*( lidar_data.intensities_ptr() + (k/3) ) ) / 255); // 0 ~ 1 , raw data : 0 ~ 254
                // Points.push_back(point);
                Points(0, k/3) = (double)*(lidar_data.points_ptr() + k);
                Points(1, k/3) = (double)*(lidar_data.points_ptr() + k + 1);
                Points(2, k/3) = (double)*(lidar_data.points_ptr() + k + 2);
            }

            // UndistortionPoints
            if(ToUndistortionPoints) MoveDistortionPoints(Points, LidarRotation, j, num_seqs);

            // for(size_t i = 0; i < Points.size(); i ++){
            //     Eigen::Vector3d NoDistortionPoint;
            //     NoDistortionPoint.x() = Points[i].x();
            //     NoDistortionPoint.y() = Points[i].y();
            //     NoDistortionPoint.z() = Points[i].z();
                    
            //     // PublishPoints.push_back(NoDistortionPoint);
            // }
            double timestamp = lidar_data.timestamp_ns * 10e-10;        
            db->Lidartimestamps.push_back(timestamp);
            db->LidarPoints.push_back(Points);
        }
        double timestamp_ = lidar_data.timestamp_ns * 10e-10;
        db->LidarLastseqtimestamps.push_back(timestamp_);
        // std::cout << setprecision(19) << timestamp_ << std::endl;
        // std::cout << "num 76 : " << cnt_76 << "  " << "num_75 : " << cnt_75 << std::endl;
        ifs.close();
        Lidarline_num++;
    }    
    
    LidarcsvFile.close();
    return EXIT_SUCCESS;
}


int main(int argc, char **argv) 
{
    
    ros::init(argc, argv, "TestPublishData");
    ros::NodeHandle nh("~");
    
    // launch parameter
    rosbag::Bag bag;
    bool to_bag; 
    // ToUndistortionPoints;
    std::string data_dir, output_bag_file;
    int publish_delay, PublishTimeStampType;
    
    nh.getParam("PublishTimeStampType", PublishTimeStampType);
    nh.getParam("data_dir", data_dir);
    nh.getParam("to_bag", to_bag);
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


    

    ////////// Read IMU data ///////////
    std::cout << " Load imu Data ... " << std::endl;
    std::string IMUcsvPath = data_dir + "imu_data.csv";
    ReadIMUdata(IMUcsvPath, &DB);
    
    ////////// VIO data /////////////
    std::cout << " Load VIO Data ... " << std::endl;
    std::string Rovins2PosePath = data_dir + "rovins2_all_frames.txt";
    
    Vector6d p;
    p << 0, 0, 0, 0, 0, 0;
    DB.VIOLidarPoses.push_back(p);
    DB.VIOtimestamps.push_back(0.0);

    ReadVIOdata(Rovins2PosePath, &DB);

    
    //////////// SLAM Pose data ( Keyframe ) ///////////////
    std::cout << " Load slam data ... " << std::endl;
    std::string SlamPosePath = data_dir + "slam_poses.txt";
    ReadSlamdata(SlamPosePath, &DB);

    //////////// Lidar timestamp.csv path ////////////////
    std::cout << " Load Lidar Data ... " << std::endl;
    std::string LidarcsvPath = data_dir + "lidar_timestamp.csv";
    std::string Lidar_binary_path = data_dir + "lidar/";
    ReadLidardata(LidarcsvPath, Lidar_binary_path, &DB, true);
    
    for(int i = 1; i < DB.VIOtimestamps.size(); i++){
        
        double MinVal = 1000;
        double Minidx = -1;
        for(int j = 0; j < DB.LidarLastseqtimestamps.size(); j ++){

            double difftime = std::fabs(DB.VIOtimestamps[i] - DB.LidarLastseqtimestamps[j]);
            if(difftime < MinVal){
                MinVal = difftime;
                Minidx = j;
            }
        }

        DB.VIOidx2Lidaridx[i] = Minidx;
        
    }

    std::cout << " Finish !! " << std::endl;
    std::cout << " Start Publish !!! " << std::endl;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////// Publish LidarScanTimeStamp ///////////////////////////////////////////////////

if(PublishTimeStampType == 0){
    int StartIdx = 0;
    int LastIdx = 0;
    // publish delay
    ros::Rate r(10.0);
    int Publish_cnt = 0;
    for(int i = 0; i < DB.LidarLastseqtimestamps.size(); i++){
        
        std::vector<Eigen::Vector3d> PublishPoints;
        PublishPoints.clear();

        while(DB.Lidartimestamps[LastIdx] <= DB.LidarLastseqtimestamps[i]){
            LastIdx++;
        }
                
        for(int j = StartIdx; j < LastIdx; j++){
            
            for(int k = 0; k < DB.LidarPoints[j].cols(); k++){

                Eigen::Vector3d point;
                point.x() = DB.LidarPoints[j](0, k);
                point.y() = DB.LidarPoints[j](1, k);
                point.z() = DB.LidarPoints[j](2, k);
                PublishPoints.push_back(point);
            } 
        }
        // std::cout << "PointCloud size : " << PublishPoints.size() << std::endl;
        std::cout << "Timestamp : " << ros::Time().fromSec(DB.LidarLastseqtimestamps[i]) << std::endl;
        

        
        // find lidartimestamp - cam timestamp
        int Minidx = -1;
        double Mindiff = 1000;
        for(size_t j = 0; j < DB.VIOtimestamps.size(); j++){
            double diff = std::fabs(DB.VIOtimestamps[j] - DB.LidarLastseqtimestamps[i]);
            if(diff < Mindiff){
                Mindiff = diff;
                Minidx = j;
            }    
        }
        Eigen::Quaterniond q = ToQuaternion(DB.VIOLidarPoses[Minidx]);
        Eigen::Vector3d p;
        p << DB.VIOLidarPoses[Minidx][3], DB.VIOLidarPoses[Minidx][4], DB.VIOLidarPoses[Minidx][5];            

        if(Publish_cnt % 10 == 0){

        
        
        // publish pointcloud
        sensor_msgs::PointCloud2 output;
        // pcl::toROSMsg(PublishPoints, output);
        output = ConverToROSmsg(PublishPoints);
        output.header.stamp = ros::Time().fromSec(DB.LidarLastseqtimestamps[i]);
        output.header.frame_id = "/camera_init";
        pubLaserCloud.publish(output);


        // publish odometry
        // std::cout << "Publish!" << std::endl;
        nav_msgs::Odometry VIOodometry;
        VIOodometry.header.frame_id = LidarFrame;
        VIOodometry.child_frame_id = "/laser_odom";
        VIOodometry.header.stamp = ros::Time().fromSec(DB.LidarLastseqtimestamps[i]);
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

        }
        
        StartIdx = LastIdx;
        Publish_cnt++;
        if(!ros::ok()) break;
        r.sleep();
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////// Publish Camtimestamp all frames ////////////////////////////////////////////////////////
if(PublishTimeStampType == 1){
    int StartIdx = 0;
    int LastIdx = 0;
    int LastLastIdx = 0;
    // publish delay
    ros::Rate r(10.0);
    for(int i = 1; i < DB.VIOtimestamps.size(); i+=2){
        
        std::vector<Eigen::Vector3d> PublishPoints;
        PublishPoints.clear();

        Eigen::Quaterniond q = ToQuaternion(DB.VIOLidarPoses[i]);
        Eigen::Vector3d p;
        p << DB.VIOLidarPoses[i][3], DB.VIOLidarPoses[i][4], DB.VIOLidarPoses[i][5];

        int LidarIdx = DB.VIOidx2Lidaridx[i]; 

        while(DB.Lidartimestamps[LastIdx] <= DB.VIOtimestamps[i]){
            LastIdx++;
        }
        // StartIdx = LastIdx - 76;
        // if(StartIdx < 0) StartIdx = 0;
        // double StartTime = DB.VIOtimestamps[i] -0.1; // 100ms
        // for(int j = 0; j < DB.Lidartimestamps.size(); j++){
        //             if(DB.LidarLastseqtimestamps[LidarIdx - 1] == DB.Lidartimestamps[k]) StartIdx = k + 1;
        //             if(DB.LidarLastseqtimestamps[LidarIdx] == DB.Lidartimestamps[k]) LastIdx = k + 1;
        //         }        
        // while(DB.Lidartimestamps[LastIdx] <= DB.LidarLastseqtimestamps[LidarIdx]){
        //     LastIdx++;
        // }
        for(int j = StartIdx; j < LastIdx; j++){
            
            for(int k = 0; k < DB.LidarPoints[j].cols(); k++){

                Eigen::Vector3d point;
                point.x() = DB.LidarPoints[j](0, k);
                point.y() = DB.LidarPoints[j](1, k);
                point.z() = DB.LidarPoints[j](2, k);
                PublishPoints.push_back(point);
            } 
        }
        // std::cout << "PointCloud size : " << PublishPoints.size() << std::endl;
        // std::cout << "Timestamp : " << ros::Time().fromSec(DB.VIOtimestamps[i]) << std::endl;

        // publish pointcloud
        sensor_msgs::PointCloud2 output;
        // pcl::toROSMsg(PublishPoints, output);
        output = ConverToROSmsg(PublishPoints);
        output.header.stamp = ros::Time().fromSec(DB.VIOtimestamps[i]);
        output.header.frame_id = "/camera_init";
        pubLaserCloud.publish(output);        
        
        // publish odometry
        // std::cout << "Publish!" << std::endl;
        nav_msgs::Odometry VIOodometry;
        VIOodometry.header.frame_id = LidarFrame;
        VIOodometry.child_frame_id = "/laser_odom";
        VIOodometry.header.stamp = ros::Time().fromSec(DB.VIOtimestamps[i]);
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

        StartIdx = LastIdx;
        if(!ros::ok()) break;
        r.sleep();                              
    }
}        
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////// Publish Cam Keyframe timestamp  ////////////////////////////////////////////////////////
if(PublishTimeStampType == 2){
    
    int StartIdx = 0;
    int LastIdx = 0;
    // publish delay
    ros::Rate r(20.0);
    
    for(int i = 1; i < DB.VIOtimestamps.size(); i++){
        
        std::vector<Eigen::Vector3d> PublishPoints;
        PublishPoints.clear();



        int SlamIdx = -1;
        for(int j = 0; j < DB.Slamcamidxs.size(); j++){
            
            if(i == DB.Slamcamidxs[j]){
                std::cout << " Keyframe !!  Idx is : " << i << std::endl;
                Eigen::Quaterniond q = ToQuaternion(DB.SlamKFPoses[j]);
                Eigen::Vector3d p;
                p << DB.SlamKFPoses[j][3], DB.SlamKFPoses[j][4], DB.SlamKFPoses[j][5];                
            
                int LidarIdx = DB.VIOidx2Lidaridx[i];
                for(int k = 0; k < DB.Lidartimestamps.size(); k++){
                    if(DB.LidarLastseqtimestamps[LidarIdx - 1] == DB.Lidartimestamps[k]) StartIdx = k + 1;
                    if(DB.LidarLastseqtimestamps[LidarIdx] == DB.Lidartimestamps[k]) LastIdx = k + 1;
                }
                // std::cout << LastIdx - StartIdx << std::endl;
                for(int t = StartIdx; t < LastIdx; t++){
                    
                    for(int k = 0; k < DB.LidarPoints[t].cols(); k++){

                        Eigen::Vector3d point;
                        point.x() = DB.LidarPoints[t](0, k);
                        point.y() = DB.LidarPoints[t](1, k);
                        point.z() = DB.LidarPoints[t](2, k);
                        PublishPoints.push_back(point);
                    } 
                }

                // publish pointcloud
                sensor_msgs::PointCloud2 output;
                // pcl::toROSMsg(PublishPoints, output);
                output = ConverToROSmsg(PublishPoints);
                output.header.stamp = ros::Time().fromSec(DB.SlamKFtimestamps[j]);
                output.header.frame_id = "/camera_init";
                pubLaserCloud.publish(output);        
                
                // publish odometry
                // std::cout << "Publish!" << std::endl;
                nav_msgs::Odometry VIOodometry;
                VIOodometry.header.frame_id = LidarFrame;
                VIOodometry.child_frame_id = "/laser_odom";
                VIOodometry.header.stamp = ros::Time().fromSec(DB.SlamKFtimestamps[j]);
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
            
            }

        }
        
        if(!ros::ok()) break;
        r.sleep();                              
    }

}
            
            
            
            
        
        

        


        
        
        
        
        
        
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
    /*
    std::ifstream LidarcsvFile(LidarcsvPath, std::ifstream::in);

    if(!LidarcsvFile.is_open()){
        std::cout << " Lidar csv file failed to open " << std::endl;
        return EXIT_FAILURE;
    }
    
    // Read Lidar timestamp.csv
    Eigen::Matrix4d LidarRotation;
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
        
        
        std::cout << " File number : " << fidx << "     " << std::endl;
        
        // Read Binary data file
        int num_seqs = 0;
        ifs.read((char*)&num_seqs, sizeof(int));
         
        // Integral IMU rotation to one lidar scan
        Eigen::Matrix4d IMURotation_integral = Eigen::Matrix4d::Identity();
        while(LidarScantimestamp > IMUtimestamps[IMUcount]){
            Eigen::Matrix4d IMURotation = gyroToRotation(IMUGyros[IMUcount]);
            IMURotation_integral = IMURotation * IMURotation_integral;
            IMUcount++;
        }

        Eigen::Matrix4d RT_ = IMUToRig * IMURotation_integral * IMUToRig.inverse();
        Eigen::Matrix4d RT = LidarToRig.inverse() * RT_ * LidarToRig;
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
        // Eigen::Quaterniond q = ToQuaternion(VIOLidarPoses[Minidx]);

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
    */
    return 0;
}


    

    

    
    






       
            
    

       
            

            


    







             

    



