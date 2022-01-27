#pragma once

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include "sort_lidarpoints/feature_info.h"

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>

#include <pcl/point_types.h>

typedef pcl::PointXYZI PointType;


const std::vector<float> imu2rig_pose = {-0.011773881878296,-2.212344247385963,2.229193892963689,-0.016975989407230,0.016444757006134,0.128779023189435};
const std::vector<float> lidar2rig_pose = {1.5620435019860173, -0.005377623186353324, 0.003014408980859652, -8.458334129298635E-4, -0.19542397891778734, -0.0012719333618026098};
typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
Eigen::Vector3d Origin{0.0, 0.0, 0.0};
std::string LidarFrame = "/camera_init";
struct Line;
struct Plane;
sensor_msgs::PointCloud2 ConverToROSmsg(const std::vector<Eigen::Vector3d> &PointCloud);
visualization_msgs::MarkerArray ConverToROSmsg(const std::vector<Line> &line, const ros::Time &timestamp, const std::string &frameid);
visualization_msgs::MarkerArray ConverToROSmsg(const ros::Time &timestamp, const std::string &frameid);
std::vector<Eigen::Vector3d> ConvertFromROSmsg(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
std::vector<Eigen::Vector3d> ConvertFromROSmsg(const sort_lidarpoints::feature_infoConstPtr &laserCloudMsg);
Eigen::Matrix3d NormalToRotation(Eigen::Vector3d n);
Eigen::Matrix4d To44RT(Vector6d rot);

struct OdometryData
{
    std::map< int, Vector6d > Odometry_WPose;
    std::map< int, Vector6d > MappingHighFrequency_WPose;
    std::map< int, Vector6d > Mapping_WPose;

    // Vector6d RelPose(int a, int b){;}
};

struct Line
{
    Eigen::Vector3d p1;
    Eigen::Vector3d p2;
    int id;
    Eigen::Vector3d directionVec() {return p2 - p1;}
};

struct Plane
{
    Eigen::Vector3d normal;
    Eigen::Vector3d centroid;
    double scale = 2.0;
    int id;
    
};

Vector6d To6DOF(Eigen::Quaterniond q, Eigen::Vector3d t)
{
    Eigen::AngleAxisd rod(q);
    Eigen::Vector3d r(rod.axis());
    double angle = rod.angle();
    r *= angle;

    Vector6d Pose;
    Pose << r.x(), r.y(), r.z(), t.x(), t.y(), t.z();

    return Pose;
}

Vector6d To6DOF(Eigen::Matrix4d RT)
{
    Eigen::Matrix3d R = RT.block<3, 3>(0, 0);
    Eigen::AngleAxisd rod(R);
    Eigen::Vector3d r(rod.axis());
    double angle = rod.angle();
    r *= angle;
    
    Vector6d Pose;
    Pose << r.x(), r.y(), r.z(), RT(0, 3), RT(1, 3), RT(2, 3);
    return Pose;
}

Vector6d RelativePose(std::map< int, Vector6d > Odometry_WPose, int a, int b)
{
    // if(a <= b){
        Eigen::Matrix4d RT_a = To44RT(Odometry_WPose[a]);
        Eigen::Matrix4d RT_b = To44RT(Odometry_WPose[b]);
        Eigen::Matrix4d RelPose = RT_a.inverse() * RT_b;
        Vector6d Relative_Pose = To6DOF(RelPose);
        
        return Relative_Pose;
    // }
}

Eigen::Matrix4f To44RT(Vector6f rot)
{

    cv::Mat R( 1, 3, CV_32FC1);
    R.at<float>(0, 0) = rot[0];
    R.at<float>(0, 1) = rot[1];
    R.at<float>(0, 2) = rot[2];

    cv::Rodrigues(R, R);

    Eigen::Matrix4f RT;
    RT << R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2), rot[3],
                R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2), rot[4],
                R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2), rot[5],
                0,                 0,                   0,                  1;

    return RT;
}

Eigen::Matrix4d To44RT(Vector6d rot)
{

    cv::Mat R( 1, 3, CV_64FC1);
    R.at<double>(0, 0) = rot[0];
    R.at<double>(0, 1) = rot[1];
    R.at<double>(0, 2) = rot[2];

    cv::Rodrigues(R, R);

    Eigen::Matrix4d RT;
    RT << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), rot[3],
                R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), rot[4],
                R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), rot[5],
                0,                 0,                   0,                  1;

    return RT;
}

Eigen::Matrix4f To44RT(std::vector<float> rot)
{

    cv::Mat R( 1, 3, CV_32FC1);
    R.at<float>(0, 0) = rot[0];
    R.at<float>(0, 1) = rot[1];
    R.at<float>(0, 2) = rot[2];

    cv::Rodrigues(R, R);

    Eigen::Matrix4f RT;
    RT << R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2), rot[3],
                R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2), rot[4],
                R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2), rot[5],
                0,                 0,                   0,                  1;

    return RT;
}

float ToAngle(Eigen::Matrix4f LidarRotation)
{
    float data[] = {    LidarRotation(0, 0), LidarRotation(0, 1), LidarRotation(0, 2),
                        LidarRotation(1, 0), LidarRotation(1, 1), LidarRotation(1, 2),
                        LidarRotation(2, 0), LidarRotation(2, 1), LidarRotation(2, 2)};

    
    cv::Mat rot(3, 3, CV_32FC1, data);
    cv::Rodrigues(rot, rot);
    float angle = sqrt( rot.at<float>(0, 0) * rot.at<float>(0, 0) + 
                        rot.at<float>(1, 0) * rot.at<float>(1, 0) +
                        rot.at<float>(2, 0) * rot.at<float>(2, 0) );

    return angle;
}

Eigen::Vector3f ToAxis(Eigen::Matrix4f LidarRotation)
{
    float angle = ToAngle(LidarRotation);
    float data[] = {    LidarRotation(0, 0), LidarRotation(0, 1), LidarRotation(0, 2),
                        LidarRotation(1, 0), LidarRotation(1, 1), LidarRotation(1, 2),
                        LidarRotation(2, 0), LidarRotation(2, 1), LidarRotation(2, 2)};

    
    cv::Mat rot(3, 3, CV_32FC1, data);
    cv::Rodrigues(rot, rot);

    Eigen::Vector3f Axis;
    Axis << rot.at<float>(0, 0), rot.at<float>(1, 0), rot.at<float>(2, 0);
    Axis = Axis / angle;

    return Axis;

}

void FeatureToMsg(  sort_lidarpoints::feature_info &msg, 
                    const std::vector<Line> line,
                    const std::vector<Plane> plane)
{
    // Line information
    for(size_t i = 0; i < line.size(); i++){
        
        msg.p1_x[i] = line[i].p1.x();
        msg.p1_y[i] = line[i].p1.y();
        msg.p1_z[i] = line[i].p1.z();

        msg.p2_x[i] = line[i].p2.x();
        msg.p2_y[i] = line[i].p2.y();
        msg.p2_z[i] = line[i].p2.z();
    }

    // Plane information
    for(size_t i = 0; i < plane.size(); i++){
        
        msg.normal_x[i] = plane[i].normal.x();
        msg.normal_y[i] = plane[i].normal.y();
        msg.normal_z[i] = plane[i].normal.z();

        msg.centroid_x[i] = plane[i].centroid.x();
        msg.centroid_y[i] = plane[i].centroid.y();
        msg.centroid_z[i] = plane[i].centroid.z();

        msg.scale[i] = plane[i].scale;
    }
}

void MsgToFeature(  const sort_lidarpoints::feature_infoConstPtr &FeatureMsg,
                    std::vector<Line> &line, std::vector<Plane> &plane)
{
    // Line Information
    for(size_t i = 0; i < FeatureMsg->p1_x.size(); i++){
        
        line[i].p1.x() = FeatureMsg->p1_x[i];
        line[i].p1.y() = FeatureMsg->p1_y[i];
        line[i].p1.z() = FeatureMsg->p1_z[i];

        line[i].p2.x() = FeatureMsg->p2_x[i];
        line[i].p2.y() = FeatureMsg->p2_y[i];
        line[i].p2.z() = FeatureMsg->p2_z[i];
    }

    // Plane Information
    for(size_t i = 0; i < FeatureMsg->normal_x.size(); i++){

        plane[i].normal.x() = FeatureMsg->normal_x[i];
        plane[i].normal.y() = FeatureMsg->normal_y[i];
        plane[i].normal.z() = FeatureMsg->normal_z[i];

        plane[i].centroid.x() = FeatureMsg->centroid_x[i];
        plane[i].centroid.y() = FeatureMsg->centroid_y[i];
        plane[i].centroid.z() = FeatureMsg->centroid_z[i];

        plane[i].scale = FeatureMsg->scale[i];
    }
}

void PublishFeature(const ros::Publisher &publisher, sort_lidarpoints::feature_info &pubmsg, const std::vector<Line> line, const std::vector<Plane> plane, const ros::Time &timestamp, const std::string &frameid)
{
    // sort_lidarpoints::feature_info pubmsg;
    
    // memory
    // Line
    pubmsg.p1_x.assign(line.size(), 0);
    pubmsg.p1_y.assign(line.size(), 0);
    pubmsg.p1_z.assign(line.size(), 0);
    pubmsg.p2_x.assign(line.size(), 0);
    pubmsg.p2_y.assign(line.size(), 0);
    pubmsg.p2_z.assign(line.size(), 0);
    
    // Plane
    pubmsg.normal_x.assign(plane.size(), 0);
    pubmsg.normal_y.assign(plane.size(), 0);
    pubmsg.normal_z.assign(plane.size(), 0);
    pubmsg.centroid_x.assign(plane.size(), 0);
    pubmsg.centroid_y.assign(plane.size(), 0);
    pubmsg.centroid_z.assign(plane.size(), 0);
    pubmsg.scale.assign(plane.size(), 0);
    
    FeatureToMsg(pubmsg, line, plane);
    
    // pubmsg.header.stamp = timestamp;
    // pubmsg.header.frame_id = frameid;

    publisher.publish(pubmsg);    
}

void IMUdataToMsg(sort_lidarpoints::feature_info &msg, const Vector6f IMUdata)
{
    msg.gyro_rx = IMUdata[0];
    msg.gyro_ry = IMUdata[1];
    msg.gyro_rz = IMUdata[2];
    msg.gyro_x = IMUdata[3];
    msg.gyro_y = IMUdata[4];
    msg.gyro_z = IMUdata[5];
}

void PublishIMUandcloud(const ros::Publisher & publisher, const std::vector<Eigen::Vector3d> &Points, const Vector6f IMUdata, const ros::Time &timestamp, const std::string &frameid )
{
    sort_lidarpoints::feature_info pubmsg;
    sensor_msgs::PointCloud2 output;
    
    IMUdataToMsg(pubmsg, IMUdata);
    output = ConverToROSmsg(Points);
    pubmsg.cloud_undistortion = output;
    pubmsg.header.stamp = timestamp;
    pubmsg.header.frame_id = frameid;

    publisher.publish(pubmsg);
}


void PublishPointCloud(const ros::Publisher &publisher, const std::vector<Eigen::Vector3d> &pointclouds, const ros::Time &timestamp, const std::string &frameid)
{
    sensor_msgs::PointCloud2 pubmsg;
    pubmsg = ConverToROSmsg(pointclouds);
    pubmsg.header.stamp = timestamp;
    pubmsg.header.frame_id = frameid;
    publisher.publish(pubmsg);    
}

void PublishLine(const ros::Publisher &publisher, const std::vector<Line> &line, const ros::Time &timestamp, const std::string &frameid)
{
    visualization_msgs::MarkerArray pubmsg;
    pubmsg = ConverToROSmsg(line, timestamp, frameid);

    visualization_msgs::MarkerArray DeleteMarkermsg;
    DeleteMarkermsg = ConverToROSmsg(timestamp, frameid);
    
    publisher.publish(DeleteMarkermsg);
    publisher.publish(pubmsg);    
}

void VisualizeLine(const rviz_visual_tools::RvizVisualToolsPtr VisualLine, const std::vector<Line> line)
{
    VisualLine->deleteAllMarkers();
    for(size_t i = 0; i < line.size(); i++)
        VisualLine->publishLine(line[i].p1, line[i].p2, rviz_visual_tools::BLUE, rviz_visual_tools::XLARGE);
    VisualLine->trigger();
}

void VisualizePlane(const rviz_visual_tools::RvizVisualToolsPtr VisualPlane, const std::vector<Plane> plane)
{
   VisualPlane->deleteAllMarkers();
   // Publish Plane
   for(size_t i = 0; i < plane.size(); i++){

        Eigen::Isometry3d VisualizePlane;
        VisualizePlane.translation() = plane[i].centroid;
        VisualizePlane.linear() = NormalToRotation(plane[i].normal);
        std::string id = std::to_string(plane[i].id);
        VisualPlane->publishXYPlane(VisualizePlane, rviz_visual_tools::TRANSLUCENT_DARK, plane[i].scale);
        VisualPlane->publishText(VisualizePlane, id, rviz_visual_tools::WHITE, rviz_visual_tools::XXXXLARGE, false);
        VisualPlane->trigger();
   }    
}
        



void VisualizePlaneNormal(const rviz_visual_tools::RvizVisualToolsPtr VisualPlaneNormal, const std::vector<Plane> plane)
{
    VisualPlaneNormal->deleteAllMarkers();
    // publish plane normal
    for(size_t i = 0; i < plane.size(); i++){    
        
        Eigen::Isometry3d VisualizePlaneNormal;
        VisualizePlaneNormal.translation() = plane[i].centroid;
        VisualizePlaneNormal.linear() = NormalToRotation(plane[i].normal);
        VisualPlaneNormal->publishZArrow(VisualizePlaneNormal, rviz_visual_tools::RED, rviz_visual_tools::LARGE, 1.0);
        VisualPlaneNormal->trigger();
    }
}

sensor_msgs::PointCloud2 ConverToROSmsg(const std::vector<Eigen::Vector3d> &PointCloud)
{
    struct point { float x, y, z; };
    const size_t PointCloudNum = PointCloud.size();

    std::vector<uint8_t> data_buffer(PointCloudNum * sizeof(point));
    size_t idx = 0;

    point *dataptr = (point*) data_buffer.data();

    for(auto i : PointCloud){
        dataptr[idx++] = {(float)i(0), (float)i(1), (float)i(2)};
    }

    static const char* const names[3] = { "x", "y", "z" };
    static const std::size_t offsets[3] = { offsetof(point, x), offsetof(point, y), offsetof(point, z) };
    std::vector<sensor_msgs::PointField> fields(3);
    for (int i=0; i < 3; i++) {
        fields[i].name = names[i];
        fields[i].offset = offsets[i];
        fields[i].datatype = sensor_msgs::PointField::FLOAT32;
        fields[i].count = 1;
    }


    sensor_msgs::PointCloud2 msg;
    msg.height = 1;
    msg.width = PointCloudNum;
    msg.fields = fields;
    msg.is_bigendian = true;
    msg.point_step = sizeof(point);
    msg.row_step = sizeof(point) * msg.width;
    msg.data = std::move(data_buffer);
    msg.is_dense = true;

    return msg;
}

std::vector<Eigen::Vector3d> ConvertFromROSmsg(sensor_msgs::PointCloud2 &PointCloud)
{
    struct point { float x, y, z; };
    const size_t PointCloudNum = PointCloud.width;

    std::vector<uint8_t> data_buffer(PointCloudNum * sizeof(point));
    data_buffer = std::move(PointCloud.data);
    point *dataptr = (point*) data_buffer.data();

    // size_t idx = 0;
    // for(auto i : PointCloud){
    //     dataptr[idx++] = {(float)i(0), (float)i(1), (float)i(2)};
    // }

    // static const char* const names[3] = { "x", "y", "z" };
    // static const std::size_t offsets[3] = { offsetof(point, x), offsetof(point, y), offsetof(point, z) };
    // std::vector<sensor_msgs::PointField> fields(3);
    // for (int i=0; i < 3; i++) {
    //     fields[i].name = names[i];
    //     fields[i].offset = offsets[i];
    //     fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    //     fields[i].count = 1;
    // }

    std::vector<Eigen::Vector3d> points(PointCloudNum);
    for(size_t i = 0; i < PointCloudNum; i++){
        points[i].x() = (double)dataptr[i].x;
        points[i].y() = (double)dataptr[i].y;
        points[i].z() = (double)dataptr[i].z;

    }
    
    // sensor_msgs::PointCloud2 msg;
    // msg.height = 1;
    // msg.width = PointCloudNum;
    // msg.fields = fields;
    // msg.is_bigendian = true;
    // msg.point_step = sizeof(point);
    // msg.row_step = sizeof(point) * msg.width;
    // msg.data = std::move(data_buffer);
    // msg.is_dense = true;

    return points;
}

visualization_msgs::MarkerArray ConverToROSmsg(const std::vector<Line> &line, const ros::Time &timestamp, const std::string &frameid)
{
    visualization_msgs::MarkerArray line_arr;



    for(size_t i = 0; i < line.size(); i++){
        visualization_msgs::Marker msg;
        msg.header.stamp = timestamp;
        msg.header.frame_id = frameid;
        msg.id = (int)i;
        msg.action = visualization_msgs::Marker::ADD;
        msg.pose.orientation.w = 1.0;
        msg.scale.x = 0.03;
        msg.scale.y = 0.08;
        msg.scale.z = 0.05;
        msg.color.r = 1.0;
        msg.color.a = 1.0;          
        msg.type = visualization_msgs::Marker::ARROW;
        
        geometry_msgs::Point p;
        p.x = line[i].p1.x();
        p.y = line[i].p1.y();
        p.z = line[i].p1.z();
        msg.points.push_back(p);

        p.x = line[i].p2.x();
        p.y = line[i].p2.y();
        p.z = line[i].p2.z();
        msg.points.push_back(p);

        line_arr.markers.push_back(msg);
    }


  return line_arr;
}

visualization_msgs::MarkerArray ConverToROSmsg(const ros::Time &timestamp, const std::string &frameid)
{
    visualization_msgs::MarkerArray msg_arr;
    visualization_msgs::Marker msg;
    
    msg.header.stamp = timestamp;
    msg.header.frame_id = frameid;
    msg.action = visualization_msgs::Marker::DELETEALL;
    msg_arr.markers.push_back(msg);
    
    return msg_arr;
}

void InputSlamPose(Eigen::Matrix4d RT, double *q, double *t)
{
    Eigen::Matrix3d r = RT.block<3, 3>(0, 0);
    Eigen::Quaterniond qq(r);

    q[0] = qq.x();
    q[1] = qq.y();
    q[2] = qq.z();
    q[3] = qq.w();

    t[0] = RT(0, 3);
    t[1] = RT(1, 3);
    t[2] = RT(2, 3);

}
// std::vector<Eigen::Vector3d> ConvertFromROSmsg(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
// {
//     std::vector<Eigen::Vector3d> laserPoints;
//     uint8_t* byte = const_cast<uint8_t*>(&laserCloudMsg->data[0]);
//     float* floatByte = reinterpret_cast<float*>(byte);
//     int pointcloudNum = laserCloudMsg->data.size() / (sizeof(float) * 3);
//     laserPoints.resize(pointcloudNum);

//     for(int i = 0; i < pointcloudNum; i++){
        
//         laserPoints[i].x() = (double)floatByte[3 * i];
//         laserPoints[i].y() = (double)floatByte[3 * i + 1];
//         laserPoints[i].z() = (double)floatByte[3 * i + 2];
//     }

//     return laserPoints;
// }

// std::vector<Eigen::Vector3d> ConvertFromROSmsg(const sort_lidarpoints::feature_infoConstPtr &laserCloudMsg)
// {
//     std::vector<Eigen::Vector3d> laserPoints;
//     sensor_msgs::PointCloud2 ExtractPoints;
//     ExtractPoints = laserCloudMsg->cloud_undistortion;
//     laserPoints = ConvertFromROSmsg(ExtractPoints);
//     // uint8_t* byte = const_cast<uint8_t*>(&laserCloudMsg->data[0]);
//     // float* floatByte = reinterpret_cast<float*>(byte);
//     // int pointcloudNum = laserCloudMsg->data.size() / (sizeof(float) * 3);
//     // laserPoints.resize(pointcloudNum);

//     // for(int i = 0; i < pointcloudNum; i++){
        
//     //     laserPoints[i].x() = (double)floatByte[3 * i];
//     //     laserPoints[i].y() = (double)floatByte[3 * i + 1];
//     //     laserPoints[i].z() = (double)floatByte[3 * i + 2];
//     // }

//     return laserPoints;
// }

float VerticalAngle(Eigen::Vector3d p){
  return atan(p.z() / sqrt(p.x() * p.x() + p.y() * p.y())) * 180 / M_PI;
}

double PointDistance(Eigen::Vector3d p){
  return sqrt(p.x()*p.x() + p.y()*p.y() + p.z()*p.z());
}

double PointDistance(Eigen::Vector3d p1, Eigen::Vector3d p2){
  return sqrt((p1.x()-p2.x())*(p1.x()-p2.x()) + (p1.y()-p2.y())*(p1.y()-p2.y()) + (p1.z()-p2.z())*(p1.z()-p2.z()));
}

double Point2LineDistance(Line l, Eigen::Vector3d p){
    return ((p - l.p1).cross(p - l.p2)).norm() / (l.p1 - l.p2).norm();
}

double LineToLineDistance(Line l1, Line l2){
    
    if(0)
        ;// parallel
    else{
        
        Eigen::Vector3d b1 = l1.directionVec();
        Eigen::Vector3d b2 = l2.directionVec();
        Eigen::Vector3d normal = b1.cross(b2);
        return std::abs((l1.p1 - l2.p1).dot(normal)) / normal.norm();
    }
}

double CosRaw2(double a, double b, float ang){
    return sqrt(a * a + b * b - 2 * a * b * cos(ang * M_PI / 180));
}

double NextChannelPointDis(Eigen::Vector3d p, float VerticalAngleRatio){
    
    double dist = PointDistance(p);
    float VerticalAngle_ = VerticalAngle(p);
    double PointToLine = dist * cos(std::abs(VerticalAngle_) * M_PI / 180);
    float theta = std::abs(VerticalAngleRatio + VerticalAngle_);
    double NextPointDis = PointToLine / cos(theta * M_PI / 180);
    
    return CosRaw2(dist, NextPointDis, VerticalAngleRatio);
}

Eigen::Matrix3d NormalToRotation(Eigen::Vector3d n){
    Eigen::Matrix3d r;
    Eigen::Vector3d UnitVec = Eigen::Vector3d::UnitZ();
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(UnitVec, n);
    return q.toRotationMatrix();
}

double Point2PlaneDistance(Plane p, Eigen::Vector3d x){
    Eigen::Vector3d b = x - p.centroid;
    return std::abs(p.normal.dot(b)) / p.normal.norm();
}

double DiffOriginToPlaneDistance(Plane p1, Plane p2){
    return std::abs(Point2PlaneDistance(p1, Origin) - Point2PlaneDistance(p2, Origin));
}

float AngleBetweenPlane(Plane p1, Plane p2){
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(p1.normal, p2.normal);
    return 2 * std::atan2(q.vec().norm(), std::fabs(q.w())) * ( 180 / M_PI );
}