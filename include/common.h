#pragma once

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>

#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <pcl/point_types.h>

typedef pcl::PointXYZI PointType;
typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
Eigen::Vector3d Origin{0.0, 0.0, 0.0};

const std::vector<double> imu2rig_pose{-0.011773881878296, -2.212344247385963, 2.229193892963689, -0.016975989407230, 0.016444757006134, 0.128779023189435};
const std::vector<double> lidar2rig_pose{1.5620435019860173, -0.005377623186353324, 0.003014408980859652, -8.458334129298635E-4, -0.19542397891778734, -0.0012719333618026098};

std::string LidarFrame = "/camera_init";

sensor_msgs::PointCloud2 ConverToROSmsg(const std::vector<Eigen::Vector3d> &PointCloud);
visualization_msgs::MarkerArray ConverToROSmsg(const ros::Time &timestamp, const std::string &frameid);
std::vector<Eigen::Vector3d> ConvertFromROSmsg(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
Eigen::Matrix4d To44RT(Vector6d rot);

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

struct DataBase
{
    std::vector<double> IMUtimestamps;
    std::vector<Eigen::Vector3f> IMUGyros;

    std::vector<double> VIOtimestamps;
    std::vector<Vector6d> VIOLidarPoses;

    std::vector<double> SlamKFtimestamps;
    std::vector<Vector6d> SlamKFPoses;
    std::vector<int> Slamcamidxs;

    std::vector<double> LidarLastseqtimestamps;
    std::vector<double> Lidartimestamps;
    std::vector<Eigen::Matrix3Xd> LidarPoints;

};

struct OdometryData
{
    std::map< int, Vector6d > Odometry_WPose;
    std::map< int, Vector6d > MappingHighFrequency_WPose;
    std::map< int, Vector6d > Mapping_WPose;

    // Vector6d RelPose(int a, int b){;}
};

Eigen::Vector3d ToVec3(Eigen::Matrix3d rot)
{
    Eigen::AngleAxisd rod(rot);
    Eigen::Vector3d axis(rod.axis());
    double angle = rod.angle();
    axis *= angle;

    Eigen::Vector3d vec3;
    vec3 << axis.x(), axis.y(), axis.z();

    return vec3;
}

Eigen::Vector3f ToVec3(Eigen::Matrix3f rot)
{
    Eigen::AngleAxisf rod(rot);
    Eigen::Vector3f axis(rod.axis());
    float angle = rod.angle();
    axis *= angle;

    Eigen::Vector3f vec3;
    vec3 << axis.x(), axis.y(), axis.z();

    return vec3;
}

Eigen::Matrix3d ToMat33(Eigen::Vector3d rod)
{
    Eigen::AngleAxisd r(rod.norm(), rod.normalized());
    Eigen::Matrix3d rot = r.toRotationMatrix();

    return rot;
}

Eigen::Matrix3f ToMat33(Eigen::Vector3f rod)
{
    Eigen::AngleAxisf r(rod.norm(), rod.normalized());
    Eigen::Matrix3f rot = r.toRotationMatrix();

    return rot;
}

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
    Eigen::Vector3d rod = ToVec3(R);
    
    Vector6d Pose;
    Pose << rod.x(), rod.y(), rod.z(), RT(0, 3), RT(1, 3), RT(2, 3);
    return Pose;
}

Eigen::Quaterniond ToQuaternion(const Vector6d Pose)
{
    Eigen::Matrix4d Pos = To44RT(Pose);
    Eigen::Matrix3d rot = Pos.block<3, 3>(0, 0); 

    Eigen::Quaterniond q(rot);

    return q; 
}



Eigen::Matrix4f To44RT(Vector6f pose)
{
    Eigen::Vector3f rod;
    rod << pose[0], pose[1], pose[2];
    Eigen::Matrix3f rot = ToMat33(rod);

    Eigen::Matrix4f RT;
    RT <<   rot(0, 0), rot(0, 1), rot(0, 2), pose[3],
            rot(1, 0), rot(1, 1), rot(1, 2), pose[4],
            rot(2, 0), rot(2, 1), rot(2, 2), pose[5],
            0,         0,         0,         1;

    return RT;
}

Eigen::Matrix4d To44RT(Vector6d pose)
{
    Eigen::Vector3d rod;
    rod << pose[0], pose[1], pose[2];
    Eigen::Matrix3d rot = ToMat33(rod);

    Eigen::Matrix4d RT;
    RT <<   rot(0, 0), rot(0, 1), rot(0, 2), pose[3],
            rot(1, 0), rot(1, 1), rot(1, 2), pose[4],
            rot(2, 0), rot(2, 1), rot(2, 2), pose[5],
            0,         0,         0,         1;

    return RT;
}

Eigen::Matrix4d To44RT(std::vector<double> pose)
{
    Eigen::Vector3d rod;
    rod << pose[0], pose[1], pose[2];

    Eigen::Matrix3d R = ToMat33(rod);

    Eigen::Matrix4d RT;
    RT <<   R(0, 0), R(0, 1), R(0, 2), pose[3],
            R(1, 0), R(1, 1), R(1, 2), pose[4],
            R(2, 0), R(2, 1), R(2, 2), pose[5],
            0,       0,       0,       1;

    return RT;
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

double ToAngle(Eigen::Matrix4d LidarRotation)
{
    Eigen::Matrix3d rot = LidarRotation.block<3, 3>(0, 0);
    Eigen::AngleAxisd rod(rot);
    double angle = rod.angle();

    return angle;
}

Eigen::Vector3d ToAxis(Eigen::Matrix4d LidarRotation)
{
    Eigen::Matrix3d rot = LidarRotation.block<3, 3>(0, 0);
    Eigen::AngleAxisd rod(rot);
    Eigen::Vector3d Axis = rod.axis();    

    return Axis;
}

void PublishPointCloud(const ros::Publisher &publisher, const std::vector<Eigen::Vector3d> &pointclouds, const ros::Time &timestamp, const std::string &frameid)
{
    sensor_msgs::PointCloud2 pubmsg;
    pubmsg = ConverToROSmsg(pointclouds);
    pubmsg.header.stamp = timestamp;
    pubmsg.header.frame_id = frameid;
    publisher.publish(pubmsg);    
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

    std::vector<Eigen::Vector3d> points(PointCloudNum);
    for(size_t i = 0; i < PointCloudNum; i++){
        points[i].x() = (double)dataptr[i].x;
        points[i].y() = (double)dataptr[i].y;
        points[i].z() = (double)dataptr[i].z;
    }

    return points;
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

float VerticalAngle(Eigen::Vector3d p){
  return atan(p.z() / sqrt(p.x() * p.x() + p.y() * p.y())) * 180 / M_PI;
}

double PointDistance(Eigen::Vector3d p){
  return sqrt(p.x()*p.x() + p.y()*p.y() + p.z()*p.z());
}

double PointDistance(Eigen::Vector3d p1, Eigen::Vector3d p2){
  return sqrt((p1.x()-p2.x())*(p1.x()-p2.x()) + (p1.y()-p2.y())*(p1.y()-p2.y()) + (p1.z()-p2.z())*(p1.z()-p2.z()));
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