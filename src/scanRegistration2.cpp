// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <boost/smart_ptr.hpp>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <set>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include "common.h"
#include "tic_toc.h"
#include "aloam/feature_info.h"

std_msgs::Header header;
const int systemDelay = 0; 
int systemInitCount = 0;
bool systemInited = false;
int N_SCANS = 0;
int cloudSize = 0;
std::vector<int> scanStartInd(N_SCANS, 0);
std::vector<int> scanEndInd(N_SCANS, 0);

float cloudCurvature[400000];
int cloudSortInd[400000];
int cloudNeighborPicked[400000];
int cloudLabel[400000];

std::vector<Eigen::Vector3d> cornerPointsSharp;
std::vector<Eigen::Vector3d> cornerPointsLessSharp;
std::vector<Eigen::Vector3d> surfPointsFlat;
std::vector<Eigen::Vector3d> surfPointsLessFlat;

std::vector<std::vector<Eigen::Vector3d>> ReferencePlanePoints;

std::vector<int> FeatureNumByScan(N_SCANS);
std::vector<std::vector<int>> PointIndexByChannel; // ( edge → 0 , plane → 1 , pass point → -1, occluded/parallel → -2 )


// downsize leaf
std::vector<int> LeafIds;
 

bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }
bool SameLeaf(int i, int j) { return LeafIds[i] < LeafIds[j]; }

// publish pointcloud
ros::Publisher pubLaserCloud;
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;
ros::Publisher pubReferencePlanePoints;

// subscribe imu data and publish Line and Plane
ros::Publisher pubFeature;
ros::Subscriber subFeature;

float VerticalAngelRatio = 0;

Eigen::Vector3d ZVec = Eigen::Vector3d::UnitZ();
Eigen::Matrix3d Iden = Eigen::Matrix3d::Identity();

sort_lidarpoints::feature_info FeatureInfo;

const size_t kMaxNumberOfPoints = 1e5;

////////// Parameter //////////

// Remove too closed points from lidar
double LidarToPointsThres = 1.0;
Eigen::Vector3d DownSizeLeafSize(0.2, 0.2, 0.2); // downsizefiltering to plane points

// double MaxPointsDis(const std::vector<Eigen::Vector3d> RefPoints)
// {
//     double MaxDis = 0;
//     for(auto i : RefPoints){
//         for(auto j : RefPoints){

//             double Dis = PointDistance(i, j);
//             if(MaxDis < Dis)
//                 MaxDis = Dis;
//         }
//     }

//     return MaxDis;
// }

// double OverlapRatio(const std::vector<Eigen::Vector3d> InputRefPoints,
//                     const std::vector<Eigen::Vector3d> RefPoints)
// {
    
//     int TotalSize = std::min(InputRefPoints.size(), RefPoints.size());
//     int cnt = 0;
//     for(auto i : RefPoints){
//         for(auto j : InputRefPoints){
//             if(i == j){
//                 cnt++;
//                 break;
//             }
//         }
//     }
//     double Ratio = (double)cnt / (double)TotalSize;
//     // std::cout << "Ratio : " << Ratio << std::endl;
//     return Ratio;
// }

// std::vector<Line> SelectLine(  const std::vector<std::vector<Eigen::Vector3d>> CornerPointByChannel, 
//                                 std::vector<std::vector<int>> CornerPointByChannelIdx)
// {
//     size_t StartChannelNum = 0;
//     int ChannelPass = 1;
//     std::vector<Line> line; 
//     while(StartChannelNum < 16){
//         for(size_t k = 0; k < CornerPointByChannel[StartChannelNum].size(); k++){
//             // int k = 0;

//             if(CornerPointByChannelIdx[StartChannelNum].size() == 0) 
//                 continue;

//             if(CornerPointByChannelIdx[StartChannelNum][k] == 1) 
//                 continue;
            
//             std::vector<Eigen::Vector3d> ReferencePoint;
//             Line line_;
//             size_t cnt = 0;
//             for(size_t i = StartChannelNum ; i < CornerPointByChannel.size(); i ++){
                

//                 if(i == StartChannelNum){
//                     ReferencePoint.push_back(CornerPointByChannel[i][k]);
//                     line_.p1 = ReferencePoint[0];
//                     CornerPointByChannelIdx[i][k] = 1;
//                     continue;
//                 }
                
//                 if(CornerPointByChannel[i].size() == 0){
//                     ChannelPass++;
//                     continue;
//                 }
                
//                 double MinDist = 10;
//                 int Minidx = 0;
//                 for(size_t j = 0; j < CornerPointByChannel[i].size(); j ++){
//                     double dist = PointDistance(ReferencePoint.back(), CornerPointByChannel[i][j]);

//                     if(MinDist > dist){
//                         MinDist = dist;
//                         Minidx = j;
//                     }
//                 }
//                 // double LidarToPoint = PointDistance(ReferencePoint);
//                 double NextChannelPointDist = NextChannelPointDis(ReferencePoint.back(), VerticalAngelRatio * (float)ChannelPass);
//                 // std::cout << "LidarToPoint : " << LidarToPoint << std::endl;
//                 // std::cout << "Min distance : " << MinDist << std::endl;
//                 // std::cout << "thres : " << thres << std::endl;
                
//                 if(MinDist > NextChannelPointDist * SortLinePointsThres ){ 
//                     ChannelPass = 1;
//                     break;
//                 } 
//                 // std::cout << "Line Min distance : " << MinDist << std::endl;

//                 ReferencePoint.push_back(CornerPointByChannel[i][Minidx]);
//                 CornerPointByChannelIdx[i][Minidx] = 1;
//                 line_.p2 = ReferencePoint.back();
//                 ChannelPass = 1;
//             }
//             if(ReferencePoint.size() > 2){
//                 for(size_t i = 0; i < ReferencePoint.size(); i++){
//                     double PointToLineDis = Point2LineDistance(line_, ReferencePoint[i]);
//                     // std::cout << PointToLineDis << std::endl;
//                     if(PointToLineDis < PointToLineThres){
//                         cnt++;
//                     }
//                 }
                
//                 if(ReferencePoint.size() == cnt)   
//                     line.push_back(line_);

//             }
//         }
//         // std::cout << StartChannelNum << std::endl;
//         StartChannelNum++;
//     }

//     std::cout << "Line Num : " << line.size() << std::endl;
//     return line;
// }

// std::vector<Line> ClusteringLine(std::vector<Line> line)
// {
//     std::vector<Line> MergedLine;

//     // Line Direction

//     // Distance Line to Line

//     return MergedLine;
// }

// std::vector<Eigen::Vector3d> PlanePointsforPlane(const Eigen::Vector3d p)
// {
//     std::vector<Eigen::Vector3d> PlanePoints;
    
//     for(size_t i = 0; i < surfPointsLessFlat.size(); i++){

//         double PointDis = PointDistance(surfPointsLessFlat[i], p);
//         if(PointDis < SearchPlanePointDis)
//             PlanePoints.push_back(surfPointsLessFlat[i]);
//     }
    
//     return PlanePoints;
// }

// Plane PlaneFromPoints(const std::vector<Eigen::Vector3d> p)
// {
//     Plane plane;

//     Eigen::Matrix3Xd points(3, p.size());
//     for(size_t i = 0; i < p.size(); i++)
//         points.col(i) = p[i];
//     Eigen::Vector3d c(points.row(0).mean(), points.row(1).mean(), points.row(2).mean());

//     points.row(0).array() -= c.x();
//     points.row(1).array() -= c.y();
//     points.row(2).array() -= c.z();
//     auto svd = points.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeThinV);

//     Eigen::Vector3d n = svd.matrixU().col(2);
//     plane.normal = n;
//     plane.centroid = c;

//     return plane;
// }

// std::vector<Plane> SelectPlane(const std::vector<Eigen::Vector3d> surfPointsFlat)
// {
//     std::vector<Plane> plane;
    
//     for(size_t i = 0; i < surfPointsFlat.size(); i++){
        
//         std::vector<Eigen::Vector3d> ReferencePlanePoint = PlanePointsforPlane(surfPointsFlat[i]);
//         if(ReferencePlanePoint.size() < 35) continue;
//         // std::cout << "Total ReferencePlanePoint Num : " << ReferencePlanePoint.size() << std::endl;
//         Plane plane_ = PlaneFromPoints(ReferencePlanePoint);
        
//         // remove ground point
//         // if(ZVec.cross(plane_.normal).norm() < 0.5) continue;
        
//         int cnt = 0;
//         for(size_t i = 0; i < ReferencePlanePoint.size(); i++){
//             double PointToPlaneDis = Point2PlaneDistance(plane_, ReferencePlanePoint[i]);
//             if(PointToPlaneDis < PointToPlaneThres){
//                 cnt++;
//             }

//         }
//         double SuccessPlanePointRatio = (double)cnt / (double)ReferencePlanePoint.size();
//         // std::cout << "Success plane Point Ratio : " << SuccessPlanePointRatio << std::endl;
//         if(SuccessPlanePointRatio > SuccessPlanePointRatioThres){
//             plane.push_back(plane_);
//             ReferencePlanePoints.push_back(ReferencePlanePoint);
//         }

//     }
//     return plane;
// }

// std::vector<Plane> ClusteringPlane(std::vector<Plane> *plane)
// {
//     std::vector<Plane> MergedPlane;
//     std::vector<std::vector<Eigen::Vector3d>> RefPlanePoints(ReferencePlanePoints);
//     ReferencePlanePoints.clear();
//     while(plane->size()){

    
//         int InputPlane = 0;
//         std::vector<Plane> SamePlane;
//         std::vector<int> SamePlaneInd;
//         SamePlane.push_back((*plane)[InputPlane]);
//         SamePlaneInd.push_back(InputPlane);
//         for(size_t i = InputPlane + 1; i < plane->size(); i++){
            
//             // Normal
//             // double CrossNorm = ((*plane)[InputPlane].normal).cross((*plane)[i].normal).norm(); 
//             // Angle between normal
//             float angle = AngleBetweenPlane((*plane)[InputPlane], (*plane)[i]);
//             // Distance between Plane and Point(centroid)
//             double PointToPlane = Point2PlaneDistance((*plane)[InputPlane], (*plane)[i].centroid);
//             // std::cout << PointToPlane << std::endl; 
            
//             // Distance between centroid
//             double PointToPoint = PointDistance((*plane)[InputPlane].centroid, (*plane)[i].centroid);
//             // std::cout << PointToPoint << std::endl;

//             // Overlap Ratio
//             double Ratio = OverlapRatio(RefPlanePoints[InputPlane], RefPlanePoints[i]);
//             // std::cout << "Ratio : " << Ratio << std::endl;
//             if((angle < 2.0 || angle > 178) && PointToPlane < PointToPlaneThres && PointToPoint < PointToPointThres){
//                 // && PointToPoint < PointToPointThres && Ratio > 0.01
//                 // std::cout << "cross norm value : " << CrossNorm << std::endl;
//                 // std::cout << "centroid and plane distance : " << PointToPlane << std::endl;
//                 // std::cout << "distance between Centroids: " << PointToPoint << std::endl;
//                 SamePlane.push_back((*plane)[i]);
//                 SamePlaneInd.push_back(i);
//             }
//         }

//         // Merge SamePlanes
//         Eigen::Vector3d c(0, 0, 0);
//         Eigen::Vector3d n(0, 0, 0);
//         Plane plane_;
//         std::vector<Eigen::Vector3d> RefPlanePoint;
//         for(size_t i = 0; i < SamePlane.size(); i++){
            
//             // Merge and if there is a same point, delete it
//             for(auto j : RefPlanePoints[SamePlaneInd[i]]){
                
//                 auto it = std::find(RefPlanePoint.begin(), RefPlanePoint.end(), j);
//                 if(it == RefPlanePoint.end())
//                     RefPlanePoint.push_back(j);
//             }
//             if(n.dot(SamePlane[i].normal) < 0){
//                 SamePlane[i].normal *= -1;
//             }
//             c += SamePlane[i].centroid;
//             n += SamePlane[i].normal;
//         }


//         double ScaleDis = MaxPointsDis(RefPlanePoint);
//         // std::cout << ScaleDis << std::endl;
//         ReferencePlanePoints.push_back(RefPlanePoint);


//         plane_.centroid = c / (double)SamePlane.size();
//         plane_.normal = n / (double)SamePlane.size();
//         // plane_.scale = 1.0 + (double)SamePlane.size() * 0.1;
//         plane_.scale = ScaleDis / 2.0;
//         // std::cout << SamePlane.size() << std::endl;
//         // std::cout << plane_.scale << std::endl;
        
//         MergedPlane.push_back(plane_);
        
//         // Erase Clustered Plane
//         int EraseInd = 0;
//         for(size_t i = 0; i < SamePlane.size(); i++){
//             plane->erase(plane->begin() + SamePlaneInd[i] - EraseInd);
//             RefPlanePoints.erase(RefPlanePoints.begin() + SamePlaneInd[i] - EraseInd);
//             EraseInd++;
//         }
//     }

//     return MergedPlane;
        
// }

// std::vector<Plane> ClusteringPlane2(std::vector<Plane> *plane)
// {
//     std::vector<Plane> MergedPlane;
//     std::vector<std::vector<Eigen::Vector3d>> RefPlanePoints(ReferencePlanePoints);
//     ReferencePlanePoints.clear();
    
//     Plane CurrMergePlane = plane->front();
//     std::vector<Eigen::Vector3d> RefPlanePoint(RefPlanePoints.front());
//     plane->erase(plane->begin());
//     RefPlanePoints.erase(RefPlanePoints.begin());
    
//     while(plane->size()){
        
//         double MinDis = 100.0;
//         int idx = -1;
//         Plane InputPlane = CurrMergePlane;
//         for(size_t i = 0; i < plane->size(); i++){
            
//             // Normal
//             // double CrossNorm = InputPlane.normal.cross((*plane)[i].normal).norm(); 
//             // Angle between normal
//             float angle = AngleBetweenPlane(CurrMergePlane, (*plane)[i]);
//             // std::cout << angle << std::endl;
            
//             // Distance between Plane and Point(centroid)
//             double PointToPlane = Point2PlaneDistance(CurrMergePlane, (*plane)[i].centroid);
//             // std::cout << PointToPlane << std::endl; 
            
//             // Distance between centroid
//             double PointToPoint = PointDistance(CurrMergePlane.centroid, (*plane)[i].centroid);
//             // std::cout << PointToPoint << std::endl;
            
//             // Overlap Ratio
//             double Ratio = OverlapRatio(RefPlanePoint, RefPlanePoints[i]);
//             // std::cout << "Ratio : " << Ratio << std::endl;            
            
//             if((angle < 2.0 || angle > 178) && PointToPlane < PointToPlaneThres && PointToPoint < PointToPointThres ){
//                 // && PointToPoint < PointToPointThres && Ratio > 0.01
//                 if(MinDis > PointToPoint){
//                     MinDis = PointToPoint;
//                     idx = i;
//                 }

//             }
//         }
                
//         // merged plane
//         if(idx == -1){
            
//             double ScaleDis = MaxPointsDis(RefPlanePoint); 
//             CurrMergePlane.scale = ScaleDis / 2.0;            
//             MergedPlane.push_back(CurrMergePlane);
//             CurrMergePlane = plane->front();
//             plane->erase(plane->begin());
            
//             ReferencePlanePoints.push_back(RefPlanePoint);
//             RefPlanePoint = RefPlanePoints.front();
//             RefPlanePoints.erase(RefPlanePoints.begin());
//         }
//         else{
        
//             CurrMergePlane.centroid = (InputPlane.centroid + (*plane)[idx].centroid) / 2;
//             if(InputPlane.normal.dot((*plane)[idx].normal) < 0) InputPlane.normal *= -1;
//             CurrMergePlane.normal = (InputPlane.normal + (*plane)[idx].normal) / 2;
            
//             // Visualize Plane Scale and delete same point
//             for(auto i : RefPlanePoints[idx]){
//                 auto it = std::find(RefPlanePoint.begin(), RefPlanePoint.end(), i);
//                 if(it == RefPlanePoint.end())
//                     RefPlanePoint.push_back(i);
//             }

            
//             plane->erase(plane->begin() + idx);
//             RefPlanePoints.erase(RefPlanePoints.begin() + idx);
//         }
        

    
//     }


//     return MergedPlane;
// }


// void MergedOverlappedPlane(std::vector<Plane> *plane)
// {
//     std::vector<Plane> copy_plane(*plane);
//     plane->clear();
//     std::vector<std::vector<Eigen::Vector3d>> RefPlanePoints(ReferencePlanePoints);
//     ReferencePlanePoints.clear();
    
//     Plane InputPlane;
//     std::vector<Eigen::Vector3d> RefPlanePoint;
    

//     while(copy_plane.size()){
        
//         std::vector<Plane> SamePlanes;
//         std::vector<int> SamePlanesInd;
        
//         InputPlane = copy_plane.front();
//         copy_plane.erase(copy_plane.begin());

//         RefPlanePoint = RefPlanePoints.front();
//         RefPlanePoints.erase(RefPlanePoints.begin());
        
//         SamePlanes.push_back(InputPlane);
//         bool IsSamePlane = false;
        
//         for(size_t i = 0; i < copy_plane.size(); i++){

//             // double CrossNorm = InputPlane.normal.cross(copy_plane[i].normal).norm(); 
//             // Angle between normal
//             float angle = AngleBetweenPlane(InputPlane, copy_plane[i]);
//             // Distance between Plane and Point(centroid)
//             double PointToPlane = Point2PlaneDistance(InputPlane, copy_plane[i].centroid);
            
//             // Distance between centroid
//             double PointToPoint = PointDistance(InputPlane.centroid, copy_plane[i].centroid);
            
//             if((angle < 2.0 || angle > 178.0)){
                
//                 // Calculate Overlap Ratio
//                 // std::cout << "Overlap Ratio " << std::endl;
//                 double Ratio = OverlapRatio(RefPlanePoint, RefPlanePoints[i]);
//                 if(Ratio > OverlapRatiothres){
//                     std::cout << "success overlap" <<std::endl;
//                     SamePlanes.push_back(copy_plane[i]);
//                     SamePlanesInd.push_back(i);
//                     IsSamePlane = true;
//                 }
//             }
//         }
            
//         if(IsSamePlane){
            
//             // input biggest size of SamePlanes
//             size_t MaxSize = RefPlanePoint.size();
//             int ind = -1;
//             for(auto i : SamePlanesInd){
                
//                 if(MaxSize < RefPlanePoints[i].size()){

//                     MaxSize = RefPlanePoints[i].size();
//                     ind = i;
//                 }
//             }    
//             if(ind == -1) {
//                 plane->push_back(InputPlane);
//                 ReferencePlanePoints.push_back(RefPlanePoint);
//             }
//             else {
//                 plane->push_back(copy_plane[ind]);
//                 ReferencePlanePoints.push_back(RefPlanePoints[ind]);
//             }
//             int EraseInd = 0;
//             for(auto i : SamePlanesInd){
//                 copy_plane.erase(copy_plane.begin() + i - EraseInd);
//                 RefPlanePoints.erase(RefPlanePoints.begin() + i - EraseInd);
//                 EraseInd++;
//             }

//         }
//         else{
//             plane->push_back(InputPlane);
//             ReferencePlanePoints.push_back(RefPlanePoint);
//         }
            
//     }
    
// }

// DownsizeFiltering
void getMinMax(std::vector< Eigen::Vector3d > &inCloud, Eigen::Vector3d &minp, Eigen::Vector3d &maxp)
{
    for(size_t i = 0; i < inCloud.size(); i++){
        minp.x() = std::min(minp.x(), inCloud[i].x());
        minp.y() = std::min(minp.y(), inCloud[i].y());
        minp.z() = std::min(minp.z(), inCloud[i].z());
	
        maxp.x() = std::max(maxp.x(), inCloud[i].x());
        maxp.y() = std::max(maxp.y(), inCloud[i].y());
        maxp.z() = std::max(maxp.z(), inCloud[i].z());
    }
}

void DownSizeFiltering(Eigen::Vector3d &LeafSize, std::vector< Eigen::Vector3d > &InCloud, std::vector< Eigen::Vector3d > &OutCloud)
{
	
    // Compute minimum and maximum point values
    Eigen::Vector3d minp(DBL_MAX, DBL_MAX, DBL_MAX), maxp(-DBL_MAX, -DBL_MAX, -DBL_MAX);
    getMinMax(InCloud, minp, maxp);

    // Compute Leaf Count
    Eigen::Vector3i MaxLeafCount(ceil((maxp.x() - minp.x())/LeafSize.x()), ceil((maxp.y() - minp.y())/LeafSize.y()), ceil((maxp.z() - minp.z())/LeafSize.z()));
    std::vector<int> LeafInd;


    // Leaf Idx
    LeafIds.clear();
    for(size_t i = 0; i < InCloud.size(); i++){

        int LeafCount_x = ceil((InCloud[i].x() - minp.x()) / LeafSize.x());
        int LeafCount_y = ceil((InCloud[i].y() - minp.y()) / LeafSize.y());
        int LeafCount_z = ceil((InCloud[i].z() - minp.z()) / LeafSize.z());

        int LeafId = (LeafCount_x - 1) + (LeafCount_y - 1) * MaxLeafCount.x() + (LeafCount_z - 1) * MaxLeafCount.x() * MaxLeafCount.y();
        
        LeafInd.push_back(i);
        LeafIds.push_back(LeafId);

    }
    
    std::sort(LeafInd.begin(), LeafInd.end(), SameLeaf);
    
    for(size_t cp = 0; cp < InCloud.size();){
        Eigen::Vector3d Centroid(InCloud[LeafInd[cp]].x(), InCloud[LeafInd[cp]].y(), InCloud[LeafInd[cp]].z());
        size_t i = cp + 1;
        while(i < InCloud.size() && LeafIds[LeafInd[cp]] == LeafIds[LeafInd[i]]){
            Centroid += InCloud[LeafInd[i]];
            ++i;
        }
        
        Centroid.x() /= (double)(i - cp);
        Centroid.y() /= (double)(i - cp);
        Centroid.z() /= (double)(i - cp);

        OutCloud.push_back(Centroid);

        cp = i;
    }


}

void RemoveClosedPointCloud(std::vector<Eigen::Vector3d> *pointcloud)
{
    std::vector<Eigen::Vector3d> CloudOut;
    CloudOut.resize(pointcloud->size());

    size_t j = 0;
    for (size_t i = 0; i < pointcloud->size(); ++i)
    {
        double distance = PointDistance((*pointcloud)[i]);
        if (distance * distance < LidarToPointsThres * LidarToPointsThres)
            continue;
        CloudOut[j] = (*pointcloud)[i];
        j++;
    }
    
    pointcloud->clear();
    pointcloud->assign(CloudOut.begin(), CloudOut.begin() + j);
}
    
void RemoveNaNFromPointCloud(std::vector<Eigen::Vector3d> *pointcloud)
{
    std::vector<Eigen::Vector3d> CloudOut;
    CloudOut.resize(pointcloud->size());    
    
    size_t j = 0;
    // Remove Nan, Inf
    for(size_t i = 0; i < pointcloud->size(); i++){
        
        bool is_nan_x = isnan((*pointcloud)[i].x());
        bool is_nan_y = isnan((*pointcloud)[i].y());
        bool is_nan_z = isnan((*pointcloud)[i].z());
        if(is_nan_x || is_nan_y || is_nan_z)
            continue;

        bool is_inf_x = isinf((*pointcloud)[i].x());
        bool is_inf_y = isinf((*pointcloud)[i].y());
        bool is_inf_z = isinf((*pointcloud)[i].z());
        if(is_inf_x || is_inf_y || is_inf_z)
            continue;
        
        CloudOut[j] = (*pointcloud)[i];
        j++;
    }
        
    pointcloud->clear();
    pointcloud->assign(CloudOut.begin(), CloudOut.begin() + j);
}
    
int LaserCloudIdxToScanIdx(const int ind, const std::vector<int>& FeatureNumByScan)
{
    int indbyscan = ind;
    for(size_t i = 0; i < FeatureNumByScan.size(); i++){
        indbyscan -= FeatureNumByScan[i];
        if(indbyscan < 0){
            indbyscan += FeatureNumByScan[i];
            // std::cout << "scan id : " << i << std::endl;
            break;
        }
    }

    return indbyscan;
}

int LaserCloudIdxToScanIdx(const int ind, const std::vector<int>& FeatureNumByScan, int* Scanid)
{
    int indbyscan = ind;
    for(size_t i = 0; i < FeatureNumByScan.size(); i++){
        indbyscan -= FeatureNumByScan[i];
        if(indbyscan < 0){
            indbyscan += FeatureNumByScan[i];
            // std::cout << "scan id : " << i << std::endl;
            *Scanid = i;
            break;
        }
    }

    return indbyscan;
}

void DividePointsByChannel( const std::vector<Eigen::Vector3d>& laserPoints, 
                            std::vector<Eigen::Matrix3Xd> *laserCloudScans)
{
    // Calculate Max and Min Vertical Angle
    std::vector<float> Verticalangles;
    Eigen::Vector3d point;
    std::vector<int> PointNumByScanID(N_SCANS, 0);
    
    for(int i = 0; i < cloudSize; i++){
        
        point.x() = laserPoints[i].x();
        point.y() = laserPoints[i].y();
        point.z() = laserPoints[i].z();        
        float angle = VerticalAngle(point);
        Verticalangles.push_back(angle);
    }

    float MaxAngle = *max_element(Verticalangles.begin(), Verticalangles.end());
    float MinAngle = *min_element(Verticalangles.begin(), Verticalangles.end());
    VerticalAngelRatio = (MaxAngle - MinAngle) / N_SCANS;
    
    // Sort Points by channel
    for (int i = 0; i < cloudSize; i++){
        point.x() = laserPoints[i].x();
        point.y() = laserPoints[i].y();
        point.z() = laserPoints[i].z();
        
        float angle = VerticalAngle(point);
        int scanID = 0;
        for(int j = 0; j < N_SCANS; j++){
            if(angle < MinAngle + VerticalAngelRatio * (j + 1)){
                scanID = j;
                break;
            }
        }

        (*laserCloudScans)[scanID](0, PointNumByScanID[scanID]) = point.x();
        (*laserCloudScans)[scanID](1, PointNumByScanID[scanID]) = point.y();
        (*laserCloudScans)[scanID](2, PointNumByScanID[scanID]) = point.z();

        PointNumByScanID[scanID]++;

    }
    
    // resize             
    for(int i = 0; i < N_SCANS; i++)
        (*laserCloudScans)[i].conservativeResize(3, PointNumByScanID[i]);
    
    
    PointIndexByChannel.resize(N_SCANS);
    FeatureNumByScan.clear();
    for(size_t i = 0; i < laserCloudScans->size(); i++){
        FeatureNumByScan.push_back((*laserCloudScans)[i].cols());
        PointIndexByChannel[i].assign((*laserCloudScans)[i].cols(), -1);
        // std::cout << PointIndexByChannel[i] << "  ";

    }
}

void SetPointCloudAndDistance(  const std::vector<Eigen::Matrix3Xd> &laserCloudScans, 
                                std::vector<Eigen::Vector3d> *laserCloud, 
                                std::vector<double> *PointRange)
{
    scanStartInd.resize(N_SCANS);
    scanEndInd.resize(N_SCANS);
    
    int idx = 0;
    for (int i = 0; i < N_SCANS; i++)
    { 
        scanStartInd[i] = laserCloud->size() + 5;
        for(int j = 0; j < laserCloudScans[i].cols(); j++){
            laserCloud->resize(idx + 1);
            Eigen::Vector3d p;
            p << laserCloudScans[i](0, j), laserCloudScans[i](1, j), laserCloudScans[i](2, j);
            (*laserCloud)[idx] = p;
            PointRange->push_back(PointDistance((*laserCloud)[idx]));
            idx++;
        //     // std::cout << PointDistance(laserCloud[idx]) << std::endl;
        }

        scanEndInd[i] = laserCloud->size() - 6;
        // std::cout << " start index : " << scanStartInd[i] << " end index : " << scanEndInd[i] << std::endl;
    }
}

void CalculateCurvature(const std::vector<double>& PointRange)
{
    for (int i = 5; i < cloudSize - 5; i++){         
        double cloudDiff = PointRange[i - 5] + PointRange[i - 4] +
                            PointRange[i - 3] + PointRange[i - 2] +
                            PointRange[i - 1] + PointRange[i + 1] +
                            PointRange[i + 2] + PointRange[i + 3] +
                            PointRange[i + 4] + PointRange[i + 5] +
                            - 10 * PointRange[i];
        cloudCurvature[i] = cloudDiff * cloudDiff;
        cloudSortInd[i] = i;
        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
    }
}

void MarkOccludedPoints(const std::vector<Eigen::Vector3d>& laserCloud, 
                        const std::vector<double>& PointRange)
{
    // Occluded and Parallel beam
    int occluded_cnt = 0;
    int parallel_cnt = 0;
    for (int i = 5; i < cloudSize - 6; i++){
        
        // occluded points
        double depth1 = PointRange[i];
        double depth2 = PointRange[i + 1];
        // double Diff = PointDistance(laserCloud[i + 1], laserCloud[i]);
        double Diff = std::abs(laserCloud[i + 1].x() - laserCloud[i].x());

        if (Diff * Diff< 0.05){
            if (depth1 - depth2 > 0.3){
                occluded_cnt++;
                cloudNeighborPicked[i - 5] = 1;
                cloudNeighborPicked[i - 4] = 1;
                cloudNeighborPicked[i - 3] = 1;
                cloudNeighborPicked[i - 2] = 1;
                cloudNeighborPicked[i - 1] = 1;
                cloudNeighborPicked[i] = 1;
            }else if (depth2 - depth1 > 0.3){
                occluded_cnt++;
                cloudNeighborPicked[i + 1] = 1;
                cloudNeighborPicked[i + 2] = 1;
                cloudNeighborPicked[i + 3] = 1;
                cloudNeighborPicked[i + 4] = 1;
                cloudNeighborPicked[i + 5] = 1;
                cloudNeighborPicked[i + 6] = 1;
            }
        } 
            
        // parallel beam
        double diff1 = PointDistance(laserCloud[i - 1], laserCloud[i]);
        double diff2 = PointDistance(laserCloud[i + 1], laserCloud[i]);

        if (diff1 > 0.02 * PointRange[i] && diff2 > 0.02 * PointRange[i]){
            parallel_cnt++;
            cloudNeighborPicked[i] = 1;
        }
      
    }
    
    std::cout << "Occluded points : " << occluded_cnt << std::endl;
    std::cout << "Parallel points : " << parallel_cnt << std::endl;

}

void DividePointsByEdgeAndPlane(const std::vector<Eigen::Vector3d>& laserCloud, std::vector<std::vector<Eigen::Vector3d>> *CornerPointByChannel)
{

    cornerPointsSharp.clear();
    cornerPointsLessSharp.clear();
    surfPointsFlat.clear();
    surfPointsLessFlat.clear();
    
    //test
    // CornerPointByChannel.clear();
    
    // Edge Points and Plane Points
    for (int i = 0; i < N_SCANS; i++)
    {
        if( scanEndInd[i] - scanStartInd[i] < 6)
            continue;
        std::vector<Eigen::Vector3d> surfPointsLessFlatScan;
        for (int j = 0; j < 6; j++){
            
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6; 
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

            std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);

            // Edge Points
            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--){
                int ind = cloudSortInd[k]; 
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > 0.3)
                {

                    largestPickedNum++;
                    if (largestPickedNum <= 2){                        
                        cloudLabel[ind] = 2;
                        cornerPointsSharp.push_back(laserCloud[ind]);
                        cornerPointsLessSharp.push_back(laserCloud[ind]);
                        int indbyscan = LaserCloudIdxToScanIdx(ind, FeatureNumByScan);
                        PointIndexByChannel[i][indbyscan] = 0;

                        // test
                        (*CornerPointByChannel)[i].push_back(laserCloud[ind]);
                        
            
                    }
                    else if (largestPickedNum <= 20){                        
                        cloudLabel[ind] = 1; 
                        cornerPointsLessSharp.push_back(laserCloud[ind]);
                        int indbyscan = LaserCloudIdxToScanIdx(ind, FeatureNumByScan);
                        PointIndexByChannel[i][indbyscan] = 0;

                        // test
                        (*CornerPointByChannel)[i].push_back(laserCloud[ind]);

                    }
                    else
                        break;
                    
                    

                    cloudNeighborPicked[ind] = 1; 

                    for (int l = 1; l <= 5; l++){
                        double diff = PointDistance(laserCloud[ind + l], laserCloud[ind + l - 1]);
                        if(diff * diff > 0.05)
                            break;
                        
                        cloudNeighborPicked[ind + l] = 1;
                    }
                        

                    for (int l = -1; l >= -5; l--){
                        double diff = PointDistance(laserCloud[ind + l], laserCloud[ind + l + 1]);
                        if(diff * diff > 0.05)
                            break;
                        
                        cloudNeighborPicked[ind + l] = 1;
                    }
                        

                }
            }
            
            // Plane Points
            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++){
                int ind = cloudSortInd[k];

                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < 0.1)
                {
                    cloudLabel[ind] = -1; 
                    surfPointsFlat.push_back(laserCloud[ind]);
                    // RefefrencePlanePointTest
                    int indbyscan = LaserCloudIdxToScanIdx(ind, FeatureNumByScan);
                    PointIndexByChannel[i][indbyscan] = 1;


                    smallestPickedNum++;
                    if (smallestPickedNum >= 10)
                        break;
                    

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++){ 
                        double diff = PointDistance(laserCloud[ind + l], laserCloud[ind + l - 1]);
                        if(diff * diff > 0.05)
                            break;

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--){
                        double diff = PointDistance(laserCloud[ind + l], laserCloud[ind + l - 1]);
                        if(diff * diff > 0.05)
                            break;

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++){
                if (cloudLabel[k] <= 0){

                    int Scanid_ = 0;

                    //downsize test
                    surfPointsLessFlatScan.push_back(laserCloud[k]);

                    int indbyscan = LaserCloudIdxToScanIdx(k, FeatureNumByScan, &Scanid_);
                    PointIndexByChannel[Scanid_][indbyscan] = 1;

                }
            }
        }
        
        DownSizeFiltering(DownSizeLeafSize, surfPointsLessFlatScan, surfPointsLessFlat);
        
    } 
        // std::cout << " flat points num After downsize filtering : " << surfPointsLessFlat.size() << std::endl;
}





void laserCloudHandler(const sort_lidarpoints::feature_infoConstPtr &laserCloudMsg)
{
    if (!systemInited){ 
        systemInitCount++;
        if (systemInitCount >= systemDelay)
            systemInited = true;
        else
            return;
    }
        
    TicToc t_whole;
    TicToc t_prepare;
    
    // Storage pointcloud from ROS msg        
    FeatureInfo = *laserCloudMsg; 
    std::vector<Eigen::Vector3d> laserPoints = ConvertFromROSmsg(FeatureInfo.cloud_undistortion);
    
    // Remove useless points
    RemoveClosedPointCloud(&laserPoints);
    RemoveNaNFromPointCloud(&laserPoints);
    
    cloudSize = laserPoints.size();
    // std::cout << "Point Num  :  " << cloudSize << std::endl; 
    
    // Sort Points by Scan Line
    std::vector<Eigen::Matrix3Xd> laserCloudScans(N_SCANS, Eigen::Matrix3Xd(3, kMaxNumberOfPoints ));
    DividePointsByChannel(laserPoints, &laserCloudScans);
    
    // Total lasercloud and point distance
    std::vector<Eigen::Vector3d> laserCloud;
    std::vector<double> PointRange;
    SetPointCloudAndDistance(laserCloudScans, &laserCloud, &PointRange);

    // printf("prepare time %f \n", t_prepare.toc());

    // Caculate Curvature and Mark Occluded and Parallel Points    
    CalculateCurvature(PointRange);
    MarkOccludedPoints(laserCloud, PointRange);

    // test
    std::vector<std::vector<Eigen::Vector3d>> CornerPointByChannel;
    
    CornerPointByChannel.resize(N_SCANS);
    
    // sort Lidar points to edge and plane
    DividePointsByEdgeAndPlane(laserCloud, &CornerPointByChannel);

    std::vector<std::vector<int>> CornerPointByChannelIdx;
    CornerPointByChannelIdx.resize(N_SCANS);
    for(size_t i = 0; i < CornerPointByChannel.size(); i++){
        CornerPointByChannelIdx[i].resize(CornerPointByChannel[i].size());
    }


    // print edge and plane num
    std::cout << "Total laserCloud num : " << laserCloud.size() << std::endl;
    std::cout << "cornerPointsSharp num : " << cornerPointsSharp.size() << std::endl;
    std::cout << "cornerPointsLessSharp num : " << cornerPointsLessSharp.size() << std::endl;
    std::cout << "surfPointsFlat num : " << surfPointsFlat.size() << std::endl;
    std::cout << "surfPointsLessFlat num : " << surfPointsLessFlat.size() << std::endl;    

    



    


    // Publish Points
    PublishPointCloud(pubLaserCloud, laserCloud, laserCloudMsg->header.stamp, LidarFrame);
    PublishPointCloud(pubCornerPointsSharp, cornerPointsSharp, laserCloudMsg->header.stamp, LidarFrame);
    PublishPointCloud(pubCornerPointsLessSharp, cornerPointsLessSharp, laserCloudMsg->header.stamp, LidarFrame);
    PublishPointCloud(pubSurfPointsFlat, surfPointsFlat, laserCloudMsg->header.stamp, LidarFrame);
    PublishPointCloud(pubSurfPointsLessFlat, surfPointsLessFlat, laserCloudMsg->header.stamp, LidarFrame);


    printf("scan registration time %f ms *************\n", t_whole.toc());
    if(t_whole.toc() > 100)
        ROS_WARN("scan registration process over 100ms");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanRegistration");
    ros::NodeHandle nh("~");

    nh.param<int>("scan_line", N_SCANS, 16);
    nh.param<double>("LidarToPointsThres", LidarToPointsThres, 5.0);

    //printf("scan line number %d \n", N_SCANS);

    if(N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64){
        //printf("only support velodyne with 16, 32 or 64 scan line!");
        return 0;
    }
    // ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, laserCloudHandler);

    subFeature = nh.subscribe<sort_lidarpoints::feature_info>("/velodyne_points", 100, laserCloudHandler);

    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);

    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);

    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);

    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);

    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);

    ros::spin();

    return 0;
}

    


        






    
  

    
    


    
    

    




    
    




    

    

 






    
 
    
    

    




