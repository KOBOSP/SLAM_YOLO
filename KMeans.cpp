#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <string>

//rviz
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

// darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

//synchronizer
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>


const double FLOATEPS(10e-6);
//const double M_PI=3.14159265358979323846


class KMeans
{
public:
    int m_dimNum;
    int m_clusterNum;
    int m_maxIterNum;
    double m_endError;
    std::vector<Eigen::Vector3d> m_means;

    KMeans(int clusterNum,int maxIterNum,double endError):m_clusterNum(clusterNum),m_maxIterNum(maxIterNum),m_endError(endError),m_dimNum(3){};

    int getKMeansLabel(Eigen::Vector3d sample){
        int label=-1;
        double dist = -1;
        for (int i = 0; i < m_clusterNum; i++){
            double temp = calcDistance(sample, m_means[i]);
            if (temp < dist || label == -1){
                dist = temp;
                label = i;
            }
        }
        return label;
    }

    double calcDistance(Eigen::Vector3d A, Eigen::Vector3d B){
        return ((A-B).norm());
    }

    int ClusterKMeans(std::vector<Eigen::Vector3d> data){
        double lastdist=0,currdist = 0;
        int unchanged = 0,iterNum = 0;;
        bool loop = true;
        std::vector<Eigen::Vector3d> tmpclustersum;
        std::vector<int> tmpclusternum;
        for (int i = 0; i < m_clusterNum; i++){
            m_means.push_back(data[rand()%data.size()]);
            tmpclustersum.push_back(Eigen::Vector3d(0.0,0.0,0.0));
            tmpclusternum.push_back(0);
        }
        while (loop)
        {
            lastdist=currdist;
            tmpclustersum.clear();
            for (int i = 0; i < m_clusterNum; i++){
                currdist=0;
                tmpclustersum.push_back(Eigen::Vector3d(0.0,0.0,0.0));
                tmpclusternum[i]=0;
            }
            for(int i=0;i<data.size();i++){
                int label=getKMeansLabel(data[i]);
                currdist+=calcDistance(data[i],m_means[label]);
                tmpclustersum[label]+=data[i];
                tmpclusternum[label]++;
            }
            for (int i = 0; i < m_clusterNum; i++) {
                m_means[i]=tmpclustersum[i]/tmpclusternum[i];
            }

            iterNum++;
            if (fabs(lastdist - currdist) < m_endError * lastdist){
                unchanged++;
            }
            if (iterNum >= m_maxIterNum || unchanged >= 3){
                loop = false;
            }
            printf("%d/%d,%lf\n",iterNum,m_maxIterNum,currdist);
        }
        return 0;
    }
};





