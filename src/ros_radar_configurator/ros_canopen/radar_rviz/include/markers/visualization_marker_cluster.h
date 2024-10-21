#ifndef MARKERS_VISUALIZATION_MARKER_CLUSTER_H
#define MARKERS_VISUALIZATION_MARKER_CLUSTER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pb_msgs/ContiRadar.h>
#include <pb_msgs/dbscanOutput.h>
#include <pb_msgs/individualRadar.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <iostream>
#include <cmath>

namespace markers
{

   class MarkerClassClusterDecoded
   {
      public:

         MarkerClassClusterDecoded();
         //void callback(const pb_msgs::ClusterRadar & input,const pb_msgs::dbscanOutput & input1);
         void callback(const pb_msgs::individualRadar & input);

         
         //void callback1(const pb_msgs::dbscanOutput &input1);

      private:
         
         ros::NodeHandle n_; 
         ros::Publisher pub_;
         ros::Subscriber sub_;
         //ros::Subscriber dbscan;
   };//End of class

   // class MarkerClassFiltered
   // {
   //    public:

   //       MarkerClassFiltered();

   //       void callback(const pb_msgs::ClusterRadar & input);

   //    private:
         
   //       ros::NodeHandle n_; 
   //       ros::Publisher pub_;
   //       ros::Subscriber sub_;

   // };//End of class

};  // namespace markers

#endif