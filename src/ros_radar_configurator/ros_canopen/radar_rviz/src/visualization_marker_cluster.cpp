#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pb_msgs/ClusterRadar.h>
#include <pb_msgs/dbscanOutput.h>
#include <pb_msgs/individualRadar.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <markers/visualization_marker_cluster.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <cmath>
#include <ros/console.h>

namespace markers
{

    MarkerClassClusterDecoded::MarkerClassClusterDecoded()
        {
           //Topic you want to publish
           pub_ = n_.advertise<visualization_msgs::Marker>("/visualization_marker_cluster", 50);
           //Topic you want to subscribe
           sub_ = n_.subscribe("individualRadar", 50, &MarkerClassClusterDecoded::callback, this);

           //dbscan = n_.subscribe("dbscanOutput", 50, &MarkerClassClusterDecoded::callback, this);

        }

   	//void MarkerClassClusterDecoded::callback(const pb_msgs::ClusterRadar & input, const pb_msgs::dbscanOutput & input1)
    void MarkerClassClusterDecoded::callback(const pb_msgs::individualRadar & input)
      	{	
         	visualization_msgs::Marker sphere;
            visualization_msgs::Marker arrow;
        //    visualization_msgs::TEXT_VIEW_FACING text;
            //.... do something with the input and generate the output...
           
           
            //-----
         //    text.header.frame_id = "/husky_a1_tf/radar";
         // 	text.ns = "markers";
         // 	text.action = visualization_msgs::Marker::ADD;

         // 	text.header.stamp = ros::Time();
         // 	text.lifetime = ros::Duration(0.5);

         // 	text.id = input.id;

         // 	text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

         // 	text.scale.x = 1.0;
         //    text.scale.y = 1.0;
         //    text.scale.z = 1.0;
         // //-------------


         	sphere.header.frame_id = "odom";
         	sphere.ns = "markers";
         	sphere.action = visualization_msgs::Marker::ADD;

         	sphere.header.stamp = ros::Time();
         	sphere.lifetime = ros::Duration(0.5);

         	sphere.id = input.id;

         	sphere.type = visualization_msgs::Marker::SPHERE;

         	sphere.scale.x = 1.0;
            sphere.scale.y = 1.0;
            sphere.scale.z = 1.0;

            //ROS_DEBUG_STREAM_NAMED("test_only %s", input1.clusterLabels);

            if (input.label == 0) //if DBSCAN outputs reflector prediction, color it red
            {
                
                sphere.color.r = 0;
                sphere.color.g = 128;
                sphere.color.b = 0;
                sphere.color.a = 0.3;
                //text.text = "Reflector";

            }
            else if (input.label == 1)
            {
                sphere.color.r = 255;
                sphere.color.g = 0;
                sphere.color.b = 0;
                sphere.color.a = 0.3;
            }
            else if (input.label == -1)
            {
                sphere.color.r = 255;
                sphere.color.g = 0;
                sphere.color.b = 0;
                sphere.color.a = 0.3;
            }


            sphere.color.r = 1.0f;
            sphere.color.a = 1.0;

            sphere.pose.orientation.w = 1.0;
            sphere.pose.position.x = input.long_dist;
            sphere.pose.position.y = input.lat_dist;
            sphere.pose.position.z = 0;




         	pub_.publish(sphere);
            // sensor_msgs::PointCloud2 msg;
            // //.... do something with the input and generate the output...

         	// sphere.header.frame_id = "/radar";
         	// sphere.ns = "markers";
         	// sphere.action = visualization_msgs::Marker::ADD;

         	// sphere.header.stamp = ros::Time();
         	// sphere.lifetime = ros::Duration(0.5);

         	// sphere.id = input.target_id;

         	// sphere.type = visualization_msgs::Marker::SPHERE;

         	// sphere.scale.x = 0.1;
            // sphere.scale.y = 0.1;
            // sphere.scale.z = 0.1;

            // sphere.color.r = 1.0f;
            // sphere.color.a = 1.0;

            // sphere.pose.orientation.w = 1.0;
            // sphere.pose.position.x = input.longitude_dist;
            // sphere.pose.position.y = input.lateral_dist;
            // sphere.pose.position.z = 0;

         	// pub_.publish(sphere);

      	}

        
       //--------------------------------------------------------------------

    // MarkerClassFiltered::MarkerClassFiltered()
    //     {
    //        //Topic you want to publish
    //        pub_ = n_.advertise<std_msgs::UInt8MultiArray>("/visualization_marker_filtered", 50);

    //        //Topic you want to subscribe
    //        sub_ = n_.subscribe("dbscanOutput", 50, &MarkerClassFiltered::callback, this);
    //     }


    // void MarkerClassFiltered::callback(const pb_msgs::dbscanOutput & input1)
    // {	
    //         std_msgs::UInt8MultiArray labels;
    //         //.... do something with the input and generate the output...
    //         labels.layout.dim[0] = input1.clusterLabels[0];

    //         pub_.publish(labels);



    // }    

    // MarkerClassFiltered::MarkerClassFiltered()
    //     {
    //        //Topic you want to publish
    //        pub_ = n_.advertise<visualization_msgs::Marker>("/visualization_marker_filtered", 50);

    //        //Topic you want to subscribe
    //        sub_ = n_.subscribe("dbscanOutput", 50, &MarkerClassFiltered::callback, this);
    //     }

    // void MarkerClassFiltered::callback(const pb_msgs::dbscanOutput & input)
    //     {   
    //         visualization_msgs::Marker cube;
            
    //         points.type = visualization_msgs::Marker::CUBE;
    //         cube.scale.x = 1.0;
    //         cube.scale.y=1.0;
    //         cube.scale.z=1.0;


    //         //.... do something with the input and generate the output...

    //         // points.header.frame_id = line_strip.header.frame_id = "/radar";
    //         // points.ns = line_strip.ns = "markers";
    //         // points.action = line_strip.action = visualization_msgs::Marker::ADD;
    //         // points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

    //         // points.header.stamp = line_strip.header.stamp = ros::Time();
    //         // points.lifetime = line_strip.lifetime = ros::Duration(0.2);

    //         // points.id = input.obstacle_id / 10;
    //         // line_strip.id = input.obstacle_id;

    //         // points.type = visualization_msgs::Marker::POINTS;
    //         // line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    //         // points.scale.x = 0.1;
    //         // points.scale.y = 0.1;
    //         // line_strip.scale.x = 0.1;

    //         // if (input.meas_state == 0 || input.meas_state == 4)
    //         // {
    //         //     line_strip.color.r = 1.0f;
    //         //     line_strip.color.a = 1.0;
    //         // }
    //         // else if (input.meas_state == 1 || input.meas_state == 5)
    //         // {
    //         //     line_strip.color.g = 1.0f;
    //         //     line_strip.color.a = 1.0; 
    //         // }
    //         // else if (input.meas_state == 2)
    //         // {
    //         //     line_strip.color.b = 1.0f;
    //         //     line_strip.color.a = 1.0;
    //         // }
    //         // else if (input.meas_state == 3)
    //         // {
    //         //     line_strip.color.b = 0.8f;
    //         //     line_strip.color.g = 0.8f;
    //         //     line_strip.color.a = 1.0;
    //         // }


    //         // // Create the vertices for the points and lines
    //         // for (uint32_t i = 0; i < 5; i++)
    //         // {
    //         //     float x;
    //         //     float y;
    //         //     float z;
    //         //     if (i==0 || i==4)
    //         //     {
    //         //         x = input.longitude_dist + (input.width/2)*tan((input.orientation_angle*M_PI)/180);
    //         //         y = input.lateral_dist + input.width / 2;
    //         //         z = input.length / 2;
    //         //     }
    //         //     else if (i==1)
    //         //     {
    //         //         x = input.longitude_dist - (input.width/2)*tan((input.orientation_angle*M_PI)/180);
    //         //         y = input.lateral_dist - input.width / 2;
    //         //         z = input.length / 2;
    //         //     }
    //         //     else if (i==2)
    //         //     {
    //         //         x = input.longitude_dist - (input.width/2)*tan((input.orientation_angle*M_PI)/180);
    //         //         y = input.lateral_dist - input.width / 2;
    //         //         z = - input.length / 2;
    //         //     }
    //         //     else if (i==3)
    //         //     {
    //         //         x = input.longitude_dist + (input.width/2)*tan((input.orientation_angle*M_PI)/180);
    //         //         y = input.lateral_dist + input.width / 2;
    //         //         z = - input.length / 2;
    //         //     }
   
    //         //     geometry_msgs::Point p;
    //         //     p.x = x;
    //         //     p.y = y;
    //         //     p.z = z;
  
    //         //     line_strip.points.push_back(p);

    //         // }
  
    //         pub_.publish(cube);

    //     } 

}  //Namespace markers
