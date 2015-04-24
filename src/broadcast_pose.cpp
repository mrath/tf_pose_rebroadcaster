#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>


int main(int argc, char** argv){
     ros::init(argc, argv, "tf_pose_broadcaster");
     
     ros::NodeHandle node("~");
     
     std::string parentFrame, childFrame, outputTopic;
     
     // Get parameters
     if (!node.getParam("tf_parent",parentFrame))
     {
         ROS_ERROR("Required parameter 'tf_parent' not specified");
         return -1;
     }
     if (!node.getParam("tf_child",childFrame))
     {
         ROS_ERROR("Required parameter 'tf_child' not specified");
         return -1;
     }
     if (!node.getParam("output_topic",outputTopic))
     {
         ROS_INFO("Using default topic for output: /tf_pose");
         outputTopic = "/tf_pose";
     }
     node.getParam("output_topic",outputTopic);
     
     //Create ROS interfaces
     //tf2_ros::Buffer tfBuffer;
     tf::TransformListener tfListener;
     ros::Publisher posePub = node.advertise<geometry_msgs::PoseStamped>(outputTopic, 5);
     
     ros::Rate rate(10.0);
     
     while (node.ok())
     {
         // Try to find the transform
         tf::StampedTransform stampedTransform;
         ros::Time now = ros::Time::now();
         // first, check if the transform exists, and if it doesn't, do nothing
         try
         {
             tfListener.waitForTransform(parentFrame, childFrame, now, ros::Duration(0.5));
         }
         catch (tf::TransformException &ex)
         {
             rate.sleep();
             continue;
         }
         // Next, look up the transform
         try
         {
             tfListener.lookupTransform(parentFrame, childFrame, ros::Time(0), stampedTransform);
         }
         catch (tf::TransformException &ex)
         {
             ROS_WARN("%s",ex.what());
             ros::Duration(0.5).sleep();
             continue;
         }
         //Convert tf datatype to ROS message
         geometry_msgs::TransformStamped poseTF;
         tf::transformStampedTFToMsg(stampedTransform,poseTF);
         //Convert to pose
         geometry_msgs::PoseStamped outputPose; // The pose to rebroadcast
         outputPose.header = poseTF.header;
         outputPose.pose.position.x = poseTF.transform.translation.x;
         outputPose.pose.position.y = poseTF.transform.translation.y;
         outputPose.pose.position.z = poseTF.transform.translation.z;
         outputPose.pose.orientation = poseTF.transform.rotation;
         // publish it
         posePub.publish(outputPose);
         
         rate.sleep();
     }
     return 0;
}
