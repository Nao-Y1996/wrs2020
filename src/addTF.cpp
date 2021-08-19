#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32MultiArray.h>



void GetPositions(const std_msgs::Float32MultiArray& msg){
    int length = msg.data.size()/3;
    for (int i=0; i<length; i++){
        float x = msg.data[3*i];
        float y = msg.data[3*i+1];
        float z = msg.data[3*i+2];
        std::cout << '[' << x <<','<< y <<','<< z <<']'<< std::endl;
        }
}



int main(int argc, char** argv){
    ros::init(argc, argv, "CreateStableTF");
    ros::NodeHandle node;

    tf::TransformBroadcaster br;
    tf::Transform transform;

    ros::Subscriber sub = node.subscribe("ARmarker_position", 1000, GetPositions);

    ros::spin();
    std::cout << "Start creating stable marker." << std::endl;


//   ros::Rate rate(10.0);
//   while (node.ok()){

//     transform.setOrigin( tf::Vector3(1.0, 0.0, 0.6) );
//     transform.setRotation( tf::Quaternion(-0.5, 0.52, -0.477, 0.5) );
//     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "new_tf"));
    
//     rate.sleep();
//   }
  return 0;
};