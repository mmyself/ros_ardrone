#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

tf::TransformBroadcaster *broadcaster;


void odomCallback(nav_msgs::Odometry const &odometry) {
	const double degree = M_PI/180;

    // robot state
    double angle=0;

	// message declarations
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "world";
    odom_trans.child_frame_id = "base_link";
		
	// update transform
    // (moving in a circle with radius=2)
    odom_trans.header.stamp = ros::Time::now();
    
	/*odom_trans.transform.translation.x = cos(angle)*2;
    odom_trans.transform.translation.y = sin(angle)*2;
    odom_trans.transform.translation.z = .7;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);*/


	odom_trans.transform.translation.x = odometry.pose.pose.position.x;
    odom_trans.transform.translation.y = odometry.pose.pose.position.y;
    odom_trans.transform.translation.z = odometry.pose.pose.position.z;
    odom_trans.transform.rotation = odometry.pose.pose.orientation;



    //send the transform
    broadcaster->sendTransform(odom_trans);
	//addTransform(transforms, odom_trans);
  	//broadcaster->sendTransform(transforms);

	// new robot state
	angle += degree/4;

}



int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_to_tf");
    ros::NodeHandle n;
    //tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

	double angle=0;

 	broadcaster = new tf::TransformBroadcaster;	
	
	ros::Subscriber sub;

	sub = n.subscribe("/odom", 10, &odomCallback);

    
 	/*while (ros::ok()) {


		// update transform
        // (moving in a circle with radius=2)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = cos(angle)*2;
        odom_trans.transform.translation.y = sin(angle)*2;
        odom_trans.transform.translation.z = .7;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

        //send the transform
        broadcaster.sendTransform(odom_trans);

		// new robot state
		angle += degree/4;

		// This will adjust as needed per iteration
        loop_rate.sleep();

	}*/

	ros::spin();
  	delete broadcaster;

    return 0;
}
