/************************************************************
 * Name: blobs_test.cpp
 * Authors: David Mattia, Gina Gilmartin, Bridge Harrington,
			Daniel Huang
 * Date: 02/14/2015
 *
 * Description: 
 *				 
 *		
 ***********************************************************/
#include <kobuki_msgs/BumperEvent.h> 
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <cmvision/Blob.h>
#include <cmvision/Blobs.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

enum Color {
	ORANGE,
	BLUE
};

// Global Data Structures
struct blob_color {
	int red, green, blue;
	bool seen;
};

// A vector that can add weighted vectors with linear and rotational values
class MotionVector {
public:
	// Default constructor. Set linear speed to @linear
	// and rotational speed to @angular
	MotionVector(double linear = 0.0, double angular = 0.0, unsigned int weight = 0) :
		weight(weight) {
		twist.linear.x = linear;
		twist.angular.z = angular;
	}
	// Set the values of @twist to @linear and @angular with weight @weight
	void updateVector(double linear, double angular, unsigned int w) {
		twist.linear.x = linear;
		twist.angular.z = angular;
		weight = w;
	}
	// Adds the values in @linear and @angular to @twist's 
	// current values
	void addVector(double linear, double angular, unsigned int w) {
		twist.linear.x = ((weight * twist.linear.x) + (w * linear)) / (weight + w);
		twist.angular.z = ((weight * twist.angular.z) + (w * angular)) / (weight + w);
		weight += w;
	}
	MotionVector operator=(const MotionVector &mv) {
		twist.linear.x = mv.twist.linear.x;
		twist.angular.z = mv.twist.angular.z;
		weight = mv.weight;
		return *this;
	}
	MotionVector operator+(const MotionVector &mv) {
		MotionVector toReturn;
		if (weight + mv.weight) {
			toReturn.twist.linear.x = ((weight * twist.linear.x) + (mv.weight * mv.twist.linear.x)) / (weight + mv.weight);
			toReturn.twist.angular.z = ((weight * twist.angular.z) + (mv.weight * mv.twist.angular.z)) / (weight + mv.weight);
			toReturn.weight = weight + mv.weight;
		}
		return toReturn;
	}
	MotionVector operator+=(const MotionVector &mv) {
		*this = *this + mv;	
		return *this;
	}
	// Resets speed, rotation, and weights to 0
	void reset() {
		twist.linear.x = twist.angular.z = weight = 0;
	}
	// Returns the current twist
	geometry_msgs::Twist getTwist() {
		return twist;
	}
	// Prints a nicely formatted string for this MotionVector
	void pretty_print() {
		ROS_INFO("Motion Vector:\n\tLinear: %.2f\n\tAngular: %.2f\n\tWeight: %d", twist.linear.x, twist.angular.z, weight);
	}
private:
	geometry_msgs::Twist twist;
	unsigned int weight;
};

// Global Parameters
const int NUM_COLORS = 2;
struct blob_color colors[NUM_COLORS] = {
	{ORANGE, ORANGE, ORANGE, false},
	{BLUE, BLUE, BLUE, false}
};

MotionVector endpointVector, coneVector, avoidanceVector;
bool shouldFinish = false;

unsigned int cycleNumber = 0;
unsigned int lastCycleNumberChanged = 0;
bool inAvoidance = false;
bool goLeft = true; // Robot goes left around a cone if true, right if false

/************************************************************
 * Function Name: pclCallback								
 * Parameters: a PointCloud message with data
 * Returns: nothing
 * 
 * Description: 
 * 
 **********************************************************/
void pclCallback(const PointCloud::ConstPtr& cloud)
{
	avoidanceVector.reset();
	pcl::PointXYZ closestPoint; // Point with the closest z in the latest point cloud

	float min_z = 1e6;
	BOOST_FOREACH (const pcl::PointXYZ& pt, cloud->points)
	{
		if (pt.z < min_z) {
			closestPoint = pt;
			min_z = pt.z;
		}
	}
	//ROS_INFO("Minimum z is: %.3f", min_z);
	if (min_z < 0.65) {
		if(!inAvoidance && (cycleNumber - lastCycleNumberChanged > 20)) {
			goLeft = !goLeft;
			lastCycleNumberChanged = cycleNumber;
			ROS_INFO("Changing Sides to %d", goLeft);
		}
		inAvoidance = true;

		// There is a close object. Update avoidanceVector
		float linearSpeed = min_z - 0.4;
		float angularDirection = 1.0 - min_z;
		unsigned int weight = 1000;
		if (closestPoint.x < 0) {
			angularDirection *= -1;
		}
		avoidanceVector.updateVector(linearSpeed, angularDirection, weight);
	} else {
		inAvoidance = false;
	}
}

/************************************************************
 * Function Name: blobsCallBack
 * Parameters: const cmvision::Blobs
 * Returns: void
 *
 * Description: This is the callback function of the /blobs topic
 ***********************************************************/
void blobsCallBack (const cmvision::Blobs& blobsIn)
{
	unsigned int centerAround = 175; // Value in x range to center the robot onto (0-600)
	if(goLeft) centerAround = 425;

	coneVector.reset();
	// Find the largest blob
	cmvision::Blob largest_blob;
	unsigned int max_blob_size_seen = 0;
	for(int i = 0; i < blobsIn.blob_count; ++i) {
		if (blobsIn.blobs[i].red == ORANGE && blobsIn.blobs[i].area > max_blob_size_seen) {
			largest_blob = blobsIn.blobs[i];
			max_blob_size_seen = blobsIn.blobs[i].x;
		}
		if (blobsIn.blobs[i].red == BLUE && blobsIn.blobs[i].area > 30000) {
			shouldFinish = true;
		}
	}
	if(blobsIn.blob_count) {
		float angularDirection = 0.0;

		if (largest_blob.x < centerAround + 10) {
			angularDirection = 0.7;
		} else if (largest_blob.x > centerAround - 10) {
			angularDirection = -0.7;
		}
		coneVector.updateVector(0.4, angularDirection, 3);
	}
}
 
/************************************************************
 * Function Name: main
 * Parameters: int argc, char **argv
 * Returns: int
 *
 * Description: This is the main function. It will subscribe
 *				to the /blobs topic and will move to blobs in order
 *				based upon the values in the ros parameter
 *				seek_visit_order
 ***********************************************************/ 
int main(int argc, char **argv)
{
	 ros::init(argc, argv, "blobs_test");
	
	// Create handle that will be used for both subscribing and publishing. 
	ros::NodeHandle n;
	
	//subscribe to /blobs topic 
	ros::Subscriber blobsSubscriber = n.subscribe("/blobs", 100, blobsCallBack);	

	// subscribe to pcl topic
	ros::Subscriber pointCloudSubscriber = n.subscribe<PointCloud>("/camera/depth/points", 1, pclCallback);

	// publish the geometry message twist message
	ros::Publisher velocityPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);

	// Set the loop frequency in Hz.
	ros::Rate loop_rate(10);

	MotionVector mv;
	endpointVector.updateVector(0.2, 0.0, 1);

	while(!shouldFinish && ros::ok())
	{
		mv = endpointVector + coneVector + avoidanceVector;
		//mv.pretty_print();

		velocityPublisher.publish(mv.getTwist());
		ros::spinOnce();
		loop_rate.sleep();
		cycleNumber++;
	} 
	return 0;
}
