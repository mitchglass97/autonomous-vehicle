#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"
#include <sstream>
#include <iostream>
#include <vector>
#include "math.h"
using namespace std;

#define ARRAY_START 0       //start index of raw data array scan->ranges
#define ARRAY_END 1439      //end index of raw data array scan->ranges
#define CAR_WIDTH 72        //Width of the car in cm
#define THRESHOLD 2       //The threshold for determining if a ray is open or closed. (meters)
#define MINIMUM_LENGTH 80   //minimum points in succession required for car to fit in gap
#define TRACK_WIDTH 244     //expected track width in cm
#define LIDAR_OFFSET 30       //offset distance of the lidar position to the wheel edge
#define ULTRASONIC_OFFSET 12   //offset distance of the US sensors to the wheel edge

//==============================================GLOBALS (sorry cs people)========================================================

float rightSensor, leftSensor;	//Right and Left ultrasonic sensor
float danger = (TRACK_WIDTH/9);	//danger zone threshold
float warning = (TRACK_WIDTH/6);	//warning zone threshold
float lidarDistanceLeft;         //Holds average of 5 lidar rays to the immediate left of the vehicle front
float lidarDistanceRight;        //Same as above but right side

bool right_danger, left_danger, right_warning, left_warning;    //Ultrasonic warning flags

float potentiometerAngle;                          //outputAngle is the angle at which the lidar wishes to turn to
float outputAngle;                                 //potentiometerAngle is the current wheel angle
//==============================================STRUCT DECLARATION(S)/FUNCTION PROTOTYPES========================================

struct GapInfo{int startIndex, endIndex, length;};
int angleRound(float numToRound, int multiple);

//==============================================LIDAR DATA INTERPRETTING=========================================================

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    ///////////////GENERAL DEBUG STATEMENTS//////////////////
    //ROS_INFO("Straight Ahead: %f", scan->ranges[ARRAY_START]); //ranges array has 1440 points indexed 0-1439
    ///////////////EVERY DIFFERENT SECTION OF CODE HAS A HEADER EXPLAINING ITS PURPOSE////////////////////////
    ///////////////COMMENTED OUT ROS_INFO STATEMENTS CAN BE USED FOR DEBUGGING////////////////////////////////
    
    float internalAngle;                //holds a running calculation for the outputAngle
    float rangeData[481];               //480 points = 120 degrees
    int sweep[481];                     //Open/Closed array

    /*This block is responsible for grabbing lidar data points to the immediate right or left.
    These data points will be used as ultrasonic data points, raising their flags if distance
    thresholds are met.*/
    lidarDistanceLeft = 0;
    lidarDistanceRight = 0;
    lidarDistanceLeft = (scan->ranges[360]*100)-LIDAR_OFFSET;   //Retrieves the info, converts to cm, and subtracts offset
    lidarDistanceRight = (scan->ranges[1080]*100)-LIDAR_OFFSET;
    ROS_INFO("lidarDistanceLeft: %f | lidarDistanceRight: %f",lidarDistanceLeft,lidarDistanceRight);

    /*This for block is responsible for grabbing points from the raw data
    and isolating them into an array named "rangeData"*/

    //ROS_INFO("Creating rangeData array...\n");
    for(int i=-240;i<240;i++) {
        int index = 480 - (i+240);
        if(i<0) {
            rangeData[index] = scan->ranges[1440+i];
            if(isinf(rangeData[index])) rangeData[index] = 26;
        } else {
            rangeData[index] = scan->ranges[i];
            if(isinf(rangeData[index])) rangeData[index] = 26;
        }
        
        //ROS_INFO("Added to rangeData at index %i: %f", index, rangeData[index]);
    }
    

    /*This for block is responsible for turning "rangeData" into a "binary" array
    named "sweep". 1 denotes that it is past our threshold, 0 denotes it is not.*/

    //ROS_INFO("Creating sweep array...\n");
    for(int i=0;i<480;i++) {
        if(rangeData[i]<THRESHOLD) {
            sweep[i] = 0;
        } else {
            sweep[i] = 1;
        }
        //ROS_INFO("sweep at index %i corresponding to rangeData %f: %i", i, rangeData[i], sweep[i]);
    }

    /*This block of code takes the sweep array and creates a vector called gapArray
    which contains any gaps in the data (consecutive 1's). The data is stored into
    gapArray in blocks of 3. The first number is start index, second is end index,
    and third is gap length.*/
    
    vector<int> gapArray;
    int previous = 0;
    GapInfo tempGap;
    
    tempGap.startIndex = 0;
    tempGap.endIndex = 0;
    tempGap.length = 0;
    
    for(int i=0;i<480;i++) {
        
        if(i!=0) previous = sweep[i-1];

        //ROS_INFO("sweep data at %i: %i, previous: %i",i,sweep[i],previous);

        if(sweep[i] == 1 && i!=479) {

            if((previous==0)) {tempGap.startIndex = i;}
            tempGap.length++;

        } else if(((sweep[i] == 0 && previous == 1) || i==479)){

            tempGap.length--;
            tempGap.endIndex = i-1;
            gapArray.push_back(tempGap.startIndex);
            gapArray.push_back(tempGap.endIndex);
            gapArray.push_back(tempGap.length);
            //ROS_INFO("data added: %i, %i, %i",tempGap.startIndex, tempGap.endIndex, tempGap.length);
            tempGap.length=0;

        }
    }

    /*This for block is responsible for looking at all available gaps and seeing which ones
    the car can fit through. If the car can fit through the gap, it will calculate the center
    of the gap and find the optimal angle required to go towards the center.*/
    
    GapInfo validGap;
    int vector_length;
    float centerpoint,angleCandidate;
    vector<float> angleCandidates;
    

    for(vector<int>::const_iterator i = gapArray.begin(); i != gapArray.end(); i += 3) {
        //ROS_INFO("Vector length: %i", *(i+2));
        vector_length = *(i+2);

        if(vector_length >= MINIMUM_LENGTH) {
            validGap.length = *(i+2);
            validGap.startIndex = *i;
            validGap.endIndex = *(i+1);
            
            centerpoint = (validGap.startIndex + validGap.length/2);
            angleCandidate = (-60 + centerpoint/4);
            angleCandidates.push_back(angleCandidate);
            //ROS_INFO("degree of gap center: %f", angleCandidate);
        }
        
    }
    
    /*As of now, if there are multiple valid gaps the code will choose whichever one is easier
    to turn towards. This is what the following if block does. It only executes if there is a valid gap.*/
    
    if(angleCandidates.size() > 0) {
        float bestAngle = angleCandidates.front();
        //ROS_INFO("Best Angle initial: %f",bestAngle);
        for(vector<float>::const_iterator i = angleCandidates.begin(); i != angleCandidates.end(); i++) {

            if(abs(*i) > bestAngle) {bestAngle = *i;}
            
        }
        
        internalAngle = bestAngle;      
        //ROS_INFO("Best angle: %f", internalAngle);
    } else {
        internalAngle = -500;
        ROS_INFO("No gap detected! Stop or Reverse");
    }

//===================================================ULTRASONIC SENSOR FLAGS=======================================================
    if(angleCandidates.size() > 0) {
	    if(right_danger){
		    internalAngle = -30;
		    ROS_INFO("right danger");
	    }
	    else if(right_warning && internalAngle > 0){
		    internalAngle = ((internalAngle*lidarDistanceRight)/warning)-5;
		    ROS_INFO("right warning");
	    }
        else if(right_warning && internalAngle <= 0) {
            internalAngle = -30;
            ROS_INFO("right warning max");
        }

	    if(left_danger){
		    internalAngle = 30;
		    ROS_INFO("left danger");
	    }
	    else if(left_warning && internalAngle < 0){
		    internalAngle = ((internalAngle*lidarDistanceLeft)/warning)+5;
		    ROS_INFO("left warning");
	    }
        else if(left_warning && internalAngle >= 0) {
            internalAngle = 30;
            ROS_INFO("left warning max");
        }
    }

	outputAngle = angleRound(internalAngle, 5); //Use user-defined angleRound() utility function
    ROS_INFO("Best angle: %f", outputAngle);
}

//===================================================ULTRASONIC DATA INTERPRETTATION=======================================================

void ultrasonicCallback(const std_msgs::Int32::ConstPtr& msg)
{
  if(msg->data > 400 && msg->data < 1000) {
     //ROS_INFO("Right: %dcm", msg->data-400);
     rightSensor = msg->data-400-ULTRASONIC_OFFSET;
  } else if(msg->data > 1000) {
     potentiometerAngle = (msg->data-1050);
  } else if(msg->data==-1) {
     //ROS_INFO("Left: inf");
     leftSensor = -1;
  } else if(msg->data==-2) {
     //ROS_INFO("Right: inf");
     rightSensor = -2;
  } else {
     //ROS_INFO("Left: %dcm", msg->data);
     leftSensor = (msg->data) - ULTRASONIC_OFFSET;
  }


  //============================MAIN ULTRASONIC LOGIC GOES HERE======================================
  if((rightSensor<=danger && rightSensor>0) || lidarDistanceRight<=danger ) {
	right_danger = true;
	right_warning = false;
  } else if((lidarDistanceRight<=warning && lidarDistanceRight>danger)) {
	right_warning = true;
	right_danger = false;
  } else {
	right_danger = false;
	right_warning = false;
  }

  if((leftSensor<=danger && leftSensor>0) || lidarDistanceLeft<=danger) {
	left_danger = true;
	left_warning = false;
  } else if((lidarDistanceLeft<=warning && lidarDistanceLeft>danger)) {
	left_warning = true;
	left_danger = false;
  } else {
	left_danger = false;
	left_warning = false;
  }
}

//==============================================UTILITY FUNCTIONS=========================================================

//function found at:
//https://stackoverflow.com/questions/3407012/c-rounding-up-to-the-nearest-multiple-of-a-number/9194117
int angleRound(float numToRound, int multiple)
{
    int numToRoundi = (int)numToRound;
    if (multiple == 0)
        return numToRound;

    int remainder = abs(numToRoundi) % multiple;
    if (remainder == 0)
        return numToRoundi;

    if (numToRound < 0)
        return -(abs(numToRoundi) - remainder);
    else
        return numToRoundi + multiple - remainder;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "master_node");  // Required, string specifies what the node name will be

  //Must specify a node handle
  ros::NodeHandle t;
  ros::Subscriber sub = t.subscribe("scan", 1000, lidarCallback);
  ros::Subscriber sub2 = t.subscribe("chatter", 1000, ultrasonicCallback);	//Required, specify topic you will subscribe to and function that will be called when data is detected
  ros::Publisher master_pub = t.advertise<std_msgs::Float32>("master_output", 1000);

  ros::Rate loop_rate(10);
  int count = 0;
  while(ros::ok())
  {
    std_msgs::Float32 output;
    //std_msgs::Float32 currentAngle;
    //currentAngle.data = potentiometerAngle + 450;
	output.data = outputAngle;
	master_pub.publish(output);
    //master_pub.publish(currentAngle);


	ros::spinOnce();
  	loop_rate.sleep();
  	++count;
  }

  return 0;
}
