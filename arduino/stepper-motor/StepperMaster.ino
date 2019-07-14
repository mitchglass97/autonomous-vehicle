#include <ros.h>                        //ROS library
#include <std_msgs/Float32.h>
#include <math.h>

ros::NodeHandle nr;             //Initialize ROS node handler

#define POTENTIOMETER_DATA A1   //Analog in pin for wheel angle potentiometer

//Stepper motor global control variables
const byte stepperPulse = 22;
const byte stepperDir = 23;
long currentMicros = 0; long previousMicros = 0;
float optAngle = 0;


void rosCallback(const std_msgs::Float32& optimalAngle) { //This is called upon receiving info from ROS
   optAngle = optimalAngle.data;
}

ros::Subscriber<std_msgs::Float32> sub("master_output", &rosCallback); //You need this to subscribe to ROS


void setup() {
  // put your setup code here, to run once:
  nr.initNode();       //Intitialize ROS node
  nr.subscribe(sub);     //Tell the ROS node to get ready to subscribe and listen
  Serial.begin(57600); // Starting Serial Terminal, ROS uses 57600 baud

  
    //FOLLOWING 3 LINES ARE FOR STEPPER MOTOR
   pinMode(stepperPulse, OUTPUT);
   pinMode(stepperDir, OUTPUT);
   digitalWrite(stepperDir, LOW);

}

void loop() {
  // put your main code here, to run repeatedly:
  static float currAngle = 0;
  static byte counter = 0;

  float useAngle = optAngle;
  if(useAngle < -21) {
    useAngle = -20;
  } else if(useAngle > 27) {
    useAngle = 25;
  }
  
  /*if(abs(currAngle - useAngle) < 2) {
    currAngle = measureAngle();
  } else {
    currAngle = measureAngle();
    //nr.loginfo("Executing Step...");
    stepperControl(currAngle, useAngle);      //step command, takes in current angle and optimal angle
    
  }*/
  
  while(1) {  //keep stepping until current angle = optimal angle
    currAngle = measureAngle();
    useAngle = optAngle;
    if(useAngle < -21) {
      useAngle = -20;
    } else if(useAngle > 21) {
      useAngle = 20;
    }
    
    stepperControl(currAngle,useAngle);
    nr.spinOnce();
    //nr.loginfo("step");
    if((currAngle > useAngle-1) && (currAngle < useAngle+1)) {break;}
     
  } 

  //nr.spinOnce();
  memory = useAngle;  //Once done stepping, remember current used angle
  //nr.loginfo("Turn completed");
  
  while(1) {  //Once 3 optAngle points in a row are not the same as the one stored, repeat loop
    if((useAngle > memory-1) && (useAngle < memory+1) && (currAngle > memory-1) && (currAngle < memory+1)) {counter=0;} else {counter++;}
    currAngle = measureAngle();
    useAngle = optAngle;
    if(useAngle < -21) {
      useAngle = -20;
    } else if(useAngle > 21) {
      useAngle = 20;
    }  
    if(counter==3) {break;}
    nr.spinOnce();
  }
  
  //nr.spinOnce();

}

void stepperControl(float currAngle, float desiredAngle) {
   static uint8_t turnState;
   static int turnSpeed = 1000; //lower number = higher speed

  /*if(desiredAngle < -21) {
    desiredAngle = -20;
  } else if(desiredAngle > 27) {
    desiredAngle = 25;
  }*/
  
   if(currAngle > desiredAngle) {
    turnState = HIGH;
   } else if(currAngle < desiredAngle) {
    turnState = LOW;
   } else {
    return;
   }
  
  digitalWrite(stepperDir, turnState);

  currentMicros = micros();

   if(currentMicros - previousMicros >= turnSpeed){
  
      previousMicros = currentMicros;
      
      digitalWrite(stepperPulse,HIGH);
      
      delayMicroseconds(250); //Set Value
      
      digitalWrite(stepperPulse,LOW);
    
   }
}

float measureAngle() {
  int potValue = analogRead(POTENTIOMETER_DATA); //0 - 474, -30 - 400, 30 - 569 //0 - 374, -30 - 294, 30 - 460 //Center: 31-32 min: 0 max: 

  //float voltage = (float)potValue * ADC_REF / 569; // Integer value here depends on potentiometer
  //float degree = ((voltage * FULL_ANGLE) / ADC_REF)-30;

  float degree = ((potValue-294)/2.767)-30;

  //Serial.println(potValue); //debug statement
  //Serial.print(degree);
  //Serial.print("\n");
  
  //Angle.data=degree + 1050; //1050 is used to encode the message so master code can distinguish from ultrasonic data
  //chatter.publish(&Angle);
  //nh.spinOnce();
  char payload[25];  //debug lines for printing while connected to ROS
  sprintf(payload,"%d",angleRound(degree,1));
  nr.loginfo(payload);
  
  return degree;
}

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
