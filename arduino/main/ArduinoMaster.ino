#include <ros.h>                        //ROS library
#include <std_msgs/Int32.h> //Documentation on http://wiki.ros.org/std_msgs
#include <std_msgs/Float32.h>
#include <math.h>
#include <SoftwareSerial.h>
//#include <TimerThree.h>     //Custom library pulled from arduino playground website (for interrupts)

ros::NodeHandle nh;             //Initialize ROS node handler
std_msgs::Int32 Distance;       //ROS requires its own data type(s)
ros::Publisher chatter("chatter",&Distance);  //You need this to publish to ROS

#define ADC_REF 3.3             //Maximum voltage on potentiometer
#define FULL_ANGLE 60.0         //Total amount of angles
#define MOTOR_VOLTAGE_PIN 7     //PWM output pin (analog out) for motor control

const byte pingPinL = 53; // Trigger Pin of Ultrasonic Sensor Left
const byte echoPinL = 52; // Echo Pin of Ultrasonic Sensor Left
const byte pingPinR = 47; // Trigger Pin of Ultrasonic Sensor Right //51
const byte echoPinR = 46; // Echo Pin of Ultrasonic Sensor Right    //50

const byte LEDpinR = 44;  //Debug LEDS for sensors (not required for code to operate)
const byte LEDpinL = 50;

const byte bluetoothTx = 10;
const byte bluetoothRx = 11;

bool noGap = false;

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

void rosCallback(const std_msgs::Float32& optimalAngle) {
   if(optimalAngle.data == -500) {
    noGap = true;
   }
}

ros::Subscriber<std_msgs::Float32> sub("master_output", &rosCallback); //You need this to subscribe to ROS


void setup() {

   nh.initNode();       //Intitialize ROS node
   nh.advertise(chatter); //Tell the ROS node to get ready for publishing
   nh.subscribe(sub);     //Tell the ROS node to get ready to subscribe and listen
   Serial.begin(57600); // Starting Serial Terminal, ROS uses 57600 baud
   bluetooth.begin(115200); //bluetooth baud rate
   bluetooth.print("$");    //module setup sequence
   bluetooth.print("$");
   bluetooth.print("$");
   delay(100);

   bluetooth.println("U,9600,N");
   bluetooth.begin(9600);
   
   
   pinMode(pingPinL, OUTPUT);
   pinMode(echoPinL, INPUT);
   pinMode(pingPinR, OUTPUT);
   pinMode(echoPinR, INPUT);

   pinMode(LEDpinL, OUTPUT);
   pinMode(LEDpinR, OUTPUT);
   digitalWrite(LEDpinL, LOW);
   digitalWrite(LEDpinR, LOW);

   analogWrite(MOTOR_VOLTAGE_PIN, 79);   //controls motor voltage via PWM pin and RC circuit
   //Voltage is determined by second argument of function; the maximum voltage is 5V
   //The second argument ranges from 0 to 255 and is directly proportional to 0-5V
   
}

void loop() {
  ultrasonicPulse();
  delay(190);
  static int voltage = 79; //Ratio from 0 to 255

  if(bluetooth.available()) {
    char symbol = (char)bluetooth.read();
    if(symbol=='a'){
      nh.loginfo("killswitch"); //kill
      voltage = 0;
    } else if(symbol=='b') {    //resume
      voltage = 79;
      noGap = false;
    } else if(symbol=='u' && voltage < 96) {    //increment motor power
      voltage += 2;
      noGap = false;
    } else if(symbol=='d') {    //decrement motor power
      voltage -= 2;
    }
    analogWrite(MOTOR_VOLTAGE_PIN, voltage);
  }

  if(noGap) {
    analogWrite(MOTOR_VOLTAGE_PIN, 0);
  }

  nh.spinOnce();  
}

void ultrasonicPulse() {
   long duration, inches, cm;
   
   //Retrieve Data from Left Sensor
   triggerPin(pingPinL);      //Send a trigger to tell left sensor to take a measurement

   duration = pulseIn(echoPinL, HIGH);  //Measures how long the ultrasonic pulse is traveling
   
   inches = microsecondsToInches(duration);   //Unit conversions
   cm = microsecondsToCentimeters(duration);
   
   if(cm>400) {
    inches = -1;
    cm = -1;
   }
   
   if(inches<6 && inches>0) {
    digitalWrite(LEDpinL, HIGH);
   } else {
    digitalWrite(LEDpinL, LOW);
   }

   /*Serial.print("Left: ");    //uncomment to send info directly to USB port
   Serial.print(inches);
   Serial.print("in, ");
   Serial.print(cm);
   Serial.print("cm");
   Serial.println();*/

   Distance.data=cm;            //Sets the special ROS datatype to equal our cm measurement
   chatter.publish(&Distance);  //Publisher sends the data over USB to ROS node
   nh.spinOnce();               //This line is recommended after you've published information to a ROS node

   
   //Retrieve Data from Right Sensor
   triggerPin(pingPinR);     //Repeat same process as above for right sensor

   duration = pulseIn(echoPinR, HIGH);
   
   inches = microsecondsToInches(duration) + 400; //Encode with 400 so master can distinguish between sensors
   cm = microsecondsToCentimeters(duration) + 400;

   if(cm>800) {
    inches = -2;
    cm = -2;
   }

   if(inches<406 && inches>400) {
    digitalWrite(LEDpinR, HIGH);
   } else {
    digitalWrite(LEDpinR, LOW);
   }

   Distance.data=cm;
   chatter.publish(&Distance);
   nh.spinOnce();

}

void triggerPin(int pin){ //Sends a 10us pulse to the trigger pin of HC-SR04 ultrasonic sensor
   digitalWrite(pin, LOW);
   delayMicroseconds(2);
   digitalWrite(pin, HIGH);
   delayMicroseconds(10);
   digitalWrite(pin, LOW);
}

long microsecondsToInches(long microseconds) {
   return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}

/*void testing() {
  static bool state = false;
  state = !state;
  if(state) {
    digitalWrite(LEDpinL, HIGH);
  } else {
    digitalWrite(LEDpinL, LOW);
  }
}*/
