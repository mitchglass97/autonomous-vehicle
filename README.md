# Autonomous Vehicle

The goal of this project was to create a fully autonomous go-kart that could navigate through standard loop race tracks. This was a 2018-2019 senior design project at Valparaiso University. I worked on a team with 5 talented engineering students (mechanical, electrical, & computer). Together, we worked on the project which had been passed down to us from a team from the previous year. The first team essentially put together a working standard go-kart, and our mission was to implement autonomous steering.

# Algorithm 

In simple terms, the navigation algorithm directs the go-kart to travel towards the furthest point that the LIDAR can see at any time. The inputs to the algorithm are from a LIDAR sensor and two ultrasonic senors. The output is simply a desired wheel angle. A stepper motor turns the wheels (rack and pinion steering) until the reading for the current wheel angle (potentiometer attactched to front left wheel) matches our desired angle with 1 degree tolerance.

The LIDAR unit, which is the crux of the algorithm, is mounted to the front of the go-kart and has a 360 degree view in a 2D plane. The LIDAR continuously spins and returns a distance measurement for each ~1 degree. We only use data from 60 degrees on either side of straight ahead. We used a moving average to smooth out these values.

The ultrasonic sensors are mounted on the sides of the go-kart, and their purpose is to flag whether the go-kart is too close to a wall or another vehicle. If a sensor returns a distance measurement less than a certain threshold, the algorithm either dampens or completely overrides the desired angle.

# Technology

RPLIDAR-A3: LIDAR sensor  
HC-SR04: Ultrasonic sensors . 
Arduino Mega: Microcontroller board used to handle sensor data  
Dell Inspiron Micro Desktop i3050: Onboard computer . 
SparkFun Bluetooth Mate Gold: Bluetooth sensor (start and stop with a phone app) . 

# The Team

Leonardo Chai  
Raquel Haro  
Mitchell Glass  
Alejandro Luna  
Stamatis Moundoulas  
Stanley Igras  
Advisor: Professor El-Howayek  
