# Rover pathfinding in real time using RC car
This project aims to create a real-time pathfinding and obstacle avoidance assistance system for a human-driven RC-car navigating towards a single designated point. While the operator retains control of the vehicle, our objective is to create a system that provides intelligent guidance, suggesting optimal paths and alerting the driver to potential obstacles in real-time.

This involves integrating environmental sensors to perceive the surroundings, implementing algorithms to calculate efficient and safe paths to the target, identifying the location of the RC car, and developing a viewer to convey this information to the human driver.

The RC Car is driven on a pre-calculated short path through a 6m x 8m grid, which is displayed on a web dashboard. A mobile node will be attached to the top of the RC car. As the car is driven through the grid, obstacles are detected and this data is sent via Bluetooth to the Base Node. This data is interpreted, and a new shortest path is calculated and the dashboard is updated with the new path. Obstacle detection is done through two ultrasonic sensors that are rotated with a servo motor for a 360-degree sonar.

View more information about the project here [https://github.com/s-shalomi/sisyphus-red/wiki/Project-Overview]

# Running the project
## Pre-requisities
- Hardware as detailed here[https://github.com/s-shalomi/sisyphus-red/wiki/Project-Overview]
- WSL Virtual Environment with Zephyr SDK
- RC Car must have sensors and mobile mode attached to it

Each component must be built and flashed separately.

## Base Node:

## Mobile Node:

## Ultrasonic Sensor:

## Accelerometer and Gyroscope:

## Web Dashboard:


# sensor node: 
build sensor node with:  
> ```west build -b thingy52/nrf52832 mycode/apps/sensor --pristine```  

flash sensor node with:  
> ```west flash --runner jlink```  

view sensor node with:  
> ```JLinkRTTLogger -Device nrf52832_xxaa A -RTTChannel 1 -if SWD -Speed 4000 ~/rtt.log```  
> ```nc localhost 19021```  

# mobile node
build mobile node with:  
>  ```west build -b nrf52840dk/nrf52840 --pristine auto mycode/apps/mobile```  

flash mobile node with:  
> ```west flash --runner jlink```  

view mobile node with:  
> ```screen /dev/ttyACM0 115200```  
