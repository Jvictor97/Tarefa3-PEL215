// File:          cpp_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>
#include <queue>
#include <utility>
#include <webots/Supervisor.hpp>
#include <webots/GPS.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

void printPair(std::pair<int, int> p) 
{ 
    // Gives first element from queue pair 
    int f = p.first; 
  
    // Gives second element from queue pair 
    int s = p.second; 
  
    std::cout << "(" << f << ", " << s << ") "; 
} 

// Print the Queue of Pairs 
void showQueue(std::queue<std::pair<int, int> > gq) 
{ 
    // Print element untill the 
    // queue is not empty 
    while (!gq.empty()) { 
        printPair(gq.front()); 
        gq.pop(); 
    } 
  
    std::cout << '\n'; 
} 
  

int main(int argc, char **argv) {
 
  float gridMap[125][100];
  
  for(int i = 0; i < 125; i++) {
    for(int j = 0; j < 100; j++) {
      gridMap[i][j] = 0.5;
    }
  }
  
  std::cout<<"gridMap: "<<gridMap[0][0];
  
  std::queue<std::pair<int,int>> q;
  q.push({0, 0});
  q.push({0, 1});
  
  showQueue(q);
  showQueue(q);

  // create the Robot instance.
  Supervisor *robot = new Supervisor();
  GPS *gps = robot->getGPS("gps");
  gps->enable(500);
  
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  //Motor *leftMotor = robot->getMotor("left wheel motor");
  //Motor *rightMotor = robot->getMotor("right wheel motor");
  Field *trans = robot->getSelf()->getField("rotation");
  
  //leftMotor->setPosition(INFINITY);
  //rightMotor->setPosition(INFINITY);
  
  while (robot->step(timeStep) != -1) {
    

    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
    /*const double * values = gps->getValues();
    
    if (values != NULL) 
      std::cout << "x: " << values[0] 
                << "  y: " << values[1] << std::endl;*/
                
    const double *values = trans->getSFRotation();
     std::cout << "Robot rotation: " 
               << values[0] << ' ' << values[1] << ' ' 
               << values[2] << ' ' << values[3] << std::endl;
    
    //leftMotor->setVelocity(2.0);
    //rightMotor->setVelocity(1.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
