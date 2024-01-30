#include <ros/ros.h>

#include <thread>  
#include <chrono>

#include <termios.h>
#include <unistd.h>
#include <signal.h>

#include "types.h"
#include "world.h"
#include "robot.h"
#include "lidar.h"
#include "utils.h"

int main(int argc, char** argv) {

  ros::init(argc, argv, "mover_node");
  ros::NodeHandle nh("/");
 
  int NUM_ROBOTS = -1;
  NUM_ROBOTS = stoi(argv[1]);

  vector<ros::Publisher> publishers_vector;
  for (int i=0; i < NUM_ROBOTS; i++) {
    ros::Publisher foo_pub  = nh.advertise<geometry_msgs::Twist>("robot_" + to_string(i) + "/cmd_vel", 1000);
    publishers_vector.push_back(foo_pub);
  }

  struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  bool select_robot = true; 
  int robot_idx = -1;
  bool clear = false;
  bool quit = false;
  
  string input;
  geometry_msgs::Twist msg;
  ros::Rate rate(10);


  while (ros::ok()) {

    if (quit){
      cout << "Killing..." << endl; 
      // killTerminal();
      break;
    }

    if (select_robot) {
      tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // deactivate getchar terminal functionalities

      if (clear) {
        system("cls||clear"); // clear terminal screen
        clear = false;
      }

      while (true) {

        cout << "\nSelect robot from 0 to " << NUM_ROBOTS-1 << endl; 
        cin >> input;

        if (input == "exit") killTerminal();

        try {
          robot_idx = stoi(input);
          if (robot_idx < 0 || robot_idx > NUM_ROBOTS-1) {
            system("cls||clear"); 
            cout << "Invalid input. Please try again." << endl;
          }
          else {
            cout << "\nRobot " << robot_idx << " choosen" << "\n" << endl;
            cout << "Move the robot with 'w', 'a', 's', 'd'.\n" << endl;
            cout << "Press 'c' to change robot or 'q' to exit.\n" << endl;
            break;
          }
        } 
        catch (const exception& e) {
          system("cls||clear"); 
          cout << "Invalid input. Please try again." << endl;
        }
      }
      select_robot = false;
      char none = getchar(); // just to ignore Enter signal
      tcsetattr(STDIN_FILENO, TCSANOW, &newt); // reactivate getchar term functionalities
    }

    msg.linear.x = 0.0;
    msg.angular.z = 0.0;

    this_thread::sleep_for(chrono::milliseconds(25)); // sleep for x milliseconds
    publishers_vector[robot_idx].publish(msg);
    ros::spinOnce();
 
    char ch = getchar(); 

    switch (ch) {
      case 'w': clearTerminal(); cout << "forward\n"; msg.linear.x = 3.0; cout.flush(); break;
      case 's': clearTerminal(); cout << "back\n"; msg.linear.x = -3.0; cout.flush(); break;
      case 'd': clearTerminal(); cout << "right\n"; msg.angular.z = -2.5; cout.flush(); break;
      case 'a': clearTerminal(); cout << "left\n"; msg.angular.z = 2.5; cout.flush(); break;
      case 'c': {
        clearTerminal(); 
        select_robot = true; 
        clear = true;
        break;
      }
      case 'q': {
        clearTerminal(); 
        cout << "Quitting...\n"; cout.flush(); 
        quit = true;
        break;
      }
      default: clearTerminal(); cerr << "Invalid command: " << ch << endl; cout.flush(); break;
    }
     

    publishers_vector[robot_idx].publish(msg);
    ros::spinOnce();

  }
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // deactivate getchar terminal functionalities
  killTerminal();
  return 0;
}
