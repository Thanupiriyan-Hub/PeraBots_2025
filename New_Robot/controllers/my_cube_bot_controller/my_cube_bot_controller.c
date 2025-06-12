#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/camera.h>
#include <stdio.h>


#define TIME_STEP 32
#define SPEED     6.0  // rad/s

int main() {
  wb_robot_init();

  // Get motor devices
  WbDeviceTag left_motor  = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");

  // Set motors to velocity control mode (infinite position)
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  // Set desired velocities
  wb_motor_set_velocity(left_motor,  SPEED);   // forward
  wb_motor_set_velocity(right_motor, SPEED);   // forward

    // Enable camera
  WbDeviceTag cam = wb_robot_get_device("camera");
  wb_camera_enable(cam, TIME_STEP);
  
  // Main loop
  while (wb_robot_step(TIME_STEP) != -1) {
    // Robot will keep moving forward
  }

  wb_robot_cleanup();
  return 0;
}
