/*
 * File:          ctrl_1c.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <stdio.h>
#include <stdlib.h>
#include <webots/motor.h>
#include <webots/accelerometer.h>

/*
 * You may want to add macros here.
 */
// #define TIME_STEP 64

#define NUMBER_OF_INFRARED_SENSORS 8
static const char *infrared_sensors_names[NUMBER_OF_INFRARED_SENSORS] = {
  // turret sensors
  "rear left infrared sensor", "left infrared sensor", "front left infrared sensor", "front infrared sensor",
  "front right infrared sensor", "right infrared sensor", "rear right infrared sensor", "rear infrared sensor"};

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  int time_step = (int)wb_robot_get_basic_time_step();

  WbDeviceTag infrared_sensors[NUMBER_OF_INFRARED_SENSORS];
  for (int i = 0; i < NUMBER_OF_INFRARED_SENSORS; ++i) {
      infrared_sensors[i] = wb_robot_get_device(infrared_sensors_names[i]);
      wb_distance_sensor_enable(infrared_sensors[i], time_step);}
   /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
   
   const double *table[13][3];
   *table = strcpy(wb_distance_sensor_get_lookup_table(infrared_sensors[3]));
   for (int i =0; i<13; i++){
      for (int j = 0; j<3; j++){
         printf("%f   ", *table[i][j]);
      }
      printf("/n");
   }
   WbDeviceTag left_motor, right_motor;
   left_motor = wb_robot_get_device("left wheel motor");
   right_motor = wb_robot_get_device("right wheel motor");
   wb_motor_set_position(left_motor, INFINITY);
   wb_motor_set_position(right_motor, INFINITY);
   wb_motor_set_velocity(left_motor, 20.0);
   wb_motor_set_velocity(right_motor, 20.0);
   
  int last_display_second = 0;

  while (wb_robot_step(time_step) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

    /* Process sensor data here */
    int display_second = (int)wb_robot_get_time();
    if (display_second != last_display_second) {
      last_display_second = display_second;
      printf("time = %d [s]\n", display_second);
      for (int i = 0; i < NUMBER_OF_INFRARED_SENSORS; ++i)
          printf("- infrared sensor('%s') = %f\n", infrared_sensors_names[i],
                 wb_distance_sensor_get_value(infrared_sensors[i]));
     }
    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
