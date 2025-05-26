  // File:          ctrl_1.cpp
// Date:         04.10.2023
// Description:  praca inz
// Author:       Mateusz Smolarczyk
// Modifications:

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/PositionSensor.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <stdbool.h>
# define PI           3.14159265358979323846  /* pi */
# define DISTANCE_BETWEEN_WHEELS 105.4 //mm
# define WHEEL_R 21 //mm
// All the webots classes are defined in the "webots" namespace
using namespace webots;
static const char *infared_sensor_names[8] = {
  "rear right infrared sensor",
  "right infrared sensor",
  "front right infrared sensor",
  "front infrared sensor", 
  "front left infrared sensor",
  "left infrared sensor",
  "rear left infrared sensor",
  "rear infrared sensor"};


typedef struct Point {
  double x;
  double y;};

typedef struct Robot_position {
  double x;
  double y;
  double angle;};
  
typedef struct Obstacle {
  Point *points;
  int number_of_points;
  Point center;};

void odometry(double left_pos_diff, double right_pos_dif, Robot_position *robot_position,int front){
  double dl = left_pos_diff * WHEEL_R;
  double dr = right_pos_dif * WHEEL_R;
  double theta = (dr - dl)/(DISTANCE_BETWEEN_WHEELS);
  double d = (dl + dr)/2.0 * front;
  if (fabs(theta) > 0.04){
      d = ((dl + dr)*sin(theta/2))/theta * front;}
    
  robot_position->x = robot_position->x + d * cos(robot_position->angle + theta/2.0);
  robot_position->y = robot_position->y + d * sin(robot_position->angle + theta/2.0);
  double temp_angle = robot_position->angle + theta;
  if (temp_angle < -PI){
  temp_angle = temp_angle + 2*PI;}
  if (temp_angle > PI){
  temp_angle = temp_angle - 2*PI;} 
  robot_position->angle = temp_angle;
}

// double rad_to_position(double rad_of_robot){
  // double dist_of_wheel = DISTANCE_BETWEEN_WHEELS * rad_of_robot / 2;
  // return dist_of_wheel/WHEEL_R;}

double angle_to_point(Point end_point, Robot_position robot_position){
  double x_diff = end_point.x - robot_position.x;
  double y_diff = end_point.y - robot_position.y;

  double absolute;
  if (x_diff == 0){absolute = PI/2;} 
  else{absolute = atan((y_diff)/(x_diff));}

  if(x_diff <= 0 and y_diff < 0){
    absolute = absolute - PI;}
  if(x_diff <= 0 and y_diff > 0){
    absolute = absolute + PI;}
  double result = absolute - robot_position.angle;
  if (result > PI){result = result - 2*PI;}
  if (result < -PI){result = result + 2*PI;}
  return result;}

Point distance_to_point(Robot_position robot_position, double sensor_angle, double distance){
  double angle = robot_position.angle + sensor_angle;

  double x = robot_position.x + distance * cos(angle);
  double y = robot_position.y + distance * sin(angle);
  return Point{x,y};}

// void turn_to_point(Point end_point, Robot_position robot_position, double* right_pos, double* left_pos){
  // double rad_angle = angle_to_point(end_point, robot_position);
  // double turn_angle = rad_to_position(rad_angle);
  // *right_pos = *right_pos + turn_angle;
  // *left_pos = *left_pos - turn_angle;}

// void from_point_to_point(double end_x, double end_y, double x, double y, double angle, double* right_pos, double* left_pos){
  // double distance = sqrt(pow(end_x - x, 2) + pow(end_y - y, 2));
  // *right_pos = *right_pos + distance/WHEEL_R;
  // *left_pos = *left_pos + distance/WHEEL_R;}

// double degree_to_position(int degree){
  // double rad_of_robot = degree * PI / 180;
  // double dist_of_wheel = DISTANCE_BETWEEN_WHEELS * rad_of_robot / 2;
  // return dist_of_wheel/WHEEL_R;}
  
double sensor_to_distance(double x, const double data[39]) {
    for (int i = 1; i < 13; i++) {
        if (x > data[i * 3 + 1]) {
            double x0 = data[i * 3 + 1];
            double x1 = data[(i-1) * 3 + 1];
            double y0 = data[i * 3];
            double y1 = data[(i-1) * 3];
            double distance_m = y0 + (x - x0) * (y1 - y0) / (x1 - x0);
            return distance_m * 1000 + 70.4;
        }
    }
}

void diagnosis(const bool *used_sensors,  bool *turn_right, bool *front, int *avoid_direction, int *way){
  int right_front_points = 0;
  int left_front_points = 0;
  int right_rear_points = 0;
  int left_rear_points = 0;

  int direction_points = 0; // prawo ujemne, lewo dodatnie
  
  if(used_sensors[0]){left_rear_points += 3;}
  if(used_sensors[1]){
    left_rear_points += 1;
    right_front_points += 1;}
  if(used_sensors[2]){right_front_points += 3;}
  if(used_sensors[3]){
    right_front_points += 2;
    left_front_points += 2;}
  if(used_sensors[4]){left_front_points += 3;}
  if(used_sensors[5]){
    left_front_points += 1;
    right_rear_points += 1;}
  if(used_sensors[6]){right_rear_points += 3;}
  if(used_sensors[7]){
    left_rear_points += 2;
    right_rear_points += 2;}

  int max_points = right_front_points;
    *avoid_direction = 1;
    *turn_right = 0;
  if (left_front_points > max_points) {
    *avoid_direction = -1;
    *turn_right = 1;
    max_points = left_front_points;
  }

  if (right_rear_points > max_points) {
    *avoid_direction = 1;
    *turn_right = 0;
    *front = 0;
    *way = -1;
    max_points = right_rear_points;
  }

  if (left_rear_points > max_points) {
    *avoid_direction = -1;
    *turn_right = 1;
    *front = 0;
    *way = -1;
    max_points = left_rear_points;
  }
}
  
void oscilation_set(bool right, bool right_front, bool front, bool left_front, bool left, int *poziom_wachan, double *oscilation_speed, int *left_angle, int *right_angle){
  if(right_front and front and left_front){
    *poziom_wachan = 33;
    *oscilation_speed = 0.9;} // 22.5 stopnia 33 0.9
  else{
    if((front) or (right_front and left_front)){  
      *poziom_wachan = 48;
      *oscilation_speed = 1.2;} //45 stopni 48 1.2
    else{
     if(right_front){
        *left_angle = 90;
        *right_angle = 0;} 
      if(left_front){
        *left_angle = 0;
        *right_angle = 90;}// Błąd - poprawić to miejsce, źle ify 
      else{
        if(right){
          *left_angle = 135;
          *right_angle = 0;} //90 stopni
         if(left){
            *left_angle = 0;
            *right_angle = 135;} //135 stopni
      }
    }
  }
}

void oscilation_set_side(bool right, bool right_front, bool front, bool left_front, bool left, int *poziom_wachan, double *oscilation_speed, int *left_angle, int *right_angle){
  if(right_front and front and left_front){
    *poziom_wachan = 33;
    *oscilation_speed = 0.9;} // 22.5 stopnia 33 0.9
  else{
    if((front and left and right) or (right_front and left_front) or (front and left and right_front) or (front and right and left_front)){  
      *poziom_wachan = 40;
      *oscilation_speed = 0.9;} //45 stopni 48 1.2
    else{
      if(right_front){
        *left_angle = 135;
        *right_angle = 45;} //135 prawo 45 lewo
      if(left_front){
        *left_angle = 45;
        *right_angle = 135;} //135 prawo 45 lewo
      if(front){
        *left_angle = 90;
        *right_angle = 90;} //45 prawo 135 lewo
    }
  }
}

int stop_turn_parameters(int angle){
  if(angle == 45){return 40;}
  if(angle == 90){return 120;}
  if(angle == 135){return 180;}
  if(angle == 0){return 0;}
}
// int hit_detection(double acc, int break_flag){
  // if (acc < -3 and break_flag == 0){
    // return 1;
  // }
  // else{
  // return 0;}
// }


int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  int timeStep = (int)robot->getBasicTimeStep();

  // motor setup
  Motor *left_motor = robot->getMotor("left wheel motor");
  Motor *right_motor = robot->getMotor("right wheel motor");  
  double motor_speed = PI/3;
  left_motor->setVelocity(motor_speed);
  right_motor->setVelocity(motor_speed);
  left_motor->setPosition(INFINITY);
  right_motor->setPosition(INFINITY);
  int iter = 0;
  bool turn_left_done = 0;
  int stop_and_turn_iter = 140;
  int go_front = 1;
  int poziom_wachan = 0;
  int kierunek_wachan = 1;
  double left_proportion = 1;
  double right_proportion = 1;
  double oscilation_speed = 0;
  double oscilation_speed_side = 0;
  double oscilation_speed_front = 0;
  int poziom_wachan_side = 0;
  int poziom_wachan_front = 0;
  int left_angle_front = 0;
  int right_angle_front = 0;
  int left_angle_side = 0;
  int right_angle_side = 0;
  int right_angle;
  int left_angle;
  //distance sensor setup
  bool used_sensors[8] = {0,1,1,1,1,0,0,0};
    // bool used_sensors[8] = {0,0,0,1,0,0,0,0};

  // bool used_sensors[8] = {0,0,0,0,0,1,1,1};

  DistanceSensor *infared_distanceSensor[8];
  for (int i = 0; i < 8; ++i) {
      infared_distanceSensor[i] = robot->getDistanceSensor(infared_sensor_names[i]);
      infared_distanceSensor[i]->enable(timeStep);}
      
  double infared_sensors_angle[9] = {-PI *3/4, -PI/2, -PI/4, 0, PI/4, PI/2, PI*3/4, PI, -PI};
  const double *lookup_table = infared_distanceSensor[0]->getLookupTable();
  for(int i = 0; i < 13; i++){
      std::cout << lookup_table[i*3+0] << ", "<< lookup_table[i*3+1] << ", "<< lookup_table[i*3+2] << std::endl;}
  bool obstacle_avoid_flag = 0;
  int obstacle_idx;
 
  bool turn_right;
  bool front = 1;
  int avoid_direction = 0; // prawo: -1, lewo: 1, zero: obojętnie
  int way = 1;
  
  diagnosis(used_sensors, &turn_right, &front, &avoid_direction, &way);
  if(front){
    oscilation_set(used_sensors[1], used_sensors[2], used_sensors[3], used_sensors[4], used_sensors[5], &poziom_wachan_front, &oscilation_speed_front, &left_angle_front, &right_angle_front);
    if(turn_right){
      oscilation_set_side(used_sensors[2], used_sensors[3], used_sensors[4], used_sensors[5], used_sensors[6], &poziom_wachan_side, &oscilation_speed_side, &left_angle_side, &right_angle_side);
    }
    else{
      oscilation_set_side(used_sensors[0], used_sensors[1], used_sensors[2], used_sensors[3], used_sensors[4], &poziom_wachan_side, &oscilation_speed_side, &left_angle_side, &right_angle_side);
    }
  }
  else{
    oscilation_set(used_sensors[1], used_sensors[0], used_sensors[7], used_sensors[6], used_sensors[5], &poziom_wachan_front, &oscilation_speed_front, &left_angle_front, &right_angle_front);
    if(turn_right){
      oscilation_set_side(used_sensors[2], used_sensors[1], used_sensors[0], used_sensors[7], used_sensors[6], &poziom_wachan_side, &oscilation_speed_side, &left_angle_side, &right_angle_side);
    }
    else{
      oscilation_set_side(used_sensors[0], used_sensors[7], used_sensors[6], used_sensors[5], used_sensors[4], &poziom_wachan_side, &oscilation_speed_side, &left_angle_side, &right_angle_side);
    }
  }
  
  std::cout << "front: " << front << std::endl;
  std::cout << "avoid_direction: " << avoid_direction << std::endl;
    std::cout << "turn_right: " << turn_right << std::endl;
  std::cout << "avoid_direction: " << avoid_direction << std::endl;
    std::cout << "poziom_wachan_front: " << poziom_wachan_front << std::endl;
  std::cout << "poziom_wachan_side: " << poziom_wachan_side << std::endl;
  std::cout << "oscilation_speed_front: " << oscilation_speed_front << std::endl;
  std::cout << "oscilation_speed_side: " << oscilation_speed_side << std::endl;

  std::cout << "left_angle_front: " << left_angle_front << std::endl;
  std::cout << "right_angle_front: " << right_angle_front << std::endl;
  std::cout << "left_angle_side: " << left_angle_side << std::endl;
  std::cout << "right_angle_side: " << right_angle_side << std::endl;

  if(not front){
    infared_sensors_angle[0] = PI *1/4;
    infared_sensors_angle[1] = PI/2;
    infared_sensors_angle[2] = PI*3/4;
    infared_sensors_angle[3] = PI;
    infared_sensors_angle[4] = -PI*3/4;
    infared_sensors_angle[5] = -PI/2;
    infared_sensors_angle[6] = -PI/4;
    infared_sensors_angle[7] = 0;
    motor_speed = motor_speed * (-1);}
  //position sensor setup
  PositionSensor *position[2];
  position[0] = robot->getPositionSensor("left wheel sensor");
  position[1] = robot->getPositionSensor("right wheel sensor");
  position[0]->enable(timeStep);
  position[1]->enable(timeStep);
  
  double last_left_position = position[0]->getValue();
  double last_right_position = position[1]->getValue();
  double actual_left_position = position[0]->getValue();
  double actual_right_position = position[1]->getValue();

  Point end_point{1000, 0};
  double start_angle = PI;
  if (front){start_angle = 0;}
  Robot_position robot_position{0, 0, start_angle};
  Robot_position robot_absolute_position{0, 0, start_angle}; //position with angle to target
  double direction;
  //accelerometer setup
  // Accelerometer *acc = robot->getAccelerometer("accelerometer");
  // acc->enable(timeStep);
  //obstacles
  int number_of_obstacles = 0;
  int number_of_avoid_points = 0;
  int avoid_point_idx = 0;
  Obstacle *obstacle_list = (Obstacle *)malloc(50 * sizeof(Obstacle));
  Obstacle *avoid_list = (Obstacle *)malloc(20 * sizeof(Obstacle));
  Point path[30000];
  int iteration = 0;
  //-------------------------------------------------------------------------
  // ----------------------------- MAIN LOOP --------------------------------
  //-------------------------------------------------------------------------
  while (robot->step(timeStep) != -1) {
    
     //read sensors
    double inf_dist_value[8];
    for (int i = 0; i < 8; ++i) {
      if(used_sensors[i]){inf_dist_value[i] = infared_distanceSensor[i]->getValue();}}
    // const double *value_acc = acc->getValues();
    actual_left_position = position[0]->getValue();
    actual_right_position = position[1]->getValue();
    double left_pos_diff = actual_left_position - last_left_position;
    double right_pos_diff = actual_right_position - last_right_position;
    // if(not front){
    // left_pos_diff = left_pos_diff * (-1);
    // right_pos_diff = right_pos_diff * (-1);}
    //calculate odometry
    if(not isnan(left_pos_diff) and not isnan(right_pos_diff)){
      odometry(left_pos_diff, right_pos_diff, &robot_position, way);
      odometry(left_pos_diff * left_proportion, right_pos_diff * right_proportion, &robot_absolute_position, way);
      robot_absolute_position.x = robot_position.x;
      robot_absolute_position.y = robot_position.y;
      path[iteration] = Point{robot_position.x, robot_position.y};


      iteration += 1;}
    
    if(obstacle_avoid_flag == 0){direction = angle_to_point(end_point, robot_absolute_position);}
    double dist_to_target = sqrt(pow(robot_position.x - end_point.x, 2) + pow(robot_position.y - end_point.y, 2));

    // warunek dotarcia do celu
    if (dist_to_target < 10){
      left_motor->setPosition(actual_left_position);
      right_motor->setPosition(actual_right_position);
      left_motor->setVelocity(0);
      right_motor->setVelocity(0);
      break;
    }
      
    std::cout << "x: " << robot_position.x << std::endl;
    std::cout << "y: " << robot_position.y << std::endl;
    std::cout << "angle: " << robot_position.angle << std::endl;
    std::cout << "obstacle_avoid_flag: " << obstacle_avoid_flag << std::endl;
    std::cout << "robot_absolute_position.angle: " << robot_absolute_position.angle << std::endl;
    // std::cout << "direction: " << direction << std::endl;
    // std::cout << "left_pos_diff: " << left_pos_diff << std::endl;
    // std::cout << "right_pos_diff: " << right_pos_diff << std::endl;
    // std::cout << "left_proportion: " << left_proportion << std::endl;
    // std::cout << "right_proportion: " << right_proportion << std::endl;
    // std::cout << "right_pos_diff: " << right_pos_diff << std::endl;
    // std::cout << "number_of_obstacles: " << number_of_obstacles << std::endl;
    // std::cout << "iteration: " << iteration << std::endl;
    // std::cout << "iter: " << iter << std::endl;

    // Map points
    for(int i = 0; i < 8; i++){
      if(used_sensors[i]){
        if (inf_dist_value[i] > 180){
          double distance_to_obstacle = sensor_to_distance(inf_dist_value[i], lookup_table);
          Point point = distance_to_point(robot_position, infared_sensors_angle[i], distance_to_obstacle);
              // std::cout << "point.x: " << point.x << std::endl;
    // std::cout << "point.y: " << point.y << std::endl;

          bool new_obstacle = 1;
          for(int j = 0; j < number_of_obstacles; j++){
            double dist_to_center = sqrt(pow(point.x - obstacle_list[j].center.x, 2) + pow(point.y - obstacle_list[j].center.y, 2));
            if(dist_to_center < 100){
              new_obstacle = 0;
              obstacle_list[j].number_of_points += 1;
              obstacle_list[j].points = (Point *)realloc(obstacle_list[j].points, (obstacle_list[j].number_of_points) * sizeof(Point));
              obstacle_list[j].points[obstacle_list[j].number_of_points - 1] = point;
              obstacle_list[j].center.x = (obstacle_list[j].center.x * (obstacle_list[j].number_of_points - 1) + point.x)/obstacle_list[j].number_of_points;
              obstacle_list[j].center.y = (obstacle_list[j].center.y * (obstacle_list[j].number_of_points - 1) + point.y)/obstacle_list[j].number_of_points;          
            }
          }
          if(new_obstacle){
            number_of_obstacles += 1;
            Obstacle obstacle = Obstacle{(Point *)malloc(sizeof(Point)), 1, point};
            obstacle.points[0] = point;
            obstacle_list[number_of_obstacles - 1] = obstacle;
          }
          
          
          // avoid obstacle
          if(obstacle_avoid_flag){
          // use only side radars
            bool detect_avoid_point_flag = 0;
            if(front){
              if(turn_right and i > 1){detect_avoid_point_flag = 1;}
              if(turn_right == 0 and (i < 5 or i == 7)){detect_avoid_point_flag = 1;}}
            else{
              if(turn_right and  (i < 4 or i > 5)){detect_avoid_point_flag = 1;}
              if(turn_right == 0 and (i > 2 or i == 0)){detect_avoid_point_flag = 1;}}
            if(detect_avoid_point_flag){
              new_obstacle = 1;
              for(int j = 0; j < number_of_avoid_points; j++){
                double dist_to_center = sqrt(pow(point.x - avoid_list[j].center.x, 2) + pow(point.y - avoid_list[j].center.y, 2));
                if(dist_to_center < 20){
                  new_obstacle = 0;
                  avoid_list[j].number_of_points += 1;
                  avoid_list[j].points = (Point *)realloc(avoid_list[j].points, (avoid_list[j].number_of_points) * sizeof(Point));
                  avoid_list[j].points[avoid_list[j].number_of_points - 1] = point;
                  avoid_list[j].center.x = (avoid_list[j].center.x * (avoid_list[j].number_of_points - 1) + point.x)/avoid_list[j].number_of_points;
                  avoid_list[j].center.y = (avoid_list[j].center.y * (avoid_list[j].number_of_points - 1) + point.y)/avoid_list[j].number_of_points;          
                }
              }
              if(new_obstacle){
                if (avoid_point_idx > 19){avoid_point_idx = 0;}
                if(number_of_avoid_points < 20){number_of_avoid_points += 1;}
                else{free(avoid_list[avoid_point_idx].points);}
                Obstacle obstacle = Obstacle{(Point *)malloc(sizeof(Point)), 1, point};
                obstacle.points[0] = point;
                avoid_list[avoid_point_idx] = obstacle;
                avoid_point_idx += 1;
              }        
            }
          }
        }
      }
    }
     // Detect obstacle on way
    if (obstacle_avoid_flag == 0){
       for(int i = 0; i < number_of_obstacles; i++){
         if(obstacle_list[i].number_of_points > 3){     
           double angle_to_obstacle = angle_to_point(obstacle_list[i].center, robot_absolute_position);
               // std::cout << "angle_to_obstacle: " << angle_to_obstacle << std::endl;

           if (angle_to_obstacle > -PI/4 and angle_to_obstacle < PI/4){
             obstacle_idx = i;
             obstacle_avoid_flag = 1;
             if(avoid_direction == 0){
               if(angle_to_obstacle >= 0){turn_right = 1;}
               else{turn_right = 0;}
             }
           }
         }
       }
     }
     
     // Avoid obstacle
     else{
       double angle_to_corner;
       int idx_point;
       if(turn_right){angle_to_corner = PI/2;}
       else{angle_to_corner = -PI/2;}

       for(int i = 0; i < number_of_avoid_points; i++){
         // find corner point
         double angle_to_obstacle_point = angle_to_point(avoid_list[i].center, robot_absolute_position);

         if(angle_to_obstacle_point > -PI/2 and angle_to_obstacle_point < PI/2){

           if(turn_right){
             if(angle_to_obstacle_point < angle_to_corner){
               angle_to_corner = angle_to_obstacle_point;
               idx_point = i;}
           }
           else{
             if(angle_to_obstacle_point > angle_to_corner){
             angle_to_corner = angle_to_obstacle_point;
             idx_point = i;}
           }
         }
       }
        // calculate direction   
       double dist_to_point = sqrt(pow(robot_position.x - avoid_list[idx_point].center.x, 2) + pow(robot_position.y - avoid_list[idx_point].center.y, 2));
       double beta;
       if(dist_to_point > 160){beta = asin(160/dist_to_point);}
       else{beta = asin(80/dist_to_point) * 2;}
       
       double angle_to_target = angle_to_point(end_point, robot_absolute_position);

       if(turn_right){
         angle_to_corner = angle_to_corner - beta;
         if((angle_to_corner < angle_to_target) or ((angle_to_corner > angle_to_target) and (angle_to_target < -2.5))){direction = angle_to_corner;}
         else{direction = angle_to_target;}
         }
       else{
         angle_to_corner = angle_to_corner + beta;
         if((angle_to_corner > angle_to_target) or ((angle_to_corner < angle_to_target) and (angle_to_target > 2.5))){direction = angle_to_corner;}
         else{direction = angle_to_target;}
       }                
       
     //stop avoid obstacle
       if(fabs(direction - angle_to_target) < 0.05){
         bool stop = 1;
         for(int i = 0; i < number_of_avoid_points; i++){
           double angle_to_obstacle = angle_to_point(avoid_list[i].center, robot_absolute_position);
           if (angle_to_obstacle > -PI/4 and angle_to_obstacle < PI/4){stop = 0;}
         }
         if(stop){
           obstacle_avoid_flag = 0;
           for(int i = 0; i < number_of_avoid_points; i++){free(avoid_list[i].points);}
           number_of_avoid_points = 0;
           avoid_point_idx = 0;
         }
       }
     }
     
    // sterowanie napedami
    double right_offset = 0;
    double left_offset = 0;
    double right_ster = 0;
    double left_ster = 0;

    if(front){
      if(direction < 0){
        right_ster = motor_speed*direction * 20;
        left_ster = motor_speed*direction * (-10);
        }
      else{
        right_ster = motor_speed*direction * 10;
        left_ster = motor_speed*direction * (-20);}  
      if(right_ster > motor_speed){right_ster = motor_speed;}
      if(right_ster < -2*motor_speed){right_ster = -2*motor_speed;}
      if(left_ster > motor_speed){left_ster = motor_speed;}
      if(left_ster < -2*motor_speed){left_ster = -2*motor_speed;}
    }
    else{
      if(direction < 0){
        right_ster = motor_speed*direction * (-10);
        left_ster = motor_speed*direction * 20;}
      else{
        right_ster = motor_speed*direction * (-20);
        left_ster = motor_speed*direction * 10;}  
      if(right_ster < motor_speed){right_ster = motor_speed;}
      if(right_ster > -2*motor_speed){right_ster = -2*motor_speed;}
      if(left_ster < motor_speed){left_ster = motor_speed;}
      if(left_ster > -2*motor_speed){left_ster = -2*motor_speed;}
    }
    
    //czy omijamy cel?
    if(obstacle_avoid_flag == 0){
      oscilation_speed = oscilation_speed_front;
      poziom_wachan = poziom_wachan_front;
      left_angle = left_angle_front;
      right_angle = right_angle_front;
    }
    if(go_front == 1 and obstacle_avoid_flag == 1){
      oscilation_speed = oscilation_speed_side;
      poziom_wachan = poziom_wachan_side;
      left_angle = left_angle_side;
      right_angle = right_angle_side;
    }
    
    right_proportion = 1;
    left_proportion = 1;
        std::cout << "right_angle " << right_angle << std::endl;
    std::cout << "left_angle " << left_angle << std::endl;

    //ruch oscylacyjny
    if(right_angle == 0 and left_angle == 0){
      iter += kierunek_wachan;
      if(iter > poziom_wachan){
        kierunek_wachan = kierunek_wachan * (-1);
      } 
      if(iter < -poziom_wachan){
        kierunek_wachan = kierunek_wachan * (-1);
      }
  
      if(dist_to_target > 140){
        left_offset = left_ster + kierunek_wachan*motor_speed*oscilation_speed;
        right_offset = right_ster - kierunek_wachan*motor_speed*oscilation_speed;}
      else{
        left_offset = left_ster;
        right_offset = right_ster;}
      if(motor_speed + left_offset == 0){left_proportion = 0;}
      else{left_proportion = (motor_speed + left_ster)/(motor_speed + left_offset);}
      if(motor_speed + right_offset == 0){right_proportion = 0;}
      else{right_proportion = (motor_speed + right_ster)/(motor_speed + right_offset);}
    }
    //ruch ze zatrzymanie i obrotem
    else{
      stop_and_turn_iter += go_front;
      if(stop_and_turn_iter > 140){
        go_front = 0;
        right_proportion = 0;
        left_proportion = 0;
        int poziom_wachan_right = stop_turn_parameters(right_angle);
        int poziom_wachan_left = stop_turn_parameters(left_angle);
        iter += kierunek_wachan;

        if(iter > poziom_wachan_right){
            kierunek_wachan = kierunek_wachan * (-1);
        } 
        if(iter < -poziom_wachan_left){
          kierunek_wachan = kierunek_wachan * (-1);
          turn_left_done = 1;
        }
        if(turn_left_done and iter == 0){
          stop_and_turn_iter = 0;
          go_front = 1;
          kierunek_wachan = 1;
          turn_left_done = 0;
        }
        
        right_offset = -kierunek_wachan*motor_speed - motor_speed;
        left_offset = kierunek_wachan*motor_speed - motor_speed;
      }
      else{
        right_offset = right_ster;
        left_offset = left_ster;
      }
    }
    
    right_motor->setVelocity(motor_speed + right_offset);
    left_motor->setVelocity(motor_speed + left_offset);
    last_left_position = actual_left_position;
    last_right_position = actual_right_position;
    // std::cout << "left_spped: " << motor_speed + left_offset << std::endl;
    // std::cout << "right_speed: " << motor_speed + right_offset << std::endl;
    // std::cout << "left_offset: " << left_offset << std::endl;
    // std::cout << "right_offset: " << right_offset << std::endl;
    // std::cout << "left_ster: " << left_ster << std::endl;
    // std::cout << "right_ster: " << right_ster << std::endl;
    // std::cout << "left_proportion: " << left_proportion << std::endl;
    // std::cout << "right_proportion: " << right_proportion << std::endl;

  };

  // Enter here exit cleanup code.
        std::ofstream centre("centre.csv");
      centre << "x,y" << std::endl;

  for(int i = 0; i < number_of_obstacles; i++){  
    // char buffer[16];
    // snprintf(buffer, sizeof(buffer), "output_file%d.txt", i);    

        std::string nazwaPliku = "plik" + std::to_string(i) + ".csv";
      std::ofstream plik(nazwaPliku);
      plik << "x,y" << std::endl;

    for(int j = 0; j < obstacle_list[i].number_of_points; j++){
      plik << obstacle_list[i].points[j].x <<", "<< obstacle_list[i].points[j].y << std::endl;}
      centre << obstacle_list[i].center.x << ", " << obstacle_list[i].center.y << std::endl;

      free(obstacle_list[i].points);}
    
        std::string nazwaPliku2 = "path.csv";
      std::ofstream plik(nazwaPliku2);
            plik << "x,y" << std::endl;

   for(int j = 0; j < 30000; j++){
      plik << path[j].x <<", "<< path[j].y << std::endl;}
  delete robot;
  return 0;
}
