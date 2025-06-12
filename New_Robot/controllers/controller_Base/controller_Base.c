#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/camera.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP        32
#define MAX_LOG_ENTRIES 20000
#define WHEEL_RADIUS     0.0205    // meters
#define AXLE_LENGTH      0.052     // meters
#define FORWARD_SPEED    6.0       // rad/s

#define TURN_SPEED1       5.0       // rad/s
#define TURN_SPEED2       4.0       // rad/s
#define TURN_SPEED3       3.0       // rad/s
#define TURN_SPEED4       2.0       // rad/s


// Lap mode: 1 = learn & log, 2 = replay
#define LAP 1

// Log entry for pose + wheel commands
typedef struct {
  double x, y, theta;
  double vl, vr;
} LogEntry;
static LogEntry log_data[MAX_LOG_ENTRIES];
static int      log_count = 0;

// Pose & encoder state
static double pose_x = 0.0, pose_y = 0.0, pose_theta = 0.0;
static double last_left = 0.0, last_right = 0.0;

// Device tags
static WbDeviceTag left_motor, right_motor, left_encoder, right_encoder, cam;

// Helpers to get Webots timestep
static int get_time_step() {
  static int ts = -1;
  if (ts < 0)
    ts = wb_robot_get_basic_time_step();
  return ts;
}

// Single simulation step
static void step_sim() {
  if (wb_robot_step(get_time_step()) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

// Update differential-drive pose
static void update_pose() {
  double l = wb_position_sensor_get_value(left_encoder);
  double r = wb_position_sensor_get_value(right_encoder);
  double dl = (l - last_left) * WHEEL_RADIUS;
  double dr = (r - last_right) * WHEEL_RADIUS;
  double dc = (dl + dr) * 0.5;
  double dtheta = (dr - dl) / AXLE_LENGTH;
  pose_x     += dc * cos(pose_theta + dtheta*0.5);
  pose_y     += dc * sin(pose_theta + dtheta*0.5);
  pose_theta += dtheta;
  last_left  = l;
  last_right = r;
}

// Log one step
static void log_step(double vl, double vr) {
  if (log_count < MAX_LOG_ENTRIES) {
    log_data[log_count].x     = pose_x;
    log_data[log_count].y     = pose_y;
    log_data[log_count].theta = pose_theta;
    log_data[log_count].vl    = vl;
    log_data[log_count].vr    = vr;
    log_count++;
  }
}


// Write Lap1 log
static void write_log() {
  FILE *f = fopen("lap1_log.txt","w");
  if (!f) return;
  for (int i = 0; i < log_count; ++i)
    fprintf(f,"%.6f %.6f %.6f %.6f %.6f\n",
            log_data[i].x, log_data[i].y, log_data[i].theta,
            log_data[i].vl, log_data[i].vr);
  fclose(f);
}

int main() {
  wb_robot_init();

  // Motors
  left_motor  = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor,  0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // Encoders
  left_encoder  = wb_robot_get_device("left wheel sensor");
  right_encoder = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_encoder,  get_time_step());
  wb_position_sensor_enable(right_encoder, get_time_step());

  // Camera
  cam = wb_robot_get_device("camera");
  wb_camera_enable(cam, get_time_step());

  // Warm up sensors
  for (int i = 0; i < 10; ++i) step_sim();
  last_left  = wb_position_sensor_get_value(left_encoder);
  last_right = wb_position_sensor_get_value(right_encoder);

#if LAP == 2
  // Load replay log
  FILE *f = fopen("lap1_log.txt","r");
  if (!f) {
    printf("Error: cannot open lap1_log.txt\n");
    return 1;
  }
  while (fscanf(f,"%lf %lf %lf %lf %lf",
                &log_data[log_count].x,
                &log_data[log_count].y,
                &log_data[log_count].theta,
                &log_data[log_count].vl,
                &log_data[log_count].vr) == 5)
    log_count++;
  fclose(f);
#endif

  int replay_idx = 0;
  int Alightcount = 0;
  
  // Main control loop
  while (wb_robot_step(get_time_step()) != -1) {
    if (LAP == 1) {
      // 1) Update pose
      update_pose();

      // 3) Color-based region scan (unchanged)
      const unsigned char *img = wb_camera_get_image(cam);
      int width  = wb_camera_get_width(cam);
      int height = wb_camera_get_height(cam);
      int red = 0;
      int LB1=0,LB2=0,LB3=0,LB4=0,LW1=0,LW2=0,LW3=0,LW4=0,RB1=0,RB2=0,RB3=0,RB4=0,RW1=0,RW2=0,RW3 =0,RW4 = 0;
      int x0 = 0, x1 = 104, y0 = height - 30;
      for (int y = y0; y < height; ++y) {
        for (int x = x0; x < x1; ++x) {
          int r = wb_camera_image_get_red(img,width,x,y);
          int g = wb_camera_image_get_green(img,width,x,y);
          int b = wb_camera_image_get_blue(img,width,x,y);
          if (r>200&&g<30&&b<30) {
            if (x>width/2-10 && x<width/2+10 && y > 65) red++;
          } else if (r<150&&g<150&&b<150) {
            if      (x<13)       LB4++;
            else if (x<26)       LB3++;
            else if (x<39)       LB2++;
            else if (x<52)       LB1++;
            else if (x<65)       RB1++;
            else if (x<78)       RB2++;
            else if (x<91)       RB3++;
            else                 RB4++;
          } else if (r>150&&g>150&&b>150) {
            if      (x<13)       LW4++;
            else if (x<26)       LW3++;
            else if (x<39)       LW2++;
            else if (x<52)       LW1++;
            else if (x<65)       RW1++;
            else if (x<78)       RW2++;
            else if (x<91)       RW3++;
            else                 RW4++;
          }
        }
      }

      // 4) Decision
      double vl_cmd=0.0, vr_cmd=0.0;
      printf("Ltotal=%d\n", LW4 + LW3 + LW2 + LW1);
      // printf("LW4=%d\n", LW4);
      if (red>0) {
        printf("RED detected — stopping\n");
      } else if (RB1>RW1) {
        printf("Turn Left in Speed 5\n");
        Alightcount = 0;
        vl_cmd = -TURN_SPEED1; vr_cmd= TURN_SPEED1;
      } else if (LB1>LW1) {
        printf("Turn Right in Speed 5\n");
        Alightcount = 0;
        vl_cmd = TURN_SPEED1;  vr_cmd=-TURN_SPEED1;
      } else if (RB2>RW2) {
        printf("Turn Left in Speed 4\n");
        vl_cmd = -TURN_SPEED2; vr_cmd= TURN_SPEED2;
      } else if (LB2>LW2) {
        printf("Turn Right in Speed 4\n");
        vl_cmd = TURN_SPEED2;  vr_cmd=-TURN_SPEED2;
      } else if (RB3>RW3) {
        printf("Turn Left in Speed 3\n");
        vl_cmd = -TURN_SPEED3; vr_cmd= TURN_SPEED3;
      } else if (LB3>LW3) {
        printf("Turn Right in Speed 3\n");
        vl_cmd = TURN_SPEED3;  vr_cmd=-TURN_SPEED3;
      } 
      else if (RB4>RW4) {
        printf("Turn Left in Speed 4\n");
        vl_cmd = -TURN_SPEED4; vr_cmd= TURN_SPEED4;
      } else if (LB4>LW4) {
        printf("Turn Right in Speed 4\n");
        vl_cmd = TURN_SPEED3;  vr_cmd=-TURN_SPEED3;
      }
      //else if ((LW4 + LW3 + LW2 + LW1 ) > 1450) {
      // else if (Alightcount > 5) {
        // if ((LW4 + LW3 + LW2 + LW1 ) > 1450) {
        // printf("ALIGN\n");
        // vl_cmd = -TURN_SPEED4;  vr_cmd=TURN_SPEED4;
        // }
        // Alightcount = 0;
      // } 
      else {
        printf("Forward\n");
        Alightcount++;
        vl_cmd = vr_cmd = FORWARD_SPEED;
        // vl_cmd = 4;  vr_cmd=6;
      }

      // 5) Actuate & log
      wb_motor_set_velocity(left_motor,  vl_cmd);
      wb_motor_set_velocity(right_motor, vr_cmd);
      log_step(vl_cmd, vr_cmd);

       // Log and print in console
    log_step(vl_cmd, vr_cmd);
    int idx = log_count - 1;
    printf("LOG[%d] x=%.3f y=%.3f theta=%.3f vl=%.2f vr=%.2f\n",
           idx,
           log_data[idx].x,
           log_data[idx].y,
           log_data[idx].theta,
           log_data[idx].vl,
           log_data[idx].vr);
  

    } else {
      // Replay Lap2
      if (replay_idx < log_count) {
        wb_motor_set_velocity(left_motor,  log_data[replay_idx].vl);
        wb_motor_set_velocity(right_motor, log_data[replay_idx].vr);
        replay_idx++;
      } else {
        wb_motor_set_velocity(left_motor,  0.0);
        wb_motor_set_velocity(right_motor, 0.0);
        break;
      }
    }
  }

#if LAP == 1
  write_log();
#endif

  wb_robot_cleanup();
  return 0;
}