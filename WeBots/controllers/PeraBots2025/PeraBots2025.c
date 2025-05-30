// my_controller.c
// e-puck: color navigation + pose estimation + distance sensors (ps1, ps6) + Lap1 logging + Lap2 replay

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
#define TURN_SPEED       5.0       // rad/s

// Use only two distance sensors: ps1 (right) and ps6 (left)
#define DISTANCE_SENSORS_NUMBER 2
static WbDeviceTag distance_sensors[DISTANCE_SENSORS_NUMBER];
static double    distance_sensors_raw[DISTANCE_SENSORS_NUMBER];
static const char *distance_sensors_names[DISTANCE_SENSORS_NUMBER] = { "ps1", "ps6" };

// Convert raw IR reading to approximate centimeters (empirical)
static double raw_to_cm(double raw) {
  // tweak 1234 to calibrate; avoids division by zero
  return (raw < 5.0) ? 0.0 : 1234.0 / (raw + 1.0);
}

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

// Read IR sensors into raw array
static void read_distance_sensors() {
  for (int i = 0; i < DISTANCE_SENSORS_NUMBER; ++i)
    distance_sensors_raw[i] = wb_distance_sensor_get_value(distance_sensors[i]);
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

  // Distance sensors ps1, ps6
  for (int i = 0; i < DISTANCE_SENSORS_NUMBER; ++i) {
    distance_sensors[i] = wb_robot_get_device(distance_sensors_names[i]);
    wb_distance_sensor_enable(distance_sensors[i], get_time_step());
  }

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

  // Main control loop
  while (wb_robot_step(get_time_step()) != -1) {
    if (LAP == 1) {
      // 1) Update pose
      update_pose();

      // 2) Read & print distance sensors
      read_distance_sensors();
      for (int i = 0; i < DISTANCE_SENSORS_NUMBER; ++i) {
        double raw = distance_sensors_raw[i];
        double cm  = raw_to_cm(raw);
        printf("Sensor %s: raw=%.1f, approx=%.2f cm\n",
               distance_sensors_names[i], raw, cm);
      }

      // 3) Color-based region scan (unchanged)
      const unsigned char *img = wb_camera_get_image(cam);
      int width  = wb_camera_get_width(cam);
      int height = wb_camera_get_height(cam);
      int red = 0, black_left=0, white_left=0, black_right=0, white_right=0;
      int white_left_Edge=0, white_right_Edge=0;
      int x0 = width/2 - 50, x1 = width/2 + 50, y0 = height - 30;
      for (int y = y0; y < height; ++y) {
        for (int x = x0; x < x1; ++x) {
          int r = wb_camera_image_get_red(img,width,x,y);
          int g = wb_camera_image_get_green(img,width,x,y);
          int b = wb_camera_image_get_blue(img,width,x,y);
          if (r>200&&g<30&&b<30) {
            if (x>width/2-10 && x<width/2+5) red++;
          } else if (r<50&&g<50&&b<50) {
            if (x<width/2) black_left++; else black_right++;
          } else if (r>150&&g>150&&b>150) {
            if      (x<20)       white_left_Edge++;
            else if (x<52)       white_left++;
            else if (x<84)       white_right++;
            else                 white_right_Edge++;
          }
        }
      }

      // 4) Decision
      double vl_cmd=0.0, vr_cmd=0.0;
      printf("white_right_Edge=%d\n", white_right_Edge);
      if (red>0) {
        printf("RED detected — stopping\n");
      } else if (black_right>white_right) {
        printf("Obstacle right — turn left\n");
        vl_cmd = -TURN_SPEED; vr_cmd= TURN_SPEED;
      } else if (black_left>white_left) {
        printf("Obstacle left — turn right\n");
        vl_cmd = TURN_SPEED;  vr_cmd=-TURN_SPEED;
      } else if (white_left_Edge>500) {
        printf("Align left\n");
        vl_cmd = -3.0; vr_cmd = 3.0;
      } else {
        printf("Forward\n");
        vl_cmd = vr_cmd = FORWARD_SPEED;
      }

      // 5) Actuate & log
      wb_motor_set_velocity(left_motor,  vl_cmd);
      wb_motor_set_velocity(right_motor, vr_cmd);
      log_step(vl_cmd, vr_cmd);

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
