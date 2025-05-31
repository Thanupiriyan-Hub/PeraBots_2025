#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/camera.h>
#include <webots/position_sensor.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP        32
#define MAX_LOG_ENTRIES 20000
#define WHEEL_RADIUS     0.0205    // [m]
#define AXLE_LENGTH      0.052     // [m]

// “Forward” speed in Lap 1 (constant)
#define FORWARD_SPEED    6.0       // [rad/s]

// Four discrete turn speeds (you can tune these if you like)
#define TURN_SPEED1      5.0       // [rad/s]
#define TURN_SPEED2      4.0       // [rad/s]
#define TURN_SPEED3      3.0       // [rad/s]
#define TURN_SPEED4      2.0       // [rad/s]

// Lap mode: set to 1 to “learn & log,” set to 2 to “load log & replay”.
// (You must recompile after changing this #define.)
#define LAP 2

typedef struct {
  double x, y, theta;   // (x,y,θ) at that timestep
  double vl, vr;        // left‐wheel & right‐wheel speeds (rad/s)
} LogEntry;

static LogEntry log_data[MAX_LOG_ENTRIES];
static int log_count = 0;

// “Ground‐truth” pose & last‐encoder values
static double pose_x = 0.0, pose_y = 0.0, pose_theta = 0.0;
static double last_left = 0.0, last_right = 0.0;

// Device tags
static WbDeviceTag left_motor, right_motor;
static WbDeviceTag left_encoder, right_encoder;
static WbDeviceTag cam;

//-----------------------------------------------------------------------------
// Fetch Webots basic timestep once
static int get_time_step() {
  static int ts = -1;
  if (ts < 0)
    ts = wb_robot_get_basic_time_step();
  return ts;
}

// Wrapper around wb_robot_step(...)
static void step_sim() {
  if (wb_robot_step(get_time_step()) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

// Differential‐drive odometry update
static void update_pose() {
  double l = wb_position_sensor_get_value(left_encoder);
  double r = wb_position_sensor_get_value(right_encoder);
  double dl = (l - last_left)  * WHEEL_RADIUS;
  double dr = (r - last_right) * WHEEL_RADIUS;
  double dc = 0.5 * (dl + dr);
  double dtheta = (dr - dl) / AXLE_LENGTH;
  // “Midpoint” rotation for smoother integration
  pose_x     += dc * cos(pose_theta + 0.5 * dtheta);
  pose_y     += dc * sin(pose_theta + 0.5 * dtheta);
  pose_theta += dtheta;
  last_left  = l;
  last_right = r;
}

// Log a single entry: (current pose, plus commanded vl, vr)
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

// Write the entire Lap 1 log to disk
static void write_log() {
  FILE *f = fopen("lap1_log.txt", "w");
  if (!f) {
    printf("Error: could not open lap1_log.txt for writing.\n");
    return;
  }
  for (int i = 0; i < log_count; i++) {
    fprintf(f,
            "%.6f %.6f %.6f   %.6f %.6f\n", 
            log_data[i].x,
            log_data[i].y,
            log_data[i].theta,
            log_data[i].vl,
            log_data[i].vr);
  }
  fclose(f);
}

//-----------------------------------------------------------------------------
// Main entry point
int main() {
  wb_robot_init();

  //—————— INITIALIZE MOTORS & SENSORS ——————
  left_motor  = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor,  INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor,  0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  left_encoder  = wb_robot_get_device("left wheel sensor");
  right_encoder = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_encoder,  get_time_step());
  wb_position_sensor_enable(right_encoder, get_time_step());

  cam = wb_robot_get_device("camera");
  wb_camera_enable(cam, get_time_step());

  // Warm up for a few steps so sensors have valid readings
  for (int i = 0; i < 10; i++)
    step_sim();

  // Record the initial encoder values so odometry starts “zeroed”
  last_left  = wb_position_sensor_get_value(left_encoder);
  last_right = wb_position_sensor_get_value(right_encoder);

#if LAP == 2
  //—————— LAP 2: LOAD “lap1_log.txt” INTO log_data[] ——————
  FILE *f = fopen("lap1_log.txt", "r");
  if (!f) {
    printf("Error: cannot open lap1_log.txt for replay. Did you set LAP=2 and recompile?\n");
    exit(1);
  }
  log_count = 0;
  while (log_count < MAX_LOG_ENTRIES) {
    double x, y, t, vl, vr;
    int scanned = fscanf(f, "%lf %lf %lf %lf %lf\n", &x, &y, &t, &vl, &vr);
    if (scanned != 5) break;
    log_data[log_count].x     = x;
    log_data[log_count].y     = y;
    log_data[log_count].theta = t;
    log_data[log_count].vl    = vl;
    log_data[log_count].vr    = vr;
    log_count++;
  }
  fclose(f);

  // We do *not* update pose_x, pose_y, pose_theta in Lap 2 (we’re just replaying open‐loop velocities).
  // In a perfect frictionless world, replaying these exact vl/vr at each timestep would retrace the same path.
  // In practice, small differences cause drift. If you want to “execute recorded poses” exactly,
  // you must switch to a closed‐loop pose controller (not shown here).
#endif

  // Replay index for Lap 2
  int replay_idx = 0;

  // A simple “edge‐alignment” counter (used only in Lap 1)
  int alignCount = 0;

  //—————— MAIN CONTROL LOOP ——————
  while (wb_robot_step(get_time_step()) != -1) {

    if (LAP == 1) {
      //---------------- LAP 1: LEARN & LOG ----------------
      // 1) Update odometry from encoders
      update_pose();

      // 2) Grab the camera image and do your color‐based scanning
      //    We have eight “bins” on each side (left/right), plus “red” detection.
      const unsigned char *img = wb_camera_get_image(cam);
      int width  = wb_camera_get_width(cam);
      int height = wb_camera_get_height(cam);

      int red = 0;
      // Left‐side black/white bins: LB4, LB3, LB2, LB1 / LW4, LW3, LW2, LW1
      int LB1 = 0, LB2 = 0, LB3 = 0, LB4 = 0;
      int LW1 = 0, LW2 = 0, LW3 = 0, LW4 = 0;
      // Right‐side black/white bins: RB1, RB2, RB3, RB4 / RW1, RW2, RW3, RW4
      int RB1 = 0, RB2 = 0, RB3 = 0, RB4 = 0;
      int RW1 = 0, RW2 = 0, RW3 = 0, RW4 = 0;

      // Define the scanning window: x ∈ [0, 104), y ∈ [height−30, height)
      // You can tweak these “bins” as you like; this matches your original code structure.
      int x0 = 0;
      int x1 = 104;
      int y0 = height - 30;

      for (int y = y0; y < height; y++) {
        for (int x = x0; x < x1; x++) {
          int r = wb_camera_image_get_red(img, width, x, y);
          int g = wb_camera_image_get_green(img, width, x, y);
          int b = wb_camera_image_get_blue(img, width, x, y);
          // Detect RED (stop line)
          if (r > 200 && g < 30 && b < 30) {
            // Only count if near center‐bottom of image
            if (x > width/2 - 10 && x < width/2 + 10 && y > 65)
              red++;
          }
          // Detect “Black” pixels
          else if (r < 150 && g < 150 && b < 150) {
            if      (x < 13)       LB4++;
            else if (x < 26)       LB3++;
            else if (x < 39)       LB2++;
            else if (x < 52)       LB1++;
            else if (x < 65)       RB1++;
            else if (x < 78)       RB2++;
            else if (x < 91)       RB3++;
            else                   RB4++;
          }
          // Detect “White” pixels
          else if (r > 150 && g > 150 && b > 150) {
            if      (x < 13)       LW4++;
            else if (x < 26)       LW3++;
            else if (x < 39)       LW2++;
            else if (x < 52)       LW1++;
            else if (x < 65)       RW1++;
            else if (x < 78)       RW2++;
            else if (x < 91)       RW3++;
            else                   RW4++;
          }
        }
      }

      // 3) Decision logic: pick vl_cmd, vr_cmd based on bins
      double vl_cmd = 0.0;
      double vr_cmd = 0.0;

      printf("L‐white total = %d\n", LW4 + LW3 + LW2 + LW1);

      if (red > 0) {
        // Stop if we see red
        printf("RED detected → STOP\n");
        vl_cmd = vr_cmd = 0.0;
      }
      else if (RB1 > RW1) {
        // Far‐left obstacle ⇒ turn left fast
        printf("Turn Left (Speed1)\n");
        alignCount = 0;
        vl_cmd = -TURN_SPEED1;
        vr_cmd =  TURN_SPEED1;
      }
      else if (LB1 > LW1) {
        // Far‐right obstacle ⇒ turn right fast
        printf("Turn Right (Speed1)\n");
        alignCount = 0;
        vl_cmd =  TURN_SPEED1;
        vr_cmd = -TURN_SPEED1;
      }
      else if (RB2 > RW2) {
        // Next‐inward bin → turn left at Speed2
        printf("Turn Left (Speed2)\n");
        vl_cmd = -TURN_SPEED2;
        vr_cmd =  TURN_SPEED2;
      }
      else if (LB2 > LW2) {
        // Next‐inward bin → turn right at Speed2
        printf("Turn Right (Speed2)\n");
        vl_cmd =  TURN_SPEED2;
        vr_cmd = -TURN_SPEED2;
      }
      else if (RB3 > RW3) {
        printf("Turn Left (Speed3)\n");
        vl_cmd = -TURN_SPEED3;
        vr_cmd =  TURN_SPEED3;
      }
      else if (LB3 > LW3) {
        printf("Turn Right (Speed3)\n");
        vl_cmd =  TURN_SPEED3;
        vr_cmd = -TURN_SPEED3;
      }
      else if (RB4 > RW4) {
        printf("Turn Left (Speed4)\n");
        vl_cmd = -TURN_SPEED4;
        vr_cmd =  TURN_SPEED4;
      }
      else if (LB4 > LW4) {
        printf("Turn Right (Speed4)\n");
        // *** BUG FIX: This line previously used TURN_SPEED3 by mistake.
        vl_cmd =  TURN_SPEED4;
        vr_cmd = -TURN_SPEED4;
      }
      else if (alignCount > 5) {
        // If we’ve been “straight” for 5 frames, force a slight align
        if ((LW4 + LW3 + LW2 + LW1) > 1450) {
          printf("ALIGN (slow turn)\n");
          vl_cmd = -TURN_SPEED4;
          vr_cmd =  TURN_SPEED4;
        }
        alignCount = 0;
      }
      else {
        // Otherwise, go straight
        printf("Forward\n");
        alignCount++;
        vl_cmd = vr_cmd = FORWARD_SPEED;
      }

      // 4) Actuate motors
      wb_motor_set_velocity(left_motor,  vl_cmd);
      wb_motor_set_velocity(right_motor, vr_cmd);

      // 5) Log this timestep (pose + wheel speeds) ONCE
      log_step(vl_cmd, vr_cmd);

      // Print to console for debugging
      int idx = log_count - 1;
      printf(
        "LOG[%d]  x=%.3f  y=%.3f  θ=%.3f   vl=%.2f   vr=%.2f\n",
        idx,
        log_data[idx].x,
        log_data[idx].y,
        log_data[idx].theta,
        log_data[idx].vl,
        log_data[idx].vr
      );
    }
    else {
      //---------------- LAP 2: REPLAY logged wheel speeds ----------------
      if (replay_idx < log_count) {
        // Set wheel speeds exactly as recorded
        wb_motor_set_velocity(left_motor,  log_data[replay_idx].vl);
        wb_motor_set_velocity(right_motor, log_data[replay_idx].vr);
        replay_idx++;
      } else {
        // Once we’ve replayed all entries, stop
        wb_motor_set_velocity(left_motor,  0.0);
        wb_motor_set_velocity(right_motor, 0.0);
        break;
      }
    }
  }

#if LAP == 1
  // When Lap 1 finishes, dump the log to disk
  write_log();
#endif

  wb_robot_cleanup();
  return 0;
}
