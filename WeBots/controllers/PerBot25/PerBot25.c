#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/camera.h>
#include <webots/position_sensor.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TIME_STEP        32
#define MAX_LOG_ENTRIES 20000
#define WHEEL_RADIUS     0.0205    // meters
#define AXLE_LENGTH      0.052     // meters

// “Forward” speed in Lap 1 (constant)
#define FORWARD_SPEED    6.0       // rad/s

// Four discrete turn speeds (you can tune these if you like)
#define TURN_SPEED1      5.0       // rad/s
#define TURN_SPEED2      4.0       // rad/s
#define TURN_SPEED3      3.0       // rad/s
#define TURN_SPEED4      2.0       // rad/s

// Lap mode: set to 1 → “learn & log”
//           set to 2 → “load lap1_log.txt & replay”
//           set to 3 → “smooth lap1_log.txt → lap1_log_smoothed.txt & replay smoothed”
#define LAP 1

typedef struct {
  double x, y, theta;   // pose at that timestep (odometry)
  double vl, vr;        // commanded left/right wheel speeds (rad/s)
} LogEntry;

static LogEntry log_data[MAX_LOG_ENTRIES];
static int      log_count = 0;

// “Ground‐truth” odometry pose & last‐encoder values
static double pose_x = 0.0, pose_y = 0.0, pose_theta = 0.0;
static double last_left = 0.0, last_right = 0.0;

// Device tags
static WbDeviceTag left_motor, right_motor;
static WbDeviceTag left_encoder, right_encoder;
static WbDeviceTag cam;


//—————————————————————————————————————————————————————————————
// Fetch Webots basic timestep exactly once
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

//—————————————————————————————————————————————————————————————
// Differential‐drive odometry update (every Lap 1 timestep)
static void update_pose() {
  double l = wb_position_sensor_get_value(left_encoder);
  double r = wb_position_sensor_get_value(right_encoder);
  double dl = (l - last_left)  * WHEEL_RADIUS;  // meters traveled by left wheel
  double dr = (r - last_right) * WHEEL_RADIUS;  // meters traveled by right wheel
  double dc = 0.5 * (dl + dr);                   // forward distance
  double dtheta = (dr - dl) / AXLE_LENGTH;       // change in heading

  // “Midpoint” integration for smoothness
  pose_x     += dc * cos(pose_theta + 0.5 * dtheta);
  pose_y     += dc * sin(pose_theta + 0.5 * dtheta);
  pose_theta += dtheta;

  last_left  = l;
  last_right = r;
}

//—————————————————————————————————————————————————————————————
// Log one timestep: store (x,y,θ) + (vl,vr)
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

//—————————————————————————————————————————————————————————————
// Write the entire Lap 1 log into “lap1_log.txt”
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

//—————————————————————————————————————————————————————————————
// “Post‐process” (smooth) function.
// Reads “input_filename” (the just‐written lap1_log.txt),
// looks for each index i where (vl == +3.0 && vr == −3.0), and
// does a linear ramp over SMOOTH_WINDOW entries before/after that index.
// Then writes the result to “output_filename”. 
// You can tweak SMOOTH_WINDOW or the target speeds as needed.
static void smooth_logged_velocities(const char *input_filename, const char *output_filename) {
  LogEntry buffer[MAX_LOG_ENTRIES];
  int count = 0;

  // 1) Read all entries from input_filename into buffer[]
  FILE *f_in = fopen(input_filename, "r");
  if (!f_in) {
    printf("Error: cannot open %s for smoothing\n", input_filename);
    return;
  }
  while (count < MAX_LOG_ENTRIES) {
    double x, y, th, vl, vr;
    int scanned = fscanf(f_in, "%lf %lf %lf %lf %lf\n", &x, &y, &th, &vl, &vr);
    if (scanned != 5) break;
    buffer[count].x     = x;
    buffer[count].y     = y;
    buffer[count].theta = th;
    buffer[count].vl    = vl;
    buffer[count].vr    = vr;
    count++;
  }
  fclose(f_in);

  // 2) For each “sharp turn” where (vl == +3.0 and vr == −3.0), ramp velocities
  const int SMOOTH_WINDOW_ENTRIES = 5;      // how many entries before/after to smooth
  const double BASE_FORWARD = FORWARD_SPEED; // typically 6.0
  const double TURN_VL =  3.0;
  const double TURN_VR = -3.0;

  for (int i = 0; i < count; i++) {
    if (buffer[i].vl == TURN_VL && buffer[i].vr == TURN_VR) {
      // Found a “turn” at index i. Smooth over [i - W .. i + W].
      int start = (i - SMOOTH_WINDOW_ENTRIES < 0)        ? 0          : i - SMOOTH_WINDOW_ENTRIES;
      int mid   = i;
      int end   = (i + SMOOTH_WINDOW_ENTRIES >= count)   ? (count - 1) : i + SMOOTH_WINDOW_ENTRIES;

      // Ramp‐in: j = start .. mid
      int rampInLength = mid - start;
      if (rampInLength > 0) {
        for (int j = start; j < mid; j++) {
          // t ∈ [0,1]: 0 at j=start → 1 at j=mid
          double t = (double)(j - start) / (double)(rampInLength);
          buffer[j].vl = BASE_FORWARD + t * (TURN_VL - BASE_FORWARD);
          buffer[j].vr = BASE_FORWARD + t * (TURN_VR - BASE_FORWARD);
        }
      }
      // Ensure the midpoint is exactly the turn speeds
      buffer[mid].vl = TURN_VL;
      buffer[mid].vr = TURN_VR;

      // Ramp‐out: j = mid .. end
      int rampOutLength = end - mid;
      if (rampOutLength > 0) {
        for (int j = mid; j <= end; j++) {
          double t = (double)(j - mid) / (double)(rampOutLength);
          buffer[j].vl = TURN_VL + t * (BASE_FORWARD - TURN_VL);
          buffer[j].vr = TURN_VR + t * (BASE_FORWARD - TURN_VR);
        }
      }

      // Skip ahead so we don’t re‐smooth overlapping windows
      i = end;
    }
  }

  // 3) Write the smoothed buffer[] back into output_filename
  FILE *f_out = fopen(output_filename, "w");
  if (!f_out) {
    printf("Error: cannot open %s for writing smoothed data\n", output_filename);
    return;
  }
  for (int i = 0; i < count; i++) {
    fprintf(f_out,
            "%.6f %.6f %.6f   %.6f %.6f\n",
            buffer[i].x,
            buffer[i].y,
            buffer[i].theta,
            buffer[i].vl,
            buffer[i].vr);
  }
  fclose(f_out);

  printf("Smoothing complete: processed %d entries from %s → %s\n",
         count, input_filename, output_filename);
}

//—————————————————————————————————————————————————————————————
// Main entry point
int main() {
  wb_robot_init();

  //--------------------------------------
  // 1) Initialize devices
  //--------------------------------------
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

  // Warm‐up steps so sensors return valid readings
  for (int i = 0; i < 10; i++)
    step_sim();

  // Zero the odometry
  last_left  = wb_position_sensor_get_value(left_encoder);
  last_right = wb_position_sensor_get_value(right_encoder);
  pose_x = pose_y = pose_theta = 0.0;
  log_count = 0;  // in case this is re‐run in the same session

#if (LAP == 2) || (LAP == 3)
  //--------------------------------------
  // 2) LAP 2 or LAP 3 → Load an existing log
  //--------------------------------------
  // If LAP 3, first smooth “lap1_log.txt” → “lap1_log_smoothed.txt”, then load
  if (LAP == 3) {
    // Wait a fraction of a second so the file system is flushed (if needed).
    // In practice, lap1_log.txt already exists from a previous LAP 1 run.
    // Now produce lap1_log_smoothed.txt:
    smooth_logged_velocities("lap1_log.txt", "lap1_log_smoothed.txt");
  }

  // Decide which file to load:
  const char *to_load = (LAP == 2 ? "lap1_log.txt" : "lap1_log_smoothed.txt");
  FILE *f = fopen(to_load, "r");
  if (!f) {
    printf("Error: cannot open %s for replay. Did you run Lap 1 first?\n", to_load);
    fflush(stdout);
    wb_robot_cleanup();
    return 1;
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

  // In LAP 2/3 we do not call update_pose() anymore (we're purely open‐loop replay).
#endif

  int replay_idx = 0;   // for LAP 2 & 3
  int alignCount = 0;   // reused in LAP 1

  //--------------------------------------
  // 3) Main control loop
  //--------------------------------------
  while (wb_robot_step(get_time_step()) != -1) {

    if (LAP == 1) {
      //---------------------- LAP 1: LEARN & LOG ----------------------
      //  (a) Update odometry
      update_pose();
    // … after update_pose() …

    // (b) collect bins from camera
    const unsigned char *img = wb_camera_get_image(cam);
    int width  = wb_camera_get_width(cam);
    int height = wb_camera_get_height(cam);
    
    // inside the “LAP == 1” block, after reading camera:

    // 1) compute rawError ∈ [−blackCount … +blackCount]:
    double rawError = 0.0;
    int blackCount = 0;
    int red = 0;
    int threshold = 80;  // “black” threshold
    int midX = 52;
    int y0 = height - 25;
    for (int y = y0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        int r = wb_camera_image_get_red(img,   width, x, y);
        int g = wb_camera_image_get_green(img, width, x, y);
        int b = wb_camera_image_get_blue(img,  width, x, y);
        // RED line → STOP
        if (r > 200 && g < 30 && b < 30) {
          if (x > width/2 - 10 && x < width/2 + 10 && y > 65)
            red++;
        }
        else if (r < threshold && g < threshold && b < threshold) {
          int sign = (x < midX) ? -1.0 : +1.0;
          rawError += sign;
          blackCount++;
        }
        
        
      }
    }
    double vl_cmd = 0.0, vr_cmd = 0.0;
    if (red > 0) {
      // STOP immediately if red line
      printf("RED detected → STOP\n");
      vl_cmd = vr_cmd = 0.0;
    } else{
    // Normalize to [−1 … +1]:
    double error = (blackCount > 0) ? (rawError / (double)blackCount) : 0.0;

    // 2) apply a proportional gain (you could add I or D later)
    const double Kp = 8.0;  // start here; if too timid, raise to 5 or 6; if too twitchy, lower to 2 or 3.
    double turn = Kp * error;

    // 3) compute wheel speeds
    double base = FORWARD_SPEED;
    vl_cmd = base - turn;
    vr_cmd = base + turn;

    // clamp them so they never exceed ±base
    if (vl_cmd >  base) vl_cmd =  base;
    if (vl_cmd < -base) vl_cmd = -base;
    if (vr_cmd >  base) vr_cmd =  base;
    if (vr_cmd < -base) vr_cmd = -base;
    }

    // (d) Command motors and log
    wb_motor_set_velocity(left_motor,  vl_cmd);
    wb_motor_set_velocity(right_motor, vr_cmd);
    log_step(vl_cmd, vr_cmd);

    // (e) Debugging print
    int idx = log_count - 1;
    printf("LOG[%d]  x=%.3f  y=%.3f  θ=%.3f   vl=%.2f   vr=%.2f\n",
           idx,
           log_data[idx].x,
           log_data[idx].y,
           log_data[idx].theta,
           log_data[idx].vl,
           log_data[idx].vr);
    }
    else {
      //---------------------- LAP 2 or LAP 3: REPLAY ----------------------
      // Both modes 2 & 3 simply replay their preloaded log_data[].
      // In mode 3, log_data[] was loaded from lap1_log_smoothed.txt.

      if (replay_idx < log_count) {
        // Re‐apply the recorded wheel speeds open‐loop
        wb_motor_set_velocity(left_motor,  log_data[replay_idx].vl);
        wb_motor_set_velocity(right_motor, log_data[replay_idx].vr);
        replay_idx++;
      } else {
        // Once done, stop the robot
        wb_motor_set_velocity(left_motor,  0.0);
        wb_motor_set_velocity(right_motor, 0.0);
        break;
      }
    }
  }

  //--------------------------------------
  // 4) End of simulation / cleanup
  //--------------------------------------
#if LAP == 1
  // When Lap 1 finishes, write out the raw log, so next lap can use it
  write_log();
  printf("Lap 1 complete.  Logged %d timesteps → lap1_log.txt\n", log_count);
  fflush(stdout);
#endif

  wb_robot_cleanup();
  return 0;
}
