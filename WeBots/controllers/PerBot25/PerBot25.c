// PerBot25.c
// e-puck: Stage 1 Simulation – Lap1 learning + Lap2 A* path optimization

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
#define FORWARD_SPEED    6.0       // rad/s
#define TURN_SPEED       5.0       // rad/s

// Lap mode: 1 = learning & logging, 2 = optimized replay
#define LAP 2

// Grid parameters (2m x 2m arena, 5cm cells)
#define CELL_SIZE    0.05
#define GRID_X       40
#define GRID_Y       40
#define ORIGIN_X    -1.0
#define ORIGIN_Y    -1.0

typedef struct { double x, y, theta, vl, vr; } LogEntry;
static LogEntry log_data[MAX_LOG_ENTRIES];
static int      obs_flag[MAX_LOG_ENTRIES];
static int      log_count = 0;

typedef struct { int x, y; } Cell;
static int  grid_map[GRID_X][GRID_Y];
static Cell path[GRID_X * GRID_Y];
static int  path_len = 0;

// Pose estimation
static double pose_x = 0.0, pose_y = 0.0, pose_theta = 0.0;
static double last_left = 0.0, last_right = 0.0;

// Device tags
static WbDeviceTag left_motor, right_motor;
static WbDeviceTag left_enc, right_enc, cam;

// Helpers
static int get_ts() {
  static int ts = -1;
  if (ts < 0) ts = wb_robot_get_basic_time_step();
  return ts;
}
static void step_sim() {
  if (wb_robot_step(get_ts()) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

// Update pose from encoders
static void update_pose() {
  double l = wb_position_sensor_get_value(left_enc);
  double r = wb_position_sensor_get_value(right_enc);
  double dl = (l - last_left) * WHEEL_RADIUS;
  double dr = (r - last_right) * WHEEL_RADIUS;
  double dc = 0.5 * (dl + dr);
  double dtheta = (dr - dl) / AXLE_LENGTH;
  pose_x += dc * cos(pose_theta + 0.5 * dtheta);
  pose_y += dc * sin(pose_theta + 0.5 * dtheta);
  pose_theta += dtheta;
  last_left = l;
  last_right = r;
}

// Log current step
static void log_step(double vl, double vr, int flag) {
  if (log_count < MAX_LOG_ENTRIES) {
    log_data[log_count].x     = pose_x;
    log_data[log_count].y     = pose_y;
    log_data[log_count].theta = pose_theta;
    log_data[log_count].vl    = vl;
    log_data[log_count].vr    = vr;
    obs_flag[log_count]       = flag;
    log_count++;
  }
}
static void write_log() {
  FILE *f = fopen("lap1_log.txt", "w");
  if (!f) return;
  for (int i = 0; i < log_count; ++i)
    fprintf(f, "%.4f %.4f %.4f %.4f %.4f %d\n",
            log_data[i].x, log_data[i].y, log_data[i].theta,
            log_data[i].vl, log_data[i].vr, obs_flag[i]);
  fclose(f);
}

// Convert world coords to grid cell
static Cell pose_to_cell(double x, double y) {
  int gx = (int)floor((x - ORIGIN_X) / CELL_SIZE);
  int gy = (int)floor((y - ORIGIN_Y) / CELL_SIZE);
  if (gx < 0) gx = 0;
  if (gx >= GRID_X) gx = GRID_X - 1;
  if (gy < 0) gy = 0;
  if (gy >= GRID_Y) gy = GRID_Y - 1;
  return (Cell){gx, gy};
}

// Heuristic for A*
static int heuristic(Cell a, Cell b) {
  return abs(a.x - b.x) + abs(a.y - b.y);
}

// A* search on grid_map
static int astar(Cell start, Cell goal) {
  typedef struct { Cell pos; int g, h, f; Cell parent; } Node;
  static Node open_list[GRID_X * GRID_Y];
  int open_count = 0;
  static int closed[GRID_X][GRID_Y];
  memset(closed, 0, sizeof(closed));
  path_len = 0;

  // init
  open_list[open_count++] = (Node){start, 0, heuristic(start, goal), heuristic(start, goal), start};

  while (open_count > 0) {
    int idx = 0;
    for (int i = 1; i < open_count; ++i)
      if (open_list[i].f < open_list[idx].f) idx = i;
    Node cur = open_list[idx];
    open_list[idx] = open_list[--open_count];

    if (cur.pos.x == goal.x && cur.pos.y == goal.y) {
      // reconstruct path
      Cell p = cur.pos;
      while (!(p.x == start.x && p.y == start.y)) {
        path[path_len++] = p;
        // use parent
        p = cur.parent;
      }
      path[path_len++] = start;
      for (int i = 0; i < path_len/2; ++i) {
        Cell t = path[i]; path[i] = path[path_len-1-i]; path[path_len-1-i] = t;
      }
      return 1;
    }

    closed[cur.pos.x][cur.pos.y] = 1;
    // neighbors
    const int dx[4] = {1,-1,0,0};
    const int dy[4] = {0,0,1,-1};
    for (int k = 0; k < 4; ++k) {
      Cell nb = {cur.pos.x + dx[k], cur.pos.y + dy[k]};
      if (nb.x < 0 || nb.y < 0 || nb.x >= GRID_X || nb.y >= GRID_Y) continue;
      if (grid_map[nb.x][nb.y] || closed[nb.x][nb.y]) continue;
      int g2 = cur.g + 1;
      int h2 = heuristic(nb, goal);
      open_list[open_count++] = (Node){nb, g2, h2, g2 + h2, cur.pos};
    }
  }
  return 0;
}

// Motion primitives
static void turn_to(double target) {
  double diff = atan2(sin(target - pose_theta), cos(target - pose_theta));
  double v = (diff > 0 ? TURN_SPEED : -TURN_SPEED);
  wb_motor_set_velocity(left_motor, -v);
  wb_motor_set_velocity(right_motor, v);
  while (fabs(diff) > 0.05) {
    update_pose(); step_sim();
    diff = atan2(sin(target - pose_theta), cos(target - pose_theta));
  }
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor,0);
}
static void move_dist(double dist) {
  int steps = (int)(fabs(dist) / (WHEEL_RADIUS * FORWARD_SPEED) * 1000.0 / TIME_STEP);
  wb_motor_set_velocity(left_motor, FORWARD_SPEED);
  wb_motor_set_velocity(right_motor, FORWARD_SPEED);
  for (int i = 0; i < steps; ++i) { update_pose(); step_sim(); }
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
}

int main() {
  wb_robot_init();
  // init devices
  left_motor  = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor,0);

  left_enc  = wb_robot_get_device("left wheel sensor");
  right_enc = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_enc, get_ts());
  wb_position_sensor_enable(right_enc, get_ts());
  cam = wb_robot_get_device("camera");
  wb_camera_enable(cam, get_ts());

  for (int i = 0; i < 10; ++i) step_sim();
  last_left  = wb_position_sensor_get_value(left_enc);
  last_right = wb_position_sensor_get_value(right_enc);

  if (LAP == 1) {
    // Lap1: learning & logging
    while (wb_robot_step(get_ts()) != -1) {
      update_pose();
      // color detection region
      const unsigned char *img = wb_camera_get_image(cam);
      int w = wb_camera_get_width(cam), h = wb_camera_get_height(cam);
      int red=0, bl=0, br=0, whl=0, whr=0;
      int x0=w/2-50, x1=w/2+50, y0=h-30;
      for (int y=y0; y<h; ++y) {
        for (int x=x0; x<x1; ++x) {
          int r = wb_camera_image_get_red(img,w,x,y);
          int g = wb_camera_image_get_green(img,w,x,y);
          int b = wb_camera_image_get_blue(img,w,x,y);
          if (r>200 && g<30 && b<30) {
            if (x>w/2-10 && x<w/2+5) red++;
          } else if (r<50 && g<50 && b<50) {
            if (x<w/2) bl++; else br++;
          } else if (r>150 && g>150 && b>150) {
            if (x<w/2) whl++; else whr++;
          }
        }
      }
      int flag = (br > whr) || (bl > whl);
      double vl = 0, vr = 0;
      if (red>0) {
        vl = vr = 0;
      } else if (br>whr) {
        vl=-TURN_SPEED; vr=TURN_SPEED;
      } else if (bl>whl) {
        vl=TURN_SPEED; vr=-TURN_SPEED;
      } else {
        vl=vr=FORWARD_SPEED;
      }
      wb_motor_set_velocity(left_motor, vl);
      wb_motor_set_velocity(right_motor, vr);
      log_step(vl, vr, flag);
    }
    wb_motor_set_velocity(left_motor,0); wb_motor_set_velocity(right_motor,0);
    write_log();
  }

  else {
    // Lap2: build grid, A*, execute optimized path
    // 1) load log into grid
    memset(grid_map,0,sizeof(grid_map));
    for (int i = 0; i < log_count; ++i) {
      Cell c = pose_to_cell(log_data[i].x, log_data[i].y);
      if (obs_flag[i]) grid_map[c.x][c.y]=1;
    }
    Cell start = pose_to_cell(log_data[0].x, log_data[0].y);
    Cell goal  = pose_to_cell(log_data[log_count-1].x, log_data[log_count-1].y);
    if (!astar(start, goal)) {
      printf("A*: no path found!\n");
      wb_robot_cleanup();
      return 1;
    }
    // 2) follow waypoints
    for (int i = 1; i < path_len; ++i) {
      double wx = ORIGIN_X + (path[i].x + 0.5)*CELL_SIZE;
      double wy = ORIGIN_Y + (path[i].y + 0.5)*CELL_SIZE;
      update_pose();
      double ang = atan2(wy-pose_y, wx-pose_x);
      turn_to(ang);
      double dist = hypot(wx-pose_x, wy-pose_y);
      move_dist(dist);
    }
  }

  wb_robot_cleanup();
  return 0;
}
