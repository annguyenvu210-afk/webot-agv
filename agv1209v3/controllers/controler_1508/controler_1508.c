/*
 * Simple line-following controller for the two-wheel AGV demo.
 *
 * The camera is used as a 1-D magnetic/line sensor. When more than one
 * dark segment is visible (at a branch), the segment nearest to the one
 * selected in the previous frame is kept. This makes the AGV continue on
 * its current path instead of steering toward the average of two branches.
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include <webots/camera.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 10
#define DT (TIME_STEP / 1000.0)

#define LINE_THRESHOLD 100
#define SENSOR_WIDTH_M 0.15

#define WHEEL_RADIUS_M 0.08
#define HALF_TRACK_M 0.20
#define MAX_WHEEL_SPEED 15.625 /* 1.25 m/s / 0.08 m */

#define CRUISE_SPEED_MPS 0.65
#define MIN_CURVE_SPEED_MPS 0.28
#define SEARCH_SPEED_MPS 0.15

#define STEERING_KP 9.0
#define MAX_ANGULAR_SPEED 1.00
#define LINE_ERROR_DEADBAND_M 0.0025

#define ERROR_FILTER_ALPHA 0.15
#define ANGULAR_COMMAND_ALPHA 0.12
#define LINEAR_COMMAND_ALPHA 0.08

#define WHEEL_ACCEL_LIMIT 10.0 /* rad/s^2 */
#define STARTUP_DELAY_STEPS 50
#define LOST_LINE_SEARCH_STEPS 60
#define LOST_LINE_MIN_ERROR_M 0.004

static double clamp_value(double value, double minimum, double maximum) {
  if (value < minimum)
    return minimum;
  if (value > maximum)
    return maximum;
  return value;
}

static double move_towards(double current, double target, double max_delta) {
  const double delta = target - current;
  if (delta > max_delta)
    return current + max_delta;
  if (delta < -max_delta)
    return current - max_delta;
  return target;
}

/*
 * Finds all contiguous dark segments in the camera row and returns one.
 * At a branch, selecting the segment nearest to the previous selection is
 * more stable than averaging all dark pixels together.
 */
static bool find_line_center(const unsigned char *image, int width, int height,
                             bool have_previous, double previous_center,
                             double *selected_center) {
  const int y = height / 2;
  const double image_center = (width - 1) / 2.0;
  double best_score = INFINITY;
  double best_center = image_center;
  int segment_pixel_sum = 0;
  int segment_pixel_count = 0;

  for (int x = 0; x <= width; ++x) {
    bool is_dark = false;
    if (x < width) {
      const int gray = wb_camera_image_get_gray(image, width, x, y);
      is_dark = gray <= LINE_THRESHOLD;
    }

    if (is_dark) {
      segment_pixel_sum += x;
      ++segment_pixel_count;
      continue;
    }

    if (segment_pixel_count > 0) {
      const double center = (double)segment_pixel_sum / segment_pixel_count;
      const double reference = have_previous ? previous_center : image_center;
      const double score = fabs(center - reference);

      if (score < best_score) {
        best_score = score;
        best_center = center;
      }

      segment_pixel_sum = 0;
      segment_pixel_count = 0;
    }
  }

  if (!isfinite(best_score))
    return false;

  *selected_center = best_center;
  return true;
}

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag line_sensor = wb_robot_get_device("mgs1600");
  WbDeviceTag left_motor = wb_robot_get_device("Lmotor");
  WbDeviceTag right_motor = wb_robot_get_device("Rmotor");

  wb_camera_enable(line_sensor, TIME_STEP);
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  const int image_width = wb_camera_get_width(line_sensor);
  const int image_height = wb_camera_get_height(line_sensor);
  const double meters_per_pixel = SENSOR_WIDTH_M / (image_width - 1);

  bool have_previous_segment = false;
  double previous_segment_center = (image_width - 1) / 2.0;
  double last_valid_error = 0.0;
  double filtered_error = 0.0;
  double angular_command = 0.0;
  double linear_command = 0.0;
  double left_command = 0.0;
  double right_command = 0.0;
  int lost_line_steps = 0;
  int step_count = 0;

  while (wb_robot_step(TIME_STEP) != -1) {
    const unsigned char *image = wb_camera_get_image(line_sensor);
    double line_center = previous_segment_center;
    const bool line_found = image != NULL &&
                            find_line_center(image, image_width, image_height,
                                             have_previous_segment,
                                             previous_segment_center,
                                             &line_center);

    double linear_speed = 0.0;
    double angular_speed = 0.0;

    if (line_found) {
      const double image_center = (image_width - 1) / 2.0;
      const double raw_error = (line_center - image_center) * meters_per_pixel;

      have_previous_segment = true;
      previous_segment_center = line_center;
      last_valid_error = raw_error;
      lost_line_steps = 0;

      filtered_error += ERROR_FILTER_ALPHA * (raw_error - filtered_error);

      double steering_error = filtered_error;
      if (fabs(steering_error) < LINE_ERROR_DEADBAND_M)
        steering_error = 0.0;

      /* A proportional controller is sufficient for this low-speed demo. */
      angular_speed = STEERING_KP * steering_error;
      angular_speed = clamp_value(angular_speed, -MAX_ANGULAR_SPEED,
                                  MAX_ANGULAR_SPEED);

      /* Slow down progressively in a curve, but never stop abruptly. */
      linear_speed = CRUISE_SPEED_MPS / (1.0 + 1.5 * fabs(angular_speed));
      linear_speed = fmax(linear_speed, MIN_CURVE_SPEED_MPS);
    } else {
      ++lost_line_steps;

      /*
       * Keep turning slowly in the last known direction for a short time.
       * If the line is not found again, stop instead of exiting the program.
       */
      if (lost_line_steps <= LOST_LINE_SEARCH_STEPS &&
          fabs(last_valid_error) >= LOST_LINE_MIN_ERROR_M) {
        linear_speed = SEARCH_SPEED_MPS;
        angular_speed = copysign(0.60, last_valid_error);
      }
    }

    /* Filter both commands so camera pixel changes cannot shake the motors. */
    angular_command +=
        ANGULAR_COMMAND_ALPHA * (angular_speed - angular_command);
    linear_command += LINEAR_COMMAND_ALPHA * (linear_speed - linear_command);

    double left_target =
        (linear_command + angular_command * HALF_TRACK_M) / WHEEL_RADIUS_M;
    double right_target =
        (linear_command - angular_command * HALF_TRACK_M) / WHEEL_RADIUS_M;

    left_target = clamp_value(left_target, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
    right_target = clamp_value(right_target, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);

    if (step_count < STARTUP_DELAY_STEPS) {
      left_target = 0.0;
      right_target = 0.0;
    }

    const double max_wheel_delta = WHEEL_ACCEL_LIMIT * DT;
    left_command = move_towards(left_command, left_target, max_wheel_delta);
    right_command = move_towards(right_command, right_target, max_wheel_delta);

    wb_motor_set_velocity(left_motor, left_command);
    wb_motor_set_velocity(right_motor, right_command);

    if (step_count % 10 == 0) {
      const double line_error_mm = last_valid_error * 1000.0;
      printf("line=%s error=%7.2f mm v=%5.2f m/s w=%5.2f rad/s "
             "left=%6.2f right=%6.2f\n",
             line_found ? "found" : "lost ", line_error_mm, linear_command,
             angular_command, left_command, right_command);
    }

    ++step_count;
  }

  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  wb_robot_cleanup();
  return 0;
}
