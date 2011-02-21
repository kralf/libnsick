/***************************************************************************
 *   Copyright (C) 2008 by Ralf Kaestner                                   *
 *   ralf.kaestner@gmail.com                                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include "nsick.h"

const char* nsick_errors[] = {
  "success",
  "error opening EPOS",
  "error closing EPOS",
  "error homing EPOS",
  "error traveling EPOS profile",
  "wait operation timed out",
};

#define NSICK_PARAMETER_START_POSITION       "start-pos"
#define NSICK_PARAMETER_END_POSITION         "end-pos"
#define NSICK_PARAMETER_MAX_VELOCITY         "max-vel"
#define NSICK_PARAMETER_MAX_ACCELERATION     "max-acc"
#define NSICK_PARAMETER_SENSOR_X             "sensor-x"
#define NSICK_PARAMETER_SENSOR_Y             "sensor-y"
#define NSICK_PARAMETER_SENSOR_Z             "sensor-z"
#define NSICK_PARAMETER_SENSOR_YAW           "sensor-yaw"
#define NSICK_PARAMETER_SENSOR_PITCH         "sensor-pitch"
#define NSICK_PARAMETER_SENSOR_ROLL          "sensor-roll"

param_t nsick_default_params[] = {
  {NSICK_PARAMETER_START_POSITION, "-45.0"},
  {NSICK_PARAMETER_END_POSITION, "45.0"},
  {NSICK_PARAMETER_MAX_VELOCITY, "45.0"},
  {NSICK_PARAMETER_MAX_ACCELERATION, "45.0"},

  {NSICK_PARAMETER_SENSOR_X, "0.037"},
  {NSICK_PARAMETER_SENSOR_Y, "0.0"},
  {NSICK_PARAMETER_SENSOR_Z, "-0.032"},
  {NSICK_PARAMETER_SENSOR_YAW, "0.0"},
  {NSICK_PARAMETER_SENSOR_PITCH, "0.0"},
  {NSICK_PARAMETER_SENSOR_ROLL, "0.0"},

  {NSICK_PARAMETER_CONTROL_FREQUENCY, "10.0"},
};

config_t nsick_default_config = {
  nsick_default_params,
  sizeof(nsick_default_params)/sizeof(param_t),
};

void nsick_init(nsick_device_p dev, epos_node_p node, can_device_p can_dev,
    config_p config) {
  if (!node) {
    dev->node = malloc(sizeof(epos_node_t));
    epos_init(dev->node, can_dev, 0);
  }
  else
    dev->node = node;

  config_init_default(&dev->config, &nsick_default_config);
  if (config)
    config_set(&dev->config, config);

  dev->start_pos = deg_to_rad(config_get_float(&dev->config,
    NSICK_PARAMETER_START_POSITION));
  dev->end_pos = deg_to_rad(config_get_float(&dev->config,
    NSICK_PARAMETER_END_POSITION));
  dev->velocity = deg_to_rad(config_get_float(&dev->config,
    NSICK_PARAMETER_MAX_VELOCITY));
  dev->acceleration = deg_to_rad(config_get_float(&dev->config,
    NSICK_PARAMETER_MAX_ACCELERATION));

  dev->control_freq = config_get_float(&dev->config,
    NSICK_PARAMETER_CONTROL_FREQUENCY);

  transform_pose_t pose;
  transform_pose_init(&pose,
    config_get_float(&dev->config, NSICK_PARAMETER_SENSOR_X),
    config_get_float(&dev->config, NSICK_PARAMETER_SENSOR_Y),
    config_get_float(&dev->config, NSICK_PARAMETER_SENSOR_Z),
    config_get_float(&dev->config, NSICK_PARAMETER_SENSOR_YAW),
    config_get_float(&dev->config, NSICK_PARAMETER_SENSOR_PITCH),
    config_get_float(&dev->config, NSICK_PARAMETER_SENSOR_ROLL));
  nsick_sensor_init(&dev->sensor, &pose);

  pthread_mutex_init(&dev->mutex, NULL);
}

int nsick_init_arg(nsick_device_p dev, int argc, char **argv, const char*
    prefix, const char* args) {
  int result;
  epos_node_p node = malloc(sizeof(epos_node_t));

  if (!(result = epos_init_arg(node, argc, argv, 0, args))) {
    config_t config;
    if (result = config_init_arg(&config, argc, argv, (prefix) ? prefix :
        NSICK_CONFIG_ARG_PREFIX, args)) {
      config_print_usage(stdout, argv[0], args, result);
      config_print_help(stdout, &nsick_default_config, NSICK_CONFIG_ARG_PREFIX);
      config_print_help(stdout, &epos_default_config, EPOS_CONFIG_ARG_PREFIX);
      config_print_help(stdout, &can_default_config, CAN_CONFIG_ARG_PREFIX);
    }
    else
      nsick_init(dev, node, 0, &config);
    
    config_destroy(&config);
  }
  else
    config_print_help(stdout, &nsick_default_config, NSICK_CONFIG_ARG_PREFIX);

  return result;
}

void nsick_destroy(nsick_device_p dev) {
  epos_destroy(dev->node);
  config_destroy(&dev->config);

  pthread_mutex_destroy(&dev->mutex);
}

int nsick_open(nsick_device_p dev) {
  if (!epos_open(dev->node))
    return NSICK_ERROR_NONE;
  else
    return NSICK_ERROR_OPEN;
}

int nsick_close(nsick_device_p dev) {
  if (!epos_close(dev->node))
    return NSICK_ERROR_NONE;
  else
    return NSICK_ERROR_CLOSE;
}

int nsick_home(nsick_device_p dev, double timeout) {
  if (!epos_home(dev->node, timeout))
    return NSICK_ERROR_NONE;
  else
    return NSICK_ERROR_HOME;
}

int nsick_home_stop(nsick_device_p dev) {
  if (!epos_home_stop(dev->node))
    return NSICK_ERROR_NONE;
  else
    return NSICK_ERROR_HOME;
}

int nsick_home_wait(nsick_device_p dev, double timeout) {
  if (!epos_home_wait(dev->node, timeout))
    return NSICK_ERROR_NONE;
  else
    return NSICK_ERROR_WAIT_TIMEOUT;
}

void* nsick_control(void* device) {
  nsick_device_p dev = device;
  int result = EPOS_ERROR_NONE;
  int move_start = 1;
  double timestamp;

  epos_position_profile_init(&dev->profile, dev->start_pos, dev->velocity,
    dev->acceleration, dev->acceleration, epos_profile_sinusoidal);

  pthread_mutex_lock(&dev->mutex);
  while (!thread_test_exit(&dev->thread) &&
      (!dev->max_sweeps || (dev->num_sweeps < dev->max_sweeps)) &&
      !(dev->result = epos_position_profile_start(dev->node,
        &dev->profile))) {
    timer_start(&timestamp);

    while (!thread_test_exit(&dev->thread) &&
        epos_profile_wait(dev->node, 0.0)) {
      pthread_mutex_unlock(&dev->mutex);

      timer_wait(timestamp, dev->control_freq);
      timer_start(&timestamp);

      pthread_mutex_lock(&dev->mutex);
    }

    dev->profile.target_value = (dev->profile.target_value == dev->start_pos) ?
      dev->end_pos : dev->start_pos;

    if (move_start)
      move_start = 0;
    else
      ++dev->num_sweeps;
  }
  pthread_mutex_unlock(&dev->mutex);

  return 0;
}

void nsick_cleanup(void* device) {
  nsick_device_p dev = device;
  epos_position_profile_stop(dev->node);
}

int nsick_start(nsick_device_p dev, size_t max_sweeps) {
  dev->max_sweeps = max_sweeps;
  dev->num_sweeps = 0;
  dev->result = EPOS_ERROR_NONE;
  
  thread_start(&dev->thread, nsick_control, nsick_cleanup, dev, 0.0);
}

int nsick_stop(nsick_device_p dev) {
  thread_exit(&dev->thread, 1);

  if (!dev->result)
    return NSICK_ERROR_NONE;
  else
    return NSICK_ERROR_PROFILE;
}

int nsick_wait(nsick_device_p dev, double timeout) {
  if (thread_wait(&dev->thread, timeout) != THREAD_ERROR_WAIT_TIMEOUT)
    return NSICK_ERROR_NONE;
  else
    return NSICK_ERROR_WAIT_TIMEOUT;
}

double nsick_get_pose(nsick_device_p dev, transform_pose_p pose) {
  double timestamp;

  pthread_mutex_lock(&dev->mutex);
  timer_start(&timestamp);
  float pitch = epos_get_position(dev->node);
  timer_correct(&timestamp);
  pthread_mutex_unlock(&dev->mutex);

  nsick_sensor_get_pose(&dev->sensor, pitch, pose);
  
  return timestamp;
}

double nsick_get_pose_estimate(nsick_device_p dev, transform_pose_p pose) {
  double timestamp;

  pthread_mutex_lock(&dev->mutex);
  timer_start(&timestamp);
  float pitch = epos_position_profile_estimate(&dev->profile, timestamp);
  pthread_mutex_unlock(&dev->mutex);

  nsick_sensor_get_pose(&dev->sensor, pitch, pose);

  return timestamp;
}
