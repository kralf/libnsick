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

#include <macros.h>

#include "nsick.h"

const char* nsick_errors[] = {
  "Success",
  "Nodding SICK device configuration error",
  "Failed to connect nodding SICK device",
  "Failed to disconnect nodding SICK device",
  "Failed to home nodding SICK device",
  "Profile travel failed for nodding SICK device",
  "Nodding SICK device wait operation timed out",
};

config_param_t nsick_default_params[] = {
  {NSICK_PARAMETER_START_POSITION,
    config_param_type_float,
    "-45.0",
    "(-inf, inf)",
    "Start position of the profile in [deg]",
  },
  {NSICK_PARAMETER_END_POSITION,
    config_param_type_float,
    "45.0",
    "(-inf, inf)",
    "End position of the profile in [deg]",
  },
  {NSICK_PARAMETER_MAX_VELOCITY,
    config_param_type_float,
    "45.0",
    "[0.0, inf)",
    "Maximum velocity of the profile in [deg/s]",
  },
  {NSICK_PARAMETER_MAX_ACCELERATION,
    config_param_type_float,
    "45.0",
    "[0.0, inf)",
    "Maximum acceleration of the profile in [deg/s^2]",
  },
  {NSICK_PARAMETER_SENSOR_X,
    config_param_type_float,
    "0.037",
    "(-inf, inf)",
    "X-offset of the sensor origin in [m]"
  },
  {NSICK_PARAMETER_SENSOR_Y,
    config_param_type_float,
    "0.0",
    "(-inf, inf)",
    "Y-offset of the sensor origin in [m]"
  },
  {NSICK_PARAMETER_SENSOR_Z,
    config_param_type_float,
    "-0.032",
    "(-inf, inf)",
    "Z-offset of the sensor origin in [m]"
  },
  {NSICK_PARAMETER_SENSOR_YAW,
    config_param_type_float,
    "0.0",
    "(-inf, inf)",
    "Yaw-offset of the sensor origin in [deg]"
  },
  {NSICK_PARAMETER_SENSOR_PITCH,
    config_param_type_float,
    "0.0",
    "(-inf, inf)",
    "Pitch-offset of the sensor origin in [deg]"
  },
  {NSICK_PARAMETER_SENSOR_ROLL,
    config_param_type_float,
    "0.0",
    "(-inf, inf)",
    "Roll-offset of the sensor origin in [deg]"
  },
  {NSICK_PARAMETER_CONTROL_FREQUENCY,
    config_param_type_float,
    "10.0",
    "[0.0, inf)",
    "Frequency of the control loop in [Hz]"
  },
};

const config_default_t nsick_default_config = {
  nsick_default_params,
  sizeof(nsick_default_params)/sizeof(config_param_t),
};

void nsick_init_components(nsick_device_t* dev, epos_node_t* node,
  can_device_t* can_dev);

void nsick_init(nsick_device_t* dev, epos_node_t* node, can_device_t*
    can_dev) {
  config_init_default(&dev->config, &nsick_default_config);
  error_init(&dev->error, nsick_errors);
  
  nsick_init_components(dev, node, can_dev);
}

int nsick_init_config(nsick_device_t* dev, epos_node_t* node, can_device_t*
    can_dev, const config_t* config) {
  config_init_default(&dev->config, &nsick_default_config);
  error_init(&dev->error, nsick_errors);
  
  if (config_set(&dev->config, config))
    error_blame(&dev->error, &dev->config.error, NSICK_ERROR_CONFIG);
  nsick_init_components(dev, node, can_dev);
  
  return dev->error.code;
}

int nsick_init_config_parse(nsick_device_t* dev, config_parser_t* parser,
    const char* option_group, int argc, char **argv, config_parser_exit_t
    exit) {
  config_init_default(&dev->config, &nsick_default_config);
  error_init(&dev->error, nsick_errors);
  
  option_group = option_group ? option_group : NSICK_CONFIG_PARSER_OPTION_GROUP;
  config_parser_add_option_group(parser, option_group, &nsick_default_config,
    "Nodding SICK device options", 
    "These options control the settings which are specific to the nodding "
    "SICK device.");

  epos_node_t* node = malloc(sizeof(epos_node_t));
  if (epos_node_init_config_parse(node, parser, 0, argc, argv, exit))
    error_blame(&dev->error, &node->error, NSICK_ERROR_CONFIG);
  else if (config_set(&dev->config, &config_parser_get_option_group(
      parser, option_group)->options))
    error_blame(&dev->error, &dev->config.error, NSICK_ERROR_CONFIG);
  nsick_init_components(dev, node, 0);

  return dev->error.code;
}

void nsick_init_components(nsick_device_t* dev, epos_node_t* node,
    can_device_t* can_dev) {
  if (!node) {
    dev->node = malloc(sizeof(epos_node_t));
    epos_node_init(dev->node, can_dev);
  }
  else
    dev->node = node;
  
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

  pthread_mutex_init(&dev->control_mutex, NULL);
  pthread_mutex_init(&dev->profile_mutex, NULL);
}

void nsick_destroy(nsick_device_t* dev) {
  epos_node_destroy(dev->node);
  config_destroy(&dev->config);

  pthread_mutex_destroy(&dev->control_mutex);
  pthread_mutex_destroy(&dev->profile_mutex);
}

int nsick_connect(nsick_device_t* dev) {
  error_clear(&dev->error);
  
  if (epos_node_connect(dev->node))
    error_blame(&dev->error, &dev->node->error, NSICK_ERROR_CONNECT);
  
  return dev->error.code;
}

int nsick_disconnect(nsick_device_t* dev) {
  error_clear(&dev->error);
  
  if (epos_node_disconnect(dev->node))
    error_blame(&dev->error, &dev->node->error, NSICK_ERROR_DISCONNECT);
  
  return dev->error.code;
}

int nsick_home(nsick_device_t* dev, double timeout) {
  error_clear(&dev->error);
  
  if (epos_node_home(dev->node, timeout)) {
    if (dev->node->dev.error.code == EPOS_DEVICE_ERROR_WAIT_TIMEOUT)
      error_blame(&dev->error, &dev->node->error, NSICK_ERROR_WAIT_TIMEOUT);
    else
      error_blame(&dev->error, &dev->node->error, NSICK_ERROR_HOME);
  }
  
  return dev->error.code;
}

int nsick_home_stop(nsick_device_t* dev) {
  error_clear(&dev->error);
  
  if (epos_home_stop(dev->node))
    error_blame(&dev->error, &dev->node->error, NSICK_ERROR_HOME);
  
  return dev->error.code;
}

int nsick_home_wait(nsick_device_t* dev, double timeout) {
  error_clear(&dev->error);
  
  if (epos_home_wait(dev->node, timeout)) {
    if (dev->node->dev.error.code == EPOS_DEVICE_ERROR_WAIT_TIMEOUT)
      error_blame(&dev->error, &dev->node->error, NSICK_ERROR_WAIT_TIMEOUT);
    else
      error_blame(&dev->error, &dev->node->error, NSICK_ERROR_HOME);
  }
  
  return dev->error.code;
}

void* nsick_control(void* device) {
  nsick_device_t* dev = device;
  int result = EPOS_ERROR_NONE;
  int move_start = 1;
  double timestamp;

  pthread_mutex_lock(&dev->profile_mutex);
  epos_position_profile_init(&dev->profile, dev->start_pos, dev->velocity,
    dev->acceleration, dev->acceleration, epos_profile_sinusoidal, 0);

  pthread_mutex_lock(&dev->control_mutex);
  while (!thread_test_exit(&dev->thread) &&
      (!dev->max_sweeps || (dev->num_sweeps < dev->max_sweeps)) &&
      !(dev->result = epos_position_profile_start(dev->node,
        &dev->profile))) {
    pthread_mutex_unlock(&dev->profile_mutex);
    timer_start(&timestamp);

    while (!thread_test_exit(&dev->thread) &&
        epos_profile_wait(dev->node, 0.0)) {
      pthread_mutex_unlock(&dev->control_mutex);

      timer_wait(timestamp, dev->control_freq);
      timer_start(&timestamp);

      pthread_mutex_lock(&dev->control_mutex);
    }

    pthread_mutex_lock(&dev->profile_mutex);
    dev->profile.target_value = (dev->profile.target_value == dev->start_pos) ?
      dev->end_pos : dev->start_pos;

    if (move_start)
      move_start = 0;
    else
      ++dev->num_sweeps;
  }
  pthread_mutex_unlock(&dev->control_mutex);
  pthread_mutex_unlock(&dev->profile_mutex);

  return 0;
}

void nsick_cleanup(void* device) {
  nsick_device_t* dev = device;
  epos_position_profile_stop(dev->node);
}

int nsick_start(nsick_device_t* dev, size_t max_sweeps) {
  dev->max_sweeps = max_sweeps;
  dev->num_sweeps = 0;
  dev->result = EPOS_ERROR_NONE;
  
  thread_start(&dev->thread, nsick_control, nsick_cleanup, dev, 0.0);
}

int nsick_stop(nsick_device_t* dev) {
  error_clear(&dev->error);
  
  thread_exit(&dev->thread, 1);
  if (dev->result)
    error_set(&dev->error, NSICK_ERROR_PROFILE);
  
  return dev->error.code;
}

int nsick_wait(nsick_device_t* dev, double timeout) {
  error_clear(&dev->error);
  
  if (thread_wait(&dev->thread, timeout) == THREAD_ERROR_WAIT_TIMEOUT)
    error_set(&dev->error, NSICK_ERROR_WAIT_TIMEOUT);  
  
  return dev->error.code;
}

double nsick_get_pose(nsick_device_t* dev, transform_pose_t* pose) {
  double timestamp;

  pthread_mutex_lock(&dev->control_mutex);
  timer_start(&timestamp);
  float pitch = epos_node_get_position(dev->node);
  timer_correct(&timestamp);
  pthread_mutex_unlock(&dev->control_mutex);

  nsick_sensor_get_pose(&dev->sensor, pitch, pose);
  
  return timestamp;
}

double nsick_get_pose_estimate(nsick_device_t* dev, transform_pose_t* pose) {
  double timestamp;

  pthread_mutex_lock(&dev->profile_mutex);
  timer_start(&timestamp);
  epos_profile_value_t value = epos_position_profile_eval(
    &dev->profile, timestamp);
  pthread_mutex_unlock(&dev->profile_mutex);

  nsick_sensor_get_pose(&dev->sensor, value.position, pose);

  return timestamp;
}
