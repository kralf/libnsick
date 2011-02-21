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

#ifndef NSICK_H
#define NSICK_H

#include <libepos/epos.h>
#include <libepos/home.h>
#include <libepos/position_profile.h>

#include <tulibs/config.h>
#include <tulibs/thread.h>

#include "sensor.h"

/** \file nsick.h
  * \brief Nodding SICK device functions
  */

/** Predefined nodding SICK constants
  */
#define NSICK_WAIT_FOREVER                   -1.0

#define NSICK_CONFIG_ARG_PREFIX              "nsick"

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
#define NSICK_PARAMETER_CONTROL_FREQUENCY    "ctrl-freq"

/** Predefined nodding SICK error codes
  */
#define NSICK_ERROR_NONE                       0
#define NSICK_ERROR_OPEN                       1
#define NSICK_ERROR_CLOSE                      2
#define NSICK_ERROR_HOME                       3
#define NSICK_ERROR_PROFILE                    4
#define NSICK_ERROR_WAIT_TIMEOUT               5

/** \brief Predefined nodding SICK error descriptions
  */
extern const char* nsick_errors[];

/** \brief Predefined nodding SICK default configuration
  */
extern config_t nsick_default_config;

/** \brief Structure defining a nodding SICK device
  */
typedef struct nsick_device_t {
  epos_node_p node;                 //!< The nodding SICK controller node.
  nsick_sensor_t sensor;            //!< The nodding SICK sensor.

  config_t config;                  //!< The nodding SICK configuration.

  float start_pos;                  //!< The start position in [rad].
  float end_pos;                    //!< The end position in [rad].
  float velocity;                   //!< The maximum velocity in [rad/s].
  float acceleration;               //!< The maximum acceleration in [rad/s^2].
  
  float control_freq;               //!< The control frequency in [Hz].
  size_t max_sweeps;                //!< The maximum number of sweeps.
  size_t num_sweeps;                //!< The number of sweeps traveled.
  
  thread_t thread;                  //!< The nodding SICK control thread.
  pthread_mutex_t mutex;            //!< The nodding SICK device mutex.
  int result;                       //!< The nodding SICK control result.

  epos_position_profile_t profile;  //!< The nodding SICK motion profile.
} nsick_device_t, *nsick_device_p;

/** \brief Initialize the nodding SICK device
  * \param[in] dev The nodding SICK device to be initialized.
  * \param[in] node The EPOS node of the nodding SICK device. If
  *   null, a node will be created from default parameters. Otherwise,
  *   the device will take possession of the node.
  * \param[in] can_dev The CAN communication device of the nodding SICK
  *   device. If null, a device will be created from default parameters.
  * \param[in] config The optional nodding SICK configuration parameters.
  *   Can be null.
  */
void nsick_init(
  nsick_device_p dev,
  epos_node_p node,
  can_device_p can_dev,
  config_p config);

/** \brief Initialize the nodding SICK device from command line arguments
  * \param[in] dev The nodding SICK device to be initialized.
  * \param[in] argc The number of supplied command line arguments.
  * \param[in] argv The list of supplied command line arguments.
  * \param[in] prefix An optional argument prefix.
  * \param[in] args An optional string naming the expected arguments.
  * \return The resulting configuration error code.
  */
int nsick_init_arg(
  nsick_device_p dev,
  int argc,
  char **argv,
  const char* prefix,
  const char* args);

/** \brief Destroy the nodding SICK device
  * \param[in] dev The nodding SICK device to be destroyed.
  */
void nsick_destroy(
  nsick_device_p dev);

/** \brief Open the nodding SICK device
  * This method opens the communication with the EPOS node of the device.
  * \param[in] dev The initialized nodding SICK device to be opened.
  * \return The resulting error code.
  */
int nsick_open(
  nsick_device_p dev);

/** \brief Close the nodding SICK device
  * This method closes the communication with the EPOS node of the device.
  * \param[in] dev The opened nodding SICK device to be closed.
  * \return The resulting error code.
  */
int nsick_close(
  nsick_device_p dev);

/** \brief Home the nodding SICK device
  * \param[in] dev The opened nodding SICK device to be homed.
  * \param[in] timeout The timeout of the wait operation in [s].
  * \return The resulting error code.
  */
int nsick_home(
  nsick_device_p dev,
  double timeout);

/** \brief Stop the nodding SICK homing operation
  * \param[in] dev The nodding SICK device to stop the homing operation for.
  * \return The resulting error code.
  */
int nsick_home_stop(
  nsick_device_p dev);

/** \brief Wait for completion of the nodding SICK profile travel
  * This method waits until the nodding SICK has completed homing.
  * \param[in] dev The nodding SICK device to complete homing.
  * \param[in] timeout The timeout of the wait operation in [s].
  * \return The resulting error code.
  */
int nsick_home_wait(
  nsick_device_p dev,
  double timeout);

/** \brief Start the nodding SICK device
  * This method starts the control thread of the nodding SICK device and
  * returns immediately.
  * \param[in] dev The opened nodding SICK device to be started.
  * \param[in] max_sweeps The maximum number of sweeps the nodding SICK will
  *   travel. Note that passing zero will imply infinitely many sweeps.
  * \return The resulting error code.
  */
int nsick_start(
  nsick_device_p dev,
  size_t max_sweeps);

/** \brief Stop the nodding SICK device
  * This method stops the control thread of the nodding SICK device and
  * waits for it to terminate.
  * \param[in] dev The started nodding SICK device to be stopped.
  * \return The resulting error code.
  */
int nsick_stop(
  nsick_device_p dev);

/** \brief Wait for completion of the nodding SICK profile travel
  * This method waits until the nodding SICK has completed the requested
  * number of sweeps.
  * \param[in] dev The started nodding SICK to complete the profile travel.
  * \param[in] timeout The timeout of the wait operation in [s].
  * \return The resulting error code.
  */
int nsick_wait(
  nsick_device_p dev,
  double timeout);

/** \brief Retrieve the actual pose of the nodding SICK sensor
  * Note that depending on the communication interface used to drive the
  * nodding SICK, this method may deliver slow results.
  * \param[in] dev The started nodding SICK to retrieve the actual sensor
  *   pose for.
  * \param[out] pose The retrieved actual pose of the sensor.
  * \return The timestamp of the actual sensor pose in [s].
  */
double nsick_get_pose(
  nsick_device_p dev,
  transform_pose_p pose);

/** \brief Retrieve the estimated pose of the nodding SICK sensor
  * \param[in] dev The started nodding SICK to estimate the sensor pose for.
  * \param[out] pose The estimated pose of the sensor.
  * \return The timestamp of the estimated sensor pose in [s].
  */
double nsick_get_pose_estimate(
  nsick_device_p dev,
  transform_pose_p pose);

#endif
