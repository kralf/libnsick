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

#include <epos.h>
#include <home.h>
#include <position_profile.h>

#include <config/config.h>
#include <thread/thread.h>
#include <error/error.h>

#include "sensor.h"

/** \file nsick.h
  * \brief Nodding SICK device functions
  */

/** \brief Predefined nodding SICK configuration parser option group
  */
#define NSICK_CONFIG_PARSER_OPTION_GROUP     "nsick"

/** \name Parameters
  * \brief Predefined nodding SICK parameters
  */
//@{
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
//@}

/** \name Constants
  * \brief Predefined nodding SICK constants
  */
//@{
#define NSICK_WAIT_FOREVER                   -1.0
//@}

/** \name Error Codes
  * \brief Predefined nodding SICK error codes
  */
//@{
#define NSICK_ERROR_NONE                       0
//!< Success
#define NSICK_ERROR_CONFIG                     1
//!< Nodding SICK device configuration error
#define NSICK_ERROR_CONNECT                    2
//!< Failed to connect nodding SICK device
#define NSICK_ERROR_DISCONNECT                 3
//!< Failed to disconnect nodding SICK device
#define NSICK_ERROR_HOME                       4
//!< Failed to home nodding SICK device
#define NSICK_ERROR_PROFILE                    5
//!< Profile travel failed for nodding SICK device
#define NSICK_ERROR_WAIT_TIMEOUT               6
//!< Nodding SICK device wait operation timed out
//@}

/** \brief Predefined nodding SICK error descriptions
  */
extern const char* nsick_errors[];

/** \brief Predefined nodding SICK default configuration
  */
extern const config_default_t nsick_default_config;

/** \brief Structure defining a nodding SICK device
  */
typedef struct nsick_device_t {
  epos_node_t* node;                //!< The nodding SICK controller node.
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
  pthread_mutex_t control_mutex;    //!< The nodding SICK control mutex.
  pthread_mutex_t profile_mutex;    //!< The nodding SICK profile mutex.
  int result;                       //!< The nodding SICK control result.

  epos_position_profile_t profile;  //!< The nodding SICK motion profile.
  
  error_t error;                    //!< The most recent device error.
} nsick_device_t;

/** \brief Initialize the nodding SICK device
  * \note The device will be initialized using default configuration parameters.
  * \param[in] dev The nodding SICK device to be initialized.
  * \param[in] node The EPOS node of the nodding SICK device. If
  *   null, a node will be created from default parameters. Otherwise,
  *   the device will take possession of the node.
  * \param[in] can_dev The CAN communication device of the nodding SICK
  *   device. If null, a device will be created from default parameters.
  */
void nsick_init(
  nsick_device_t* dev,
  epos_node_t* node,
  can_device_t* can_dev);

/** \brief Initialize the nodding SICK device from configuration
  * \param[in] dev The nodding SICK device to be initialized.
  * \param[in] node The EPOS node of the nodding SICK device. If
  *   null, a node will be created from default parameters. Otherwise,
  *   the device will take possession of the node.
  * \param[in] can_dev The CAN communication device of the nodding SICK
  *   device. If null, a device will be created from default parameters.
  * \param[in] config The optional nodding SICK configuration parameters.
  *   Can be null.
  * \return The resulting error code.
  */
int nsick_init_config(
  nsick_device_t* dev,
  epos_node_t* node,
  can_device_t* can_dev,
  const config_t* config);

/** \brief Initialize the nodding SICK device by parsing command line arguments
  * \param[in] dev The nodding SICK device to be initialized.
  * \param[in] parser The initialized configuration parser which will
  *   be used to parse the command line arguments into the nodding SICK
  *   device configuration.
  * \param[in] option_group An optional name of the parser option group
  *   containing the nodding SICK device configuration parameters. If null,
  *   the default name is chosen.
  * \param[in] argc The number of supplied command line arguments.
  * \param[in] argv The list of supplied command line arguments.
  * \param[in] exit The exit policy of the parser in case of an error
  *   or help request.
  * \return The resulting error code.
  */
int nsick_init_config_parse(
  nsick_device_t* dev,
  config_parser_t* parser,
  const char* option_group,
  int argc,
  char **argv,
  config_parser_exit_t exit);

/** \brief Destroy the nodding SICK device
  * \param[in] dev The nodding SICK device to be destroyed.
  */
void nsick_destroy(
  nsick_device_t* dev);

/** \brief Connect the nodding SICK device
  * This method connects the EPOS node of the device.
  * \param[in] dev The initialized nodding SICK device to be connected.
  * \return The resulting error code.
  */
int nsick_connect(
  nsick_device_t* dev);

/** \brief Disconnect the nodding SICK device
  * This method disconnects the EPOS node of the device.
  * \param[in] dev The connected nodding SICK device to be disconnected.
  * \return The resulting error code.
  */
int nsick_close(
  nsick_device_t* dev);

/** \brief Home the nodding SICK device
  * \param[in] dev The connected nodding SICK device to be homed.
  * \param[in] timeout The timeout of the wait operation in [s].
  * \return The resulting error code.
  */
int nsick_home(
  nsick_device_t* dev,
  double timeout);

/** \brief Stop the nodding SICK homing operation
  * \param[in] dev The nodding SICK device to stop the homing operation for.
  * \return The resulting error code.
  */
int nsick_home_stop(
  nsick_device_t* dev);

/** \brief Wait for completion of the nodding SICK profile travel
  * This method waits until the nodding SICK has completed homing.
  * \param[in] dev The nodding SICK device to complete homing.
  * \param[in] timeout The timeout of the wait operation in [s].
  * \return The resulting error code.
  */
int nsick_home_wait(
  nsick_device_t* dev,
  double timeout);

/** \brief Start the nodding SICK device
  * This method starts the control thread of the nodding SICK device and
  * returns immediately.
  * \param[in] dev The connected nodding SICK device to be started.
  * \param[in] max_sweeps The maximum number of sweeps the nodding SICK will
  *   travel. Note that passing zero will imply infinitely many sweeps.
  * \return The resulting error code.
  */
int nsick_start(
  nsick_device_t* dev,
  size_t max_sweeps);

/** \brief Stop the nodding SICK device
  * This method stops the control thread of the nodding SICK device and
  * waits for it to terminate.
  * \param[in] dev The started nodding SICK device to be stopped.
  * \return The resulting error code.
  */
int nsick_stop(
  nsick_device_t* dev);

/** \brief Wait for completion of the nodding SICK profile travel
  * This method waits until the nodding SICK has completed the requested
  * number of sweeps.
  * \param[in] dev The started nodding SICK to complete the profile travel.
  * \param[in] timeout The timeout of the wait operation in [s].
  * \return The resulting error code.
  */
int nsick_wait(
  nsick_device_t* dev,
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
  nsick_device_t* dev,
  transform_pose_t* pose);

/** \brief Retrieve the estimated pose of the nodding SICK sensor
  * \param[in] dev The started nodding SICK to estimate the sensor pose for.
  * \param[out] pose The estimated pose of the sensor.
  * \return The timestamp of the estimated sensor pose in [s].
  */
double nsick_get_pose_estimate(
  nsick_device_t* dev,
  transform_pose_t* pose);

#endif
