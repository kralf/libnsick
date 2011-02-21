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

#ifndef NSICK_SENSOR_H
#define NSICK_SENSOR_H

#include <tulibs/transform.h>

/** \file sensor.h
  * \brief Nodding SICK sensor functions
  */

/** \brief Structure defining a nodding SICK sensor
  */
typedef struct nsick_sensor_t {
  transform_pose_t pose;              //!< The pose of the SICK sensor.
} nsick_sensor_t, *nsick_sensor_p;

/** \brief Initialize the nodding SICK sensor
  * \param[in] sensor The nodding SICK sensor to be initialized.
  * \param[in] pose The pose of the nodding SICK sensor, i.e. its static
  *   offset with respect to the setup's motion axis.
  */
void nsick_sensor_init(
  nsick_sensor_p sensor,
  transform_pose_p pose);

/** \brief Retrieve the pose of the nodding SICK sensor
  * \param[in] sensor The nodding SICK sensor to retrieve the pose for.
  * \param[in] pitch The angular pitch of the sensor in [rad].
  * \param[out] pose The retrieved pose of the sensor.
  */
void nsick_sensor_get_pose(
  nsick_sensor_p sensor,
  float pitch,
  transform_pose_p pose);

/** \brief Transform a point from sensor to device coordinates
  * \param[in] sensor The nodding SICK sensor to transform the point for.
  * \param[in] pitch The angular pitch of the sensor in [rad].
  * \param[in,out] point The point to be transformed.
  */
void nsick_sensor_transform_point(
  nsick_sensor_p sensor,
  float pitch,
  transform_point_p point);

/** \brief Transform an array of points from sensor to device coordinates
  * \param[in] sensor The nodding SICK sensor to transform the points for.
  * \param[in] pitch The angular pitch of the sensor in [rad].
  * \param[in,out] points The array of points to be transformed.
  * \param[in] num_points The number of points in the array.
  */
void nsick_sensor_transform_points(
  nsick_sensor_p sensor,
  float pitch,
  transform_point_p points,
  size_t num_points);
  
#endif
