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

#include "sensor.h"

void nsick_sensor_init(nsick_sensor_p sensor, const transform_pose_t* pose) {
  transform_pose_copy(&sensor->pose, pose);
}

void nsick_sensor_get_pose(nsick_sensor_p sensor, float pitch,  
    transform_pose_t* pose) {
  transform_point_t origin;
  transform_t rotation;
  
  transform_point_init(&origin, sensor->pose.x, sensor->pose.y,
    sensor->pose.z);
  transform_init_rotation(rotation, 0.0, pitch, 0.0);  
  transform_point(rotation, &origin);

  pose->x = origin.x;
  pose->y = origin.y;
  pose->z = origin.z;
  pose->yaw = sensor->pose.yaw;
  pose->pitch = sensor->pose.pitch+pitch;
  pose->roll = sensor->pose.roll;
}

void nsick_sensor_transform_point(nsick_sensor_p sensor, float pitch,
    transform_point_t* point){
  transform_pose_t pose;
  transform_t transform;
  
  nsick_sensor_get_pose(sensor, pitch, &pose);
  transform_init_pose(transform, &pose);
  transform_point(transform, point);
}

void nsick_sensor_transform_points(nsick_sensor_p sensor, float pitch,
    transform_point_t* points, size_t num_points) {
  transform_pose_t pose;
  transform_t transform;

  nsick_sensor_get_pose(sensor, pitch, &pose);
  transform_init_pose(transform, &pose);
  transform_points(transform, points, num_points);
}
