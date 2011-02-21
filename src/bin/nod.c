/***************************************************************************
 *   Copyright (C) 2004 by Ralf Kaestner                                   *
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

#include <stdio.h>
#include <signal.h>

#include "nsick.h"

int quit = 0;

void nsick_signaled(int signal) {
  quit = 1;
}

int main(int argc, char **argv) {
  nsick_device_t dev;
  transform_pose_t pose;
  
  if (nsick_init_arg(&dev, argc, argv, 0, "SWEEPS"))
    return -1;
  size_t max_sweeps = atoi(argv[1]);

  signal(SIGINT, nsick_signaled);

  if (!nsick_open(&dev)) {
    fprintf(stderr, "\rHoming: ");
    fflush(stderr);
    
    if (!nsick_home(&dev, 0.0)) {
      while (!quit && nsick_home_wait(&dev, 0.1));
      if (!quit) {
        fprintf(stderr, "success\n");

        if (!nsick_start(&dev, max_sweeps)) {
          fprintf(stderr, "Origin: %8s  %8s  %8s  %8s  %12s\n",
            "time [s]", "x [m]", "y [m]", "z [m]", "pitch [deg]");
          while (!quit && nsick_wait(&dev, 0.1)) {
            double time = nsick_get_pose_estimate(&dev, &pose);
            fprintf(stdout,
              "%16.4f  %8.4f  %8.4f  %8.4f  %12.2f\n",
              time, pose.x, pose.y, pose.z, rad_to_deg(pose.pitch));
          }
          nsick_stop(&dev);
        }
      }
      else {
        nsick_home_stop(&dev);
        fprintf(stderr, "interrupt\n");
      }
    }
    else
      fprintf(stderr, "failure\n");
  }
  nsick_close(&dev);

  nsick_destroy(&dev);
  return 0;
}
