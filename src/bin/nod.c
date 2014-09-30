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

#include <config/parser.h>

#include <macros.h>

#include "nsick.h"

#define NSICK_NOD_PARAMETER_NUM_SWEEPS                    "NUM_SWEEPS"

config_param_t nsick_nod_default_arguments_params[] = {
  {NSICK_NOD_PARAMETER_NUM_SWEEPS,
    config_param_type_int,
    "0",
    "[0, 10000]",
    "The number of sweeping profile travels to be performed or zero "
    "for continuous sweeping"
  },
};

const config_default_t nsick_nod_default_arguments = {
  nsick_nod_default_arguments_params,
  sizeof(nsick_nod_default_arguments_params)/sizeof(config_param_t),
};

int quit = 0;

void nsick_signaled(int signal) {
  quit = 1;
}

int main(int argc, char **argv) {
  config_parser_t parser;
  nsick_device_t dev;
  transform_pose_t pose;
  
  config_parser_init_default(&parser, &nsick_nod_default_arguments, 0,
    "Sweep nodding SICK device",
    "Establish the communication with a connected nodding SICK device, "
    "attempt to home it, and perform a sweeping profile travel. The "
    "sweeping will be stopped if SIGINT is received or the specified "
    "number of sweeps is reached.");  
  nsick_init_config_parse(&dev, &parser, 0, argc, argv,
    config_parser_exit_error);
  config_parser_destroy(&parser);
  
  int max_sweeps = config_get_int(&parser.arguments,
    NSICK_NOD_PARAMETER_NUM_SWEEPS);
  
  signal(SIGINT, nsick_signaled);

  nsick_connect(&dev);
  error_exit(&dev.error);
  
  fprintf(stderr, "\rHoming: ");
  fflush(stderr);

  nsick_home(&dev, 0.0);
  if (dev.error.code != NSICK_ERROR_WAIT_TIMEOUT) {
    fprintf(stderr, "failure\n");
    error_exit(&dev.error);
  }

  while (!quit) {
    if (nsick_home_wait(&dev, 0.1) != NSICK_ERROR_WAIT_TIMEOUT) {
      fprintf(stderr, "failure\n");
      error_exit(&dev.error);
    }
  }
  
  if (!quit) {
    fprintf(stderr, "success\n");
    
    nsick_start(&dev, max_sweeps);
    error_exit(&dev.error);
      
    fprintf(stderr, "Origin: %8s  %8s  %8s  %8s  %12s\n",
      "time [s]", "x [m]", "y [m]", "z [m]", "pitch [deg]");
    
    while (!quit) {
      if (nsick_wait(&dev, 0.1) != NSICK_ERROR_WAIT_TIMEOUT)
        error_exit(&dev.error);
      
      double time = nsick_get_pose_estimate(&dev, &pose);
      fprintf(stdout,
        "%16.4f  %8.4f  %8.4f  %8.4f  %12.2f\n",
        time, pose.x, pose.y, pose.z, rad_to_deg(pose.pitch));
    }
    
    nsick_stop(&dev);
    error_exit(&dev.error);
  }
  else {
    nsick_home_stop(&dev);
    fprintf(stderr, "interrupt\n");
  }
  
  nsick_disconnect(&dev);
  error_exit(&dev.error);

  nsick_destroy(&dev);
  return 0;
}
