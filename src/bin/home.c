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

#include "nsick.h"

int quit = 0;

void nsick_signaled(int signal) {
  quit = 1;
}

int main(int argc, char **argv) {
  config_parser_t parser;
  nsick_device_t dev;
  transform_pose_t pose;
  
  config_parser_init(&parser,
    "Home nodding SICK device",
    "Establish the communication with a connected nodding SICK device and "
    "attempt to home it. The homing will be stopped if SIGINT is received "
    "or the homing operation is completed.");  
  nsick_init_config_parse(&dev, &parser, 0, argc, argv,
    config_parser_exit_error);
  config_parser_destroy(&parser);

  signal(SIGINT, nsick_signaled);

  nsick_connect(&dev);
  error_exit(&dev.error);

  nsick_home(&dev, 0.0);
  if (dev.error.code != NSICK_ERROR_WAIT_TIMEOUT)
    error_exit(&dev.error);

  while (!quit) {
    if (nsick_home_wait(&dev, 0.1) != NSICK_ERROR_WAIT_TIMEOUT)
      error_exit(&dev.error);
  };
  
  nsick_home_stop(&dev);
  error_exit(&dev.error);
  
  nsick_disconnect(&dev);
  error_exit(&dev.error);

  nsick_destroy(&dev);
  return 0;
}
