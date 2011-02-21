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
  
  if (nsick_init_arg(&dev, argc, argv, 0, 0))
    return -1;

  signal(SIGINT, nsick_signaled);

  if (nsick_open(&dev))
    return -1;
  if (!nsick_home(&dev, 0.0)) {
    while (!quit && nsick_home_wait(&dev, 0.1));
    nsick_home_stop(&dev);
  }
  nsick_close(&dev);

  nsick_destroy(&dev);
  return 0;
}
