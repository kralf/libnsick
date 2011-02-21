cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  libnsick
  VERSION 0.1
  RELEASE alpha
  SUMMARY "A C-based driver library for the nodding SICK sensor."
  AUTHOR "Ralf Kaestner"
  CONTACT "ralf.kaestner@gmail.com"
  HOME "http://aslforge.ethz.ch/projects/libnsick"
  LICENSE "GNU Lesser General Public License (LGPL)"
  PREFIX nsick-
  INSTALL /usr
)

remake_doc(
  html
)

remake_pack_deb(
  libepos tulibs
)
remake_pack_deb(
  COMPONENT utils
  DESCRIPTION "utilities"
  libnsick
)
remake_pack_deb(
  COMPONENT dev
  DESCRIPTION "development headers"
  libnsick libepos-dev tulibs-dev
)
remake_pack_deb(
  ARCH all
  COMPONENT doc
  DESCRIPTION "documentation"
)
