cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  libnsick
  VERSION 0.1
  RELEASE alpha
  SUMMARY "A C-based driver library for the nodding SICK sensor."
  AUTHOR "Ralf Kaestner"
  CONTACT "ralf.kaestner@gmail.com"
  HOME "http://github.com/kralf/libnsick"
  LICENSE "GNU Lesser General Public License (LGPL)"
  PREFIX nsick-
  INSTALL /usr
)

remake_doc(
  html
)

remake_pack_deb(
  DEPENDS libepos tulibs
)
remake_pack_deb(
  COMPONENT utils
  DESCRIPTION "utilities"
  DEPENDS libnsick
)
remake_pack_deb(
  COMPONENT dev
  DESCRIPTION "development headers"
  DEPENDS libnsick libepos-dev tulibs-dev
)
remake_pack_deb(
  ARCH all
  COMPONENT doc
  DESCRIPTION "documentation"
)

remake_distribute_deb(
  DISTRIBUTION lucid
  SECTION libs
  UPLOAD ppa:kralf/asl
  DEPENDS libepos-dev tulibs-dev remake pkg-config doxygen
  PASS CMAKE_BUILD_TYPE LIBNSICK_GIT_REVISION
)
remake_distribute_deb(
  DISTRIBUTION precise
  SECTION libs
  UPLOAD ppa:kralf/asl
  DEPENDS libepos-dev tulibs-dev remake pkg-config doxygen
  PASS CMAKE_BUILD_TYPE LIBNSICK_GIT_REVISION
)
remake_distribute_deb(
  DISTRIBUTION trusty
  SECTION libs
  UPLOAD ppa:kralf/asl
  DEPENDS libepos-dev tulibs-dev remake pkg-config doxygen
  PASS CMAKE_BUILD_TYPE LIBNSICK_GIT_REVISION
)
