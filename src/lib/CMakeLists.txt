remake_add_library(
  nsick PREFIX OFF
  LINK ${TULIBS_LIBRARIES} ${LIBEPOS_LIBRARIES}
)
remake_add_headers()
remake_pkg_config_generate(REQUIRES tulibs libepos)
