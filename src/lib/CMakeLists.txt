remake_find_library(
  tulibs-thread thread.h
  PACKAGE tulibs
)
remake_find_library(
  tulibs-transform transform.h
  PACKAGE tulibs
)
remake_find_library(
  epos epos.h
  PACKAGE libepos
)

remake_add_library(
  nsick PREFIX OFF
  LINK ${TULIBS_THREAD_LIBRARY} ${TULIBS_TRANSFORM_LIBRARY} ${EPOS_LIBRARY}
)
remake_add_headers()
