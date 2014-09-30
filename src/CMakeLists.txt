remake_find_package(tulibs CONFIG)
remake_find_package(libepos CONFIG)

remake_include(${TULIBS_INCLUDE_DIRS} ${LIBEPOS_INCLUDE_DIRS})
remake_add_directories(lib)
remake_add_directories(bin COMPONENT utils)
