if (CONFIG_LIB_SAM_BLEND)
  zephyr_include_directories(include/)

  include(src/CMakeLists.txt)
endif()
