#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "CppUnitLite" for configuration "Release"
set_property(TARGET CppUnitLite APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(CppUnitLite PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libCppUnitLite.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS CppUnitLite )
list(APPEND _IMPORT_CHECK_FILES_FOR_CppUnitLite "${_IMPORT_PREFIX}/lib/libCppUnitLite.a" )

# Import target "wrap" for configuration "Release"
set_property(TARGET wrap APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(wrap PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/wrap"
  )

list(APPEND _IMPORT_CHECK_TARGETS wrap )
list(APPEND _IMPORT_CHECK_FILES_FOR_wrap "${_IMPORT_PREFIX}/bin/wrap" )

# Import target "gtsam" for configuration "Release"
set_property(TARGET gtsam APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(gtsam PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "/usr/lib/x86_64-linux-gnu/libboost_serialization.so;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_thread.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so;/usr/lib/x86_64-linux-gnu/libboost_regex.so;/usr/lib/x86_64-linux-gnu/libboost_timer.so;/usr/lib/x86_64-linux-gnu/libboost_chrono.so;/usr/lib/libtbb.so;/usr/lib/libtbbmalloc.so"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libgtsam.so.3.2.0"
  IMPORTED_SONAME_RELEASE "libgtsam.so.3"
  )

list(APPEND _IMPORT_CHECK_TARGETS gtsam )
list(APPEND _IMPORT_CHECK_FILES_FOR_gtsam "${_IMPORT_PREFIX}/lib/libgtsam.so.3.2.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
