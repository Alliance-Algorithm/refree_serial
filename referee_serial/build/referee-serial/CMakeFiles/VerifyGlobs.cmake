# CMAKE generated file: DO NOT EDIT!
# Generated by CMake Version 3.22
cmake_policy(SET CMP0009 NEW)

# PROJECT_SOURCE at CMakeLists.txt:15 (file)
file(GLOB_RECURSE NEW_GLOB LIST_DIRECTORIES false "/workspace/referee_serial/src/referee-serial/src/*.c")
set(OLD_GLOB
  )
if(NOT "${NEW_GLOB}" STREQUAL "${OLD_GLOB}")
  message("-- GLOB mismatch!")
  file(TOUCH_NOCREATE "/workspace/referee_serial/build/referee-serial/CMakeFiles/cmake.verify_globs")
endif()

# PROJECT_SOURCE at CMakeLists.txt:15 (file)
file(GLOB_RECURSE NEW_GLOB LIST_DIRECTORIES false "/workspace/referee_serial/src/referee-serial/src/*.cpp")
set(OLD_GLOB
  "/workspace/referee_serial/src/referee-serial/src/referee_serial.cpp"
  )
if(NOT "${NEW_GLOB}" STREQUAL "${OLD_GLOB}")
  message("-- GLOB mismatch!")
  file(TOUCH_NOCREATE "/workspace/referee_serial/build/referee-serial/CMakeFiles/cmake.verify_globs")
endif()
