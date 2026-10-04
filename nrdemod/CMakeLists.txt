cmake_minimum_required(VERSION 3.15)
project(nrdemod VERSION 1.0.0 LANGUAGES CXX)

option(NRDEMOD_BUILD_TESTS "Build the C++ self test" ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# Python loads this by file name, so keep it stable on every platform.
add_library(nrdemod SHARED
  src/NrOfdmCore.cpp
  src/nrdemod_c.cpp
)

target_include_directories(nrdemod PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_definitions(nrdemod PRIVATE NRDEMOD_BUILD_SHARED)

set_target_properties(nrdemod PROPERTIES
  OUTPUT_NAME nrdemod
  PREFIX ""                 # libnrdemod.so -> nrdemod.so, matches nrdemod.dll
  WINDOWS_EXPORT_ALL_SYMBOLS OFF
)

if(MSVC)
  target_compile_options(nrdemod PRIVATE /O2 /EHsc /bigobj)
  target_compile_definitions(nrdemod PRIVATE _USE_MATH_DEFINES NOMINMAX)
else()
  target_compile_options(nrdemod PRIVATE -O2 -Wall -Wextra)
endif()

# Drop the shared library next to the Python package so `import nrdemod` finds
# it without any extra environment setup.
set_target_properties(nrdemod PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/python/nrdemod/_lib
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/python/nrdemod/_lib
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib
)
foreach(cfg IN ITEMS DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
  set_target_properties(nrdemod PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY_${cfg} ${CMAKE_CURRENT_SOURCE_DIR}/python/nrdemod/_lib
    RUNTIME_OUTPUT_DIRECTORY_${cfg} ${CMAKE_CURRENT_SOURCE_DIR}/python/nrdemod/_lib
  )
endforeach()

if(NRDEMOD_BUILD_TESTS)
  enable_testing()
  add_executable(nrdemod_core_test tests/NrOfdmCoreTest.cpp src/NrOfdmCore.cpp)
  target_include_directories(nrdemod_core_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  if(MSVC)
    target_compile_definitions(nrdemod_core_test PRIVATE _USE_MATH_DEFINES NOMINMAX)
    target_compile_options(nrdemod_core_test PRIVATE /EHsc /bigobj)
  endif()
  add_test(NAME nrdemod_core_test COMMAND nrdemod_core_test)
endif()

install(TARGETS nrdemod
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
