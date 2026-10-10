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

# Second shared library: native WiFi (a/b/n/ac/ax/ad/be) / Bluetooth LE / UWB
# cores behind the demod_c.h flat C ABI. Kept separate from nrdemod (5G NR)
# since they have no 5G NR dependency and existing nrdemod consumers should
# not need to relink against them.
add_library(demod SHARED
  src/WifiOfdmCore.cpp
  src/Wifi11nOfdmCore.cpp
  src/Wifi11acOfdmCore.cpp
  src/Wifi11axOfdmCore.cpp
  src/Wifi11adOfdmCore.cpp
  src/Wifi11bOfdmCore.cpp
  src/Wifi11beOfdmCore.cpp
  src/BtGfskCore.cpp
  src/UwbHrpCore.cpp
  src/demod_c.cpp
)

target_include_directories(demod PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_definitions(demod PRIVATE NRDEMOD_BUILD_SHARED)

set_target_properties(demod PROPERTIES
  OUTPUT_NAME demod
  PREFIX ""
  WINDOWS_EXPORT_ALL_SYMBOLS OFF
)

if(MSVC)
  target_compile_options(demod PRIVATE /O2 /EHsc /bigobj)
  target_compile_definitions(demod PRIVATE _USE_MATH_DEFINES NOMINMAX)
else()
  target_compile_options(demod PRIVATE -O2 -Wall -Wextra)
endif()

set_target_properties(demod PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/python/nrdemod/_lib
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/python/nrdemod/_lib
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib
)
foreach(cfg IN ITEMS DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
  set_target_properties(demod PROPERTIES
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

  set(DEMOD_CORE_TESTS
    WifiOfdmCoreTest:WifiOfdmCore
    Wifi11nOfdmCoreTest:Wifi11nOfdmCore
    Wifi11acOfdmCoreTest:Wifi11acOfdmCore
    Wifi11axOfdmCoreTest:Wifi11axOfdmCore
    Wifi11adOfdmCoreTest:Wifi11adOfdmCore
    Wifi11bOfdmCoreTest:Wifi11bOfdmCore
    Wifi11beOfdmCoreTest:Wifi11beOfdmCore
    BtGfskCoreTest:BtGfskCore
    UwbHrpCoreTest:UwbHrpCore
  )
  foreach(entry IN LISTS DEMOD_CORE_TESTS)
    string(REPLACE ":" ";" pair "${entry}")
    list(GET pair 0 testName)
    list(GET pair 1 coreName)
    add_executable(${testName} tests/${testName}.cpp src/${coreName}.cpp)
    target_include_directories(${testName} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    if(MSVC)
      target_compile_definitions(${testName} PRIVATE _USE_MATH_DEFINES NOMINMAX)
      target_compile_options(${testName} PRIVATE /EHsc /bigobj)
    endif()
    add_test(NAME ${testName} COMMAND ${testName})
  endforeach()
endif()

install(TARGETS nrdemod demod
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
