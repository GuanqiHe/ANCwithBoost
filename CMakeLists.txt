# project specific information
cmake_minimum_required(VERSION 3.19)


if (NOT DEFINED SYSTEM_TYPE)
    set (SYSTEM_TYPE "Windows")
    message("System type not defined, set Windows as default")
endif()



if (DEFINED BUILD_SIMULATION_ONLY)
    message("Detect flag -DBUILD_SIMULATION_ONLY=${BUILD_SIMULATION_ONLY}")
endif()




if (${SYSTEM_TYPE} STREQUAL "Windows")
    
    if (NOT DEFINED BUILD_SIMULATION_ONLY)

        message("System: Windows")

        set(CMAKE_SYSTEM_NAME Linux)
        set(CMAKE_SYSTEM_PROCESSOR x86_64)

        set(toolchainpath C:/build/23.0/x64/sysroots)

        set(CMAKE_C_COMPILER ${toolchainpath}/x86_64-w64-mingw32/usr/bin/x86_64-nilrt-linux/x86_64-nilrt-linux-gcc.exe)
        set(CMAKE_CXX_COMPILER ${toolchainpath}/x86_64-w64-mingw32/usr/bin/x86_64-nilrt-linux/x86_64-nilrt-linux-g++.exe)

        set(CMAKE_CXX_STANDARD 17)
        set(CMAKE_CXX_STANDARD_REQUIRED ON)
        # set(CMAKE_CXX_EXTENSIONS OFF)

        set(CMAKE_SYSROOT ${toolchainpath}/core2-64-nilrt-linux)
        set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${toolchainpath}/core2-64-nilrtlinux/usr/include/c++/10.3.0 ${toolchainpath}/core2-64-nilrt-linux/usr/include/c++/10.3.0/x86_64-nilrtlinux)

        set(CMAKE_CXX_FLAGS "-Wall -fmessage-length=0 -std=c++17")
        set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
        set(CMAKE_CXX_FLAGS_RELEASE "-O3")

        set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
        set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
        set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
        set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)     

    else()


        message("BUILD_SIMULATION_ONLY")

        set(CMAKE_CXX_STANDARD 17)
        set(CMAKE_CXX_STANDARD_REQUIRED ON)

        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
        set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g3")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

        set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
        set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
        set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
        set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)     

    endif()
    

elseif (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR "Haven't integrate Linux compiler setup yet")
else ()
    message(FATAL_ERROR "Unsupport compile platform")
endif()


project(ActiveNoiseCancellation)


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/run/bin/${PREFIX})
set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)
set(HEADER_DIRS include lib/Eigen lib lib/boost src lib/yaml-cpp lib/msgpack/include libs/nidaqmx utils)
set(LIB_DIRS lib libs/nidaqmx)

if (NOT DEFINED BUILD_SIMULATION_ONLY)
    set(LIB libnidaqmx.so libyaml-cpp.a m pthread)
else()
    set(LIB m libyaml-cpp.a pthread)
endif()


if(DEFINED PREFIX)
    message("PREFIX: ${PREFIX}")
    set(hardware_exe ${PREFIX}_hardware)
    set(simulation_exe ${PREFIX}_simulation)
else()
    set(hardware_exe ${PREFIX}_hardware)
    set(simulation_exe ${PREFIX}_simulation)
endif()

if (NOT DEFINED BUILD_HARDWARE_TEST)

    if (NOT DEFINED BUILD_SIMULATION_ONLY)
        add_executable(${hardware_exe} src/daq_wrapper.cpp ${SRC_DIR}/controlFunction.cpp utils/utils.cpp)
        target_include_directories(${hardware_exe} PUBLIC ${HEADER_DIRS} ${SRC_DIR})
        target_link_directories(${hardware_exe} PUBLIC ${LIB_DIRS})
        target_link_libraries(${hardware_exe} PUBLIC ${LIB})
    endif()

    add_executable(${simulation_exe} src/daq_sim.cpp ${SRC_DIR}/controlFunction.cpp utils/utils.cpp)
    target_include_directories(${simulation_exe} PUBLIC ${HEADER_DIRS} ${SRC_DIR})
    target_link_directories(${simulation_exe} PUBLIC ${LIB_DIRS})
    target_link_libraries(${simulation_exe} PUBLIC ${LIB})

else()

    add_executable(daq_sine src/daq_sine.cpp)
    target_include_directories(daq_sine PUBLIC ${HEADER_DIRS})
    target_link_directories(daq_sine PUBLIC ${LIB_DIRS})
    target_link_libraries(daq_sine PUBLIC ${LIB})
endif()