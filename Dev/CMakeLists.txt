# Authors: Mohamed Elobaid	

cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

project(pioneer_p3dx_SDLTFL)


find_package(Threads REQUIRED)


set(EXE_TARGET_NAME pioneer_SDTFL)

# set cpp files
set(${EXE_TARGET_NAME}_SRC
  src/main.cpp
  src/pioneer_p3dx.cpp
  src/extApi.c
  src/extApiPlatform.c)

# set hpp files
set(${EXE_TARGET_NAME}_HDR
  include/pioneer_p3dx.h
  include/extApi.h
  include/extApiPlatform.h)

# add include directories to the build.
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  $ENV{REMOTE_API}/include
  $ENV{REMOTE_API}/remoteApi
  $ENV{EIGEN}
  $ENV{inih}/include/inih
)


add_definitions (-DNON_MATLAB_PARSING)
add_definitions (-DMAX_EXT_API_CONNECTIONS=255)
add_definitions (-DDO_NOT_USE_SHARED_MEMORY)

# add an executable to the project using the specified source files.
add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC} ${${EXE_TARGET_NAME}_HDR})

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

target_link_libraries(${EXE_TARGET_NAME} Threads::Threads)
if (UNIX)
target_link_libraries(${EXE_TARGET_NAME} $ENV{inih}/lib/libinihcpp.a)
install(TARGETS ${EXE_TARGET_NAME} DESTINATION ${CMAKE_BUILD_TYPE}/bin)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/App/conf.ini DESTINATION ${CMAKE_BUILD_TYPE}/bin)    
endif (UNIX)

if (MSVC)
target_link_libraries(${EXE_TARGET_NAME} $ENV{inih}/lib/inihcpp.lib)  
install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/App/conf.ini DESTINATION bin)  
endif (MSVC)


