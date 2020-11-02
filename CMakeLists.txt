cmake_minimum_required(VERSION 3.10)

# project name is set and the version
project(mp3Encoder VERSION 1.0)


# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)


# this allows us to pass the verison number to the source code
configure_file(basicConfig.h.in config.h)

# the executable file
# Note that the source code for main.cxx is provided in the same directory the file is
# add_executable(mp3Encoder ${CMAKE_SOURCE_DIR}/src/main.cpp)

#set - creates variables

#compile jpegCodec implementation 
set(var_jpeg_codec_obj
${CMAKE_SOURCE_DIR}/src/jpegCodec/jpeg_dct.cpp
${CMAKE_SOURCE_DIR}/src/jpegCodec/jpeg_headers.cpp
${CMAKE_SOURCE_DIR}/src/jpegCodec/jpeg_main.cpp
) 
add_library(var_jpeg_codec_lib STATIC ${var_jpeg_codec_obj}) #changed OBJECT to STATIC for docker

set(var_main_compile ${CMAKE_SOURCE_DIR}/src/main.cpp  ${CMAKE_SOURCE_DIR}/src/stb_image_implementation
${CMAKE_SOURCE_DIR}/include/codec.h  
${CMAKE_SOURCE_DIR}/include/jpegCodec.h  
${CMAKE_SOURCE_DIR}/include/stb_image.h  
${CMAKE_SOURCE_DIR}/include/stb_image_write.h  
)

add_executable(jpeg_encoder ${var_main_compile})

# since the configure_file of basic config will be written in the binary tree, 
# we must add that directory to the list of paths to search for include files.
target_include_directories(jpeg_encoder PUBLIC
                           "${PROJECT_BINARY_DIR}"
                           )

# for the pthread library we need to add it
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)


target_link_libraries(jpeg_encoder PRIVATE Threads::Threads var_jpeg_codec_lib)


target_include_directories(jpeg_encoder PUBLIC "${CMAKE_SOURCE_DIR}/include")  #our -I$PWD/include




