# ----------------------------------------#
# Hesai Tech <http://www.hesaitech.com>  #
# ----------------------------------------#

cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# set(CMAKE_CXX_STANDARD 17)
project(libhesai)
# if(POLICY CMP0074)
#     cmake_policy(SET CMP0074 NEW)
# else()
#     cmake_policy(SET CMP0065 NEW)
# endif()

# set(CMAKE_BUILD_TYPE ON)

# Add each module's entry
set(WITH_LIDARBOARD ON CACHE BOOL "Build lidarboard code")
set(WITH_UDPPARSER ON CACHE BOOL "Build udp parser code")
set(WITH_SERIALCLIENT ON CACHE BOOL "Build serial client code")
set(WITH_PTCCLIENT ON CACHE BOOL "Build ptcClient code")
if(WITH_LIDARBOARD AND WITH_UDPPARSER AND WITH_SERIALCLIENT AND WITH_PTCCLIENT)
    set(WITH_LIDARLIB ON CACHE BOOL "Build lidar code")
endif()

include_directories("${PROJECT_BINARY_DIR}")

if(WITH_PTCCLIENT)
    if(WITH_PTCS_USE)
        find_package(OpenSSL REQUIRED)
    endif(WITH_PTCS_USE)
endif(WITH_PTCCLIENT)

find_package(Threads REQUIRED)

include_directories(./)

# Make each module an 'OBJECT' library, can be refered to by get_link_libraries
if(WITH_UDPPARSER)
    add_library(udpParser_lib UdpParser/udp_parser.cc)
    target_link_libraries(udpParser_lib log_lib platutils_lib)
    target_include_directories(
        udpParser_lib
        PUBLIC UdpProtocol UdpParser/include UdpParser/src UdpParser ./
    )
endif(WITH_UDPPARSER)

if(WITH_PTCCLIENT)
    if(WITH_PTCS_USE)
        set(SSL_SOURCE PtcClient/src/tcp_ssl_client.cc)
    else()
        set(SSL_SOURCE "")
    endif()
    add_library(
        ptcClient_lib
        PtcClient/src/ptc_client.cc
        PtcClient/src/tcp_client.cc
        PtcParser/src/general_ptc_parser.cc
        PtcParser/src/ptc_1_0_parser.cc
        PtcParser/src/ptc_2_0_parser.cc
        PtcParser/ptc_parser.cc
        ${SSL_SOURCE}
    )
    target_link_libraries(ptcClient_lib platutils_lib)
    if(WITH_PTCS_USE)
        target_link_libraries(ptcClient_lib ${OPENSSL_LIBRARIES})
    endif(WITH_PTCS_USE)
    target_include_directories(
        ptcClient_lib
        PUBLIC PtcClient/include PtcParser/include PtcParser
    )
    if(WITH_PTCS_USE)
        target_include_directories(ptcClient_lib PUBLIC ${OPENSSL_INCLUDE_DIR})
        target_compile_definitions(ptcClient_lib PUBLIC WITH_PTCS_USE=1)
    endif(WITH_PTCS_USE)
endif(WITH_PTCCLIENT)

if(WITH_LIDARBOARD)
    add_library(
        source_lib
        Source/src/source.cc
        Source/src/socket_source.cc
        Source/src/pcap_source.cc
        Source/src/serial_source.cc
        Source/src/pcap_saver.cc
    )
    target_link_libraries(source_lib platutils_lib)
    target_include_directories(
        source_lib
        PUBLIC Source/include Container/include Container/src
    )

    if(WITH_SERIALCLIENT)
        add_library(serialClient_lib SerialClient/src/serial_client.cc)
        target_link_libraries(serialClient_lib source_lib)
        target_include_directories(serialClient_lib PUBLIC SerialClient/include)
    endif(WITH_SERIALCLIENT)
endif(WITH_LIDARBOARD)

# platutils lib
add_library(
    platutils_lib
    Common/src/plat_utils.cc
    Common/src/auto_tick_count.cc
)
target_link_libraries(platutils_lib log_lib)
target_include_directories(platutils_lib PUBLIC Common/include)
# log lib
add_library(log_lib SHARED Logger/src/logger.cc)
target_link_libraries(log_lib ${CMAKE_THREAD_LIBS_INIT})
target_include_directories(log_lib PUBLIC Logger/include)
target_compile_definitions(log_lib PRIVATE LOG_LIB_API_EXPORTS)

if(WITH_LIDARLIB)
    add_library(lidar_lib Lidar/lidar.cc)
    target_link_libraries(
        lidar_lib
        log_lib
        platutils_lib
        udpParser_lib
        ptcClient_lib
        source_lib
        serialClient_lib
        ${CMAKE_THREAD_LIBS_INIT}
    )
    if(WITH_UDPPARSERGPU)
        target_link_libraries(lidar_lib udpParserGpu_lib)
    endif(WITH_UDPPARSERGPU)
    target_include_directories(lidar_lib PUBLIC Lidar ./)
endif(WITH_LIDARLIB)
