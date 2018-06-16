cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)

project(femm42)

set(CMAKE_MFC_FLAG 2)
add_definitions(-D_AFXDLL)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_LIST_DIR}/debug")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_LIST_DIR}/release")

add_subdirectory(femmplot)
add_subdirectory(liblua)
add_subdirectory(ResizableLib)
add_subdirectory(femm)
add_subdirectory(triangle)

