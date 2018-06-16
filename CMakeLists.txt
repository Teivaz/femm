cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)

project(femm42)

set(CMAKE_MFC_FLAG 2)
add_definitions(-D_AFXDLL)

add_subdirectory(femmplot)
add_subdirectory(liblua)
add_subdirectory(ResizableLib)
add_subdirectory(femm)


