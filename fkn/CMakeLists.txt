project("fkn")

set(header_files
	"femmedoccore.h"
	"fkn.h"
	"fknDlg.h"
	"fullmatrix.h"
	"mesh.h"
	"resource.h"
	"spars.h"
	"StdAfx.h"
)
source_group("Header Files" FILES ${header_files})

set(source_files
	"cspars.cpp"
	"cuthill.cpp"
	"femmedoccore.cpp"
	"fkn.cpp"
	"fknDlg.cpp"
	"fullmatrix.cpp"
	"main.cpp"
	"matprop.cpp"
	"prob1big.cpp"
	"prob2big.cpp"
	"prob3big.cpp"
	"prob4big.cpp"
	"spars.cpp"
	"StdAfx.cpp"
)
source_group("Source Files" FILES ${source_files})

set(resource_files
	"fkn.rc"
	"res/fkn.ico"
	"res/fkn.rc2"
)
source_group("Resource Files" FILES ${resource_files})

add_executable("fkn"
	"${header_files}"
	"${source_files}"
	"${resource_files}"
)

target_compile_definitions("fkn" PUBLIC "WIN32" "_WINDOWS")
set_target_properties("fkn" PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")
target_link_libraries("fkn" "liblua")
set_source_files_properties("StdAfx.cpp" PROPERTIES COMPILE_FLAGS "/YcStdAfx.h")
target_compile_options("fkn" PUBLIC "/YuStdAfx.h")
