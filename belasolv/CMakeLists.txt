project("belasolv")

set(header_files
	"belasolv.h"
	"belasolvDlg.h"
	"femmedoccore.h"
	"mesh.h"
	"resource.h"
	"spars.h"
	"StdAfx.h"
)
source_group("Header Files" FILES ${header_files})

set(source_files
	"belasolv.cpp"
	"belasolvDlg.cpp"
	"cuthill.cpp"
	"femmedoccore.cpp"
	"main.cpp"
	"prob1big.cpp"
	"spars.cpp"
	"StdAfx.cpp"
)
source_group("Source Files" FILES ${source_files})

set(resource_files
	"belasolv.rc"
	"res/belasolv.ico"
	"res/belasolv.rc2"
)
source_group("Resource Files" FILES ${resource_files})

add_executable("belasolv"
	"${header_files}"
	"${source_files}"
	"${resource_files}"
)

target_compile_definitions("belasolv" PUBLIC "WIN32" "_WINDOWS")
set_target_properties("belasolv" PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")

set_source_files_properties("StdAfx.cpp" PROPERTIES COMPILE_FLAGS "/YcStdAfx.h")
target_compile_options("belasolv" PUBLIC "/YuStdAfx.h")
