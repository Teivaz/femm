project("hsolv")

set(header_files
	"complex.h"
	"HSOLV.H"
	"hsolvDlg.h"
	"hsolvdoc.h"
	"MESH.H"
	"RESOURCE.H"
	"SPARS.H"
	"StdAfx.h"
)
source_group("Header Files" FILES ${header_files})

set(source_files
	"complex.cpp"
	"CUTHILL.CPP"
	"hsolv.cpp"
	"hsolvDlg.cpp"
	"hsolvdoc.cpp"
	"MAIN.CPP"
	"prob1big.cpp"
	"SPARS.CPP"
	"STDAFX.CPP"
)
source_group("Source Files" FILES ${source_files})

set(resource_files
	"hsolv.rc"
	"res/hsolv.ico"
	"res/hsolv.rc2"
)
source_group("Resource Files" FILES ${resource_files})

add_executable("hsolv"
	"${header_files}"
	"${source_files}"
	"${resource_files}"
)

target_compile_definitions("hsolv" PUBLIC "WIN32" "_WINDOWS")
set_target_properties("hsolv" PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")

set_source_files_properties("StdAfx.cpp" PROPERTIES COMPILE_FLAGS "/YcStdAfx.h")
target_compile_options("hsolv" PUBLIC "/YuStdAfx.h")
