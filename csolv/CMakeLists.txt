project("csolv")

set(header_files
	"complex.h"
	"CSOLV.H"
	"CSOLVDLG.H"
	"femmedoccore.h"
	"MESH.H"
	"RESOURCE.H"
	"SPARS.H"
	"StdAfx.h"
)
source_group("Header Files" FILES ${header_files})

set(source_files
	"complex.cpp"
	"csolv.cpp"
	"CSOLVDLG.CPP"
	"cspars.cpp"
	"CUTHILL.CPP"
	"femmedoccore.cpp"
	"MAIN.CPP"
	"PROB1BIG.CPP"
	"STDAFX.CPP"
)
source_group("Source Files" FILES ${source_files})

set(resource_files
	"CSOLV.RC"
	"res/csolv.ico"
	"res/csolv.rc2"
)
source_group("Resource Files" FILES ${resource_files})

add_executable("csolv"
	"${header_files}"
	"${source_files}"
	"${resource_files}"
)

target_compile_definitions("csolv" PUBLIC "WIN32" "_WINDOWS")
set_target_properties("csolv" PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")

set_source_files_properties("StdAfx.cpp" PROPERTIES COMPILE_FLAGS "/YcStdAfx.h")
target_compile_options("csolv" PUBLIC "/YuStdAfx.h")
