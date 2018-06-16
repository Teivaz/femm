project("femmplot")

set(header_files
	"femmplot.h"
	"femmplotDoc.h"
	"femmplotView.h"
	"MainFrm.h"
	"resource.h"
	"StdAfx.h"
)
source_group("Header Files" FILES ${header_files})

set(source_files
	"femmplot.cpp"
	"femmplotDoc.cpp"
	"femmplotView.cpp"
	"MainFrm.cpp"
	"StdAfx.cpp"
)
source_group("Source Files" FILES ${source_files})

set(resource_files
	"femmplot.rc"
	"res/femmplot.ico"
	"res/femmplot.rc2"
	"res/femmplotDoc.ico"
)
source_group("Resource Files" FILES ${resource_files})

add_executable("femmplot"
	"${header_files}"
	"${source_files}"
	"${resource_files}"
)

target_compile_definitions("femmplot" PUBLIC "WIN32" "_WINDOWS")
set_target_properties("femmplot" PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")
