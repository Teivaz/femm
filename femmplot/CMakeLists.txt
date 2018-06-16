project("femmplot")

set(header_files
	"femmplot.h"
	"femmplotDoc.h"
	"femmplotView.h"
	"MainFrm.h"
	"resource.h"
	"StdAfx.h"
)

set(source_files
	"femmplot.cpp"
	"femmplotDoc.cpp"
	"femmplotView.cpp"
	"MainFrm.cpp"
	"StdAfx.cpp"
)

set(resource_files
	"femmplot.rc"
	"res/femmplot.ico"
	"res/femmplot.rc2"
	"res/femmplotDoc.ico"
)

add_executable("femmplot"
	"${header_files}"
	"${source_files}"
	"${resource_files}"
)

target_compile_definitions("femmplot" PUBLIC "WIN32" "_WINDOWS")
