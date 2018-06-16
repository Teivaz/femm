project("triangle")

set(header_files
	"resource.h"
	"triangle.h"
)
source_group("Header Files" FILES ${header_files})

set(source_files
	"triangle.c"
)
source_group("Source Files" FILES ${source_files})

set(resource_files
	"icon1.ico"
	"triangle.rc"
)
source_group("Resource Files" FILES ${resource_files})

add_executable("triangle" "${source_files}" "${header_files}" "${resource_files}")
