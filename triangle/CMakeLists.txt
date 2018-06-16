project("triangle")

set(header_files
	"resource.h"
	"triangle.h"
)

set(source_files
	"triangle.c"
)

set(resource_files
	"icon1.ico"
	"triangle.rc"
)

add_executable("triangle" "${source_files}" "${header_files}" "${resource_files}")
#target_link_libraries("triangle" kernel32.lib;user32.lib;gdi32.lib;winspool.lib;comdlg32.lib;advapi32.lib;shell32.lib;ole32.lib;oleaut32.lib;uuid.lib;odbc32.lib;odbccp32.lib)
