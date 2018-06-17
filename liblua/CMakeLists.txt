project("liblua")

set(header_files
	"COMPLEX.H"
	"lapi.h"
	"lauxlib.h"
	"lcode.h"
	"ldebug.h"
	"ldo.h"
	"lfunc.h"
	"lgc.h"
	"llex.h"
	"llimits.h"
	"lmem.h"
	"lobject.h"
	"lopcodes.h"
	"lparser.h"
	"lstate.h"
	"lstring.h"
	"ltable.h"
	"ltm.h"
	"lua.h"
	"luadebug.h"
	"lualib.h"
	"lundump.h"
	"lvm.h"
	"lzio.h"
)
source_group("Header Files" FILES ${header_files})

set(source_files
	"COMPLEX.CPP"
	"lapi.cpp"
	"lauxlib.cpp"
	"lbaselib.cpp"
	"lcode.cpp"
	"ldblib.cpp"
	"ldebug.cpp"
	"ldo.cpp"
	"lfunc.cpp"
	"lgc.cpp"
	"liolib.cpp"
	"llex.cpp"
	"lmathlib.cpp"
	"lmem.cpp"
	"lobject.cpp"
	"lparser.cpp"
	"lstate.cpp"
	"lstring.cpp"
	"lstrlib.cpp"
	"ltable.cpp"
	"ltests.cpp"
	"ltm.cpp"
	"lundump.cpp"
	"lvm.cpp"
	"lzio.cpp"
)
source_group("Source Files" FILES ${source_files})

add_library("liblua" "${source_files}" "${header_files}")

target_include_directories("liblua" PUBLIC "${CMAKE_CURRENT_LIST_DIR}")
