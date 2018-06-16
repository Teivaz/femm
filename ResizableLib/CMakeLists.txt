project("ResizableLib")

set(source_files
	"ResizableDialog.cpp"
	"ResizableFormView.cpp"
	"ResizableFrame.cpp"
	"ResizableGrip.cpp"
	"ResizableLayout.cpp"
	"ResizableLib.dsp"
	"ResizableLib.dsw"
	"ResizableLib.vcproj"
	"ResizableLib.vcxproj"
	"ResizableLib.vcxproj.filters"
	"ResizableMDIChild.cpp"
	"ResizableMDIFrame.cpp"
	"ResizableMinMax.cpp"
	"ResizablePage.cpp"
	"ResizablePageEx.cpp"
	"ResizableSheet.cpp"
	"ResizableSheetEx.cpp"
	"ResizableState.cpp"
	"StdAfx.cpp"
)
set(header_files
	"ResizableMsgSupport.inl"
	"ResizableDialog.h"
	"ResizableFormView.h"
	"ResizableFrame.h"
	"ResizableGrip.h"
	"ResizableLayout.h"
	"ResizableMDIChild.h"
	"ResizableMDIFrame.h"
	"ResizableMinMax.h"
	"ResizableMsgSupport.h"
	"ResizablePage.h"
	"ResizablePageEx.h"
	"ResizableSheet.h"
	"ResizableSheetEx.h"
	"ResizableState.h"
	"StdAfx.h"
)

add_library("ResizableLib" "${source_files}" "${header_files}")

target_compile_definitions("ResizableLib" PUBLIC "WIN32" "_WINDOWS")

set_source_files_properties("StdAfx.cpp" PROPERTIES COMPILE_FLAGS "/YcStdAfx.h")
target_compile_options("ResizableLib" PUBLIC "/YuStdAfx.h")
