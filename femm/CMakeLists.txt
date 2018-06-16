project("femm")

# beladraw
set(beladraw_header
	"bd_BdryDlg.h"
	"bd_CircProp.h"
	"bd_libdlg.h"
	"bd_MatDlg.h"
	"bd_NodeProp.h"
	"bd_nosebl.h"
	"bd_OpArcSegDlg.h"
	"bd_OpBlkDlg.h"
	"bd_OpNodeDlg.h"
	"bd_OpSegDlg.h"
	"bd_Pref.h"
	"bd_probdlg.h"
	"bd_PtProp.h"
	"beladrawDoc.h"
	"beladrawView.h"
)
source_group("beladraw\\beladraw header" FILES ${beladraw_header})
set(beladraw_source
	"bd_BdryDlg.cpp"
	"bd_CircProp.cpp"
	"bd_libdlg.cpp"
	"bd_MatDlg.cpp"
	"bd_movecopy.cpp"
	"bd_NodeProp.cpp"
	"bd_nosebl.cpp"
	"bd_OpArcSegDlg.cpp"
	"bd_OpBlkDlg.cpp"
	"bd_OpNodeDlg.cpp"
	"bd_OpSegDlg.cpp"
	"bd_Pref.cpp"
	"bd_probdlg.cpp"
	"bd_PtProp.cpp"
	"bd_writepoly.cpp"
	"beladrawDoc.cpp"
	"beladrawLua.cpp"
	"beladrawView.cpp"
)
source_group("beladraw\\beladraw source" FILES ${beladraw_source})

# belaview
set(belaview_header
	"bv_BlockInt.h"
	"bv_CircDlg.h"
	"bv_CplotDlg2.h"
	"bv_DPlotDlg2.h"
	"bv_LIntDlg.h"
	"bv_Pref.h"
	"bv_problem.h"
	"bv_VPlotDlg.h"
	"bv_XYPlotDlg.h"
	"belaviewDoc.h"
	"belaviewView.h"
)
source_group("belaview\\belaview header" FILES ${belaview_header})
set(belaview_source
	"bv_BlockInt.cpp"
	"bv_CircDlg.cpp"
	"bv_CplotDlg2.cpp"
	"bv_DPlotDlg2.cpp"
	"bv_LIntDlg.cpp"
	"bv_makemask.cpp"
	"bv_Pref.cpp"
	"bv_problem.cpp"
	"bv_VPlotDlg.cpp"
	"bv_XYPlotDlg.cpp"
	"belaviewDoc.cpp"
	"belaviewLua.cpp"
	"belaviewView.cpp"
)
source_group("belaview\\belaview source" FILES ${belaview_source})

# cdraw
set(cdraw_header
	"CDRAWDOC.H"
	"cdrawView.h"
	"cd_BdryDlg.h"
	"cd_CircProp.h"
	"cd_libdlg.h"
	"cd_MatDlg.h"
	"cd_NodeProp.h"
	"cd_nosebl.h"
	"cd_OpArcSegDlg.h"
	"cd_OpBlkDlg.h"
	"cd_OpNodeDlg.h"
	"cd_OpSegDlg.h"
	"CD_PREF.H"
	"cd_probdlg.h"
	"cd_PtProp.h"
)
source_group("cdraw\\cdraw header" FILES ${cdraw_header})
set(cdraw_source
	"CDRAWDOC.CPP"
	"CDRAWLUA.CPP"
	"cdrawView.cpp"
	"cd_BdryDlg.cpp"
	"cd_CircProp.cpp"
	"cd_libdlg.cpp"
	"cd_MatDlg.cpp"
	"cd_movecopy.cpp"
	"cd_NodeProp.cpp"
	"cd_nosebl.cpp"
	"cd_OpArcSegDlg.cpp"
	"cd_OpBlkDlg.cpp"
	"cd_OpNodeDlg.cpp"
	"cd_OpSegDlg.cpp"
	"CD_PREF.CPP"
	"cd_probdlg.cpp"
	"cd_PtProp.cpp"
	"cd_writepoly.cpp"
)
source_group("cdraw\\cdraw source" FILES ${cdraw_source})

# common
set(common_header
	"ArcDlg.h"
	"bhplot.h"
	"EnterPt.h"
	"ExteriorProps.h"
	"fullmatrix.h"
	"GeneralPrefs.h"
	"GRIDDLG.H"
	"KbdZoom.h"
	"LuaConsoleDlg.h"
	"luaDDX.h"
	"LuaEdit.h"
	"MyTabCtrl.h"
	"OutBox.h"
	"PromptBox.h"
	"ResizableDialog.h"
	"ResizableGrip.h"
	"ResizableLayout.h"
	"ResizableMinMax.h"
	"ResizableState.h"
	"spars.h"
	"Xyplot.h"
)
source_group("common\\Common Header" FILES ${common_header})
set(common_source
	"ArcDlg.cpp"
	"bhplot.cpp"
	"EnterPt.cpp"
	"ExteriorProps.cpp"
	"fullmatrix.cpp"
	"GeneralPrefs.cpp"
	"GRIDDLG.CPP"
	"KbdZoom.cpp"
	"LuaConsoleDlg.cpp"
	"luaDDX.cpp"
	"LuaEdit.cpp"
	"MyTabCtrl.cpp"
	"OutBox.cpp"
	"PromptBox.cpp"
	"spars.cpp"
	"Xyplot.cpp"
)
source_group("common\\Common Source" FILES ${common_source})

# cview
set(cview_header
	"CVIEWDOC.H"
	"cviewView.h"
	"cv_BlockInt.h"
	"cv_CircDlg.h"
	"cv_CplotDlg2.h"
	"cv_DPlotDlg2.h"
	"cv_LIntDlg.h"
	"CV_PREF.H"
	"cv_problem.h"
	"cv_VPlotDlg.h"
	"cv_XYPlotDlg.h"
)
source_group("cview\\cview header" FILES ${cview_header})
set(cview_source
	"CVIEWDOC.CPP"
	"CVIEWLUA.CPP"
	"cviewView.cpp"
	"cv_BlockInt.cpp"
	"cv_CircDlg.cpp"
	"cv_CplotDlg2.cpp"
	"cv_DPlotDlg2.cpp"
	"cv_LIntDlg.cpp"
	"cv_makemask.cpp"
	"CV_PREF.CPP"
	"cv_problem.cpp"
	"cv_VPlotDlg.cpp"
	"cv_XYPlotDlg.cpp"
)
source_group("cview\\cview source" FILES ${cview_source})

# femme
set(femme_header
	"BdryDlg.h"
	"BHData.h"
	"BHDatafile.h"
	"CircProp.h"
	"CopyDlg.h"
	"fe_libdlg.h"
	"FemmeDoc.h"
	"FemmeView.h"
	"GroupNumber.h"
	"MatDlg.h"
	"MirrorDlg.h"
	"NodeProp.h"
	"NOSEBL.H"
	"OpArcSegDlg.h"
	"OpBlkDlg.h"
	"OpGrp.h"
	"OpNodeDlg.h"
	"OpSegDlg.h"
	"Pref.h"
	"probdlg.h"
	"PtProp.h"
	"ScaleDlg.h"
)
source_group("femme\\femme header" FILES ${femme_header})
set(femme_source
	"BdryDlg.cpp"
	"BHData.cpp"
	"BHDatafile.cpp"
	"CircProp.cpp"
	"CopyDlg.cpp"
	"fe_libdlg.cpp"
	"FemmeDoc.cpp"
	"femmeLua.cpp"
	"FemmeView.cpp"
	"GroupNumber.cpp"
	"MatDlg.cpp"
	"MirrorDlg.cpp"
	"MOVECOPY.CPP"
	"NodeProp.cpp"
	"NOSEBL.CPP"
	"OpArcSegDlg.cpp"
	"OpBlkDlg.cpp"
	"OpGrp.cpp"
	"OpNodeDlg.cpp"
	"OpSegDlg.cpp"
	"Pref.cpp"
	"probdlg.cpp"
	"PtProp.cpp"
	"ScaleDlg.cpp"
	"writepoly.cpp"
)
source_group("femme\\femme source" FILES ${femme_source})

# femmplot
set(femmplot_header
	"femmplotDoc.h"
	"femmplotView.h"
)
source_group("femmplot\\femmplot header" FILES ${femmplot_header})
set(femmplot_source
	"femmplotDoc.cpp"
	"femmplotView.cpp"
)
source_group("femmplot\\femmplot source" FILES ${femmplot_source})

# femmview
set(femmview_header
	"BendContourDlg.h"
	"BlockInt.h"
	"CircDlg.h"
	"CPlotDlg.h"
	"CplotDlg2.h"
	"FemmviewDoc.h"
	"FemmviewView.h"
	"GridMod.h"
	"LIntDlg.h"
	"MaskProgress.h"
	"MyMsgBox.h"
	"Problem.h"
	"viewpref.h"
	"VPlotDlg.h"
	"XYPlotDlg.h"
)
source_group("femmview\\femmview header" FILES ${femmview_header})
set(femmview_source
	"BendContourDlg.cpp"
	"BlockInt.cpp"
	"CircDlg.cpp"
	"CPlotDlg.cpp"
	"CplotDlg2.cpp"
	"FemmviewDoc.cpp"
	"femmviewLua.cpp"
	"FemmviewView.cpp"
	"GridMod.cpp"
	"LIntDlg.cpp"
	"makemask.cpp"
	"MaskProgress.cpp"
	"MyMsgBox.cpp"
	"Problem.cpp"
	"viewpref.cpp"
	"VPlotDlg.cpp"
	"XYPlotDlg.cpp"
)
source_group("femmview\\femmview source" FILES ${femmview_source})

# hdraw
set(hdraw_header
	"HDRAWDOC.H"
	"hdrawView.h"
	"hd_BdryDlg.h"
	"hd_CircProp.h"
	"hd_libdlg.h"
	"hd_MatDlg.h"
	"hd_NodeProp.h"
	"hd_nosebl.h"
	"hd_OpArcSegDlg.h"
	"hd_OpBlkDlg.h"
	"hd_OpNodeDlg.h"
	"hd_OpSegDlg.h"
	"HD_PREF.H"
	"hd_probdlg.h"
	"hd_PtProp.h"
	"KCurve.h"
	"TKDatafile.h"
)
source_group("hdraw\\hdraw header" FILES ${hdraw_header})
set(hdraw_source
	"HDRAWDOC.CPP"
	"HDRAWLUA.CPP"
	"hdrawView.cpp"
	"hd_BdryDlg.cpp"
	"hd_CircProp.cpp"
	"hd_libdlg.cpp"
	"hd_MatDlg.cpp"
	"hd_movecopy.cpp"
	"hd_NodeProp.cpp"
	"hd_nosebl.cpp"
	"hd_OpArcSegDlg.cpp"
	"hd_OpBlkDlg.cpp"
	"hd_OpNodeDlg.cpp"
	"hd_OpSegDlg.cpp"
	"HD_PREF.CPP"
	"hd_probdlg.cpp"
	"hd_PtProp.cpp"
	"hd_writepoly.cpp"
	"KCurve.cpp"
	"TKDatafile.cpp"
)
source_group("hdraw\\hdraw source" FILES ${hdraw_source})

# hview
set(hview_header
	"hviewDoc.h"
	"hviewView.h"
	"hv_BlockInt.h"
	"hv_CircDlg.h"
	"hv_CplotDlg2.h"
	"hv_DPlotDlg2.h"
	"hv_LIntDlg.h"
	"HV_PREF.H"
	"hv_problem.h"
	"hv_VPlotDlg.h"
	"hv_XYPlotDlg.h"
)
source_group("hview\\hview header" FILES ${hview_header})
set(hview_source
	"hviewDoc.cpp"
	"hviewLua.cpp"
	"hviewView.cpp"
	"hv_BlockInt.cpp"
	"hv_CircDlg.cpp"
	"hv_CplotDlg2.cpp"
	"hv_DPlotDlg2.cpp"
	"hv_LIntDlg.cpp"
	"HV_PREF.CPP"
	"hv_problem.cpp"
	"hv_VPlotDlg.cpp"
	"hv_XYPlotDlg.cpp"
)
source_group("hview\\hview source" FILES ${hview_source})

# MDI_Shell
set(MDI_Shell_header
	"ActiveFEMM.h"
	"ChildFrm.h"
	"COMPLEX.H"
	"DXFImport.h"
	"femm.h"
	"GapIntegral.h"
	"GapPlotDlg.h"
	"LibFolderInfo.h"
	"MainFrm.h"
	"MakeABCDlg.h"
	"MDITabs.h"
	"MyCommandLineInfo.h"
	"MyRecentFileList.h"
	"NewDocDlg.h"
	"Preferences.h"
	"resource.h"
	"StdAfx.h"
)
source_group("MDI Shell\\Header Files" FILES ${MDI_Shell_header})
set(MDI_Shell_source
	"ActiveFEMM.cpp"
	"ChildFrm.cpp"
	"DXFImport.cpp"
	"femm.cpp"
	"femm.odl"
	"femm.rc"
	"GapIntegral.cpp"
	"GapPlotDlg.cpp"
	"LibFolderInfo.cpp"
	"MainFrm.cpp"
	"MakeABCDlg.cpp"
	"MDITabs.cpp"
	"MyCommandLineInfo.cpp"
	"MyRecentFileList.cpp"
	"NewDocDlg.cpp"
	"Preferences.cpp"
	"StdAfx.cpp"
)
source_group("MDI Shell\\Source Files" FILES ${MDI_Shell_source} ${precompiled_header_source})
set(MDI_Shell_resource
	"res/bitmap1.bmp"
	"res/bmp00001.bmp"
	"res/bmp00002.bmp"
	"res/bmp00003.bmp"
	"res/bmp00004.bmp"
	"res/bmp00005.bmp"
	"res/bmp00006.bmp"
	"res/bmp00007.bmp"
	"res/bmp00013.bmp"
	"res/femm.rc2"
	"res/femmetyp.bmp"
	"res/femmtype.bmp"
	"res/femmview.bmp"
	"res/hdrawtyp.bmp"
	"res/hviewtyp.bmp"
	"res/idr_beladrawtype.ico"
	"res/idr_belaviewtype.ico"
	"res/idr_cdrawtype.ico"
	"res/idr_cviewtype.ico"
	"res/idr_femmetype.ico"
	"res/idr_femmplottype.ico"
	"res/idr_femmviewtype.ico"
	"res/idr_hdrawtype.ico"
	"res/idr_hviewtype.ico"
	"res/idr_mainframe.ico"
	"res/left1.bmp"
	"res/left2.bmp"
	"res/main1.bmp"
	"res/main2.bmp"
	"res/Toolbar.bmp"
	"res/toolbar1.bmp"
	"res/tree.bmp"
	"res/viewtool.bmp"
)
source_group("MDI Shell\\Resource Files" FILES ${MDI_Shell_resource})


set(unused_files
	"backup_resource.h"
	"femmDoc.cpp"
	"femmDoc.h"
	"LUA.H"
	"luadebug.h"
	"mathlink.h"
	"res/cdrawtyp.bmp"
	"res/cviewtyp.bmp"
	"ResizableFormView.h"
	"ResizableFrame.h"
	"ResizableMDIChild.h"
	"ResizableMDIFrame.h"
	"ResizableMsgSupport.h"
	"ResizablePage.h"
	"ResizablePageEx.h"
	"ResizableSheet.h"
	"ResizableSheetEx.h"
)

set(other_files
	"res/femm.tlb"
)

add_executable("femm"
	"${beladraw_header}"
	"${beladraw_source}"
	"${belaview_header}"
	"${belaview_source}"
	"${cdraw_header}"
	"${cdraw_source}"
	"${common_header}"
	"${common_source}"
	"${cview_header}"
	"${cview_source}"
	"${femme_header}"
	"${femme_source}"
	"${femmplot_header}"
	"${femmplot_source}"
	"${femmview_header}"
	"${femmview_source}"
	"${hdraw_header}"
	"${hdraw_source}"
	"${hview_header}"
	"${hview_source}"
	"${MDI_Shell_header}"
	"${MDI_Shell_source}"
)

target_compile_definitions("femm" PUBLIC "WIN32" "_WINDOWS")

set_target_properties("femm" PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")
target_link_libraries("femm" "liblua" "ResizableLib")

set_source_files_properties("StdAfx.cpp" PROPERTIES COMPILE_FLAGS "/YcStdAfx.h")
target_compile_options("femm" PUBLIC "/YuStdAfx.h")
