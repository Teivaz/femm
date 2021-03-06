// CplotDlg2.cpp : implementation file
//

#include "stdafx.h"
#include "femm.h"
#include "hv_CplotDlg2.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// hvCCPlotDlg2 dialog


hvCCPlotDlg2::hvCCPlotDlg2(CWnd* pParent /*=NULL*/)
	: CDialog(hvCCPlotDlg2::IDD, pParent)
{
	//{{AFX_DATA_INIT(hvCCPlotDlg2)
	m_numcontours = 0;
	m_showa = FALSE;
	m_ahigh = 0.0;
	m_alow = 0.0;
	m_showmask = FALSE;
	//}}AFX_DATA_INIT
}


void hvCCPlotDlg2::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(hvCCPlotDlg2)
	DDX_Text(pDX, IDC_HV_NUMCONTOURS, m_numcontours);
	DDV_MinMaxInt(pDX, m_numcontours, 4, 999);
	DDX_Check(pDX, IDC_HV_SHOW_A, m_showa);
	DDX_Text(pDX, IDC_HV_AHIGH, m_ahigh);
	DDX_Text(pDX, IDC_HV_ALOW, m_alow);
	DDX_Check(pDX, IDC_HV_SHOW_MASK2, m_showmask);
	//}}AFX_DATA_MAP
	DDX_Control(pDX, IDC_HV_NUMCONTOURS, m_IDC_numcontours);
	DDX_Control(pDX, IDC_HV_AHIGH, m_IDC_ahigh);
	DDX_Control(pDX, IDC_HV_ALOW, m_IDC_alow);
}


BEGIN_MESSAGE_MAP(hvCCPlotDlg2, CDialog)
	//{{AFX_MSG_MAP(hvCCPlotDlg2)
	ON_BN_CLICKED(IDC_HV_DFLT1, OnDflt1)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// hvCCPlotDlg2 message handlers

void hvCCPlotDlg2::OnDflt1() 
{
	// TODO: Add your control notification handler code here
	UpdateData(TRUE);
	m_alow=Alb;
	m_ahigh=Aub;
	m_numcontours=19;
	UpdateData(FALSE);
}
