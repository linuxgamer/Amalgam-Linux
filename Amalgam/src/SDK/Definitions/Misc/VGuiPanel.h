#pragma once
#include "VGUI.h"

// Minimal vgui2 Panel / VPanel wrappers used by the Model Preview feature.

class Panel;

class Panel
{
public:
	// IClientPanel::GetVPanel is the first virtual of a constructed vgui::Panel (vtable index 0).
	virtual VPANEL GetVPanel() const = 0;
};

class VPanel
{
public:
	VPanel();
	virtual ~VPanel();

	virtual void Init(Panel* attachedClientPanel);

	virtual void* Plat();
	virtual void  SetPlat(void* pl);

	virtual int  GetHPanel() { return _hPanel; }
	virtual void SetHPanel(int hPanel) { _hPanel = hPanel; }

	virtual bool IsPopup();
	virtual void SetPopup(bool state);
	virtual bool IsFullyVisible();

	virtual void SetPos(int x, int y);
	virtual void GetPos(int& x, int& y);
	virtual void SetSize(int wide, int tall);
	virtual void GetSize(int& wide, int& tall);
	virtual void SetMinimumSize(int wide, int tall);
	virtual void GetMinimumSize(int& wide, int& tall);
	virtual void SetZPos(int z);
	virtual int  GetZPos();

	virtual void GetAbsPos(int& x, int& y);
	virtual void GetClipRect(int& x0, int& y0, int& x1, int& y1);
	virtual void SetInset(int left, int top, int right, int bottom);
	virtual void GetInset(int& left, int& top, int& right, int& bottom);

	virtual void Solve();

	virtual void SetVisible(bool state);
	virtual void SetEnabled(bool state);
	virtual bool IsVisible();
	virtual bool IsEnabled();
	virtual void SetParent(VPanel* newParent);
	virtual int  GetChildCount();
	virtual VPanel* GetChild(int index);
	virtual VPanel* GetParent();
	virtual void MoveToFront();
	virtual void MoveToBack();
	virtual bool HasParent(VPanel* potentialParent);

	virtual void* GetChildren();

	virtual const char* GetName();
	virtual const char* GetClassName();

	virtual int GetScheme();

	virtual void SendMessage(void* params, VPANEL ifromPanel);

	// The one we actually call: returns the companion client Panel*.
	virtual Panel* Client() const = 0;

public:
	char m_padding[20];
	VPanel* _parent;
	void*   _plat;
	HPanel  _hPanel;
	Panel*  _clientPanel;
};
