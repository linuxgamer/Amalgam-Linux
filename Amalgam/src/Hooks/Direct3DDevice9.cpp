#include "Direct3DDevice9.h"

#include "../SDK/SDK.h"
#include "../Features/ImGui/Render.h"
#include "../Features/ImGui/Menu/Menu.h"
#include "../Features/Misc/Misc.h"

// #define USE_CUSTOM_GUI // Disabled - using original menu

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

// Helper to check if any menu is open. The pixel-surf-assist per-point pop-up (ENTER on a point)
// counts too: it needs the same cursor unlock + ImGui mouse routing the main menu gets.
inline bool IsAnyMenuOpen()
{
#ifdef USE_CUSTOM_GUI
	return F::CustomMenu.m_bIsOpen || F::Misc.AssistPointMenuOpen();
#else
	return F::Menu.m_bIsOpen || F::Misc.AssistPointMenuOpen();
#endif
}

MAKE_HOOK(Direct3DDevice9_Present, U::Memory.GetVirtual(I::DirectXDevice, 17), HRESULT,
	IDirect3DDevice9* pDevice, const RECT* pSource, const RECT* pDestination, const RGNDATA* pDirtyRegion)
{
	if (!G::Unload)
		F::Render.Render(pDevice);

	return CALL_ORIGINAL(pDevice, pSource, pDestination, pDirtyRegion);
}

MAKE_HOOK(Direct3DDevice9_Reset, U::Memory.GetVirtual(I::DirectXDevice, 16), HRESULT,
	LPDIRECT3DDEVICE9 pDevice, D3DPRESENT_PARAMETERS* pPresentationParameters)
{
	// Release POOL_DEFAULT resources before reset
	F::Render.ReleaseLogoSvgs();
	F::Render.ReleaseSeparatorSvgs();
	F::Render.ReleaseAvatarTextures(); // avatars are DEFAULT pool now; lazily recreated after reset
	ImGui_ImplDX9_InvalidateDeviceObjects();
	const HRESULT Original = CALL_ORIGINAL(pDevice, pPresentationParameters);
	ImGui_ImplDX9_CreateDeviceObjects();
	// Recreate logo + separator textures after device is restored
	if (SUCCEEDED(Original))
	{
		F::Render.LoadLogoSvg(pDevice);
		F::Render.LoadSeparatorSvgs(pDevice);
	}
	return Original;
}

LONG __stdcall WndProc::Func(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	// ENTER on a hovered pixel-surf-assist point opens its per-point pop-up. Eaten here so the
	// game doesn't also open chat on the same press; the pop-up then runs under the
	// IsAnyMenuOpen path below (cursor + ImGui mouse routing) until closed.
	if (!IsAnyMenuOpen() && uMsg == WM_KEYDOWN && wParam == VK_RETURN && F::Misc.m_iAssistPointHover >= 0)
	{
		F::Misc.OpenAssistPointMenu(F::Misc.m_iAssistPointHover);
		return 1;
	}

	if (IsAnyMenuOpen())
	{
		ImGui_ImplWin32_WndProcHandler(hWnd, uMsg, wParam, lParam);

		// While the per-point pop-up is open the keyboard is fully captured too (movement keys
		// freeze with the view, and the closing ENTER never reaches the game's chat bind).
#ifndef USE_CUSTOM_GUI
		if ((ImGui::GetIO().WantTextInput || F::Menu.m_bInKeybind || F::Misc.AssistPointMenuOpen()) && WM_KEYFIRST <= uMsg && uMsg <= WM_KEYLAST)
#else
		if ((ImGui::GetIO().WantTextInput || F::Misc.AssistPointMenuOpen()) && WM_KEYFIRST <= uMsg && uMsg <= WM_KEYLAST)
#endif
		{
			I::InputSystem->ResetInputState();
			return 1;
		}

		if (WM_MOUSEFIRST <= uMsg && uMsg <= WM_MOUSELAST)
			return 1;
	}

	return CallWindowProc(Original, hWnd, uMsg, wParam, lParam);
}

MAKE_HOOK(VGuiSurface_LockCursor, U::Memory.GetVirtual(I::MatSystemSurface, 62), void,
	void* rcx)
{
	if (IsAnyMenuOpen())
		return I::MatSystemSurface->UnlockCursor();

	CALL_ORIGINAL(rcx);
}

MAKE_HOOK(VGuiSurface_SetCursor, U::Memory.GetVirtual(I::MatSystemSurface, 51), void,
	void* rcx, HCursor cursor)
{
	if (IsAnyMenuOpen())
	{
		switch (F::Render.Cursor)
		{
		case 0: cursor = 2; break;
		case 1: cursor = 3; break;
		case 2: cursor = 12; break;
		case 3: cursor = 11; break;
		case 4: cursor = 10; break;
		case 5: cursor = 9; break;
		case 6: cursor = 8; break;
		case 7: cursor = 14; break;
		case 8: cursor = 13; break;
		}
	}

	CALL_ORIGINAL(rcx, cursor);
}

void WndProc::Initialize()
{
	hwWindow = SDK::GetTeamFortressWindow();

	Original = reinterpret_cast<WNDPROC>(SetWindowLongPtr(hwWindow, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(Func)));
}

void WndProc::Unload()
{
	SetWindowLongPtr(hwWindow, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(Original));
}