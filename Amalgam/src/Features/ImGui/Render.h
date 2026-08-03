#pragma once
#include "../../SDK/SDK.h"
#include <ImGui/imgui_impl_dx9.h>
#include <ImGui/imgui.h>
#include <d3d9.h>
#include <atomic>

class CRender
{
public:
	void Render(IDirect3DDevice9* pDevice);
	void Initialize(IDirect3DDevice9* pDevice);

	void LoadColors();
	void LoadFonts();
	void LoadStyle();
	void LoadLogoSvg(IDirect3DDevice9* pDevice);
	void ReleaseLogoSvgs();
	void UpdateCustomLogo(IDirect3DDevice9* pDevice);  // re-rasterizes the pasted custom SVG when it changes
	IDirect3DTexture9* GetActiveLogoTex();             // selected built-in or custom logo (may be nullptr)
	void LoadSeparatorSvgs(IDirect3DDevice9* pDevice);
	void ReleaseSeparatorSvgs();
	void UpdateCustomSeparator(IDirect3DDevice9* pDevice); // re-rasterizes the pasted custom separator SVG when it changes

	// Steam avatar -> D3D9 texture cache for the ImGui Minimal spectator list (the VGui surface avatar
	IDirect3DTexture9* GetAvatarTex(uint32_t iAccountID);
	void ReleaseAvatarTextures();

	// Built-in logos (accent-tinted at draw time). Index matches Vars::Menu::LogoStyleEnum value
	// (0 == Chud, 1 == Star of David,...). Custom (pasted SVG) lives in m_pLogoCustomTex.
	static constexpr int kBuiltinLogoCount = 5;
	IDirect3DTexture9* m_pLogoTex[kBuiltinLogoCount] = {};
	IDirect3DTexture9* m_pLogoCustomTex = nullptr;
	std::string m_sLogoCustomLast; // cached source to detect custom-SVG changes

	// Watermark separator glyphs (accent-tinted at draw time). Index matches g_WmSepGlyphs /
	// (Vars::Menu::WmSepGlyphEnum value - 1). Sized at runtime from g_iWmSepGlyphCount.
	static constexpr int kMaxSepGlyphs = 24;
	IDirect3DTexture9* m_pSepTex[kMaxSepGlyphs] = {};
	IDirect3DTexture9* m_pSepCustomTex = nullptr; // pasted custom separator SVG (used when glyph == Custom)
	std::string m_sSepCustomLast;                 // cached source to detect custom-SVG changes

	std::unordered_map<uint32_t, IDirect3DTexture9*> m_mAvatarTex; // account id -> avatar texture
	IDirect3DDevice9* m_pCurDevice = nullptr;                      // last device from Render(), for lazy avatar uploads

	int Cursor = 2;

	// Colors
	ImColor Accent = {};
	ImColor Background0 = {};
	ImColor Background0p5 = {};
	ImColor Background1 = {};
	ImColor Background1p5 = {};
	ImColor Background1p5L = {};
	ImColor Background2 = {};
	ImColor Inactive = {};
	ImColor Active = {};

	// Fonts
	ImFont* FontSmall = nullptr;
	ImFont* FontRegular = nullptr;
	ImFont* FontBold = nullptr;
	ImFont* FontLarge = nullptr;
	ImFont* FontMono = nullptr;
	ImFont* FontMedia = nullptr; // rebuilt at MediaPlayerScale so the overlay text stays crisp instead of bitmap-stretched

	ImFont* IconFont = nullptr;

	// Set from the (non-render) config-load thread to force the render thread to rebuild the ImGui
	// font atlas next frame. The per-frame value comparison alone can miss the startup auto-load
	std::atomic<bool> m_bRebuildFonts{ false };
};

ADD_FEATURE(CRender, Render);