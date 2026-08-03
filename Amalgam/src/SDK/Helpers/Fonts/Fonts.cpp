#include "Fonts.h"
#include "Doctrine_W00_Regular_font.h"
#include "../../Vars.h"
#ifdef ALETHERIUM_CUSTOM_FONTS
#include "../../../Features/ImGui/Fonts/AppleFont/SFProDisplayRegular.h" // menu body font (GDI family "SF Pro Display")
#endif

#include <Windows.h>
#pragma comment(lib, "gdi32.lib")

// Register embedded fonts with GDI once for this process so the engine surface
// (which rasterizes glyphs through GDI) can create them by their family name.
// AddFontMemResourceEx works straight from memory, so no temp file is needed.
static void RegisterEmbeddedFonts()
{
	static bool s_bRegistered = false;
	if (s_bRegistered)
		return;

	DWORD nInstalled = 0;
	AddFontMemResourceEx(
		const_cast<unsigned char*>(Doctrine_W00_Regular_compressed_data),
		static_cast<DWORD>(sizeof(Doctrine_W00_Regular_compressed_data)),
		nullptr, &nInstalled);

#ifdef ALETHERIUM_CUSTOM_FONTS
	// SF Pro Display (the apple-font menu typeface) so notifications can match the menu.
	AddFontMemResourceEx(
		SFProDisplayRegular,
		static_cast<DWORD>(sizeof(SFProDisplayRegular)),
		nullptr, &nInstalled);
#endif

	s_bRegistered = true;
}

void CFonts::Reload(float flDPI)
{
	RegisterEmbeddedFonts();

	// ESP name/labels/text font — user-configurable family + size (empty/0 = default Verdana 12).
	{
		const bool bEspCustom = !Vars::ESP::FontFamily.Value.empty();
		const char* szEspFam = bEspCustom ? Vars::ESP::FontFamily.Value.c_str() : "Verdana";
		int nEspTall = Vars::ESP::FontSize.Value > 0
			? int(Vars::ESP::FontSize.Value * flDPI)
			: int(12.f * flDPI);
		if (nEspTall < 1) nEspTall = 1;
		const int nEspFlags = Vars::ESP::FontNoAntialias.Value ? 0 : FONTFLAG_ANTIALIAS;
		m_mFonts[FONT_ESP] = { szEspFam, nEspTall, nEspFlags, 0 };
		// Status "flags" labels reuse the ESP face but at a user scale (Vars::ESP::FlagScale).
		int nLabelTall = int(nEspTall * Vars::ESP::FlagScale.Value);
		if (nLabelTall < 1) nLabelTall = 1;
		m_mFonts[FONT_ESP_LABEL] = { szEspFam, nLabelTall, nEspFlags, 0 };
	}
	m_mFonts[FONT_INDICATORS] = { "Verdana", int(22.f * flDPI), FONTFLAG_ANTIALIAS, 700 };
	// Thin alternative for the velocity/keybind indicators (GDI family name from the TTF name table).
	m_mFonts[FONT_INDICATORS_THIN] = { "DoctrineW00-Regular", int(24.f * flDPI), FONTFLAG_ANTIALIAS, 400 };
	// Delusional (delnew) spectator-list font: Tahoma 13, normal weight (their surface::fonts::esp).
	// Tracks the spectator-list scale slider so the panel and its text grow together.
	int nSpectatorTall = int(13.f * flDPI * Vars::Menu::SpectatorListScale.Value);
	if (nSpectatorTall < 1) nSpectatorTall = 1;
	m_mFonts[FONT_SPECTATOR] = { "Tahoma", nSpectatorTall, FONTFLAG_ANTIALIAS, 0 };
	// Minimal spec-list font: follows the menu/body font so the panel matches the custom font. Uses the
	// user-selected "Main menu font" family when set (best-effort GDI family, same as the ESP/keybind
	// surface fonts), else the embedded menu body face (SF Pro Display on custom-font builds, Verdana
	// otherwise). Sized off the spectator-list scale slider like FONT_SPECTATOR. The family is copied
	// into m_szMinimalFamily (fixed owned buffer, never a ConfigVar.Value.c_str()) so a concurrent
	// config-load Reload can't free it mid-use — see the m_szMinimalFamily note in Fonts.h.
	{
#ifdef ALETHERIUM_CUSTOM_FONTS
		const char* szMenuDefault = "SF Pro Display";
#else
		const char* szMenuDefault = "Verdana";
#endif
		const std::string& sMenuFont = Vars::Menu::Style::Font.Value;
		strncpy_s(m_szMinimalFamily, sMenuFont.empty() ? szMenuDefault : sMenuFont.c_str(), _TRUNCATE);
		// Sized to match the watermark (which is Scale(13) * Style::FontScale): fold FontScale in on top
		// of the spec-list scale slider so the title reads at the watermark's size, not smaller.
		int nMinimalTall = int(13.f * flDPI * Vars::Menu::Style::FontScale.Value * Vars::Menu::SpectatorListScale.Value);
		if (nMinimalTall < 1) nMinimalTall = 1;
		m_mFonts[FONT_SPECTATOR_MINIMAL]       = { m_szMinimalFamily, nMinimalTall, FONTFLAG_ANTIALIAS, 300 }; // thin player rows
		m_mFonts[FONT_SPECTATOR_MINIMAL_TITLE] = { m_szMinimalFamily, nMinimalTall, FONTFLAG_ANTIALIAS, 600 }; // semibold title = watermark
		// Kamidere panel font: same menu body face (shares m_szMinimalFamily), normal weight, AA. Sized off
		// the spec-list scale slider like FONT_SPECTATOR (no FontScale fold — it's a panel, not the watermark).
		int nKamidereTall = int(22.f * flDPI * Vars::Menu::SpectatorListScale.Value);
		if (nKamidereTall < 1) nKamidereTall = 1;
		m_mFonts[FONT_SPECTATOR_KAMIDERE] = { m_szMinimalFamily, nKamidereTall, FONTFLAG_ANTIALIAS, 400 };
	}
	// Clarity spec-list scheme font: clarity-source's watermark_font, exactly — Tahoma 12,
	// weight 500, dropshadow (no antialias). Tracks the scale slider like FONT_SPECTATOR.
	int nClarityTall = int(12.f * flDPI * Vars::Menu::SpectatorListScale.Value);
	if (nClarityTall < 1) nClarityTall = 1;
	m_mFonts[FONT_SPECTATOR_CLARITY] = { "Tahoma", nClarityTall, FONTFLAG_DROPSHADOW, 500 };
	// Same scheme font, bold (weight 700), for a thicker "spectators" title.
	m_mFonts[FONT_SPECTATOR_CLARITY_TITLE] = { "Tahoma", nClarityTall, FONTFLAG_DROPSHADOW, 700 };
	// Toast notification font — match the menu body font.
#ifdef ALETHERIUM_CUSTOM_FONTS
	m_mFonts[FONT_NOTIFY] = { "SF Pro Display", int(13.f * flDPI), FONTFLAG_ANTIALIAS, 400 };
#else
	m_mFonts[FONT_NOTIFY] = { "Verdana", int(13.f * flDPI), FONTFLAG_ANTIALIAS, 400 };
#endif
	// User-configurable keybind/velocity fonts.  Default to FONT_INDICATORS values;
	// Reload() is called again when the user changes the font vars.
	{
		// When no custom family is chosen, fall back to the default (Verdana/700) or
		// default-thin (DoctrineW00-Regular/400) family, matching the shared indicator
		// fonts — but still honour the keybind size slider so it affects those too.
		const bool bKbThin = Vars::Visuals::UI::IndicatorThinFont.Value;
		const bool bKbCustom = !Vars::Visuals::UI::KeybindFontFamily.Value.empty();
		const char* kbFam = bKbCustom
			? Vars::Visuals::UI::KeybindFontFamily.Value.c_str()
			: (bKbThin ? "DoctrineW00-Regular" : "Verdana");
		const int nKbWeight = bKbCustom ? 700 : (bKbThin ? 400 : 700);
		const float flKbDefault = bKbThin ? 24.f : 22.f;
		int kbSz = Vars::Visuals::UI::KeybindFontSize.Value > 0
			? int(Vars::Visuals::UI::KeybindFontSize.Value * flDPI)
			: int(flKbDefault * flDPI);
		m_mFonts[FONT_KEYBIND] = { kbFam, kbSz, FONTFLAG_ANTIALIAS, nKbWeight };

		// Same default-family/size fallback as the keybind font so the velocity size
		// slider scales the default and default-thin fonts too.
		const bool bVCustom = !Vars::Visuals::UI::VelocityFontFamily.Value.empty();
		const char* vFam = bVCustom
			? Vars::Visuals::UI::VelocityFontFamily.Value.c_str()
			: (bKbThin ? "DoctrineW00-Regular" : "Verdana");
		const int nVWeight = bVCustom ? 700 : (bKbThin ? 400 : 700);
		const float flVDefault = bKbThin ? 24.f : 22.f;
		int vSz = Vars::Visuals::UI::VelocityFontSize.Value > 0
			? int(Vars::Visuals::UI::VelocityFontSize.Value * flDPI)
			: int(flVDefault * flDPI);
		m_mFonts[FONT_VELOCITY] = { vFam, vSz, FONTFLAG_ANTIALIAS, nVWeight };
	}

	for (auto& [_, fFont] : m_mFonts)
	{
		if (fFont.m_dwFont = I::MatSystemSurface->CreateFont())
			I::MatSystemSurface->SetFontGlyphSet(fFont.m_dwFont, fFont.m_szName, fFont.m_nTall, fFont.m_nWeight, 0, 0, fFont.m_nFlags);
	}
}

const Font_t& CFonts::GetFont(EFonts eFont)
{
	return m_mFonts[eFont];
}