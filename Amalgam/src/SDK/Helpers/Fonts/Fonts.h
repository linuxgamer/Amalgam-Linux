#pragma once
#include "../../../Utils/Macros/Macros.h"
#include "../../Definitions/Interfaces/IMatSystemSurface.h"
#include <unordered_map>

enum EFonts
{
	FONT_ESP,
	FONT_ESP_LABEL, // ESP status "flags" labels — ESP face at a user scale (Vars::ESP::FlagScale)
	FONT_INDICATORS,
	FONT_INDICATORS_THIN,
	FONT_SPECTATOR,
	FONT_SPECTATOR_MINIMAL, // Minimal spec-list player rows — menu body face, thin (weight 300), watermark-sized
	FONT_SPECTATOR_MINIMAL_TITLE, // Minimal spec-list "SPECTATORS" title — same face/size, semibold (weight 600) = watermark
	FONT_SPECTATOR_CLARITY, // Clarity spec-list scheme font — clarity-source watermark_font (Tahoma 12, weight 500, dropshadow)
	FONT_SPECTATOR_CLARITY_TITLE, // Clarity spec-list "spectators" title — same font, bold (weight 700) for a thicker label
	FONT_SPECTATOR_KAMIDERE, // Kamidere spec-list — menu body face (follows "Main menu font"), normal weight, AA
	FONT_NOTIFY,       // toast notifications — matches the menu body font (SF Pro Display 13px)
	FONT_KEYBIND,      // keybind indicator label — user-configurable family+size
	FONT_VELOCITY      // velocity indicator label — user-configurable family+size
};

struct Font_t
{
	const char* m_szName;
	int m_nTall, m_nFlags, m_nWeight;
	unsigned long m_dwFont;
};

class CFonts
{
private:
	std::unordered_map<EFonts, Font_t> m_mFonts = {};
	// Owned storage for the Minimal spec-list font family (tracks the user's "Main menu font").
	// Font_t::m_szName points here instead of into the ConfigVar's std::string, so a concurrent
	// Reload from the config-load thread can't free the buffer mid-use (it's overwritten in place,
	// never reallocated) — that use-after-free is what crashed SetFontGlyphSet before.
	char m_szMinimalFamily[128] = {};

public:
	void Reload(float flDPI = 1.f);
	const Font_t& GetFont(EFonts eFont);
};

ADD_FEATURE_CUSTOM(CFonts, Fonts, H);