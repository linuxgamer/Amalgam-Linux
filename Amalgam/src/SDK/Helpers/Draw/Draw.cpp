#include "Draw.h"

#include "../../SDK.h"
#include "../../Definitions/Interfaces.h"
#include "../../../Utils/Math/Math.h"
#include "../../../Utils/Timer/Timer.h"
#include <array>
#include <ranges>

MAKE_SIGNATURE(CHudBaseDeathNotice_GetIcon, "client.dll", "40 53 48 81 EC ? ? ? ? 48 8B DA", 0x0);
MAKE_SIGNATURE(RenderLine, "engine.dll", "48 89 5C 24 ? 48 89 74 24 ? 44 89 44 24", 0x0);
MAKE_SIGNATURE(RenderBox, "engine.dll", "48 83 EC ? 8B 84 24 ? ? ? ? 4D 8B D8", 0x0);
MAKE_SIGNATURE(RenderWireframeBox, "engine.dll", "48 89 5C 24 ? 48 89 74 24 ? 48 89 7C 24 ? 55 41 54 41 55 41 56 41 57 48 8D AC 24 ? ? ? ? 48 81 EC ? ? ? ? 49 8B F9", 0x0);
MAKE_SIGNATURE(RenderWireframeSweptBox, "engine.dll", "48 8B C4 48 89 58 ? 48 89 50 ? 48 89 48 ? 55 56 57 41 54 41 55 41 56 41 57 48 8D A8 ? ? ? ? 48 81 EC ? ? ? ? 0F 29 70", 0x0);
MAKE_SIGNATURE(RenderTriangle, "engine.dll", "48 83 EC ? 41 8B C1 44 88 4C 24", 0x0);
MAKE_SIGNATURE(RenderSphere, "engine.dll", "48 8B C4 44 89 48 ? F3 0F 11 48", 0x0);

void CDraw::Start(bool bBadFontCheck)
{
	I::MatSystemSurface->StartDrawing();
	I::MatSystemSurface->DisableClipping(true);

	if (bBadFontCheck)
	{
		static Timer tTimer = {};
		if (tTimer.Run(1.f))
		{
			if (!GetTextSize("", H::Fonts.GetFont(FONT_ESP)).y)
				H::Fonts.Reload(Vars::Menu::Scale[DEFAULT_BIND]);
		}
	}
}

void CDraw::End()
{
	I::MatSystemSurface->FinishDrawing();
}

void CDraw::StartClipping(int x, int y, int w, int h)
{
	I::MatSystemSurface->DisableClipping(false);
	I::MatSystemSurface->SetClippingRect(x, y, x + w, y + h);
}

void CDraw::EndClipping()
{
	I::MatSystemSurface->DisableClipping(true);
}

void CDraw::UpdateScreenSize()
{
	I::MatSystemSurface->GetScreenSize(m_nScreenW, m_nScreenH);
}

void CDraw::UpdateW2SMatrix()
{
	CViewSetup tViewSetup;
	if (I::Client->GetPlayerView(tViewSetup))
	{
		static VMatrix mWorldToView, mViewToProjection, mWorldToPixels;
		I::RenderView->GetMatricesForView(tViewSetup, &mWorldToView, &mViewToProjection, &m_mWorldToProjection, &mWorldToPixels);
	}
}

Vec2 CDraw::GetTextSize(const char* text, const Font_t& tFont)
{
	int w = 0, h = 0;
	I::MatSystemSurface->GetTextSize(tFont.m_dwFont, SDK::ConvertUtf8ToWide(text).c_str(), w, h);
	return { float(w), float(h) };
}

Vec2 CDraw::GetTextSize(const wchar_t* text, const Font_t& tFont)
{
	int w = 0, h = 0;
	I::MatSystemSurface->GetTextSize(tFont.m_dwFont, text, w, h);
	return { float(w), float(h) };
}

static wchar_t s_wstr[1024] = { '\0' };
void CDraw::String(const Font_t& tFont, int x, int y, Color_t tColor, EAlign eAlign, const char* str)
{
	wsprintfW(s_wstr, L"%hs", str);
	const auto dwFont = tFont.m_dwFont;

	Vec2 vSize = GetTextSize(str, tFont);
	switch (eAlign)
	{
	case ALIGN_TOPLEFT: break;
	case ALIGN_TOP: x -= vSize.x / 2; break;
	case ALIGN_TOPRIGHT: x -= vSize.x; break;
	case ALIGN_LEFT: y -= vSize.y / 2; break;
	case ALIGN_CENTER: x -= vSize.x / 2; y -= vSize.y / 2; break;
	case ALIGN_RIGHT: x -= vSize.x; y -= vSize.y / 2; break;
	case ALIGN_BOTTOMLEFT: y -= vSize.y; break;
	case ALIGN_BOTTOM: x -= vSize.x / 2; y -= vSize.y; break;
	case ALIGN_BOTTOMRIGHT: x -= vSize.x; y -= vSize.y; break;
	}

	I::MatSystemSurface->DrawSetTextPos(x, y);
	I::MatSystemSurface->DrawSetTextFont(dwFont);
	I::MatSystemSurface->DrawSetTextColor(tColor.r, tColor.g, tColor.b, tColor.a);
	I::MatSystemSurface->DrawPrintText(s_wstr, int(wcslen(s_wstr)));
}
void CDraw::String(const Font_t& tFont, int x, int y, Color_t tColor, EAlign eAlign, const wchar_t* wstr)
{
	const auto dwFont = tFont.m_dwFont;

	Vec2 vSize = GetTextSize(wstr, tFont);
	switch (eAlign)
	{
	case ALIGN_TOPLEFT: break;
	case ALIGN_TOP: x -= vSize.x / 2; break;
	case ALIGN_TOPRIGHT: x -= vSize.x; break;
	case ALIGN_LEFT: y -= vSize.y / 2; break;
	case ALIGN_CENTER: x -= vSize.x / 2; y -= vSize.y / 2; break;
	case ALIGN_RIGHT: x -= vSize.x; y -= vSize.y / 2; break;
	case ALIGN_BOTTOMLEFT: y -= vSize.y; break;
	case ALIGN_BOTTOM: x -= vSize.x / 2; y -= vSize.y; break;
	case ALIGN_BOTTOMRIGHT: x -= vSize.x; y -= vSize.y; break;
	}

	I::MatSystemSurface->DrawSetTextPos(x, y);
	I::MatSystemSurface->DrawSetTextFont(dwFont);
	I::MatSystemSurface->DrawSetTextColor(tColor.r, tColor.g, tColor.b, tColor.a);
	I::MatSystemSurface->DrawPrintText(wstr, int(wcslen(wstr)));
}
void CDraw::StringOutlined(const Font_t& tFont, int x, int y, Color_t tColor, Color_t tColorOut, EAlign eAlign, const char* str, bool bAlpha, int iStyle)
{
	wsprintfW(s_wstr, L"%hs", str);
	const auto dwFont = tFont.m_dwFont;

	Vec2 vSize = GetTextSize(s_wstr, tFont);
	switch (eAlign)
	{
	case ALIGN_TOPLEFT: break;
	case ALIGN_TOP: x -= vSize.x / 2; break;
	case ALIGN_TOPRIGHT: x -= vSize.x; break;
	case ALIGN_LEFT: y -= vSize.y / 2; break;
	case ALIGN_CENTER: x -= vSize.x / 2; y -= vSize.y / 2; break;
	case ALIGN_RIGHT: x -= vSize.x; y -= vSize.y / 2; break;
	case ALIGN_BOTTOMLEFT: y -= vSize.y; break;
	case ALIGN_BOTTOM: x -= vSize.x / 2; y -= vSize.y; break;
	case ALIGN_BOTTOMRIGHT: x -= vSize.x; y -= vSize.y; break;
	}

	// Outline footprint: iStyle 0 = single drop shadow, 1 = full surround, -1 = follow CheapText global.
	const bool bCheap = iStyle == 0 || (iStyle != 1 && Vars::Menu::CheapText.Value);
	std::vector<std::pair<int, int>> vOutline = { { 1, 1 } };
	if (!bCheap)
		vOutline = { { -1, 0 }, { 0, -1 }, { 1, 0 }, { 0, 1 }, { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 } };

	if (bAlpha && !bCheap)
		tColorOut = tColorOut.Alpha(tColorOut.a * Math::RemapVal(tColorOut.Brightness(), 0, 255, 0.5f, 0.1f));

	if (tColorOut.a)
	{
		for (auto& [x2, y2] : vOutline)
		{
			I::MatSystemSurface->DrawSetTextPos(x + x2, y + y2);
			I::MatSystemSurface->DrawSetTextFont(dwFont);
			I::MatSystemSurface->DrawSetTextColor(tColorOut.r, tColorOut.g, tColorOut.b, tColorOut.a);
			I::MatSystemSurface->DrawPrintText(s_wstr, int(wcslen(s_wstr)));
		}
	}

	I::MatSystemSurface->DrawSetTextPos(x, y);
	I::MatSystemSurface->DrawSetTextFont(dwFont);
	I::MatSystemSurface->DrawSetTextColor(tColor.r, tColor.g, tColor.b, tColor.a);
	I::MatSystemSurface->DrawPrintText(s_wstr, int(wcslen(s_wstr)));
}
void CDraw::StringOutlined(const Font_t& tFont, int x, int y, Color_t tColor, Color_t tColorOut, EAlign eAlign, const wchar_t* wstr, bool bAlpha, int iStyle)
{
	const auto dwFont = tFont.m_dwFont;

	Vec2 vSize = GetTextSize(wstr, tFont);
	switch (eAlign)
	{
	case ALIGN_TOPLEFT: break;
	case ALIGN_TOP: x -= vSize.x / 2; break;
	case ALIGN_TOPRIGHT: x -= vSize.x; break;
	case ALIGN_LEFT: y -= vSize.y / 2; break;
	case ALIGN_CENTER: x -= vSize.x / 2; y -= vSize.y / 2; break;
	case ALIGN_RIGHT: x -= vSize.x; y -= vSize.y / 2; break;
	case ALIGN_BOTTOMLEFT: y -= vSize.y; break;
	case ALIGN_BOTTOM: x -= vSize.x / 2; y -= vSize.y; break;
	case ALIGN_BOTTOMRIGHT: x -= vSize.x; y -= vSize.y; break;
	}

	// Outline footprint: iStyle 0 = single drop shadow, 1 = full surround, -1 = follow CheapText global.
	const bool bCheap = iStyle == 0 || (iStyle != 1 && Vars::Menu::CheapText.Value);
	std::vector<std::pair<int, int>> vOutline = { { 1, 1 } };
	if (!bCheap)
		vOutline = { { -1, 0 }, { 0, -1 }, { 1, 0 }, { 0, 1 }, { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 } };

	if (bAlpha && !bCheap)
		tColorOut = tColorOut.Alpha(tColorOut.a * Math::RemapVal(tColorOut.Brightness(), 0, 255, 0.5f, 0.1f));

	if (tColorOut.a)
	{
		for (auto& [x2, y2] : vOutline)
		{
			I::MatSystemSurface->DrawSetTextPos(x + x2, y + y2);
			I::MatSystemSurface->DrawSetTextFont(dwFont);
			I::MatSystemSurface->DrawSetTextColor(tColorOut.r, tColorOut.g, tColorOut.b, tColorOut.a);
			I::MatSystemSurface->DrawPrintText(wstr, int(wcslen(wstr)));
		}
	}

	I::MatSystemSurface->DrawSetTextPos(x, y);
	I::MatSystemSurface->DrawSetTextFont(dwFont);
	I::MatSystemSurface->DrawSetTextColor(tColor.r, tColor.g, tColor.b, tColor.a);
	I::MatSystemSurface->DrawPrintText(wstr, int(wcslen(wstr)));
}

void CDraw::Line(int x1, int y1, int x2, int y2, Color_t tColor)
{
	I::MatSystemSurface->DrawSetColor(tColor.r, tColor.g, tColor.b, tColor.a);
	I::MatSystemSurface->DrawLine(x1, y1, x2, y2);
}

// Thick line = a filled quad expanded by half the thickness along the perpendicular. A textured
// polygon (not stamped 1px lines) so it stays solid + stable when the endpoints move sub-pixel.
void CDraw::LineThick(int x1, int y1, int x2, int y2, Color_t tColor, int iThickness)
{
	if (iThickness <= 1)
		return Line(x1, y1, x2, y2, tColor);

	const float dx = float(x2 - x1), dy = float(y2 - y1);
	const float flLen = sqrtf(dx * dx + dy * dy);
	if (flLen <= 0.f)
		return Line(x1, y1, x2, y2, tColor);

	const float flHalf = iThickness / 2.f;
	const float px = -dy / flLen * flHalf, py = dx / flLen * flHalf; // perpendicular * half-width
	std::vector<Vertex_t> v = {
		Vertex_t({ { x1 + px, y1 + py } }),
		Vertex_t({ { x2 + px, y2 + py } }),
		Vertex_t({ { x2 - px, y2 - py } }),
		Vertex_t({ { x1 - px, y1 - py } }),
	};
	FillPolygon(v, tColor);
}

void CDraw::FillPolygon(std::vector<Vertex_t> vVertices, Color_t tColor)
{
	// Bind a 1x1 opaque-white texture so DrawTexturedPolygon samples white and renders the vertex colour
	// at FULL coverage - matching DrawFilledRect / ImGui's AddRectFilled. An unset texture id samples
	// whatever was last bound (often a mostly-transparent glyph atlas), which made rounded fills read
	// translucent and inconsistently transparent vs the menu/watermark (spectator list, checkpoints,
	// model preview). One white texel makes every FillRoundRect panel solid and consistent.
	static int iId = 0;
	static bool bWhiteSet = false;
	if (!I::MatSystemSurface->IsTextureIDValid(iId))
	{
		iId = I::MatSystemSurface->CreateNewTextureID(true);
		bWhiteSet = false;
	}
	if (!bWhiteSet)
	{
		const unsigned char uWhite[4] = { 255, 255, 255, 255 };
		I::MatSystemSurface->DrawSetTextureRGBAEx(iId, uWhite, 1, 1, IMAGE_FORMAT_RGBA8888);
		bWhiteSet = true;
	}

	I::MatSystemSurface->DrawSetColor(tColor.r, tColor.g, tColor.b, tColor.a);
	I::MatSystemSurface->DrawSetTexture(iId);
	I::MatSystemSurface->DrawTexturedPolygon(int(vVertices.size()), vVertices.data());
}
void CDraw::LinePolygon(std::vector<Vertex_t> vVertices, Color_t tColor)
{
	static int iId = 0;
	if (!I::MatSystemSurface->IsTextureIDValid(iId))
		iId = I::MatSystemSurface->CreateNewTextureID();

	I::MatSystemSurface->DrawSetColor(tColor.r, tColor.g, tColor.b, tColor.a);
	I::MatSystemSurface->DrawSetTexture(iId);
	I::MatSystemSurface->DrawTexturedPolyLine(vVertices.data(), int(vVertices.size()));
}

void CDraw::FillRect(int x, int y, int w, int h, Color_t tColor)
{
	I::MatSystemSurface->DrawSetColor(tColor.r, tColor.g, tColor.b, tColor.a);
	I::MatSystemSurface->DrawFilledRect(x, y, x + w, y + h);
}
void CDraw::LineRect(int x, int y, int w, int h, Color_t tColor)
{
	I::MatSystemSurface->DrawSetColor(tColor.r, tColor.g, tColor.b, tColor.a);
	I::MatSystemSurface->DrawOutlinedRect(x, y, x + w, y + h);
}
void CDraw::GradientRect(int x, int y, int w, int h, Color_t tColorTop, Color_t tColorBottom, bool bHorizontal)
{
	I::MatSystemSurface->DrawSetColor(tColorTop.r, tColorTop.g, tColorTop.b, tColorTop.a);
	I::MatSystemSurface->DrawFilledRectFade(x, y, x + w, y + h, tColorTop.a, tColorBottom.a, bHorizontal);
	I::MatSystemSurface->DrawSetColor(tColorBottom.r, tColorBottom.g, tColorBottom.b, tColorBottom.a);
	I::MatSystemSurface->DrawFilledRectFade(x, y, x + w, y + h, tColorTop.a, tColorBottom.a, bHorizontal);
}
void CDraw::FillRectOutline(int x, int y, int w, int h, Color_t tColor, Color_t tColorOut)
{
	FillRect(x, y, w, h, tColor);
	LineRect(x - 1, y - 1, w + 2, h + 2, tColorOut);
}
void CDraw::LineRectOutline(int x, int y, int w, int h, Color_t tColor, Color_t tColorOut, bool bInside)
{
	LineRect(x, y, w, h, tColor);
	LineRect(x - 1, y - 1, w + 2, h + 2, tColorOut);
	if (bInside)
		LineRect(x + 1, y + 1, w - 2, h - 2, tColorOut);
}
void CDraw::LineCornerRect(int x, int y, int w, int h, Color_t tColor, float flLengthFrac)
{
	int cw = std::max(int(w * flLengthFrac), 1);
	int ch = std::max(int(h * flLengthFrac), 1);
	// top-left
	Line(x, y, x + cw, y, tColor);
	Line(x, y, x, y + ch, tColor);
	// top-right
	Line(x + w, y, x + w - cw, y, tColor);
	Line(x + w, y, x + w, y + ch, tColor);
	// bottom-left
	Line(x, y + h, x + cw, y + h, tColor);
	Line(x, y + h, x, y + h - ch, tColor);
	// bottom-right
	Line(x + w, y + h, x + w - cw, y + h, tColor);
	Line(x + w, y + h, x + w, y + h - ch, tColor);
}
void CDraw::LineCornerRectOutline(int x, int y, int w, int h, Color_t tColor, Color_t tColorOut, float flLengthFrac, bool bInside)
{
	LineCornerRect(x, y, w, h, tColor, flLengthFrac);
	LineCornerRect(x - 1, y - 1, w + 2, h + 2, tColorOut, flLengthFrac);
	if (bInside)
		LineCornerRect(x + 1, y + 1, w - 2, h - 2, tColorOut, flLengthFrac);
}
void CDraw::FillRectPercent(int x, int y, int w, int h, float t, Color_t tColor, Color_t tColorOut, EAlign eAlign, bool bAdjust)
{
	if (!bAdjust)
		FillRect(x - 1, y - 1, w + 2, h + 2, tColorOut);
	int nw = w, nh = h;
	switch (eAlign)
	{
	case ALIGN_LEFT: nw *= t; break;
	case ALIGN_RIGHT: nw *= t; x += w - nw; break;
	case ALIGN_TOP: nh *= t; break;
	case ALIGN_BOTTOM: nh *= t; y += h - nh; break;
	}
	if (bAdjust)
		FillRect(x - 1, y - 1, nw + 2, nh + 2, tColorOut);
	FillRect(x, y, nw, nh, tColor);
}
// Builds the rounded-rect perimeter vertex ring at fractional position/size/radius. Shared by the
// fill/line draws and their antialiasing feather passes (faded sub-pixel-offset rings) so surface
// panels read as smooth as the AA'd ImGui menu. Mirrors the original corner layout of Fill/LineRoundRect.
static void BuildRoundRectRing(std::vector<Vertex_t>& out, float x, float y, float w, float h, float radius, int iCount)
{
	out.clear();
	radius = std::max(0.f, radius);
	const int _iCount = std::max(iCount / 4, 2);
	const float flDelta = 90.f / (_iCount - 1);
	for (int i = 0; i < 4; i++)
	{
		const float _x = x + ((i < 2) ? (w - radius) : radius);
		const float _y = y + ((i % 3) ? (h - radius) : radius);

		const float a = 90.f * i;
		for (int j = 0; j < _iCount; j++)
		{
			const float _a = DEG2RAD(a + j * flDelta);
			out.emplace_back(Vertex_t({ { _x + radius * sinf(_a), _y - radius * cosf(_a) } }));
		}
	}
}
void CDraw::FillRoundRect(int x, int y, int w, int h, int iRadius, Color_t tColor, int iCount)
{
	// Plain rounded fill (no feather): feathering a fill paints a faint ring just outside its edge,
	// which doubles up where panels stack overlapping fills (e.g. Kamidere's body + header) and reads
	// as a misaligned border. Edge antialiasing belongs to the outline strokes, not the fill.
	std::vector<Vertex_t> vVertices = {};
	BuildRoundRectRing(vVertices, float(x), float(y), float(w), float(h), float(iRadius), iCount);
	FillPolygon(vVertices, tColor);
}
void CDraw::FillRoundRectSolid(int x, int y, int w, int h, int iRadius, Color_t tColor)
{
	// Opaque rounded fill that matches the ImGui menu/watermark exactly. The body is drawn with
	// DrawFilledRect (same primitive as FillRect / the menu) instead of DrawTexturedPolygon, which
	// blends a touch lighter and made the spectator-list panel read more see-through than the menu.
	// Only the small corner discs stay on the textured-poly path.
	iRadius = std::clamp(iRadius, 0, std::min(w, h) / 2);
	if (iRadius <= 0) { FillRect(x, y, w, h, tColor); return; }

	FillRect(x + iRadius, y, w - 2 * iRadius, h, tColor);                       // middle column, full height
	FillRect(x, y + iRadius, iRadius, h - 2 * iRadius, tColor);                 // left strip
	FillRect(x + w - iRadius, y + iRadius, iRadius, h - 2 * iRadius, tColor);   // right strip

	auto Corner = [&](float cx, float cy, float a0)
	{
		std::vector<Vertex_t> v = { { { cx, cy } } };
		const int seg = 8;
		for (int i = 0; i <= seg; i++)
		{
			const float a = DEG2RAD(a0 + 90.f * float(i) / seg);
			v.emplace_back(Vector2D{ cx + iRadius * cosf(a), cy + iRadius * sinf(a) });
		}
		FillPolygon(v, tColor);
	};
	Corner(float(x + iRadius),     float(y + iRadius),     180.f); // top-left
	Corner(float(x + w - iRadius), float(y + iRadius),     270.f); // top-right
	Corner(float(x + w - iRadius), float(y + h - iRadius), 0.f);   // bottom-right
	Corner(float(x + iRadius),     float(y + h - iRadius), 90.f);  // bottom-left
}
void CDraw::LineRoundRect(int x, int y, int w, int h, int iRadius, Color_t tColor, int iCount)
{
	const float fw = float(w - 1), fh = float(h - 1);
	std::vector<Vertex_t> vVertices = {};
	BuildRoundRectRing(vVertices, float(x), float(y), fw, fh, float(iRadius), iCount);
	LinePolygon(vVertices, tColor);

	// Antialias the 1px stroke: two faded sub-pixel-offset rings either side widen it to ~2px soft edge.
	if (tColor.a > 0)
	{
		const byte a = (byte)std::clamp(int(tColor.a * 0.4f), 0, 255);
		if (a)
		{
			const Color_t fc = { tColor.r, tColor.g, tColor.b, a };
			BuildRoundRectRing(vVertices, x - 0.6f, y - 0.6f, fw + 1.2f, fh + 1.2f, iRadius + 0.6f, iCount);
			LinePolygon(vVertices, fc);
			BuildRoundRectRing(vVertices, x + 0.6f, y + 0.6f, fw - 1.2f, fh - 1.2f, iRadius - 0.6f, iCount);
			LinePolygon(vVertices, fc);
		}
	}
}

void CDraw::PanelOutline(int x, int y, int w, int h, int iRadius, float flAlpha)
{
	const int iLayers = std::clamp(Vars::Menu::Style::OutlineCount.Value, 0, 4);
	if (iLayers <= 0)
		return;

	flAlpha = std::clamp(flAlpha, 0.f, 1.f);

	// Crisp 1px rounded ring (no per-ring feather) so the stacked body rings build clean thickness
	// without their feathers overlapping into mush; the whole outline is feathered once at its edges.
	std::vector<Vertex_t> vRing = {};
	auto CrispRing = [&](float e, Color_t col)
	{
		BuildRoundRectRing(vRing, x - e, y - e, (w - 1) + e * 2.f, (h - 1) + e * 2.f, iRadius + e, 64);
		LinePolygon(vRing, col);
	};

	int iOffset = 0; // accumulated thickness of the layers already drawn (each stacks outside the last)
	for (int i = 0; i < iLayers; ++i)
	{
		Color_t c; float flThickRaw, flBlurRaw;
		if (!GetStyleOutlineLayer(i, c, flThickRaw, flBlurRaw))
			break;
		const int iThick = std::max(1, int(Scale(flThickRaw)));

		// Main stroke: concentric crisp 1px rings from iOffset outward to build the requested thickness.
		const Color_t mc = { c.r, c.g, c.b, (byte)std::clamp(int(c.a * flAlpha), 0, 255) };
		for (int t = 0; t < iThick; ++t)
			CrispRing(float(iOffset + t), mc);

		// Antialias feather: faded rings just OUTSIDE this layer's outer edge soften the otherwise-hard
		// surface poly-line edges. Matches DrawStyledOutline (ImGui), which strokes each layer cleanly
		// outside the panel edge with no inward bleed - so surface panels (spectator list, checkpoints)
		// carry the exact same outline look as the watermark/menu instead of feathering into the fill.
		if (mc.a > 0)
		{
			const float fOut = float(iOffset + iThick - 1);
			for (int s = 1; s <= 2; ++s)
			{
				const byte fa = (byte)std::clamp(int(mc.a * (s == 1 ? 0.4f : 0.18f)), 0, 255);
				if (!fa)
					continue;
				const Color_t fc = { c.r, c.g, c.b, fa };
				CrispRing(fOut + s * 0.6f, fc);
			}
		}
		iOffset += iThick;
	}
}

void CDraw::StyledPanel(int x, int y, int w, int h, float flAlpha)
{
	// One core for every surface panel - mirrors CMenu::DrawStyledBackground (ImGui): rounded fill in
	// Style::Color at Style::Rounding, then the shared outline stack, both scaled by flAlpha. Keeping
	// the fill + outline paired in one place is what stops panels from drifting apart in rounding,
	// colour or outline (the bug this replaces, where each panel re-typed the pair by hand).
	// A prior surface draw (e.g. ESP fade) can leave the global alpha multiplier < 1, which silently
	// scales this panel down so it reads MORE transparent than the menu/watermark (which draw via ImGui
	// and aren't affected). Reset it so every surface panel draws at its true Style alpha - mirrors the
	// reset ModelPreview::DrawBackground already does. This is the "spectator list looks more see-through
	// than the menu" fix.
	I::MatSystemSurface->DrawSetAlphaMultiplier(1.f);

	flAlpha = std::clamp(flAlpha, 0.f, 1.f);
	const int iRadius = std::max(0, int(Scale(Vars::Menu::Style::Rounding.Value)));
	const Color_t c = Vars::Menu::Style::Color.Value;
	const Color_t tBg = { c.r, c.g, c.b, (byte)std::clamp(int(c.a * flAlpha), 0, 255) };
	FillRoundRectSolid(x, y, w, h, iRadius, tBg); // DrawFilledRect body so the panel matches the menu/watermark opacity
	PanelOutline(x, y, w, h, iRadius, flAlpha);
}

void CDraw::FillCircle(int x, int y, float iRadius, int iSegments, Color_t tColor)
{
	std::vector<Vertex_t> vVertices = {};

	const float step = static_cast<float>(PI) * 2.0f / iSegments;
	for (float a = 0; a < PI * 2.0f; a += step)
		vVertices.emplace_back(Vector2D{ iRadius * cosf(a) + x, iRadius * sinf(a) + y });

	FillPolygon(vVertices, tColor);
}
void CDraw::LineCircle(int x, int y, float iRadius, int iSegments, Color_t tColor)
{
	I::MatSystemSurface->DrawSetColor(tColor.r, tColor.g, tColor.b, tColor.a);
	I::MatSystemSurface->DrawOutlinedCircle(x, y, iRadius, iSegments);
}

void CDraw::Texture(const char* sTexture, int x, int y, int w, int h, EAlign eAlign)
{
	static std::unordered_map<uint32_t, int> mTextures = {};

	switch (eAlign)
	{
	case ALIGN_TOPLEFT: break;
	case ALIGN_TOP: x -= w / 2; break;
	case ALIGN_TOPRIGHT: x -= w; break;
	case ALIGN_LEFT: y -= h / 2; break;
	case ALIGN_CENTER: x -= w / 2; y -= h / 2; break;
	case ALIGN_RIGHT: x -= w; y -= h / 2; break;
	case ALIGN_BOTTOMLEFT: y -= h; break;
	case ALIGN_BOTTOM: x -= w / 2; y -= h; break;
	case ALIGN_BOTTOMRIGHT: x -= w; y -= h; break;
	}

	auto& nTexture = mTextures[FNV1A::Hash32Const(sTexture)];
	if (!nTexture)
		I::MatSystemSurface->DrawSetTextureFile(nTexture = I::MatSystemSurface->CreateNewTextureID(), sTexture, false, true);

	I::MatSystemSurface->DrawSetColor(255, 255, 255, 255);
	I::MatSystemSurface->DrawSetTexture(nTexture);
	I::MatSystemSurface->DrawTexturedRect(x, y, x + w, y + h);
}
void CDraw::TextureId(int nTexture, int x, int y, int w, int h, Color_t tColor, EAlign eAlign)
{
	if (nTexture == -1)
		return;

	switch (eAlign)
	{
	case ALIGN_TOPLEFT: break;
	case ALIGN_TOP: x -= w / 2; break;
	case ALIGN_TOPRIGHT: x -= w; break;
	case ALIGN_LEFT: y -= h / 2; break;
	case ALIGN_CENTER: x -= w / 2; y -= h / 2; break;
	case ALIGN_RIGHT: x -= w; y -= h / 2; break;
	case ALIGN_BOTTOMLEFT: y -= h; break;
	case ALIGN_BOTTOM: x -= w / 2; y -= h; break;
	case ALIGN_BOTTOMRIGHT: x -= w; y -= h; break;
	}

	I::MatSystemSurface->DrawSetColor(tColor.r, tColor.g, tColor.b, tColor.a);
	I::MatSystemSurface->DrawSetTexture(nTexture);
	I::MatSystemSurface->DrawTexturedRect(x, y, x + w, y + h);
}
CHudTexture* CDraw::GetIcon(const char* szIcon, int eIconFormat)
{
	return S::CHudBaseDeathNotice_GetIcon.Call<CHudTexture*>(nullptr, szIcon, eIconFormat);
}
int CDraw::CreateTextureFromArray(const unsigned char* rgba, int w, int h)
{
	const int nTextureIdOut = I::MatSystemSurface->CreateNewTextureID(true);
	I::MatSystemSurface->DrawSetTextureRGBAEx(nTextureIdOut, rgba, w, h, IMAGE_FORMAT_RGBA8888);
	return nTextureIdOut;
}
int CDraw::NewProceduralTexId()
{
	return I::MatSystemSurface->CreateNewTextureID(true);
}
void CDraw::DrawStyleTexture(int iId, const unsigned char* rgba, int texW, int texH, int x, int y, int w, int h, bool bFullUV)
{
	if (iId == -1 || !rgba)
		return;
	I::MatSystemSurface->DrawSetTextureRGBAEx(iId, rgba, texW, texH, IMAGE_FORMAT_RGBA8888);
	I::MatSystemSurface->DrawSetColor(255, 255, 255, 255);
	I::MatSystemSurface->DrawSetTexture(iId);
	if (bFullUV)
		I::MatSystemSurface->DrawTexturedRect(x, y, x + w, y + h);
	else
	{
		// Blur: sample the slice of the (full-screen) blurred frame directly behind the panel.
		const float u0 = float(x) / m_nScreenW, v0 = float(y) / m_nScreenH;
		const float u1 = float(x + w) / m_nScreenW, v1 = float(y + h) / m_nScreenH;
		I::MatSystemSurface->DrawTexturedSubRect(x, y, x + w, y + h, u0, v0, u1, v1);
	}
}
void CDraw::DrawHudTexture(float x, float y, float s, const CHudTexture* pTexture, Color_t clr)
{
	if (!pTexture)
		return;

	if (pTexture->bRenderUsingFont)
	{
		I::MatSystemSurface->DrawSetTextFont(pTexture->hFont);
		I::MatSystemSurface->DrawSetTextColor(clr.r, clr.g, clr.b, clr.a);
		I::MatSystemSurface->DrawSetTextPos(x, y);
		I::MatSystemSurface->DrawUnicodeChar(pTexture->cCharacterInFont);
	}
	else if (pTexture->textureId != -1)
	{
		I::MatSystemSurface->DrawSetTexture(pTexture->textureId);
		I::MatSystemSurface->DrawSetColor(clr.r, clr.g, clr.b, clr.a);
		I::MatSystemSurface->DrawTexturedSubRect(x, y, x + pTexture->Width() * s, y + pTexture->Height() * s, pTexture->texCoords[0], pTexture->texCoords[1], pTexture->texCoords[2], pTexture->texCoords[3]);
	}
}
void CDraw::DrawHudTextureByName(float x, float y, float s, const char* sTexture, Color_t clr)
{
	const CHudTexture* pIcon = GetIcon(sTexture, 0);

	if (!pIcon)
		return;

	if (pIcon->bRenderUsingFont)
	{
		I::MatSystemSurface->DrawSetTextFont(pIcon->hFont);
		I::MatSystemSurface->DrawSetTextColor(clr.r, clr.g, clr.b, clr.a);
		I::MatSystemSurface->DrawSetTextPos(x, y);
		I::MatSystemSurface->DrawUnicodeChar(pIcon->cCharacterInFont);
	}
	else if (pIcon->textureId != -1)
	{
		I::MatSystemSurface->DrawSetTexture(pIcon->textureId);
		I::MatSystemSurface->DrawSetColor(clr.r, clr.g, clr.b, clr.a);
		I::MatSystemSurface->DrawTexturedSubRect(x, y, x + pIcon->Width() * s, y + pIcon->Height() * s, pIcon->texCoords[0], pIcon->texCoords[1], pIcon->texCoords[2], pIcon->texCoords[3]);
	}
}

void CDraw::Avatar(int x, int y, int w, int h, const uint32 nFriendID, EAlign eAlign)
{
	if (!nFriendID)
		return;

	switch (eAlign)
	{
	case ALIGN_TOPLEFT: break;
	case ALIGN_TOP: x -= w / 2; break;
	case ALIGN_TOPRIGHT: x -= w; break;
	case ALIGN_LEFT: y -= h / 2; break;
	case ALIGN_CENTER: x -= w / 2; y -= h / 2; break;
	case ALIGN_RIGHT: x -= w; y -= h / 2; break;
	case ALIGN_BOTTOMLEFT: y -= h; break;
	case ALIGN_BOTTOM: x -= w / 2; y -= h; break;
	case ALIGN_BOTTOMRIGHT: x -= w; y -= h; break;
	}

	const int nTexture = GetAvatarTexture(nFriendID);
	if (nTexture != -1)
	{
		I::MatSystemSurface->DrawSetColor(255, 255, 255, 255);
		I::MatSystemSurface->DrawSetTexture(nTexture);
		I::MatSystemSurface->DrawTexturedRect(x, y, x + w, y + h);
	}
}
int CDraw::GetAvatarTexture(const uint32 nFriendID)
{
	if (m_mAvatars.contains(nFriendID))
		return m_mAvatars[nFriendID];

	const CSteamID tSteamID(nFriendID, k_EUniversePublic, k_EAccountTypeIndividual);
	const int nAvatar = I::SteamFriends->GetMediumFriendAvatar(tSteamID);
	if (nAvatar <= 0)
	{
		// Non-friends' avatars aren't cached locally - ask Steam to download it (see ISteamFriends note).
		// Don't cache the miss; it pops in a later frame once the PersonaStateChanged callback lands.
		I::SteamFriends->RequestUserInformation(tSteamID, false);
		return -1;
	}

	uint32 newW = 0, newH = 0;
	if (I::SteamUtils->GetImageSize(nAvatar, &newW, &newH))
	{
		const uint32 nSize = newW * newH * uint32(sizeof(uint8) * 4);
		auto* pData = static_cast<uint8*>(std::malloc(nSize));
		if (!pData)
			return -1;

		int nResult = -1;
		if (I::SteamUtils->GetImageRGBA(nAvatar, pData, nSize))
		{
			const int nTextureID = I::MatSystemSurface->CreateNewTextureID(true);
			if (I::MatSystemSurface->IsTextureIDValid(nTextureID))
			{
				I::MatSystemSurface->DrawSetTextureRGBA(nTextureID, pData, newW, newH, 0, false);
				nResult = m_mAvatars[nFriendID] = nTextureID;
			}
		}

		std::free(pData);
		return nResult;
	}

	return -1;
}
void CDraw::AvatarCircle(int x, int y, float flRadius, const uint32 nFriendID, int iSegments, byte cAlpha)
{
	if (!nFriendID)
		return;

	const int nTexture = GetAvatarTexture(nFriendID);
	if (nTexture == -1)
		return;

	// Circle polygon with texcoords mapping the avatar square onto it.
	std::vector<Vertex_t> vVertices;
	vVertices.reserve(iSegments);
	const float flStep = static_cast<float>(PI) * 2.f / iSegments;
	for (int i = 0; i < iSegments; i++)
	{
		const float a = i * flStep;
		const float c = cosf(a), s = sinf(a);
		vVertices.emplace_back(Vector2D{ x + flRadius * c, y + flRadius * s }, Vector2D{ 0.5f + 0.5f * c, 0.5f + 0.5f * s });
	}

	I::MatSystemSurface->DrawSetColor(255, 255, 255, cAlpha);
	I::MatSystemSurface->DrawSetTexture(nTexture);
	I::MatSystemSurface->DrawTexturedPolygon(iSegments, vVertices.data());
}
void CDraw::ClearAvatarCache()
{
	for (int iID : m_mAvatars | std::views::values)
	{
		I::MatSystemSurface->DeleteTextureByID(iID);
		I::MatSystemSurface->DestroyTextureID(iID);
	}

	m_mAvatars.clear();
}

void CDraw::RenderLine(const Vec3& vStart, const Vec3& vEnd, Color_t tColor, bool bZBuffer)
{
	if (!tColor.a)
		return;

	S::RenderLine.Call<void>(std::ref(vStart), std::ref(vEnd), tColor, bZBuffer);
}

void CDraw::RenderPath(const std::vector<Vec3>& vPath, Color_t tColor, bool bZBuffer, int iStyle, float flTime, int iSeparatorSpacing, float flSeparatorLength)
{
	if (!tColor.a || iStyle == Vars::Visuals::Simulation::StyleEnum::Off)
		return;

	for (size_t i = 1; i < vPath.size(); i++)
	{
		if (flTime < 0.f && vPath.size() - i > -flTime)
			continue;

		switch (iStyle)
		{
		case Vars::Visuals::Simulation::StyleEnum::Line:
		{
			RenderLine(vPath[i - 1], vPath[i], tColor, bZBuffer);
			break;
		}
		case Vars::Visuals::Simulation::StyleEnum::Separators:
		{
			RenderLine(vPath[i - 1], vPath[i], tColor, bZBuffer);
			if (!(i % iSeparatorSpacing))
			{
				const Vec3& vStart = vPath[i - 1];
				const Vec3& vEnd = vPath[i];

				Vec3 vDir = (vEnd - vStart).Normalized2D();
				vDir = Math::RotatePoint(vDir * flSeparatorLength, {}, { 0, 90, 0 });
				RenderLine(vEnd, vEnd + vDir, tColor, bZBuffer);
			}
			break;
		}
		case Vars::Visuals::Simulation::StyleEnum::Spaced:
		{
			if (!(i % 2))
				RenderLine(vPath[i - 1], vPath[i], tColor, bZBuffer);
			break;
		}
		case Vars::Visuals::Simulation::StyleEnum::Arrows:
		{
			if (!(i % 3))
			{
				const Vec3& vStart = vPath[i - 1];
				const Vec3& vEnd = vPath[i];

				if (!(vStart - vEnd).IsZero())
				{
					Vec3 vAngles = Math::VectorAngles(vEnd - vStart);
					Vec3 vForward, vRight, vUp; Math::AngleVectors(vAngles, &vForward, &vRight, &vUp);
					RenderLine(vEnd, vEnd - vForward * 5 + vRight * 5, tColor, bZBuffer);
					RenderLine(vEnd, vEnd - vForward * 5 - vRight * 5, tColor, bZBuffer);
				}
			}
			break;
		}
		case Vars::Visuals::Simulation::StyleEnum::Boxes:
		{
			RenderLine(vPath[i - 1], vPath[i], tColor, bZBuffer);
			if (!(i % iSeparatorSpacing))
				RenderWireframeBox(vPath[i], { -1, -1, -1 }, { 1, 1, 1 }, {}, tColor, bZBuffer);
			break;
		}
		}
	}
}

void CDraw::RenderBox(const Vec3& vOrigin, const Vec3& vMins, const Vec3& vMaxs, const Vec3& vAngles, Color_t tColor, bool bZBuffer, bool bInsideOut)
{
	if (!tColor.a)
		return;

	S::RenderBox.Call<void>(std::ref(vOrigin), std::ref(vAngles), std::ref(vMins), std::ref(vMaxs), tColor, bZBuffer, bInsideOut);
}

void CDraw::RenderWireframeBox(const Vec3& vOrigin, const Vec3& vMins, const Vec3& vMaxs, const Vec3& vAngles, Color_t tColor, bool bZBuffer)
{
	if (!tColor.a)
		return;

	S::RenderWireframeBox.Call<void>(std::ref(vOrigin), std::ref(vAngles), std::ref(vMins), std::ref(vMaxs), tColor, bZBuffer);
}

void CDraw::RenderWireframeSweptBox(const Vector& vStart, const Vector& vEnd, const Vec3& vMins, const Vec3& vMaxs, const Vec3& vAngles, Color_t tColor, bool bZBuffer)
{
	if (!tColor.a)
		return;

	S::RenderWireframeSweptBox.Call<void>(std::ref(vStart), std::ref(vEnd), std::ref(vAngles), std::ref(vMins), std::ref(vMaxs), tColor, bZBuffer);
}

void CDraw::RenderTriangle(const Vector& vPoint1, const Vector& vPoint2, const Vector& vPoint3, Color_t tColor, bool bZBuffer)
{
	if (!tColor.a)
		return;

	S::RenderTriangle.Call<void>(std::ref(vPoint1), std::ref(vPoint2), std::ref(vPoint3), tColor, bZBuffer);
}

void CDraw::RenderSphere(const Vector& vCenter, float flRadius, int nTheta, int nPhi, Color_t tColor, IMaterial* pMaterial)
{
	if (!tColor.a)
		return;

	S::RenderSphere.Call<void>(std::ref(vCenter), flRadius, nTheta, nPhi, tColor, pMaterial);
}

void CDraw::RenderSphere(const Vector& vCenter, float flRadius, int nTheta, int nPhi, Color_t tColor, bool bZBuffer)
{
	if (!tColor.a)
		return;

	static auto pVertexColor = *reinterpret_cast<IMaterial**>(U::Memory.RelToAbs(S::VertexColor()));
	static auto pVertexColorIgnoreZ = *reinterpret_cast<IMaterial**>(U::Memory.RelToAbs(S::VertexColorIgnoreZ()));
	auto pMaterial = bZBuffer ? pVertexColor : pVertexColorIgnoreZ;

	RenderSphere(vCenter, flRadius, nTheta, nPhi, tColor, pMaterial);
}

void CDraw::RenderWireframeSphere(const Vector& vCenter, float flRadius, int nTheta, int nPhi, Color_t tColor, bool bZBuffer)
{
	if (!tColor.a)
		return;

	static auto pWireframe = *reinterpret_cast<IMaterial**>(U::Memory.RelToAbs(S::Wireframe()));
	static auto pWireframeIgnoreZ = *reinterpret_cast<IMaterial**>(U::Memory.RelToAbs(S::WireframeIgnoreZ()));
	auto pMaterial = bZBuffer ? pWireframe : pWireframeIgnoreZ;

	RenderSphere(vCenter, flRadius, nTheta, nPhi, tColor, pMaterial);
}