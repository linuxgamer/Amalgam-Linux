#include "Render.h"

#include "../../Hooks/Direct3DDevice9.h"
#include <ImGui/imgui_impl_win32.h>
#include <ImGui/imgui_internal.h> // ImClamp / IM_PI for the custom crosshair
#ifdef IMGUI_ENABLE_FREETYPE
#include <ImGui/imgui_freetype.h> // the reference-style crisp pixel font hinting flags
#endif
#include "Fonts/MaterialDesign/MaterialIcons.h"
#include "Fonts/MaterialDesign/IconDefinitions.h"
#include "Fonts/CascadiaMono/CascadiaMono.h"
#include "Fonts/Roboto/RobotoMedium.h"
#include "Fonts/Roboto/RobotoBlack.h"
#include "Fonts/AppleFont/SFProDisplayRegular.h" // SF Pro Display (apple-font) menu font
#include "Fonts/AppleFont/SFProDisplaySemibold.h"
// Embedded menu-font presets (subset to Latin+Cyrillic, see tools/font_to_header.py).
#include "Fonts/Kodchasan/KodchasanRegular.h"
#include "Fonts/Kodchasan/KodchasanBold.h"
#include "Fonts/SNPro/SNProRegular.h"
#include "Fonts/SNPro/SNProBold.h"
#include "Fonts/OpenSans/OpenSansRegular.h"
#include "Fonts/OpenSans/OpenSansBold.h"
#include "Fonts/Montserrat/MontserratRegular.h"
#include "Fonts/Montserrat/MontserratBold.h"
#include "Menu/Menu.h"
#include "../../../media_player.h"

#include <cmath>     // crosshair math
#include <algorithm> // std::clamp / std::max
#include <cstring>   // strlen for separator-glyph table

#define NANOSVG_IMPLEMENTATION
#define NANOSVGRAST_IMPLEMENTATION
#include "nanosvg.h"
#include "nanosvgrast.h"
#include "chud_svg.h"
#include "logo_svgs.h"
#include "wm_sep_svgs.h"


// #define USE_CUSTOM_GUI // Disabled - using original menu

// ==== the reference custom crosshair (full port) ===============================================
namespace CrosshairPort
{
	constexpr float kChUnitToPx = 2.0f;
	struct ChArm { float x1, y1, x2, y2; };

	inline bool ChAA() { return Vars::Visuals::UI::CrosshairAntialias.Value; }
	inline ImU32 PackCol(Color_t c) { return IM_COL32(c.r, c.g, c.b, c.a); }

	// Representative RGB for each TF2 killstreak sheen (idleeffect 1-7), used when the crosshair is
	// set to track the sheen colour. Alpha is supplied by the caller (the user's crosshair alpha).
	inline Color_t SheenColor(int iAlpha)
	{
		switch (Vars::Visuals::Effects::WeaponSheen.Value)
		{
		case 1:  return Color_t(220, 220, 220, iAlpha); // Team Shine
		case 2:  return Color_t(255, 235, 100, iAlpha); // Deadly Daffodil
		case 3:  return Color_t(255, 150, 40, iAlpha);  // Manndarin
		case 4:  return Color_t(130, 230, 60, iAlpha);  // Mean Green
		case 5:  return Color_t(40, 200, 120, iAlpha);  // Agonizing Emerald
		case 6:  return Color_t(150, 90, 230, iAlpha);  // Villainous Violet
		case 7:  return Color_t(255, 80, 120, iAlpha);  // Hot Rod
		default: return Color_t(255, 255, 255, iAlpha);
		}
	}

	static ImU32 ChLerpColor(ImU32 c0, ImU32 c1, float t)
	{
		if (t <= 0.f) return c0;
		if (t >= 1.f) return c1;
		auto l = [](int a, int b, float t) { return (int)(a + (b - a) * t + 0.5f); };
		return IM_COL32(
			l((c0 >> 0) & 0xFF, (c1 >> 0) & 0xFF, t),
			l((c0 >> 8) & 0xFF, (c1 >> 8) & 0xFF, t),
			l((c0 >> 16) & 0xFF, (c1 >> 16) & 0xFF, t),
			l((c0 >> 24) & 0xFF, (c1 >> 24) & 0xFF, t));
	}

	static inline ImU32 ChScaleAlpha(ImU32 col, float t)
	{
		if (t <= 0.f) return col & 0x00FFFFFFu;
		if (t >= 1.f) return col;
		int a = (int)(((col >> 24) & 0xFF) * t + 0.5f);
		return (col & 0x00FFFFFFu) | ((ImU32)a << 24);
	}

	static ChArm ChMakeArm(float cx, float cy, float thPx, float len, float gap, int dir)
	{
		float lo = ChAA() ? (thPx * 0.5f) : floorf(thPx * 0.5f);
		float hi = ChAA() ? (thPx * 0.5f) : (thPx - lo);
		ChArm a{};
		switch (dir)
		{
		case 0: a = { cx - gap - len, cy - lo, cx - gap,       cy + hi }; break;
		case 1: a = { cx + gap,       cy - lo, cx + gap + len, cy + hi }; break;
		case 2: a = { cx - lo, cy + gap,       cx + hi, cy + gap + len }; break;
		case 3: a = { cx - lo, cy - gap - len, cx + hi, cy - gap       }; break;
		}
		return a;
	}

	// Rect with sub-pixel AA fringe on all 4 edges (or a hard rect when AA is off).
	static void ChDrawRect(ImDrawList* dl, float x1, float y1, float x2, float y2, ImU32 col)
	{
		if (!ChAA())
		{
			dl->AddRectFilled(ImVec2(floorf(x1), floorf(y1)), ImVec2(ceilf(x2), ceilf(y2)), col);
			return;
		}

		float cx1 = ceilf(x1), cy1 = ceilf(y1);
		float cx2 = floorf(x2), cy2 = floorf(y2);
		float fL = cx1 - x1, fR = x2 - cx2, fT = cy1 - y1, fB = y2 - cy2;

		if (cx1 < cx2 && cy1 < cy2)
			dl->AddRectFilled(ImVec2(cx1, cy1), ImVec2(cx2, cy2), col);

		if (fL > 0.001f) dl->AddRectFilled(ImVec2(cx1 - 1.f, cy1), ImVec2(cx1, cy2), ChScaleAlpha(col, fL));
		if (fR > 0.001f) dl->AddRectFilled(ImVec2(cx2, cy1), ImVec2(cx2 + 1.f, cy2), ChScaleAlpha(col, fR));
		if (fT > 0.001f) dl->AddRectFilled(ImVec2(cx1, cy1 - 1.f), ImVec2(cx2, cy1), ChScaleAlpha(col, fT));
		if (fB > 0.001f) dl->AddRectFilled(ImVec2(cx1, cy2), ImVec2(cx2, cy2 + 1.f), ChScaleAlpha(col, fB));

		if (fL > 0.001f && fT > 0.001f) dl->AddRectFilled(ImVec2(cx1 - 1.f, cy1 - 1.f), ImVec2(cx1, cy1), ChScaleAlpha(col, fL * fT));
		if (fR > 0.001f && fT > 0.001f) dl->AddRectFilled(ImVec2(cx2, cy1 - 1.f), ImVec2(cx2 + 1.f, cy1), ChScaleAlpha(col, fR * fT));
		if (fL > 0.001f && fB > 0.001f) dl->AddRectFilled(ImVec2(cx1 - 1.f, cy2), ImVec2(cx1, cy2 + 1.f), ChScaleAlpha(col, fL * fB));
		if (fR > 0.001f && fB > 0.001f) dl->AddRectFilled(ImVec2(cx2, cy2), ImVec2(cx2 + 1.f, cy2 + 1.f), ChScaleAlpha(col, fR * fB));

		// AA fix: with AA on, default crosshair geometry is all even (unit*2) and centred on an integer
		// pixel, so every edge is integer-aligned and the fractional fringes above are all zero - AA was
		if (cx1 < cx2 && cy1 < cy2)
		{
			const ImU32 fe = ChScaleAlpha(col, 0.5f);
			if (fL <= 0.001f) dl->AddRectFilled(ImVec2(cx1 - 1.f, cy1), ImVec2(cx1, cy2), fe);
			if (fR <= 0.001f) dl->AddRectFilled(ImVec2(cx2, cy1), ImVec2(cx2 + 1.f, cy2), fe);
			if (fT <= 0.001f) dl->AddRectFilled(ImVec2(cx1, cy1 - 1.f), ImVec2(cx2, cy1), fe);
			if (fB <= 0.001f) dl->AddRectFilled(ImVec2(cx1, cy2), ImVec2(cx2, cy2 + 1.f), fe);
		}
	}

	// One arm with gradient and/or taper, drawn as 1px axial slices so AA stays correct.
	static void ChDrawArmFancy(ImDrawList* dl, const ChArm& a, int dir,
		ImU32 centerCol, ImU32 outerCol, float gradStart, float gradEnd, float nearScale, float farScale)
	{
		gradStart = ImClamp(gradStart, 0.f, 1.f);
		gradEnd = ImClamp(gradEnd, gradStart, 1.f);

		const float baseHalf = (dir <= 1) ? (a.y2 - a.y1) * 0.5f : (a.x2 - a.x1) * 0.5f;
		const float innerHalf = baseHalf * nearScale;
		const float outerHalf = baseHalf * farScale;
		const float centerLine = (dir <= 1) ? (a.y1 + a.y2) * 0.5f : (a.x1 + a.x2) * 0.5f;
		const float gRange = gradEnd - gradStart;
		const bool hz = (dir <= 1);

		float axStart, axEnd;
		switch (dir)
		{
		case 0:  axStart = a.x2; axEnd = a.x1; break;
		case 1:  axStart = a.x1; axEnd = a.x2; break;
		case 2:  axStart = a.y1; axEnd = a.y2; break;
		default: axStart = a.y2; axEnd = a.y1; break;
		}

		const float xMin = fminf(axStart, axEnd);
		const float xMax = fmaxf(axStart, axEnd);
		const float axSpan = axEnd - axStart;

		for (float x = floorf(xMin); x < xMax - 1e-4f; x += 1.f)
		{
			const float rx1 = fmaxf(x, xMin);
			const float rx2 = fminf(x + 1.f, xMax);
			const float xMid = (rx1 + rx2) * 0.5f;

			const float t = (fabsf(axSpan) > 1e-5f) ? ImClamp((xMid - axStart) / axSpan, 0.f, 1.f) : 0.5f;
			const float h = innerHalf + t * (outerHalf - innerHalf);
			const float g = (gRange > 0.001f) ? ImClamp((t - gradStart) / gRange, 0.f, 1.f) : (t >= gradStart ? 1.f : 0.f);
			const ImU32 col = ChLerpColor(centerCol, outerCol, g);

			if (hz) ChDrawRect(dl, rx1, centerLine - h, rx2, centerLine + h, col);
			else    ChDrawRect(dl, centerLine - h, rx1, centerLine + h, rx2, col);
		}
	}

	static void ChDrawOutline(ImDrawList* dl, float ax1, float ay1, float ax2, float ay2,
		float otUnits, ImU32 col, bool sideL, bool sideR, bool sideT, bool sideB)
	{
		const float px = ChAA() ? (otUnits * kChUnitToPx) : ((otUnits < 1.0f) ? 1.0f : floorf(otUnits * kChUnitToPx));
		float expL = sideL ? px : 0.f, expR = sideR ? px : 0.f, expT = sideT ? px : 0.f, expB = sideB ? px : 0.f;
		ChDrawRect(dl, ax1 - expL, ay1 - expT, ax2 + expR, ay2 + expB, col);
	}

	static void ChDrawOutlineBlur(ImDrawList* dl, float ax1, float ay1, float ax2, float ay2,
		float otUnits, ImU32 col, float blurRadius, bool sideL, bool sideR, bool sideT, bool sideB)
	{
		const float baseA = (float)((col >> 24) & 0xFF) / 255.f;
		const ImU32 rgb = col & 0x00FFFFFFu;
		const float maxExpand = blurRadius * kChUnitToPx;
		const float sigma = maxExpand * 0.4f + 0.5f;
		const int N = 16;
		const float baseThick = ChAA() ? (otUnits * kChUnitToPx) : ((otUnits < 1.0f) ? 1.0f : floorf(otUnits * kChUnitToPx));

		const bool allSides = sideL && sideR && sideT && sideB;
		float cumTransp = 1.f;
		for (int i = N - 1; i >= 0; --i)
		{
			const float br = ((float)(i + 1) / N) * maxExpand;
			const float target = baseA * expf(-0.5f * (br / sigma) * (br / sigma));
			if (cumTransp < 1e-4f) break;
			const float layerA = 1.f - (1.f - target) / cumTransp;
			if (layerA <= 0.f) continue;
			const int ai = (int)(layerA * 255.f + 0.5f);
			if (ai <= 0) continue;
			cumTransp *= (1.f - layerA);
			const float px = baseThick + br;
			float eL = sideL ? px : 0.f, eR = sideR ? px : 0.f, eT = sideT ? px : 0.f, eB = sideB ? px : 0.f;
			if (allSides)
				dl->AddRectFilled(ImVec2(ax1 - px, ay1 - px), ImVec2(ax2 + px, ay2 + px), rgb | ((ImU32)ai << 24), px);
			else
				ChDrawRect(dl, ax1 - eL, ay1 - eT, ax2 + eR, ay2 + eB, rgb | ((ImU32)ai << 24));
		}
		ChDrawOutline(dl, ax1, ay1, ax2, ay2, otUnits, col, sideL, sideR, sideT, sideB);
	}

	// While our custom crosshair is on, force the game's "crosshair" cvar to 0 so the default HUD
	void ApplyConVar()
	{
		static auto pCrosshair = H::ConVars.FindVar("crosshair");
		if (!pCrosshair)
			return;
		static bool bForced = false;
		if (Vars::Visuals::UI::Crosshair.Value)
		{
			if (pCrosshair->GetInt() != 0)
				pCrosshair->SetValue(0);
			bForced = true;
		}
		else if (bForced)
		{
			pCrosshair->SetValue(1);
			bForced = false;
		}
	}

	void Render()
	{
		using namespace Vars::Visuals::UI;
		if (!Crosshair.Value)
			return;
		auto pLocal = H::Entities.GetLocal();
		if (!I::EngineClient->IsInGame() || !pLocal || !pLocal->IsAlive())
			return;

		ImGuiIO& io = ImGui::GetIO();
		ImDrawList* dl = ImGui::GetBackgroundDrawList();
		const int chVtxStart = dl->VtxBuffer.Size;

		const bool subPixel = (CrosshairThickness.Value < 1.0f);
		const float thPx = ChAA() ? (CrosshairThickness.Value * kChUnitToPx) : (subPixel ? 1.0f : floorf(CrosshairThickness.Value * kChUnitToPx));

		float cx = io.DisplaySize.x * 0.5f;
		float cy = io.DisplaySize.y * 0.5f;
		if (!ChAA())
		{
			const float cxOff = ((int)thPx % 2 != 0) ? 0.5f : 0.0f;
			cx = floorf(cx) + cxOff;
			cy = floorf(cy) + cxOff;
		}

		const float len = ChAA() ? (CrosshairLength.Value * kChUnitToPx) : floorf(CrosshairLength.Value * kChUnitToPx);
		const float gap = ChAA() ? (CrosshairGap.Value * kChUnitToPx) : floorf(CrosshairGap.Value * kChUnitToPx);

		float dynGapPx = 0.f;
		if (CrosshairDynamic.Value && CrosshairDynamicRef.Value > 0.f)
		{
			float speed = pLocal->m_vecVelocity().Length2D();
			if (speed > CrosshairDynamicMin.Value)
			{
				float range = CrosshairDynamicRef.Value - CrosshairDynamicMin.Value;
				float t = (range > 0.f) ? (speed - CrosshairDynamicMin.Value) / range : 1.f;
				if (t > 1.f) t = 1.f;
				dynGapPx = ChAA() ? (t * CrosshairDynamicMax.Value * kChUnitToPx) : floorf(t * CrosshairDynamicMax.Value * kChUnitToPx);
			}
		}
		const float effectiveGap = gap + dynGapPx;
		const float gapAsp = CrosshairGapAspect.Value > 0.01f ? CrosshairGapAspect.Value : 0.01f;
		const float hGap = ChAA() ? (effectiveGap * (gapAsp >= 1.0f ? gapAsp : 1.0f)) : floorf(effectiveGap * (gapAsp >= 1.0f ? gapAsp : 1.0f));
		const float vGap = ChAA() ? (effectiveGap * (gapAsp <= 1.0f ? (1.0f / gapAsp) : 1.0f)) : floorf(effectiveGap * (gapAsp <= 1.0f ? (1.0f / gapAsp) : 1.0f));

		const float aspect = CrosshairAspect.Value > 0.01f ? CrosshairAspect.Value : 0.01f;
		const float hLen = ChAA() ? (len * (aspect >= 1.0f ? aspect : 1.0f)) : floorf(len * (aspect >= 1.0f ? aspect : 1.0f));
		const float vLen = ChAA() ? (len * (aspect <= 1.0f ? (1.0f / aspect) : 1.0f)) : floorf(len * (aspect <= 1.0f ? (1.0f / aspect) : 1.0f));

		// Crosshair colour (optionally tracking the sheen palette, preserving the user's alpha).
		Color_t tColBase = Vars::Colors::Crosshair.Value;
		if (CrosshairUseSheen.Value && Vars::Visuals::Effects::WeaponSheen.Value)
			tColBase = SheenColor(tColBase.a);
		ImU32 col = PackCol(tColBase);

		const bool fancyArm = (!subPixel) && (CrosshairGradient.Value || CrosshairTaper.Value);
		ImU32 gradCenterCol = col, gradOuterCol = col;
		float gradStart = 0.f, gradEnd = 1.f;
		float taperNear = 1.0f, taperFar = 1.0f;
		if (fancyArm)
		{
			if (CrosshairGradient.Value)
			{
				gradCenterCol = PackCol(Vars::Colors::CrosshairGradCenter.Value);
				gradOuterCol = PackCol(Vars::Colors::CrosshairGradOuter.Value);
				gradStart = CrosshairGradStart.Value;
				gradEnd = CrosshairGradEnd.Value;
			}
			taperNear = CrosshairTaper.Value ? CrosshairTaperNear.Value : 1.0f;
			taperFar = CrosshairTaper.Value ? CrosshairTaperFar.Value : 1.0f;
		}

		Color_t tOutlineBase = Vars::Colors::CrosshairOutline.Value;
		ImU32 outl = PackCol(tOutlineBase);

		const int iOutlineSides = CrosshairOutlineSides.Value;
		const bool sL = iOutlineSides & CrosshairOutlineSidesEnum::Left;
		const bool sR = iOutlineSides & CrosshairOutlineSidesEnum::Right;
		const bool sT = iOutlineSides & CrosshairOutlineSidesEnum::Top;
		const bool sB = iOutlineSides & CrosshairOutlineSidesEnum::Bottom;

		auto armLen = [&](int dir) { return (dir <= 1) ? hLen : vLen; };
		auto armGap = [&](int dir) { return (dir <= 1) ? hGap : vGap; };
		auto armEnabled = [&](int dir) { return (CrosshairArms.Value & (1 << dir)) != 0; };

		if (CrosshairOutline.Value)
		{
			const float ot = CrosshairOutlineThick.Value < 0.1f ? 0.1f : CrosshairOutlineThick.Value;
			const float otPx = ChAA() ? (ot * kChUnitToPx) : ((ot < 1.f) ? 1.f : floorf(ot * kChUnitToPx));
			const float armBaseHalf = thPx * 0.5f;
			if (CrosshairOutlineOnLines.Value)
			{
				for (int i = 0; i < 4; ++i)
				{
					if (!armEnabled(i)) continue;
					ChArm a = ChMakeArm(cx, cy, thPx, armLen(i), armGap(i), i);
					if (fancyArm && !CrosshairOutlineBlur.Value)
					{
						ChArm outA = a;
						if (i <= 1) { outA.x1 -= otPx; outA.x2 += otPx; }
						else { outA.y1 -= otPx; outA.y2 += otPx; }
						const float addScale = armBaseHalf > 0.f ? otPx / armBaseHalf : 0.f;
						ChDrawArmFancy(dl, outA, i, outl, outl, 0.f, 0.f, taperNear + addScale, taperFar + addScale);
					}
					else if (CrosshairOutlineBlur.Value)
						ChDrawOutlineBlur(dl, a.x1, a.y1, a.x2, a.y2, ot, outl, CrosshairOutlineBlurRadius.Value, sL, sR, sT, sB);
					else
						ChDrawOutline(dl, a.x1, a.y1, a.x2, a.y2, ot, outl, sL, sR, sT, sB);
				}
			}
			if (CrosshairDot.Value && CrosshairOutlineOnDot.Value)
			{
				float r = thPx * 0.5f;
				if (CrosshairDotCircle.Value && CrosshairOutlineBlur.Value)
				{
					const float baseA2 = (float)((outl >> 24) & 0xFF) / 255.f;
					const ImU32 oRgb = outl & 0x00FFFFFFu;
					const float maxExp2 = CrosshairOutlineBlurRadius.Value * kChUnitToPx;
					const float sigma2 = maxExp2 * 0.4f + 0.5f;
					float cumT2 = 1.f;
					for (int i = 15; i >= 0; --i)
					{
						const float br2 = ((float)(i + 1) / 16) * maxExp2;
						const float tgt = baseA2 * expf(-0.5f * (br2 / sigma2) * (br2 / sigma2));
						if (cumT2 < 1e-4f) break;
						const float la = 1.f - (1.f - tgt) / cumT2;
						if (la <= 0.f) continue;
						const int ai2 = (int)(la * 255.f + 0.5f);
						if (ai2 <= 0) continue;
						cumT2 *= (1.f - la);
						dl->AddCircleFilled(ImVec2(cx, cy), r + otPx + br2, oRgb | ((ImU32)ai2 << 24));
					}
					dl->AddCircleFilled(ImVec2(cx, cy), r + otPx, outl);
				}
				else if (CrosshairDotCircle.Value)
					dl->AddCircleFilled(ImVec2(cx, cy), r + otPx, outl);
				else if (CrosshairOutlineBlur.Value)
					ChDrawOutlineBlur(dl, cx - r, cy - r, cx + r, cy + r, ot, outl, CrosshairOutlineBlurRadius.Value, sL, sR, sT, sB);
				else
					ChDrawOutline(dl, cx - r, cy - r, cx + r, cy + r, ot, outl, sL, sR, sT, sB);
			}
		}

		for (int i = 0; i < 4; ++i)
		{
			if (!armEnabled(i)) continue;
			ChArm a = ChMakeArm(cx, cy, thPx, armLen(i), armGap(i), i);
			if (subPixel)
			{
				ChDrawRect(dl, a.x1, a.y1, a.x2, a.y1 + 1.f, col);
				ChDrawRect(dl, a.x1, a.y1, a.x1 + 1.f, a.y2, col);
			}
			else if (fancyArm)
				ChDrawArmFancy(dl, a, i, gradCenterCol, gradOuterCol, gradStart, gradEnd, taperNear, taperFar);
			else
				ChDrawRect(dl, a.x1, a.y1, a.x2, a.y2, col);
		}

		if (CrosshairDot.Value)
		{
			float lo = thPx * 0.5f;
			ImU32 dotCol = CrosshairDotCustomColor.Value
				? PackCol(Vars::Colors::CrosshairDot.Value)
				: ((CrosshairGradient.Value && !subPixel) ? gradCenterCol : col);
			if (CrosshairDotCircle.Value)
				dl->AddCircleFilled(ImVec2(cx, cy), lo, dotCol);
			else
				ChDrawRect(dl, cx - lo, cy - lo, cx + lo, cy + lo, dotCol);
		}

		if (fabsf(CrosshairRotation.Value) > 0.01f)
		{
			const float rad = CrosshairRotation.Value * IM_PI / 180.f;
			const float ca = cosf(rad), sa = sinf(rad);
			for (int vi = chVtxStart; vi < dl->VtxBuffer.Size; ++vi)
			{
				float dx = dl->VtxBuffer[vi].pos.x - cx;
				float dy = dl->VtxBuffer[vi].pos.y - cy;
				dl->VtxBuffer[vi].pos.x = cx + dx * ca - dy * sa;
				dl->VtxBuffer[vi].pos.y = cy + dx * sa + dy * ca;
			}
		}
	}
}

void CRender::Render(IDirect3DDevice9* pDevice)
{
	using namespace ImGui;

	static std::once_flag initFlag;
	std::call_once(initFlag, [&]
		{
			Initialize(pDevice);
		});

	LoadColors();
	{
		static float flStaticScale = Vars::Menu::Scale.Value;
		float flOldScale = flStaticScale;
		float flNewScale = flStaticScale = Vars::Menu::Scale.Value;
		if (flNewScale != flOldScale)
		{
			LoadFonts();
			LoadStyle();
		}
	}
	{
		// Rebuild the atlas so the media-player font is re-baked at the new scale (crisp, not stretched)
		static float flStaticMediaScale = Vars::Menu::MediaPlayerScale.Value;
		float flOldMediaScale = flStaticMediaScale;
		float flNewMediaScale = flStaticMediaScale = Vars::Menu::MediaPlayerScale.Value;
		if (flNewMediaScale != flOldMediaScale)
			LoadFonts();
	}
	{
		// Rebuild the atlas when the user changes the menu font, font scale or AA.
		static std::string sStaticFont = Vars::Menu::Style::Font.Value;
		static int iStaticFontPreset = Vars::Menu::Style::FontPreset.Value;
		static float flStaticFontScale = Vars::Menu::Style::FontScale.Value;
		static bool bStaticFontAA = Vars::Menu::Style::FontAntiAlias.Value;
		const bool bRebuildRequested = m_bRebuildFonts.exchange(false);
		if (bRebuildRequested
			|| sStaticFont != Vars::Menu::Style::Font.Value
			|| iStaticFontPreset != Vars::Menu::Style::FontPreset.Value
			|| flStaticFontScale != Vars::Menu::Style::FontScale.Value
			|| bStaticFontAA != Vars::Menu::Style::FontAntiAlias.Value)
		{
			sStaticFont = Vars::Menu::Style::Font.Value;
			iStaticFontPreset = Vars::Menu::Style::FontPreset.Value;
			flStaticFontScale = Vars::Menu::Style::FontScale.Value;
			bStaticFontAA = Vars::Menu::Style::FontAntiAlias.Value;
			LoadFonts();
		}
	}

	m_pCurDevice = pDevice; // for lazy avatar-texture uploads in the Minimal spectator list (render thread)

	// Create/swap the album-art texture here (render thread) from bytes the media-player
	UploadPendingAlbumArt(pDevice);

	// Rebuild the pasted custom logo texture (render thread, device available) when it changes.
	UpdateCustomLogo(pDevice);
	// Same for the pasted custom watermark-separator texture.
	UpdateCustomSeparator(pDevice);

	DWORD dwOldRGB; pDevice->GetRenderState(D3DRS_SRGBWRITEENABLE, &dwOldRGB);
	pDevice->SetRenderState(D3DRS_SRGBWRITEENABLE, false);
	ImGui_ImplDX9_NewFrame();
	ImGui_ImplWin32_NewFrame();
	NewFrame();

	// Use custom menu if enabled, otherwise use original menu
	// Toggle with a define or runtime variable
#ifdef USE_CUSTOM_GUI
	F::CustomMenu.Draw();
#else
	F::Menu.Render();
#endif

	CrosshairPort::ApplyConVar(); // hide the default crosshair (crosshair 0) while ours is enabled
	CrosshairPort::Render(); // custom crosshair into the background draw list (renders every frame)

	EndFrame();
	ImGui::Render();
	ImGui_ImplDX9_RenderDrawData(GetDrawData());
	pDevice->SetRenderState(D3DRS_SRGBWRITEENABLE, dwOldRGB);
}

void CRender::LoadColors()
{
	using namespace ImGui;

	auto ColorToVec = [](Color_t tColor) -> ImColor
		{
			return { tColor.r / 255.f, tColor.g / 255.f, tColor.b / 255.f, tColor.a / 255.f };
		};

	Accent = ColorToVec(Vars::Menu::Theme::Accent.Value);
	// the reference-style two-tone: near-black window with distinctly lighter group-box bodies and crisp borders
	Background0 = ColorToVec(Vars::Menu::Theme::Background.Value);
	Background0p5 = ColorToVec(Vars::Menu::Theme::Background.Value.Lerp({ 127, 127, 127 }, 1.8f / 9, LerpEnum::NoAlpha));
	Background1 = ColorToVec(Vars::Menu::Theme::Background.Value.Lerp({ 127, 127, 127 }, 1.f / 9, LerpEnum::NoAlpha));
	Background1p5 = ColorToVec(Vars::Menu::Theme::Background.Value.Lerp({ 127, 127, 127 }, 1.5f / 9, LerpEnum::NoAlpha));
	Background1p5L = { Background1p5.Value.x * 1.1f, Background1p5.Value.y * 1.1f, Background1p5.Value.z * 1.1f, Background1p5.Value.w };
	Background2 = ColorToVec(Vars::Menu::Theme::Background.Value.Lerp({ 127, 127, 127 }, 2.8f / 9, LerpEnum::NoAlpha));
	Inactive = ColorToVec(Vars::Menu::Theme::Inactive.Value);
	Active = ColorToVec(Vars::Menu::Theme::Active.Value);

	ImVec4* colors = GetStyle().Colors;
	colors[ImGuiCol_Border] = Background2;
	colors[ImGuiCol_Button] = {};
	colors[ImGuiCol_ButtonHovered] = {};
	colors[ImGuiCol_ButtonActive] = {};
	colors[ImGuiCol_FrameBg] = Background1p5;
	colors[ImGuiCol_FrameBgHovered] = Background1p5L;
	colors[ImGuiCol_FrameBgActive] = Background1p5;
	colors[ImGuiCol_Header] = {};
	colors[ImGuiCol_HeaderHovered] = { Background1p5L.Value.x * 1.1f, Background1p5L.Value.y * 1.1f, Background1p5L.Value.z * 1.1f, Background1p5.Value.w }; // divd by 1.1
	colors[ImGuiCol_HeaderActive] = Background1p5;
	colors[ImGuiCol_ModalWindowDimBg] = { Background0.Value.x, Background0.Value.y, Background0.Value.z, 0.4f };
	colors[ImGuiCol_PopupBg] = Background1p5L;
	colors[ImGuiCol_ResizeGrip] = {};
	colors[ImGuiCol_ResizeGripActive] = {};
	colors[ImGuiCol_ResizeGripHovered] = {};
	// The highlight line ImGui draws along a window edge while it can be resized (the "drag width/height"
	colors[ImGuiCol_SeparatorHovered] = Accent;
	colors[ImGuiCol_SeparatorActive] = Accent;
	colors[ImGuiCol_ScrollbarBg] = {};
	colors[ImGuiCol_Text] = Active;
	colors[ImGuiCol_WindowBg] = {};
}

// --- Menu font resolution -------------------------------------------------------------------------
namespace
{
	bool FontFileExists(const std::string& s) { return !s.empty() && GetFileAttributesA(s.c_str()) != INVALID_FILE_ATTRIBUTES; }

	std::string LowerStr(std::string s)
	{
		for (char& c : s) if (c >= 'A' && c <= 'Z') c = char(c - 'A' + 'a');
		return s;
	}

	// System + per-user Fonts folders (the set GDI resolves installed fonts against).
	std::vector<std::string> FontDirs()
	{
		std::vector<std::string> v;
		char szWin[MAX_PATH] = {};
		if (GetWindowsDirectoryA(szWin, MAX_PATH)) v.emplace_back(std::string(szWin) + "\\Fonts\\");
		char szLocal[MAX_PATH] = {};
		if (GetEnvironmentVariableA("LOCALAPPDATA", szLocal, MAX_PATH)) v.emplace_back(std::string(szLocal) + "\\Microsoft\\Windows\\Fonts\\");
		return v;
	}

	// Registry entries store either a bare file name (system fonts, relative to the Windows Fonts
	// dir) or a full path (per-user fonts). Resolve either to an existing full path, or "".
	std::string ResolveFontFile(const std::string& sFile)
	{
		if (sFile.empty()) return "";
		if (sFile.find('\\') != std::string::npos || sFile.find('/') != std::string::npos || sFile.find(':') != std::string::npos)
			return FontFileExists(sFile) ? sFile : "";
		for (const auto& d : FontDirs())
		{
			std::string p = d + sFile;
			if (FontFileExists(p)) return p;
		}
		return "";
	}

	// Search one Fonts registry key for a family. bBold prefers a "<family> bold/semibold" face,
	// otherwise a non-bold face. Returns the resolved full file path or "".
	std::string FindFontInRegistry(HKEY hRoot, const std::string& sFamilyLower, bool bBold)
	{
		HKEY hKey;
		if (RegOpenKeyExA(hRoot, R"(SOFTWARE\Microsoft\Windows NT\CurrentVersion\Fonts)", 0, KEY_READ, &hKey) != ERROR_SUCCESS)
			return "";

		std::string sExact, sPrefix, sFallback;
		char szName[512]; BYTE abData[1024]; DWORD i = 0;
		for (;;)
		{
			DWORD cName = (DWORD)sizeof(szName), cData = (DWORD)sizeof(abData), dwType = 0;
			const LONG r = RegEnumValueA(hKey, i++, szName, &cName, nullptr, &dwType, abData, &cData);
			if (r == ERROR_NO_MORE_ITEMS) break;
			if (r != ERROR_SUCCESS || dwType != REG_SZ) continue;

			const std::string sFile(reinterpret_cast<char*>(abData));
			std::string sDisp = LowerStr(std::string(szName, cName));
			if (const size_t p = sDisp.rfind(" ("); p != std::string::npos) sDisp.erase(p); // strip " (TrueType)" etc
			while (!sDisp.empty() && sDisp.back() == ' ') sDisp.pop_back();
			if (sDisp.rfind(sFamilyLower, 0) != 0) continue; // must start with the family name

			const bool bHasBold = sDisp.find("bold") != std::string::npos || sDisp.find("semibold") != std::string::npos;
			if (bBold)
			{
				if (sDisp == sFamilyLower + " bold" && sExact.empty()) sExact = sFile;
				else if (bHasBold && sPrefix.empty()) sPrefix = sFile;
				else if (sFallback.empty()) sFallback = sFile;
			}
			else
			{
				if (sDisp == sFamilyLower && sExact.empty()) sExact = sFile;
				else if (!bHasBold && sPrefix.empty()) sPrefix = sFile;
				else if (sFallback.empty()) sFallback = sFile;
			}
		}
		RegCloseKey(hKey);

		for (const std::string* s : { &sExact, &sPrefix, &sFallback })
			if (!s->empty())
			{
				std::string p = ResolveFontFile(*s);
				if (!p.empty()) return p;
			}
		return "";
	}

	// Resolve a typed name to regular + bold file paths. Tries a file name/stem in the font folders
	bool ResolveMenuFont(const std::string& sQuery, std::string& sRegular, std::string& sBold)
	{
		sRegular.clear(); sBold.clear();
		if (sQuery.empty()) return false;

		// 1) File name / stem directly in the font directories.
		{
			const size_t iDot = sQuery.find_last_of('.');
			const bool bHasExt = iDot != std::string::npos && iDot > 0;
			const std::string sExt = bHasExt ? sQuery.substr(iDot) : std::string();
			const std::string sStem = bHasExt ? sQuery.substr(0, iDot) : sQuery;
			static const char* const kExts[] = { ".ttf", ".otf", ".ttc" };
			for (const auto& d : FontDirs())
			{
				if (bHasExt) { std::string p = d + sStem + sExt; if (FontFileExists(p)) sRegular = p; }
				else for (const char* e : kExts) { std::string p = d + sStem + e; if (FontFileExists(p)) { sRegular = p; break; } }
				if (sRegular.empty()) continue;

				const std::string e = sRegular.substr(sRegular.find_last_of('.'));
				for (const char* szSuffix : { "b", "bd", "-bold", " bold", "bold" })
				{
					std::string p = d + sStem + szSuffix + e;
					if (FontFileExists(p)) { sBold = p; break; }
				}
				if (sBold.empty()) sBold = sRegular;
				return true;
			}
		}

		// 2) Installed family name via the Fonts registry (per-user first, then per-machine).
		const std::string sFam = LowerStr(sQuery);
		for (HKEY root : { HKEY_CURRENT_USER, HKEY_LOCAL_MACHINE })
		{
			std::string reg = FindFontInRegistry(root, sFam, false);
			if (!reg.empty())
			{
				sRegular = reg;
				std::string regBold = FindFontInRegistry(root, sFam, true);
				sBold = regBold.empty() ? reg : regBold;
				return true;
			}
		}
		return false;
	}
}

void CRender::LoadFonts()
{
	static bool bHasLoaded = false;

	auto& io = ImGui::GetIO();
	if (bHasLoaded)
	{
		ImGui_ImplDX9_InvalidateDeviceObjects();
		io.Fonts->ClearFonts();
	}

	ImFontConfig fontConfig;
	fontConfig.OversampleH = 2;
	// Basic Latin + Latin Supplement + Cyrillic + Hiragana + Katakana + Halfwidth/Fullwidth Forms
	constexpr ImWchar fontRange[]{ 0x0020, 0x00FF, 0x0400, 0x044F, 0x3040, 0x30FF, 0xFF00, 0xFFEF, 0 };

	// Japanese kana merge: use MS Gothic if present (ships with Windows in all locales)
	static const char* const kJpFont = R"(C:\Windows\Fonts\msgothic.ttc)";
	const bool bJpFont = (GetFileAttributesA(kJpFont) != INVALID_FILE_ATTRIBUTES);
	ImFontConfig mergeCfg;
	mergeCfg.MergeMode = true;
	mergeCfg.OversampleH = 2;
	constexpr ImWchar kanaRange[]{ 0x3040, 0x30FF, 0xFF00, 0xFFEF, 0 };

	// Media-player overlay font: bake at the actual pixel size (base 13 * MediaPlayerScale) so the text
	// is rasterized crisp instead of stretching the 13px bitmap. Render() rebuilds the atlas when the scale changes.
	const float flMediaScale = std::clamp(Vars::Menu::MediaPlayerScale.Value, 0.5f, 2.5f);
	// User font-size multiplier (Visuals > MENU > Settings "Font scale") applied on top of the menu
	// scale to every baked ImGui font. Render() rebuilds the atlas when it changes.
	const float flFontScale = std::clamp(Vars::Menu::Style::FontScale.Value, 0.5f, 2.f);
	auto FS = [&](float flBase) { return H::Draw.Scale(flBase) * flFontScale; };
	const float flMediaSize = H::Draw.Scale(13) * flMediaScale * flFontScale;

	// Font anti-aliasing toggle (Visuals > MENU > Settings). Off = crisp monochrome glyphs (FreeType
	// builds only; the stb builds always antialias). Applied to fontConfig below in the FreeType branches.
	const bool bFontAA = Vars::Menu::Style::FontAntiAlias.Value;

	// ---- Optional user-selected menu font (Visuals > MENU > Settings "Main menu font") ----
	// Accepts an installed font's family name ("Tahoma", "Segoe UI", "Comic Sans MS") OR a file
	const int iPreset = Vars::Menu::Style::FontPreset.Value;
	std::string sUserFont = Vars::Menu::Style::Font.Value;
	sUserFont.erase(0, sUserFont.find_first_not_of(" \t\r\n"));
	if (const size_t iEnd = sUserFont.find_last_not_of(" \t\r\n"); iEnd != std::string::npos)
		sUserFont.erase(iEnd + 1);
	std::string sFontFile, sFontBoldFile;
	if (iPreset == Vars::Menu::Style::FontPresetEnum::Custom)
		ResolveMenuFont(sUserFont, sFontFile, sFontBoldFile);

	if (iPreset == Vars::Menu::Style::FontPresetEnum::Custom && !sFontFile.empty())
	{
		fontConfig.OversampleH = bFontAA ? 2 : 1;
		fontConfig.OversampleV = bFontAA ? 2 : 1;
		fontConfig.PixelSnapH = !bFontAA;
#ifdef ALETHERIUM_CUSTOM_FONTS
		fontConfig.FontBuilderFlags = bFontAA ? ImGuiFreeTypeBuilderFlags_LightHinting
			: (ImGuiFreeTypeBuilderFlags_Monochrome | ImGuiFreeTypeBuilderFlags_MonoHinting); // crisp non-AA
#endif
		FontSmall = io.Fonts->AddFontFromFileTTF(sFontFile.c_str(), FS(12), &fontConfig, fontRange);
		if (bJpFont) { ImFontConfig m = mergeCfg; io.Fonts->AddFontFromFileTTF(kJpFont, FS(12), &m, kanaRange); }
		FontRegular = io.Fonts->AddFontFromFileTTF(sFontFile.c_str(), FS(13), &fontConfig, fontRange);
		if (bJpFont) { ImFontConfig m = mergeCfg; io.Fonts->AddFontFromFileTTF(kJpFont, FS(13), &m, kanaRange); }
		FontBold = io.Fonts->AddFontFromFileTTF(sFontBoldFile.c_str(), FS(13), &fontConfig, fontRange);
		if (bJpFont) { ImFontConfig m = mergeCfg; io.Fonts->AddFontFromFileTTF(kJpFont, FS(13), &m, kanaRange); }
		FontLarge = io.Fonts->AddFontFromFileTTF(sFontFile.c_str(), FS(14), &fontConfig, fontRange);
		if (bJpFont) { ImFontConfig m = mergeCfg; io.Fonts->AddFontFromFileTTF(kJpFont, FS(14), &m, kanaRange); }
		FontMono = io.Fonts->AddFontFromFileTTF(sFontFile.c_str(), FS(13), &fontConfig, fontRange);
		if (bJpFont) { ImFontConfig m = mergeCfg; io.Fonts->AddFontFromFileTTF(kJpFont, FS(13), &m, kanaRange); }
		FontMedia = io.Fonts->AddFontFromFileTTF(sFontBoldFile.c_str(), flMediaSize, &fontConfig, fontRange);
		if (bJpFont) { ImFontConfig m = mergeCfg; io.Fonts->AddFontFromFileTTF(kJpFont, flMediaSize, &m, kanaRange); }
	}
	else
	{
		// Embedded menu-font preset. Each is a regular + semibold face baked straight from memory
		// (subset to Latin+Cyrillic, see tools/font_to_header.py). Custom-but-unresolved falls back
		const unsigned char* pReg = Kodchasan_Regular;  int iRegSz = sizeof(Kodchasan_Regular);
		const unsigned char* pBold = Kodchasan_Bold; int iBoldSz = sizeof(Kodchasan_Bold);
		switch (iPreset)
		{
		case Vars::Menu::Style::FontPresetEnum::SFProDisplay:
			pReg = SFProDisplayRegular;  iRegSz = sizeof(SFProDisplayRegular);
			pBold = SFProDisplaySemibold; iBoldSz = sizeof(SFProDisplaySemibold); break;
		case Vars::Menu::Style::FontPresetEnum::SNPro:
			pReg = SNPro_Regular;  iRegSz = sizeof(SNPro_Regular);
			pBold = SNPro_Bold; iBoldSz = sizeof(SNPro_Bold); break;
		case Vars::Menu::Style::FontPresetEnum::OpenSans:
			pReg = OpenSans_Regular;  iRegSz = sizeof(OpenSans_Regular);
			pBold = OpenSans_Bold; iBoldSz = sizeof(OpenSans_Bold); break;
		case Vars::Menu::Style::FontPresetEnum::Montserrat:
			pReg = Montserrat_Regular;  iRegSz = sizeof(Montserrat_Regular);
			pBold = Montserrat_Bold; iBoldSz = sizeof(Montserrat_Bold); break;
		default: break; // Kodchasan / Custom-unresolved
		}
		fontConfig.OversampleH = bFontAA ? 2 : 1;
		fontConfig.OversampleV = bFontAA ? 2 : 1;
		fontConfig.PixelSnapH = !bFontAA;
		fontConfig.FontDataOwnedByAtlas = false;
		fontConfig.RasterizerMultiply = 1.0f;
#ifdef ALETHERIUM_CUSTOM_FONTS
		fontConfig.FontBuilderFlags = bFontAA ? ImGuiFreeTypeBuilderFlags_LightHinting
			: (ImGuiFreeTypeBuilderFlags_Monochrome | ImGuiFreeTypeBuilderFlags_MonoHinting); // crisp non-AA
#endif
		mergeCfg.OversampleH = bFontAA ? 2 : 1;
		mergeCfg.OversampleV = bFontAA ? 2 : 1;
		mergeCfg.PixelSnapH = !bFontAA;
		FontSmall = io.Fonts->AddFontFromMemoryTTF(const_cast<unsigned char*>(pReg), iRegSz, FS(12), &fontConfig, fontRange);
		if (bJpFont) { ImFontConfig m = mergeCfg; io.Fonts->AddFontFromFileTTF(kJpFont, FS(12), &m, kanaRange); }
		FontRegular = io.Fonts->AddFontFromMemoryTTF(const_cast<unsigned char*>(pReg), iRegSz, FS(13), &fontConfig, fontRange);
		if (bJpFont) { ImFontConfig m = mergeCfg; io.Fonts->AddFontFromFileTTF(kJpFont, FS(13), &m, kanaRange); }
		FontBold = io.Fonts->AddFontFromMemoryTTF(const_cast<unsigned char*>(pBold), iBoldSz, FS(13), &fontConfig, fontRange);
		if (bJpFont) { ImFontConfig m = mergeCfg; io.Fonts->AddFontFromFileTTF(kJpFont, FS(13), &m, kanaRange); }
		FontLarge = io.Fonts->AddFontFromMemoryTTF(const_cast<unsigned char*>(pReg), iRegSz, FS(15), &fontConfig, fontRange);
		if (bJpFont) { ImFontConfig m = mergeCfg; io.Fonts->AddFontFromFileTTF(kJpFont, FS(15), &m, kanaRange); }
		FontMono = io.Fonts->AddFontFromMemoryTTF(const_cast<unsigned char*>(pReg), iRegSz, FS(13), &fontConfig, fontRange);
		if (bJpFont) { ImFontConfig m = mergeCfg; io.Fonts->AddFontFromFileTTF(kJpFont, FS(13), &m, kanaRange); }
		FontMedia = io.Fonts->AddFontFromMemoryTTF(const_cast<unsigned char*>(pBold), iBoldSz, flMediaSize, &fontConfig, fontRange);
		if (bJpFont) { ImFontConfig m = mergeCfg; io.Fonts->AddFontFromFileTTF(kJpFont, flMediaSize, &m, kanaRange); }
	}

	ImFontConfig iconConfig;
	iconConfig.PixelSnapH = true;
	constexpr ImWchar iconRange[]{ short(ICON_MIN_MD), short(ICON_MAX_MD), 0 };
	IconFont = io.Fonts->AddFontFromMemoryCompressedTTF(MaterialIcons_compressed_data, MaterialIcons_compressed_size, H::Draw.Scale(16), &iconConfig, iconRange);

	io.Fonts->Build();
	io.ConfigDebugHighlightIdConflicts = false;

	bHasLoaded = true;
}

void CRender::LoadStyle()
{
	using namespace ImGui;

	auto& style = GetStyle();
	style.ButtonTextAlign = { 0.5f, 0.5f }; // Center button text
	style.CellPadding = { H::Draw.Scale(4), 0 };
	style.ChildBorderSize = 0.f;
	style.ChildRounding = 0.f; // the reference-style square corners
	style.FrameBorderSize = 0.f;
	style.FramePadding = { 0, 0 };
	style.FrameRounding = 0.f; // the reference-style square corners
	style.ItemInnerSpacing = { 0, 0 };
	style.ItemSpacing = { H::Draw.Scale(8), H::Draw.Scale(4) }; // the reference-style tighter vertical row gap
	style.PopupBorderSize = 0.f;
	style.PopupRounding = 0.f; // the reference-style square corners
	style.ScrollbarSize = 6.f + H::Draw.Scale(3);
	style.ScrollbarRounding = 0.f;
	style.WindowBorderSize = 0.f;
	style.WindowPadding = { 0, 0 };
	style.WindowRounding = 0.f; // the reference-style square corners
}

// Shared SVG -> alpha-mask texture path. Rasterizes svgText (any nul-terminated SVG, of byte
// length nLen) into a high-res square canvas, keeps only the coverage as alpha, and forces RGB
static IDirect3DTexture9* RasterizeSvgToAlphaTex(IDirect3DDevice9* pDevice, const char* svgText, size_t nLen)
{
	if (!pDevice || !svgText || !nLen) return nullptr;

	// Make a writable, nul-terminated copy — nsvgParse modifies the input buffer in-place.
	std::vector<char> buf(svgText, svgText + nLen);
	buf.push_back('\0');

	NSVGimage* pImg = nsvgParse(buf.data(), "px", 96.f);
	if (!pImg) return nullptr;
	if (pImg->width <= 0.f || pImg->height <= 0.f) { nsvgDelete(pImg); return nullptr; }

	// High-resolution supersample so the shape stays crisp (and antialiased by nanosvg's edge
	// coverage) when minified to the small on-screen logo/watermark size.
	constexpr int kSz = 256;
	NSVGrasterizer* pRast = nsvgCreateRasterizer();
	if (!pRast) { nsvgDelete(pImg); return nullptr; }

	// Aspect-fit the SVG into the square canvas (matches the Logo_RasterizeSVG).
	const float scaleX = static_cast<float>(kSz) / pImg->width;
	const float scaleY = static_cast<float>(kSz) / pImg->height;
	const float scale = (scaleX < scaleY) ? scaleX : scaleY;

	std::vector<unsigned char> rgba(kSz * kSz * 4);
	nsvgRasterize(pRast, pImg, 0, 0, scale, rgba.data(), kSz, kSz, kSz * 4);
	nsvgDeleteRasterizer(pRast);
	nsvgDelete(pImg);

	IDirect3DTexture9* pStage = nullptr;
	if (FAILED(pDevice->CreateTexture(kSz, kSz, 1, 0, D3DFMT_A8R8G8B8, D3DPOOL_SYSTEMMEM, &pStage, nullptr)))
		return nullptr;

	D3DLOCKED_RECT lr{};
	if (FAILED(pStage->LockRect(0, &lr, nullptr, 0))) { pStage->Release(); return nullptr; }
	{
		const int iRowPitch = lr.Pitch / 4;
		auto* pDst = static_cast<unsigned int*>(lr.pBits);
		for (int y = 0; y < kSz; ++y)
			for (int x = 0; x < kSz; ++x)
			{
				const unsigned char a = rgba[(y * kSz + x) * 4 + 3]; // coverage mask
				// ARGB, RGB forced white so the accent/white tint passed to AddImage() shows.
				pDst[y * iRowPitch + x] = (static_cast<unsigned int>(a) << 24) | 0x00FFFFFFu;
			}
	}
	pStage->UnlockRect(0);

	IDirect3DTexture9* pTex = nullptr;
	if (FAILED(pDevice->CreateTexture(kSz, kSz, 0, D3DUSAGE_AUTOGENMIPMAP, D3DFMT_A8R8G8B8, D3DPOOL_DEFAULT, &pTex, nullptr)))
	{
		pStage->Release();
		return nullptr;
	}

	const HRESULT hrUpdate = pDevice->UpdateTexture(pStage, pTex);
	pStage->Release();
	if (FAILED(hrUpdate)) { pTex->Release(); return nullptr; }

	pTex->GenerateMipSubLevels();
	return pTex;
}

// Steam avatar (RGBA, row-major) -> a DEFAULT-pool A8R8G8B8 D3D9 texture for ImGui AddImageRounded.
static IDirect3DTexture9* CreateAvatarTexD3D(IDirect3DDevice9* pDevice, const uint8_t* pRGBA, uint32_t w, uint32_t h)
{
	if (!pDevice || !pRGBA || !w || !h)
		return nullptr;

	// Proven pattern (same as RasterizeSvgToAlphaTex / album art): SYSTEMMEM staging -> DEFAULT pool ->
	// UpdateTexture. The old MANAGED + direct-LockRect texture never rendered in the ImGui DX9 pass.
	IDirect3DTexture9* pStage = nullptr;
	if (FAILED(pDevice->CreateTexture(w, h, 1, 0, D3DFMT_A8R8G8B8, D3DPOOL_SYSTEMMEM, &pStage, nullptr)))
		return nullptr;

	D3DLOCKED_RECT lr{};
	if (FAILED(pStage->LockRect(0, &lr, nullptr, 0))) { pStage->Release(); return nullptr; }
	for (uint32_t y = 0; y < h; ++y)
	{
		auto* pRow = reinterpret_cast<uint32_t*>(static_cast<uint8_t*>(lr.pBits) + size_t(y) * lr.Pitch);
		for (uint32_t x = 0; x < w; ++x)
		{
			const uint8_t* p = pRGBA + (size_t(y) * w + x) * 4;
			pRow[x] = (uint32_t(p[3]) << 24) | (uint32_t(p[0]) << 16) | (uint32_t(p[1]) << 8) | uint32_t(p[2]);
		}
	}
	pStage->UnlockRect(0);

	IDirect3DTexture9* pTex = nullptr;
	if (FAILED(pDevice->CreateTexture(w, h, 1, 0, D3DFMT_A8R8G8B8, D3DPOOL_DEFAULT, &pTex, nullptr)))
	{
		pStage->Release();
		return nullptr;
	}

	const HRESULT hrUpdate = pDevice->UpdateTexture(pStage, pTex);
	pStage->Release();
	if (FAILED(hrUpdate)) { pTex->Release(); return nullptr; }

	return pTex;
}

IDirect3DTexture9* CRender::GetAvatarTex(uint32_t iAccountID)
{
	// TEMP diagnostic: log each account id's outcome once (or when it changes). Remove once avatars confirmed.
	static std::unordered_map<uint32_t, int> s_mLogged;
	auto Log = [&](int iState, const char* sMsg)
		{
			if (auto it = s_mLogged.find(iAccountID); it != s_mLogged.end() && it->second == iState)
				return;
			s_mLogged[iAccountID] = iState;
			SDK::Output("Avatar", std::format("id={} {}", iAccountID, sMsg).c_str(), { 255, 200, 100 }, OUTPUT_DEBUG);
		};

	if (!iAccountID || !m_pCurDevice || !I::SteamFriends || !I::SteamUtils)
	{
		Log(1, std::format("guard fail (id={} dev={} fr={} ut={})", iAccountID != 0, m_pCurDevice != nullptr, I::SteamFriends != nullptr, I::SteamUtils != nullptr).c_str());
		return nullptr;
	}

	if (auto it = m_mAvatarTex.find(iAccountID); it != m_mAvatarTex.end())
		return it->second;

	// Steam loads avatars asynchronously; the handle is 0 until ready. Don't cache misses - retry next
	// frame until the image is available.
	const CSteamID tSteamID(iAccountID, k_EUniversePublic, k_EAccountTypeIndividual);
	const int iAvatar = I::SteamFriends->GetMediumFriendAvatar(tSteamID);
	if (iAvatar <= 0)
	{
		// Non-friends' avatars aren't cached locally - ask Steam to download it (see ISteamFriends note).
		// Pops in on a later frame once the PersonaStateChanged callback lands.
		I::SteamFriends->RequestUserInformation(tSteamID, false);
		Log(2, "GetMediumFriendAvatar <= 0 (requested)");
		return nullptr;
	}

	uint32 w = 0, h = 0;
	if (!I::SteamUtils->GetImageSize(iAvatar, &w, &h) || !w || !h)
	{
		Log(3, "GetImageSize fail");
		return nullptr;
	}

	std::vector<uint8_t> vData(size_t(w) * h * 4);
	if (!I::SteamUtils->GetImageRGBA(iAvatar, vData.data(), int(vData.size())))
	{
		Log(4, "GetImageRGBA fail");
		return nullptr;
	}

	IDirect3DTexture9* pTex = CreateAvatarTexD3D(m_pCurDevice, vData.data(), w, h);
	if (pTex)
		m_mAvatarTex[iAccountID] = pTex;
	Log(pTex ? 5 : 6, pTex ? std::format("OK tex created {}x{}", w, h).c_str() : "CreateAvatarTexD3D returned null");
	return pTex;
}

void CRender::ReleaseAvatarTextures()
{
	for (auto& [id, pTex] : m_mAvatarTex)
		if (pTex) pTex->Release();
	m_mAvatarTex.clear();
}

void CRender::ReleaseLogoSvgs()
{
	for (auto& pTex : m_pLogoTex)
		if (pTex) { pTex->Release(); pTex = nullptr; }
	if (m_pLogoCustomTex) { m_pLogoCustomTex->Release(); m_pLogoCustomTex = nullptr; }
	m_sLogoCustomLast.clear();
}

void CRender::LoadLogoSvg(IDirect3DDevice9* pDevice)
{
	ReleaseLogoSvgs();
	if (!pDevice) return;
	// chud.svg has no fill attribute, so nanosvg rasterizes it solid black.
	m_pLogoTex[0] = RasterizeSvgToAlphaTex(pDevice, chud_svg_data, chud_svg_size); // enum 0 == Chud
	// The remaining built-ins (Star of David, Star, Heart, Clarity) come from logo_svgs.h, mapped
	// to enum values 1.. (index i here == enum value i + 1).
	const int iCount = ImMin(g_iLogoGlyphCount, kBuiltinLogoCount - 1);
	for (int i = 0; i < iCount; ++i)
		m_pLogoTex[i + 1] = RasterizeSvgToAlphaTex(pDevice, g_LogoGlyphs[i].svg, strlen(g_LogoGlyphs[i].svg));
}

void CRender::UpdateCustomLogo(IDirect3DDevice9* pDevice)
{
	// Only the "Custom" style needs the pasted SVG; re-rasterize whenever the source string changes
	// (rare, so cheap). Invalid/empty markup leaves m_pLogoCustomTex null -> the logo simply hides.
	if (Vars::Menu::LogoStyle.Value != Vars::Menu::LogoStyleEnum::LogoCustom)
		return;
	const std::string& sSvg = Vars::Menu::LogoCustomSvg.Value;
	if (sSvg == m_sLogoCustomLast)
		return;
	m_sLogoCustomLast = sSvg;
	if (m_pLogoCustomTex) { m_pLogoCustomTex->Release(); m_pLogoCustomTex = nullptr; }
	if (!sSvg.empty() && pDevice)
		m_pLogoCustomTex = RasterizeSvgToAlphaTex(pDevice, sSvg.c_str(), sSvg.size());
}

IDirect3DTexture9* CRender::GetActiveLogoTex()
{
	const int iSel = Vars::Menu::LogoStyle.Value;
	if (iSel == Vars::Menu::LogoStyleEnum::LogoCustom)
		return m_pLogoCustomTex;
	if (iSel >= 0 && iSel < kBuiltinLogoCount)
		return m_pLogoTex[iSel];
	return m_pLogoTex[0];
}

void CRender::ReleaseSeparatorSvgs()
{
	for (auto& pTex : m_pSepTex)
		if (pTex) { pTex->Release(); pTex = nullptr; }
	if (m_pSepCustomTex) { m_pSepCustomTex->Release(); m_pSepCustomTex = nullptr; }
	m_sSepCustomLast.clear();
}

void CRender::UpdateCustomSeparator(IDirect3DDevice9* pDevice)
{
	// Only the "Custom" separator glyph needs the pasted SVG; re-rasterize whenever the source string
	// changes (rare). Invalid/empty markup leaves m_pSepCustomTex null -> the separator falls back to text.
	if (Vars::Menu::WmSepGlyph.Value != Vars::Menu::WmSepGlyphEnum::WmSepCustom)
		return;
	const std::string& sSvg = Vars::Menu::WmSepCustomSvg.Value;
	if (sSvg == m_sSepCustomLast)
		return;
	m_sSepCustomLast = sSvg;
	if (m_pSepCustomTex) { m_pSepCustomTex->Release(); m_pSepCustomTex = nullptr; }
	if (!sSvg.empty() && pDevice)
		m_pSepCustomTex = RasterizeSvgToAlphaTex(pDevice, sSvg.c_str(), sSvg.size());
}

void CRender::LoadSeparatorSvgs(IDirect3DDevice9* pDevice)
{
	ReleaseSeparatorSvgs();
	if (!pDevice) return;

	const int iCount = ImMin(g_iWmSepGlyphCount, kMaxSepGlyphs);
	for (int i = 0; i < iCount; ++i)
		m_pSepTex[i] = RasterizeSvgToAlphaTex(pDevice, g_WmSepGlyphs[i].svg, strlen(g_WmSepGlyphs[i].svg));
}

void CRender::Initialize(IDirect3DDevice9* pDevice)
{
	// Initialize ImGui and device
	ImGui::CreateContext();
	ImGui_ImplWin32_Init(WndProc::hwWindow);
	ImGui_ImplDX9_Init(pDevice);

	auto& io = ImGui::GetIO();
	//io.IniFilename = nullptr;
	io.LogFilename = nullptr;

	LoadFonts();
	LoadStyle();

	// Set device for MediaPlayer (old simple way)
	Update(pDevice);
	LoadLogoSvg(pDevice);
	LoadSeparatorSvgs(pDevice);
}