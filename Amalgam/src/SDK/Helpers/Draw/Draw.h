#pragma once
#include "../Fonts/Fonts.h"
#include "../../Definitions/Definitions.h"
#include "../../Vars.h"

MAKE_SIGNATURE(InitializeStandardMaterials, "engine.dll", "48 83 EC ? 80 3D ? ? ? ? ? 0F 85 ? ? ? ? 48 89 5C 24 ? B9 ? ? ? ? 48 89 7C 24 ? C6 05", 0x0);
MAKE_SIGNATURE(Wireframe, "engine.dll", "48 89 05 ? ? ? ? E8 ? ? ? ? 48 85 C0 74 ? 48 8D 15 ? ? ? ? 48 8B C8 E8 ? ? ? ? 48 8B F8 EB ? 48 8B FB 41 B8 ? ? ? ? 48 8D 15 ? ? ? ? 48 8B CF E8 ? ? ? ? 41 B8 ? ? ? ? 48 8D 15 ? ? ? ? 48 8B CF E8 ? ? ? ? 41 B8", 0x0);
MAKE_SIGNATURE(WireframeIgnoreZ, "engine.dll", "48 89 05 ? ? ? ? E8 ? ? ? ? 48 85 C0 74 ? 48 8D 15 ? ? ? ? 48 8B C8 E8 ? ? ? ? 48 8B F8 EB ? 48 8B FB 41 B8 ? ? ? ? 48 8D 15 ? ? ? ? 48 8B CF E8 ? ? ? ? 41 B8 ? ? ? ? 48 8D 15 ? ? ? ? 48 8B CF E8 ? ? ? ? 48 8B 0D", 0x0);
MAKE_SIGNATURE(VertexColor, "engine.dll", "48 89 05 ? ? ? ? E8 ? ? ? ? 48 8B 7C 24", 0x0);
MAKE_SIGNATURE(VertexColorIgnoreZ, "engine.dll", "48 89 05 ? ? ? ? 48 83 C4 ? C3 CC CC CC CC CC CC CC CC CC 48 8B C4", 0x0);

// Per-layer accessor for the stacked panel outlines (Vars::Menu::Style). Returns false when i is
// outside the 4 fixed layers. Centralizes the layer CVars so the ImGui path (DrawStyledBackground)
// and the surface path (CDraw::PanelOutline) read identical settings. Layers stack outermost.
inline bool GetStyleOutlineLayer(int i, Color_t& tColor, float& flThick, float& flBlur)
{
	switch (i)
	{
	case 0: tColor = Vars::Menu::Style::OutlineColor1.Value; flThick = Vars::Menu::Style::OutlineThickness1.Value; flBlur = Vars::Menu::Style::OutlineBlur1.Value; return true;
	case 1: tColor = Vars::Menu::Style::OutlineColor2.Value; flThick = Vars::Menu::Style::OutlineThickness2.Value; flBlur = Vars::Menu::Style::OutlineBlur2.Value; return true;
	case 2: tColor = Vars::Menu::Style::OutlineColor3.Value; flThick = Vars::Menu::Style::OutlineThickness3.Value; flBlur = Vars::Menu::Style::OutlineBlur3.Value; return true;
	case 3: tColor = Vars::Menu::Style::OutlineColor4.Value; flThick = Vars::Menu::Style::OutlineThickness4.Value; flBlur = Vars::Menu::Style::OutlineBlur4.Value; return true;
	default: return false;
	}
}

enum EAlign
{
	ALIGN_TOPLEFT,
	ALIGN_TOP,
	ALIGN_TOPRIGHT,
	ALIGN_LEFT,
	ALIGN_CENTER,
	ALIGN_RIGHT,
	ALIGN_BOTTOMLEFT,
	ALIGN_BOTTOM,
	ALIGN_BOTTOMRIGHT
};

enum Scale_
{
	Scale_None = 0,
	Scale_Round = 1,
	Scale_Floor = 2,
	Scale_Ceil = 3
};

class CDraw
{
private:
	std::unordered_map<uint32, int> m_mAvatars = {};
	int GetAvatarTexture(const uint32 nFriendID); // cached steam avatar texture id, -1 if unavailable

public:
	inline float Scale(float flN = 1.f, int iFlags = Scale_None, float flScale = Vars::Menu::Scale.Value)
	{
		flN *= flScale;
		switch (iFlags)
		{
		case Scale_Round: flN = roundf(flN); break;
		case Scale_Floor: flN = floorf(flN); break;
		case Scale_Ceil: flN = ceilf(flN); break;
		}
		return flN;
	}

	inline float Unscale(float flN = 1.f, int iFlags = Scale_None, float flScale = Vars::Menu::Scale.Value)
	{
		flN /= flScale;
		switch (iFlags)
		{
		case Scale_Round: flN = roundf(flN); break;
		case Scale_Floor: flN = floorf(flN); break;
		case Scale_Ceil: flN = ceilf(flN); break;
		}
		return flN;
	}

	void Start(bool bBadFontCheck = false);
	void End();
	void StartClipping(int x, int y, int w, int h);
	void EndClipping();

	void UpdateScreenSize();
	void UpdateW2SMatrix();

	Vec2 GetTextSize(const char* text, const Font_t& tFont);
	Vec2 GetTextSize(const wchar_t* text, const Font_t& tFont);

	void String(const Font_t& tFont, int x, int y, Color_t tColor, EAlign eAlign, const char* str);
	void String(const Font_t& tFont, int x, int y, Color_t tColor, EAlign eAlign, const wchar_t* wstr);
	// iStyle: -1 = default (single drop if CheapText, else full outline), 0 = shadow (single offset), 1 = full outline.
	void StringOutlined(const Font_t& tFont, int x, int y, Color_t tColor, Color_t tColorOut, EAlign eAlign, const char* str, bool bAlpha = true, int iStyle = -1);
	void StringOutlined(const Font_t& tFont, int x, int y, Color_t tColor, Color_t tColorOut, EAlign eAlign, const wchar_t* wstr, bool bAlpha = true, int iStyle = -1);

	void Line(int x1, int y1, int x2, int y2, Color_t tColor);
	void LineThick(int x1, int y1, int x2, int y2, Color_t tColor, int iThickness); // stamps parallel 1px lines
	void FillPolygon(std::vector<Vertex_t> vVertices, Color_t tColor);
	void LinePolygon(std::vector<Vertex_t> vVertices, Color_t tColor);

	void FillRect(int x, int y, int w, int h, Color_t tColor);
	void LineRect(int x, int y, int w, int h, Color_t tColor);
	void GradientRect(int x, int y, int w, int h, Color_t tColorTop, Color_t tColorBottom, bool bHorizontal);
	void FillRectOutline(int x, int y, int w, int h, Color_t tColor, Color_t tColorOut = { 0, 0, 0, 255 });
	void LineRectOutline(int x, int y, int w, int h, Color_t tColor, Color_t tColorOut = { 0, 0, 0, 255 }, bool bInside = true);
	// Draws only the corners of a rect. flLengthFrac is the fraction (0-0.5) of each side used per corner.
	void LineCornerRect(int x, int y, int w, int h, Color_t tColor, float flLengthFrac = 0.25f);
	void LineCornerRectOutline(int x, int y, int w, int h, Color_t tColor, Color_t tColorOut = { 0, 0, 0, 255 }, float flLengthFrac = 0.25f, bool bInside = true);
	void FillRectPercent(int x, int y, int w, int h, float t, Color_t tColor, Color_t tColorOut = { 0, 0, 0, 255 }, EAlign eAlign = ALIGN_LEFT, bool bAdjust = false);
	void FillRoundRect(int x, int y, int w, int h, int iRadius, Color_t tColor, int iCount = 64);
	void FillRoundRectSolid(int x, int y, int w, int h, int iRadius, Color_t tColor); // body via DrawFilledRect (opaque, menu-matching) + poly corners
	void LineRoundRect(int x, int y, int w, int h, int iRadius, Color_t tColor, int iCount = 64);
	// Shared styled-panel outline (Vars::Menu::Style Outline/Color/Thickness/Blur) for surface-drawn
	// panels (Minimal spectator list, Checkpoints). Drawn just outside the panel; thickness is built
	// from concentric rings and blur from faded expanded passes. Mirrors CMenu::DrawStyledBackground.
	void PanelOutline(int x, int y, int w, int h, int iRadius, float flAlpha = 1.f);
	// THE single surface-panel background: a rounded fill in Style::Color at Style::Rounding, then the
	// shared PanelOutline stack. This is the exact surface-side twin of CMenu::DrawStyledBackground
	// (the ImGui watermark/menu/media path), so EVERY overlay panel - ImGui or surface - reads with the
	// same rounding, colour, alpha and outline. All surface panels (spectator list, Checkpoints, model
	// preview, recorder) must route their background through here so none can drift. flAlpha fades both
	// the fill and the outline together for pop-in/out panels.
	void StyledPanel(int x, int y, int w, int h, float flAlpha = 1.f);

	void FillCircle(int x, int y, float iRadius, int iSegments, Color_t tColor);
	void LineCircle(int x, int y, float iRadius, int iSegments, Color_t tColor);

	void Texture(const char* sTexture, int x, int y, int w, int h, EAlign eAlign = ALIGN_CENTER);
	// Draw an already-created surface texture id, tinted by tColor.
	void TextureId(int nTexture, int x, int y, int w, int h, Color_t tColor, EAlign eAlign = ALIGN_CENTER);
	CHudTexture* GetIcon(const char* szIcon, int eIconFormat = 0);
	int CreateTextureFromArray(const unsigned char* rgba, int w, int h);
	// Procedural surface texture reused across frames (Config > STYLE effect mirror for the spectator
	// list). NewProceduralTexId() makes one id; DrawStyleTexture() re-uploads the RGBA8888 buffer into
	// it and draws it - whole texture (bFullUV, Balatro) or by screen-rect sub-UV (Blur).
	int NewProceduralTexId();
	void DrawStyleTexture(int iId, const unsigned char* rgba, int texW, int texH, int x, int y, int w, int h, bool bFullUV);
	void DrawHudTexture(float x, float y, float s, const CHudTexture* pTexture, Color_t tColor = { 255, 255, 255, 255 });
	void DrawHudTextureByName(float x, float y, float s, const char* sTexture, Color_t tColor = { 255, 255, 255, 255 });
	void Avatar(int x, int y, int w, int h, const uint32 nFriendID, EAlign eAlign = ALIGN_CENTER);
	// Avatar drawn as a textured circle polygon (x/y = center).
	void AvatarCircle(int x, int y, float flRadius, const uint32 nFriendID, int iSegments = 32, byte cAlpha = 255);
	void ClearAvatarCache();

	void RenderLine(const Vec3& vStart, const Vec3& vEnd, Color_t tColor, bool bZBuffer = false);
	void RenderPath(const std::vector<Vec3>& vPath, Color_t tColor, bool bZBuffer = false,
		int iStyle = Vars::Visuals::Simulation::StyleEnum::Line, float flTime = 0.f,
		int iSeparatorSpacing = Vars::Visuals::Simulation::SeparatorSpacing.Value,
		float flSeparatorLength = Vars::Visuals::Simulation::SeparatorLength.Value);
	void RenderBox(const Vec3& vOrigin, const Vec3& vMins, const Vec3& vMaxs, const Vec3& vAngles, Color_t tColor, bool bZBuffer = false, bool bInsideOut = false);
	void RenderWireframeBox(const Vec3& vOrigin, const Vec3& vMins, const Vec3& vMaxs, const Vec3& vAngles, Color_t tColor, bool bZBuffer = false);
	void RenderWireframeSweptBox(const Vector& vStart, const Vector& vEnd, const Vec3& vMins, const Vec3& vMaxs, const Vec3& vAngles, Color_t tColor, bool bZBuffer = false);
	void RenderTriangle(const Vector& vPoint1, const Vector& vPoint2, const Vector& vPoint3, Color_t tColor, bool bZBuffer = false);
	void RenderSphere(const Vector& vCenter, float flRadius, int nTheta, int nPhi, Color_t tColor, IMaterial* pMaterial);
	void RenderSphere(const Vector& vCenter, float flRadius, int nTheta, int nPhi, Color_t tColor, bool bZBuffer = false);
	void RenderWireframeSphere(const Vector& vCenter, float flRadius, int nTheta, int nPhi, Color_t tColor, bool bZBuffer = false);

	int m_nScreenW = 0, m_nScreenH = 0;
	VMatrix m_mWorldToProjection = {};
};

ADD_FEATURE_CUSTOM(CDraw, Draw, H);