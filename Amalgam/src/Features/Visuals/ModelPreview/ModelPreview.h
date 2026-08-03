#pragma once
#include "../../../SDK/SDK.h"

// ESP "Model Preview" window. Renders models/player/heavy.mdl (stand_MELEE) inside the menu so
// you can see how your ESP/chams/glow look on a real player model, with drag-to-rotate.

struct model_t;
struct Group_t;
class CBaseEntity;
class IMaterial;
class ITexture;
class IMatRenderContext;

// Diagnostics surfaced in the menu so build-specific failures stay visible without a debugger.
// Remove once the preview is confirmed.
struct ModelPreviewDbg_t
{
	bool bEntitySigs = false;  // CBaseFlex ctor + SetModelByIndex resolved
	bool bModel      = false;  // heavy model loaded
	bool bEntity     = false;  // fake entity built
	bool bRT         = false;  // render-target texture + material created
	int  iRenderDrew = 0;      // times the model was rendered to the texture
	int  iBlits      = 0;      // times the texture was blitted to screen
	int  iRectW = 0, iRectH = 0;
	int  iTexW = 0, iTexH = 0;
};
extern ModelPreviewDbg_t g_MPDbg;

// Skeleton hitboxes stashed from the (known-good) SetupBones in the VGui pass so the ImGui overlay can
// project them without re-running SetupBones in a different render context. Order is fixed; the menu
namespace MPBone
{
	enum EBone { Head, Spine2, Pelvis, LUpperarm, LForearm, LHand, RUpperarm, RForearm, RHand,
		LThigh, LCalf, LFoot, RThigh, RCalf, RFoot, Count };
}

class CModelPreview
{
private:
	// Manually-allocated fake entity (never registered in the client entity list).
	CBaseEntity*   m_pEnt          = nullptr;
	bool           m_bEntInit      = false;

	const model_t* m_pModel        = nullptr;   // returned by the GetModel hook while in the menu
	bool           m_bModelLoaded  = false;
	int            m_iSequence     = 0;         // resolved "stand_MELEE" sequence
	int            m_iBuiltConnected = -1;      // connection state the entity's model was set for
	int            m_iLiveIndex      = -2;      // engine precache index the entity was last set to (re-resolve when it changes - new map / 2nd game rebuilds the string table, so the old index goes stale and the model vanishes)

	// Our render target + the material that samples it for the screen blit.
	ITexture*      m_pRTTexture    = nullptr;
	IMaterial*     m_pRTMaterial   = nullptr;

	// Screen-space rect the menu wants us to draw into (set every frame from the ESP tab). m_iPanelTop is
	// the top of the WHOLE panel (above the header); the body rect (m_iX/Y/W/H) is below the header.
	int  m_iX = 0, m_iY = 0, m_iW = 0, m_iH = 0;
	int  m_iPanelTop = 0;
	int  m_iPanelBottom = 0; // bottom of the WHOLE panel (below the footer buttons) for the bg
	bool m_bWantRender = false;

	// Supersample resolution actually rendered into the RT (panel size * kSupersample, clamped to the
	// RT). The blit downsamples this to the panel rect, smoothing the otherwise-aliased model edges.
	int  m_iSSW = 0, m_iSSH = 0;
	static constexpr int kSupersample = 2;

	// Preview camera projector + the exact panel rect the model was blitted into (both captured in the
	// VGui pass), so the 2D ESP overlay (drawn later, in the ImGui pass) can world->panel project in
	VMatrix m_mPreviewWTP        = {};
	int     m_iVX = 0, m_iVY = 0, m_iVW = 0, m_iVH = 0;
	bool    m_bPreviewViewValid  = false;

	Vec3 m_aPreviewBones[MPBone::Count] = {};
	bool m_bPreviewBonesValid = false;

	// Camera / rotation.
	float m_flYaw      = 180.f;
	float m_flAnimRate = 0.5f;
	float m_flZoom     = 1.f / 1.1f; // scroll-wheel zoom: >1 pulls the camera in, <1 pushes it out (1 = fit); default = one scroll step smaller

	// Which ESP section's styling the preview represents (0 = enemy, 1 = teammate). Selected by the
	// enemy/teammate tabs in the preview window; consumed by Stage 2 (chams/glow/skeleton on the ent).
	int  m_iSection = 0;

	const char* m_szModel    = "models/player/heavy.mdl";
	const char* m_szSequence = "stand_MELEE";

	// Extra distance factor when fitting the model into the view (so he isn't edge-to-edge). Shared by
	// AdjustCamera() and GetModelDrawWidth() so the menu can size the panel to the rendered model width.
	static constexpr float kFitPadding = 1.2f;

	bool EnsureModel();
	bool EnsureEntity();
	bool EnsureRenderTarget();
	int  ResolveSequence(const char* szName);
	void AdjustCamera(CViewSetup& view);
	void RenderToTexture();

	// The actual per-entity engine draws (chams/plain model, glow, bone capture) on our synthetic entity.
	bool RenderModelGuarded(IMatRenderContext* pRenderContext, Group_t* pGroup, bool bChamsInvisible, float flSS);
	bool m_bDrawFaulted = false; // set after a guarded-draw fault; reset on (dis)connect transition

public:
	// True only while the preview's fake entity is being drawn into our RT. The DrawModelExecute hook
	// reads it to skip its in-game gating (chams m_mEntities early-return, cosmetic removal, viewmodel
	bool m_bDrawing = false;

	void Run();            // per-frame (VGui paint pass): init + render the model into the RT (no blit)
	void BlitToScreen();   // matsystem blit of the RT to the panel rect - called from an ImGui draw callback (Present pass) so it lands on the ImGui bg, over the HUD
	void DrawBackground(); // DEAD: bg is now an ImGui rect in RenderModelPreviewWindow (exact menu match, no surface sRGB)
	void DrawOverlay();    // surface-draw the 2D ESP (box/bones/health/uber/name/class+weapon icon+text) over the blit - called from an ImGui draw callback
	void Free();

	// Driven by the menu each frame.
	void SetRect(int x, int y, int w, int h) { m_iX = x; m_iY = y; m_iW = w; m_iH = h; }
	void SetPanelTop(int y)                  { m_iPanelTop = y; } // full-panel top (above the header) for the bg
	void SetPanelBottom(int y)               { m_iPanelBottom = y; } // full-panel bottom (below the footer buttons) for the bg
	void SetWantRender(bool b)               { m_bWantRender = b; }
	bool WantRender() const                  { return m_bWantRender; }
	void AddYaw(float flDelta)               { m_flYaw += flDelta; }
	void AddZoom(float flDelta)              { m_flZoom = std::clamp(m_flZoom * (1.f + flDelta * 0.1f), 0.4f, 6.f); }
	void SetSection(int i)                   { m_iSection = i; }
	int  GetSection() const                  { return m_iSection; }

	CBaseEntity*   GetEntity() const { return m_pEnt; }
	const model_t* GetModelPtr() const { return m_pModel; }

	// World -> panel-screen (px) using the stashed preview camera. Returns false if behind the camera or
	// the view isn't ready. Used by the menu's 2D ESP overlay (do NOT use SDK::W2S - wrong camera).
	bool ProjectToPanel(const Vec3& vWorld, float& flScreenX, float& flScreenY) const;
	bool PreviewViewValid() const { return m_bPreviewViewValid; }

	// Stashed skeleton (world space), or nullptr if not available this frame. Indexed by MPBone::EBone.
	const Vec3* GetPreviewBones() const { return m_bPreviewBonesValid ? m_aPreviewBones : nullptr; }
	void GetPanelRect(int& x, int& y, int& w, int& h) const { x = m_iVX; y = m_iVY; w = m_iVW; h = m_iVH; }

	// Screen-space AABB of the model's render bounds (rotated by the current yaw) in panel px - the ESP
	// box, rotation-accurate. False if the view/model isn't ready or it projects fully behind the camera.
	bool GetPreviewBox(float& flX0, float& flY0, float& flX1, float& flY1) const;

	// On-screen width (px) the model occupies when fitted into a body of the given height. Lets the menu
	// size the panel to "model width + padding". Returns a sane fallback before the model has loaded.
	float GetModelDrawWidth(float flBodyHeightPx);
};

ADD_FEATURE(CModelPreview, ModelPreview);
