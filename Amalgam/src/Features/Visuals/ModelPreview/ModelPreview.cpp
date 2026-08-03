#include "ModelPreview.h"

#include "../../../SDK/Definitions/Misc/Studio.h"
#include "../../../SDK/Definitions/Misc/LightDesc.h"
#include "../Materials/Materials.h"
#include "../Groups/Groups.h"
#include "../Chams/Chams.h"
#include "../Glow/Glow.h"

#include <algorithm>
#include <cmath>
#include <cfloat>
#include <string>
#include <format>

// Build-specific entity signatures. Resolved lazily (not via
// MAKE_SIGNATURE) so a stale pattern just disables the preview instead of bricking the inject.
using CBaseFlex_Constructor_t        = void(__fastcall*)(void*);
using CBaseEntity_SetModelByIndex_t  = void(__fastcall*)(void*, unsigned short);

static CBaseFlex_Constructor_t       fnCBaseFlex_Constructor       = nullptr;
static CBaseEntity_SetModelByIndex_t fnCBaseEntity_SetModelByIndex = nullptr;
static bool                          s_bSigsResolved               = false;
static bool                          s_bSigsValid                  = false;

// NOTE (flicker/disappear refs - see memory insane-reference-dir): RT uses MATERIAL_RT_DEPTH_SEPARATE so the
// in-game scene depth can't occlude the isolated preview model; Run() re-resolves the precache index every
ModelPreviewDbg_t g_MPDbg;

static void ResolveSigs()
{
	if (s_bSigsResolved)
		return;

	s_bSigsResolved = true;

	const uintptr_t dwCtor     = U::Memory.FindSignature("client.dll", "48 89 5C 24 ? 48 89 6C 24 ? 48 89 74 24 ? 57 41 56 41 57 48 83 EC ? 48 8B D9 E8 ? ? ? ? 33 ED");
	const uintptr_t dwSetModel = U::Memory.FindSignature("client.dll", "48 89 5C 24 ? 57 48 83 EC ? 66 89 91");

	fnCBaseFlex_Constructor       = reinterpret_cast<CBaseFlex_Constructor_t>(dwCtor);
	fnCBaseEntity_SetModelByIndex = reinterpret_cast<CBaseEntity_SetModelByIndex_t>(dwSetModel);

	s_bSigsValid = dwCtor && dwSetModel;
	g_MPDbg.bEntitySigs = s_bSigsValid;
}

// ---------------------------------------------------------------------------
int CModelPreview::ResolveSequence(const char* szName)
{
	if (!m_pModel)
		return 0;

	auto pHDR = I::ModelInfoClient->GetStudiomodel(m_pModel);
	if (!pHDR)
		return 0;

	for (int i = 0; i < pHDR->numlocalseq; i++)
	{
		auto pSeq = pHDR->pLocalSeqdesc(i);
		if (pSeq && pSeq->pszLabel() && !_stricmp(pSeq->pszLabel(), szName))
			return i;
	}
	return 0;
}

// ---------------------------------------------------------------------------
bool CModelPreview::EnsureModel()
{
	if (m_bModelLoaded && m_pModel)
		return true;

	m_pModel = I::ModelInfoClient->FindOrLoadModel(m_szModel);
	if (!m_pModel)
		return false;

	m_iSequence    = ResolveSequence(m_szSequence);
	m_bModelLoaded = true;
	return true;
}

// ---------------------------------------------------------------------------
bool CModelPreview::EnsureEntity()
{
	if (m_bEntInit && m_pEnt)
		return true;

	ResolveSigs();
	if (!s_bSigsValid)
		return false;

	if (!m_pEnt)
	{
		// CBaseFlex is a big object; over-allocate and zero so the ctor has room.
		constexpr size_t iEntSize = 0x400ull * 10ull;
		m_pEnt = reinterpret_cast<CBaseEntity*>(malloc(iEntSize));
		if (!m_pEnt)
			return false;
		memset(m_pEnt, 0, iEntSize);
	}

	// Run the real CBaseFlex constructor so the vtable + members are valid.
	fnCBaseFlex_Constructor(m_pEnt);

	// In the menu GetModelIndex returns -1 (nothing precached); the GetModel hook fills in our
	// model pointer during this call. In a match we use the heavy model's real index.
	int iIndex = I::ModelInfoClient->GetModelIndex(m_szModel);
	if (iIndex < 0)
		iIndex = 1;
	fnCBaseEntity_SetModelByIndex(m_pEnt, static_cast<unsigned short>(iIndex));
	m_iBuiltConnected = I::EngineClient->IsConnected() ? 1 : 0;

	m_pEnt->SetAbsOrigin(Vec3(0.f, 0.f, 0.f));
	m_pEnt->SetAbsAngles(Vec3(0.f, m_flYaw, 0.f));

	m_bEntInit = true;
	return true;
}

// ---------------------------------------------------------------------------
// Render target + material that samples it - mirrors CCameraWindow::Initialize.
bool CModelPreview::EnsureRenderTarget()
{
	if (m_pRTTexture && m_pRTMaterial)
		return true;

	if (!m_pRTTexture)
	{
		// RGBA16161616F (true HDR float target WITH a float alpha channel). We need alpha so the bg can be
		// cleared to alpha 0 and only the model's pixels stay opaque - the blit then composites over whatever
		m_pRTTexture = I::MaterialSystem->CreateNamedRenderTargetTextureEx(
			"AletheriumModelPreviewRT",
			1, 1, RT_SIZE_FULL_FRAME_BUFFER,
			IMAGE_FORMAT_RGBA16161616F, MATERIAL_RT_DEPTH_SEPARATE,
			TEXTUREFLAGS_CLAMPS | TEXTUREFLAGS_CLAMPT,
			CREATERENDERTARGETFLAGS_HDR);
		if (!m_pRTTexture)
			return false;
		m_pRTTexture->IncrementReferenceCount();
	}

	if (!m_pRTMaterial)
	{
		// $translucent so the blit blends using the texture's alpha (transparent background).
		KeyValues* kv = new KeyValues("UnlitGeneric");
		kv->SetString("$basetexture", "AletheriumModelPreviewRT");
		kv->SetInt("$translucent", 1);
		kv->SetInt("$ignorez", 1);
		kv->SetInt("$nocull", 1);
		m_pRTMaterial = F::Materials.Create("AletheriumModelPreviewMat", kv);
		if (!m_pRTMaterial)
			return false;
	}

	g_MPDbg.bRT   = true;
	g_MPDbg.iTexW = m_pRTTexture->GetActualWidth();
	g_MPDbg.iTexH = m_pRTTexture->GetActualHeight();
	return true;
}

// ---------------------------------------------------------------------------
// Width (px) the fitted model occupies in a body of height flBodyHeightPx. With the panel sized wider
float CModelPreview::GetModelDrawWidth(float flBodyHeightPx)
{
	float flAspect = 0.42f; // fallback (~heavy) until the model loads
	if (m_pModel)
	{
		Vec3 vMins, vMaxs;
		I::ModelInfoClient->GetModelRenderBounds(m_pModel, vMins, vMaxs);
		const float flH = fabsf(vMaxs.z - vMins.z);
		const float flW = std::max(fabsf(vMaxs.x - vMins.x), fabsf(vMaxs.y - vMins.y));
		if (flH > 0.01f)
			flAspect = flW / flH;
	}
	return (flBodyHeightPx / kFitPadding) * flAspect;
}

// ---------------------------------------------------------------------------
void CModelPreview::AdjustCamera(CViewSetup& view)
{
	if (!m_pModel)
		return;

	Vec3 vMins, vMaxs;
	I::ModelInfoClient->GetModelRenderBounds(m_pModel, vMins, vMaxs);

	const float flModelHeight = fabsf(vMaxs.z - vMins.z);
	const float flModelWidth  = std::max(fabsf(vMaxs.x - vMins.x), fabsf(vMaxs.y - vMins.y));

	// Fit the model into BOTH the view height and width so a thin panel just pulls the camera back
	// (showing the whole model smaller) instead of clipping his sides. fov is the horizontal FOV;
	const float flAspect   = view.height > 0 ? float(view.width) / float(view.height) : 1.f;
	const float flHalfHFov = DEG2RAD(view.fov) * 0.5f;
	const float flHalfVFov = atanf(tanf(flHalfHFov) / std::max(0.01f, flAspect));

	const float flDistW = (flModelWidth  * 0.5f) / std::max(0.01f, tanf(flHalfHFov));
	const float flDistH = (flModelHeight * 0.5f) / std::max(0.01f, tanf(flHalfVFov));

	float flDist = std::max(flDistW, flDistH) * kFitPadding; // padding so he isn't edge-to-edge
	flDist /= std::max(0.01f, m_flZoom);                     // scroll-wheel zoom (>1 pulls the camera in)
	flDist = std::clamp(flDist, 30.f, 1000.f);

	const float flCenterZ = (vMins.z + vMaxs.z) * 0.5f;

	view.origin = Vec3(-flDist, 0.f, flCenterZ);
	view.angles = Vec3(0.f, 0.f, 0.f); // look down +X toward the model
}

// ---------------------------------------------------------------------------
// The risky part of the preview: drawing our synthetic (not-in-entity-list) entity through the engine's
bool CModelPreview::RenderModelGuarded(IMatRenderContext* pRenderContext, Group_t* pGroup, bool bChamsInvisible, float flSS)
{
	__try
	{
		I::ModelRender->SuppressEngineLighting(true);
		// m_bDrawing tells the DrawModelExecute hook this is our preview ent: skip the in-game gating that
		// keys off entity_index (chams m_mEntities early-return etc.) so the model can't flicker in a match.
		m_bDrawing = true;
		// None-chams = invisible (draw nothing, glow below is the only render - exactly like a real player).
		// Chams set = draw chams. No chams (or disabled section) = the plain default heavy.
		if (!bChamsInvisible)
		{
			if (pGroup && pGroup->m_tChams())
				F::Chams.DrawPreview(m_pEnt, pGroup->m_tChams, pRenderContext);
			else
				m_pEnt->DrawModel(STUDIO_RENDER);
		}
		m_bDrawing = false;
		I::ModelRender->SuppressEngineLighting(false);

		// Glow outline (opt-in per section). DrawPreview owns its own alpha-write state: it renders the halo
		// into a dedicated RGBA preview buffer (black bg, dimmed glow colour) and composites it onto this
		if (pGroup && pGroup->m_tGlow())
			F::Glow.DrawPreview(m_pEnt, pGroup->m_tGlow, pGroup->m_tColor, pRenderContext, m_iSSW, m_iSSH, flSS);

		// Stash the skeleton (in this known-good bone-setup context) for the 2D overlay's bone lines.
		m_bPreviewBonesValid = false;
		if (pGroup && (pGroup->m_iESP & ESPEnum::Bones))
		{
			auto pPlayer = m_pEnt->As<CTFPlayer>();
			matrix3x4 aBones[MAXSTUDIOBONES];
			if (pPlayer->SetupBones(aBones, MAXSTUDIOBONES, BONE_USED_BY_ANYTHING, I::GlobalVars->curtime))
			{
				static const int kHitboxes[MPBone::Count] = {
					HITBOX_HEAD, HITBOX_SPINE2, HITBOX_PELVIS,
					HITBOX_LEFT_UPPERARM, HITBOX_LEFT_FOREARM, HITBOX_LEFT_HAND,
					HITBOX_RIGHT_UPPERARM, HITBOX_RIGHT_FOREARM, HITBOX_RIGHT_HAND,
					HITBOX_LEFT_THIGH, HITBOX_LEFT_CALF, HITBOX_LEFT_FOOT,
					HITBOX_RIGHT_THIGH, HITBOX_RIGHT_CALF, HITBOX_RIGHT_FOOT };
				for (int i = 0; i < MPBone::Count; i++)
					m_aPreviewBones[i] = pPlayer->GetHitboxCenter(aBones, pPlayer->GetBaseToHitbox(kHitboxes[i]));
				m_bPreviewBonesValid = true;
			}
		}
		return true;
	}
	__except (EXCEPTION_EXECUTE_HANDLER)
	{
		// Leave shared render state sane for the rest of the frame's draws.
		m_bDrawing = false;
		m_bPreviewBonesValid = false;
		I::ModelRender->SuppressEngineLighting(false);
		return false;
	}
}

// ---------------------------------------------------------------------------
// Render the model into the top-left m_iW x m_iH region of our own render target. Because it's
void CModelPreview::RenderToTexture()
{
	auto pRenderContext = I::MaterialSystem->GetRenderContext();
	if (!pRenderContext)
		return;

	static ITexture* pCubeMap = nullptr;
	if (!pCubeMap)
		pCubeMap = I::MaterialSystem->FindTexture("editor/cubemap.hdr", "CubeMapTexture", true);

	// Keep the model facing the menu's rotation and advance the looping animation.
	m_pEnt->SetAbsOrigin(Vec3(0.f, 0.f, 0.f));
	m_pEnt->SetAbsAngles(Vec3(0.f, m_flYaw, 0.f));
	m_pEnt->As<CBaseAnimating>()->m_nSequence() = m_iSequence;
	// Teammate tab = BLU team skin (1); enemy = RED (0). TF2 player models: 0 red, 1 blu, 2 red-invuln,
	// 3 blu-invuln (skin 2 showed RED, so blu is 1).
	m_pEnt->As<CBaseAnimating>()->m_nSkin() = GetSection() == 1 ? 1 : 0;
	m_pEnt->As<CBaseAnimating>()->m_flCycle()   = fmodf(m_pEnt->As<CBaseAnimating>()->m_flCycle() + I::GlobalVars->frametime * m_flAnimRate, 1.f);
	m_pEnt->m_flAnimTime() = I::GlobalVars->curtime;

	// Supersample: render the model at kSupersample x the panel size into the (full-frame) RT, then let
	// the blit downscale it. This is cheap SSAA - it removes the jaggies that made the model look lower
	const int iRTW = m_pRTTexture->GetActualWidth();
	const int iRTH = m_pRTTexture->GetActualHeight();
	float flSS = float(kSupersample);
	if (m_iW > 0) flSS = std::min(flSS, float(iRTW) / float(m_iW));
	if (m_iH > 0) flSS = std::min(flSS, float(iRTH) / float(m_iH));
	flSS = std::max(flSS, 1.f);
	m_iSSW = int(m_iW * flSS);
	m_iSSH = int(m_iH * flSS);

	CViewSetup view;
	memset(&view, 0, sizeof(CViewSetup));
	view.x       = 0;        view.y      = 0;
	view.width   = m_iSSW;   view.height = m_iSSH;
	view.fov     = 35.f;
	view.zNear   = 4.f;      view.zFar   = 4096.f;
	view.m_bOrtho = false;
	AdjustCamera(view);

	// Lower flat ambient than before so the directional light actually shapes the model (it used to be
	// near-fullbright). The single local light below gives him front-up-left shading.
	static Vec3 aAmbient[6] =
	{
		Vec3(0.32f, 0.32f, 0.34f), Vec3(0.32f, 0.32f, 0.34f),
		Vec3(0.32f, 0.32f, 0.34f), Vec3(0.32f, 0.32f, 0.34f),
		Vec3(0.32f, 0.32f, 0.34f), Vec3(0.32f, 0.32f, 0.34f),
	};

	// A bright local light placed in front of / above / to the model's left (the camera looks down +X
	// from -X, so the model's front faces -X toward us). Constant attenuation (no falloff) keeps it even.
	LightDesc_t light;
	memset(&light, 0, sizeof(light));
	light.m_Type         = MATERIAL_LIGHT_POINT;
	light.m_Color        = Vec3(1.8f, 1.75f, 1.65f); // tweak up for punchier, down if highlights blow out
	light.m_Position     = Vec3(view.origin.x * 0.6f, fabsf(view.origin.x) * 0.5f, view.origin.z + 36.f);
	light.m_Range        = 0.f;
	light.m_Attenuation0 = 1.f;
	light.m_Attenuation1 = 0.f;
	light.m_Attenuation2 = 0.f;
	light.m_Flags        = LIGHTTYPE_OPTIMIZATIONFLAGS_HAS_ATTENUATION0;

	pRenderContext->PushRenderTargetAndViewport();
	pRenderContext->SetRenderTarget(m_pRTTexture);
	pRenderContext->Viewport(0, 0, m_iSSW, m_iSSH);

	if (pCubeMap)
		pRenderContext->BindLocalCubemap(pCubeMap);
	pRenderContext->SetLightingOrigin(Vec3(0.f, 0.f, 0.f));
	pRenderContext->SetAmbientLight(0.32f, 0.32f, 0.34f);
	I::StudioRender->SetAmbientLightColors(aAmbient);
	I::StudioRender->SetLocalLights(1, &light);

	Frustum dummyFrustum;
	I::RenderView->Push3DView(view, VIEW_CLEAR_COLOR | VIEW_CLEAR_DEPTH | VIEW_CLEAR_STENCIL, m_pRTTexture, dummyFrustum);

	// Capture the engine's view/projection while the preview view is active, so the menu's 2D ESP overlay
	// can world->panel project exactly where the model lands. worldToProjection = proj * view.
	{
		VMatrix mView, mProj;
		pRenderContext->GetMatrix(MATERIAL_VIEW, &mView);
		pRenderContext->GetMatrix(MATERIAL_PROJECTION, &mProj);
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
			{
				float flSum = 0.f;
				for (int k = 0; k < 4; k++)
					flSum += mProj.m[i][k] * mView.m[k][j];
				m_mPreviewWTP.m[i][j] = flSum;
			}
		m_iVX = m_iX; m_iVY = m_iY; m_iVW = m_iW; m_iVH = m_iH;
		m_bPreviewViewValid = true;
	}
	I::RenderView->SetColorModulation(1.f, 1.f, 1.f);
	I::RenderView->SetBlend(1.f);

	// Pick the ESP section the enemy/teammate tabs selected (0 = enemy, 1 = teammate). Everything the
	// preview shows (chams now; glow + 2D ESP later) is read from this section's Group_t so the panel
	const int iSec = GetSection() == 1 ? SectionEnum::Teammates : SectionEnum::Enemies;
	Group_t* pGroup = iSec < int(F::Groups.m_vGroups.size()) ? &F::Groups.m_vGroups[iSec] : nullptr;
	// Mirror in-game (Groups::Store skips !m_bEnabled sections): a disabled enemy/teammate section
	// changes NOTHING about the heavy - no chams, no glow, no ESP, just the plain default model.
	if (pGroup && !pGroup->m_bEnabled)
		pGroup = nullptr;

	// In-game a "None" (additive-black) chams material renders the player INVISIBLE: the engine's real
	bool bChamsInvisible = false;
	if (pGroup && pGroup->m_tChams())
	{
		bChamsInvisible = true;
		for (const auto& [sName, tColor] : pGroup->m_tChams.Visible)
			if (FNV1A::Hash32(sName.c_str()) != FNV1A::Hash32Const("None"))
			{
				bChamsInvisible = false;
				break;
			}
	}

	// Transparent TINT cham = a visible cham whose colour alpha < 255 (additive/translucent overlay like
	// Tint/Shine/Fresnel, or any low-alpha tint). Clearing the RT to the dark bg COLOUR would let that gray
	bool bChamsTransparent = false;
	if (pGroup && pGroup->m_tChams() && !bChamsInvisible)
		for (const auto& [sName, tColor] : pGroup->m_tChams.Visible)
			if (tColor.a < 255)
			{
				bChamsTransparent = true;
				break;
			}

	// Clear to the bg COLOUR but alpha 0 so the model's anti-aliased edges blend toward the bg (no dark
	const Color_t cClear = Vars::Menu::Style::Color.Value;
	if (bChamsTransparent)
		pRenderContext->ClearColor4ub(0, 0, 0, 0);
	else
		pRenderContext->ClearColor4ub(cClear.r, cClear.g, cClear.b, 0);
	pRenderContext->ClearBuffers(true, true);

	// Force alpha writes ON for the model draw so the (opaque) model stamps alpha=1 into the transparent
	// RT - this is the dest-alpha the $translucent blit needs to show the model solid over the rounded bg.
	pRenderContext->OverrideAlphaWriteEnable(true, true);

	// All the per-entity engine draws on our synthetic ent (base model/chams, glow, bone capture) run inside
	if (!m_bDrawFaulted)
	{
		if (!RenderModelGuarded(pRenderContext, pGroup, bChamsInvisible, flSS))
		{
			m_bDrawFaulted = true;
			I::ModelRender->ForcedMaterialOverride(nullptr); // clear the override a mid-draw fault may have left set
		}
	}

	pRenderContext->OverrideAlphaWriteEnable(false, false);

	I::RenderView->PopView(dummyFrustum);

	if (pCubeMap)
		pRenderContext->BindLocalCubemap(nullptr);

	pRenderContext->PopRenderTargetAndViewport();
	pRenderContext->Release();

	g_MPDbg.iRenderDrew++;
}

// ---------------------------------------------------------------------------
// Blit the rendered texture onto the screen at the menu's rect (CCameraWindow::Draw pattern).
void CModelPreview::BlitToScreen()
{
	// Called from an ImGui draw callback (Present). On the FIRST frame the menu opens, SetWantRender(true)
	if (!m_bWantRender || !m_pRTMaterial || !m_pRTTexture)
		return;
	if (m_iW <= 0 || m_iH <= 0 || m_iSSW <= 0 || m_iSSH <= 0)
		return;

	auto pRenderContext = I::MaterialSystem->GetRenderContext();
	if (!pRenderContext)
		return;

	// Keep the panel fully solid (bg cleared opaque, model opaque) - no overall fade.
	m_pRTMaterial->AlphaModulate(1.f);

	// Source = the supersampled region we rendered (0,0.. m_iSSW,m_iSSH); dest = the panel rect. The
	// bilinear downscale is the actual anti-aliasing.
	pRenderContext->DrawScreenSpaceRectangle(
		m_pRTMaterial,
		m_iX, m_iY, m_iW, m_iH,
		0.f, 0.f, static_cast<float>(m_iSSW), static_cast<float>(m_iSSH),
		m_pRTTexture->GetActualWidth(), m_pRTTexture->GetActualHeight(),
		nullptr, 1, 1);

	pRenderContext->Release();

	g_MPDbg.iBlits++;
}

// ---------------------------------------------------------------------------
bool CModelPreview::ProjectToPanel(const Vec3& vWorld, float& flScreenX, float& flScreenY) const
{
	if (!m_bPreviewViewValid)
		return false;

	const auto& m = m_mPreviewWTP.m;
	const float clipX = m[0][0] * vWorld.x + m[0][1] * vWorld.y + m[0][2] * vWorld.z + m[0][3];
	const float clipY = m[1][0] * vWorld.x + m[1][1] * vWorld.y + m[1][2] * vWorld.z + m[1][3];
	const float clipW = m[3][0] * vWorld.x + m[3][1] * vWorld.y + m[3][2] * vWorld.z + m[3][3];
	if (clipW < 0.001f)
		return false; // behind the camera

	const float flNdcX = clipX / clipW;
	const float flNdcY = clipY / clipW;
	flScreenX = m_iVX + (flNdcX * 0.5f + 0.5f) * m_iVW;
	flScreenY = m_iVY + (0.5f - flNdcY * 0.5f) * m_iVH;
	return true;
}

// ---------------------------------------------------------------------------
bool CModelPreview::GetPreviewBox(float& flX0, float& flY0, float& flX1, float& flY1) const
{
	if (!m_bPreviewViewValid || !m_pModel)
		return false;

	Vec3 vMins, vMaxs;
	I::ModelInfoClient->GetModelRenderBounds(m_pModel, vMins, vMaxs);

	// The entity sits at the origin, rotated by m_flYaw about Z (matching RenderToTexture).
	const float flRad = DEG2RAD(m_flYaw);
	const float flCos = cosf(flRad), flSin = sinf(flRad);

	flX0 = flY0 = FLT_MAX;
	flX1 = flY1 = -FLT_MAX;
	bool bAny = false;
	for (int i = 0; i < 8; i++)
	{
		const Vec3 vLocal((i & 1) ? vMaxs.x : vMins.x, (i & 2) ? vMaxs.y : vMins.y, (i & 4) ? vMaxs.z : vMins.z);
		const Vec3 vWorld(vLocal.x * flCos - vLocal.y * flSin, vLocal.x * flSin + vLocal.y * flCos, vLocal.z);

		float sx, sy;
		if (!ProjectToPanel(vWorld, sx, sy))
			continue;
		flX0 = std::min(flX0, sx); flY0 = std::min(flY0, sy);
		flX1 = std::max(flX1, sx); flY1 = std::max(flY1, sy);
		bAny = true;
	}
	return bAny;
}

// ---------------------------------------------------------------------------
void CModelPreview::DrawBackground()
{
	if (!m_bWantRender || m_iW <= 0 || m_iH <= 0)
		return;

	// A prior surface draw can leave the global alpha multiplier < 1, which silently scaled our bg down
	// (it read more transparent than the style colour's alpha). Reset it so the bg draws at its true alpha.
	I::MatSystemSurface->DrawSetAlphaMultiplier(1.f);

	const int iTop = m_iPanelTop > 0 ? m_iPanelTop : m_iY; // fall back to body top if not set yet
	const int iBot = m_iPanelBottom > 0 ? m_iPanelBottom : (m_iY + m_iH); // cover footer buttons too
	const int iH = iBot - iTop;
	// Fill only (shared Style colour + rounding). The outline is drawn ImGui-side over the chrome in
	// RenderModelPreviewWindow (DrawStyledOutline) so it sits above the 3D model blit, not under it -
	Color_t cBg = Vars::Menu::Style::Color.Value;
	// The main menu draws its bg via ImGui with D3DRS_SRGBWRITEENABLE forced OFF (Render.cpp) - raw bytes into
	// the framebuffer. This panel MUST draw via surface (it sits UNDER the 3D model blit + the surface 2D-ESP
	auto SrgbEncode = [](uint8_t v) -> uint8_t
	{
		const float c = v / 255.f;
		const float e = c <= 0.0031308f ? 12.92f * c : 1.055f * std::pow(c, 1.f / 2.4f) - 0.055f;
		return (uint8_t)std::clamp(int(e * 255.f + 0.5f), 0, 255);
	};
	cBg.r = SrgbEncode(cBg.r);
	cBg.g = SrgbEncode(cBg.g);
	cBg.b = SrgbEncode(cBg.b);
	const int iRound = std::max(0, int(H::Draw.Scale(Vars::Menu::Style::Rounding.Value)));
	// Solid body (DrawFilledRect) so opacity matches the menu/watermark + StyledPanel exactly. Raw
	// FillRoundRect (textured-poly) read a touch lighter, so this panel looked more transparent.
	H::Draw.FillRoundRectSolid(m_iX, iTop, m_iW, iH, iRound, cBg);
}

// ---------------------------------------------------------------------------
// Surface-draw the 2D ESP onto the blitted model. Runs in the VGui paint pass inside an H::Draw.Start()
void CModelPreview::DrawOverlay()
{
	if (!m_bWantRender || !m_bPreviewViewValid)
		return;

	const int iSec = GetSection() == 1 ? SectionEnum::Teammates : SectionEnum::Enemies;
	if (iSec >= int(F::Groups.m_vGroups.size()))
		return;
	Group_t& tGroup = F::Groups.m_vGroups[iSec];
	if (!tGroup.m_bEnabled) // disabled section: nothing changed about the heavy, no 2D ESP either
		return;
	const int iESP = tGroup.m_iESP;

	float fbx0, fby0, fbx1, fby1;
	if (!GetPreviewBox(fbx0, fby0, fbx1, fby1))
		return;

	I::MatSystemSurface->DrawSetAlphaMultiplier(1.f); // don't inherit a scaled alpha from a prior draw
	H::Draw.StartClipping(m_iVX, m_iVY, m_iVW, m_iVH);

	const auto& fFont = H::Fonts.GetFont(FONT_ESP);
	const int nTall = fFont.m_nTall + H::Draw.Scale(2);

	const int x = int(fbx0), y = int(fby0);
	const int w = int(fbx1 - fbx0), h = int(fby1 - fby0);
	const int l = x - H::Draw.Scale(6), r = x + w + H::Draw.Scale(6), m = x + w / 2;
	const int t = y - H::Draw.Scale(5), b = y + h + H::Draw.Scale(5);
	int lOffset = 0, rOffset = 0, bOffset = 0, tOffset = 0;

	// Sample stats (heavy / fists preview).
	const float flHealth = 0.72f;
	const int   iHealth = 216;
	const float flUber = 0.5f;

	const int iTextStyle = Vars::ESP::TextStyle.Value == Vars::ESP::TextStyleEnum::TextShadow ? 0 : 1;
	const Color_t tBgRaw = Vars::Menu::Theme::Background.Value;
	const Color_t tOutline = Vars::ESP::TextStyle.Value == Vars::ESP::TextStyleEnum::TextNone ? Color_t{ 0, 0, 0, 0 } : tBgRaw;

	// --- Box ---
	if (iESP & ESPEnum::Box)
	{
		const Color_t tBox = tGroup.ElementColor(ESPEnum::Box);
		if (tGroup.m_bCornerBox)
		{
			if (tGroup.m_bBoxOutline) H::Draw.LineCornerRectOutline(x, y, w, h, tBox, { 0, 0, 0, 255 }, tGroup.m_flCornerLength);
			else                      H::Draw.LineCornerRect(x, y, w, h, tBox, tGroup.m_flCornerLength);
		}
		else
		{
			if (tGroup.m_bBoxOutline) H::Draw.LineRectOutline(x, y, w, h, tBox);
			else                      H::Draw.LineRect(x, y, w, h, tBox);
		}
	}

	// --- Bones ---
	if ((iESP & ESPEnum::Bones) && m_bPreviewBonesValid)
	{
		const Color_t tBones = tGroup.ElementColor(ESPEnum::Bones);
		const int iBoneThick = std::max(1, int(H::Draw.Scale(Vars::ESP::SkeletonThickness.Value, Scale_Round)));
		static const int kChains[5][4] = {
			{ MPBone::Head, MPBone::Spine2, MPBone::Pelvis, -1 },
			{ MPBone::Spine2, MPBone::LUpperarm, MPBone::LForearm, MPBone::LHand },
			{ MPBone::Spine2, MPBone::RUpperarm, MPBone::RForearm, MPBone::RHand },
			{ MPBone::Pelvis, MPBone::LThigh, MPBone::LCalf, MPBone::LFoot },
			{ MPBone::Pelvis, MPBone::RThigh, MPBone::RCalf, MPBone::RFoot },
		};
		for (auto& chain : kChains)
		{
			int px = 0, py = 0; bool bPrev = false;
			for (int j = 0; j < 4; j++)
			{
				if (chain[j] < 0) break;
				float sx, sy;
				if (!ProjectToPanel(m_aPreviewBones[chain[j]], sx, sy)) { bPrev = false; continue; }
				if (bPrev) H::Draw.LineThick(px, py, int(sx), int(sy), tBones, iBoneThick);
				px = int(sx); py = int(sy); bPrev = true;
			}
		}
	}

	// --- Bars (health left, uber bottom) ---
	const int iSpace = H::Draw.Scale(4);
	const int iThickness = H::Draw.Scale(2, Scale_Round);
	if (iESP & ESPEnum::HealthBar)
	{
		const int iW = H::Draw.Scale(tGroup.m_flHealthBarWidth, Scale_Round);
		const Color_t tOut = tGroup.m_bHealthBarOutline ? Color_t{ 0, 0, 0, 255 } : Color_t{ 0, 0, 0, 0 };
		const Color_t tHp = tGroup.ElementColor(ESPEnum::HealthBar);
		const bool bBg = tGroup.m_bHealthBarBackground;
		const Color_t tBg = tGroup.m_tHealthBarBackgroundColor;
		switch (tGroup.m_iHealthBarPosition)
		{
		case 1: { const int iBarX = x + w + iSpace + rOffset; if (bBg) H::Draw.FillRect(iBarX, y, iW, h, tBg); H::Draw.FillRectPercent(iBarX, y, iW, h, flHealth, tHp, tOut, ALIGN_BOTTOM, true); rOffset += iSpace + iW; break; }
		case 2: { const int iBarY = y - iSpace - iW - tOffset; if (bBg) H::Draw.FillRect(x, iBarY, w, iW, tBg); H::Draw.FillRectPercent(x, iBarY, w, iW, flHealth, tHp, tOut, ALIGN_LEFT, true); tOffset += iSpace + iW; break; }
		case 3: { const int iBarY = y + h + iSpace + bOffset; if (bBg) H::Draw.FillRect(x, iBarY, w, iW, tBg); H::Draw.FillRectPercent(x, iBarY, w, iW, flHealth, tHp, tOut, ALIGN_LEFT, true); bOffset += iSpace + iW; break; }
		default: { const int iBarX = x - iSpace - iW - lOffset; if (bBg) H::Draw.FillRect(iBarX, y, iW, h, tBg); H::Draw.FillRectPercent(iBarX, y, iW, h, flHealth, tHp, tOut, ALIGN_BOTTOM, true); lOffset += iSpace + iW; break; }
		}
	}
	if (iESP & ESPEnum::UberBar)
	{
		H::Draw.FillRectPercent(x, y + h + iSpace + bOffset, w, iThickness, flUber, tGroup.ElementColor(ESPEnum::UberBar), { 0, 0, 0, 255 }, ALIGN_LEFT, false);
		bOffset += iSpace + iThickness;
	}

	// --- Text (anchors mirror CESP::DrawPlayers) ---
	const int iNameMode = tGroup.m_iNamePosition == 1 ? ALIGN_BOTTOM : ALIGN_TOP;
	int iNameCX = m, iNameCY = t - fFont.m_nTall / 2; float flNameW = 0.f;
	if (iESP & ESPEnum::Name)
	{
		flNameW = H::Draw.GetTextSize("Player", fFont).x;
		iNameCX = m;
		if (iNameMode == ALIGN_BOTTOM)
		{
			iNameCY = b + bOffset + fFont.m_nTall / 2;
			H::Draw.StringOutlined(fFont, m, b + bOffset, tGroup.ElementColor(ESPEnum::Name), tOutline, ALIGN_TOP, "Player", true, iTextStyle);
			bOffset += nTall;
		}
		else
		{
			iNameCY = t - tOffset - fFont.m_nTall / 2;
			H::Draw.StringOutlined(fFont, m, t - tOffset, tGroup.ElementColor(ESPEnum::Name), tOutline, ALIGN_BOTTOM, "Player", true, iTextStyle);
			tOffset += nTall;
		}
	}

	// --- Avatar circle (your own avatar) left of the name, mirrors CESP::DrawPlayers ---
	if (iESP & ESPEnum::Avatar)
	{
		const uint32_t uLocalID = I::SteamUser ? I::SteamUser->GetSteamID().GetAccountID() : 0;
		if (uLocalID)
		{
			const float flRad = fFont.m_nTall / 3.f;
			const int iCx = iNameCX - int(flNameW / 2.f) - int(flRad) - H::Draw.Scale(6);
			const int iCy = iNameCY;
			H::Draw.AvatarCircle(iCx, iCy, flRad, uLocalID);
			if (tGroup.m_bAvatarOutline)
				H::Draw.LineCircle(iCx, iCy, flRad, 32, { 0, 0, 0, 255 });
		}
	}
	if (iESP & ESPEnum::ClassText)
	{
		H::Draw.StringOutlined(fFont, r, y - H::Draw.Scale(2) + rOffset, tGroup.ElementColor(ESPEnum::ClassText), tOutline, ALIGN_TOPLEFT, SDK::GetClassByIndex(TF_CLASS_HEAVY, false), true, iTextStyle);
		rOffset += nTall;
	}
	if (iESP & ESPEnum::WeaponText)
	{
		H::Draw.StringOutlined(fFont, m, b + bOffset, tGroup.ElementColor(ESPEnum::WeaponText), tOutline, ALIGN_TOP, "Fists", true, iTextStyle);
		bOffset += nTall;
	}
	if (iESP & ESPEnum::HealthText)
		H::Draw.StringOutlined(fFont, l - lOffset, y - H::Draw.Scale(2) + h - int(h * std::min(flHealth, 1.f)), tGroup.ElementColor(ESPEnum::HealthText), tOutline, ALIGN_TOPRIGHT, std::to_string(iHealth).c_str(), true, iTextStyle);
	if (iESP & ESPEnum::UberText)
		H::Draw.StringOutlined(fFont, r, y + h, tGroup.ElementColor(ESPEnum::UberText), tOutline, ALIGN_TOPLEFT, std::format("{:.0f}%", flUber * 100).c_str(), true, iTextStyle);

	// --- Class icon (heavy), position + simple-SVG option mirror CESP ---
	if (iESP & ESPEnum::ClassIcon)
	{
		const bool bSimple = tGroup.m_bSimpleClassIcon;
		float flDistMul = 1.f;
		if (tGroup.m_bClassIconDistanceScale)
			flDistMul = std::clamp(float(h) / float(H::Draw.Scale(100)), 0.35f, 1.f);
		const int iSize = H::Draw.Scale(18.f * tGroup.m_flClassIconScale * flDistMul * (bSimple ? 1.1f : 1.f), Scale_Round);
		int iIconX = m, iIconY = t - tOffset; EAlign eIconAlign = ALIGN_BOTTOM;
		switch (tGroup.m_iClassIconPosition)
		{
		case 1: iIconX = m;           iIconY = b + bOffset; eIconAlign = ALIGN_TOP;   bOffset += iSize + H::Draw.Scale(2); break;
		case 2: iIconX = l - lOffset; iIconY = y + h / 2;   eIconAlign = ALIGN_RIGHT; lOffset += iSize + iSpace;           break;
		case 3: iIconX = r + rOffset; iIconY = y + h / 2;   eIconAlign = ALIGN_LEFT;  rOffset += iSize + iSpace;           break;
		default: iIconX = m;          iIconY = t - tOffset; eIconAlign = ALIGN_BOTTOM; tOffset += iSize + H::Draw.Scale(2); break;
		}
		if (bSimple)
		{
			const int iIcon = GetSimpleClassIconTex(TF_CLASS_HEAVY, iSize);
			if (tGroup.m_bSimpleClassIconOutline)
			{
				const int iO = std::max(1, int(H::Draw.Scale(1, Scale_Round)));
				const Color_t tBlack = { 0, 0, 0, tGroup.m_tSimpleClassIconColor.a };
				for (int dy = -1; dy <= 1; ++dy)
					for (int dx = -1; dx <= 1; ++dx)
						if (dx || dy)
							H::Draw.TextureId(iIcon, iIconX + dx * iO, iIconY + dy * iO, iSize, iSize, tBlack, eIconAlign);
			}
			H::Draw.TextureId(iIcon, iIconX, iIconY, iSize, iSize, tGroup.m_tSimpleClassIconColor, eIconAlign);
		}
		else
			H::Draw.Texture("hud/leaderboard_class_heavy.vtf", iIconX, iIconY, iSize, iSize, eIconAlign);
	}

	// --- Weapon icon (fists), below the box like CESP ---
	if (iESP & ESPEnum::WeaponIcon)
	{
		if (CHudTexture* pIcon = H::Draw.GetIcon("d_fists"))
		{
			const float flW = float(pIcon->Width()), flH = float(pIcon->Height());
			const float flScale = H::Draw.Scale(std::min((w + 40) / 2.f, 80.f) / std::max(flW, flH * 2.f));
			H::Draw.DrawHudTexture(m - flW / 2.f * flScale, float(b + bOffset), flScale, pIcon, tGroup.ElementColor(ESPEnum::WeaponIcon));
		}
	}

	// --- Status "flags" labels (demo: shows whichever effects are selected) ---
	if (iESP & ESPEnum::Labels)
	{
		const int iFlags = Vars::ESP::FlagEffects.Value;
		const Color_t tBad = Vars::Colors::IndicatorTextBad.Value, tAct = Vars::Menu::Theme::Active.Value;
		std::vector<std::pair<const char*, Color_t>> vLabels;
		if (iFlags & Vars::ESP::FlagEffectsEnum::Invuln)    vLabels.emplace_back("Invuln", tBad);
		if (iFlags & Vars::ESP::FlagEffectsEnum::Invis)     vLabels.emplace_back("Invis", tAct);
		if (iFlags & Vars::ESP::FlagEffectsEnum::Crit)      vLabels.emplace_back("Crit", tBad);
		if (iFlags & Vars::ESP::FlagEffectsEnum::Disguised) vLabels.emplace_back("Disguised", tAct);
		if (!vLabels.empty())
		{
			const auto& fLabel = H::Fonts.GetFont(FONT_ESP_LABEL);
			const int nLabelTall = fLabel.m_nTall + H::Draw.Scale(2);
			int iLabelOff = 0;
			for (auto& [szText, tColor] : vLabels)
			{
				switch (Vars::ESP::FlagPosition.Value)
				{
				case Vars::ESP::FlagPositionEnum::Left:
					H::Draw.StringOutlined(fLabel, l - lOffset, y - H::Draw.Scale(2) + iLabelOff, tColor, tOutline, ALIGN_TOPRIGHT, szText, true, iTextStyle);
					iLabelOff += nLabelTall;
					break;
				case Vars::ESP::FlagPositionEnum::Above:
					H::Draw.StringOutlined(fLabel, m, t - tOffset, tColor, tOutline, ALIGN_BOTTOM, szText, true, iTextStyle);
					tOffset += nLabelTall;
					break;
				case Vars::ESP::FlagPositionEnum::Below:
					H::Draw.StringOutlined(fLabel, m, b + bOffset, tColor, tOutline, ALIGN_TOP, szText, true, iTextStyle);
					bOffset += nLabelTall;
					break;
				default: // Right
					H::Draw.StringOutlined(fLabel, r, y - H::Draw.Scale(2) + rOffset, tColor, tOutline, ALIGN_TOPLEFT, szText, true, iTextStyle);
					rOffset += nLabelTall;
					break;
				}
			}
		}
	}

	H::Draw.EndClipping();
}

// ---------------------------------------------------------------------------
void CModelPreview::Run()
{
	if (!m_bWantRender)
		return;

	if (m_iW <= 0 || m_iH <= 0)
		return;

	g_MPDbg.iRectW = m_iW;
	g_MPDbg.iRectH = m_iH;

	const bool bModel = EnsureModel();
	g_MPDbg.bModel = bModel && m_pModel;
	if (!bModel)
		return;

	const bool bEnt = EnsureEntity();
	g_MPDbg.bEntity = bEnt && m_pEnt;
	if (!bEnt)
		return;

	if (!EnsureRenderTarget())
		return;

	// Re-assert the model + cached model_t* whenever it can have gone stale. Two triggers:
	// 1. We crossed between menu and match (model_t* / precache index differ).
	const bool bConnected = I::EngineClient->IsConnected();
	const int  iLiveIndex = bConnected ? I::ModelInfoClient->GetModelIndex(m_szModel) : -1;
	if (bConnected != (m_iBuiltConnected == 1) || (bConnected && iLiveIndex != m_iLiveIndex) || !m_pModel)
	{
		m_bDrawFaulted = false; // new connection state / map: re-arm the 3D draw (retry after a prior fault)
		m_bModelLoaded = false;
		EnsureModel(); // refresh m_pModel (used for render bounds / SetupBones); a stale model_t crashes/garbages

		int iIndex = bConnected ? iLiveIndex : I::ModelInfoClient->GetModelIndex(m_szModel);
		if (iIndex < 0)
			iIndex = 1;
		fnCBaseEntity_SetModelByIndex(m_pEnt, static_cast<unsigned short>(iIndex));
		m_iBuiltConnected = bConnected ? 1 : 0;
		m_iLiveIndex      = iLiveIndex;
	}

	// In-game guard: if the precache index isn't valid yet (mid level-load) or the model_t didn't resolve,
	// skip this frame instead of drawing a stale/garbage model (which flickered/vanished). Self-heals next
	if (bConnected && (iLiveIndex < 0 || !m_pModel))
		return;

	RenderToTexture();
	BlitToScreen();
}

// ---------------------------------------------------------------------------
void CModelPreview::Free()
{
	if (m_pRTMaterial)
	{
		m_pRTMaterial->DecrementReferenceCount();
		m_pRTMaterial->DeleteIfUnreferenced();
		m_pRTMaterial = nullptr;
	}
	if (m_pRTTexture)
	{
		m_pRTTexture->DecrementReferenceCount();
		m_pRTTexture->DeleteIfUnreferenced();
		m_pRTTexture = nullptr;
	}

	if (m_pEnt)
	{
		free(m_pEnt);
		m_pEnt = nullptr;
	}
	m_bEntInit     = false;
	m_bModelLoaded = false;
	m_pModel       = nullptr;
}
