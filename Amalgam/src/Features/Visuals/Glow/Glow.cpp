#include "Glow.h"

#include "../Groups/Groups.h"
#include "../Materials/Materials.h"
#include "../FakeAngle/FakeAngle.h"
#include "../../Backtrack/Backtrack.h"

void CGlow::SetupBegin(IMatRenderContext* pRenderContext)
{
	m_tOriginalColor = I::RenderView->GetColorModulation();
	m_flOriginalBlend = I::RenderView->GetBlend();
	I::ModelRender->GetMaterialOverride(&m_pOriginalMaterial, &m_iOriginalOverride);

	I::RenderView->SetBlend(0.f);
	I::RenderView->SetColorModulation(1.f, 1.f, 1.f);
	I::ModelRender->ForcedMaterialOverride(m_pMatGlowColor);

	pRenderContext->SetStencilEnable(true);
	pRenderContext->SetStencilCompareFunction(STENCILCOMPARISONFUNCTION_ALWAYS);
	pRenderContext->SetStencilPassOperation(STENCILOPERATION_REPLACE);
	pRenderContext->SetStencilFailOperation(STENCILOPERATION_KEEP);
	pRenderContext->SetStencilZFailOperation(STENCILOPERATION_REPLACE);
	pRenderContext->SetStencilReferenceValue(1);
	pRenderContext->SetStencilWriteMask(0xFF);
	pRenderContext->SetStencilTestMask(0x0);
}
void CGlow::SetupMid(IMatRenderContext* pRenderContext, int w, int h)
{
	pRenderContext->SetStencilEnable(false);

	pRenderContext->PushRenderTargetAndViewport();
	pRenderContext->SetRenderTarget(m_pRenderBuffer1);
	pRenderContext->Viewport(0, 0, w, h);
	pRenderContext->ClearColor4ub(0, 0, 0, 0);
	pRenderContext->ClearBuffers(true, false, false);
}
void CGlow::SetupEnd(Glow_t tGlow, IMatRenderContext* pRenderContext, int w, int h)
{
	pRenderContext->PopRenderTargetAndViewport();

	if (tGlow.Blur)
	{
		m_pBloomAmount->SetFloatValue(tGlow.Blur);

		pRenderContext->PushRenderTargetAndViewport();
		{
			pRenderContext->Viewport(0, 0, w, h);
			pRenderContext->SetRenderTarget(m_pRenderBuffer2);
			pRenderContext->DrawScreenSpaceRectangle(m_pMatBlurX, 0, 0, w, h, 0.f, 0.f, w - 1, h - 1, w, h);
			pRenderContext->SetRenderTarget(m_pRenderBuffer1);
			pRenderContext->DrawScreenSpaceRectangle(m_pMatBlurY, 0, 0, w, h, 0.f, 0.f, w - 1, h - 1, w, h);
		}
		pRenderContext->PopRenderTargetAndViewport();
	}

	pRenderContext->SetStencilEnable(true);
	pRenderContext->SetStencilCompareFunction(STENCILCOMPARISONFUNCTION_EQUAL);
	pRenderContext->SetStencilPassOperation(STENCILOPERATION_KEEP);
	pRenderContext->SetStencilFailOperation(STENCILOPERATION_KEEP);
	pRenderContext->SetStencilZFailOperation(STENCILOPERATION_KEEP);
	pRenderContext->SetStencilReferenceValue(0);
	pRenderContext->SetStencilWriteMask(0x0);
	pRenderContext->SetStencilTestMask(0xFF);

	if (tGlow.Stencil)
	{
		int iSide = (tGlow.Stencil + 1) / 2.f;
		pRenderContext->DrawScreenSpaceRectangle(m_pMatHaloAddToScreen, -iSide, 0, w, h, 0.f, 0.f, w - 1, h - 1, w, h);
		pRenderContext->DrawScreenSpaceRectangle(m_pMatHaloAddToScreen, 0, -iSide, w, h, 0.f, 0.f, w - 1, h - 1, w, h);
		pRenderContext->DrawScreenSpaceRectangle(m_pMatHaloAddToScreen, iSide, 0, w, h, 0.f, 0.f, w - 1, h - 1, w, h);
		pRenderContext->DrawScreenSpaceRectangle(m_pMatHaloAddToScreen, 0, iSide, w, h, 0.f, 0.f, w - 1, h - 1, w, h);
		if (int iCorner = tGlow.Stencil / 2.f)
		{
			pRenderContext->DrawScreenSpaceRectangle(m_pMatHaloAddToScreen, -iCorner, -iCorner, w, h, 0.f, 0.f, w - 1, h - 1, w, h);
			pRenderContext->DrawScreenSpaceRectangle(m_pMatHaloAddToScreen, iCorner, iCorner, w, h, 0.f, 0.f, w - 1, h - 1, w, h);
			pRenderContext->DrawScreenSpaceRectangle(m_pMatHaloAddToScreen, iCorner, -iCorner, w, h, 0.f, 0.f, w - 1, h - 1, w, h);
			pRenderContext->DrawScreenSpaceRectangle(m_pMatHaloAddToScreen, -iCorner, iCorner, w, h, 0.f, 0.f, w - 1, h - 1, w, h);
		}
	}
	if (tGlow.Blur)
		pRenderContext->DrawScreenSpaceRectangle(m_pMatHaloAddToScreen, 0, 0, w, h, 0.f, 0.f, w - 1, h - 1, w, h);

	pRenderContext->SetStencilEnable(false);

	I::RenderView->SetColorModulation(m_tOriginalColor);
	I::RenderView->SetBlend(m_flOriginalBlend);
	I::ModelRender->ForcedMaterialOverride(m_pOriginalMaterial, m_iOriginalOverride);
}

void CGlow::DrawModel(CBaseEntity* pEntity)
{
	m_bRendering = true;

	if (pEntity->IsPlayer())
	{
		auto pPlayer = pEntity->As<CTFPlayer>();
		float flOldInvisibility = pPlayer->m_flInvisibility();
		pPlayer->m_flInvisibility() = 0.f;
		pEntity->DrawModel(STUDIO_RENDER | STUDIO_NOSHADOWS);
		pPlayer->m_flInvisibility() = flOldInvisibility;
	}
	else
		pEntity->DrawModel(STUDIO_RENDER | STUDIO_NOSHADOWS);

	m_bRendering = false;
}



void CGlow::Store(CTFPlayer* pLocal)
{
	m_mEntities.clear();
	if (!pLocal || !F::Groups.GroupsActive())
		return;

	for (auto& [pEntity, pGroup] : F::Groups.GetGroup())
	{
		if (pEntity->IsDormant() || !pEntity->ShouldDraw())
			continue;

		Color_t tColor = F::Groups.GetColor(pEntity, pGroup);
		if (pGroup->m_tGlow()
			&& SDK::IsOnScreen(pEntity, pEntity->IsBaseCombatWeapon() || pEntity->IsWearable()))
		{
			// Visible/invisible glow split (player & building sections): use the invisible color when the
			// local player has no clear line of sight to the entity. Keeps the visible color's alpha.
			Color_t tGlowColor = tColor;
			if (pGroup->m_bGlowVisCheck && (pEntity->IsPlayer() || pEntity->IsBuilding())
				&& !SDK::VisPos(pLocal, pEntity, pLocal->GetEyePosition(), pEntity->GetCenter()))
				tGlowColor = pGroup->m_tGlowInvisibleColor;

			m_mEntities[pGroup->m_tGlow].emplace_back(pEntity, tGlowColor);
		}

		if (pEntity->IsPlayer() && pEntity != pLocal && pGroup->m_bBacktrack && pGroup->m_tBacktrackGlow()
			&& (F::Backtrack.GetFakeLatency() || F::Backtrack.GetFakeInterp() > G::Lerp || F::Backtrack.GetWindow()))
		{
			auto pWeapon = H::Entities.GetWeapon();
			if (pWeapon && (pGroup->m_iBacktrackDraw & BacktrackEnum::Always || G::PrimaryWeaponType != EWeaponType::PROJECTILE))
			{
				bool bShowFriendly = false, bShowEnemy = true;
				if (G::PrimaryWeaponType == EWeaponType::MELEE && SDK::AttribHookValue(0, "speed_buff_ally", pWeapon) > 0)
					bShowFriendly = true;
				else if (pWeapon->GetWeaponID() == TF_WEAPON_MEDIGUN)
					bShowFriendly = true, bShowEnemy = false;

				if (bShowEnemy && pEntity->m_iTeamNum() != pLocal->m_iTeamNum() || bShowFriendly && pEntity->m_iTeamNum() == pLocal->m_iTeamNum())
					m_mEntities[pGroup->m_tBacktrackGlow].emplace_back(pEntity, tColor, 1 | (pGroup->m_iBacktrackDraw << 1));
			}
		}
	}

	Group_t* pGroup = nullptr;
	if (F::FakeAngle.bDrawChams && F::FakeAngle.bBonesSetup
		&& F::Groups.GetGroup(TargetsEnum::FakeAngle, pGroup) && pGroup->m_tGlow())
	{	// fakeangle
		m_mEntities[pGroup->m_tGlow].emplace_back(pLocal, pGroup->m_tColor, 1 | (true << 1));
	}
}

void CGlow::RenderMain()
{
	const int w = H::Draw.m_nScreenW, h = H::Draw.m_nScreenH;
	if (w < 1 || h < 1 || w > 8192 || h > 4320)
		return;

	auto pRenderContext = I::MaterialSystem->GetRenderContext();
	if (!pRenderContext || !m_pMatGlowColor || !m_pMatBlurX || !m_pMatBlurY || !m_pMatHaloAddToScreen)
		return F::Materials.ReloadMaterials();

	for (auto& [tGlow, vInfo] : m_mEntities)
	{
		SetupBegin(pRenderContext);
		for (auto& tInfo : vInfo)
		{
			m_iFlags = tInfo.m_iFlags;
			DrawModel(tInfo.m_pEntity);
			m_iFlags = false;
		}
		SetupMid(pRenderContext, w, h);
		for (auto& tInfo : vInfo)
		{
			I::RenderView->SetColorModulation(tInfo.m_cColor);
			I::RenderView->SetBlend(tInfo.m_cColor.a / 255.f);

			m_iFlags = tInfo.m_iFlags;
			DrawModel(tInfo.m_pEntity);
			m_iFlags = false;
		}
		SetupEnd(tGlow, pRenderContext, w, h);
	}
}

void CGlow::RenderBacktrack(const DrawModelState_t& pState, const ModelRenderInfo_t& pInfo)
{
	auto pEntity = I::ClientEntityList->GetClientEntity(pInfo.entity_index)->As<CTFPlayer>();
	if (!pEntity || !pEntity->IsPlayer())
		return;



	std::vector<TickRecord*> vRecords = {};
	if (!F::Backtrack.GetRecords(pEntity, vRecords))
		return;
	vRecords = F::Backtrack.GetValidRecords(vRecords);
	if (!vRecords.size())
		return;

	int iFlags = (~1 & m_iFlags) >> 1;
	bool bDrawLast = iFlags & BacktrackEnum::Last;
	bool bDrawFirst = iFlags & BacktrackEnum::First;

	auto drawModel = [&](Vec3& vOrigin, const DrawModelState_t& pState, const ModelRenderInfo_t& pInfo, matrix3x4* pBoneToWorld, float flBlend)
		{
			if (!SDK::IsOnScreen(pEntity, vOrigin))
				return;

			//float flOriginalBlend = I::RenderView->GetBlend();
			//I::RenderView->SetBlend(flBlend * flOriginalBlend);
			static auto IVModelRender_DrawModelExecute = U::Hooks.m_mHooks["IVModelRender_DrawModelExecute"];
			IVModelRender_DrawModelExecute->Call<void>(I::ModelRender, pState, pInfo, pBoneToWorld);
			//I::RenderView->SetBlend(flOriginalBlend);
		};
	if (!bDrawLast && !bDrawFirst)
	{
		for (auto pRecord : vRecords)
		{
			if (float flBlend = Math::RemapVal(pEntity->GetAbsOrigin().DistTo(pRecord->m_vOrigin), 1.f, 24.f, 0.f, 1.f))
				drawModel(pRecord->m_vOrigin, pState, pInfo, pRecord->m_aBones, flBlend);
		}
	}
	else
	{
		if (bDrawLast)
		{
			auto pRecord = vRecords.back();
			if (float flBlend = Math::RemapVal(pEntity->GetAbsOrigin().DistTo(pRecord->m_vOrigin), 1.f, 24.f, 0.f, 1.f))
				drawModel(pRecord->m_vOrigin, pState, pInfo, pRecord->m_aBones, flBlend);
		}
		if (bDrawFirst)
		{
			auto pRecord = vRecords.front();
			if (float flBlend = Math::RemapVal(pEntity->GetAbsOrigin().DistTo(pRecord->m_vOrigin), 1.f, 24.f, 0.f, 1.f))
				drawModel(pRecord->m_vOrigin, pState, pInfo, pRecord->m_aBones, flBlend);
		}
	}
}
void CGlow::RenderFakeAngle(const DrawModelState_t& pState, const ModelRenderInfo_t& pInfo)
{
	static auto IVModelRender_DrawModelExecute = U::Hooks.m_mHooks["IVModelRender_DrawModelExecute"];
	IVModelRender_DrawModelExecute->Call<void>(I::ModelRender, pState, pInfo, F::FakeAngle.aBones);
}
void CGlow::RenderHandler(const DrawModelState_t& pState, const ModelRenderInfo_t& pInfo, matrix3x4* pBoneToWorld)
{
	if (!m_iFlags)
	{
		static auto IVModelRender_DrawModelExecute = U::Hooks.m_mHooks["IVModelRender_DrawModelExecute"];
		IVModelRender_DrawModelExecute->Call<void>(I::ModelRender, pState, pInfo, pBoneToWorld);
	}
	else
	{
		if (pInfo.entity_index != I::EngineClient->GetLocalPlayer())
			RenderBacktrack(pState, pInfo);
		else
			RenderFakeAngle(pState, pInfo);
	}
}

void CGlow::RenderViewmodel(void* ecx, int flags)
{
	if (!F::Groups.GroupsActive())
		return;

	const int w = H::Draw.m_nScreenW, h = H::Draw.m_nScreenH;
	if (w < 1 || h < 1 || w > 8192 || h > 4320)
		return;

	auto pRenderContext = I::MaterialSystem->GetRenderContext();
	if (!pRenderContext || !m_pMatGlowColor || !m_pMatBlurX || !m_pMatBlurY || !m_pMatHaloAddToScreen)
		return F::Materials.ReloadMaterials();



	Group_t* pGroup = nullptr;
	if (!F::Groups.GetGroup(TargetsEnum::ViewmodelWeapon, pGroup) || !pGroup->m_tGlow())
		return;

	static auto CBaseAnimating_InternalDrawModel = U::Hooks.m_mHooks["CBaseAnimating_InternalDrawModel"];

	pRenderContext->CullMode(MATERIAL_CULLMODE_CCW); // glow won't work properly with MATERIAL_CULLMODE_CW
	SetupBegin(pRenderContext);
	CBaseAnimating_InternalDrawModel->Call<int>(ecx, flags);
	SetupMid(pRenderContext, w, h);
	I::RenderView->SetColorModulation(pGroup->m_tColor);
	I::RenderView->SetBlend(pGroup->m_tColor.a / 255.f);
	CBaseAnimating_InternalDrawModel->Call<int>(ecx, flags);
	SetupEnd(pGroup->m_tGlow, pRenderContext, w, h);

	pRenderContext->CullMode(G::FlipViewmodels ? MATERIAL_CULLMODE_CW : MATERIAL_CULLMODE_CCW);
}
void CGlow::RenderViewmodel(const DrawModelState_t& pState, const ModelRenderInfo_t& pInfo, matrix3x4* pBoneToWorld)
{
	if (!F::Groups.GroupsActive())
		return;

	const int w = H::Draw.m_nScreenW, h = H::Draw.m_nScreenH;
	if (w < 1 || h < 1 || w > 8192 || h > 4320)
		return;

	auto pRenderContext = I::MaterialSystem->GetRenderContext();
	if (!pRenderContext || !m_pMatGlowColor || !m_pMatBlurX || !m_pMatBlurY || !m_pMatHaloAddToScreen)
		return F::Materials.ReloadMaterials();



	Group_t* pGroup = nullptr;
	if (!F::Groups.GetGroup(TargetsEnum::ViewmodelHands, pGroup) || !pGroup->m_tGlow())
		return;

	static auto IVModelRender_DrawModelExecute = U::Hooks.m_mHooks["IVModelRender_DrawModelExecute"];

	pRenderContext->CullMode(MATERIAL_CULLMODE_CCW); // glow won't work properly with MATERIAL_CULLMODE_CW
	SetupBegin(pRenderContext);
	IVModelRender_DrawModelExecute->Call<void>(I::ModelRender, pState, pInfo, pBoneToWorld);
	SetupMid(pRenderContext, w, h);
	I::RenderView->SetColorModulation(pGroup->m_tColor);
	I::RenderView->SetBlend(pGroup->m_tColor.a / 255.f);
	IVModelRender_DrawModelExecute->Call<void>(I::ModelRender, pState, pInfo, pBoneToWorld);
	SetupEnd(pGroup->m_tGlow, pRenderContext, w, h);

	pRenderContext->CullMode(G::FlipViewmodels ? MATERIAL_CULLMODE_CW : MATERIAL_CULLMODE_CCW);
}



void CGlow::DrawPreview(CBaseEntity* pEntity, const Glow_t& tGlow, const Color_t& tColor, IMatRenderContext* pRenderContext, int w, int h, float flScale)
{
	if (w < 1 || h < 1)
		return;
	if (!m_pMatGlowColor || !m_pMatPreviewBlurX || !m_pMatPreviewBlurY || !m_pMatPreviewHalo || !m_pPreviewBuf1)
		return;

	// The preview buffers are full-frame; we only render into their top-left w*h corner (our preview
	// viewport). DrawScreenSpaceRectangle's last two args are the TEXTURE dims used to normalize the src
	const int tw = m_pPreviewBuf1->GetActualWidth();
	const int th = m_pPreviewBuf1->GetActualHeight();

	// 1) Silhouette -> stencil (ref 1) in the caller's RT. SetupBegin also forces the glow-colour material
	// + blend 0 (so the model is invisible here, only the stencil is written) and saves render state.
	SetupBegin(pRenderContext);
	DrawModel(pEntity);

	// 2) Coloured model into PreviewBuf1's top-left w*h corner.
	pRenderContext->SetStencilEnable(false);
	pRenderContext->PushRenderTargetAndViewport();
	pRenderContext->SetRenderTarget(m_pPreviewBuf1);
	pRenderContext->Viewport(0, 0, tw, th); // clear the WHOLE buffer so the blur margins (x>w) are clean black/alpha0
	pRenderContext->ClearColor4ub(0, 0, 0, 0);
	pRenderContext->ClearBuffers(true, false, false);
	pRenderContext->Viewport(0, 0, w, h);   // render the model into the top-left corner only
	pRenderContext->OverrideAlphaWriteEnable(true, true);
	I::RenderView->SetColorModulation(tColor);
	I::RenderView->SetBlend(tColor.a / 255.f);
	DrawModel(pEntity);
	pRenderContext->OverrideAlphaWriteEnable(false, true);
	pRenderContext->PopRenderTargetAndViewport(); // back to the caller's RT + viewport

	// 3) Separable blur PreviewBuf1 <-> 2 over the same corner. The blur shader is float4, so it spreads the
	// coverage ALPHA into a soft halo (rgb stays the flat glow colour from the step-2 clear). The shader's
	if (tGlow.Blur)
	{
		m_pPreviewBloomAmount->SetFloatValue(1.f);
		int iPasses = int(tGlow.Blur * flScale + 0.5f);
		if (iPasses < 1)  iPasses = 1;
		if (iPasses > 24) iPasses = 24;

		pRenderContext->PushRenderTargetAndViewport();
		// Scratch buf2 must also be clean past the w*h corner or multi-pass taps pull in stale texels.
		pRenderContext->SetRenderTarget(m_pPreviewBuf2);
		pRenderContext->Viewport(0, 0, tw, th);
		pRenderContext->ClearColor4ub(0, 0, 0, 0);
		pRenderContext->ClearBuffers(true, false, false);

		pRenderContext->Viewport(0, 0, w, h);
		for (int p = 0; p < iPasses; p++)
		{
			pRenderContext->SetRenderTarget(m_pPreviewBuf2);
			pRenderContext->DrawScreenSpaceRectangle(m_pMatPreviewBlurX, 0, 0, w, h, 0.f, 0.f, float(w - 1), float(h - 1), tw, th);
			pRenderContext->SetRenderTarget(m_pPreviewBuf1);
			pRenderContext->DrawScreenSpaceRectangle(m_pMatPreviewBlurY, 0, 0, w, h, 0.f, 0.f, float(w - 1), float(h - 1), tw, th);
		}
		pRenderContext->PopRenderTargetAndViewport();
	}

	// 4) Composite the halo onto the caller's RT, masked to OUTSIDE the silhouette (stencil EQUAL ref 0) -
	// EXACTLY the in-game SetupEnd, just with alpha writes forced on. The $additive (ONE,ONE) blit adds
	pRenderContext->OverrideAlphaWriteEnable(true, true);
	pRenderContext->SetStencilEnable(true);
	pRenderContext->SetStencilCompareFunction(STENCILCOMPARISONFUNCTION_EQUAL);
	pRenderContext->SetStencilPassOperation(STENCILOPERATION_KEEP);
	pRenderContext->SetStencilFailOperation(STENCILOPERATION_KEEP);
	pRenderContext->SetStencilZFailOperation(STENCILOPERATION_KEEP);
	pRenderContext->SetStencilReferenceValue(0);
	pRenderContext->SetStencilWriteMask(0x0);
	pRenderContext->SetStencilTestMask(0xFF);

	constexpr float kHaloThick = 1.6f;
	if (tGlow.Stencil)
	{
		int iSide = int(((tGlow.Stencil + 1) / 2) * flScale * kHaloThick);
		if (iSide < 1) iSide = 1;
		pRenderContext->DrawScreenSpaceRectangle(m_pMatPreviewHalo, -iSide, 0, w, h, 0.f, 0.f, float(w - 1), float(h - 1), tw, th);
		pRenderContext->DrawScreenSpaceRectangle(m_pMatPreviewHalo, 0, -iSide, w, h, 0.f, 0.f, float(w - 1), float(h - 1), tw, th);
		pRenderContext->DrawScreenSpaceRectangle(m_pMatPreviewHalo, iSide, 0, w, h, 0.f, 0.f, float(w - 1), float(h - 1), tw, th);
		pRenderContext->DrawScreenSpaceRectangle(m_pMatPreviewHalo, 0, iSide, w, h, 0.f, 0.f, float(w - 1), float(h - 1), tw, th);
		if (const int iCorner = int((tGlow.Stencil / 2) * flScale * kHaloThick))
		{
			pRenderContext->DrawScreenSpaceRectangle(m_pMatPreviewHalo, -iCorner, -iCorner, w, h, 0.f, 0.f, float(w - 1), float(h - 1), tw, th);
			pRenderContext->DrawScreenSpaceRectangle(m_pMatPreviewHalo, iCorner, iCorner, w, h, 0.f, 0.f, float(w - 1), float(h - 1), tw, th);
			pRenderContext->DrawScreenSpaceRectangle(m_pMatPreviewHalo, iCorner, -iCorner, w, h, 0.f, 0.f, float(w - 1), float(h - 1), tw, th);
			pRenderContext->DrawScreenSpaceRectangle(m_pMatPreviewHalo, -iCorner, iCorner, w, h, 0.f, 0.f, float(w - 1), float(h - 1), tw, th);
		}
	}
	if (tGlow.Blur)
		pRenderContext->DrawScreenSpaceRectangle(m_pMatPreviewHalo, 0, 0, w, h, 0.f, 0.f, float(w - 1), float(h - 1), tw, th);

	pRenderContext->SetStencilEnable(false);
	pRenderContext->OverrideAlphaWriteEnable(false, true);

	// Restore the render state SetupBegin saved.
	I::RenderView->SetColorModulation(m_tOriginalColor);
	I::RenderView->SetBlend(m_flOriginalBlend);
	I::ModelRender->ForcedMaterialOverride(m_pOriginalMaterial, m_iOriginalOverride);
}

void CGlow::Initialize()
{
	int nWidth, nHeight; I::MatSystemSurface->GetScreenSize(nWidth, nHeight);

	if (!m_pMatGlowColor)
	{
		m_pMatGlowColor = I::MaterialSystem->FindMaterial("dev/glow_color", TEXTURE_GROUP_OTHER);
		m_pMatGlowColor->IncrementReferenceCount();
		F::Materials.m_mMatList[m_pMatGlowColor];
	}

	// Use RT_SIZE_FULL_FRAME_BUFFER (not RT_SIZE_LITERAL) so the glow buffers track the frame buffer.
	if (!m_pRenderBuffer1)
	{
		m_pRenderBuffer1 = I::MaterialSystem->CreateNamedRenderTargetTextureEx(
			"RenderBuffer1",
			nWidth, nHeight,
			RT_SIZE_FULL_FRAME_BUFFER,
			IMAGE_FORMAT_RGB888,
			MATERIAL_RT_DEPTH_SHARED,
			TEXTUREFLAGS_CLAMPS | TEXTUREFLAGS_CLAMPT | TEXTUREFLAGS_EIGHTBITALPHA,
			CREATERENDERTARGETFLAGS_HDR
		);
		m_pRenderBuffer1->IncrementReferenceCount();
	}

	if (!m_pRenderBuffer2)
	{
		m_pRenderBuffer2 = I::MaterialSystem->CreateNamedRenderTargetTextureEx(
			"RenderBuffer2",
			nWidth, nHeight,
			RT_SIZE_FULL_FRAME_BUFFER,
			IMAGE_FORMAT_RGB888,
			MATERIAL_RT_DEPTH_SHARED,
			TEXTUREFLAGS_CLAMPS | TEXTUREFLAGS_CLAMPT | TEXTUREFLAGS_EIGHTBITALPHA,
			CREATERENDERTARGETFLAGS_HDR
		);
		m_pRenderBuffer2->IncrementReferenceCount();
	}

	if (!m_pMatHaloAddToScreen)
	{
		KeyValues* kv = new KeyValues("UnlitGeneric");
		kv->SetString("$basetexture", "RenderBuffer1");
		kv->SetString("$additive", "1");
		m_pMatHaloAddToScreen = F::Materials.Create("MatHaloAddToScreen", kv);
	}

	if (!m_pMatBlurX)
	{
		KeyValues* kv = new KeyValues("BlurFilterX");
		kv->SetString("$basetexture", "RenderBuffer1");
		m_pMatBlurX = F::Materials.Create("MatBlurX", kv);
	}

	if (!m_pMatBlurY)
	{
		KeyValues* kv = new KeyValues("BlurFilterY");
		kv->SetString("$basetexture", "RenderBuffer2");
		m_pMatBlurY = F::Materials.Create("MatBlurY", kv);
		m_pBloomAmount = m_pMatBlurY->FindVar("$bloomamount", nullptr);
	}

	// --- Model Preview glow (transparent-panel path) ---
	if (!m_pPreviewBuf1)
	{
		m_pPreviewBuf1 = I::MaterialSystem->CreateNamedRenderTargetTextureEx(
			"AletheriumPreviewGlow1",
			nWidth, nHeight,
			RT_SIZE_FULL_FRAME_BUFFER,
			IMAGE_FORMAT_RGBA8888,
			MATERIAL_RT_DEPTH_SHARED,
			TEXTUREFLAGS_CLAMPS | TEXTUREFLAGS_CLAMPT | TEXTUREFLAGS_EIGHTBITALPHA,
			CREATERENDERTARGETFLAGS_HDR
		);
		m_pPreviewBuf1->IncrementReferenceCount();
	}

	if (!m_pPreviewBuf2)
	{
		m_pPreviewBuf2 = I::MaterialSystem->CreateNamedRenderTargetTextureEx(
			"AletheriumPreviewGlow2",
			nWidth, nHeight,
			RT_SIZE_FULL_FRAME_BUFFER,
			IMAGE_FORMAT_RGBA8888,
			MATERIAL_RT_DEPTH_SHARED,
			TEXTUREFLAGS_CLAMPS | TEXTUREFLAGS_CLAMPT | TEXTUREFLAGS_EIGHTBITALPHA,
			CREATERENDERTARGETFLAGS_HDR
		);
		m_pPreviewBuf2->IncrementReferenceCount();
	}

	// $additive (ONE,ONE) - matches the in-game halo (m_pMatHaloAddToScreen). The buffer is cleared black
	// and holds the dimmed glow colour (alpha baked in via SetBlend at draw), so additive adds light only
	if (!m_pMatPreviewHalo)
	{
		KeyValues* kv = new KeyValues("UnlitGeneric");
		kv->SetString("$basetexture", "AletheriumPreviewGlow1");
		kv->SetInt("$additive", 1);
		kv->SetInt("$ignorez", 1);
		kv->SetInt("$nocull", 1);
		m_pMatPreviewHalo = F::Materials.Create("AletheriumPreviewHalo", kv);
	}

	// The blur shader is float4 (it blurs alpha too), so blurring these RGBA buffers softens the coverage
	// alpha into the halo just like the live glow softens colour.
	if (!m_pMatPreviewBlurX)
	{
		KeyValues* kv = new KeyValues("BlurFilterX");
		kv->SetString("$basetexture", "AletheriumPreviewGlow1");
		m_pMatPreviewBlurX = F::Materials.Create("AletheriumPreviewBlurX", kv);
	}

	if (!m_pMatPreviewBlurY)
	{
		KeyValues* kv = new KeyValues("BlurFilterY");
		kv->SetString("$basetexture", "AletheriumPreviewGlow2");
		m_pMatPreviewBlurY = F::Materials.Create("AletheriumPreviewBlurY", kv);
		m_pPreviewBloomAmount = m_pMatPreviewBlurY->FindVar("$bloomamount", nullptr);
	}
}

void CGlow::Unload()
{
	if (m_pMatGlowColor)
	{
		m_pMatGlowColor->DecrementReferenceCount();
		m_pMatGlowColor->DeleteIfUnreferenced();
		m_pMatGlowColor = nullptr;
	}

	if (m_pMatBlurX)
	{
		m_pMatBlurX->DecrementReferenceCount();
		m_pMatBlurX->DeleteIfUnreferenced();
		m_pMatBlurX = nullptr;
	}

	if (m_pMatBlurY)
	{
		m_pMatBlurY->DecrementReferenceCount();
		m_pMatBlurY->DeleteIfUnreferenced();
		m_pMatBlurY = nullptr;
	}

	if (m_pMatHaloAddToScreen)
	{
		m_pMatHaloAddToScreen->DecrementReferenceCount();
		m_pMatHaloAddToScreen->DeleteIfUnreferenced();
		m_pMatHaloAddToScreen = nullptr;
	}

	if (m_pRenderBuffer1)
	{
		m_pRenderBuffer1->DecrementReferenceCount();
		m_pRenderBuffer1->DeleteIfUnreferenced();
		m_pRenderBuffer1 = nullptr;
	}

	if (m_pRenderBuffer2)
	{
		m_pRenderBuffer2->DecrementReferenceCount();
		m_pRenderBuffer2->DeleteIfUnreferenced();
		m_pRenderBuffer2 = nullptr;
	}

	if (m_pMatPreviewHalo)
	{
		m_pMatPreviewHalo->DecrementReferenceCount();
		m_pMatPreviewHalo->DeleteIfUnreferenced();
		m_pMatPreviewHalo = nullptr;
	}

	if (m_pMatPreviewBlurX)
	{
		m_pMatPreviewBlurX->DecrementReferenceCount();
		m_pMatPreviewBlurX->DeleteIfUnreferenced();
		m_pMatPreviewBlurX = nullptr;
	}

	if (m_pMatPreviewBlurY)
	{
		m_pMatPreviewBlurY->DecrementReferenceCount();
		m_pMatPreviewBlurY->DeleteIfUnreferenced();
		m_pMatPreviewBlurY = nullptr;
	}

	if (m_pPreviewBuf1)
	{
		m_pPreviewBuf1->DecrementReferenceCount();
		m_pPreviewBuf1->DeleteIfUnreferenced();
		m_pPreviewBuf1 = nullptr;
	}

	if (m_pPreviewBuf2)
	{
		m_pPreviewBuf2->DecrementReferenceCount();
		m_pPreviewBuf2->DeleteIfUnreferenced();
		m_pPreviewBuf2 = nullptr;
	}
}