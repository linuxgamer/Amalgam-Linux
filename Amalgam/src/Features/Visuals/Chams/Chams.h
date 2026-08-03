#pragma once
#include "../../../SDK/SDK.h"

class CChams
{
private:
	void DrawModel(CBaseEntity* pEntity, Chams_t& tChams, IMatRenderContext* pRenderContext, bool bTwoModels = true);

	void RenderBacktrack(const DrawModelState_t& pState, const ModelRenderInfo_t& pInfo);
	void RenderFakeAngle(const DrawModelState_t& pState, const ModelRenderInfo_t& pInfo);

	struct ChamsInfo_t
	{
		CBaseEntity* m_pEntity;
		Chams_t m_tChams;
		int m_iFlags = 0;
	};
	std::vector<ChamsInfo_t> m_vEntities = {};

	Color_t m_tOriginalColor = {};
	float m_flOriginalBlend = 1.f;
	IMaterial* m_pOriginalMaterial = nullptr;
	OverrideType_t m_iOriginalOverride = OVERRIDE_NORMAL;

	int m_iFlags = false;

public:
	void Store(CTFPlayer* pLocal);
	void RenderMain();
	void RenderHandler(const DrawModelState_t& pState, const ModelRenderInfo_t& pInfo, matrix3x4* pBoneToWorld);

	bool RenderViewmodel(void* ecx, int flags, int* iReturn);
	bool RenderViewmodel(const DrawModelState_t& pState, const ModelRenderInfo_t& pInfo, matrix3x4* pBoneToWorld);

	// Model Preview: draw chams onto an arbitrary (out-of-entity-list) entity into the caller's
	// already-pushed render target/view. Visible-only (no stencil) to keep it simple - the preview
	// has nothing to occlude it. Mirrors the live overlay-on-base-model look.
	void DrawPreview(CBaseEntity* pEntity, Chams_t& tChams, IMatRenderContext* pRenderContext) { DrawModel(pEntity, tChams, pRenderContext, false); }

	bool m_bRendering = false;

	std::unordered_map<int, bool> m_mEntities = {};
};

ADD_FEATURE(CChams, Chams);