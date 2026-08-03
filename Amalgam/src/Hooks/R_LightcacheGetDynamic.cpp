#include "../SDK/SDK.h"

// In the main menu there is no world/light cache, so the engine's dynamic light lookup
// dereferences null while setting up bones for our preview model and crashes. When not
struct LightingState_t
{
	Vec3  r_boxcolor[6];
	int   numlights;
	void* locallight[4];
};

MAKE_HOOK(R_LightcacheGetDynamic,
	U::Memory.FindSignature("engine.dll", "48 89 5C 24 ? 44 89 4C 24 ? 4C 89 44 24 ? 55 56 57 41 54 41 55 41 56 41 57 48 81 EC ? ? ? ? 48 8B 05"),
	void*, void* pThis, Vec3* vOrigin, LightingState_t* pLightState, void* a1, int a2, bool a3)
{
	if (I::EngineClient->IsInGame())
		return CALL_ORIGINAL(pThis, vOrigin, pLightState, a1, a2, a3);

	if (pLightState)
	{
		for (int i = 0; i < 6; i++)
		{
			pLightState->r_boxcolor[i] = Vec3(0.5f, 0.5f, 0.5f);
			if (i < 4)
				pLightState->locallight[i] = nullptr;
		}
		pLightState->numlights = 0;
	}
	return nullptr;
}
