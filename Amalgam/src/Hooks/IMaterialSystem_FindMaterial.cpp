#include "../SDK/SDK.h"

// Redirects the CPrecipitation rain sprite ("particle/rain") to the configured snow sprite while
// snow precipitation is active, so the native rain becomes snow. Slot 71 = IMaterialSystem::FindMaterial.
MAKE_HOOK(IMaterialSystem_FindMaterial, U::Memory.GetVirtual(I::MaterialSystem, 71), IMaterial*,
	void* rcx, const char* pMaterialName, const char* pTextureGroupName, bool bComplain, const char* pComplainPrefix)
{
	if (!G::Unload && pMaterialName &&
		Vars::Visuals::Weather::Precipitation.Value == Vars::Visuals::Weather::PrecipitationEnum::Snow)
	{
		if (!strcmp(pMaterialName, "particle/rain") || !strcmp(pMaterialName, "particles/rain"))
		{
			const char* pSnow = Vars::Visuals::Weather::SnowSprite.Value.c_str();
			if (pSnow && pSnow[0])
				pMaterialName = pSnow;
		}
	}

	return CALL_ORIGINAL(rcx, pMaterialName, pTextureGroupName, bComplain, pComplainPrefix);
}
