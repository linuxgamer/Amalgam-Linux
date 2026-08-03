#include "../SDK/SDK.h"

MAKE_SIGNATURE(CTFRevolver_GetWeaponCrosshairScale, "client.dll", "48 89 5C 24 ? 48 89 74 24 ? 57 48 83 EC ? 48 8B F2 48 8B F9 E8 ? ? ? ? 48 8B D8 48 85 C0 0F 84", 0x0);

MAKE_HOOK(CTFRevolver_GetWeaponCrosshairScale, S::CTFRevolver_GetWeaponCrosshairScale(), void,
	void* rcx, float& flScale)
{
	// skip headshot-charge crosshair scaling -> amby xhair stays fixed size
	if (Vars::Misc::Game::NoAmbyCrosshairResize.Value)
		return;

	return CALL_ORIGINAL(rcx, flScale);
}
