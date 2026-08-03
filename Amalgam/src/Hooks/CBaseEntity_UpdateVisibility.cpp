#include "../SDK/SDK.h"

#include "../Features/Visuals/ModelPreview/ModelPreview.h"

// On level load/unload the engine walks the leaf system and calls UpdateVisibility on
// every registered entity. Our hand-rolled preview entity is not safe to run through it,
// so short-circuit the call for just that one pointer.
MAKE_HOOK(CBaseEntity_UpdateVisibility,
	U::Memory.FindSignature("client.dll", "48 89 5C 24 ? 48 89 74 24 ? 57 48 83 EC ? 48 8B D9 48 8B 0D"),
	void*, void* pEnt)
{
	if (pEnt && pEnt == reinterpret_cast<void*>(F::ModelPreview.GetEntity()))
		return nullptr;

	return CALL_ORIGINAL(pEnt);
}
