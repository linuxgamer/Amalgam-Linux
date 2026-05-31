#include "../SDK/SDK.h"

#include "../Features/Visuals/Materials/Materials.h"

MAKE_HOOK(IMatSystemSurface_OnScreenSizeChanged, U::Memory.GetVirtual(I::MatSystemSurface, 111), void,
	void* rcx, int nOldWidth, int nOldHeight)
{
	DEBUG_RETURN(IMatSystemSurface_OnScreenSizeChanged, rcx, nOldWidth, nOldHeight);

        CALL_ORIGINAL(rcx, nOldWidth, nOldHeight);

        if (!G::Unload)
        {
                H::Fonts.Reload();
                F::Materials.ReloadMaterials();
        }
