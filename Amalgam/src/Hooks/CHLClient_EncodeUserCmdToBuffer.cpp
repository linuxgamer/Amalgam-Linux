#include "../SDK/SDK.h"

// Clean POV demos - usercmd path, INTENTIONALLY pass-through now.
MAKE_HOOK(CHLClient_EncodeUserCmdToBuffer, U::Memory.GetVirtual(I::Client, 24), void,
	void* rcx, bf_write& buf, int slot)
{
#ifdef DEBUG_HOOKS
	if (!Vars::Hooks::CHLClient_EncodeUserCmdToBuffer[DEFAULT_BIND])
		return CALL_ORIGINAL(rcx, buf, slot);
#endif

	CALL_ORIGINAL(rcx, buf, slot);
}
