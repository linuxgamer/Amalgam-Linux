#include "../SDK/SDK.h"

MAKE_SIGNATURE(CBaseAnimating_MaintainSequenceTransitions, "client.dll", "4C 89 4C 24 ? 41 56", 0x0);

MAKE_HOOK(CBaseAnimating_MaintainSequenceTransitions, S::CBaseAnimating_MaintainSequenceTransitions(), void,
	void* rcx, void* boneSetup, float flCycle, Vec3 pos[], Vector4D q[])
{
#ifdef DEBUG_HOOKS
	if (!Vars::Hooks::CBaseAnimating_MaintainSequenceTransitions[DEFAULT_BIND])
		return CALL_ORIGINAL(rcx, boneSetup, flCycle, pos, q);
#endif

	// Upstream Amalgam stubs this out entirely (empty body) so animation poses snap straight to the
	// current sequence with no blend - a hitbox/resolver technique. But MaintainSequenceTransitions
	// runs for EVERY animating entity (local player + viewmodel included) on every bone setup, so
	// killing it unconditionally makes all anims snap between each other ("choppy / not interpolating")
	// even with nothing enabled. Only drop the transition blend when interpolation removal is actually
	// requested; otherwise run the engine's vanilla sequence-transition blending so anims look
	// non-injected when idle. (The engine itself already early-returns this during prediction, so the
	// local player's predicted pose is unaffected either way.)
	if (Vars::Visuals::Removals::Interpolation.Value)
		return;

	CALL_ORIGINAL(rcx, boneSetup, flCycle, pos, q);
}