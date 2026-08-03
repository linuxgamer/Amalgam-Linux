#include "../SDK/SDK.h"

// Clean POV demos (primary fix - mirrors the get_local_view_angles_hook).
// The engine demo recorder fills each demo cmdinfo's local view angle from CPrediction::GetLocalViewAngles
MAKE_SIGNATURE(CPrediction_GetLocalViewAngles, "client.dll", "40 53 48 83 EC ? 48 8B DA E8 ? ? ? ? 48 8B C8 48 85 C0 75 ? 0F 57 C0", 0x0);

MAKE_HOOK(CPrediction_GetLocalViewAngles, S::CPrediction_GetLocalViewAngles(), void,
	void* rcx, Vec3* ang)
{
#ifdef DEBUG_HOOKS
	if (!Vars::Hooks::CPrediction_GetLocalViewAngles[DEFAULT_BIND])
		return CALL_ORIGINAL(rcx, ang);
#endif

	CALL_ORIGINAL(rcx, ang);

	if (!ang)
		return;
	// Clean POV demos cleans the silent snap; Fake POV (Movement > FAKE POV) adds a deliberate offset.
	// Either one wants to drive the recorded demo camera.
	if (!Vars::Misc::Game::CleanPOVDemos.Value && !G::FakePOVActive)
		return;
	if (!I::Prediction || I::Prediction->m_bInPrediction)
		return;
	if (!I::EngineClient || !I::EngineClient->IsRecordingDemo() || I::EngineClient->IsPlayingDemo())
		return;
	if (!H::Entities.GetLocal())
		return;

	Vec3 vAng = G::RealViewAngles;
	if (G::FakePOVActive)
	{	// Add the smoothed fake-POV offset (relative to the real view), clamping/normalizing the result.
		float flPitch = vAng.x + G::FakePOVOffset.x;
		vAng.x = flPitch < -89.f ? -89.f : (flPitch > 89.f ? 89.f : flPitch);
		vAng.y += G::FakePOVOffset.y;
		while (vAng.y > 180.f) vAng.y -= 360.f;
		while (vAng.y < -180.f) vAng.y += 360.f;
	}
	*ang = vAng;
}
