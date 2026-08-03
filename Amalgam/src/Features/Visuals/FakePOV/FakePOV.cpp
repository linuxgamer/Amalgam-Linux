#include "FakePOV.h"

#include <algorithm>
#include <cmath>

// Normalize an angle to [-180, 180).
static inline float NormalizeDeg(float a)
{
	a = fmodf(a + 180.f, 360.f);
	if (a < 0.f)
		a += 360.f;
	return a - 180.f;
}

// Shortest-path angular approach (frame-rate independent via the already-baked smoothing factor k).
static inline float ApproachAngle(float flCur, float flTgt, float flK)
{
	return flCur + NormalizeDeg(flTgt - flCur) * flK;
}

// "Snap view on disable" (input thread, called from CHLClient_CreateMove AFTER the RealViewAngles stash).
void CFakePOV::CreateMove(CTFPlayer* pLocal)
{
	const bool bRecording = I::EngineClient && I::EngineClient->IsRecordingDemo() && !I::EngineClient->IsPlayingDemo();
	const bool bNowEnabled = Vars::Misc::Movement::FakePOV.Value;

	if (m_iSnapState == 1)
	{	// Hold frame: the moved aim has now been stashed into G::RealViewAngles, so retire the offset and
		// reset the smoothing so a later re-enable animates a fresh turn from the new forward.
		G::FakePOVOffset = Vec3();
		G::FakePOVActive = false;
		m_flYaw = m_flPitch = m_flSpin = m_flArrowAng = m_flArrowLen = 0.f;
		m_iSnapState = 0;
	}
	else if (Vars::Misc::Movement::FakePOVSnapView.Value && bRecording && m_bPrevEnabled && !bNowEnabled && G::FakePOVActive)
	{	// Falling edge while recording: snap real aim onto the current fake direction.
		const Vec3 vOff = G::FakePOVOffset; // full smoothed offset showing right now
		Vec3 vTarget = I::EngineClient->GetViewAngles();
		float flPitch = vTarget.x + vOff.x;
		vTarget.x = flPitch < -89.f ? -89.f : (flPitch > 89.f ? 89.f : flPitch);
		vTarget.y += vOff.y;
		while (vTarget.y > 180.f) vTarget.y -= 360.f;
		while (vTarget.y < -180.f) vTarget.y += 360.f;
		vTarget.z = 0.f;
		I::EngineClient->SetViewAngles(vTarget);

		// Hold the offset one more frame so the override stays (oldReal + offset) until the stash catches up.
		G::FakePOVOffset = vOff;
		G::FakePOVActive = true;
		m_iSnapState = 1;
	}

	m_bPrevEnabled = bNowEnabled;
}

void CFakePOV::Draw(CTFPlayer* pLocal)
{
	if (!pLocal)
		return;

	// While the snap state machine owns the offset (CreateMove on the input thread), don't run the smoothing
	// or republish here - it would clobber the held/dropped offset and re-introduce the ease-back swing.
	if (m_iSnapState != 0)
		return;

	// Per-frame dt (same source the indicators use). Clamp so a hitch/pause never snaps the turn.
	const float flDt = std::min(I::GlobalVars->frametime, 0.1f);

	const bool bPlaying = I::EngineClient && I::EngineClient->IsPlayingDemo();
	const bool bEnabled = Vars::Misc::Movement::FakePOV.Value && !bPlaying;
	const int iMode = Vars::Misc::Movement::FakePOVMode.Value;

	// Target offset (relative to the real view) + target arrow placement for the current mode.
	float flTPitch = 0.f, flTYaw = 0.f, flTArrowAng = 0.f, flTLen = 0.f;
	if (bEnabled)
	{
		flTLen = 1.f;
		switch (iMode)
		{
		case Vars::Misc::Movement::FakePOVModeEnum::Right:     flTYaw = -90.f; flTArrowAng = 90.f;  break;
		case Vars::Misc::Movement::FakePOVModeEnum::Left:      flTYaw =  90.f; flTArrowAng = 270.f; break;
		case Vars::Misc::Movement::FakePOVModeEnum::Up:        flTPitch = -89.f; flTArrowAng = 0.f;   break;
		case Vars::Misc::Movement::FakePOVModeEnum::Bottom:    flTPitch =  89.f; flTArrowAng = 180.f; break;
		case Vars::Misc::Movement::FakePOVModeEnum::Backwards: flTYaw = 180.f; flTArrowAng = 180.f; break;
		case Vars::Misc::Movement::FakePOVModeEnum::Spinning:
			// Free-running accumulator: the cam tracks it with a constant lag, so it rotates at the
			// real spin speed regardless of the smoothing factor. Arrow turns the opposite way (CCW).
			m_flSpin = NormalizeDeg(m_flSpin + Vars::Misc::Movement::FakePOVSpinSpeed.Value * flDt);
			flTYaw = m_flSpin;
			flTArrowAng = -m_flSpin;
			break;
		}
	}
	else
	{
		// Disabled / playing back: ease the cam back to forward AND swing the arrow back to "up"
		// (forward) while it retracts, so it visibly turns back - and, crucially, so the NEXT enable
		flTArrowAng = 0.f;
	}

	// Keep the spin accumulator synced to the live yaw whenever we aren't actively spinning, so a
	// later spin resumes seamlessly from the current position instead of snapping.
	if (!bEnabled || iMode != Vars::Misc::Movement::FakePOVModeEnum::Spinning)
		m_flSpin = m_flYaw;

	const float flK = 1.f - expf(-std::max(0.1f, Vars::Misc::Movement::FakePOVSmoothSpeed.Value) * flDt);
	m_flYaw      = ApproachAngle(m_flYaw, flTYaw, flK);
	m_flPitch    = m_flPitch + (flTPitch - m_flPitch) * flK;
	m_flArrowAng = ApproachAngle(m_flArrowAng, flTArrowAng, flK);
	m_flArrowLen = m_flArrowLen + (flTLen - m_flArrowLen) * flK;

	// Publish for the demo-angle hook (CPrediction::GetLocalViewAngles) + the CreateMove stash gate.
	G::FakePOVOffset = Vec3(m_flPitch, m_flYaw, 0.f);
	G::FakePOVActive = bEnabled || fabsf(m_flYaw) > 0.05f || fabsf(m_flPitch) > 0.05f;

	if (Vars::Misc::Movement::FakePOVArrow.Value && !bPlaying && m_flArrowLen > 0.01f)
		DrawArrow();
}

void CFakePOV::DrawArrow()
{
	const float flLen = std::clamp(m_flArrowLen, 0.f, 1.f);
	Color_t tCol = Vars::Misc::Movement::FakePOVArrowColor.Value;
	tCol.a = static_cast<byte>(tCol.a * flLen);
	if (!tCol.a)
		return;

	const float flCx = H::Draw.m_nScreenW * 0.5f;
	const float flCy = H::Draw.m_nScreenH * 0.5f;
	const float flSize = std::max(2.f, Vars::Misc::Movement::FakePOVArrowSize.Value);
	// Fixed radius (the set distance) - the arrow rides this circle and only the alpha eases in, so it
	// rotates around the center instead of spiraling out from the middle as it appears.
	const float flDist = std::max(0.f, Vars::Misc::Movement::FakePOVArrowDist.Value);

	// Screen-space direction: 0 deg = up, clockwise (screen Y points down).
	const float flRad = DEG2RAD(m_flArrowAng);
	const float flDx = sinf(flRad);
	const float flDy = -cosf(flRad);
	const float flPx = -flDy; // perpendicular (arrow's right)
	const float flPy = flDx;

	const float flBaseR = flDist;
	const float flTipR = flDist + flSize * 1.6f;
	const float flHalf = flSize * 0.75f;

	const float flTipX = flCx + flDx * flTipR;
	const float flTipY = flCy + flDy * flTipR;
	const float flBaseX = flCx + flDx * flBaseR;
	const float flBaseY = flCy + flDy * flBaseR;

	// Filled arrowhead.
	H::Draw.FillPolygon(
		{
			{ { flTipX, flTipY } },
			{ { flBaseX + flPx * flHalf, flBaseY + flPy * flHalf } },
			{ { flBaseX - flPx * flHalf, flBaseY - flPy * flHalf } }
		}, tCol);

	// Stem from a small inner gap out to the head (only once the arrow has slid far enough out).
	/*const float flInnerR = flSize * 0.9f;
	if (flBaseR > flInnerR + 1.f)
		H::Draw.Line(
			static_cast<int>(flCx + flDx * flInnerR), static_cast<int>(flCy + flDy * flInnerR),
			static_cast<int>(flBaseX), static_cast<int>(flBaseY), tCol);
	*/
			
}
