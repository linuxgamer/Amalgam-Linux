#pragma once
#include "../../../SDK/SDK.h"

// "Fake POV" (Movement > FAKE POV). Bakes a fake camera angle into a recorded POV demo through the
// Clean POV demo path (CPrediction::GetLocalViewAngles) WITHOUT touching live aim - the offset is only
class CFakePOV
{
public:
	void Draw(CTFPlayer* pLocal);       // advance smoothing, publish the offset, draw the indicator arrow
	void CreateMove(CTFPlayer* pLocal); // "Snap view on disable": runs on the input thread (see FakePOV.cpp)

private:
	float m_flYaw = 0.f;       // smoothed yaw offset applied to the demo cam (deg, relative to real view)
	float m_flPitch = 0.f;     // smoothed pitch offset (deg)
	float m_flSpin = 0.f;      // free-running spin accumulator (deg, normalized) for Spinning mode
	float m_flArrowAng = 0.f;  // smoothed arrow screen angle (deg, 0 = up, clockwise)
	float m_flArrowLen = 0.f;  // smoothed arrow presence 0..1 (0 = hidden / retracted to center)

	// "Snap view on disable" state machine (driven from CreateMove on the input thread):
	// 0 = idle, 1 = hold frame (real aim just moved; keep the offset one more frame so the recorded
	int m_iSnapState = 0;
	bool m_bPrevEnabled = false; // FakePOV toggle state last CreateMove, for falling-edge detection

	void DrawArrow();
};

ADD_FEATURE(CFakePOV, FakePOV);
