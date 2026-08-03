#pragma once
#include "../../SDK/SDK.h"

class CMisc
{
private:
	void AutoJump(CTFPlayer* pLocal, CUserCmd* pCmd);
	void AutoJumpbug(CTFPlayer* pLocal, CUserCmd* pCmd);
	void AutoStrafe(CTFPlayer* pLocal, CUserCmd* pCmd);
	void MovementLock(CTFPlayer* pLocal, CUserCmd* pCmd);
	void BreakJump(CTFPlayer* pLocal, CUserCmd* pCmd);
	void AntiAFK(CTFPlayer* pLocal, CUserCmd* pCmd);
	void InstantRespawnMVM(CTFPlayer* pLocal);
	void NoisemakerSpam(CTFPlayer* pLocal);

	void CheatsBypass();
	void WeaponSway();

	void TauntKartControl(CTFPlayer* pLocal, CUserCmd* pCmd);
	void FastMovement(CTFPlayer* pLocal, CUserCmd* pCmd);

	void AutoPeek(CTFPlayer* pLocal, CUserCmd* pCmd, bool bPost = false);
	void EdgeJump(CTFPlayer* pLocal, CUserCmd* pCmd, bool bPost = false);

	// EdgeBug
	void EdgeBugPrePrediction(CTFPlayer* pLocal, CUserCmd* pCmd);
	bool EdgeBugCheck(CTFPlayer* pLocal, CUserCmd* pCmd);
	void RestoreEntityToPredicted();
	void CorrectMovement(CUserCmd* pCmd, Vec3 vWishAngle, Vec3 vOldAngles);
	void AutoStrafeEdgeBug(CUserCmd* pCmd, CTFPlayer* pLocal);

	void LongJump(CTFPlayer* pLocal, CUserCmd* pCmd);
	void MiniJump(CTFPlayer* pLocal, CUserCmd* pCmd);
	void AutoAlign(CTFPlayer* pLocal, CUserCmd* pCmd);
	void PixelSurf(CTFPlayer* pLocal, CUserCmd* pCmd);

	bool m_bPeekPlaced = false;
	Vec3 m_vPeekReturnPos = {};

	// EdgeBug data
	struct EdgeBugCmd_t {
		Vec3 viewangles;
		float forwardmove;
		float sidemove;
		int buttons;
		Vec3 origin;
	};

	Vec3 m_vEdgeBugVelocityBackup = {};
	Vec3 m_vEdgeBugPredictedVelocity = {};
	int m_iEdgeBugFlags = 0;
	bool m_bEdgeBugDetected = false;
	bool m_bEdgeBugDuck = false;
	int m_iEdgeBugLockTicks = 0;
	int m_iEdgeBugPredictTick = 0;
	int m_iEdgeBugCurrentTick = 0;
	int m_iEdgeBugSearchMode = 0; // 0-5: different search patterns
	EdgeBugCmd_t m_EdgeBugCmds[64] = {};

	// Advanced edgebug
	Vec3 m_vEdgeBugTargetAngles = {};
	Vec3 m_vEdgeBugSmoothAngles = {};
	Vec3 m_vEdgeBugOriginalAngles = {};
	float m_flEdgeBugOriginalForward = 0.f;
	float m_flEdgeBugOriginalSide = 0.f;
	int m_iEdgeBugPredictionTimestamp = 0;
	int m_iEdgeBugMouseOffset = 0;

	// LongJump data
	int m_iLongJumpSavedTick = 0;
	bool m_bLongJumpDetected = false;

	// MiniJump data
	bool m_bMiniJumpShouldDuck = false;
	bool m_bMiniJumpDetected = false;
	int m_iPrePredictionFlags = 0; // Flags before prediction for mini jump detection

	// AutoAlign data
	bool m_bWallDetected = false;
	float m_flAutoAlignStartCircle = 0.f;

	// PixelSurf data
	bool m_bShouldPixelSurf = false;
	int m_iPixelSurfTicks = 0;



public:
	void RunPre(CTFPlayer* pLocal, CUserCmd* pCmd);
	void RunPost(CTFPlayer* pLocal, CUserCmd* pCmd, bool pSendPacket);

	void Event(IGameEvent* pEvent, uint32_t uNameHash);
	int AntiBackstab(CTFPlayer* pLocal, CUserCmd* pCmd, bool bSendPacket);

	void PingReducer();
	void UnlockAchievements();
	void LockAchievements();

	void Draw(CTFPlayer* pLocal);
	void EdgeBugMouseLock(float& x, float& y);
	void EdgeBugPostPrediction(CTFPlayer* pLocal, CUserCmd* pCmd);

	int m_iWishCmdrate = -1;
	int m_iWishUpdaterate = -1;
};

ADD_FEATURE(CMisc, Misc);