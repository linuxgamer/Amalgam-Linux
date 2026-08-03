#pragma once
#include "../../../SDK/SDK.h"

#include "../AimbotGlobal/AimbotGlobal.h"

class CAimbotMelee
{
private:
	std::vector<Target_t> GetTargets(CTFPlayer* pLocal, CTFWeaponBase* pWeapon);
	bool AimFriendlyBuilding(CBaseObject* pBuilding);
	std::vector<Target_t> SortTargets(CTFPlayer* pLocal, CTFWeaponBase* pWeapon);

	// Knife backstab overrides the general aim type / FOV / autoshoot so it can run on its own
	// settings (and even when the general aimbot type is Off). Non-knife melee keeps using General.
	bool IsKnifeBackstab(CTFWeaponBase* pWeapon);
	int GetBackstabAimType(); // KNIFE dropdown index -> General aim type
	int GetAimType(CTFWeaponBase* pWeapon);
	float GetFOV(CTFWeaponBase* pWeapon);
	bool GetAutoShoot(CTFWeaponBase* pWeapon);

	int GetSwingTime(CTFWeaponBase* pWeapon, bool bVar = true);
	void UpdateInfo(CTFPlayer* pLocal, CTFWeaponBase* pWeapon, CUserCmd* pCmd, std::vector<Target_t> vTargets);
	// Predicts the eye position the server's swing trace will actually start from for THIS command:
	// the local player moved one tick (ProcessMovement runs before the knife's PrimaryAttack swing
	Vec3 PredictSwingEye(CTFPlayer* pLocal, Vec3 vEyeAngles);
	bool CanBackstab(CBaseEntity* pTarget, CTFPlayer* pLocal, Vec3 vEyeAngles);
	int CanHit(Target_t& tTarget, CTFPlayer* pLocal, CTFWeaponBase* pWeapon);
	
	bool Aim(Vec3 vCurAngle, Vec3 vToAngle, Vec3& vOut, int iMethod = Vars::Aimbot::General::AimType.Value);
	void Aim(CUserCmd* pCmd, Vec3& vAngle, int iMethod = Vars::Aimbot::General::AimType.Value);

	bool FindNearestBuildPoint(CBaseObject* pBuilding, CTFPlayer* pLocal, Vec3& vPoint);
	bool RunSapper(CTFPlayer* pLocal, CTFWeaponBase* pWeapon, CUserCmd* pCmd);

	Vec3 m_vEyePos = {};
	// The un-predicted eye position from UpdateInfo. CanHit can advance m_vEyePos one move-tick per
	// target (BackstabPredictMovement); restoring from this each target stops one target's prediction
	Vec3 m_vEyePosBase = {};
	float m_flRange = 0.f;
	bool m_bShouldSwing = false;
	bool m_bBackstabLegit = false; // Legit backstab does no view rotation, only auto-swings
	// Set when CanHit has already baked the predicted post-move eye into m_vEyePos for this swing, so
	// CanBackstab skips its own (now-redundant) per-record re-predict. See BackstabPredictMovement.
	bool m_bSwingEyePredicted = false;

	// Perf: CanBackstab's dot-only eye re-predict (the BackstabPredictMovement-OFF path) runs a full
	// engine-prediction roundtrip, and CanBackstab is called per aim-point candidate (+ overlap +
	Vec3 m_vDotEyeCache = {};
	bool m_bDotEyeCached = false;
	// Local-player sim record set emptiness, read once per CanHit instead of hash-looking-up (and
	// inserting into) m_mRecordMap repeatedly across CanHit + every CanBackstab candidate call.
	bool m_bLocalSimEmpty = true;

	// Live (real-time) target snapshot appended to the backtrack record set under BackstabUnstableConnection
	TickRecord m_tLiveRecord = {};

	int m_iDoubletapTicks = 0;

	std::unordered_map<int, std::deque<TickRecord>> m_mRecordMap;
	std::unordered_map<int, std::vector<Vec3>> m_mPaths;

public:
	void Run(CTFPlayer* pLocal, CTFWeaponBase* pWeapon, CUserCmd* pCmd);
};

ADD_FEATURE(CAimbotMelee, AimbotMelee);