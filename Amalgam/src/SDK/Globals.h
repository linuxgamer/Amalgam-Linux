#pragma once
#include "Definitions/Definitions.h"
#include "Definitions/Main/CUserCmd.h"
#include "../Utils/Signatures/Signatures.h"
#include "../Utils/Memory/Memory.h"

MAKE_SIGNATURE(RandomSeed, "client.dll", "0F B6 1D ? ? ? ? 89 9D", 0x0);

struct DrawLine_t
{
	std::pair<Vec3, Vec3> m_paOrigin;
	float m_flTime;
	Color_t m_tColor;
	bool m_bZBuffer = false;
};

struct DrawPath_t
{
	std::vector<Vec3> m_vPath;
	float m_flTime;
	Color_t m_tColor;
	int m_iStyle;
	bool m_bZBuffer = false;
};

struct DrawBox_t
{
	Vec3 m_vOrigin;
	Vec3 m_vMins;
	Vec3 m_vMaxs;
	Vec3 m_vAngles;
	float m_flTime;
	Color_t m_tColorEdge;
	Color_t m_tColorFace;
	bool m_bZBuffer = false;
};

struct DrawSphere_t
{
	Vec3 m_vOrigin;
	float m_flRadius;
	int m_nTheta;
	int m_nPhi;
	float m_flTime;
	Color_t m_tColorEdge;
	Color_t m_tColorFace;
	bool m_bZBuffer = false;
};

struct DrawSwept_t
{
	std::pair<Vec3, Vec3> m_paOrigin;
	Vec3 m_vMins;
	Vec3 m_vMaxs;
	Vec3 m_vAngles;
	float m_flTime;
	Color_t m_tColor;
	bool m_bZBuffer = false;
};

struct AimTarget_t
{
	int m_iEntIndex = 0;
	int m_iTickCount = 0;
	int m_iDuration = 32;
};

struct AimPoint_t
{
	Vec3 m_vOrigin = {};
	int m_iTickCount = 0;
	int m_iDuration = 32;
};

namespace G
{
	inline bool Unload = false;

	inline int Attacking = 0;
	inline bool Reloading = false;
	inline bool CanPrimaryAttack = false;
	inline bool CanSecondaryAttack = false;
	inline bool CanHeadshot = false;
	inline int Throwing = false;
	inline float Lerp = 0.015f;

	inline EWeaponType PrimaryWeaponType = {}, SecondaryWeaponType = {};

	inline CUserCmd* CurrentUserCmd = nullptr;
	inline CUserCmd* LastUserCmd = nullptr;
	inline CUserCmd OriginalCmd = {};

	inline AimTarget_t AimTarget = {};
	inline AimPoint_t AimPoint = {};

	inline bool SilentAngles = false;
	inline bool PSilentAngles = false;

	// Clean POV demos (primary fix): latest real engine view angle, stashed each CreateMove. The
	// CPrediction::GetLocalViewAngles hook hands this back while a demo records so the demo cmdinfo /
	// first-person camera follows the real view instead of the silent/backstab angle.
	inline Vec3 RealViewAngles = {};

	// Fake POV (Movement > FAKE POV): smoothed demo-camera angle offset (x = pitch, y = yaw), published
	// each frame by F::FakePOV and added on top of RealViewAngles by the CPrediction::GetLocalViewAngles
	// hook while a demo records. Active = the feature is on OR the offset is still easing back to rest.
	inline Vec3 FakePOVOffset = {};
	inline bool FakePOVActive = false;

	inline bool AntiAim = false;
	inline bool Choking = false;

	inline bool UpdatingAnims = false;
	inline bool FlipViewmodels = false;

	// Flip world (Visuals > World > Flip world): set true each command tick by CMisc::RunPre while the
	// feature is on and the local player is alive. Read by CFlipWorld::Render (the horizontal
	// screen-space mirror of the world frame, done in the DoPostScreenSpaceEffects pass) and by
	// CMisc::FlipWorldInput (mirrors the controls). Latched at command rate; the render reads it.
	inline bool FlipWorldActive = false;

	inline std::vector<DrawLine_t> LineStorage = {};
	inline std::vector<DrawPath_t> PathStorage = {};
	inline std::vector<DrawBox_t> BoxStorage = {};
	inline std::vector<DrawSphere_t> SphereStorage = {};
	inline std::vector<DrawSwept_t> SweptStorage = {};

	inline int* RandomSeed()
	{
		static auto pRandomSeed = reinterpret_cast<int*>(U::Memory.RelToAbs(S::RandomSeed()));
		return pRandomSeed;
	}
};