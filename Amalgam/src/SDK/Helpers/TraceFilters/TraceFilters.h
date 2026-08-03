#pragma once
#include "../../Definitions/Interfaces/IEngineTrace.h"

enum
{
	SKIP_CHECK,
	FORCE_PASS,
	FORCE_HIT
};

enum
{
	WEAPON_INCLUDE,
	WEAPON_EXCLUDE
};
enum
{
	PLAYER_DEFAULT,
	PLAYER_NONE,
	PLAYER_ALL
};
enum
{
	OBJECT_DEFAULT,
	OBJECT_NONE,
	OBJECT_ALL
};

class CTraceFilterHitscan : public ITraceFilter
{
public:
	bool ShouldHitEntity(IHandleEntity* pServerEntity, int nContentsMask) override;
	TraceType_t GetTraceType() const override;
	CBaseEntity* pSkip = nullptr;

	int iTeam = -1;
	std::vector<int> vWeapons = { TF_WEAPON_SNIPERRIFLE, TF_WEAPON_SNIPERRIFLE_CLASSIC, TF_WEAPON_SNIPERRIFLE_DECAP };
	int iType = FORCE_HIT;
	int iWeapon = WEAPON_EXCLUDE;
	bool bWeapon = false;
};

class CTraceFilterCollideable : public ITraceFilter
{
public:
	bool ShouldHitEntity(IHandleEntity* pServerEntity, int nContentsMask) override;
	TraceType_t GetTraceType() const override;
	CBaseEntity* pSkip = nullptr;

	int iTeam = -1;
	std::vector<int> vWeapons = { TF_WEAPON_CROSSBOW, TF_WEAPON_LUNCHBOX };
	int iType = FORCE_HIT;
	int iWeapon = WEAPON_INCLUDE;
	bool bWeapon = false;
	int iPlayer = PLAYER_DEFAULT;
	int iObject = OBJECT_ALL;
	bool bMisc = false;
};

class CTraceFilterWorldAndPropsOnly : public ITraceFilter
{
public:
	bool ShouldHitEntity(IHandleEntity* pServerEntity, int nContentsMask) override;
	TraceType_t GetTraceType() const override;
	CBaseEntity* pSkip = nullptr;

	int iTeam = -1;
};

// World + brush entities (func_brush, doors, trains, conveyors...) but NOT studio model props
// (prop_dynamic / prop_physics / prop_physics_multiplayer). Same as CTraceFilterWorldAndPropsOnly
// minus the model-prop classes. Used by texture bug / wall climb / air stuck so they ignore the
// models they can't work on (and don't waste bruteforce on them) while still engaging on real
// brush geometry.
class CTraceFilterWorldAndBrushOnly : public ITraceFilter
{
public:
	bool ShouldHitEntity(IHandleEntity* pServerEntity, int nContentsMask) override;
	TraceType_t GetTraceType() const override;
	CBaseEntity* pSkip = nullptr;

	int iTeam = -1;
};