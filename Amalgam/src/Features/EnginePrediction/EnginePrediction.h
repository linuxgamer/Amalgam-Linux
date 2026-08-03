#pragma once
#include "../../SDK/SDK.h"

struct DatamapRestore_t
{
	byte* m_pData = nullptr;
	size_t m_iSize = 0;
};

struct RestoreInfo_t
{
	Vec3 m_vOrigin = {};
	Vec3 m_vMins = {};
	Vec3 m_vMaxs = {};
};

class CEnginePrediction
{
private:

	CMoveData m_tMoveData = {};

	int m_nOldTickCount = 0;
	float m_flOldCurrentTime = 0.f;
	float m_flOldFrameTime = 0.f;

	DatamapRestore_t m_tLocal = {};

	// flat vector, not unordered_map: iterated whole, never looked up by key, and clear()
	// keeps capacity so the per-Simulate adjust is allocation- and hash-free
	std::vector<std::pair<CTFPlayer*, RestoreInfo_t>> m_vRestore = {};

	// the probe loops re-Simulate the same command dozens-to-hundreds of times per tick;
	// MD5 of the same command_number is pure waste after the first call
	int m_nSeedCommandNumber = -1;
	int m_iSeed = -1;

	// batch state: while a batch is open, other players are adjusted once (lazily, on the
	// first Simulate) and restored once at EndBatch instead of twice per player per Simulate
	int m_iBatchDepth = 0;
	bool m_bBatchAdjusted = false;

public:
	void Start(CTFPlayer* pLocal, CUserCmd* pCmd);
	void End(CTFPlayer* pLocal, CUserCmd* pCmd);
	void Simulate(CTFPlayer* pLocal, CUserCmd* pCmd);

	float GetTargetPredictZVelocity() const;
	bool  IsTargetPredictZVelocity(float flVz, float flEpsilon = 0.1f) const;

	void Unload();

	void AdjustPlayers(CBaseEntity* pLocal);
	void RestorePlayers();

	// Scope the player adjust/restore around a probe loop: other players don't move between
	// our own re-simulations of the same tick, so adjusting them per Simulate (SetAbsOrigin
	void BeginBatch() { m_iBatchDepth++; }
	void EndBatch()
	{
		if (--m_iBatchDepth <= 0)
		{
			m_iBatchDepth = 0;
			if (m_bBatchAdjusted)
			{
				RestorePlayers();
				m_bBatchAdjusted = false;
			}
		}
	}

	bool m_bInPrediction = false;

	// localplayer use in net_update_end
	Vec3 m_vOrigin = {};
	Vec3 m_vVelocity = {};
	Vec3 m_vDirection = {};
	Vec3 m_vAngles = {};
};

ADD_FEATURE(CEnginePrediction, EnginePrediction);

// RAII batch guard - the probe loops are full of early returns, so the restore must not
// depend on reaching the end of the function
struct EnginePredictionBatch_t
{
	EnginePredictionBatch_t() { F::EnginePrediction.BeginBatch(); }
	~EnginePredictionBatch_t() { F::EnginePrediction.EndBatch(); }

	EnginePredictionBatch_t(const EnginePredictionBatch_t&) = delete;
	EnginePredictionBatch_t& operator=(const EnginePredictionBatch_t&) = delete;
};