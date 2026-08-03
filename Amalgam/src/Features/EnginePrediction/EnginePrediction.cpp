#include "EnginePrediction.h"

#include "../Ticks/Ticks.h"

// account for interp and origin compression when simulating local player
void CEnginePrediction::AdjustPlayers(CBaseEntity* pLocal)
{
	auto& vGroup = H::Entities.GetGroup(EntityEnum::PlayerAll);

	m_vRestore.clear();
	m_vRestore.reserve(vGroup.size());

	for (auto pEntity : vGroup)
	{
		auto pPlayer = pEntity->As<CTFPlayer>();
		if (pPlayer == pLocal || pPlayer->IsDormant() || !pPlayer->IsAlive() || pPlayer->IsAGhost())			continue;

		m_vRestore.emplace_back(pPlayer, RestoreInfo_t{ pPlayer->GetAbsOrigin(), pPlayer->m_vecMins(), pPlayer->m_vecMaxs() });

		pPlayer->SetAbsOrigin(pPlayer->m_vecOrigin());
		pPlayer->m_vecMins() += 0.125f;
		pPlayer->m_vecMaxs() -= 0.125f;
	}
}
void CEnginePrediction::RestorePlayers()
{
	for (auto& [pPlayer, tRestore] : m_vRestore)
	{
		pPlayer->SetAbsOrigin(tRestore.m_vOrigin);
		pPlayer->m_vecMins() = tRestore.m_vMins;
		pPlayer->m_vecMaxs() = tRestore.m_vMaxs;
	}
	m_vRestore.clear();
}

void CEnginePrediction::Simulate(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	const int nOldTickBase = pLocal->m_nTickBase();
	const bool bOldIsFirstPrediction = I::Prediction->m_bFirstTimePredicted;
	const bool bOldInPrediction = I::Prediction->m_bInPrediction;

	I::MoveHelper->SetHost(pLocal);
	pLocal->m_pCurrentCommand() = pCmd;
	if (pCmd->command_number != m_nSeedCommandNumber)
	{
		m_nSeedCommandNumber = pCmd->command_number;
		m_iSeed = MD5_PseudoRandom(pCmd->command_number) & std::numeric_limits<int>::max();
	}
	*G::RandomSeed() = m_iSeed;

	I::Prediction->m_bFirstTimePredicted = false;
	I::Prediction->m_bInPrediction = true;
	I::Prediction->SetLocalViewAngles(pCmd->viewangles);

	const bool bBatched = m_iBatchDepth > 0;
	if (!bBatched || !m_bBatchAdjusted)
	{
		AdjustPlayers(pLocal);
		m_bBatchAdjusted = bBatched;
	}
	I::Prediction->SetupMove(pLocal, pCmd, I::MoveHelper, &m_tMoveData);
	I::GameMovement->ProcessMovement(pLocal, &m_tMoveData);
	I::Prediction->FinishMove(pLocal, pCmd, &m_tMoveData);
	if (!bBatched)
		RestorePlayers();

	I::MoveHelper->SetHost(nullptr);
	pLocal->m_pCurrentCommand() = nullptr;
	*G::RandomSeed() = -1;

	pLocal->m_nTickBase() = nOldTickBase;
	I::Prediction->m_bFirstTimePredicted = bOldIsFirstPrediction;
	I::Prediction->m_bInPrediction = bOldInPrediction;

	//if (static int nLastTickBase = 0; nLastTickBase != nOldTickBase)
    //F::MoveSim.StorePlayer(pLocal, m_tMoveData, TICKS_TO_TIME(nOldTickBase)), nLastTickBase = nOldTickBase;

	m_vOrigin = m_tMoveData.m_vecAbsOrigin;
	m_vVelocity = m_tMoveData.m_vecVelocity;
	m_vDirection = { m_tMoveData.m_flForwardMove, -m_tMoveData.m_flSideMove, m_tMoveData.m_flUpMove };
	m_vAngles = m_tMoveData.m_vecViewAngles;
}



void CEnginePrediction::Start(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	m_bInPrediction = true;
	if (!pLocal->IsAlive())
		return;

	auto pMap = pLocal->GetPredDescMap();
	if (!pMap)
		return;

	m_nOldTickCount = I::GlobalVars->tickcount;
	m_flOldCurrentTime = I::GlobalVars->curtime;
	m_flOldFrameTime = I::GlobalVars->frametime;

	I::GlobalVars->tickcount = pLocal->m_nTickBase();
	I::GlobalVars->curtime = TICKS_TO_TIME(I::GlobalVars->tickcount);
	I::GlobalVars->frametime = I::Prediction->m_bEnginePaused ? 0.f : TICK_INTERVAL;

	size_t iSize = pLocal->GetIntermediateDataSize();
	if (!m_tLocal.m_pData) 
	{
		m_tLocal.m_pData = reinterpret_cast<byte*>(I::MemAlloc->Alloc(iSize));
		m_tLocal.m_iSize = iSize;
	}
	else if (m_tLocal.m_iSize != iSize)
	{
		m_tLocal.m_pData = reinterpret_cast<byte*>(I::MemAlloc->Realloc(m_tLocal.m_pData, iSize));
		m_tLocal.m_iSize = iSize;
	}

	CPredictionCopy copy = { PC_EVERYTHING, m_tLocal.m_pData, PC_DATA_PACKED, pLocal, PC_DATA_NORMAL };
	copy.TransferData("EnginePredictionStart", pLocal->entindex(), pMap);

	Simulate(pLocal, pCmd);
}

void CEnginePrediction::End(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	m_bInPrediction = false;
	if (!pLocal->IsAlive())
		return;

	auto pMap = pLocal->GetPredDescMap();
	if (!pMap)
		return;

	I::GlobalVars->tickcount = m_nOldTickCount;
	I::GlobalVars->curtime = m_flOldCurrentTime;
	I::GlobalVars->frametime = m_flOldFrameTime;

	CPredictionCopy copy = { PC_EVERYTHING, pLocal, PC_DATA_NORMAL, m_tLocal.m_pData, PC_DATA_PACKED };
	copy.TransferData("EnginePredictionEnd", pLocal->entindex(), pMap);
}

float CEnginePrediction::GetTargetPredictZVelocity() const
{
	// Interval-derived one-tick "landed on a surface" velocity, matching the earliest WORKING
	// TextureBug (bak_20260602_110801): -(sv_gravity * interval_per_tick * 0.5) = -6.0 on TF2.
	static auto sv_gravity = H::ConVars.FindVar("sv_gravity");
	const float flGravity = sv_gravity ? sv_gravity->GetFloat() : 800.f;
	return -(flGravity * I::GlobalVars->interval_per_tick * 0.5f);
}

bool CEnginePrediction::IsTargetPredictZVelocity(float flVz, float flEpsilon) const
{
	return fabsf(flVz - GetTargetPredictZVelocity()) <= flEpsilon;
}

void CEnginePrediction::Unload()
{
	if (m_tLocal.m_pData)
	{
		I::MemAlloc->Free(m_tLocal.m_pData);
		m_tLocal = {};
	}
}