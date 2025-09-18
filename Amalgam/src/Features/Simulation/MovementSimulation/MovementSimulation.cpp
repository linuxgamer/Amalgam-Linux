#include "MovementSimulation.h"
#include "YawAccumulator.h"

#include "../../EnginePrediction/EnginePrediction.h"
#include <numeric>
#include <sstream>
#include <cmath>
#include <vector>

static CUserCmd s_tDummyCmd = {};

void CMovementSimulation::ComputeYawResidualAndConfidence(const std::deque<MoveData>& recs, int usedTicks, float estYawPerTick, float& outResidualRMS, float& outConfidence) const
{
	outResidualRMS = 0.f; outConfidence = 0.f;
	if (recs.size() < 3 || usedTicks <= 0) return;
	const float yaw0 = Math::VectorAngles(recs[1].m_vDirection).y;
	int tickAccum = 0; int pairs = 0; float sumSq = 0.f;
	for (size_t i = 1; i < recs.size(); ++i)
	{
		if (recs[i-1].m_iMode != recs[i].m_iMode) continue;
		const int dt = std::max(TIME_TO_TICKS(recs[i-1].m_flSimTime - recs[i].m_flSimTime), 1);
		tickAccum += dt; if (tickAccum > usedTicks) break;
		const float yawObs = Math::VectorAngles(recs[i].m_vDirection).y;
	float yawPred = Math::NormalizeAngle(yaw0 + estYawPerTick * tickAccum);
	float diff = Math::NormalizeAngle(yawObs - yawPred);
		sumSq += diff * diff; pairs++;
	}
	if (pairs > 0)
	{
		outResidualRMS = sqrtf(sumSq / pairs);
		outConfidence = std::clamp(1.f - (outResidualRMS / 15.f), 0.f, 1.f);
	}
}

int CMovementSimulation::ComputeStabilityScore(const std::deque<MoveData>& recs, int window) const
{
	if ((int)recs.size() < window + 1) return 0;
	window = std::min(window, (int)recs.size() - 1);
	float lastYawDelta = 0.f; float jerkSum = 0.f;
	std::vector<float> speeds; speeds.reserve(window);
	for (int i = 1; i <= window; ++i)
	{
		const float yaw1 = Math::VectorAngles(recs[i-1].m_vDirection).y;
		const float yaw2 = Math::VectorAngles(recs[i].m_vDirection).y;
		float dyaw = Math::NormalizeAngle(yaw1 - yaw2);
		float jerk = fabsf(dyaw - lastYawDelta);
		jerkSum += jerk; lastYawDelta = dyaw;
		speeds.push_back(recs[i-1].m_vVelocity.Length2D());
	}

	float mean = std::accumulate(speeds.begin(), speeds.end(), 0.f) / speeds.size();
	float var = 0.f; for (float s : speeds) { float d = s - mean; var += d * d; }
	var /= std::max<size_t>(1, speeds.size());
	int score = (int)std::round(jerkSum * 0.25f + var * 0.002f);
	return std::max(0, score);
}

float CMovementSimulation::PredictAirYawPerTick(const PlayerStorage& tStorage, float avgYaw) const
{
	const auto& md = tStorage.m_MoveData;
	// read convars
	static auto sv_airaccelerate = U::ConVars.FindVar("sv_airaccelerate");
	const float airaccel = std::max(sv_airaccelerate ? sv_airaccelerate->GetFloat() : 10.f, 1.f);
	const float dt = TICK_INTERVAL;
	const float maxspeed = std::max(md.m_flMaxSpeed, 1.f);

	// wishdir: orthogonal steer relative to avgYaw sign
	float viewYaw = md.m_vecViewAngles.y;
	float steerYaw = viewYaw + (avgYaw >= 0.f ? 90.f : -90.f);
	Vec3 wishdir{}; Math::AngleVectors({0.f, steerYaw, 0.f}, &wishdir);
	wishdir.z = 0.f; wishdir = wishdir.Normalized();

	Vec3 vel = md.m_vecVelocity; vel.z = 0.f; float speed = vel.Length();
	if (speed < 1.f) return 0.f;

	float proj = vel.Dot(wishdir);
	float add = airaccel * maxspeed * dt;
	if (proj + add > maxspeed)
		add = std::max(0.f, maxspeed - proj);
	Vec3 newVel = vel + wishdir * add;
	float yawOld = Math::VectorAngles(vel).y;
	float yawNew = Math::VectorAngles(newVel).y;
	float yawDelta = Math::NormalizeAngle(yawNew - yawOld);
	return std::clamp(yawDelta, -6.f, 6.f);
}

float CMovementSimulation::GetGroundTurnScale(const PlayerStorage& tStorage, float avgYaw) const
{
	const float k = Vars::Aimbot::Projectile::GroundTurnScaleK.Value;
	if (k <= 0.f) return 1.f;
	float v = tStorage.m_MoveData.m_vecVelocity.Length2D();
	float scale = 1.f / (1.f + k * v * v);
	return std::clamp(scale, 0.25f, 1.f);
}

// kasa fit
float CMovementSimulation::EstimateCurvatureYawPerTick(const std::deque<MoveData>& recs, int maxSamples, int& outUsedTicks) const
{
	outUsedTicks = 0;
	if ((int)recs.size() < 3) return 0.f;

	// collect a contiguous window of points from the same movement mode starting at the newest sample
	const int targetMode = recs[0].m_iMode;
	std::vector<Vec3> pts; pts.reserve(std::min(maxSamples, (int)recs.size()));
	std::vector<int> idx; idx.reserve(pts.capacity());
	for (int i = 0; i < (int)recs.size() && (int)pts.size() < maxSamples; ++i)
	{
		if (recs[i].m_iMode != targetMode) break;
		pts.push_back(recs[i].m_vOrigin);
		idx.push_back(i);
	}
	if ((int)pts.size() < 3) return 0.f;

	// compute total ticks and distance across the selected contiguous window
	float totalTicks = 0.f; float dist = 0.f;
	for (size_t k = 1; k < idx.size(); ++k)
	{
		int iPrev = idx[k - 1];
		int iCur = idx[k];
		int dt = std::max(TIME_TO_TICKS(recs[iPrev].m_flSimTime - recs[iCur].m_flSimTime), 1);
		totalTicks += dt;
		dist += (pts[k - 1] - pts[k]).Length2D();
	}
	if (totalTicks <= 0.f || dist <= 0.f) return 0.f;
	outUsedTicks = (int)totalTicks;

	double mx = 0.0, my = 0.0;
	for (const auto& p : pts) { mx += p.x; my += p.y; }
	mx /= (double)pts.size(); my /= (double)pts.size();

	double Suu = 0.0, Suv = 0.0, Svv = 0.0, Suuu = 0.0, Suvv = 0.0, Svvv = 0.0, Svuu = 0.0;
	for (const auto& p : pts)
	{
		double u = (double)p.x - mx; double v = (double)p.y - my;
		double uu = u * u, vv = v * v;
		Suu += uu; Svv += vv; Suv += u * v;
		Suuu += uu * u; Svvv += vv * v; Suvv += u * vv; Svuu += v * uu;
	}

	double det = 2.0 * (Suu * Svv - Suv * Suv);
	if (fabs(det) < 1e-6) return 0.f;

	double uc = (Svv * (Suuu + Suvv) - Suv * (Svvv + Svuu)) / det;
	double vc = (Suu * (Svvv + Svuu) - Suv * (Suuu + Suvv)) / det;
	double R2 = uc * uc + vc * vc + (Suu + Svv) / (double)pts.size();
	if (!(R2 > 0.0 && std::isfinite(R2))) return 0.f;
	float R = sqrtf((float)R2);
	if (R < 1.f) return 0.f;

	// ggregate cross products over segments (newest -> older order)
	double crossSum = 0.0;
	for (size_t k = 0; k + 2 < pts.size(); ++k)
	{
		Vec3 b = pts[k + 1] - pts[k];
		Vec3 a = pts[k + 2] - pts[k + 1];
		crossSum += (double)(a.x * b.y - a.y * b.x);
	}
	float signDir = crossSum >= 0.0 ? 1.f : -1.f;

	// mean speed over the window and corresponding yaw rate
	float v = dist / (totalTicks * TICK_INTERVAL);
	constexpr float kPi = 3.14159265358979323846f;
	float yawPerSec = (v / R) * signDir * 180.f / kPi; // deg/sec
	float yawPerTick = yawPerSec * TICK_INTERVAL;
	return std::clamp(yawPerTick, -10.f, 10.f);
}

void CMovementSimulation::GetAverageYaw(PlayerStorage& tStorage, int iSamples)
{
	auto pPlayer = tStorage.m_pPlayer;
	auto& vRecords = m_mRecords[pPlayer->entindex()];
	if (vRecords.empty()) return;

	bool bGroundInitial = tStorage.m_bDirectMove;
	float flMaxSpeed = SDK::MaxSpeed(tStorage.m_pPlayer, false, true);
	float flLowMinDist = bGroundInitial ? Vars::Aimbot::Projectile::GroundLowMinimumDistance.Value : Vars::Aimbot::Projectile::AirLowMinimumDistance.Value;
	float flLowMinSamples = bGroundInitial ? Vars::Aimbot::Projectile::GroundLowMinimumSamples.Value : Vars::Aimbot::Projectile::AirLowMinimumSamples.Value;
	float flHighMinDist = bGroundInitial ? Vars::Aimbot::Projectile::GroundHighMinimumDistance.Value : Vars::Aimbot::Projectile::AirHighMinimumDistance.Value;
	float flHighMinSamples = bGroundInitial ? Vars::Aimbot::Projectile::GroundHighMinimumSamples.Value : Vars::Aimbot::Projectile::AirHighMinimumSamples.Value;

	iSamples = std::min(iSamples, int(vRecords.size()));
	if (iSamples < 2) return;

	int modeSkips = 0;
	YawAccumulator acc;
	bool accConfigured = false;
	size_t i = 1; for (; i < (size_t)iSamples; ++i)
	{
		auto& newer = vRecords[i - 1];
		auto& older = vRecords[i];
		if (newer.m_iMode != older.m_iMode) { modeSkips++; continue; }
	// static inline bool GetYawDifference(MoveData& tRecord1, MoveData& tRecord2, bool bStart, float* pYaw, float flStraightFuzzyValue, int iMaxChanges = 0, int iMaxChangeTime = 0, float flMaxSpeed = 0.f)
		bool bGround = newer.m_iMode != 1;
		float straightFuzzy = bGround ? Vars::Aimbot::Projectile::GroundStraightFuzzyValue.Value : Vars::Aimbot::Projectile::AirStraightFuzzyValue.Value;
		int maxChanges = bGround ? Vars::Aimbot::Projectile::GroundMaxChanges.Value : Vars::Aimbot::Projectile::AirMaxChanges.Value;
		int maxChangeTime = bGround ? Vars::Aimbot::Projectile::GroundMaxChangeTime.Value : Vars::Aimbot::Projectile::AirMaxChangeTime.Value;
		if (!accConfigured)
		{
			acc.Begin(straightFuzzy, maxChanges, maxChangeTime, flMaxSpeed);
			accConfigured = true;
		}
		if (!acc.Step(newer, older))
			break;
	}

	int minStrafes = 4 + (int)(Vars::Aimbot::Projectile::GroundMaxChanges.Value); // approximate legacy req
	if (i <= size_t(minStrafes + modeSkips)) return; // insufficient valid samples

	int dynamicMin = flLowMinSamples;
	if (pPlayer->entindex() != I::EngineClient->GetLocalPlayer())
	{
		float flDistance = 0.f;
		if (auto pLocal = H::Entities.GetLocal())
			flDistance = pLocal->m_vecOrigin().DistTo(tStorage.m_pPlayer->m_vecOrigin());
		dynamicMin = flDistance < flLowMinDist ? flLowMinSamples : (int)Math::RemapVal(flDistance, flLowMinDist, flHighMinDist, flLowMinSamples + 1, flHighMinSamples);
	}

	int stabilityScore = ComputeStabilityScore(vRecords, std::min(iSamples, 12));
	tStorage.m_flStability = (float)stabilityScore;
	if (Vars::Aimbot::Projectile::UseStabilityMinSamples.Value)
	{
		int stability = stabilityScore;
		dynamicMin = std::min(iSamples, dynamicMin + std::min(stability, 8));
	}

	float avgYaw = acc.Finalize(dynamicMin, dynamicMin); // we pass same value for minTicks & dynamicMin for simplicity
	if (!avgYaw) return;

	// if curvature fit is enabled, estimate and pick lower residual
	float chosenYaw = avgYaw; float conf = 0.f; float residual = 0.f;
	ComputeYawResidualAndConfidence(vRecords, dynamicMin, avgYaw, residual, conf);
	float bestResidual = residual; float bestConf = conf;
	if (Vars::Aimbot::Projectile::UseCurvatureFit.Value)
	{
		int usedTicksCurv = 0;
		float curvYaw = EstimateCurvatureYawPerTick(vRecords, iSamples, usedTicksCurv);
		if (curvYaw != 0.f && usedTicksCurv > 0)
		{
			float r2=0.f, c2=0.f; ComputeYawResidualAndConfidence(vRecords, usedTicksCurv, curvYaw, r2, c2);
			if (r2 < bestResidual)
			{
				chosenYaw = curvYaw; bestResidual = r2; bestConf = c2;
			}
		}
	}

	tStorage.m_flAverageYaw = chosenYaw;
	tStorage.m_flAverageYawConfidence = bestConf;
	{
		std::ostringstream oss; oss << "flAverageYaw(det) " << chosenYaw << " ticks=" << acc.AccumulatedTicks() << " min=" << dynamicMin << (pPlayer->entindex() == I::EngineClient->GetLocalPlayer() ? " (local)" : "");
		SDK::Output("MovementSimulation", oss.str().c_str(), { 100, 200, 150 }, Vars::Debug::Logging.Value);
	}
}

void CMovementSimulation::Store()
{
	for (auto pEntity : H::Entities.GetGroup(EGroupType::PLAYERS_ALL))
	{
		auto pPlayer = pEntity->As<CTFPlayer>();
		auto& vRecords = m_mRecords[pPlayer->entindex()];

		if (!pPlayer->IsAlive() || pPlayer->IsAGhost() || pPlayer->m_vecVelocity().IsZero())
		{
			vRecords.clear();
			continue;
		}
		else if (!H::Entities.GetDeltaTime(pPlayer->entindex()))
			continue;

		bool bLocal = pPlayer->entindex() == I::EngineClient->GetLocalPlayer() && !I::EngineClient->IsPlayingDemo();
		Vec3 vVelocity = bLocal ? F::EnginePrediction.m_vVelocity : pPlayer->m_vecVelocity();
		Vec3 vOrigin = bLocal ? F::EnginePrediction.m_vOrigin : pPlayer->m_vecOrigin();
		Vec3 vDirection = bLocal ? Math::RotatePoint(F::EnginePrediction.m_vDirection, {}, { 0, F::EnginePrediction.m_vAngles.y, 0 }) : vVelocity.To2D();

		MoveData* pLastRecord = !vRecords.empty() ? &vRecords.front() : nullptr;
		vRecords.emplace_front(
			vDirection,
			pPlayer->m_flSimulationTime(),
			pPlayer->IsSwimming() ? 2 : pPlayer->IsOnGround() ? 0 : 1,
			vVelocity,
			vOrigin
		);
		MoveData& tCurRecord = vRecords.front();
		if (vRecords.size() > 66)
			vRecords.pop_back();

		float flMaxSpeed = SDK::MaxSpeed(pPlayer);
		if (pLastRecord)
		{
			/*
			if (tRecord.m_iMode != pLastRecord->m_iMode)
				vRecords.clear();
			else // does this eat up fps? i can't tell currently
			*/
			{
				CGameTrace trace = {};
				CTraceFilterWorldAndPropsOnly filter = {};
				SDK::TraceHull(pLastRecord->m_vOrigin, pLastRecord->m_vOrigin + pLastRecord->m_vVelocity * TICK_INTERVAL, pPlayer->m_vecMins() + 0.125f, pPlayer->m_vecMaxs() - 0.125f, pPlayer->SolidMask(), &filter, &trace);
				if (trace.DidHit() && trace.plane.normal.z < 0.707f)
					vRecords.clear();
			}
		}
		if (pPlayer->InCond(TF_COND_SHIELD_CHARGE))
		{
			s_tDummyCmd.forwardmove = 450.f;
			s_tDummyCmd.sidemove = 0.f;
			SDK::FixMovement(&s_tDummyCmd, bLocal ? F::EnginePrediction.m_vAngles : pPlayer->GetEyeAngles(), {});
			tCurRecord.m_vDirection.x = s_tDummyCmd.forwardmove;
			tCurRecord.m_vDirection.y = -s_tDummyCmd.sidemove;
		}
		else
		{
			switch (tCurRecord.m_iMode)
			{
			case 0:
				if (bLocal && Vars::Misc::Movement::Bunnyhop.Value && G::OriginalCmd.buttons & IN_JUMP)
					tCurRecord.m_vDirection = vVelocity.Normalized2D() * flMaxSpeed;
				break;
			case 1:
				tCurRecord.m_vDirection = vVelocity.Normalized2D() * flMaxSpeed;
				break;
			case 2:
				tCurRecord.m_vDirection *= 2;
			}
		}
	}

	for (auto pEntity : H::Entities.GetGroup(EGroupType::PLAYERS_ALL))
	{
		auto pPlayer = pEntity->As<CTFPlayer>();
		auto& vSimTimes = m_mSimTimes[pPlayer->entindex()];

		if (pEntity->entindex() == I::EngineClient->GetLocalPlayer() || !pPlayer->IsAlive() || pPlayer->IsAGhost())
		{
			vSimTimes.clear();
			continue;
		}

		float flDeltaTime = H::Entities.GetDeltaTime(pPlayer->entindex());
		if (!flDeltaTime)
			continue;

		vSimTimes.push_front(flDeltaTime);
		if (vSimTimes.size() > Vars::Aimbot::Projectile::DeltaCount.Value)
			vSimTimes.pop_back();
	}
}



bool CMovementSimulation::Initialize(CBaseEntity* pEntity, PlayerStorage& tStorage, bool bHitchance, bool bStrafe)
{
	if (!pEntity || !pEntity->IsPlayer() || !pEntity->As<CTFPlayer>()->IsAlive())
	{
		tStorage.m_bInitFailed = tStorage.m_bFailed = true;
		return false;
	}

	auto pPlayer = pEntity->As<CTFPlayer>();
	tStorage.m_pPlayer = pPlayer;

	I::MoveHelper->SetHost(pPlayer);
	pPlayer->m_pCurrentCommand() = &s_tDummyCmd;

	// store player restore data
	Store(tStorage);

	// store vars
	m_bOldInPrediction = I::Prediction->m_bInPrediction;
	m_bOldFirstTimePredicted = I::Prediction->m_bFirstTimePredicted;
	m_flOldFrametime = I::GlobalVars->frametime;

	// the hacks that make it work
	{
		// use raw vel

		if (pPlayer->m_bDucked() = pPlayer->IsDucking())
		{
			pPlayer->m_fFlags() &= ~FL_DUCKING; // breaks origin's z if FL_DUCKING is not removed
			pPlayer->m_flDucktime() = 0.f;
			pPlayer->m_flDuckJumpTime() = 0.f;
			pPlayer->m_bDucking() = false;
			pPlayer->m_bInDuckJump() = false;
		}

		if (pPlayer != H::Entities.GetLocal())
		{
			pPlayer->m_vecBaseVelocity() = Vec3(); // residual basevelocity causes issues
			if (pPlayer->IsOnGround())
				pPlayer->m_vecVelocity().z = std::min(pPlayer->m_vecVelocity().z, 0.f); // step fix
			else
				pPlayer->m_hGroundEntity() = nullptr; // fix for velocity.z being set to 0 even if in air
		}
		else if (Vars::Misc::Movement::Bunnyhop.Value && G::OriginalCmd.buttons & IN_JUMP)
			tStorage.m_bBunnyHop = true;
	}

	// setup move data
	if (!SetupMoveData(tStorage))
	{
		tStorage.m_bFailed = true;
		return false;
	}

	const int iStrafeSamples = tStorage.m_bDirectMove
		? Vars::Aimbot::Projectile::GroundSamples.Value
		: Vars::Aimbot::Projectile::AirSamples.Value;

	// calculate strafe if desired
	bool bCalculated = bStrafe ? StrafePrediction(tStorage, iStrafeSamples) : false;

	// really hope this doesn't work like shit
	if (bHitchance && bCalculated && !pPlayer->m_vecVelocity().IsZero())
	{
		const auto& vRecords = m_mRecords[pPlayer->entindex()];
		const auto iSamples = vRecords.size();

		float flCurrentChance = 1.f, flAverageYaw = 0.f;
		for (size_t i = 0; i < iSamples; i++)
		{
			if (vRecords.size() <= i + 2)
				break;

			const auto& pRecord1 = vRecords[i], &pRecord2 = vRecords[i + 1];
			const float flYaw1 = Math::VectorAngles(pRecord1.m_vDirection).y, flYaw2 = Math::VectorAngles(pRecord2.m_vDirection).y;
			const float flTime1 = pRecord1.m_flSimTime, flTime2 = pRecord2.m_flSimTime;
			const int iTicks = std::max(TIME_TO_TICKS(flTime1 - flTime2), 1);

			float flYaw = Math::NormalizeAngle(flYaw1 - flYaw2) / iTicks;
			flAverageYaw += flYaw;
			if (tStorage.m_MoveData.m_flMaxSpeed)
				flYaw *= std::clamp(pRecord1.m_vVelocity.Length2D() / tStorage.m_MoveData.m_flMaxSpeed, 0.f, 1.f);

			if ((i + 1) % iStrafeSamples == 0 || i == iSamples - 1)
			{
				flAverageYaw /= i % iStrafeSamples + 1;
				if (fabsf(tStorage.m_flAverageYaw - flAverageYaw) > 0.5f)
					flCurrentChance -= 1.f / ((iSamples - 1) / float(iStrafeSamples) + 1);
				flAverageYaw = 0.f;
			}
		}

		float conf = std::clamp(tStorage.m_flAverageYawConfidence, 0.f, 1.f);
		float required = 0.f;
		if (Vars::Aimbot::Projectile::HitChance.Value > 0.f)
		{
			float base = Vars::Aimbot::Projectile::HitChance.Value / 100.f;
			float maxSpeed = std::max(tStorage.m_MoveData.m_flMaxSpeed, 1.f);
			float speedFrac = std::clamp(tStorage.m_MoveData.m_vecVelocity.Length2D() / maxSpeed, 0.f, 1.f);
			float instability = std::clamp(tStorage.m_flStability / 8.f, 0.f, 1.f);
			float coverage = iStrafeSamples > 0 ? std::clamp((float(std::max<size_t>(iSamples, size_t(1)) - 1) / float(iStrafeSamples)), 0.f, 1.f) : 1.f;
			float adjust = 1.f;
			adjust *= (1.f + 0.25f * speedFrac);
			adjust *= (1.f + 0.25f * instability);
			adjust *= (1.f - 0.20f * conf);
			adjust *= (1.f + 0.15f * (1.f - coverage));
			adjust = std::clamp(adjust, 0.75f, 1.35f);
			required = std::clamp(base * adjust, 0.05f, 0.99f);
		}
		else
		{
			float maxSpeed = std::max(tStorage.m_MoveData.m_flMaxSpeed, 1.f);
			float speedFrac = std::clamp(tStorage.m_MoveData.m_vecVelocity.Length2D() / maxSpeed, 0.f, 1.f);
			required = 0.5f + 0.25f * speedFrac + 0.25f * (1.f - conf); // 50%..100% before final adjustment
			required = std::clamp(required, 0.35f, 0.95f);
		}
		required *= (1.f - 0.1f * conf);
		required = std::clamp(required, 0.1f, 0.95f);
	if (flCurrentChance < required)
		{
			{
		std::ostringstream oss; oss << "Hitchance (" << flCurrentChance * 100 << "% < " << (required * 100.f) << "%)";
				SDK::Output("MovementSimulation", oss.str().c_str(), { 80, 200, 120 }, Vars::Debug::Logging.Value);
			}

			tStorage.m_bFailed = true;
			return false;
		}
	}

	for (int i = 0; i < H::Entities.GetChoke(pPlayer->entindex()); i++)
		RunTick(tStorage);

	return true;
}

bool CMovementSimulation::SetupMoveData(PlayerStorage& tStorage)
{
	if (!tStorage.m_pPlayer)
		return false;

	tStorage.m_MoveData.m_bFirstRunOfFunctions = false;
	tStorage.m_MoveData.m_bGameCodeMovedPlayer = false;
	tStorage.m_MoveData.m_nPlayerHandle = reinterpret_cast<IHandleEntity*>(tStorage.m_pPlayer)->GetRefEHandle();

	tStorage.m_MoveData.m_vecAbsOrigin = tStorage.m_pPlayer->m_vecOrigin();
	tStorage.m_MoveData.m_vecVelocity = tStorage.m_pPlayer->m_vecVelocity();
	tStorage.m_MoveData.m_flMaxSpeed = SDK::MaxSpeed(tStorage.m_pPlayer);
	tStorage.m_MoveData.m_flClientMaxSpeed = tStorage.m_MoveData.m_flMaxSpeed;

	if (!tStorage.m_MoveData.m_vecVelocity.To2D().IsZero())
	{
		int iIndex = tStorage.m_pPlayer->entindex();
		if (iIndex == I::EngineClient->GetLocalPlayer() && G::CurrentUserCmd)
			tStorage.m_MoveData.m_vecViewAngles = G::CurrentUserCmd->viewangles;
		else
		{
			if (!tStorage.m_pPlayer->InCond(TF_COND_SHIELD_CHARGE))
				tStorage.m_MoveData.m_vecViewAngles = { 0.f, Math::VectorAngles(tStorage.m_MoveData.m_vecVelocity).y, 0.f };
			else
				tStorage.m_MoveData.m_vecViewAngles = H::Entities.GetEyeAngles(iIndex);
		}

		const auto& vRecords = m_mRecords[tStorage.m_pPlayer->entindex()];
		if (!vRecords.empty())
		{
			auto& tRecord = vRecords.front();
			if (!tRecord.m_vDirection.IsZero())
			{
				s_tDummyCmd.forwardmove = tRecord.m_vDirection.x;
				s_tDummyCmd.sidemove = -tRecord.m_vDirection.y;
				s_tDummyCmd.upmove = tRecord.m_vDirection.z;
				SDK::FixMovement(&s_tDummyCmd, {}, tStorage.m_MoveData.m_vecViewAngles);
				tStorage.m_MoveData.m_flForwardMove = s_tDummyCmd.forwardmove;
				tStorage.m_MoveData.m_flSideMove = s_tDummyCmd.sidemove;
				tStorage.m_MoveData.m_flUpMove = s_tDummyCmd.upmove;
			}
		}
	}

	tStorage.m_MoveData.m_vecAngles = tStorage.m_MoveData.m_vecOldAngles = tStorage.m_MoveData.m_vecViewAngles;
	if (auto pConstraintEntity = tStorage.m_pPlayer->m_hConstraintEntity().Get())
		tStorage.m_MoveData.m_vecConstraintCenter = pConstraintEntity->GetAbsOrigin();
	else
		tStorage.m_MoveData.m_vecConstraintCenter = tStorage.m_pPlayer->m_vecConstraintCenter();
	tStorage.m_MoveData.m_flConstraintRadius = tStorage.m_pPlayer->m_flConstraintRadius();
	tStorage.m_MoveData.m_flConstraintWidth = tStorage.m_pPlayer->m_flConstraintWidth();
	tStorage.m_MoveData.m_flConstraintSpeedFactor = tStorage.m_pPlayer->m_flConstraintSpeedFactor();

	tStorage.m_flPredictedDelta = GetPredictedDelta(tStorage.m_pPlayer);
	tStorage.m_flSimTime = tStorage.m_pPlayer->m_flSimulationTime();
	tStorage.m_flPredictedSimTime = tStorage.m_flSimTime + tStorage.m_flPredictedDelta;
	tStorage.m_vPredictedOrigin = tStorage.m_MoveData.m_vecAbsOrigin;
	tStorage.m_bDirectMove = tStorage.m_pPlayer->IsOnGround() || tStorage.m_pPlayer->IsSwimming();

	return true;
}

static inline float GetGravity()
{
	static auto sv_gravity = U::ConVars.FindVar("sv_gravity");

	return sv_gravity->GetFloat();
}

static inline float GetFrictionScale(float flVelocityXY, float flTurn, float flVelocityZ, float flMin = 50.f, float flMax = 150.f)
{
	if (0.f >= flVelocityZ || flVelocityZ > 250.f)
		return 1.f;

	static auto sv_airaccelerate = U::ConVars.FindVar("sv_airaccelerate");
	float flScale = std::max(sv_airaccelerate->GetFloat(), 1.f);
	flMin *= flScale, flMax *= flScale;

	// entity friction will be 0.25f if velocity is between 0.f and 250.f
	return Math::RemapVal(fabsf(flVelocityXY * flTurn), flMin, flMax, 1.f, 0.25f);
}

// legacy GetYawDifference & visualization removed


bool CMovementSimulation::StrafePrediction(PlayerStorage& tStorage, int iSamples)
{
	if (tStorage.m_bDirectMove
		? !(Vars::Aimbot::Projectile::StrafePrediction.Value & Vars::Aimbot::Projectile::StrafePredictionEnum::Ground)
		: !(Vars::Aimbot::Projectile::StrafePrediction.Value & Vars::Aimbot::Projectile::StrafePredictionEnum::Air))
		return false;

	GetAverageYaw(tStorage, iSamples);
	return true;
}

bool CMovementSimulation::SetDuck(PlayerStorage& tStorage, bool bDuck) // this only touches origin, bounds
{
	if (bDuck == tStorage.m_pPlayer->m_bDucked())
		return true;

	auto pGameRules = I::TFGameRules();
	auto pViewVectors = pGameRules ? pGameRules->GetViewVectors() : nullptr;
	float flScale = tStorage.m_pPlayer->m_flModelScale();

	if (!tStorage.m_pPlayer->IsOnGround())
	{
		Vec3 vHullMins = (pViewVectors ? pViewVectors->m_vHullMin : Vec3(-24, -24, 0)) * flScale;
		Vec3 vHullMaxs = (pViewVectors ? pViewVectors->m_vHullMax : Vec3(24, 24, 82)) * flScale;
		Vec3 vDuckHullMins = (pViewVectors ? pViewVectors->m_vDuckHullMin : Vec3(-24, -24, 0)) * flScale;
		Vec3 vDuckHullMaxs = (pViewVectors ? pViewVectors->m_vDuckHullMax : Vec3(24, 24, 62)) * flScale;

		if (bDuck)
			tStorage.m_MoveData.m_vecAbsOrigin += (vHullMaxs - vHullMins) - (vDuckHullMaxs - vDuckHullMins);
		else
		{
			Vec3 vOrigin = tStorage.m_MoveData.m_vecAbsOrigin - ((vHullMaxs - vHullMins) - (vDuckHullMaxs - vDuckHullMins));

			CGameTrace trace = {};
			CTraceFilterWorldAndPropsOnly filter = {};
			SDK::TraceHull(vOrigin, vOrigin, vHullMins, vHullMaxs, tStorage.m_pPlayer->SolidMask(), &filter, &trace);
			if (trace.DidHit())
				return false;

			tStorage.m_MoveData.m_vecAbsOrigin = vOrigin;
		}
	}
	tStorage.m_pPlayer->m_bDucked() = bDuck;

	return true;
}

void CMovementSimulation::SetBounds(CTFPlayer* pPlayer)
{
	if (pPlayer->entindex() == I::EngineClient->GetLocalPlayer())
		return;

	// fixes issues with origin compression
	if (auto pGameRules = I::TFGameRules())
	{
		if (auto pViewVectors = pGameRules->GetViewVectors())
		{
			pViewVectors->m_vHullMin = Vec3(-24, -24, 0) + 0.125f;
			pViewVectors->m_vHullMax = Vec3(24, 24, 82) - 0.125f;
			pViewVectors->m_vDuckHullMin = Vec3(-24, -24, 0) + 0.125f;
			pViewVectors->m_vDuckHullMax = Vec3(24, 24, 62) - 0.125f;
		}
	}
}

void CMovementSimulation::RestoreBounds(CTFPlayer* pPlayer)
{
	if (pPlayer->entindex() == I::EngineClient->GetLocalPlayer())
		return;

	if (auto pGameRules = I::TFGameRules())
	{
		if (auto pViewVectors = pGameRules->GetViewVectors())
		{
			pViewVectors->m_vHullMin = Vec3(-24, -24, 0);
			pViewVectors->m_vHullMax = Vec3(24, 24, 82);
			pViewVectors->m_vDuckHullMin = Vec3(-24, -24, 0);
			pViewVectors->m_vDuckHullMax = Vec3(24, 24, 62);
		}
	}
}

void CMovementSimulation::RunTick(PlayerStorage& tStorage, bool bPath, std::function<void(CMoveData&)>* pCallback)
{
	if (tStorage.m_bFailed || !tStorage.m_pPlayer || !tStorage.m_pPlayer->IsPlayer())
		return;

	if (bPath)
		tStorage.m_vPath.push_back(tStorage.m_MoveData.m_vecAbsOrigin);

	I::Prediction->m_bInPrediction = true;
	I::Prediction->m_bFirstTimePredicted = false;
	I::GlobalVars->frametime = I::Prediction->m_bEnginePaused ? 0.f : TICK_INTERVAL;
	SetBounds(tStorage.m_pPlayer);

	float flCorrection = 0.f;
	if (tStorage.m_flAverageYaw)
	{
		if (!tStorage.m_bDirectMove && !tStorage.m_pPlayer->InCond(TF_COND_SHIELD_CHARGE))
		{
			// apply raw measured yaw per tick for air strafing
			tStorage.m_MoveData.m_vecViewAngles.y += tStorage.m_flAverageYaw;
		}
		else
		{
			// apply raw measured yaw per tick for ground strafing
			tStorage.m_MoveData.m_vecViewAngles.y += tStorage.m_flAverageYaw;
		}
	}
	else if (!tStorage.m_bDirectMove)
		tStorage.m_MoveData.m_flForwardMove = tStorage.m_MoveData.m_flSideMove = 0.f;

	float flOldSpeed = tStorage.m_MoveData.m_flClientMaxSpeed;
	if (tStorage.m_pPlayer->m_bDucked() && tStorage.m_pPlayer->IsOnGround() && !tStorage.m_pPlayer->IsSwimming())
		tStorage.m_MoveData.m_flClientMaxSpeed /= 3;

	if (tStorage.m_bBunnyHop && tStorage.m_pPlayer->IsOnGround() && !tStorage.m_pPlayer->m_bDucked())
	{
		tStorage.m_MoveData.m_nOldButtons = 0;
		tStorage.m_MoveData.m_nButtons |= IN_JUMP;
	}

	I::GameMovement->ProcessMovement(tStorage.m_pPlayer, &tStorage.m_MoveData);
	if (pCallback)
		(*pCallback)(tStorage.m_MoveData);

	tStorage.m_MoveData.m_flClientMaxSpeed = flOldSpeed;

	tStorage.m_flSimTime += TICK_INTERVAL;
	tStorage.m_bPredictNetworked = tStorage.m_flSimTime >= tStorage.m_flPredictedSimTime;
	if (tStorage.m_bPredictNetworked)
	{
		tStorage.m_vPredictedOrigin = tStorage.m_MoveData.m_vecAbsOrigin;
		tStorage.m_flPredictedSimTime += tStorage.m_flPredictedDelta;
	}
	bool bLastbDirectMove = tStorage.m_bDirectMove;
	tStorage.m_bDirectMove = tStorage.m_pPlayer->IsOnGround() || tStorage.m_pPlayer->IsSwimming();

	if (tStorage.m_flAverageYaw)
		tStorage.m_MoveData.m_vecViewAngles.y -= flCorrection;
	else if (tStorage.m_bDirectMove && !bLastbDirectMove
		&& !tStorage.m_MoveData.m_flForwardMove && !tStorage.m_MoveData.m_flSideMove
		&& tStorage.m_MoveData.m_vecVelocity.Length2D() > tStorage.m_MoveData.m_flMaxSpeed * 0.015f)
	{
		Vec3 vDirection = tStorage.m_MoveData.m_vecVelocity.Normalized2D() * 450.f;
		s_tDummyCmd.forwardmove = vDirection.x, s_tDummyCmd.sidemove = -vDirection.y;
		SDK::FixMovement(&s_tDummyCmd, {}, tStorage.m_MoveData.m_vecViewAngles);
		tStorage.m_MoveData.m_flForwardMove = s_tDummyCmd.forwardmove, tStorage.m_MoveData.m_flSideMove = s_tDummyCmd.sidemove;
	}

	RestoreBounds(tStorage.m_pPlayer);
}

void CMovementSimulation::RunTick(PlayerStorage& tStorage, bool bPath, std::function<void(CMoveData&)> fCallback)
{
	RunTick(tStorage, bPath, &fCallback);
}

void CMovementSimulation::Restore(PlayerStorage& tStorage)
{
	if (tStorage.m_bInitFailed || !tStorage.m_pPlayer)
		return;

	I::MoveHelper->SetHost(nullptr);
	tStorage.m_pPlayer->m_pCurrentCommand() = nullptr;

	Reset(tStorage);

	I::Prediction->m_bInPrediction = m_bOldInPrediction;
	I::Prediction->m_bFirstTimePredicted = m_bOldFirstTimePredicted;
	I::GlobalVars->frametime = m_flOldFrametime;

	/*
	const bool bInitFailed = tStorage.m_bInitFailed, bFailed = tStorage.m_bFailed;
	memset(&tStorage, 0, sizeof(PlayerStorage));
	tStorage.m_bInitFailed = bInitFailed, tStorage.m_bFailed = bFailed;
	*/
}

float CMovementSimulation::GetPredictedDelta(CBaseEntity* pEntity)
{
	auto& vSimTimes = m_mSimTimes[pEntity->entindex()];
	// use latest observed network delta
	float raw = vSimTimes.empty() ? TICK_INTERVAL : vSimTimes.front();
	return std::clamp(raw, TICK_INTERVAL, 0.25f);
}

float CMovementSimulation::SmoothDelta(float newDelta)
{
	// exponential moving average smoothing for network delta prediction jitter reduction (long name = performance)
	// alpha based on spec constant (0.35), could be tied to a cvar later
	constexpr float kAlpha = 0.35f;
	if (!m_bDeltaEMAInit)
	{
		m_flDeltaEMA = newDelta;
		m_bDeltaEMAInit = true;
	}
	else
	{
		m_flDeltaEMA = kAlpha * newDelta + (1.f - kAlpha) * m_flDeltaEMA;
	}
	// clamp to reasonable bounds at least one tick at most ~0.25s (15 ticks @66.67hz)
	return std::clamp(m_flDeltaEMA, TICK_INTERVAL, 0.25f);
}

float CMovementSimulation::ClampFriction(float rawScale) const
{
	// provide bounded friction scale to avoid extreme corrections influencing yaw / movement prediction
	// desirable range [0.25, 1.25], allow mild boosts above 1 for acceleration phases
	return std::clamp(rawScale, 0.25f, 1.25f);
}

// store per-player state so we can non-destructively simulate and then restore
void CMovementSimulation::Store(PlayerStorage& tStorage)
{
	if (!tStorage.m_pPlayer) return;
	auto p = tStorage.m_pPlayer;
	auto& d = tStorage.m_PlayerData;
	d.m_vecOrigin = p->m_vecOrigin();
	d.m_vecVelocity = p->m_vecVelocity();
	d.m_vecBaseVelocity = p->m_vecBaseVelocity();
	d.m_vecViewOffset = p->m_vecViewOffset();
	d.m_hGroundEntity = p->m_hGroundEntity();
	d.m_fFlags = p->m_fFlags();
	d.m_flDucktime = p->m_flDucktime();
	d.m_flDuckJumpTime = p->m_flDuckJumpTime();
	d.m_bDucked = p->m_bDucked();
	d.m_bDucking = p->m_bDucking();
	d.m_bInDuckJump = p->m_bInDuckJump();
	d.m_flModelScale = p->m_flModelScale();
	d.m_nButtons = p->m_nButtons();
	d.m_flMaxspeed = p->m_flMaxspeed();
	d.m_flFallVelocity = p->m_flFallVelocity();
	d.m_flGravity = p->m_flGravity();
	d.m_nWaterLevel = p->m_nWaterLevel();
	d.m_nWaterType = p->m_nWaterType();
}

// restore stored state
void CMovementSimulation::Reset(PlayerStorage& tStorage)
{
	if (!tStorage.m_pPlayer) return;
	auto p = tStorage.m_pPlayer;
	const auto& d = tStorage.m_PlayerData;
	p->m_vecOrigin() = d.m_vecOrigin;
	p->m_vecVelocity() = d.m_vecVelocity;
	p->m_vecBaseVelocity() = d.m_vecBaseVelocity;
	p->m_vecViewOffset() = d.m_vecViewOffset;
	p->m_hGroundEntity() = d.m_hGroundEntity;
	p->m_fFlags() = d.m_fFlags;
	p->m_flDucktime() = d.m_flDucktime;
	p->m_flDuckJumpTime() = d.m_flDuckJumpTime;
	p->m_bDucked() = d.m_bDucked;
	p->m_bDucking() = d.m_bDucking;
	p->m_bInDuckJump() = d.m_bInDuckJump;
	p->m_flModelScale() = d.m_flModelScale;
	p->m_nButtons() = d.m_nButtons;
	p->m_flMaxspeed() = d.m_flMaxspeed;
	p->m_flFallVelocity() = d.m_flFallVelocity;
	p->m_flGravity() = d.m_flGravity;
	p->m_nWaterLevel() = d.m_nWaterLevel;
	p->m_nWaterType() = d.m_nWaterType;
}