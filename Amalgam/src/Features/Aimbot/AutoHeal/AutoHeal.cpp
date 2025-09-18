#include "AutoHeal.h"

#include "../../Players/PlayerUtils.h"
#include "../../Backtrack/Backtrack.h"
#include "../../CritHack/CritHack.h"
#include "../../Simulation/ProjectileSimulation/ProjectileSimulation.h"
#include "../../Ticks/Ticks.h"
#include "../AimbotProjectile/AimbotProjectile.h"

void CAutoHeal::ActivateOnVoice(CTFPlayer* pLocal, CWeaponMedigun* pWeapon, CUserCmd* pCmd)
{
	if (!Vars::Aimbot::Healing::ActivateOnVoice.Value)
		return;

	auto pTarget = pWeapon->m_hHealingTarget().Get();
	if (!pTarget
		|| Vars::Aimbot::Healing::HealPriority.Value == Vars::Aimbot::Healing::HealPriorityEnum::FriendsOnly
		&& !H::Entities.IsFriend(pTarget->entindex()) && !H::Entities.InParty(pTarget->entindex()))
		return;

	if (m_mMedicCallers.find(pTarget->entindex()) != m_mMedicCallers.end())
		pCmd->buttons |= IN_ATTACK2;
}

static inline Vec3 PredictOrigin(Vec3& vOrigin, Vec3 vVelocity, float flLatency, bool bTrace = true, Vec3 vMins = {}, Vec3 vMaxs = {}, unsigned int nMask = MASK_SOLID, float flNormal = 0.f)
{
	Vec3 vEnd = SDK::PredictOrigin(vOrigin, vVelocity, flLatency, bTrace, vMins, vMaxs, nMask, flNormal);
#ifdef DEBUG_VACCINATOR
	G::SweptStorage.emplace_back(std::pair<Vec3, Vec3>(vOrigin, vEnd), vMins, vMaxs, Vec3(), I::GlobalVars->curtime + 60.f, Color_t(255, 255, 255, 10), true);
#endif
	return vEnd;
}

static inline bool TraceToEntity(CTFPlayer* pPlayer, CBaseEntity* pEntity, Vec3& vPlayerOrigin, Vec3& vEntityOrigin, unsigned int nMask = MASK_SHOT | CONTENTS_GRATE)
{
	CGameTrace trace = {};
	CTraceFilterWorldAndPropsOnly filter = {};

	// Sample multiple points on the player's hull to reduce false negatives
	Vec3 vMins = pPlayer->m_vecMins() + 1;
	Vec3 vMaxs = pPlayer->m_vecMaxs() - 1;
	Vec3 vMinsS = (vMins - vMaxs) / 2;
	Vec3 vMaxsS = (vMaxs - vMins) / 2;

	std::vector<Vec3> vPoints = {
		Vec3(),
		Vec3(vMinsS.x, vMinsS.y, vMaxsS.z),
		Vec3(vMaxsS.x, vMinsS.y, vMaxsS.z),
		Vec3(vMinsS.x, vMaxsS.y, vMaxsS.z),
		Vec3(vMaxsS.x, vMaxsS.y, vMaxsS.z),
		Vec3(vMinsS.x, vMinsS.y, vMinsS.z),
		Vec3(vMaxsS.x, vMinsS.y, vMinsS.z),
		Vec3(vMinsS.x, vMaxsS.y, vMinsS.z),
		Vec3(vMaxsS.x, vMaxsS.y, vMinsS.z)
	};
	for (auto& vPoint : vPoints)
	{
		SDK::Trace(vPlayerOrigin + vPoint, vEntityOrigin, nMask, &filter, &trace);
#ifdef DEBUG_VACCINATOR
		if (trace.fraction == 1.f)
			G::LineStorage.emplace_back(std::pair<Vec3, Vec3>(trace.startpos, trace.endpos), I::GlobalVars->curtime + 60.f, Color_t(0, 255, 0, 25), true);
#endif
		if (trace.fraction == 1.f)
			return true;
	}

	return false;
}

static inline int GetShotsWithinTime(int iWeaponID, float flFireRate, float flTime)
{
	int iTicks = TIME_TO_TICKS(std::max(0.f, flTime));
	int iDelay = 1;
	switch (iWeaponID)
	{
	case TF_WEAPON_MINIGUN:
	case TF_WEAPON_PIPEBOMBLAUNCHER:
	case TF_WEAPON_CANNON:
		iDelay = 2;
	}
	int iIntervalTicks = static_cast<int>(std::ceilf(std::max(TICK_INTERVAL, flFireRate) / TICK_INTERVAL));
	int iAvail = std::max(0, iTicks - iDelay);
	return 1 + (iAvail / std::max(1, iIntervalTicks));
}

static inline float GetDamage(CBaseEntity* pProjectile, CTFPlayer* pOwner, CTFWeaponBase* pWeapon, CTFPlayer* pTarget, float flMult, Vec3 vProjectileOrigin, int* pType = nullptr)
{
	float flReturn = 100.f;
	switch (pProjectile->GetClassID())
	{
	case ETFClassID::CTFGrenadePipebombProjectile:
		flReturn = 100.f;
		break;
	case ETFClassID::CTFWeaponBaseMerasmusGrenade:
		flReturn = 50.f;
		break;
	case ETFClassID::CTFProjectile_SpellMeteorShower:
		flReturn = 100.f;
		if (pType)
			*pType = MEDIGUN_FIRE_RESIST;
		break;
	case ETFClassID::CTFProjectile_Arrow:
		flReturn = pProjectile->As<CTFProjectile_Arrow>()->CanHeadshot()
			? Math::RemapVal(pProjectile->As<CTFProjectile_Arrow>()->m_vInitialVelocity().Length(), 1800.f, 2600.f, 50.f, 120.f)
			: 40.f;
		if (pType)
			*pType = MEDIGUN_BULLET_RESIST;
		break;
	case ETFClassID::CTFProjectile_HealingBolt:
		flReturn = Math::RemapVal(pOwner->m_vecOrigin().DistTo(vProjectileOrigin), 200.f, 1600.f, 38.f, 75.f);
		if (pType)
			*pType = MEDIGUN_BULLET_RESIST;
		break;
	case ETFClassID::CTFProjectile_Rocket:
	case ETFClassID::CTFProjectile_EnergyBall:
		flReturn = flMult ? 90.f : Math::RemapVal(pOwner->m_vecOrigin().DistTo(vProjectileOrigin), 500.f, 920.f, 90.f, 48.f);
		break;
	case ETFClassID::CTFProjectile_SentryRocket:
		flReturn = flMult ? 100.f : Math::RemapVal(pOwner->m_vecOrigin().DistTo(vProjectileOrigin), 100.f, 920.f, 100.f, 50.f);
		break;
	case ETFClassID::CTFProjectile_BallOfFire:
		flReturn = pTarget->InCond(TF_COND_BURNING) || pTarget->InCond(TF_COND_BURNING_PYRO) ? 90.f : 30.f;
		if (pType)
			*pType = MEDIGUN_FIRE_RESIST;
		break;
	case ETFClassID::CTFProjectile_SpellFireball:
		flReturn = 100.f;
		if (pType)
			*pType = MEDIGUN_FIRE_RESIST;
		break;
	case ETFClassID::CTFProjectile_SpellLightningOrb:
		flReturn = 100.f;
		if (pType)
			*pType = MEDIGUN_FIRE_RESIST;
		break;
	case ETFClassID::CTFProjectile_Flare:
		flReturn = 30.f;
		if (pType)
			*pType = MEDIGUN_FIRE_RESIST;
		break;
	case ETFClassID::CTFProjectile_EnergyRing:
		flReturn = 60.f;
		if (pType)
			*pType = MEDIGUN_BULLET_RESIST;
	}
	if (pWeapon)
		flReturn *= SDK::AttribHookValue(1, "mult_dmg", pWeapon);
	return flReturn;
}

static inline float GetMult(CBaseEntity* pProjectile, CTFWeaponBase* pWeapon, CTFPlayer* pTarget)
{
	float flReturn = pTarget->IsMarked() ? 1.36f : 1.f;
	switch (pProjectile->GetClassID())
	{
	case ETFClassID::CTFGrenadePipebombProjectile:
	case ETFClassID::CTFWeaponBaseMerasmusGrenade:
	case ETFClassID::CTFProjectile_SpellMeteorShower:
		if (pProjectile->As<CTFWeaponBaseGrenadeProj>()->m_bCritical())
			flReturn = 3.f;
		else if (pProjectile->As<CTFWeaponBaseGrenadeProj>()->m_iDeflected() && (pProjectile->GetClassID() != ETFClassID::CTFGrenadePipebombProjectile || !pProjectile->GetAbsVelocity().IsZero()))
			flReturn = 1.36f;
		break;
	case ETFClassID::CTFProjectile_Arrow:
		if (pProjectile->As<CTFProjectile_Arrow>()->CanHeadshot())
		{
			flReturn = 3.f;
			break;
		}
		[[fallthrough]];
	case ETFClassID::CTFProjectile_HealingBolt:
		if (pProjectile->As<CTFProjectile_Arrow>()->m_bCritical())
			flReturn = 3.f;
		else if (pProjectile->As<CTFBaseRocket>()->m_iDeflected())
			flReturn = 1.36f;
		break;
	case ETFClassID::CTFProjectile_Rocket:
	case ETFClassID::CTFProjectile_BallOfFire:
	case ETFClassID::CTFProjectile_SentryRocket:
	case ETFClassID::CTFProjectile_SpellFireball:
	case ETFClassID::CTFProjectile_SpellLightningOrb:
		if (pProjectile->As<CTFProjectile_Rocket>()->m_bCritical())
			flReturn = 3.f;
		else if (pProjectile->As<CTFBaseRocket>()->m_iDeflected()
			|| pTarget->InCond(TF_COND_BLASTJUMPING) && pWeapon && SDK::AttribHookValue(0, "mini_crit_airborne", pWeapon) == 1)
			flReturn = 1.36f;
		break;
	case ETFClassID::CTFProjectile_EnergyBall:
		if (pProjectile->As<CTFProjectile_EnergyBall>()->m_bChargedShot())
			flReturn = 2.f; // whatever
		else if (pProjectile->As<CTFBaseRocket>()->m_iDeflected())
			flReturn = 1.36f;
		break;
	case ETFClassID::CTFProjectile_Flare:
		if (pProjectile->As<CTFProjectile_Flare>()->m_bCritical() || pTarget->InCond(TF_COND_BURNING) || pTarget->InCond(TF_COND_BURNING_PYRO))
			flReturn = 3.f;
		else if (pProjectile->As<CTFBaseRocket>()->m_iDeflected())
			flReturn = 1.36f;
	}
	return flReturn;
}

void CAutoHeal::GetDangers(CTFPlayer* pTarget, bool bVaccinator, float& flBulletOut, float& flBlastOut, float& flFireOut)
{
	if (pTarget->IsInvulnerable())
		return;

	float flBulletDanger = 0.f, flBlastDanger = 0.f, flFireDanger = 0.f;
	float flBulletResist = pTarget->InCond(TF_COND_BULLET_IMMUNE) ? 1.f : pTarget->InCond(TF_COND_MEDIGUN_UBER_BULLET_RESIST) ? 0.75f : pTarget->InCond(TF_COND_MEDIGUN_SMALL_BULLET_RESIST) ? -0.1f : 0.f;
	float flBlastResist = pTarget->InCond(TF_COND_BLAST_IMMUNE) ? 1.f : pTarget->InCond(TF_COND_MEDIGUN_UBER_BLAST_RESIST) ? 0.75f : pTarget->InCond(TF_COND_MEDIGUN_SMALL_BLAST_RESIST) ? -0.1f : 0.f;
	float flFireResist = pTarget->InCond(TF_COND_FIRE_IMMUNE) ? 1.f : pTarget->InCond(TF_COND_MEDIGUN_UBER_FIRE_RESIST) ? 0.75f : pTarget->InCond(TF_COND_MEDIGUN_SMALL_FIRE_RESIST) ? -0.1f : 0.f;

	float flLatency = F::Backtrack.GetReal();
	int iPlayerHealth = pTarget->m_iHealth();
	Vec3 vTargetOrigin = PredictOrigin(pTarget->m_vecOrigin(), pTarget->m_vecVelocity(), pTarget->entindex() != I::EngineClient->GetLocalPlayer() ? flLatency : 0.f, true, pTarget->m_vecMins(), pTarget->m_vecMaxs(), pTarget->SolidMask());
	Vec3 vTargetCenter = vTargetOrigin + pTarget->GetOffset() / 2;
	Vec3 vTargetEye = vTargetOrigin + pTarget->GetViewOffset();

	for (auto pEntity : H::Entities.GetGroup(EGroupType::PLAYERS_ENEMIES))
	{
		auto pPlayer = pEntity->As<CTFPlayer>();
		int iIndex = pPlayer->entindex();
		if (!pPlayer->CanAttack(true, false))
			continue;

		auto pWeapon = pPlayer->m_hActiveWeapon()->As<CTFWeaponBase>();
		int nWeaponID = pWeapon ? pWeapon->GetWeaponID() : 0;
		auto eWeaponType = SDK::GetWeaponType(pWeapon);
		if (eWeaponType == EWeaponType::UNKNOWN || eWeaponType == EWeaponType::MELEE // if we ever want to port this over to auto uber or something, handle melee
			|| nWeaponID == TF_WEAPON_DRG_POMSON || nWeaponID == TF_WEAPON_PIPEBOMBLAUNCHER)
			continue;

		Vec3 vPlayerOrigin = PredictOrigin(pPlayer->m_vecOrigin(), pPlayer->m_vecVelocity(), flLatency, true, pPlayer->m_vecMins(), pPlayer->m_vecMaxs(), pPlayer->SolidMask());
		Vec3 vPlayerEye = vPlayerOrigin + pPlayer->GetViewOffset();

		bool bCheater = F::PlayerUtils.HasTag(iIndex, F::PlayerUtils.TagToIndex(CHEATER_TAG));
		bool bZoom = pPlayer->InCond(TF_COND_ZOOMED);
		float flFOV = Math::CalcFov(H::Entities.GetEyeAngles(iIndex) + H::Entities.GetPingAngles(iIndex), Math::CalcAngle(vPlayerEye, bZoom ? vTargetEye : vTargetCenter));
		bool bFOV = bCheater || flFOV < (bZoom ? 10 : eWeaponType == EWeaponType::HITSCAN ? 30 : 90);
		if (!bFOV || !TraceToEntity(pTarget, pPlayer, vTargetCenter, vPlayerEye))
			continue;

		bool bCrits = pPlayer->IsCritBoosted() || F::CritHack.WeaponCanCrit(pWeapon);
		bool bMinicrits = pPlayer->IsMiniCritBoosted() || pTarget->IsMarked();
		float flDistance = vPlayerEye.DistTo(vTargetCenter);
		float flDamage, flMult = bCrits ? 3.f : bMinicrits ? 1.36f : 1.f;
		float flFireRate = std::min(pWeapon->GetFireRate(), 1.f);
		int iBulletCount = pWeapon->GetBulletsPerShot();
		int iShotsWithinTime = GetShotsWithinTime(nWeaponID, flFireRate, flLatency + TICKS_TO_TIME(bCheater ? 22 : 0));
		switch (nWeaponID)
		{
		case TF_WEAPON_SNIPERRIFLE:
		case TF_WEAPON_SNIPERRIFLE_DECAP:
		case TF_WEAPON_SNIPERRIFLE_CLASSIC:
		{
			bool bClassic = nWeaponID == TF_WEAPON_SNIPERRIFLE_CLASSIC;
			bool bHeadshot = pWeapon->As<CTFSniperRifle>()->GetRifleType() != RIFLE_JARATE;
			bool bPiss = SDK::AttribHookValue(0, "jarate_duration", pWeapon) > 0;
			auto GetSniperDot = [](CBaseEntity* pEntity) -> CSniperDot*
				{
					for (auto pDot : H::Entities.GetGroup(EGroupType::MISC_DOTS))
					{
						if (pDot->m_hOwnerEntity().Get() == pEntity)
							return pDot->As<CSniperDot>();
					}
					return nullptr;
				};
			if (CSniperDot* pPlayerDot = GetSniperDot(pEntity))
			{
				float flChargeTime = std::max(SDK::AttribHookValue(3.f, "mult_sniper_charge_per_sec", pWeapon), 1.5f);
				flDamage = Math::RemapVal(TICKS_TO_TIME(I::ClientState->m_ClockDriftMgr.m_nServerTick) - pPlayerDot->m_flChargeStartTime() - 0.3f, 0.f, flChargeTime, 50.f, 150.f);
				if (bClassic && flDamage != 150.f)
					flDamage = SDK::AttribHookValue(flDamage, "bodyshot_damage_modify", pWeapon);
				else
					flMult = std::max(bHeadshot || bClassic ? 3.f : bPiss ? 1.36f : 1.f, flMult);
				break;
			}
			if (SDK::AttribHookValue(0, "sniper_only_fire_zoomed", pWeapon) && !bZoom)
				flDamage = 0.f;
			else if (bClassic || !bZoom)
				flDamage = SDK::AttribHookValue(50.f, "bodyshot_damage_modify", pWeapon);
			else
				flDamage = 150.f, flMult = flMult = std::max(bHeadshot || bClassic ? 3.f : bPiss ? 1.36f : 1.f, flMult);
			break;
		}
		case TF_WEAPON_COMPOUND_BOW:
			flDamage = 120.f, flMult = 3.f;
			break;
		case TF_WEAPON_FLAMETHROWER:
			flDamage = 80.f * flFireRate;
			break;
		default:
			flDamage = pWeapon->GetDamage(false);
		}

		switch (eWeaponType)
		{
		case EWeaponType::HITSCAN:
		{
			float flResistMult = flBulletResist > 0.f ? (1.f - flBulletResist) : (1.f + flBulletResist);
			flDamage *= flMult * flResistMult * (pWeapon ? SDK::AttribHookValue(1, "mult_dmg", pWeapon) : 1);

			float flSpread = std::clamp(pWeapon->GetWeaponSpread(), 0.001f, 1.f);
			Vec3 vExtents = pTarget->m_vecMaxs() - pTarget->m_vecMins();
			float flTargetRadius = 0.5f * std::sqrt(vExtents.x * vExtents.x + vExtents.y * vExtents.y);
			float flSpreadRadius = std::max(1.f, flDistance * flSpread);
			float flPelletFrac = std::clamp(flTargetRadius / flSpreadRadius, 0.f, 1.f);
			float flMappedCount = std::max(1.f, iBulletCount * flPelletFrac);
			float flDamageInLatency = flDamage * flMappedCount * iShotsWithinTime;
			float flDamageDanger = flDamageInLatency / iPlayerHealth;
			float flDistanceDanger = Math::RemapVal(flDistance, 100, 100 / flSpread, 1.f, 0.001f);
			if (nWeaponID == TF_WEAPON_MINIGUN && !pPlayer->InCond(TF_COND_AIMING))
				flDistanceDanger = std::max(flDistanceDanger, 0.05f);
			if (bCheater) // may use seed pred +/ crits
				flDistanceDanger = std::max(flDistanceDanger, bCrits ? 1.f : 0.34f);

#ifdef DEBUG_VACCINATOR
			//SDK::Output("Hitscan", std::format("{}, {}, {}, {}, {}", flDamage, flSpread, flDamageInLatency, flDamageDanger, flDistanceDanger).c_str(), { 255, 200, 200 }, Vars::Debug::Logging.Value);
#endif
			flBulletDanger += flDamageDanger * flDistanceDanger;
			break;
		}
		case EWeaponType::PROJECTILE:
		{
			ProjectileInfo tProjInfo = {};
			if (!F::ProjSim.GetInfo(pPlayer, pWeapon, {}, tProjInfo, ProjSimEnum::NoRandomAngles | ProjSimEnum::MaxSpeed))
				continue;

			int iType = MEDIGUN_BLAST_RESIST;
			switch (nWeaponID)
			{
			case TF_WEAPON_COMPOUND_BOW:
			case TF_WEAPON_CROSSBOW:
			case TF_WEAPON_SHOTGUN_BUILDING_RESCUE:
			case TF_WEAPON_SYRINGEGUN_MEDIC:
			case TF_WEAPON_RAYGUN: iType = MEDIGUN_BULLET_RESIST; break;
			case TF_WEAPON_FLAMETHROWER: // maybe just test full range?
			case TF_WEAPON_FLAME_BALL:
			case TF_WEAPON_FLAREGUN:
			case TF_WEAPON_FLAREGUN_REVENGE: iType = MEDIGUN_FIRE_RESIST; break;
			}

			float flResistMultProj = 1.f;
			switch (iType)
			{
			case MEDIGUN_BULLET_RESIST: flResistMultProj = flBulletResist > 0.f ? (1.f - flBulletResist) : (1.f + flBulletResist); break;
			case MEDIGUN_BLAST_RESIST: flResistMultProj = flBlastResist > 0.f ? (1.f - flBlastResist) : (1.f + flBlastResist); break;
			case MEDIGUN_FIRE_RESIST: flResistMultProj = flFireResist > 0.f ? (1.f - flFireResist) : (1.f + flFireResist); break;
			}
			flDamage *= flMult * flResistMultProj * (pWeapon ? SDK::AttribHookValue(1, "mult_dmg", pWeapon) : 1);

			float flRadius = tProjInfo.m_flVelocity * flLatency + F::AimbotProjectile.GetSplashRadius(pWeapon, pPlayer) + pTarget->GetSize().Length() / 2;
			float flDamageInLatency = flDamage * iBulletCount * iShotsWithinTime;
			float flDamageDanger = flDamageInLatency / iPlayerHealth;
			float flDistanceDanger = Math::RemapVal(std::clamp(flDistance, 0.f, flRadius), 0.f, flRadius, 1.f, 0.001f);

#ifdef DEBUG_VACCINATOR
			SDK::Output("Projectile", std::format("{}, {}, {}, {}", flDamage, flDamageInLatency, flDamageDanger, flDistanceDanger).c_str(), { 255, 200, 200 }, Vars::Debug::Logging.Value);
#endif
			switch (iType)
			{
			case MEDIGUN_BULLET_RESIST: flBulletDanger += flDamageDanger * flDistanceDanger; break;
			case MEDIGUN_BLAST_RESIST: flBlastDanger += flDamageDanger * flDistanceDanger; break;
			case MEDIGUN_FIRE_RESIST: flFireDanger += flDamageDanger * flDistanceDanger; break;
			}
		}
		}
	}

	for (auto pEntity : H::Entities.GetGroup(EGroupType::BUILDINGS_ENEMIES))
	{
		auto pSentry = pEntity->As<CObjectSentrygun>();
		if (!pSentry->IsBuilding())
			continue;

		if (pSentry->m_hEnemy().Get() != pTarget && pSentry->m_hAutoAimTarget().Get() != pTarget || !pSentry->m_iAmmoShells())
			continue;

		float flDistance = pSentry->GetCenter().DistTo(vTargetCenter);
		float flDamage = 19.f, flMult = pTarget->IsMarked() ? 1.36f : 1.f;
		float flFireRate;
		switch (pSentry->m_bMiniBuilding() ? 0 : pSentry->m_iUpgradeLevel())
		{
		case 0:
			flDamage /= 2;
			flFireRate = pSentry->m_bPlayerControlled() ? 0.09f : 0.18f;
			break;
		case 1:
			flFireRate = pSentry->m_bPlayerControlled() ? 0.135f : 0.225f;
			break;
		default:
			flFireRate = pSentry->m_bPlayerControlled() ? 0.09f : 0.135f;
		}
		int iShotsWithinTime = GetShotsWithinTime(0, flFireRate, 1.f /*flLatency*/);
		{
			float flResistMult = flBulletResist > 0.f ? (1.f - flBulletResist) : (1.f + flBulletResist);
			flDamage *= flMult * flResistMult;
		}

		float flDamageInLatency = flDamage * iShotsWithinTime;
		float flDamageDanger = flDamageInLatency / iPlayerHealth;
		float flDistanceDanger = Math::RemapVal(flDistance, 1100.f, 3200.f, 1.f, 0.001f);

#ifdef DEBUG_VACCINATOR
		SDK::Output("Sentry", std::format("{}, {}, {}, {}", flDamage, flDamageInLatency, flDamageDanger, flDistanceDanger).c_str(), { 255, 200, 200 }, Vars::Debug::Logging.Value);
#endif
		flBulletDanger += flDamageDanger * flDistanceDanger;
	}

	for (auto pEntity : H::Entities.GetGroup(EGroupType::WORLD_PROJECTILES))
	{
		CTFPlayer* pOwner = nullptr;
		CTFWeaponBase* pWeapon = nullptr;

		switch (pEntity->GetClassID())
		{
		case ETFClassID::CTFGrenadePipebombProjectile:
		case ETFClassID::CTFWeaponBaseMerasmusGrenade:
		case ETFClassID::CTFProjectile_SpellMeteorShower:
			pWeapon = pEntity->As<CTFGrenadePipebombProjectile>()->m_hOriginalLauncher()->As<CTFWeaponBase>();
			pOwner = pEntity->As<CTFWeaponBaseGrenadeProj>()->m_hThrower()->As<CTFPlayer>();
			break;
		case ETFClassID::CTFProjectile_Arrow:
		case ETFClassID::CTFProjectile_HealingBolt:
		case ETFClassID::CTFProjectile_Rocket:
		case ETFClassID::CTFProjectile_SentryRocket:
		case ETFClassID::CTFProjectile_BallOfFire:
		case ETFClassID::CTFProjectile_SpellFireball:
		case ETFClassID::CTFProjectile_SpellLightningOrb:
		case ETFClassID::CTFProjectile_EnergyBall:
		case ETFClassID::CTFProjectile_Flare:
			pWeapon = pEntity->As<CTFBaseRocket>()->m_hLauncher()->As<CTFWeaponBase>();
			pOwner = pWeapon ? pWeapon->m_hOwner()->As<CTFPlayer>() : nullptr;
			break;
		case ETFClassID::CTFProjectile_EnergyRing:
			pWeapon = pEntity->As<CTFBaseProjectile>()->m_hLauncher()->As<CTFWeaponBase>();
			pOwner = pWeapon ? pWeapon->m_hOwner()->As<CTFPlayer>() : nullptr;
		}
		if (!pOwner
			|| (!F::AimbotGlobal.FriendlyFire() || pEntity->GetClassID() == ETFClassID::CTFProjectile_HealingBolt) && pOwner->m_iTeamNum() == pTarget->m_iTeamNum()
			|| pWeapon && !pWeapon->GetDamage())
			continue;

		Vec3 vVelocity = F::ProjSim.GetVelocity(pEntity);
		float flRadius = F::AimbotProjectile.GetSplashRadius(pEntity, pWeapon, pOwner);

		Vec3 vProjectileOrigin = PredictOrigin(pEntity->m_vecOrigin(), vVelocity, flLatency, true, pEntity->m_vecMins(), pEntity->m_vecMaxs());
		if (!TraceToEntity(pTarget, pEntity, vTargetEye, vProjectileOrigin, MASK_SHOT))
			continue;

		int iType = MEDIGUN_BLAST_RESIST;
		float flMult = GetMult(pEntity, pWeapon, pTarget);
		float flDamage = GetDamage(pEntity, pOwner, pWeapon, pTarget, flMult, vProjectileOrigin, &iType);
		float flResistMultWorld = 1.f;
		switch (iType)
		{
		case MEDIGUN_BULLET_RESIST: flResistMultWorld = flBulletResist > 0.f ? (1.f - flBulletResist) : (1.f + flBulletResist); break;
		case MEDIGUN_BLAST_RESIST: flResistMultWorld = flBlastResist > 0.f ? (1.f - flBlastResist) : (1.f + flBlastResist); break;
		case MEDIGUN_FIRE_RESIST: flResistMultWorld = flFireResist > 0.f ? (1.f - flFireResist) : (1.f + flFireResist); break;
		}
		flDamage *= flMult * flResistMultWorld;

		flRadius += vVelocity.Length() * flLatency + pTarget->GetSize().Length() / 2;
		float flDamageDanger = flDamage / iPlayerHealth;
		float flDistanceDanger = Math::RemapVal(std::max(0.f, pTarget->m_vecOrigin().DistTo(vProjectileOrigin)), 0.f, flRadius, 1.f, 0.001f);

#ifdef DEBUG_VACCINATOR
		G::SphereStorage.emplace_back(vProjectileOrigin, flRadius, 36, 36, I::GlobalVars->curtime + 60.f, Color_t(255, 255, 255, 1), Color_t(0, 0, 0, 0), true);
		SDK::Output(std::format("Projectile {}", iType).c_str(), std::format("{}, {}, {}", flDamage, flDamageDanger, flDistanceDanger).c_str(), { 255, 200, 200 }, Vars::Debug::Logging.Value);
#endif
		switch (iType)
		{
		case MEDIGUN_BULLET_RESIST:
			flBulletDanger += flDamageDanger * flDistanceDanger;
			break;
		case MEDIGUN_BLAST_RESIST:
			flBlastDanger += flDamageDanger * flDistanceDanger;
			break;
		case MEDIGUN_FIRE_RESIST:
			flFireDanger += flDamageDanger * flDistanceDanger;
		}
	}

	if (m_flDamagedTime <= TICK_INTERVAL)
	{
		if (pTarget->InCond(TF_COND_BURNING) || pTarget->InCond(TF_COND_BURNING_PYRO))
		{
			m_flDamagedTime = TICK_INTERVAL * 2;
			m_iDamagedType = MEDIGUN_FIRE_RESIST;
			m_flDamagedDPS = 8.f;
		}
	}
	m_flDamagedTime = std::max(m_flDamagedTime - TICK_INTERVAL, 0.f);
	if (m_flDamagedTime > 0.f)
	{
		switch (m_iDamagedType)
		{
		case MEDIGUN_BULLET_RESIST:
			flBulletDanger += m_flDamagedDPS / iPlayerHealth;
			break;
		case MEDIGUN_BLAST_RESIST:
			flBlastDanger += m_flDamagedDPS / iPlayerHealth;
			break;
		case MEDIGUN_FIRE_RESIST:
			flFireDanger += m_flDamagedDPS / iPlayerHealth;
		}
	}

	// ignore resistances we are already charged with, else scale
	if (flBulletResist >= (bVaccinator ? 0.75f : 1.f))
		flBulletDanger = 0.f;
	else
		flBulletDanger *= Vars::Aimbot::Healing::AutoVaccinatorBulletScale.Value / 100.f;
	if (flBlastResist >= (bVaccinator ? 0.75f : 1.f))
		flBlastDanger = 0.f;
	else
		flBlastDanger *= Vars::Aimbot::Healing::AutoVaccinatorBlastScale.Value / 100.f;
	if (flFireResist >= (bVaccinator ? 0.75f : 1.f))
		flFireDanger = 0.f;
	else
		flFireDanger *= Vars::Aimbot::Healing::AutoVaccinatorFireScale.Value / 100.f;

	flBulletOut = std::max(flBulletOut, flBulletDanger);
	flBlastOut = std::max(flBlastOut, flBlastDanger);
	flFireOut = std::max(flFireOut, flFireDanger);
}

void CAutoHeal::SwapResistType(CUserCmd* pCmd, int iType)
{
	if (m_iResistType != iType && !(G::LastUserCmd->buttons & IN_RELOAD))
		pCmd->buttons |= IN_RELOAD;
}

void CAutoHeal::ActivateResistType(CUserCmd* pCmd, int iType)
{
	if (m_flChargeLevel >= 0.25f && m_iResistType == iType)
		pCmd->buttons |= IN_ATTACK2;
}

void CAutoHeal::AutoVaccinator(CTFPlayer* pLocal, CWeaponMedigun* pWeapon, CUserCmd* pCmd)
{
	if (!Vars::Aimbot::Healing::AutoVaccinator.Value || pWeapon->GetMedigunType() != MEDIGUN_RESIST)
		return;

#ifdef DEBUG_VACCINATOR
	G::LineStorage.clear();
	G::SweptStorage.clear();
	G::SphereStorage.clear();
#endif
	if (m_iResistType == -1)
		m_iResistType = pWeapon->GetResistType();
#ifdef DEBUG_VACCINATOR
	vResistDangers = {
#else
	std::vector<std::pair<float, int>> vResistDangers = {
#endif
		{ 0.f, MEDIGUN_BULLET_RESIST },
		{ 0.f, MEDIGUN_BLAST_RESIST },
		{ 0.f, MEDIGUN_FIRE_RESIST }
	};

	std::vector<CTFPlayer*> vTargets = { pLocal };
	if (auto pTarget = pWeapon->m_hHealingTarget()->As<CTFPlayer>(); pTarget &&
		(Vars::Aimbot::Healing::HealPriority.Value <= Vars::Aimbot::Healing::HealPriorityEnum::FriendsOnly
		|| H::Entities.IsFriend(pTarget->entindex()) || H::Entities.InParty(pTarget->entindex())))
		vTargets.push_back(pTarget);

	for (auto pTarget : vTargets)
		GetDangers(pTarget, true, vResistDangers[MEDIGUN_BULLET_RESIST].first, vResistDangers[MEDIGUN_BLAST_RESIST].first, vResistDangers[MEDIGUN_FIRE_RESIST].first);
	std::sort(vResistDangers.begin(), vResistDangers.end(), [&](const std::pair<float, int>& a, const std::pair<float, int>& b) -> bool
		{
			return a.first > b.first;
		});

	int iTargetResist = vResistDangers.front().second;
	float flTargetDanger = vResistDangers.front().first;
	if (flTargetDanger)
		SwapResistType(pCmd, iTargetResist);
	if (flTargetDanger >= 1.f)
		ActivateResistType(pCmd, iTargetResist);

	if (m_bPreventResistSwap)
		pCmd->buttons &= ~IN_RELOAD;
	if (m_bPreventResistCharge)
		pCmd->buttons &= ~IN_ATTACK2;
	if (pCmd->buttons & IN_RELOAD && !(G::LastUserCmd->buttons & IN_RELOAD))
	{
		m_iResistType = (m_iResistType + 1) % 3;
		float flAdd = F::Backtrack.GetReal(MAX_FLOWS, false) * 1.5f + 0.1f;
		m_flSwapTime = I::GlobalVars->curtime + std::clamp(flAdd, 0.15f, 0.35f);
	}
	if (m_flSwapTime < I::GlobalVars->curtime)
		m_iResistType = pWeapon->GetResistType();
	m_flChargeLevel = pWeapon->m_flChargeLevel();
}

void CAutoHeal::AutoCbowHealSwitch(CTFPlayer* pLocal, CWeaponMedigun* pWeapon, CUserCmd* pCmd)
{
	if (!Vars::Aimbot::Healing::AutoArrow.Value)
		return;

	// only on medigun + ready
	if (!pLocal->CanAttack() || pWeapon->GetWeaponID() != TF_WEAPON_MEDIGUN)
		return;

	// respect inputs: allow healing beam; only block swap/uber
	if ((pCmd->buttons & (IN_ATTACK2 | IN_RELOAD))
		|| pWeapon->m_bChargeRelease())
		return;

	// vacc mode guard
	bool bVaccGuardBlock = false;
	if (Vars::Aimbot::Healing::AutoVaccinator.Value && pWeapon->GetMedigunType() == MEDIGUN_RESIST)
	{
		// only block if actively swapping/ubering; allow normal healing
		bVaccGuardBlock = (pCmd->buttons & (IN_RELOAD | IN_ATTACK2)) || I::GlobalVars->curtime < m_flSwapTime;
	}

	// anti-flap and adaptive cooldown
	if (I::GlobalVars->curtime < m_flNextArrowSwitch)
		return;
	if (m_flLastArrowShot > 0.f)
	{
		// base from slider
		float flBase = std::max(0.f, Vars::Aimbot::Healing::AutoArrowCooldown.Value);
		// factors: teammate HP deficit, distance, danger, vacc state
		float flDeficit = 0.f, flDist = 0.f, flDangerMax = 0.f;
		if (auto pHealTarget = pWeapon->m_hHealingTarget()->As<CTFPlayer>())
		{
			int iMaxHP = std::max(1, pHealTarget->GetMaxHealth());
			flDeficit = 1.f - ((float)pHealTarget->m_iHealth() / (float)iMaxHP); // 0..1
			flDist = pLocal->GetShootPos().DistTo(pHealTarget->m_vecOrigin());
			float b, l, f; b = l = f = 0.f; GetDangers(pHealTarget, pWeapon->GetMedigunType() == MEDIGUN_RESIST, b, l, f);
			flDangerMax = std::max({ b, l, f });
		}
		// map deficit (0..1) to cooldown scale (1.2..0.6) => lower HP = shorter wait
		float fDefMod = Math::Lerp(1.2f, 0.6f, std::clamp(flDeficit, 0.f, 1.f));
		// map distance (0..1500) to (0.8..1.25) => farther = longer wait to let bolt land
		float fDistMod = Math::Lerp(0.8f, 1.25f, std::clamp(flDist / 1500.f, 0.f, 1.f));
		// map danger (0..1+) to (1.0..0.7) => higher danger = shorter wait
		float fDngrMod = Math::Lerp(1.f, 0.7f, std::clamp(flDangerMax, 0.f, 1.f));
		// vacc swap window slightly lengthens
		float fVaxMod = ((pCmd->buttons & (IN_RELOAD | IN_ATTACK2)) || I::GlobalVars->curtime < m_flSwapTime) ? 1.15f : 1.f;
		float flAdapt = std::clamp(flBase * fDefMod * fDistMod * fDngrMod * fVaxMod, flBase * 0.5f, flBase * 1.5f);
		if (I::GlobalVars->curtime - m_flLastArrowShot < flAdapt)
			return;
	}

	// scan team in FOV
	const Vec3 vLocalPos = F::Ticks.GetShootPos();
	const Vec3 vLocalAngles = I::EngineClient->GetViewAngles();

	bool bNeedHeal = false;
	bool bCriticalTarget = false;
	float flHPThreshold = Vars::Aimbot::Healing::AutoArrowHealthThreshold.Value;

	// prefer current heal target first
	if (auto pHealTarget = pWeapon->m_hHealingTarget()->As<CTFPlayer>())
	{
		if (Vars::Aimbot::Healing::HealPriority.Value <= Vars::Aimbot::Healing::HealPriorityEnum::FriendsOnly
			|| H::Entities.IsFriend(pHealTarget->entindex()) || H::Entities.InParty(pHealTarget->entindex()))
		{
			int iMaxHP = std::max(1, pHealTarget->GetMaxHealth());
			float flHP = 100.f * (float)pHealTarget->m_iHealth() / (float)iMaxHP;
			float flCrit = Vars::Aimbot::Healing::AutoArrowCriticalThreshold.Value;
			float flFOVTo; Vec3 vPos, vAngleTo;
			if (F::AimbotGlobal.PlayerBoneInFOV(pHealTarget, vLocalPos, vLocalAngles, flFOVTo, vPos, vAngleTo)
				&& flFOVTo <= Vars::Aimbot::General::AimFOV.Value
				&& SDK::VisPosWorld(pLocal, pHealTarget, vLocalPos, vPos))
			{
				if (flHP <= flHPThreshold)
					bNeedHeal = true;
				if (flHP <= flCrit)
					bCriticalTarget = true;
				if (Vars::Aimbot::Healing::AutoArrowAnticipateDamage.Value)
				{
					float flBullet, flBlast, flFire; flBullet = flBlast = flFire = 0.f;
					GetDangers(pHealTarget, pWeapon->GetMedigunType() == MEDIGUN_RESIST, flBullet, flBlast, flFire);
					float flDanger = std::max({ flBullet, flBlast, flFire });
					if (flDanger >= Vars::Aimbot::Healing::AutoArrowDangerThreshold.Value)
						bCriticalTarget = bNeedHeal = true;
				}
			}
		}
	}
	for (auto pEntity : H::Entities.GetGroup(EGroupType::PLAYERS_TEAMMATES))
	{
		auto pTeammate = pEntity->As<CTFPlayer>();
		if (!pTeammate || pTeammate == pLocal || !pTeammate->IsAlive())
			continue;
		// already handled heal target above
		if (pWeapon->m_hHealingTarget().Get() == pTeammate)
			continue;

		// hp/priority gate
		int iMaxHP = std::max(1, pTeammate->GetMaxHealth());
		float flHP = 100.f * (float)pTeammate->m_iHealth() / (float)iMaxHP;
		if (flHP > flHPThreshold)
			continue;

		// crit override
		float flCrit = Vars::Aimbot::Healing::AutoArrowCriticalThreshold.Value;
		if (flHP <= flCrit)
			bCriticalTarget = true;
		if (Vars::Aimbot::Healing::HealPriority.Value == Vars::Aimbot::Healing::HealPriorityEnum::FriendsOnly
			&& !H::Entities.IsFriend(pTeammate->entindex()) && !H::Entities.InParty(pTeammate->entindex()))
			continue;

		float flFOVTo; Vec3 vPos, vAngleTo;
		if (!F::AimbotGlobal.PlayerBoneInFOV(pTeammate, vLocalPos, vLocalAngles, flFOVTo, vPos, vAngleTo))
			continue;
		// FOV gate
		if (flFOVTo > Vars::Aimbot::General::AimFOV.Value)
			continue;

		// LOS check
		if (!SDK::VisPosWorld(pLocal, pTeammate, vLocalPos, vPos))
			continue;

		bNeedHeal = true;

		// anticipate dmg
		if (Vars::Aimbot::Healing::AutoArrowAnticipateDamage.Value)
		{
			float flBullet, flBlast, flFire; flBullet = flBlast = flFire = 0.f;
			GetDangers(pTeammate, pWeapon->GetMedigunType() == MEDIGUN_RESIST, flBullet, flBlast, flFire);
			float flDanger = std::max({ flBullet, flBlast, flFire });
			// danger ≥ thr -> crit
			if (flDanger >= Vars::Aimbot::Healing::AutoArrowDangerThreshold.Value)
				bCriticalTarget = true;
		}
		break;
	}

	if (!bNeedHeal)
		return;

	// if vacc wants to swap/charge, allow critical override to pass, else block
	if (bVaccGuardBlock && !(bCriticalTarget && Vars::Aimbot::Healing::AutoArrowForceOnCritical.Value))
		return;

	// swap to cbow
	for (int i = 0; i < MAX_WEAPONS; i++)
	{
		auto pSwap = pLocal->GetWeaponFromSlot(i);
		if (!pSwap || pSwap == pWeapon || !pSwap->CanBeSelected())
			continue;
		if (pSwap->GetWeaponID() != TF_WEAPON_CROSSBOW)
			continue;
		if (!pSwap->HasPrimaryAmmoForShot())
			continue;

		// crit+force -> bypass cd
		if (!(bCriticalTarget && Vars::Aimbot::Healing::AutoArrowForceOnCritical.Value))
		{
			// cd guard
			if (I::GlobalVars->curtime < m_flNextArrowSwitch)
				break;
		}
		pCmd->weaponselect = pSwap->entindex();
		m_flNextArrowSwitch = I::GlobalVars->curtime + 0.2f; // tiny anti-flap
		break;
	}
}

void CAutoHeal::Run(CTFPlayer* pLocal, CTFWeaponBase* pWeapon, CUserCmd* pCmd)
{
	if (pWeapon->GetWeaponID() != TF_WEAPON_MEDIGUN)
	{
		m_mMedicCallers.clear();
		m_iResistType = -1;
		m_flDamagedTime = 0.f;
		return;
	}

	ActivateOnVoice(pLocal, pWeapon->As<CWeaponMedigun>(), pCmd);
	m_mMedicCallers.clear();
	
	AutoVaccinator(pLocal, pWeapon->As<CWeaponMedigun>(), pCmd);
	AutoCbowHealSwitch(pLocal, pWeapon->As<CWeaponMedigun>(), pCmd);
}

void CAutoHeal::Event(IGameEvent* pEvent, uint32_t uHash)
{
	switch (uHash)
	{
	case FNV1A::Hash32Const("player_hurt"):
	{
		if (!Vars::Aimbot::Healing::AutoVaccinator.Value)
			return;

		auto pWeapon = H::Entities.GetWeapon()->As<CWeaponMedigun>();
		if (!pWeapon
			|| pWeapon->GetWeaponID() != TF_WEAPON_MEDIGUN || pWeapon->GetMedigunType() != MEDIGUN_RESIST)
			return;

		int iVictim = I::EngineClient->GetPlayerForUserID(pEvent->GetInt("userid"));
		int iAttacker = I::EngineClient->GetPlayerForUserID(pEvent->GetInt("attacker"));
		int iDamage = pEvent->GetInt("damageamount");
		int iWeaponID = pEvent->GetInt("weaponid");
		bool bCrit = pEvent->GetBool("crit");
		bool bMinicrit = pEvent->GetBool("minicrit");

		int iTarget = pWeapon->m_hHealingTarget().GetEntryIndex();
		if (iVictim == iAttacker || iVictim != I::EngineClient->GetLocalPlayer()
			&& (iVictim != iTarget
				|| Vars::Aimbot::Healing::HealPriority.Value == Vars::Aimbot::Healing::HealPriorityEnum::FriendsOnly
				&& !H::Entities.IsFriend(iTarget) && !H::Entities.InParty(iTarget)))
			return;

		auto pEntity = I::ClientEntityList->GetClientEntity(iAttacker)->As<CTFPlayer>();
		if (!pEntity || !pEntity->IsPlayer())
			return;

		auto pWeapon2 = pEntity->m_hActiveWeapon()->As<CTFWeaponBase>();
		if (!pWeapon2 || pWeapon2->GetWeaponID() != iWeaponID || pWeapon2->GetFireRate() > 1.f)
			return;

		float flFireRate = pWeapon2->GetFireRate();
		float flMult = SDK::AttribHookValue(1, "mult_dmg", pWeapon2);
		switch (SDK::GetWeaponType(pWeapon2))
		{
		case EWeaponType::HITSCAN:
			m_iDamagedType = MEDIGUN_BULLET_RESIST;
			m_flDamagedDPS = Math::Lerp(m_flDamagedDPS, iDamage / std::max(flFireRate, TICK_INTERVAL), 0.25f);
			break;
		case EWeaponType::PROJECTILE:
			switch (iWeaponID)
			{
			case TF_WEAPON_FLAMETHROWER:
				if (!bCrit && !bMinicrit || iDamage / flMult < (bCrit ? 48 : 22))
				{
					m_iDamagedType = MEDIGUN_FIRE_RESIST;
					float flBurnMult = SDK::AttribHookValue(1, "mult_wpn_burndmg", pWeapon2);
					float flSample = (!bCrit && !bMinicrit && iDamage <= 4 * flBurnMult)
						? (Vars::Aimbot::Healing::AutoVaccinatorFlamethrowerDamageOnly.Value ? 0.f : 8.f * flBurnMult)
						: 80.f * flMult * (bCrit ? 3.f : bMinicrit ? 1.36f : 1.f);
					m_flDamagedDPS = Math::Lerp(m_flDamagedDPS, flSample, 0.25f);
				}
				else // better way to assume reflect?
				{
					m_iDamagedType = MEDIGUN_BLAST_RESIST;
					m_flDamagedDPS = Math::Lerp(m_flDamagedDPS, static_cast<float>(iDamage), 0.25f);
				}
				break;
			case TF_WEAPON_COMPOUND_BOW:
			case TF_WEAPON_CROSSBOW:
			case TF_WEAPON_SHOTGUN_BUILDING_RESCUE:
			case TF_WEAPON_SYRINGEGUN_MEDIC:
			case TF_WEAPON_RAYGUN:
				m_iDamagedType = MEDIGUN_BULLET_RESIST;
				m_flDamagedDPS = Math::Lerp(m_flDamagedDPS, iDamage / std::max(flFireRate, TICK_INTERVAL), 0.25f);
				break;
			case TF_WEAPON_FLAME_BALL:
			case TF_WEAPON_FLAREGUN:
			case TF_WEAPON_FLAREGUN_REVENGE:
				m_iDamagedType = MEDIGUN_FIRE_RESIST;
				m_flDamagedDPS = Math::Lerp(m_flDamagedDPS, iDamage / std::max(flFireRate, TICK_INTERVAL), 0.25f);
				break;
			default:
				m_iDamagedType = MEDIGUN_BLAST_RESIST;
				m_flDamagedDPS = Math::Lerp(m_flDamagedDPS, iDamage / std::max(flFireRate, TICK_INTERVAL), 0.25f);
			}
			break;
		case EWeaponType::MELEE:
			return;
		}
		if (Vars::Aimbot::Healing::AutoVaccinatorFlamethrowerDamageOnly.Value && (iWeaponID != TF_WEAPON_FLAMETHROWER || m_iDamagedType != MEDIGUN_FIRE_RESIST))
			m_flDamagedDPS = 0.f;
		if (!m_flDamagedDPS)
			return;

		m_flDamagedTime = 1.f;
#ifdef DEBUG_VACCINATOR
		SDK::Output("Hurt", std::format("{}, {}", m_iDamagedType, m_flDamagedDPS).c_str(), { 255, 100, 100 });
#endif
		break;
	}
	case FNV1A::Hash32Const("player_spawn"):
		m_flDamagedTime = 0.f;
	}
}

#ifdef DEBUG_VACCINATOR
void CAutoHeal::Draw(CTFPlayer* pLocal)
{
	auto pWeapon = H::Entities.GetWeapon()->As<CWeaponMedigun>();
	if (!pWeapon || pWeapon->GetWeaponID() != TF_WEAPON_MEDIGUN
		|| !Vars::Aimbot::Healing::AutoVaccinator.Value || pWeapon->GetMedigunType() != MEDIGUN_RESIST)
		return;

	int x = H::Draw.m_nScreenW / 2, y = 100;
	const auto& fFont = H::Fonts.GetFont(FONT_INDICATORS);
	const int nTall = fFont.m_nTall + H::Draw.Scale(1);
	y -= nTall;

	for (auto& [flDanger, iResist] : vResistDangers)
		H::Draw.StringOutlined(fFont, x, y += nTall, Vars::Menu::Theme::Active.Value, Vars::Menu::Theme::Background.Value, ALIGN_TOP, std::format("{}: {:.3f}", iResist == MEDIGUN_BULLET_RESIST ? "Bullet" : iResist == MEDIGUN_BLAST_RESIST ? "Blast" : "Fire", flDanger).c_str());
}
#endif