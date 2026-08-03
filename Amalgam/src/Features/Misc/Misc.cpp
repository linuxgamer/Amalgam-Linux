#include "Misc.h"

#include "../Backtrack/Backtrack.h"
#include "../Ticks/Ticks.h"
#include "../Players/PlayerUtils.h"
#include "../Aimbot/AutoRocketJump/AutoRocketJump.h"

#include <format>
#include "../EnginePrediction/EnginePrediction.h"

void CMisc::RunPre(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	CheatsBypass();
	WeaponSway();
	AntiAFK(pLocal, pCmd);
	InstantRespawnMVM(pLocal);
	NoisemakerSpam(pLocal);

	// Save flags before prediction for LongJump/MiniJump
	m_iPrePredictionFlags = pLocal->m_fFlags();

	if (!pLocal->IsAlive() || pLocal->IsAGhost() || pLocal->m_MoveType() != MOVETYPE_WALK || pLocal->IsSwimming()
		|| pLocal->IsTaunting() || pLocal->InCond(TF_COND_HALLOWEEN_KART) || pLocal->InCond(TF_COND_SHIELD_CHARGE))
		return;

	EdgeBugPrePrediction(pLocal, pCmd);
	AutoJump(pLocal, pCmd);
	EdgeJump(pLocal, pCmd);
	AutoJumpbug(pLocal, pCmd);
	AutoStrafe(pLocal, pCmd);
	AutoPeek(pLocal, pCmd);
	MovementLock(pLocal, pCmd);
	BreakJump(pLocal, pCmd);
}

void CMisc::RunPost(CTFPlayer* pLocal, CUserCmd* pCmd, bool pSendPacket)
{
	if (!pLocal->IsAlive() || pLocal->IsAGhost() || pLocal->m_MoveType() != MOVETYPE_WALK || pLocal->IsSwimming()
		|| pLocal->InCond(TF_COND_SHIELD_CHARGE))
		return;

	if (pLocal->IsTaunting() || pLocal->InCond(TF_COND_HALLOWEEN_KART))
		TauntKartControl(pLocal, pCmd);
	else
	{
		EdgeJump(pLocal, pCmd, true);
		AutoPeek(pLocal, pCmd, true);
		FastMovement(pLocal, pCmd);
		LongJump(pLocal, pCmd);
		MiniJump(pLocal, pCmd);
		AutoAlign(pLocal, pCmd);
		PixelSurf(pLocal, pCmd);
	}
}

void CMisc::AutoJump(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	if (!Vars::Misc::Movement::Bunnyhop.Value)
		return;

	if (auto pWeapon = H::Entities.GetWeapon(); pWeapon && pWeapon->GetWeaponID() == TF_WEAPON_GRAPPLINGHOOK && pWeapon->As<CTFGrapplingHook>()->m_hProjectile())
		return;

	static bool bStaticJump = false, bStaticGrounded = false, bLastAttempted = false;
	const bool bLastJump = bStaticJump, bLastGrounded = bStaticGrounded;
	const bool bCurJump = bStaticJump = pCmd->buttons & IN_JUMP, bCurGrounded = bStaticGrounded = pLocal->m_hGroundEntity();

	if (bCurJump && bLastJump && (bCurGrounded ? !pLocal->IsDucking() : true))
	{
		if (!(bCurGrounded && !bLastGrounded))
			pCmd->buttons &= ~IN_JUMP;

		if (!(pCmd->buttons & IN_JUMP) && bCurGrounded && !bLastAttempted)
			pCmd->buttons |= IN_JUMP;
	}

	if (Vars::Misc::Game::AntiCheatCompatibility.Value)
	{	// prevent more than 9 bhops occurring. if a server has this under that threshold they're retarded anyways
		static int iJumps = 0;
		if (bCurGrounded)
		{
			if (!bLastGrounded && pCmd->buttons & IN_JUMP)
				iJumps++;
			else
				iJumps = 0;

			if (iJumps > 9)
				pCmd->buttons &= ~IN_JUMP;
		}
	}
	bLastAttempted = pCmd->buttons & IN_JUMP;
}

void CMisc::AutoJumpbug(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	if (!Vars::Misc::Movement::AutoJumpbug.Value || !(pCmd->buttons & IN_DUCK) || pLocal->m_hGroundEntity() || pLocal->m_vecVelocity().z > -650.f)
		return;

	float flUnduckHeight = 20 * pLocal->m_flModelScale();
	float flTraceDistance = flUnduckHeight + 2;

	CGameTrace trace = {};
	CTraceFilterWorldAndPropsOnly filter = {};

	Vec3 vOrigin = pLocal->m_vecOrigin();
	SDK::TraceHull(vOrigin, vOrigin - Vec3(0, 0, flTraceDistance), pLocal->m_vecMins(), pLocal->m_vecMaxs(), pLocal->SolidMask(), &filter, &trace);
	if (!trace.DidHit() || trace.fraction * flTraceDistance < flUnduckHeight) // don't try if we aren't in range to unduck or are too low
		return;

	pCmd->buttons &= ~IN_DUCK;
	pCmd->buttons |= IN_JUMP;
}

void CMisc::AutoStrafe(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	if (!Vars::Misc::Movement::AutoStrafe.Value || pLocal->m_hGroundEntity() || !(pLocal->m_afButtonLast() & IN_JUMP) && (pCmd->buttons & IN_JUMP))
		return;

	switch (Vars::Misc::Movement::AutoStrafe.Value)
	{
	case Vars::Misc::Movement::AutoStrafeEnum::Legit:
	{
		static auto cl_sidespeed = H::ConVars.FindVar("cl_sidespeed");
		const float flSideSpeed = cl_sidespeed->GetFloat();

		if (pCmd->mousedx)
		{
			pCmd->forwardmove = 0.f;
			pCmd->sidemove = pCmd->mousedx > 0 ? flSideSpeed : -flSideSpeed;
		}
		break;
	}
	case Vars::Misc::Movement::AutoStrafeEnum::Directional:
	{
		// credits: KGB
		if (!(pCmd->buttons & (IN_FORWARD | IN_BACK | IN_MOVELEFT | IN_MOVERIGHT)))
			break;

		float flForward = pCmd->forwardmove, flSide = pCmd->sidemove;

		Vec3 vForward, vRight; Math::AngleVectors(pCmd->viewangles, &vForward, &vRight, nullptr);
		vForward.Normalize2D(), vRight.Normalize2D();

		Vec3 vWishDir = Math::VectorAngles({ vForward.x * flForward + vRight.x * flSide, vForward.y * flForward + vRight.y * flSide, 0.f });
		Vec3 vCurDir = Math::VectorAngles(pLocal->m_vecVelocity());
		float flDirDelta = Math::NormalizeAngle(vWishDir.y - vCurDir.y);
		if (fabsf(flDirDelta) > Vars::Misc::Movement::AutoStrafeMaxDelta.Value)
			break;

		float flTurnScale = Math::RemapVal(Vars::Misc::Movement::AutoStrafeTurnScale.Value, 0.f, 1.f, 0.9f, 1.f);
		float flRotation = DEG2RAD((flDirDelta > 0.f ? -90.f : 90.f) + flDirDelta * flTurnScale);
		float flCosRot = cosf(flRotation), flSinRot = sinf(flRotation);

		pCmd->forwardmove = flCosRot * flForward - flSinRot * flSide;
		pCmd->sidemove = flSinRot * flForward + flCosRot * flSide;
	}
	}
}

void CMisc::MovementLock(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	static bool bLock = false;

	if (!Vars::Misc::Movement::MovementLock.Value)
	{
		bLock = false;
		return;
	}

	static Vec3 vMove = {}, vView = {};
	if (!bLock)
	{
		bLock = true;
		vMove = { pCmd->forwardmove, pCmd->sidemove, pCmd->upmove };
		vView = pCmd->viewangles;
	}

	pCmd->forwardmove = vMove.x, pCmd->sidemove = vMove.y, pCmd->upmove = vMove.z;
	SDK::FixMovement(pCmd, vView, pCmd->viewangles);
}

void CMisc::BreakJump(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	if (!Vars::Misc::Movement::BreakJump.Value || F::AutoRocketJump.IsRunning())
		return;

	static bool bStaticJump = false;
	const bool bLastJump = bStaticJump;
	const bool bCurrJump = bStaticJump = pCmd->buttons & IN_JUMP;

	static int iTickSinceGrounded = -1;
	if (pLocal->m_hGroundEntity().Get())
		iTickSinceGrounded = -1;
	iTickSinceGrounded++;

	switch (iTickSinceGrounded)
	{
	case 0:
		if (bLastJump || !bCurrJump || pLocal->IsDucking())
			return;
		break;
	case 1:
		break;
	default:
		return;
	}

	pCmd->buttons |= IN_DUCK;
}

void CMisc::AntiAFK(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	static Timer tTimer = {};

	if (pCmd->buttons & (IN_MOVELEFT | IN_MOVERIGHT | IN_FORWARD | IN_BACK) || !pLocal->IsAlive())
		tTimer.Update();
	else if (Vars::Misc::Automation::AntiAFK.Value && tTimer.Run(25.f))
		pCmd->buttons |= IN_FORWARD;
}

void CMisc::InstantRespawnMVM(CTFPlayer* pLocal)
{
	if (!Vars::Misc::MannVsMachine::InstantRespawn.Value || pLocal->IsAlive())
		return;

	KeyValues* kv = new KeyValues("MVM_Revive_Response");
	kv->SetBool("accepted", true);
	I::EngineClient->ServerCmdKeyValues(kv);
}

void CMisc::NoisemakerSpam(CTFPlayer* pLocal)
{
	if (!Vars::Misc::Exploits::NoisemakerSpam.Value || !pLocal->IsAlive() || pLocal->IsAGhost()
		|| pLocal->m_bUsingActionSlot() || pLocal->m_flNextNoiseMakerTime() > I::GlobalVars->curtime)
		return;

	KeyValues* kv = new KeyValues("use_action_slot_item_server");
	I::EngineClient->ServerCmdKeyValues(kv);
}

void CMisc::CheatsBypass()
{
	static bool bCheatSet = false;
	static auto sv_cheats = H::ConVars.FindVar("sv_cheats");
	if (Vars::Misc::Exploits::CheatsBypass.Value)
	{
		sv_cheats->m_nValue = 1;
		bCheatSet = true;
	}
	else if (bCheatSet)
	{
		sv_cheats->m_nValue = 0;
		bCheatSet = false;
	}
}

void CMisc::WeaponSway()
{
	static auto cl_wpn_sway_interp = H::ConVars.FindVar("cl_wpn_sway_interp");
	static auto cl_wpn_sway_scale = H::ConVars.FindVar("cl_wpn_sway_scale");

	bool bSway = Vars::Visuals::Viewmodel::SwayInterp.Value || Vars::Visuals::Viewmodel::SwayScale.Value;
	cl_wpn_sway_interp->SetValue(bSway ? Vars::Visuals::Viewmodel::SwayInterp.Value : 0.f);
	cl_wpn_sway_scale->SetValue(bSway ? Vars::Visuals::Viewmodel::SwayScale.Value : 0.f);
}



void CMisc::TauntKartControl(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	if (Vars::Misc::Automation::TauntControl.Value && pLocal->IsTaunting() && pLocal->m_bAllowMoveDuringTaunt())
	{
		if (pLocal->m_bTauntForceMoveForward())
		{
			if (pCmd->buttons & IN_BACK)
				pCmd->viewangles.x = 91.f;
			else if (!(pCmd->buttons & IN_FORWARD))
				pCmd->viewangles.x = 90.f;
		}
		if (pCmd->buttons & IN_MOVELEFT)
			pCmd->sidemove = pCmd->viewangles.x == 90.f ? -450.f : -pLocal->m_flTauntForceMoveForwardSpeed();
		else if (pCmd->buttons & IN_MOVERIGHT)
			pCmd->sidemove = pCmd->viewangles.x == 90.f ? 450.f : pLocal->m_flTauntForceMoveForwardSpeed();
	}
	else if (Vars::Misc::Automation::KartControl.Value && pLocal->InCond(TF_COND_HALLOWEEN_KART))
	{
		const bool bForward = pCmd->buttons & IN_FORWARD;
		const bool bBack = pCmd->buttons & IN_BACK;
		const bool bLeft = pCmd->buttons & IN_MOVELEFT;
		const bool bRight = pCmd->buttons & IN_MOVERIGHT;

		const bool flipVar = I::GlobalVars->tickcount % 2;
		if (bForward && (!bLeft && !bRight || !flipVar))
		{
			pCmd->forwardmove = 450.f;
			pCmd->viewangles.x = 0.f;
		}
		else if (bBack && (!bLeft && !bRight || !flipVar))
		{
			pCmd->forwardmove = 450.f;
			pCmd->viewangles.x = 91.f;
		}
		else if (pCmd->buttons & (IN_FORWARD | IN_BACK | IN_MOVELEFT | IN_MOVERIGHT))
		{
			if (flipVar || !F::Ticks.CanChoke())
			{	// you could just do this if you didn't care about viewangles
				Vec3 vMove = { pCmd->forwardmove, pCmd->sidemove, 0.f };
				Vec3 vAngMoveReverse = Math::VectorAngles(vMove * -1.f);
				pCmd->forwardmove = -vMove.Length();
				pCmd->sidemove = 0.f;
				pCmd->viewangles.y = fmodf(pCmd->viewangles.y - vAngMoveReverse.y, 360.f);
				pCmd->viewangles.z = 270.f;
				G::PSilentAngles = true;
			}
		}
		else
			pCmd->viewangles.x = 90.f;

		G::SilentAngles = true;
	}
}

void CMisc::FastMovement(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	if (!pLocal->m_hGroundEntity() || pLocal->InCond(TF_COND_HALLOWEEN_KART))
		return;

	const float flSpeed = pLocal->m_vecVelocity().Length2D();
	const int flMaxSpeed = std::min(pLocal->m_flMaxspeed() * 0.9f, 520.f) - 10.f;
	const int iRun = !pCmd->forwardmove && !pCmd->sidemove ? 0 : flSpeed < flMaxSpeed ? 1 : 2;

	switch (iRun)
	{
	case 0:
	{
		if (!Vars::Misc::Movement::FastStop.Value || !flSpeed)
			return;

		Vec3 vDirection = pLocal->m_vecVelocity().ToAngle();
		vDirection.y = pCmd->viewangles.y - vDirection.y;
		Vec3 vNegatedDirection = vDirection.FromAngle() * -flSpeed;
		pCmd->forwardmove = vNegatedDirection.x;
		pCmd->sidemove = vNegatedDirection.y;

		break;
	}
	case 1:
	{
		if ((pLocal->IsDucking() ? !Vars::Misc::Movement::DuckSpeed.Value : !Vars::Misc::Movement::FastAccelerate.Value)
			|| Vars::Misc::Game::AntiCheatCompatibility.Value
			|| G::Attacking == 1 || F::Ticks.m_bDoubletap || F::Ticks.m_bSpeedhack || F::Ticks.m_bRecharge || G::AntiAim || I::GlobalVars->tickcount % 2)
			return;

		if (!(pCmd->buttons & (IN_FORWARD | IN_BACK | IN_MOVELEFT | IN_MOVERIGHT)))
			return;

		Vec3 vMove = { pCmd->forwardmove, pCmd->sidemove, 0.f };
		Vec3 vAngMoveReverse = Math::VectorAngles(vMove * -1.f);
		pCmd->forwardmove = -vMove.Length();
		pCmd->sidemove = 0.f;
		pCmd->viewangles.y = fmodf(pCmd->viewangles.y - vAngMoveReverse.y, 360.f);
		pCmd->viewangles.z = 270.f;
		G::PSilentAngles = true;

		break;
	}
	}
}

void CMisc::AutoPeek(CTFPlayer* pLocal, CUserCmd* pCmd, bool bPost)
{
	static bool bReturning = false;

	if (!bPost)
	{
		if (Vars::AutoPeek::Enabled.Value)
		{
			Vec3 vLocalPos = pLocal->m_vecOrigin();

			if (bReturning)
			{
				if (vLocalPos.DistTo2D(m_vPeekReturnPos) < 8.f)
				{
					bReturning = false;
					return;
				}

				SDK::WalkTo(pCmd, pLocal, m_vPeekReturnPos);
				pCmd->buttons &= ~IN_JUMP;
			}
			else if (!pLocal->m_hGroundEntity())
				m_bPeekPlaced = false;

			if (!m_bPeekPlaced)
			{
				m_vPeekReturnPos = vLocalPos;
				m_bPeekPlaced = true;
			}
			else
			{
				static Timer tTimer = {};
				if (tTimer.Run(0.7f))
					H::Particles.DispatchParticleEffect("ping_circle", m_vPeekReturnPos, {});
			}
		}
		else
			m_bPeekPlaced = bReturning = false;
	}
	else if (G::Attacking && m_bPeekPlaced)
		bReturning = true;
}

void CMisc::EdgeJump(CTFPlayer* pLocal, CUserCmd* pCmd, bool bPost)
{
	if (!Vars::Misc::Movement::EdgeJump.Value)
		return;

	static bool bStaticGround = false;
	if (!bPost)
		bStaticGround = pLocal->m_hGroundEntity();
	else if (bStaticGround && !pLocal->m_hGroundEntity())
		pCmd->buttons |= IN_JUMP;
}



void CMisc::Event(IGameEvent* pEvent, uint32_t uHash)
{
	switch (uHash)
	{
	case FNV1A::Hash32Const("player_spawn"):
		m_bPeekPlaced = false;
	}
}

int CMisc::AntiBackstab(CTFPlayer* pLocal, CUserCmd* pCmd, bool bSendPacket)
{
	if (!Vars::Misc::Automation::AntiBackstab.Value || !bSendPacket || G::Attacking == 1 || !pLocal || pLocal->m_MoveType() != MOVETYPE_WALK || pLocal->InCond(TF_COND_HALLOWEEN_KART))
		return 0;

	std::vector<std::pair<Vec3, CBaseEntity*>> vTargets = {};
	for (auto pEntity : H::Entities.GetGroup(EntityEnum::PlayerEnemy))
	{
		auto pPlayer = pEntity->As<CTFPlayer>();
		if (!pPlayer->IsAlive() || pPlayer->IsAGhost() || pPlayer->InCond(TF_COND_STEALTHED))
			continue;

		auto pWeapon = pPlayer->m_hActiveWeapon()->As<CTFWeaponBase>();
		if (!pWeapon
			|| pWeapon->GetWeaponID() != TF_WEAPON_KNIFE
			&& !(G::PrimaryWeaponType == EWeaponType::MELEE && SDK::AttribHookValue(0, "crit_from_behind", pWeapon) > 0)
			&& !(pWeapon->GetWeaponID() == TF_WEAPON_FLAMETHROWER && SDK::AttribHookValue(0, "set_flamethrower_back_crit", pWeapon) == 1)
			|| F::PlayerUtils.IsIgnored(pPlayer->entindex()))
			continue;

		Vec3 vLocalPos = pLocal->GetCenter();
		Vec3 vTargetPos1 = pPlayer->GetCenter();
		Vec3 vTargetPos2 = vTargetPos1 + pPlayer->m_vecVelocity() * F::Backtrack.GetReal();
		float flDistance = std::max(std::max(SDK::MaxSpeed(pPlayer), SDK::MaxSpeed(pLocal)), pPlayer->m_vecVelocity().Length());
		if ((vLocalPos.DistTo(vTargetPos1) > flDistance || !SDK::VisPosWorld(pLocal, pPlayer, vLocalPos, vTargetPos1))
			&& (vLocalPos.DistTo(vTargetPos2) > flDistance || !SDK::VisPosWorld(pLocal, pPlayer, vLocalPos, vTargetPos2)))
			continue;

		vTargets.emplace_back(vTargetPos2, pEntity);
	}
	if (vTargets.empty())
		return 0;

	std::sort(vTargets.begin(), vTargets.end(), [&](const auto& a, const auto& b) -> bool
		{
			return pLocal->GetCenter().DistTo(a.first) < pLocal->GetCenter().DistTo(b.first);
		});

	auto& pTargetPos = vTargets.front();
	switch (Vars::Misc::Automation::AntiBackstab.Value)
	{
	case Vars::Misc::Automation::AntiBackstabEnum::Yaw:
	{
		Vec3 vAngleTo = Math::CalcAngle(pLocal->m_vecOrigin(), pTargetPos.first);
		vAngleTo.x = pCmd->viewangles.x;
		SDK::FixMovement(pCmd, vAngleTo);
		pCmd->viewangles = vAngleTo;

		return 1;
	}
	case Vars::Misc::Automation::AntiBackstabEnum::Pitch:
	case Vars::Misc::Automation::AntiBackstabEnum::Fake:
	{
		bool bCheater = F::PlayerUtils.HasTag(pTargetPos.second->entindex(), F::PlayerUtils.TagToIndex(CHEATER_TAG));
		// if the closest spy is a cheater, assume auto stab is being used, otherwise don't do anything if target is in front
		if (!bCheater)
		{
			auto TargetIsBehind = [&]()
				{
					const float flCompDist = 0.0625f;
					const float flSqCompDist = 0.0884f;

					Vec3 vToTarget = (pLocal->m_vecOrigin() - pTargetPos.first).To2D();
					const float flDist = vToTarget.Normalize();
					if (flDist < flSqCompDist)
						return true;

					const float flExtra = 2.f * flCompDist / flDist; // account for origin compression
					float flPosVsTargetViewMinDot = 0.f - 0.0031f - flExtra;

					Vec3 vTargetForward; Math::AngleVectors(pCmd->viewangles, &vTargetForward);
					vTargetForward.Normalize2D();

					const float flPosVsTargetViewDot = vToTarget.Dot(vTargetForward); // Behind?

					return flPosVsTargetViewDot > flPosVsTargetViewMinDot;
				};

			if (!TargetIsBehind())
				return 0;
		}

		if (!bCheater || Vars::Misc::Automation::AntiBackstab.Value == Vars::Misc::Automation::AntiBackstabEnum::Pitch)
		{
			pCmd->forwardmove *= -1;
			pCmd->viewangles.x = 269.f;
		}
		else
		{
			pCmd->viewangles.x = 271.f;
		}
		// may slip up some auto backstabs depending on mode, though we are still able to be stabbed

		return 2;
	}
	}

	return 0;
}

void CMisc::PingReducer()
{
	static Timer tTimer = {};
	if (!tTimer.Run(0.1f))
		return;

	static auto cl_cmdrate = H::ConVars.FindVar("cl_cmdrate");
	int iTarget = Vars::Misc::Exploits::PingReducer.Value ? Vars::Misc::Exploits::PingTarget.Value : cl_cmdrate->GetInt();
	if (m_iWishCmdrate != iTarget)
	{
		m_iWishCmdrate = iTarget;

		auto pNetChan = reinterpret_cast<CNetChannel*>(I::EngineClient->GetNetChannelInfo());
		if (pNetChan && I::EngineClient->IsConnected())
		{
			NET_SetConVar tConvar = { "cl_cmdrate", std::to_string(m_iWishCmdrate).c_str() };
			pNetChan->SendNetMsg(tConvar);
		}
	}

	static auto sv_maxupdaterate = H::ConVars.FindVar("sv_maxupdaterate"); // force highest cl_updaterate command possible
	iTarget = sv_maxupdaterate->GetInt();
	if (m_iWishUpdaterate != iTarget)
	{
		m_iWishUpdaterate = iTarget;

		auto pNetChan = reinterpret_cast<CNetChannel*>(I::EngineClient->GetNetChannelInfo());
		if (pNetChan && I::EngineClient->IsConnected())
		{
			NET_SetConVar tConvar = { "cl_updaterate", std::to_string(m_iWishUpdaterate).c_str() };
			pNetChan->SendNetMsg(tConvar);
		}
	}
}

void CMisc::UnlockAchievements()
{
	const auto pAchievementMgr = U::Memory.CallVirtual<114, IAchievementMgr*>(I::EngineClient);
	if (pAchievementMgr)
	{
		I::SteamUserStats->RequestCurrentStats();
		for (int i = 0; i < pAchievementMgr->GetAchievementCount(); i++)
			pAchievementMgr->AwardAchievement(pAchievementMgr->GetAchievementByIndex(i)->GetAchievementID());
		I::SteamUserStats->StoreStats();
		I::SteamUserStats->RequestCurrentStats();
	}
}

void CMisc::LockAchievements()
{
	const auto pAchievementMgr = U::Memory.CallVirtual<114, IAchievementMgr*>(I::EngineClient);
	if (pAchievementMgr)
	{
		I::SteamUserStats->RequestCurrentStats();
		for (int i = 0; i < pAchievementMgr->GetAchievementCount(); i++)
			I::SteamUserStats->ClearAchievement(pAchievementMgr->GetAchievementByIndex(i)->GetName());
		I::SteamUserStats->StoreStats();
		I::SteamUserStats->RequestCurrentStats();
	}
}


// EdgeBug
void CMisc::RestoreEntityToPredicted()
{
	I::Prediction->RestoreEntityToPredictedFrame(I::Prediction->m_nCommandsPredicted - 1);
}

void CMisc::EdgeBugPrePrediction(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	if (!Vars::Misc::Movement::EdgeBug.Value)
	{
		m_bEdgeBugDetected = false;
		m_iEdgeBugLockTicks = 0;
		return;
	}

	// Save state before prediction
	m_vEdgeBugVelocityBackup = pLocal->m_vecVelocity();
	m_iEdgeBugFlags = pLocal->m_fFlags();
}

bool CMisc::EdgeBugCheck(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	if (!pLocal || !pLocal->IsAlive() || !I::EngineClient->IsInGame() || !I::EngineClient->IsConnected())
		return false;

	if (pLocal->m_MoveType() == MOVETYPE_LADDER || pLocal->m_MoveType() == MOVETYPE_NOCLIP)
		return false;

	static auto sv_gravity = H::ConVars.FindVar("sv_gravity");
	if (!sv_gravity)
		return false;

	float flGravity = sv_gravity->GetFloat();
	float flGravityVel = flGravity * 0.5f * TICK_INTERVAL;

	Vec3 vCurrentVelocity = pLocal->m_vecVelocity();

	// Check 1: Was falling fast and velocity reset to first-tick gravity value
	if (m_vEdgeBugVelocityBackup.z < -flGravityVel &&
		roundf(vCurrentVelocity.z) == -roundf(flGravityVel))
	{
		return true;
	}

	// Check 2: Was falling and velocity increased but still negative (edge scrape)
	if (m_vEdgeBugVelocityBackup.z < -6.0f &&
		vCurrentVelocity.z > m_vEdgeBugVelocityBackup.z &&
		vCurrentVelocity.z < -6.0f)
	{
		float flVelocityBeforePrediction = vCurrentVelocity.z;

		// Run one more prediction tick to verify
		F::EnginePrediction.Simulate(pLocal, pCmd);

		float flGravityVelocityConstant = roundf(-flGravity * TICK_INTERVAL + flVelocityBeforePrediction);

		if (flGravityVelocityConstant == roundf(pLocal->m_vecVelocity().z))
		{
			// Additional check: verify we're near a surface
			CGameTrace trace = {};
			CTraceFilterWorldAndPropsOnly filter = {};
			Vec3 vOrigin = pLocal->m_vecOrigin();
			vOrigin.z += 200.f; // Check above player

			// Radial trace to find nearby surfaces
			const float flStep = PI * 2.f / 16.f;
			for (float a = 0; a < PI * 2.f; a += flStep)
			{
				Vec3 vStart(32.f * cosf(a) + vOrigin.x, 32.f * sinf(a) + vOrigin.y, vOrigin.z);
				Vec3 vEnd = vStart - Vec3(0, 0, 300);

				SDK::Trace(vStart, vEnd, MASK_PLAYERSOLID, &filter, &trace);

				if (trace.fraction != 1.f && trace.plane.normal.z < 0.6f)
				{
					return true;
				}
			}
		}
	}

	return false;
}

// Movement correction for silent edgebug
void CMisc::CorrectMovement(CUserCmd* pCmd, Vec3 vWishAngle, Vec3 vOldAngles)
{
	if (vOldAngles.x == vWishAngle.x && vOldAngles.y == vWishAngle.y && vOldAngles.z == vWishAngle.z)
		return;

	Vec3 vWishForward, vWishRight, vWishUp;
	Vec3 vCmdForward, vCmdRight, vCmdUp;

	Vec3 vMoveData(pCmd->forwardmove, pCmd->sidemove, pCmd->upmove);

	Math::AngleVectors(vWishAngle, &vWishForward, &vWishRight, &vWishUp);
	Math::AngleVectors(vOldAngles, &vCmdForward, &vCmdRight, &vCmdUp);

	// Normalize
	float flWishForwardLen = sqrtf(vWishForward.x * vWishForward.x + vWishForward.y * vWishForward.y);
	float flWishRightLen = sqrtf(vWishRight.x * vWishRight.x + vWishRight.y * vWishRight.y);
	float flWishUpLen = sqrtf(vWishUp.z * vWishUp.z);

	Vec3 vWishForwardNorm(vWishForward.x / flWishForwardLen, vWishForward.y / flWishForwardLen, 0.f);
	Vec3 vWishRightNorm(vWishRight.x / flWishRightLen, vWishRight.y / flWishRightLen, 0.f);
	Vec3 vWishUpNorm(0.f, 0.f, vWishUp.z / flWishUpLen);

	float flCmdForwardLen = sqrtf(vCmdForward.x * vCmdForward.x + vCmdForward.y * vCmdForward.y);
	float flCmdRightLen = sqrtf(vCmdRight.x * vCmdRight.x + vCmdRight.y * vCmdRight.y);
	float flCmdUpLen = sqrtf(vCmdUp.z * vCmdUp.z);

	Vec3 vCmdForwardNorm(vCmdForward.x / flCmdForwardLen, vCmdForward.y / flCmdForwardLen, 0.f);
	Vec3 vCmdRightNorm(vCmdRight.x / flCmdRightLen, vCmdRight.y / flCmdRightLen, 0.f);
	Vec3 vCmdUpNorm(0.f, 0.f, vCmdUp.z / flCmdUpLen);

	// Calculate corrected movement
	Vec3 vCorrectMove;
	vCorrectMove.x = vCmdForwardNorm.x * (vWishForwardNorm.x * vMoveData.x + vWishRightNorm.x * vMoveData.y) +
		vCmdForwardNorm.y * (vWishForwardNorm.y * vMoveData.x + vWishRightNorm.y * vMoveData.y);

	vCorrectMove.y = vCmdRightNorm.x * (vWishForwardNorm.x * vMoveData.x + vWishRightNorm.x * vMoveData.y) +
		vCmdRightNorm.y * (vWishForwardNorm.y * vMoveData.x + vWishRightNorm.y * vMoveData.y);

	vCorrectMove.z = vCmdUpNorm.z * vWishUpNorm.z * vMoveData.z;

	// Clamp
	vCorrectMove.x = std::clamp(vCorrectMove.x, -450.f, 450.f);
	vCorrectMove.y = std::clamp(vCorrectMove.y, -450.f, 450.f);
	vCorrectMove.z = std::clamp(vCorrectMove.z, -320.f, 320.f);

	pCmd->forwardmove = vCorrectMove.x;
	pCmd->sidemove = vCorrectMove.y;
	pCmd->upmove = vCorrectMove.z;
}

// Auto strafe for edgebug search
void CMisc::AutoStrafeEdgeBug(CUserCmd* pCmd, CTFPlayer* pLocal)
{
	static float flSide = 1.f;
	flSide = -flSide;

	Vec3 vVelocity = pLocal->m_vecVelocity();
	Vec3 vWishAngle = pCmd->viewangles;

	float flSpeed = vVelocity.Length2D();
	float flIdealStrafe = std::clamp(RAD2DEG(atanf(15.f / flSpeed)), 0.f, 90.f);

	pCmd->forwardmove = 0.f;

	static auto cl_sidespeed = H::ConVars.FindVar("cl_sidespeed");
	float flSideSpeed = cl_sidespeed ? cl_sidespeed->GetFloat() : 450.f;

	static float flOldYaw = 0.f;
	float flYawDelta = remainderf(vWishAngle.y - flOldYaw, 360.f);
	float flAbsYawDelta = fabsf(flYawDelta);
	flOldYaw = vWishAngle.y;

	if (flAbsYawDelta <= flIdealStrafe || flAbsYawDelta >= 30.f)
	{
		Vec3 vVelocityDir = Math::VectorAngles(vVelocity);
		float flVelocityDelta = remainderf(vWishAngle.y - vVelocityDir.y, 360.f);
		float flRetrack = std::clamp(RAD2DEG(atanf(30.f / flSpeed)), 0.f, 90.f) * 2.f;

		if (flVelocityDelta <= flRetrack || flSpeed <= 15.f)
		{
			if (-flRetrack <= flVelocityDelta || flSpeed <= 15.f)
			{
				vWishAngle.y += flSide * flIdealStrafe;
				pCmd->sidemove = flSideSpeed * flSide;
			}
			else
			{
				vWishAngle.y = vVelocityDir.y - flRetrack;
				pCmd->sidemove = flSideSpeed;
			}
		}
		else
		{
			vWishAngle.y = vVelocityDir.y + flRetrack;
			pCmd->sidemove = -flSideSpeed;
		}

		CorrectMovement(pCmd, vWishAngle, pCmd->viewangles);
	}
	else if (flYawDelta > 0.f)
		pCmd->sidemove = -flSideSpeed;
	else
		pCmd->sidemove = flSideSpeed;
}

void CMisc::EdgeBugPostPrediction(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	if (!Vars::Misc::Movement::EdgeBug.Value)
	{
		m_bEdgeBugDetected = false;
		m_iEdgeBugLockTicks = 0;
		m_iEdgeBugCurrentTick = 0;
		m_iEdgeBugSearchMode = 0;
		return;
	}

	// Don't run if on ground or going up
	if (m_iEdgeBugFlags & FL_ONGROUND || m_vEdgeBugVelocityBackup.z > 0.f)
	{
		m_bEdgeBugDetected = false;
		m_iEdgeBugLockTicks = 0;
		m_iEdgeBugCurrentTick = 0;
		m_iEdgeBugSearchMode = 0;
		return;
	}

	// Backup original command
	int iBackupButtons = pCmd->buttons;
	float flBackupForward = pCmd->forwardmove;
	float flBackupSide = pCmd->sidemove;
	Vec3 vBackupAngles = pCmd->viewangles;

	// Calculate smooth angle delta
	static Vec3 vLastAngle = vBackupAngles;
	Vec3 vAngleDelta = (vBackupAngles - vLastAngle);
	// Clamp angle delta to prevent huge jumps
	vAngleDelta.y = std::clamp(vAngleDelta.y, -(180.f / 128.f), 180.f / 128.f);
	vAngleDelta *= 0.5f;
	vLastAngle = vBackupAngles;

	// Static variables for strafe tracking
	static Vec3 vLastStrafeAngles = vBackupAngles;
	static float flLastStrafeForward = flBackupForward;
	static float flLastStrafeSide = flBackupSide;
	static bool bAppliedStrafeLast = false;

	if (!bAppliedStrafeLast)
	{
		// Update strafe data only if we didn't strafe last time
		vLastStrafeAngles = vBackupAngles;
		flLastStrafeForward = flBackupForward;
		flLastStrafeSide = flBackupSide;
	}
	bAppliedStrafeLast = false;

	// Advanced search - только если включен
	if (!m_bEdgeBugDetected && Vars::Misc::Movement::EdgeBugAdvancedSearch.Value)
	{
		static int iLastSuccessMode = 0;
		int iSearchModes = 6; // ������ advanced search

		for (int iMode = 0; iMode < iSearchModes; iMode++)
		{
			if (m_bEdgeBugDetected)
				break;

			// Try last successful mode first
			int iCurrentMode = iMode;
			if (iLastSuccessMode && iMode == 0)
			{
				iCurrentMode = iLastSuccessMode;
				iLastSuccessMode = 0;
			}

			// Skip duck modes if already ducking
			if ((iBackupButtons & IN_DUCK) && iCurrentMode < 2)
				continue;

			RestoreEntityToPredicted();

			// Reset to backup
			pCmd->viewangles = vLastStrafeAngles;
			pCmd->forwardmove = flLastStrafeForward;
			pCmd->sidemove = flLastStrafeSide;
			pCmd->buttons = iBackupButtons;

			Vec3 vCurrentAngle = vLastStrafeAngles;
			bool bApplyStrafe = !(iCurrentMode % 2); // modes 0, 2, 4
			bool bApplyDuck = iCurrentMode > 1;

			// Predict up to 64 ticks
			for (int iTick = 0; iTick < 64; iTick++)
			{
				if (m_bEdgeBugDetected || pLocal->m_fFlags() & FL_ONGROUND || pLocal->m_vecVelocity().z > 0.f)
					break;

				// Apply duck
				if (bApplyDuck)
					pCmd->buttons |= IN_DUCK;
				else
					pCmd->buttons &= ~IN_DUCK;

				// Apply movement based on mode
				if (iCurrentMode < 2)
				{
					// Modes 0-1: Still
					pCmd->forwardmove = 0.f;
					pCmd->sidemove = 0.f;
				}
				else if (iCurrentMode < 4)
				{
					// Modes 2-3: Movement with angle delta
					pCmd->forwardmove = flLastStrafeForward;
					pCmd->sidemove = flLastStrafeSide;

					if (bApplyStrafe && fabsf(vCurrentAngle.y - vBackupAngles.y) < 179.f)
					{
						vCurrentAngle = vCurrentAngle + vAngleDelta;
						Math::ClampAngles(vCurrentAngle);
						pCmd->viewangles = vCurrentAngle;
					}
				}
				else
				{
					// Modes 4-5: Autostrafe - только если включен
					if (Vars::Misc::Movement::EdgeBugAutoStrafe.Value)
						AutoStrafeEdgeBug(pCmd, pLocal);
					else
					{
						pCmd->forwardmove = 0.f;
						pCmd->sidemove = 0.f;
					}
				}

				// Store command for this tick
				m_EdgeBugCmds[iTick].viewangles = pCmd->viewangles;
				m_EdgeBugCmds[iTick].forwardmove = pCmd->forwardmove;
				m_EdgeBugCmds[iTick].sidemove = pCmd->sidemove;
				m_EdgeBugCmds[iTick].buttons = pCmd->buttons;
				m_EdgeBugCmds[iTick].origin = pLocal->m_vecOrigin();

				// Backup velocity before prediction
				Vec3 vPrePredVelocity = pLocal->m_vecVelocity();

				// Simulate
				F::EnginePrediction.Simulate(pLocal, pCmd);

				Vec3 vPostPredVelocity = pLocal->m_vecVelocity();
				m_vEdgeBugPredictedVelocity = vPostPredVelocity;

				// Advanced edgebug detection
				float flVelocityDiff = vPrePredVelocity.z - vPostPredVelocity.z;
				Vec3 vVelocityAngleDelta = Math::VectorAngles(vPostPredVelocity) - Math::VectorAngles(vPrePredVelocity);
				Math::ClampAngles(vVelocityAngleDelta);

				// Check 1: Velocity increased while falling (edge scrape)
				bool bVelocityIncreased = floorf(vPostPredVelocity.z) > floorf(vPrePredVelocity.z) &&
					vPrePredVelocity.z < 0.f &&
					vPostPredVelocity.z < 0.f &&
					vPrePredVelocity.z * 0.25f > flVelocityDiff &&
					fabsf(vVelocityAngleDelta.y) < 45.f;

				// Check 2: Horizontal velocity increased (edge push)
				float flPreHorizSpeed = vPrePredVelocity.Length2D();
				float flPostHorizSpeed = vPostPredVelocity.Length2D();
				bool bHorizontalIncrease = flPostHorizSpeed > flPreHorizSpeed;

				// Check 3: Standard gravity reset check
				bool bGravityReset = EdgeBugCheck(pLocal, pCmd);

				// Detect edgebug
				if ((bVelocityIncreased && bHorizontalIncrease) || bGravityReset)
				{
					m_bEdgeBugDetected = true;
					m_iEdgeBugLockTicks = iTick;
					m_iEdgeBugPredictTick = iTick;
					m_iEdgeBugCurrentTick = 0;
					m_iEdgeBugSearchMode = iCurrentMode;
					m_bEdgeBugDuck = bApplyDuck;
					m_iEdgeBugPredictionTimestamp = I::GlobalVars->tickcount;
					m_vEdgeBugOriginalAngles = vBackupAngles;
					m_flEdgeBugOriginalForward = flBackupForward;
					m_flEdgeBugOriginalSide = flBackupSide;
					iLastSuccessMode = iCurrentMode;

					// If we used strafe, mark it
					if (bApplyStrafe && iCurrentMode >= 2)
					{
						bAppliedStrafeLast = true;
						vLastStrafeAngles = vCurrentAngle;
						flLastStrafeForward = pCmd->forwardmove;
						flLastStrafeSide = pCmd->sidemove;
					}

					break;
				}

				// Update backup velocity for next iteration
				m_vEdgeBugVelocityBackup = vPostPredVelocity;

				if (pLocal->m_fFlags() & FL_ONGROUND)
					break;
			}

			// Restore
			pCmd->viewangles = vBackupAngles;
			pCmd->forwardmove = flBackupForward;
			pCmd->sidemove = flBackupSide;
			pCmd->buttons = iBackupButtons;
		}
	}

	// Execute edgebug
	if (m_bEdgeBugDetected)
	{
		RestoreEntityToPredicted();

		m_iEdgeBugCurrentTick++;

		// Check if we're done
		if (m_iEdgeBugCurrentTick > m_iEdgeBugLockTicks)
		{
			I::ClientModeShared->m_pChatElement->ChatPrintf(0, "\x07" "AF96FF" "Aletherium \x07" "FFFFFF" "| Edgebug");
			m_bEdgeBugDetected = false;
			m_iEdgeBugCurrentTick = 0;
			m_iEdgeBugSearchMode = 0;
			bAppliedStrafeLast = false;
			return;
		}

		// Check distance to predicted origin
		int iCmdIndex = m_iEdgeBugCurrentTick - 1;
		if (iCmdIndex >= 0 && iCmdIndex < 64)
		{
			Vec3 vCurrentOrigin = pLocal->m_vecOrigin();
			float flDist = vCurrentOrigin.DistTo(m_EdgeBugCmds[iCmdIndex].origin);

			// If too far from predicted path, reset
			if (flDist > 1.f)
			{
				m_bEdgeBugDetected = false;
				m_iEdgeBugCurrentTick = 0;
				m_iEdgeBugSearchMode = 0;
				bAppliedStrafeLast = false;
				return;
			}

			// Apply stored command
			pCmd->buttons = m_EdgeBugCmds[iCmdIndex].buttons;
			pCmd->forwardmove = m_EdgeBugCmds[iCmdIndex].forwardmove;
			pCmd->sidemove = m_EdgeBugCmds[iCmdIndex].sidemove;

			// Silent mode - correct movement but keep visual angles
			Vec3 vTargetAngles = m_EdgeBugCmds[iCmdIndex].viewangles;

			// For modes with angle changes (2, 3, 4, 5), apply silent correction - ������ ��������
			if (m_iEdgeBugSearchMode >= 2)
			{
				CorrectMovement(pCmd, vTargetAngles, vBackupAngles);
				// Keep original angles for visual (silent)
				pCmd->viewangles = vBackupAngles;
				G::PSilentAngles = true;
			}
			else
			{
				// For still modes or if silent disabled, just apply angles normally
				pCmd->viewangles = vTargetAngles;
			}

			// Store for smooth interpolation
			m_vEdgeBugTargetAngles = vTargetAngles;
		}
	}
}

void CMisc::EdgeBugMouseLock(float& x, float& y)
{
	if (!Vars::Misc::Movement::EdgeBug.Value || !Vars::Misc::Movement::EdgeBugMouseLock.Value)
		return;

	auto pLocal = H::Entities.GetLocal();
	if (!pLocal || !pLocal->IsAlive())
		return;

	if (m_bEdgeBugDetected && m_iEdgeBugLockTicks > 0)
	{
		// Calculate mouse lock strength based on remaining ticks
		int iRemainingTicks = m_iEdgeBugPredictionTimestamp + m_iEdgeBugPredictTick - I::GlobalVars->tickcount;

		if (iRemainingTicks > 0 && x != 0.0f)
		{
			// Progressive mouse dampening - stronger as we get closer to edgebug
			float flProgress = static_cast<float>(iRemainingTicks) / static_cast<float>(m_iEdgeBugPredictTick);
			float flDampening = 1.0f - (flProgress * 0.95f); // 95% reduction at start, 0% at end

			// Store mouse offset for prediction accuracy
			m_iEdgeBugMouseOffset = static_cast<int>(std::abs(x));

			// Apply dampening
			x *= flDampening;
			y *= flDampening;
		}
		else
		{
			// Full lock when very close or past the edgebug point
			x = 0.f;
			y = 0.f;
		}
	}
}

// Long Jump - duck for 2 ticks after leaving ground (CS:GO style)
void CMisc::LongJump(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	if (!Vars::Misc::Movement::LongJump.Value)
	{
		m_bLongJumpDetected = false;
		return;
	}

	if (!pLocal || !pLocal->IsAlive())
		return;

	if (pLocal->m_MoveType() == MOVETYPE_LADDER || pLocal->m_MoveType() == MOVETYPE_NOCLIP)
		return;

	static int longjump_tick = 0;
	static bool ljbool = false;

	// Detect leaving ground - was on ground before prediction, now in air
	if ((m_iPrePredictionFlags & FL_ONGROUND) && !(pLocal->m_fFlags() & FL_ONGROUND))
	{
		pCmd->buttons |= IN_JUMP;
		ljbool = true;
		longjump_tick = I::GlobalVars->tickcount + 2;
	}

	if (ljbool)
	{
		m_bLongJumpDetected = true;

		// Hold duck for 2 ticks after leaving ground
		if (I::GlobalVars->tickcount < longjump_tick)
		{
			pCmd->buttons |= IN_DUCK;
		}

		// Reset after longjump_tick
		if (I::GlobalVars->tickcount >= longjump_tick)
		{
			ljbool = false;
			m_bLongJumpDetected = false;
		}
	}
	else
	{
		m_bLongJumpDetected = false;
	}
}
// Mini Jump - jump + duck on leaving ground for small hop
void CMisc::MiniJump(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	if (!Vars::Misc::Movement::MiniJump.Value)
	{
		m_bMiniJumpDetected = false;
		m_bMiniJumpShouldDuck = false;
		return;
	}

	// Reset duck state when landing (use pre-prediction flags)
	if (m_iPrePredictionFlags & FL_ONGROUND && pLocal->m_fFlags() & FL_ONGROUND)
		m_bMiniJumpShouldDuck = false;

	// Detect leaving ground - was on ground before prediction, not on ground after
	if ((m_iPrePredictionFlags & FL_ONGROUND) && !(pLocal->m_fFlags() & FL_ONGROUND))
	{
		pCmd->buttons |= IN_JUMP;
		pCmd->buttons |= IN_DUCK;
		m_bMiniJumpDetected = true;

		if (Vars::Misc::Movement::MiniJumpHoldDuck.Value)
			m_bMiniJumpShouldDuck = true;
	}
	else
	{
		m_bMiniJumpDetected = false;
	}

	if (m_bMiniJumpShouldDuck)
		pCmd->buttons |= IN_DUCK;
}

// Auto Align - align movement to wall for pixel surf
void CMisc::AutoAlign(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	if (!Vars::Misc::Movement::AutoAlign.Value)
	{
		m_bWallDetected = false;
		return;
	}

	if (pLocal->m_fFlags() & FL_ONGROUND)
		return;

	if (pLocal->m_MoveType() == MOVETYPE_LADDER || pLocal->m_MoveType() == MOVETYPE_NOCLIP)
	{
		m_bWallDetected = false;
		return;
	}

	float flMaxRadius = PI * 2.f;
	float flStep = flMaxRadius / 16.f;
	Vec3 vStartPos = pLocal->GetAbsOrigin();
	Vec3 vMins = pLocal->m_vecMins();
	Vec3 vMaxs = pLocal->m_vecMaxs();

	CTraceFilterWorldAndPropsOnly filter;
	CGameTrace trace;
	m_bWallDetected = false;

	// Find wall around player
	for (float a = m_flAutoAlignStartCircle; a < flMaxRadius; a += flStep)
	{
		Vec3 vEndPos;
		vEndPos.x = cosf(a) + vStartPos.x;
		vEndPos.y = sinf(a) + vStartPos.y;
		vEndPos.z = vStartPos.z;

		SDK::TraceHull(vStartPos, vEndPos, vMins, vMaxs, MASK_PLAYERSOLID, &filter, &trace);
		
		if (trace.fraction != 1.f && trace.plane.normal.z == 0.f)
		{
			m_bWallDetected = true;
			m_flAutoAlignStartCircle = a;
			break;
		}
	}

	if (!m_bWallDetected)
	{
		m_flAutoAlignStartCircle = 0.f;
		return;
	}

	// Calculate movement direction towards wall
	Vec3 vNormalPlane = Vec3(trace.plane.normal.x * -0.005f, trace.plane.normal.y * -0.005f, 0.f);
	Vec3 vWallAngle = Math::VectorAngles(vNormalPlane);
	Math::ClampAngles(vWallAngle);

	float flRotation = DEG2RAD(vWallAngle.y - pCmd->viewangles.y);
	float flCosRot = cosf(flRotation);
	float flSinRot = sinf(flRotation);

	// Try to find movement that maintains pixel surf velocity
	bool bDetect = false;
	for (float flMultiplier = 0.f; flMultiplier < 100.f; flMultiplier += 10.f)
	{
		F::Misc.RestoreEntityToPredicted();

		float flForward = flCosRot * flMultiplier;
		float flSide = -flSinRot * flMultiplier;
		pCmd->forwardmove = flForward;
		pCmd->sidemove = flSide;

		F::EnginePrediction.Simulate(pLocal, pCmd);
		
		float flNewZSpeed = pLocal->m_vecVelocity().z;
		if (flNewZSpeed == -6.25f) // Pixel surf velocity
		{
			bDetect = true;
			break;
		}
	}

	if (!bDetect)
	{
		// Default small movement towards wall
		pCmd->forwardmove = flCosRot * 10.f;
		pCmd->sidemove = -flSinRot * 10.f;
	}
}

// Pixel Surf - maintain pixel surf state by ducking
void CMisc::PixelSurf(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	if (!Vars::Misc::Movement::PixelSurf.Value)
	{
		m_bShouldPixelSurf = false;
		return;
	}

	if (!pLocal->IsAlive())
	{
		m_iPixelSurfTicks = 0;
		m_bShouldPixelSurf = false;
		return;
	}

	if (pLocal->m_MoveType() == MOVETYPE_NOCLIP || pLocal->m_MoveType() == MOVETYPE_LADDER)
		return;

	if (pLocal->m_fFlags() & FL_ONGROUND)
	{
		m_bShouldPixelSurf = false;
		return;
	}

	if (!m_bWallDetected)
		return;

	if (!m_bShouldPixelSurf)
	{
		int iBackupButtons = pCmd->buttons;
		bool bFoundPixelSurf = false;
		
		// Try standing and ducking to find pixel surf
		for (int i = 0; i < 2; i++)
		{
			F::Misc.RestoreEntityToPredicted();
			
			if (i == 0)
				pCmd->buttons &= ~IN_DUCK;
			else
				pCmd->buttons |= IN_DUCK;

			// Predict forward to find pixel surf
			for (int z = 0; z < 8; z++)
			{
				F::EnginePrediction.Simulate(pLocal, pCmd);
				
				if (pLocal->m_fFlags() & FL_ONGROUND)
					break;

				float flZVelo = pLocal->m_vecVelocity().z;
				m_bShouldPixelSurf = (flZVelo == -6.25f);
				
				if (m_bShouldPixelSurf && i == 0)
				{
					// Found pixel surf while standing - don't need to duck
					m_bShouldPixelSurf = false;
					pCmd->buttons = iBackupButtons;
					F::Misc.RestoreEntityToPredicted();
					return;
				}
				
				if (m_bShouldPixelSurf)
				{
					m_iPixelSurfTicks = I::GlobalVars->tickcount + z + 16;
					bFoundPixelSurf = true;
					// Keep the duck button that we set above (i == 1)
					break;
				}
			}
			
			if (bFoundPixelSurf)
				break;
		}
		
		// Only restore buttons if we didn't find pixel surf
		if (!bFoundPixelSurf)
		{
			pCmd->buttons = iBackupButtons;
		}
		
		F::Misc.RestoreEntityToPredicted();
	}
	else
	{
		pCmd->buttons |= IN_DUCK;
		
		if (I::GlobalVars->tickcount > m_iPixelSurfTicks)
		{
			if (pLocal->m_vecVelocity().z != -6.25f)
				m_bShouldPixelSurf = false;
		}
	}
}

void CMisc::Draw(CTFPlayer* pLocal)
{
	// Empty - EdgeBug drawing removed
}