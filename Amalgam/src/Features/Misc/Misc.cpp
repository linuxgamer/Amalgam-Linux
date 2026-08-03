#include "Misc.h"

#include "../Backtrack/Backtrack.h"
#include "../Ticks/Ticks.h"
#include "../Players/PlayerUtils.h"
#include "../Aimbot/AutoRocketJump/AutoRocketJump.h"

#include <format>
#include <fstream>
#include <filesystem>
#include <sstream>
#include "../EnginePrediction/EnginePrediction.h"
#include "../Configs/Configs.h"

// Defined in Hooks/CAttributeManager_AttribHookValue.cpp: lazily installs the
// CProxyAnimatedWeaponSheen::OnBind hook that backs the custom weapon-sheen colour.
void InstallSheenColorHook();

void CMisc::RunPre(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	InstallSheenColorHook();
	CheatsBypass();
	WeaponSway();
	AntiAFK(pLocal, pCmd);
	InstantRespawnMVM(pLocal);
	NoisemakerSpam(pLocal);
	Checkpoints(pLocal, pCmd); // KZ practice: save/teleport/noclip keys - BEFORE the movetype gate so the noclip toggle still works while noclipping

	// Flip world: latch the per-frame active flag (read by CFlipWorld::Render, which mirrors the world
	// in the DoPostScreenSpaceEffects pass) and mirror the controls to match. Active only while alive.
	G::FlipWorldActive = Vars::Visuals::World::FlipWorld.Value && pLocal->IsAlive();
	FlipWorldInput(pCmd); // runs every tick: applies the mirror while active, restores m_yaw on disable

	// Save flags + velocity before prediction (LongJump/MiniJump transitions, and the pixel-surf
	// ride detection - the reference keys its "ps" off the PRE-prediction velocity, see PixelSurf).
	m_iPrePredictionFlags = pLocal->m_fFlags();
	m_vPrePredictionVelocity = pLocal->m_vecVelocity();

	JumpStats(pLocal, pCmd); // track takeoff/landing on the real (pre-prediction) ground flags

	if (!pLocal->IsAlive() || pLocal->IsAGhost() || pLocal->m_MoveType() != MOVETYPE_WALK || pLocal->IsSwimming()
		|| pLocal->IsTaunting() || pLocal->InCond(TF_COND_HALLOWEEN_KART) || pLocal->InCond(TF_COND_SHIELD_CHARGE))
		return;

	// Snapshot the player's REAL jump input here, before AutoJump (Bunnyhop) can run.
	m_bRawJumpHeld = pCmd->buttons & IN_JUMP;

	EdgeBugPrePrediction(pLocal, pCmd);
	AutoJump(pLocal, pCmd);
	MiniJumpPre(pLocal, pCmd); // AFTER AutoJump so bhop-injected jumps also go through the minijump
	                           // queue (its landing-tick jump fired before the previous hop's duck
	EdgeJump(pLocal, pCmd);
	AutoJumpbug(pLocal, pCmd);
	AutoStrafe(pLocal, pCmd);
	StrafeOptimizer(pLocal, pCmd); // manual strafe perfecter (skips itself if AutoStrafe is on)
	AutoPeek(pLocal, pCmd);
	MovementLock(pLocal, pCmd);
	BreakJump(pLocal, pCmd);
}

// Flip world: mirror the player's controls to match the horizontally-mirrored view.
void CMisc::FlipWorldInput(CUserCmd* pCmd)
{
	// Mouse: mirror horizontal look by flipping the SIGN of the mouse-yaw factor. Letting the engine
	// apply it natively keeps turning perfectly smooth - rewriting the view angle every tick (the old
	static auto m_yaw = H::ConVars.FindVar("m_yaw");
	if (m_yaw)
	{
		const float flYaw = m_yaw->GetFloat();
		if (G::FlipWorldActive && flYaw > 0.f)
		{
			m_yaw->SetValue(-flYaw);
			m_bFlipWorldYawNegated = true;
		}
		else if (!G::FlipWorldActive && flYaw < 0.f)
		{
			// Heal a negative m_yaw whenever the feature is off.
			m_yaw->SetValue(-flYaw);
			m_bFlipWorldYawNegated = false;
		}
	}

	// Strafe keys: a horizontal mirror swaps screen left/right, so flip sidemove to still move the
	// player toward the side they press on the mirrored view.
	if (G::FlipWorldActive)
		pCmd->sidemove = -pCmd->sidemove;
}

void CMisc::RunPost(CTFPlayer* pLocal, CUserCmd* pCmd, bool pSendPacket)
{
	m_TB_WallCache.valid = false;  // reset each tick
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
		PixelSurfAssist(pLocal, pCmd);
		// PixelSurf sets IN_DUCK when a surf is catchable; AutoAlign must run AFTER it so its
		// velocity-hold search simulates with the crouch active (the reference ducks in pixel_surf, then
		PixelSurf(pLocal, pCmd);
		AutoAlign(pLocal, pCmd);
		TextureBug(pLocal, pCmd);
		HeadSurf(pLocal, pCmd);
		WallClimb(pLocal, pCmd);
		AirStuck(pLocal, pCmd);   // the reference air_stuck/wall_stuck (combined): insta-stick any vertical wall
		PixelFinder(pLocal, pCmd);
		PixelSurfLine(pLocal, pCmd); // the reference: record the surf strip when a ride begins (uses PixelSurf's m_bPixelSurfingNow set above)
		MovementRecorder(pLocal, pCmd); // record/replay the final movement cmd (runs last)
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
	m_bJumpBugHit = false; // set only on the fire tick so BreakJump yields to us

	// Gate on the held jump input, not pCmd (Bunnyhop strips IN_JUMP mid-air, so pCmd would read no jump).
	// No fall-speed or height gate - hold jump through a fall and it fires on landing.
	if (!Vars::Misc::Movement::AutoJumpbug.Value || !m_bRawJumpHeld || pLocal->m_hGroundEntity())
		return;

	// Must be descending; crouching on the way up only kills jump height.
	if (pLocal->m_vecVelocity().z >= 0.f)
		return;

	// Fire window is a tight [20u, 22u] ground band. Unducking while airborne teleports the origin down
	const float flUnduckHeight  = 20.f * pLocal->m_flModelScale();
	const float flTraceDistance = flUnduckHeight + 2.f;

	const Vec3 vOrigin = pLocal->m_vecOrigin();
	const Vec3 vMins = pLocal->m_vecMins();
	const Vec3 vMaxs = pLocal->m_vecMaxs();
	CTraceFilterWorldAndPropsOnly filter = {};

	// Crouch only right before landing, not for the whole descent: a longer "arm" trace gates the duck on
	// ground proximity. An air-duck sets FL_DUCKING in one tick, so a couple ticks of lead is enough - arm
	const float flPerTickFall = fabsf(pLocal->m_vecVelocity().z) * TICK_INTERVAL;
	const float flArmDistance = flTraceDistance + std::max(flPerTickFall * 3.f, 8.f);

	CGameTrace armTrace = {};
	SDK::TraceHull(vOrigin, vOrigin - Vec3(0, 0, flArmDistance), vMins, vMaxs, pLocal->SolidMask(), &filter, &armTrace);
	if (!armTrace.DidHit())
		return; // ground still too far below - stay standing, don't start the crouch yet

	CGameTrace trace = {};
	SDK::TraceHull(vOrigin, vOrigin - Vec3(0, 0, flTraceDistance), vMins, vMaxs, pLocal->SolidMask(), &filter, &trace);
	const float flGroundDist = trace.DidHit() ? trace.fraction * flTraceDistance : flTraceDistance;

	// Fire only when already fully ducked and the ground sits one unduck-drop below (feet in [20u, 22u]).
	// Not ducked = no unduck teleport to snap us, so clearing IN_DUCK would do nothing.
	if (pLocal->IsDucking() && trace.DidHit() && flGroundDist >= flUnduckHeight)
	{
		pCmd->buttons &= ~IN_DUCK;
		pCmd->buttons |= IN_JUMP;
		m_bJumpBugHit = true;      // BreakJump must NOT re-add IN_DUCK this tick or it cancels our unduck
		m_bJumpBugThisAir = true;  // mark this airtime so JumpStats classifies the resulting jump as "JB"
		return;
	}

	// Setup branch: armed (ground within flArmDistance) but not firing this tick.
	pCmd->buttons |= IN_DUCK;
	pCmd->buttons &= ~IN_JUMP;
}

// single-checkbox perfect strafe (steer-to-look rework of the reference auto_strafe).
void CMisc::AutoStrafe(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	m_bAutoStrafeActive = false;

	if (!Vars::Misc::Movement::AutoStrafe.Value || !pLocal || !pLocal->IsAlive())
		return;

	const int iMoveType = pLocal->m_MoveType();
	if (iMoveType == MOVETYPE_LADDER || iMoveType == MOVETYPE_NOCLIP)
		return;

	if (pLocal->m_fFlags() & FL_ONGROUND)
		return;

	const Vec3 vVelocity = pLocal->m_vecVelocity();
	const float flSpeed = vVelocity.Length2D();
	if (flSpeed <= 1.f) // need any horizontal speed to strafe against; engage as early as possible
		return;

	// Direction offset from movement keys. none -> steer straight to view.
	const bool bBack    = (pCmd->buttons & IN_BACK)      != 0;
	const bool bForward = (pCmd->buttons & IN_FORWARD)   != 0;
	const bool bRight   = (pCmd->buttons & IN_MOVERIGHT) != 0;
	const bool bLeft    = (pCmd->buttons & IN_MOVELEFT)  != 0;

	float flAngleOffset = 0.f;
	if (bBack)
	{
		flAngleOffset = -180.f;
		if (bLeft)       flAngleOffset -= 45.f;
		else if (bRight) flAngleOffset += 45.f;
	}
	else if (bLeft)
	{
		flAngleOffset = 90.f;
		if (bForward)    flAngleOffset -= 45.f;
	}
	else if (bRight)
	{
		flAngleOffset = -90.f;
		if (bForward)    flAngleOffset += 45.f;
	}

	const Vec3 vOldAngles = pCmd->viewangles;

	// Heading we want our velocity to chase (view yaw + WASD offset) and where velocity points now.
	const float flDesiredYaw = Math::NormalizeAngle(vOldAngles.y + flAngleOffset);
	const float flVelYaw     = RAD2DEG(atan2f(vVelocity.y, vVelocity.x));

	// Signed error of velocity from the heading (+ = velocity is left/CCW of where we look).
	const float flErr = Math::NormalizeAngle(flVelYaw - flDesiredYaw);

	// Heading-hugging latch (physical-minimum swing). Max-turn velocity straight toward the desired
	// heading every tick: turn left while velocity is right of the heading, right while it's left. This
	if (flErr >= 0.f)
		m_bAutoStrafeWeaveLeft = false;
	else
		m_bAutoStrafeWeaveLeft = true;
	const bool bTurnLeft = m_bAutoStrafeWeaveLeft;

	static auto cl_sidespeed   = H::ConVars.FindVar("cl_sidespeed");
	static auto cl_forwardspeed = H::ConVars.FindVar("cl_forwardspeed");
	static auto sv_airaccelerate = H::ConVars.FindVar("sv_airaccelerate");
	const float flSideSpeed = cl_sidespeed ? cl_sidespeed->GetFloat() : 450.f;
	const float flFwdSpeed  = cl_forwardspeed ? cl_forwardspeed->GetFloat() : 450.f;

	// Exact per-tick optimal wish angle off velocity. TF2 air-accel caps the added speed at 30u
	const float flWishSpeed = sqrtf(flSideSpeed * flSideSpeed + flFwdSpeed * flFwdSpeed);
	const float flAccel  = sv_airaccelerate ? sv_airaccelerate->GetFloat() : 10.f;
	const float flAddCap = std::min(30.f, flWishSpeed);
	const float flAccelSpeed = flAccel * flWishSpeed * TICK_INTERVAL; // surfaceFriction ~1 in air
	const float flCosLean = (flAccelSpeed >= flAddCap) ? 0.f : std::clamp((flAddCap - flAccelSpeed) / flSpeed, 0.f, 1.f);
	const float flSinLean = sqrtf(std::max(0.f, 1.f - flCosLean * flCosLean));

	// Build the wish in a frame whose forward IS the velocity: forward leans into velocity, side
	Vec3 vWishAngle = vOldAngles;
	vWishAngle.y = flVelYaw;
	pCmd->forwardmove = flFwdSpeed * flCosLean;
	pCmd->sidemove    = (bTurnLeft ? -flSideSpeed : flSideSpeed) * flSinLean;

	CorrectMovement(pCmd, vWishAngle, vOldAngles);
	pCmd->viewangles = vOldAngles;

	m_bAutoStrafeActive = true;
}

// Strafe optimizer. Unlike AutoStrafe (silent, fully
// automatic), this PERFECTS the strafes you make yourself: while airborne, holding only A or D (no
void CMisc::StrafeOptimizer(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	if (!Vars::Misc::Movement::StrafeOptimizer.Value || !pLocal || !pLocal->IsAlive())
		return;

	// AutoStrafe already drives perfect strafes; don't fight it if both are on.
	if (Vars::Misc::Movement::AutoStrafe.Value)
		return;

	const int iMoveType = pLocal->m_MoveType();
	if (iMoveType == MOVETYPE_LADDER || iMoveType == MOVETYPE_NOCLIP || iMoveType == MOVETYPE_OBSERVER)
		return;

	static float flPrevYaw = pCmd->viewangles.y;

	const Vec3 vVelocity = pLocal->m_vecVelocity();
	const bool bAirborne = !(pLocal->m_fFlags() & FL_ONGROUND);

	if (pCmd->forwardmove == 0.f && bAirborne
		&& vVelocity.Length2D() > float(Vars::Misc::Movement::StrafeOptimizerMinSpeed.Value))
	{
		static auto m_yaw = H::ConVars.FindVar("m_yaw");
		static auto sensitivity = H::ConVars.FindVar("sensitivity");
		const float flYawFactor = m_yaw ? m_yaw->GetFloat() : 0.022f;
		const float flSens = sensitivity ? sensitivity->GetFloat() : 1.f;
		const float flYawStep = flSens * flYawFactor; // smallest yaw change one mouse-count produces
		const float flGain = Vars::Misc::Movement::StrafeOptimizerGain.Value / 100.f;

		if (flYawStep > 0.f)
		{
			const float flVelYaw = RAD2DEG(atan2f(vVelocity.y, vVelocity.x));

			// When the view is more than 90deg off the velocity, the player is travelling
			// "backwards" relative to where they look (backward surf / air strafes). The optimal
			const bool  bBackward = fabsf(remainderf(pCmd->viewangles.y - flVelYaw, 360.f)) > 90.f;
			const float flAxisYaw = bBackward ? remainderf(flVelYaw + 180.f, 360.f) : flVelYaw;

			// Turning left: pair with the left strafe (sidemove < 0) moving forward, or the
			// right strafe (sidemove > 0) moving backward (the gaining turn direction flips).
			if ((flPrevYaw - pCmd->viewangles.y < 0.f) && (bBackward ? pCmd->sidemove > 0.f : pCmd->sidemove < 0.f))
			{
				float flStrafe = remainderf(pCmd->viewangles.y - flAxisYaw, 360.f) * flGain;
				if (flStrafe < -flYawStep)
				{
					if (flStrafe < -180.f) flStrafe = -180.f;
					pCmd->viewangles.y = remainderf(pCmd->viewangles.y - flYawStep * roundf(flStrafe / flYawStep), 360.f);
					pCmd->mousedx = short(flSens * ceilf(remainderf(flPrevYaw - pCmd->viewangles.y, 360.f) / sqrtf(flYawStep)));
					I::EngineClient->SetViewAngles(pCmd->viewangles);
				}
			}
			// Turning right: pair with the right strafe (sidemove > 0) moving forward, or the
			// left strafe (sidemove < 0) moving backward (the gaining turn direction flips).
			else if ((flPrevYaw - pCmd->viewangles.y > 0.f) && (bBackward ? pCmd->sidemove < 0.f : pCmd->sidemove > 0.f))
			{
				float flStrafe = remainderf(pCmd->viewangles.y - flAxisYaw, 360.f) * flGain;
				if (flStrafe > flYawStep)
				{
					if (flStrafe > 180.f) flStrafe = 180.f;
					pCmd->viewangles.y = remainderf(pCmd->viewangles.y - flYawStep * roundf(flStrafe / flYawStep), 360.f);
					pCmd->mousedx = short(flSens * ceilf(remainderf(flPrevYaw - pCmd->viewangles.y, 360.f) / sqrtf(flYawStep)));
					I::EngineClient->SetViewAngles(pCmd->viewangles);
				}
			}
		}
	}

	flPrevYaw = pCmd->viewangles.y;
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

	// AutoJumpbug runs before BreakJump in RunPre. On the landing tick the jumpbug clears IN_DUCK
	// and forces a fresh IN_JUMP edge; if BreakJump then re-adds IN_DUCK here it cancels the unduck
	if (m_bJumpBugHit)
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
	m_bBreakJumpThisAir = true; // mark this airtime so JumpStats names it "Break jump"
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

// Gravity-reset edgebug detection.
// vUnpredictedVel = velocity BEFORE this tick's Simulate; pLocal currently holds the post-Simulate state.
bool CMisc::EdgeBugCheck(CTFPlayer* pLocal, CUserCmd* pCmd, const Vec3& vUnpredictedVel, const Vec3& vUnpredictedOrigin, bool& bBreak, int& iSimsUsed, int iSimCap)
{
	bBreak = false;

	if (!pLocal || !pLocal->IsAlive())
	{
		bBreak = true;
		return false;
	}

	const int iMoveType = pLocal->m_MoveType();
	if (iMoveType == MOVETYPE_LADDER || iMoveType == MOVETYPE_NOCLIP)
	{
		bBreak = true;
		return false;
	}

	static auto sv_gravity = H::ConVars.FindVar("sv_gravity");
	if (!sv_gravity)
	{
		bBreak = true;
		return false;
	}

	const float flGravity = sv_gravity->GetFloat();
	const float flGravityVel = flGravity * 0.5f * TICK_INTERVAL * -1.f; // first-tick gravity velocity (negative)

	const Vec3 vPredictedVel = pLocal->m_vecVelocity();
	const int iPredictedFlags = pLocal->m_fFlags();

	const float flZVelPred = roundf(vPredictedVel.z);
	const float flSpeed2D = roundf(vPredictedVel.Length2D());

	// No point edgebugging while rising, grounded, or with no horizontal speed
	if (flZVelPred >= 0.f || (iPredictedFlags & FL_ONGROUND) || flSpeed2D == 0.f)
	{
		bBreak = true;
		return false;
	}
	// Edge scrape: was falling, velocity got reduced (less negative) but still falling -> verify with one more tick
	else if (vUnpredictedVel.z < 0.f && vPredictedVel.z > vUnpredictedVel.z && vPredictedVel.z < 0.f)
	{
		// Reachability gate: if we rose between pre- and post-sim (predicted origin above the
		// unpredicted one) the player origin can't actually hit this edge, so it's not a real edgebug.
		if (vUnpredictedOrigin.z < pLocal->m_vecOrigin().z)
			return false;

		const int iZVel = static_cast<int>(vPredictedVel.z);

		// Confirm tick counts against the per-CreateMove sim budget; out of budget -> stop this attempt.
		if (iSimsUsed >= iSimCap)
		{
			bBreak = true;
			return false;
		}
		iSimsUsed++;
		F::EnginePrediction.Simulate(pLocal, pCmd); // advance one more tick to confirm gravity reset

		const float flRoundedVel = roundf(-flGravity * TICK_INTERVAL) + iZVel;
		const float flUnpredictedVel = roundf(pLocal->m_vecVelocity().z);

		if (flRoundedVel == flUnpredictedVel || (flUnpredictedVel == 0.f && (pLocal->m_fFlags() & FL_ONGROUND)))
			return true;

		bBreak = true;
		return false;
	}
	// First-tick gravity case: fell past the gravity value and landed exactly on it.
	else if (vUnpredictedVel.z < flGravityVel && roundf(vPredictedVel.z) == roundf(flGravityVel) && iMoveType != MOVETYPE_LADDER)
	{
		return true;
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

// edge bug sound. Plays the configured stock TF2 ui/tool sound (or a custom path)
void CMisc::PlayEdgeBugSound()
{
	if (!Vars::Misc::Movement::EdgeBugSound.Value || !I::MatSystemSurface)
		return;

	const char* sFile = nullptr;
	switch (Vars::Misc::Movement::EdgeBugSound.Value)
	{
	case Vars::Misc::Movement::EdgeBugSoundEnum::PanelOpen:      sFile = "ui/panel_open.wav"; break;
	case Vars::Misc::Movement::EdgeBugSoundEnum::MenuFocus:      sFile = "ui/menu_focus.wav"; break;
	case Vars::Misc::Movement::EdgeBugSoundEnum::ButtonClick:    sFile = "ui/buttonclick.wav"; break;
	case Vars::Misc::Movement::EdgeBugSoundEnum::ButtonRollover: sFile = "ui/buttonrollover.wav"; break;
	case Vars::Misc::Movement::EdgeBugSoundEnum::Beep:           sFile = "tools/ifm/beep.wav"; break;
	case Vars::Misc::Movement::EdgeBugSoundEnum::Blip:           sFile = "hl1/fvox/blip.wav"; break;
	case Vars::Misc::Movement::EdgeBugSoundEnum::Custom:
		if (!Vars::Misc::Movement::EdgeBugSoundCustom.Value.empty())
			sFile = Vars::Misc::Movement::EdgeBugSoundCustom.Value.c_str();
		break;
	}

	if (sFile && sFile[0])
		I::MatSystemSurface->PlaySound(sFile);
}

void CMisc::EdgeBugPostPrediction(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	EnginePredictionBatch_t tPredictionBatch; // players adjusted once for the whole search, not per Simulate

	if (!Vars::Misc::Movement::EdgeBug.Value)
	{
		m_bEdgeBugDetected = false;
		m_iEdgeBugLockTicks = 0;
		m_iEdgeBugCurrentTick = 0;
		m_iEdgeBugSearchMode = 0;
		m_bEdgeBugHasPath = false;
		return;
	}

	// Don't run if we were on ground or rising before prediction
	if (m_iEdgeBugFlags & FL_ONGROUND || m_vEdgeBugVelocityBackup.z > 0.f)
	{
		m_bEdgeBugDetected = false;
		m_iEdgeBugLockTicks = 0;
		m_iEdgeBugCurrentTick = 0;
		m_iEdgeBugSearchMode = 0;
		return;
	}

	// EnginePrediction.End() (called just before us in CreateMove) restored GlobalVars time back to the
	// real render frametime. CGameMovement::ProcessMovement integrates gravity/friction/movement against
	struct FTimeGuard
	{
		int m_nTick; float m_flCur, m_flFrame;
		FTimeGuard(int iTickBase)
		{
			m_nTick = I::GlobalVars->tickcount; m_flCur = I::GlobalVars->curtime; m_flFrame = I::GlobalVars->frametime;
			I::GlobalVars->tickcount = iTickBase;
			I::GlobalVars->curtime = TICKS_TO_TIME(iTickBase);
			I::GlobalVars->frametime = I::Prediction->m_bEnginePaused ? 0.f : TICK_INTERVAL;
		}
		~FTimeGuard()
		{
			I::GlobalVars->tickcount = m_nTick; I::GlobalVars->curtime = m_flCur; I::GlobalVars->frametime = m_flFrame;
		}
	} timeGuard(pLocal->m_nTickBase());

	// Backup the real command so we can restore it after searching
	const int iBackupButtons = pCmd->buttons;
	const float flBackupForward = pCmd->forwardmove;
	const float flBackupSide = pCmd->sidemove;
	const Vec3 vBackupAngles = pCmd->viewangles;

	// ---------- SEARCH ----------
	if (!m_bEdgeBugDetected)
	{
		// Hard per-CreateMove Simulate budget (mirrors the reference MAX_PREDICTIONS). The search can
		// fan out to ~96 strafe-scan attempts, each up to 128 ticks deep with a confirm tick - tens
		constexpr int EB_SIM_CAP = 512;
		int iSimsUsed = 0;

		const int iScanTicks = std::clamp(Vars::Misc::Movement::EdgeBugLockTicks.Value, 1, 128);
		const float flStartYaw = vBackupAngles.y;
		const float flAngleLimit = Vars::Misc::Movement::EdgeBugAngleLimit.Value;

		// ---- Open-air broad-phase (perf) ----
		// The fan-out below can burn the full EB_SIM_CAP of Simulate() calls every airborne-falling
		{
			RestoreEntityToPredicted(); // sample the predicted-frame start state (attempts re-restore)
			const Vec3 vGateOrigin = pLocal->m_vecOrigin();
			const Vec3 vGateVel = pLocal->m_vecVelocity();
			const float flDt = I::GlobalVars->interval_per_tick;
			const float flT = iScanTicks * flDt;
			static auto sv_gravity_gate = H::ConVars.FindVar("sv_gravity");
			const float flG = sv_gravity_gate ? sv_gravity_gate->GetFloat() : 800.f;

			const float flReach = vGateVel.Length2D() * flT * 2.f + 256.f;        // horizontal envelope
			const float flDrop = (-vGateVel.z) * flT + 0.5f * flG * flT * flT + 64.f; // vertical envelope

			Vec3 vGateEnd = vGateOrigin; vGateEnd.z -= flDrop;
			Vec3 vGateMins = pLocal->m_vecMins(); Vec3 vGateMaxs = pLocal->m_vecMaxs();
			vGateMins.x -= flReach; vGateMins.y -= flReach;
			vGateMaxs.x += flReach; vGateMaxs.y += flReach;

			CTraceFilterWorldAndPropsOnly gateFilter;
			CGameTrace gateTrace;
			SDK::TraceHull(vGateOrigin, vGateEnd, vGateMins, vGateMaxs, pLocal->SolidMask(), &gateFilter, &gateTrace);
			// CRITICAL: the inflated hull is huge, so whenever there IS geometry in reach (i.e. an actual
			// edgebug setup) it overlaps that geometry at the trace start -> startsolid, and Source reports
			if (gateTrace.fraction >= 1.f && !gateTrace.startsolid && !gateTrace.allsolid)
			{
				// Provably open air across the whole reachable envelope - nothing to edgebug.
				// Entity already sits on the predicted frame (same state the normal no-hit path leaves).
				m_bEdgeBugHasPath = false;
				return;
			}
		}

		// Yaw-scan parameters derived from the player's actual mouse movement this tick
		static auto m_yaw = H::ConVars.FindVar("m_yaw");
		static auto sensitivity = H::ConVars.FindVar("sensitivity");
		static auto cl_sidespeed = H::ConVars.FindVar("cl_sidespeed");
		const float flMYaw = m_yaw ? m_yaw->GetFloat() : 0.022f;
		const float flSens = sensitivity ? sensitivity->GetFloat() : 1.f;
		const float flSideSpeed = cl_sidespeed ? cl_sidespeed->GetFloat() : 450.f;
		const float flMaxYaw = std::clamp(pCmd->mousedx * flMYaw * flSens, -30.f, 30.f);
		const float flStrafeSide = (flMaxYaw >= 0.f ? 1.f : -1.f) * flSideSpeed;

		// Attempt modes: 0 = still, 1 = strafe yaw-scan, 2 = auto strafe.
		// Reused member buffer: clear() keeps capacity so the fan-out build is allocation-free per tick.
		std::vector<EdgeBugAttempt_t>& vAttempts = m_vEdgeBugAttempts;
		vAttempts.clear();
		vAttempts.push_back({ true,  0, 0.f }); // crouch, still
		vAttempts.push_back({ false, 0, 0.f }); // stand, still

		// Raw-input replay (mode 3): "keep holding your actual move keys". The still attempts zero
		// movement, so a pure-keyboard back edgebug (hold S, mouse still, no autostrafe) was never
		if (flBackupForward != 0.f || flBackupSide != 0.f)
		{
			vAttempts.push_back({ true,  3, 0.f }); // crouch, raw replay
			vAttempts.push_back({ false, 3, 0.f }); // stand, raw replay
		}

		if (Vars::Misc::Movement::EdgeBugAdvancedSearch.Value && fabsf(flMaxYaw) > 0.f)
		{
			const int iDivisor = std::max(1, Vars::Misc::Movement::EdgeBugScanRadius.Value);
			const float flStep = flMaxYaw / iDivisor;
			for (float flYaw = flStep; fabsf(flYaw) <= fabsf(flMaxYaw) + 0.001f; flYaw += flStep)
			{
				vAttempts.push_back({ true,  1, flYaw }); // crouch, strafe scan
				vAttempts.push_back({ false, 1, flYaw }); // stand, strafe scan
				if (vAttempts.size() > 96)
					break; // safety clamp
			}
		}

		if (Vars::Misc::Movement::EdgeBugAutoStrafe.Value)
		{
			vAttempts.push_back({ true,  2, 0.f }); // crouch, auto strafe
			vAttempts.push_back({ false, 2, 0.f }); // stand, auto strafe
		}

		std::vector<Vec3>& vTempPath = m_vEdgeBugScratchPath; // reuse member buffer; keeps capacity across ticks

		// classify the surface the detection tick actually clipped, and reject the bug if it's
		const auto EdgeBugSurfaceUsable = [&](const Vec3& vFrom, const Vec3& vVel) -> bool
		{
			const float flDt = I::GlobalVars->interval_per_tick;
			const Vec3 vTo = vFrom + vVel * flDt;
			CTraceFilterWorldAndPropsOnly surfFilter;
			CGameTrace surfTrace;
			SDK::TraceHull(vFrom, vTo, pLocal->m_vecMins(), pLocal->m_vecMaxs(), pLocal->SolidMask(), &surfFilter, &surfTrace);
			if (surfTrace.fraction >= 1.f)
				return true;                       // nothing clipped on the sweep -> don't reject
			if (surfTrace.IsDispSurface())
				return false;                      // displacements don't edgebug (user report)
			if (fabsf(surfTrace.plane.normal.z) < 0.26f)
				return false;                      // near-vertical wall/prop -> edgebugging it does nothing
			return true;
		};

		for (const auto& tAttempt : vAttempts)
		{
			if (m_bEdgeBugDetected || iSimsUsed >= EB_SIM_CAP)
				break;

			RestoreEntityToPredicted();

			vTempPath.clear();
			vTempPath.push_back(pLocal->m_vecOrigin());

			CUserCmd predictCmd = *pCmd;
			if (tAttempt.iMode == 1)
			{
				predictCmd.forwardmove = flBackupForward;
				predictCmd.sidemove = flStrafeSide;
			}
			else if (tAttempt.iMode == 0)
			{
				predictCmd.forwardmove = 0.f;
				predictCmd.sidemove = 0.f;
			}
			else if (tAttempt.iMode == 3)
			{
				predictCmd.forwardmove = flBackupForward; // raw replay: keep real move input, no yaw change
				predictCmd.sidemove = flBackupSide;
			}

			// Duck state is constant for the whole attempt - set it once, not every scan tick.
			if (tAttempt.bDuck)
				predictCmd.buttons |= IN_DUCK;
			else
				predictCmd.buttons &= ~IN_DUCK;

			for (int t = 1; t <= iScanTicks; t++)
			{
				if (iSimsUsed >= EB_SIM_CAP)
					break; // out of sim budget this tick - stop scanning

				if (tAttempt.iMode == 1)
				{
					const float flYaw = Math::NormalizeAngle(flStartYaw + tAttempt.flYawStep * t);
					if (flAngleLimit > 0.f && fabsf(Math::NormalizeAngle(flYaw - flStartYaw)) > flAngleLimit)
						break;
					predictCmd.viewangles.y = flYaw;
				}
				else if (tAttempt.iMode == 2)
				{
					AutoStrafeEdgeBug(&predictCmd, pLocal);
				}

				// Record this tick's command for replay during execution (origin is pre-sim)
				m_EdgeBugCmds[t - 1].viewangles = predictCmd.viewangles;
				m_EdgeBugCmds[t - 1].forwardmove = predictCmd.forwardmove;
				m_EdgeBugCmds[t - 1].sidemove = predictCmd.sidemove;
				m_EdgeBugCmds[t - 1].buttons = predictCmd.buttons;
				m_EdgeBugCmds[t - 1].origin = pLocal->m_vecOrigin();

				const Vec3 vPrePredVel = pLocal->m_vecVelocity();
				const Vec3 vPrePredOrigin = pLocal->m_vecOrigin();
				if (vPrePredVel.z > 0.f)
					break;

				iSimsUsed++;
				F::EnginePrediction.Simulate(pLocal, &predictCmd);

				vTempPath.push_back(pLocal->m_vecOrigin());

				bool bBreak = false;
				if (EdgeBugCheck(pLocal, &predictCmd, vPrePredVel, vPrePredOrigin, bBreak, iSimsUsed, EB_SIM_CAP))
				{
					// reject near-vertical/displacement surfaces (can't actually edgebug them).
					if (!EdgeBugSurfaceUsable(m_EdgeBugCmds[t - 1].origin, vPrePredVel))
						break; // bad surface for this attempt - move on to the next attempt
					m_bEdgeBugDetected = true;
					m_iEdgeBugLockTicks = t;
					m_iEdgeBugPredictTick = t;
					m_iEdgeBugCurrentTick = 0;
					// iMode: 0 still / 3 raw replay -> exec 0 (direct cmd replay, angles unchanged);
					// 1 strafe-scan -> exec 2; 2 autostrafe -> exec 3 (both apply silent angles).
					m_iEdgeBugSearchMode = (tAttempt.iMode == 1) ? 2 : (tAttempt.iMode == 2 ? 3 : 0);
					m_bEdgeBugDuck = tAttempt.bDuck;
					m_iEdgeBugPredictionTimestamp = I::GlobalVars->tickcount;
					m_vEdgeBugOriginalAngles = vBackupAngles;
					m_flEdgeBugOriginalForward = flBackupForward;
					m_flEdgeBugOriginalSide = flBackupSide;
					m_vEdgeBugPredictedVelocity = pLocal->m_vecVelocity();

					// Commit predicted path for visualization
					m_vEdgeBugPath = vTempPath;
					m_vEdgeBugLandPos = pLocal->m_vecOrigin();
					m_bEdgeBugHasPath = true;
					break;
				}

				if (bBreak)
					break;
				if (pLocal->m_fFlags() & FL_ONGROUND)
					break;
			}
		}

		// Restore the real command (search only mutated copies, but be safe)
		pCmd->buttons = iBackupButtons;
		pCmd->forwardmove = flBackupForward;
		pCmd->sidemove = flBackupSide;
		pCmd->viewangles = vBackupAngles;

		// Search ran Simulate repeatedly; put the entity back on the real predicted frame
		RestoreEntityToPredicted();
	}

	// ---------- EXECUTE ----------
	if (m_bEdgeBugDetected)
	{
		RestoreEntityToPredicted();

		m_iEdgeBugCurrentTick++;

		// Finished applying the locked path
		if (m_iEdgeBugCurrentTick > m_iEdgeBugLockTicks)
		{
			if (Vars::Misc::Movement::EdgeBugChatPrint.Value)
				// Cheat title in the accent color (ToHex() emits the \x07RRGGBB chat colour code).
				I::ClientModeShared->m_pChatElement->ChatPrintf(0, std::format("{}{} \x07" "FFFFFF" "| edgebug",
					Vars::Menu::Theme::Accent.Value.ToHex(), Vars::Menu::CheatTitle.Value).c_str());
			PlayEdgeBugSound();
			m_iEdgeBugSuccessCounter++; // completion event -> drives the indicator detection effects
			m_bEdgeBugDetected = false;
			m_iEdgeBugCurrentTick = 0;
			m_iEdgeBugSearchMode = 0;
			return;
		}

		const int iCmdIndex = m_iEdgeBugCurrentTick - 1;
		if (iCmdIndex >= 0 && iCmdIndex < 128)
		{
			const Vec3 vCurrentOrigin = pLocal->m_vecOrigin();
			const float flDist = vCurrentOrigin.DistTo(m_EdgeBugCmds[iCmdIndex].origin);

			// Real position drifted from the predicted path; abort to avoid forcing a bad move.
			// 1u was far too tight - normal prediction divergence over a multi-tick countdown easily
			if (flDist > 16.f)
			{
				m_bEdgeBugDetected = false;
				m_iEdgeBugCurrentTick = 0;
				m_iEdgeBugSearchMode = 0;
				return;
			}

			pCmd->buttons = m_EdgeBugCmds[iCmdIndex].buttons;
			pCmd->forwardmove = m_EdgeBugCmds[iCmdIndex].forwardmove;
			pCmd->sidemove = m_EdgeBugCmds[iCmdIndex].sidemove;

			const Vec3 vTargetAngles = m_EdgeBugCmds[iCmdIndex].viewangles;

			// Strafe / auto-strafe modes change view angles: apply silently
			if (m_iEdgeBugSearchMode >= 2)
			{
				CorrectMovement(pCmd, vTargetAngles, vBackupAngles);
				pCmd->viewangles = vBackupAngles;
				G::PSilentAngles = true;
			}
			else
			{
				pCmd->viewangles = vTargetAngles;
			}

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

	if (!m_bEdgeBugDetected || m_iEdgeBugLockTicks <= 0)
		return;

	m_iEdgeBugMouseOffset = static_cast<int>(std::abs(x));
	const float flAmount = std::clamp(Vars::Misc::Movement::EdgeBugLockAmount.Value / 100.f, 0.f, 1.f);

	switch (Vars::Misc::Movement::EdgeBugMouseLockType.Value)
	{
	case Vars::Misc::Movement::EdgeBugMouseLockTypeEnum::Classic:
		// Dampen both axes by the lock amount
		x *= 1.f - flAmount;
		y *= 1.f - flAmount;
		break;
	case Vars::Misc::Movement::EdgeBugMouseLockTypeEnum::Static:
		// Lock horizontal turning only
		x *= 1.f - flAmount;
		break;
	case Vars::Misc::Movement::EdgeBugMouseLockTypeEnum::Dynamic:
	{
		// Progressive dampening - stronger as we approach the edgebug tick
		int iRemainingTicks = m_iEdgeBugPredictionTimestamp + m_iEdgeBugPredictTick - I::GlobalVars->tickcount;
		if (iRemainingTicks > 0 && m_iEdgeBugPredictTick > 0)
		{
			float flProgress = static_cast<float>(iRemainingTicks) / static_cast<float>(m_iEdgeBugPredictTick);
			float flDampening = 1.f - (flProgress * flAmount);
			x *= flDampening;
			y *= flDampening;
		}
		else
		{
			x = 0.f;
			y = 0.f;
		}
		break;
	}
	case Vars::Misc::Movement::EdgeBugMouseLockTypeEnum::Predicted:
		// Full lock for the whole prediction window
		x = 0.f;
		y = 0.f;
		break;
	}
}

// Long Jump - duck for 2 ticks after leaving ground (CS:GO style)
void CMisc::LongJump(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	static int  longjump_tick = 0;
	static bool ljbool        = false;

	if (!Vars::Misc::Movement::LongJump.Value)
	{
		// Full reset on disable so a bind-toggle mid-air doesn't leave ljbool=true,
		// which would cause IN_DUCK to fire the next time the feature turns on.
		m_bLongJumpDetected = false;
		ljbool              = false;
		longjump_tick       = 0;
		return;
	}

	if (!pLocal || !pLocal->IsAlive())
		return;

	if (pLocal->m_MoveType() == MOVETYPE_LADDER || pLocal->m_MoveType() == MOVETYPE_NOCLIP)
		return;

	// Any ground-leave while LongJump is enabled fires the LJ: jump injected + duck window.
	if ((m_iPrePredictionFlags & FL_ONGROUND) && !(pLocal->m_fFlags() & FL_ONGROUND)
		&& m_iAssistJumpType == JT_NONE)
	{
		pCmd->buttons |= IN_JUMP;
		ljbool        = true;
		longjump_tick = I::GlobalVars->tickcount + 2;
	}

	if (ljbool)
	{
		m_bLongJumpDetected = true;

		// Hold duck for 2 ticks after leaving ground.
		if (I::GlobalVars->tickcount < longjump_tick)
			pCmd->buttons |= IN_DUCK;

		// Reset once the duck window expires.
		if (I::GlobalVars->tickcount >= longjump_tick)
		{
			ljbool              = false;
			m_bLongJumpDetected = false;
		}
	}
	else
	{
		m_bLongJumpDetected = false;
	}
}
// Mini Jump - duck on leaving ground for a small hop.
void CMisc::MiniJump(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	if (!Vars::Misc::Movement::MiniJump.Value)
	{
		// Full reset on disable so a bind-toggle mid-air doesn't leave
		// m_bMiniJumpShouldDuck=true, which would cause perpetual IN_DUCK next enable.
		m_bMiniJumpDetected   = false;
		m_bMiniJumpShouldDuck = false;
		m_iMiniJumpPendingUntil = 0;
		return;
	}

	const bool bGroundPre = m_iPrePredictionFlags & FL_ONGROUND;
	const bool bGroundPost = pLocal->m_fFlags() & FL_ONGROUND;

	// Clear hold-duck the moment we touch ground again.
	if (bGroundPre && bGroundPost)
		m_bMiniJumpShouldDuck = false;

	// Pre-landing unduck: release our held duck while still airborne and about to land (an
	// in-air unduck is instant) so we touch down STANDING. Landing ducked is what made chains
	if (m_bMiniJumpShouldDuck && !bGroundPost && pLocal->m_vecVelocity().z < 0.f)
	{
		const float flLookahead = -pLocal->m_vecVelocity().z * I::GlobalVars->interval_per_tick * 5.f + 16.f;
		CTraceFilterWorldAndPropsOnly filter;
		CGameTrace trace;
		SDK::TraceHull(pLocal->m_vecOrigin(), pLocal->m_vecOrigin() - Vec3(0.f, 0.f, flLookahead),
			pLocal->m_vecMins(), pLocal->m_vecMaxs(), MASK_PLAYERSOLID, &filter, &trace);
		if (trace.fraction < 1.f)
			m_bMiniJumpShouldDuck = false; // ground ~5 ticks away - stand up now
	}

	// jump -> minijump repair: a jump pressed while we're still grounded AFTER prediction means
	// the engine ate the press - it refuses to jump while FL_DUCKING is set or the unduck
	if (Vars::Misc::Movement::MiniJumpQueue.Value && bGroundPre && bGroundPost && (pCmd->buttons & IN_JUMP))
	{
		m_iMiniJumpPendingUntil = I::GlobalVars->tickcount + 40;
		pCmd->buttons &= ~IN_JUMP;
	}

	// Detect a real jump takeoff.
	const bool bJumpingOff = pLocal->m_vecVelocity().z > 100.f || (pCmd->buttons & IN_JUMP);
	if (bGroundPre && !bGroundPost && bJumpingOff && m_iAssistJumpType == JT_NONE)
	{
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

	if (pCmd->buttons & IN_JUMP)
		m_iMiniJumpLastJumpCmdTick = I::GlobalVars->tickcount;
}

// Pre-prediction half of the jump->minijump repair: re-press a parked jump before prediction.
void CMisc::MiniJumpPre(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	if (!Vars::Misc::Movement::MiniJump.Value || !Vars::Misc::Movement::MiniJumpQueue.Value)
	{
		m_iMiniJumpPendingUntil = 0;
		return;
	}

	const bool bGround = pLocal->m_fFlags() & FL_ONGROUND;

	// Stale-clock guard: tickcount restarts on a new server/map, so tick references stored on the
	// previous connection sit far in the "future" - the bhop delay below would then read every
	const int iNow = I::GlobalVars->tickcount;
	if (m_iMiniJumpLandTick > iNow)
		m_iMiniJumpLandTick = 0;
	if (m_iMiniJumpLastJumpCmdTick > iNow)
		m_iMiniJumpLastJumpCmdTick = 0;
	if (m_iMiniJumpPendingUntil > iNow + 60)
		m_iMiniJumpPendingUntil = 0;

	// Track the landing tick for the bhop delay below.
	if (bGround && !m_bMiniJumpPrevGround)
		m_iMiniJumpLandTick = I::GlobalVars->tickcount;
	m_bMiniJumpPrevGround = bGround;

	if (!bGround)
	{
		if (pCmd->buttons & IN_JUMP)
		{
			m_iMiniJumpPendingUntil = I::GlobalVars->tickcount + 40; // plenty to land + unduck
			pCmd->buttons &= ~IN_JUMP;
		}
		return;
	}

	// Bhop delay: a jump on the landing tick (a bhop-injected one, or space held/spammed) fires
	// before the previous hop's duck state has settled and the mini comes out broken. Hold EVERY
	constexpr int iBhopDelayTicks = 2;
	if (I::GlobalVars->tickcount - m_iMiniJumpLandTick < iBhopDelayTicks)
	{
		if (pCmd->buttons & IN_JUMP)
		{
			m_iMiniJumpPendingUntil = I::GlobalVars->tickcount + 40;
			pCmd->buttons &= ~IN_JUMP;
		}
		return;
	}

	if (!m_iMiniJumpPendingUntil)
		return;

	if (I::GlobalVars->tickcount > m_iMiniJumpPendingUntil)
	{
		m_iMiniJumpPendingUntil = 0; // engine never came unblocked in time - drop the press
		return;
	}

	const bool bDuckBlocked = (pLocal->m_fFlags() & FL_DUCKING) || pLocal->m_bDucking() || pLocal->m_flDuckJumpTime() > 0.f;
	const bool bReleaseBlocked = I::GlobalVars->tickcount - m_iMiniJumpLastJumpCmdTick <= 1;
	if (bDuckBlocked || bReleaseBlocked)
	{
		pCmd->buttons &= ~IN_JUMP; // still waiting: keep the press parked
		return;
	}

	pCmd->buttons |= IN_JUMP;
	m_iMiniJumpPendingUntil = 0;
}

// Returns true when the player is actively steering AWAY from the given wall - i.e. their raw
static bool TB_SteeringAwayFromWall(const CUserCmd* pCmd, const Vec3& vWallNormal,
	float flForwardMove, float flSideMove, float flMinDot = 0.3f)
{
	if (flForwardMove == 0.f && flSideMove == 0.f)
		return false; // no movement keys held -> not trying to leave

	Vec3 vForward, vRight;
	Math::AngleVectors(Vec3(0.f, pCmd->viewangles.y, 0.f), &vForward, &vRight, nullptr);

	Vec3 vWish = vForward * flForwardMove + vRight * flSideMove;
	vWish.z = 0.f;
	const float flWishLen = vWish.Length2D();
	if (flWishLen < 1.f)
		return false;
	vWish = vWish * (1.f / flWishLen);

	Vec3 vNormal = vWallNormal;
	vNormal.z = 0.f;
	const float flNormLen = vNormal.Length2D();
	if (flNormLen < 0.001f)
		return false;
	vNormal = vNormal * (1.f / flNormLen);

	// The wall normal points out of the wall toward the player, so a positive dot means the wish
	// direction heads outward. flMinDot ignores into-wall and parallel holds but catches a clear
	return vWish.Dot(vNormal) > flMinDot;
}

// Auto Align - nudge movement into a nearby wall to set up / hold a pixel surf.
// Only engages while airborne AND a near-vertical wall is within flWallRange units
void CMisc::AutoAlign(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	EnginePredictionBatch_t tPredictionBatch;

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

		// Upright, flat brush wall only - reject ramps (normal.z != 0), passable trigger volumes and
		// displacements (a disp can be locally near-vertical but never holds a pixel surf).
		if (trace.fraction != 1.f && trace.plane.normal.z == 0.f
			&& !(trace.surface.flags & SURF_TRIGGER) && !trace.IsDispSurface())
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

	// Seamless leave: if the player is actively steering off the wall, don't force their movement
	// back into it - hand control straight back this tick. Without this AutoAlign re-grabs the wall
	if (TB_SteeringAwayFromWall(pCmd, trace.plane.normal, pCmd->forwardmove, pCmd->sidemove))
	{
		m_bWallDetected = false;
		return;
	}

	// Calculate movement direction towards wall
	Vec3 vNormalPlane = Vec3(trace.plane.normal.x * -0.005f, trace.plane.normal.y * -0.005f, 0.f);
	Vec3 vWallAngle = Math::VectorAngles(vNormalPlane);
	Math::ClampAngles(vWallAngle);

	float flRotation = DEG2RAD(vWallAngle.y - pCmd->viewangles.y);
	float flCosRot = cosf(flRotation);
	float flSinRot = sinf(flRotation);

	// Try to find movement that maintains pixel surf velocity.
	static auto sv_gravity = H::ConVars.FindVar("sv_gravity");
	const float flSurfZ = (sv_gravity ? sv_gravity->GetFloat() : 800.f) * I::GlobalVars->interval_per_tick * 0.5f;

	// Pin the prediction frame (same fix as TextureBug/SimJumpReach): a bare
	const int nBasePredicted = I::Prediction->m_nCommandsPredicted;
	const auto RestoreOriginal = [&]()
	{
		I::Prediction->RestoreEntityToPredictedFrame(nBasePredicted - 1);
	};

	// The hold search used to sweep ONLY the pure into-wall direction, discarding any along-wall
	// input - once pinned you couldn't adjust your position on the surf, just hope or bail
	const float flUserFwd = pCmd->forwardmove;
	const float flUserSide = pCmd->sidemove;

	Vec3 vFwdDir, vRightDir;
	Math::AngleVectors(Vec3(0.f, pCmd->viewangles.y, 0.f), &vFwdDir, &vRightDir, nullptr);
	Vec3 vWishWorld = vFwdDir * flUserFwd + vRightDir * flUserSide;
	vWishWorld.z = 0.f;

	Vec3 vInto = Vec3(-trace.plane.normal.x, -trace.plane.normal.y, 0.f);
	vInto.Normalize();
	const Vec3 vTangentDir = Vec3(trace.plane.normal.y, -trace.plane.normal.x, 0.f);
	const float flTangentInput = std::clamp(vWishWorld.Dot(vTangentDir), -450.f, 450.f);

	bool bDetect = false;
	for (int iPass = 0; iPass < 2 && !bDetect; iPass++)
	{
		const float flTangent = iPass == 0 ? flTangentInput : 0.f;
		if (iPass == 0 && fabsf(flTangent) < 10.f)
			continue; // no meaningful along-wall input - just run the pure hold pass

		for (float flMultiplier = 0.f; flMultiplier < 100.f; flMultiplier += 10.f)
		{
			RestoreOriginal();

			// Candidate move in world space: press into the wall + the player's lateral slide
			// (mult 0 + tangent = drifting along the surf without pressing in, also valid).
			const Vec3 vMove = vInto * flMultiplier + vTangentDir * flTangent;
			const float flMag = vMove.Length2D();
			if (flMag < 0.01f)
			{
				pCmd->forwardmove = 0.f;
				pCmd->sidemove = 0.f;
			}
			else
			{
				Vec3 vMoveAngle = Math::VectorAngles(vMove);
				const float flMoveRot = DEG2RAD(vMoveAngle.y - pCmd->viewangles.y);
				pCmd->forwardmove = cosf(flMoveRot) * flMag;
				pCmd->sidemove = -sinf(flMoveRot) * flMag;
			}

			F::EnginePrediction.Simulate(pLocal, pCmd);

			if (fabsf(pLocal->m_vecVelocity().z + flSurfZ) < 1.5f) // tick-correct pixel surf velocity
			{
				bDetect = true;
				break;
			}
		}
	}

	// Always hand the entity back un-simulated: leaving it one predicted tick ahead poisoned
	// every live-velocity read in the features that run after us (TextureBug's locked-band
	RestoreOriginal();

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
	EnginePredictionBatch_t tPredictionBatch;

	m_bPixelSurfingNow = false; // recomputed each tick from live velocity (drives the "ps" indicator)

	// Half-gravity per tick (negative once we add it to vz) - the velocity the engine pins you
	// to while riding a pixel surf.
	static auto sv_gravity = H::ConVars.FindVar("sv_gravity");
	const float flHalfGrav = (sv_gravity ? sv_gravity->GetFloat() : 800.f) * I::GlobalVars->interval_per_tick * 0.5f;
	constexpr float flSurfTol = 1.5f;

	// Ride detection FIRST, before any feature gate: the "ps" indicator lights when you are
	// ACTUALLY riding a surf, not when the auto-catcher merely decides to crouch, and it works
	if (pLocal->IsAlive() && pLocal->m_MoveType() != MOVETYPE_NOCLIP && pLocal->m_MoveType() != MOVETYPE_LADDER)
	{
		const bool bSurfVel = fabsf(m_vPrePredictionVelocity.z + flHalfGrav) < flSurfTol
			&& (!(m_iPrePredictionFlags & FL_ONGROUND) || m_bShouldPixelSurf);
		m_bPixelSurfingNow = bSurfVel && m_bPrevSurfVel;
		m_bPrevSurfVel = bSurfVel;
	}
	else
		m_bPrevSurfVel = false;

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

	// NOTE: do NOT gate on m_bWallDetected here. That flag is only set by
	// AutoAlign (and only when AutoAlign is enabled), so requiring it meant

	if (!m_bShouldPixelSurf)
	{
		// Pixel surfs only catch on perfectly-upright, flat brush walls. The duck prediction
		// below can momentarily hit the surf z-velocity next to a displacement (a disp is built
		{
			constexpr float flWallRange = 10.f;          // max distance from the hull to count as "next to a wall"
			constexpr float flStep = (PI * 2.f) / 16.f;  // sweep resolution
			constexpr float flUprightEps = 0.0001f;        // brush wall faces are exactly vertical (normal.z == 0)

			const Vec3 vStartPos = pLocal->GetAbsOrigin();
			const Vec3 vMins = pLocal->m_vecMins();
			const Vec3 vMaxs = pLocal->m_vecMaxs();
			CTraceFilterWorldAndPropsOnly filter;

			bool bUprightWall = false;
			for (float a = 0.f; a < PI * 2.f && !bUprightWall; a += flStep)
			{
				Vec3 vEndPos = vStartPos;
				vEndPos.x += cosf(a) * flWallRange;
				vEndPos.y += sinf(a) * flWallRange;

				CGameTrace trace;
				SDK::TraceHull(vStartPos, vEndPos, vMins, vMaxs, MASK_PLAYERSOLID, &filter, &trace);

				if (trace.fraction >= 1.f || (trace.surface.flags & SURF_TRIGGER))
					continue;
				if (trace.IsDispSurface())
					continue; // displacements aren't flat walls - pixel surfs don't hold on them
				if (fabsf(trace.plane.normal.z) < flUprightEps)
					bUprightWall = true;
			}

			if (!bUprightWall)
				return; // no flat upright wall to surf - don't crouch

			// Reject inclines (ramps / stairs). A real wall pixel-surf pins you to the SIDE of an
			// upright face with nothing solid under your feet - you fall slowly alongside it. A ramp
			{
				constexpr float flFloorRange = 18.f;   // step height - stairs/ramp underfoot, but ignores distant real ground
				constexpr float flUprightEps = 0.0001f; // a vertical face below (wall continuing down) is fine - not a floor

				const Vec3 vDownStart = pLocal->GetAbsOrigin();
				const Vec3 vDownEnd = vDownStart - Vec3(0.f, 0.f, flFloorRange);
				CTraceFilterWorldAndPropsOnly downFilter;
				CGameTrace downTrace;
				SDK::TraceHull(vDownStart, vDownEnd, pLocal->m_vecMins(), pLocal->m_vecMaxs(), MASK_PLAYERSOLID, &downFilter, &downTrace);

				if (downTrace.fraction < 1.f && !(downTrace.surface.flags & SURF_TRIGGER)
					&& fabsf(downTrace.plane.normal.z) > flUprightEps)
					return; // floor/ramp/step beneath - incline, not a wall pixel surf
			}
		}

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
				m_bShouldPixelSurf = fabsf(flZVelo + flHalfGrav) < flSurfTol; // tick-rate-correct (was exact -6.25, never matched on TF2)
				
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

		// staying pinned must not depend on continuously holding a
		// key into the wall - releasing everything keeps you on the surf, and only actively
		if (pCmd->forwardmove == 0.f && pCmd->sidemove == 0.f)
		{
			constexpr float flHoldStep = (PI * 2.f) / 16.f;
			const Vec3 vHoldStart = pLocal->GetAbsOrigin();
			CTraceFilterWorldAndPropsOnly holdFilter;
			for (float a = 0.f; a < PI * 2.f; a += flHoldStep)
			{
				const Vec3 vHoldEnd = vHoldStart + Vec3(cosf(a) * 5.f, sinf(a) * 5.f, 0.f);
				CGameTrace holdTrace;
				SDK::TraceHull(vHoldStart, vHoldEnd, pLocal->m_vecMins(), pLocal->m_vecMaxs(), MASK_PLAYERSOLID, &holdFilter, &holdTrace);
				if (holdTrace.fraction >= 1.f || fabsf(holdTrace.plane.normal.z) > 0.1f
					|| (holdTrace.surface.flags & SURF_TRIGGER))
					continue;

				Vec3 vIntoAngle = Math::VectorAngles(Vec3(-holdTrace.plane.normal.x, -holdTrace.plane.normal.y, 0.f));
				Math::ClampAngles(vIntoAngle);
				const float flHoldRot = DEG2RAD(vIntoAngle.y - pCmd->viewangles.y);
				pCmd->forwardmove = cosf(flHoldRot) * 10.f;
				pCmd->sidemove = -sinf(flHoldRot) * 10.f;
				break;
			}
		}

		if (I::GlobalVars->tickcount > m_iPixelSurfTicks)
		{
			if (fabsf(pLocal->m_vecVelocity().z + flHalfGrav) >= flSurfTol) // surf lost (was exact -6.25, never matched on TF2)
				m_bShouldPixelSurf = false;
		}
	}
}

// Epsilon from the movement math (surface extension).
static constexpr float TB_EPSILON = 0.03125f;
// Maximum Simulate() calls per bruteforce search.
static constexpr int   TB_SIM_BUDGET = 24;

// The wall sweeps (TB_DetectWall) all step through the same 16 fixed angles.
// Precompute their cos/sin once instead of recomputing transcendentals every sweep.
struct SweepDir_t { float c, s; };
constexpr float TB_SWEEP_STEP = (PI * 2.f) / 16.f;
const SweepDir_t* TB_SweepDirs16()
{
    static SweepDir_t s_aDirs[16];
    static bool s_bInit = false;
    if (!s_bInit)
    {
        for (int k = 0; k < 16; k++)
        {
            const float a = float(k) * TB_SWEEP_STEP;
            s_aDirs[k] = { cosf(a), sinf(a) };
        }
        s_bInit = true;
    }
    return s_aDirs;
}

// Negative half-gravity per tick, computed from the live sv_gravity cvar.
// E.g. 800 grav, 1/66 tick → -(800 * (1/66) * 0.5) ≈ -6.06
static float TB_HalfGravityPerTick()
{
    static auto sv_gravity = H::ConVars.FindVar("sv_gravity");
    const float flGravity = sv_gravity ? sv_gravity->GetFloat() : 800.f;
    return -(flGravity * I::GlobalVars->interval_per_tick * 0.5f);
}

// Returns true when post-simulate vz matches a pixelsurf catch (vz ≈ -half_gravity = -6.06)
// flHalf is the value from TB_HalfGravityPerTick() (negative).
static bool TB_IsPixelSurfVel(float vz, float flHalf, float tolerance = 2.f)
{
    return fabsf(vz - flHalf) < tolerance;
}

// Returns true when vz matches the positive-half-gravity value (HeadSurf catch: vz ≈ +6.06)
// flHalf is the value from TB_HalfGravityPerTick() (negative), so we negate it here.
static bool TB_IsPositiveHalfGravity(float vz, float flHalf, float tolerance = 2.f)
{
    return fabsf(vz + flHalf) < tolerance;
}

// Projection of the player half-extents onto the wall normal.
float TB_WallSupportDistance(CTFPlayer* pLocal, const Vec3& vWallNormal)
{
    const Vec3 vMins = pLocal->m_vecMins();
    const Vec3 vMaxs = pLocal->m_vecMaxs();
    const Vec3 vHalf(std::max(fabsf(vMins.x), fabsf(vMaxs.x)),
        std::max(fabsf(vMins.y), fabsf(vMaxs.y)),
        std::max(fabsf(vMins.z), fabsf(vMaxs.z)));
    return fabsf(vWallNormal.x) * vHalf.x + fabsf(vWallNormal.y) * vHalf.y + fabsf(vWallNormal.z) * vHalf.z;
}

// Single hull-trace sweep for the CLOSEST (near-)vertical wall around us. One hull trace
// per angle covers the whole player height. Filter ignores studio model props (which these
static bool TB_DetectWall(const Vec3& vStart, const Vec3& vMins, const Vec3& vMaxs,
                           float flStep, CGameTrace& outTrace, float& outAngle, float flCachedAngle = -1.f,
                           float flReach = 24.f)
{
    (void)flStep; // swept via fixed 16-angle table; kept for call-site compatibility
    CTraceFilterWorldAndBrushOnly filter;

    const auto TestDir = [&](float c, float s, CGameTrace& tr) -> bool
    {
        const Vec3 vEnd = vStart + Vec3(c * flReach, s * flReach, 0.f);
        SDK::TraceHull(vStart, vEnd, vMins, vMaxs, MASK_PLAYERSOLID, &filter, &tr);
        return tr.fraction < 1.f
            && fabsf(tr.plane.normal.z) <= 0.10f       // near-vertical only
            && !(tr.surface.flags & SURF_TRIGGER);     // skip walk-through brushes
    };

    // Fast path: re-test last tick's winning wall angle.
    if (flCachedAngle >= 0.f)
    {
        CGameTrace tr;
        if (TestDir(cosf(flCachedAngle), sinf(flCachedAngle), tr))
        {
            outTrace = tr;
            outAngle = flCachedAngle;
            return true;
        }
    }

    // Full sweep for the closest valid wall (precomputed 16-angle dir table).
    const SweepDir_t* pDirs = TB_SweepDirs16();
    float flBestFraction = 1.f;
    bool bHit = false;
    for (int k = 0; k < 16; k++)
    {
        CGameTrace trace;
        if (!TestDir(pDirs[k].c, pDirs[k].s, trace))
            continue;
        if (trace.fraction < flBestFraction)
        {
            flBestFraction = trace.fraction;
            outTrace = trace;
            outAngle = float(k) * TB_SWEEP_STEP;
            bHit = true;
        }
    }
    return bHit;
}

// Traces upward to detect a ceiling brush within flReach units above the player's head.
static bool TB_DetectCeiling(const Vec3& vPos, const Vec3& vMins, const Vec3& vMaxs,
                              float flReach, CGameTrace& traceOut)
{
    Vec3 vStart = vPos;
    Vec3 vEnd   = vPos;
    vEnd.z     += flReach;

    Ray_t ray;
    ray.Init(vStart, vEnd, vMins, vMaxs);
    CTraceFilterWorldAndPropsOnly filter;
    I::EngineTrace->TraceRay(ray, MASK_PLAYERSOLID, &filter, &traceOut);

    return traceOut.DidHit() && traceOut.plane.normal.z < -0.7f;
}

// Gap between the player hull and the wall plane (smaller == closer to flush). Pure plane
// math instead of a hull trace: subtracts the hull half-extent projected on the normal.
float TB_SupportDistance(CTFPlayer* pLocal, const CGameTrace& primary)
{
    const Vec3 vOrigin = pLocal->GetAbsOrigin();
    const Vec3& n = primary.plane.normal;
    const float flPlaneDist = vOrigin.x * n.x + vOrigin.y * n.y + vOrigin.z * n.z - primary.plane.dist;
    const Vec3 vWallNormal(n.x, n.y, 0.f);
    return fabsf(flPlaneDist) - TB_WallSupportDistance(pLocal, vWallNormal);
}
 
// Scans the wall face vertically with a thin probe to find the brush segment above the head.
static bool TB_MeasureBrushFace(const Vec3& vOrigin, const Vec3& vMins, const Vec3& vMaxs,
                                const CGameTrace& primary, float flReach,
                                float& outBottomZ, float& outHeight)
{
    (void)vMins; // hull top (vMaxs.z) is what crosses brush bottoms; mins kept for call-site symmetry
    CTraceFilterWorldAndBrushOnly filter;
    const Vec3& n = primary.plane.normal;
    const float flIntoLen = sqrtf(n.x * n.x + n.y * n.y);
    if (flIntoLen < 0.01f)
        return false;
    const Vec3 vDir(-n.x / flIntoLen, -n.y / flIntoLen, 0.f); // horizontal unit dir into the wall
    const Vec3 vProbeMin(-2.f, -2.f, -1.f), vProbeMax(2.f, 2.f, 1.f); // thin probe (near a line)

    const float flLow  = vOrigin.z - 8.f;
    const float flHigh = vOrigin.z + vMaxs.z + 16.f;
    constexpr float flZStep = 4.f;
    constexpr int kMaxSamples = 48;

    struct Sample { float z; bool present; short props; float dist; };
    Sample aSamples[kMaxSamples];
    int nSamples = 0;
    for (float z = flLow; z <= flHigh && nSamples < kMaxSamples; z += flZStep)
    {
        const Vec3 vStart(vOrigin.x, vOrigin.y, z);
        const Vec3 vEnd = vStart + vDir * flReach;
        CGameTrace tr;
        SDK::TraceHull(vStart, vEnd, vProbeMin, vProbeMax, MASK_PLAYERSOLID, &filter, &tr);
        Sample& s = aSamples[nSamples++];
        s.z = z;
        s.present = tr.fraction < 1.f && fabsf(tr.plane.normal.z) <= 0.10f && !(tr.surface.flags & SURF_TRIGGER);
        s.props = s.present ? tr.surface.surfaceProps : (short)-1;
        s.dist  = s.present ? tr.plane.dist : 0.f;
    }
    if (nSamples < 2)
        return false;

    // Anchor on the present sample nearest the hull top: that is where the head crosses brush
    // bottoms as we fall, so it is the brush segment whose bottom edge actually matters.
    const float flHeadZ = vOrigin.z + vMaxs.z;
    int iAnchor = -1;
    float flBestDz = FLT_MAX;
    for (int i = 0; i < nSamples; i++)
    {
        if (!aSamples[i].present)
            continue;
        const float dz = fabsf(aSamples[i].z - flHeadZ);
        if (dz < flBestDz) { flBestDz = dz; iAnchor = i; }
    }
    if (iAnchor < 0)
        return false; // no wall present anywhere in the window

    // Grow the contiguous same-surface segment around the anchor (matching surfaceProps AND plane).
    const short refProps = aSamples[iAnchor].props;
    const float refDist  = aSamples[iAnchor].dist;
    const auto SameFace = [&](const Sample& s) -> bool
    {
        return s.present && s.props == refProps && fabsf(s.dist - refDist) <= 2.f;
    };
    int iBottom = iAnchor, iTop = iAnchor;
    while (iBottom - 1 >= 0 && SameFace(aSamples[iBottom - 1])) iBottom--;
    while (iTop + 1 < nSamples && SameFace(aSamples[iTop + 1])) iTop++;

    outBottomZ = aSamples[iBottom].z;
    outHeight  = aSamples[iTop].z - aSamples[iBottom].z + flZStep; // +step so the span includes the sample gaps
    return true;
}

// Refresh the shared per-tick wall cache (call at start of each feature if not set yet).
void CMisc::TB_RefreshWallCache(CTFPlayer* pLocal)
{
    if (m_TB_WallCache.valid)
        return;
 
    const Vec3 vPos  = pLocal->GetAbsOrigin();
    const Vec3 vMins = pLocal->m_vecMins();
    const Vec3 vMaxs = pLocal->m_vecMaxs();
    constexpr float flStep = (PI * 2.f) / 16.f;
    const float flScanStart = (m_flTBCachedWallAngle >= 0.f) ? m_flTBCachedWallAngle : 0.f;
 
    CGameTrace trace;
    float foundAngle = 0.f;
    m_TB_WallCache.valid = TB_DetectWall(vPos, vMins, vMaxs, flStep, trace, foundAngle, flScanStart);
    if (m_TB_WallCache.valid)
    {
        m_TB_WallCache.trace      = trace;
        m_TB_WallCache.foundAngle = foundAngle;
        m_flTBCachedWallAngle     = foundAngle;
    }
    else
    {
        m_flTBCachedWallAngle = -1.f;
    }
}
 
// -- TextureBug ---------------------------------------------------------------
 
void CMisc::TextureBug(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	EnginePredictionBatch_t tPredictionBatch; // the sweeps fire many Simulates - by far the hottest batch

	const bool caught_last_tick = m_bTextureBugHit; // last tick's result (fast-path + coast gate)
	m_bTextureBugHit = false;
	m_bTBOwnsCmd = false;
	// Nothing rides a persistent lock any more - clear stale lock state so nothing downstream thinks
	// we are mid-ride.
	m_bTBLocked = false; m_bTBArrestLock = false; m_iTBPendingCatchTicks = 0;

	// consecutive hit ticks - the coast gate: only established rides may coast through misses
	static int ride_ticks = 0;
	ride_ticks = caught_last_tick ? ride_ticks + 1 : 0;

	// ride fast path: last committed plain catch (world yaw + forward + duck), retested first
	static float fast_yaw  = 0.f;
	static float fast_fwd  = 0.f;
	static bool  fast_duck = false;

	// duck state of the last commit - flips only when the current state loses the pin (flapping the
	// hull every few ticks made rides jitter and randomly crouch/uncrouch).
	static bool  ride_duck = false;

	static bool  duck_extension_pending      = false;
	static bool  duck_extension_holding      = false;
	static int   duck_extension_wait_ticks   = 0;
	static float duck_extension_yaw          = 0.f;
	static float duck_extension_forward_move = 0.f;
	static float duck_extension_side_move    = 0.f;
	static int   duck_extension_stand_buttons= 0;
	static int   duck_extension_duck_buttons = 0;

	// ride hold (coyote time): last catch move in the wall frame, replayed when the wall scan drops
	// out at a brush seam so the ride carries across to the next surface.
	static int   ride_hold_ticks_left    = 0;
	static float ride_hold_yaw           = 0.f;
	static float ride_hold_forward_move  = 0.f;
	static float ride_hold_side_move     = 0.f;
	static int   ride_hold_buttons       = 0;

	// edge stop: brake/hold at the end of a surf instead of sliding off
	static bool  edge_braking = false;
	if (!caught_last_tick) edge_braking = false;

	// creep lock: the seam look-ahead saw a pin k ticks out on the current creep move - replay it
	// verbatim until then instead of re-ranking every tick.
	static int   creep_lock_ticks   = 0;
	static float creep_lock_yaw     = 0.f;
	static float creep_lock_fwd     = 0.f;
	static int   creep_lock_buttons = 0;
	if (caught_last_tick) creep_lock_ticks = 0; // riding - the lock is approach-only

	const auto reset_duck_extension = [&]() noexcept
	{
		duck_extension_pending = false;
		duck_extension_holding = false;
		duck_extension_wait_ticks = 0;
		duck_extension_yaw = 0.f;
		duck_extension_forward_move = 0.f;
		duck_extension_side_move = 0.f;
		duck_extension_stand_buttons = 0;
		duck_extension_duck_buttons = 0;
	};

	// Set the hit flag and arm the choke-tick option on a real catch so the choke sliders keep working.
	const auto SetHit = [&](bool b)
	{
		m_bTextureBugHit = b;
		if (b && Vars::Misc::Movement::TextureBugChokeTick.Value && m_iTBChokeTicksLeft <= 0)
			m_iTBChokeTicksLeft = std::clamp(Vars::Misc::Movement::TextureBugChokeTicks.Value, 1, 14);
	};

	if (!Vars::Misc::Movement::TextureBug.Value)
		{ reset_duck_extension(); ride_hold_ticks_left = 0; creep_lock_ticks = 0; return; }
	if (!pLocal || !pLocal->IsAlive())
		{ reset_duck_extension(); ride_hold_ticks_left = 0; creep_lock_ticks = 0; return; }

	const int iMoveType = pLocal->m_MoveType();
	if (iMoveType == MOVETYPE_LADDER || iMoveType == MOVETYPE_NOCLIP
		|| iMoveType == MOVETYPE_FLY || iMoveType == MOVETYPE_OBSERVER)
		{ reset_duck_extension(); ride_hold_ticks_left = 0; creep_lock_ticks = 0; return; }

	const bool auto_crouch = Vars::Misc::Movement::TextureBugAutoCrouch.Value;
	const bool edge_stop   = Vars::Misc::Movement::TextureBugEdgeStop.Value;
	const int  hold_ticks  = std::clamp(Vars::Misc::Movement::TextureBugHoldTicks.Value, 0, 16);

	const int   original_buttons      = pCmd->buttons;
	const float original_forward_move = pCmd->forwardmove;
	const float original_side_move    = pCmd->sidemove;
	const bool  local_on_ground       = (pLocal->m_fFlags() & FL_ONGROUND) != 0;
	const bool  user_holding_jump     = (original_buttons & IN_JUMP) != 0;
	const bool  user_has_manual_movement_input = fabsf(original_forward_move) > 0.01f || fabsf(original_side_move) > 0.01f;

	if (local_on_ground && !user_holding_jump)
		{ reset_duck_extension(); ride_hold_ticks_left = 0; creep_lock_ticks = 0; return; }

	// half-gravity pin velocity (the "landed on a surface this tick" z-vel), cached for the sweep
	const float flTargetVz  = F::EnginePrediction.GetTargetPredictZVelocity();
	const float k_catch_eps = Vars::Misc::Movement::TextureBugCatchEps.Value;
	// a catch pins vz to -half; the +half sign is a "snapped onto a top surface" hit (tighter band)
	const auto is_pinned = [&](float vz, float eps) noexcept -> bool
	{
		return fabsf(vz - flTargetVz) <= eps || fabsf(vz + flTargetVz) <= std::min(eps, 0.35f);
	};

	// hard Simulate budget - the sweep can fan out to hundreds of sims in the worst tick
	int tb_sims = 0;
	constexpr int k_tb_sim_cap = 384;
	const auto sim      = [&](CUserCmd* c) { ++tb_sims; F::EnginePrediction.Simulate(pLocal, c); };
	const auto sims_left = [&]() noexcept { return k_tb_sim_cap - tb_sims; };

	// move expressed in the WALL frame -> the player command frame (cmd forward/side)
	const auto wall_to_player_fwd = [&](float wall_yaw_deg, float fwd, float side) noexcept -> float
	{
		const float delta = DEG2RAD(Math::NormalizeAngle(wall_yaw_deg - pCmd->viewangles.y));
		return fwd * cosf(delta) + side * sinf(delta);
	};
	const auto wall_to_player_side = [&](float wall_yaw_deg, float fwd, float side) noexcept -> float
	{
		const float delta = DEG2RAD(Math::NormalizeAngle(wall_yaw_deg - pCmd->viewangles.y));
		return -fwd * sinf(delta) + side * cosf(delta);
	};

	// arm the ride hold with the move that caught this tick (also the single place the ride duck
	// state is updated - every commit path passes through here)
	const auto arm_ride_hold = [&](float yaw, float fwd, float side, int buttons)
	{
		ride_duck = (buttons & IN_DUCK) != 0;
		if (hold_ticks <= 0) { ride_hold_ticks_left = 0; return; }
		ride_hold_ticks_left   = hold_ticks;
		ride_hold_yaw          = yaw;
		ride_hold_forward_move = fwd;
		ride_hold_side_move    = side;
		ride_hold_buttons      = buttons;
	};

	const Vec3 vLiveVelocity = pLocal->m_vecVelocity();

	// HeadSurf runs after us, so m_bHeadSurfHit still holds last tick's result here. If an
	// established head surf is still pinned (+half) it re-locks this tick - a wall commit would
	if (m_bHeadSurfHit && fabsf(vLiveVelocity.z + flTargetVz) <= k_catch_eps)
	{
		reset_duck_extension();
		return;
	}

	constexpr float k_pi   = 3.14159265358979323846f;
	const float max_radius = k_pi * 2.f;
	const float step       = max_radius / 16.f;
	const Vec3  start_pos  = pLocal->GetAbsOrigin();
	const Vec3  mins       = pLocal->m_vecMins();
	const Vec3  maxs       = pLocal->m_vecMaxs();
	static float start_circle = 0.f;
	bool wall_detected = false;
	CGameTrace primary;

	// per-Z line-trace scan across 16 angles for the closest near-vertical, player-facing wall.
	// hull_only = probe mode: steering only needs the yaw, so the hull normal is enough and we skip
	const auto detect_wall = [&](const Vec3& sp, const Vec3& mn, const Vec3& mx, float scan_start,
	                             CGameTrace& out_trace, float& out_angle, bool hull_only = false) -> bool
	{
		const float extra_reach          = Vars::Misc::Movement::TextureBugReach.Value;
		const float z_step               = Vars::Misc::Movement::TextureBugScanStep.Value > 0.5f ? Vars::Misc::Movement::TextureBugScanStep.Value : 1.f;
		constexpr float normal_z_epsilon = 0.05f;
		const float half_x = std::max(fabsf(mn.x), fabsf(mx.x));
		const float half_y = std::max(fabsf(mn.y), fabsf(mx.y));
		CTraceFilterWorldAndBrushOnly fil;

		for (float a = scan_start; a < scan_start + max_radius; a += step)
		{
			const float angle = fmodf(a, max_radius);
			const float dir_x = cosf(angle);
			const float dir_y = sinf(angle);
			const float ax = std::max(fabsf(dir_x), 1e-6f);
			const float ay = std::max(fabsf(dir_y), 1e-6f);
			const float box_reach  = std::min(half_x / ax, half_y / ay);
			const float ray_length = box_reach + extra_reach;
			const float z_start = sp.z + mn.z;
			const float z_end   = sp.z + mx.z;

			// hull prefilter: one sweep answers "anything at this angle?" before the ~40 per-Z lines
			CGameTrace pre;
			Ray_t pray; pray.Init(sp, Vec3(sp.x + dir_x * ray_length, sp.y + dir_y * ray_length, sp.z), mn, mx);
			I::EngineTrace->TraceRay(pray, MASK_PLAYERSOLID, &fil, &pre);
			if (!pre.DidHit())
				continue;

			if (hull_only)
			{
				if (fabsf(pre.plane.normal.z) <= normal_z_epsilon)
				{
					const float pre_facing = pre.plane.normal.x * dir_x + pre.plane.normal.y * dir_y;
					if (pre_facing <= -0.1f) { out_trace = pre; out_angle = a; return true; }
				}
				continue;
			}

			CGameTrace best_trace; best_trace.fraction = 1.f;
			bool hit_at_this_angle = false;
			for (float z = z_start; z <= z_end; z += z_step)
			{
				const Vec3 ray_start(sp.x, sp.y, z);
				const Vec3 ray_end(sp.x + dir_x * ray_length, sp.y + dir_y * ray_length, z);
				Ray_t ray; ray.Init(ray_start, ray_end, Vec3(0.f, 0.f, 0.f), Vec3(0.f, 0.f, 0.f));
				CGameTrace tr;
				I::EngineTrace->TraceRay(ray, MASK_PLAYERSOLID, &fil, &tr);
				if (tr.fraction >= 1.f) continue;
				if (fabsf(tr.plane.normal.z) > normal_z_epsilon) continue;
				const float facing = tr.plane.normal.x * dir_x + tr.plane.normal.y * dir_y;
				if (facing > -0.1f) continue;
				hit_at_this_angle = true;
				if (tr.fraction < best_trace.fraction) best_trace = tr;
			}
			if (hit_at_this_angle) { out_trace = best_trace; out_angle = a; return true; }

			// short wall that slipped between the scan lines: the line grid missed but the hull hit a
			// near-vertical facing wall - take it.
			if (fabsf(pre.plane.normal.z) <= normal_z_epsilon)
			{
				const float pre_facing = pre.plane.normal.x * dir_x + pre.plane.normal.y * dir_y;
				if (pre_facing <= -0.1f) { out_trace = pre; out_angle = a; return true; }
			}
		}
		return false;
	};

	float found_angle = 0.f;
	if (detect_wall(start_pos, mins, maxs, start_circle, primary, found_angle))
	{
		wall_detected = true;
		start_circle  = found_angle;
	}
	if (!wall_detected)
	{
		start_circle = 0.f;
		reset_duck_extension();
		creep_lock_ticks = 0; // the locked move is wall-frame - wall gone, lock is dead
		// ride hold: scan gap at a brush seam - replay the last catch move so the ride carries over.
		if (ride_hold_ticks_left > 0)
		{
			--ride_hold_ticks_left;
			SetHit(true);
			pCmd->forwardmove = wall_to_player_fwd(ride_hold_yaw, ride_hold_forward_move, ride_hold_side_move);
			pCmd->sidemove    = wall_to_player_side(ride_hold_yaw, ride_hold_forward_move, ride_hold_side_move);
			pCmd->buttons     = ride_hold_buttons;
		}
		// live pin but no wall in the scan grid (flush overlap / geometry the angle grid misses):
		// a catch is happening right now, so hold it instead of dropping the cmd. Try the user's own
		else if (is_pinned(vLiveVelocity.z, k_catch_eps))
		{
			const int nBaseNW = I::Prediction->m_nCommandsPredicted;
			bool held = false;
			for (int i = 0; i < 2 && !held; ++i)
			{
				if (i == 0 && !user_has_manual_movement_input) continue;
				I::Prediction->RestoreEntityToPredictedFrame(nBaseNW - 1);
				pLocal->SetAbsOrigin(start_pos);
				pLocal->m_vecOrigin()   = start_pos;
				pLocal->m_vecVelocity() = vLiveVelocity;
				CUserCmd test = *pCmd;
				if (i == 1) { test.forwardmove = 0.f; test.sidemove = 0.f; }
				sim(&test);
				if (is_pinned(pLocal->m_vecVelocity().z, k_catch_eps))
				{
					SetHit(true);
					pCmd->forwardmove = test.forwardmove;
					pCmd->sidemove    = test.sidemove;
					held = true;
				}
			}
			I::Prediction->RestoreEntityToPredictedFrame(nBaseNW - 1);
		}
		// look-ahead: wall on the fall path but not beside us yet -> steer now so we arrive flush.
		// Only when hands-off and edge stop is on (this is a manufactured approach, the grabby part).
		else if (!local_on_ground && !user_has_manual_movement_input && edge_stop)
		{
			const float dt = I::GlobalVars->interval_per_tick;
			// probe at a fixed fall DISTANCE (not a fixed tick count - a fixed count overshoots small
			// ledges at high fall speed). Cycle three distances so tiny ledges and fast falls both get
			const float fall_speed = std::max(-vLiveVelocity.z, 50.f);
			const float per_tick   = fall_speed * dt;
			const int   di         = pCmd->command_number % 3;
			const float dist = di == 0 ? 12.f
			                 : (di == 1 ? std::max(26.f, per_tick * 6.f)
			                            : std::max(45.f, per_tick * 11.f));
			float t = dist / fall_speed;
			t = dist / (fall_speed + 400.f * t);
			t = std::min(std::max(t / dt, 2.f), 12.f) * dt;
			Vec3 probe = start_pos + vLiveVelocity * t;
			probe.z -= 400.f * t * t; // 0.5 * g * t^2
			CGameTrace ahead; float ahead_angle = 0.f;
			if (detect_wall(probe, mins, maxs, 0.f, ahead, ahead_angle, true))
			{
				Vec3 ahead_wall = Math::VectorAngles(ahead.plane.normal * -1.f);
				Math::ClampAngles(ahead_wall);
				pCmd->forwardmove = wall_to_player_fwd(ahead_wall.y, 450.f, 0.f);
				pCmd->sidemove    = wall_to_player_side(ahead_wall.y, 450.f, 0.f);
				m_bTBOwnsCmd = true;
			}
		}
		return;
	}

	// pin the base prediction frame; every probe restores to it (Simulate drifts m_nCommandsPredicted
	// forward, so a bare RestoreEntityToPredicted would creep). RAII guard restores on every exit.
	const int nBasePredicted = I::Prediction->m_nCommandsPredicted;
	const auto RestoreOriginal = [&]() { I::Prediction->RestoreEntityToPredictedFrame(nBasePredicted - 1); };
	struct StateGuard { int base; ~StateGuard() { I::Prediction->RestoreEntityToPredictedFrame(base - 1); } } prediction_state_guard{ nBasePredicted };

	const auto apply_predicted_state = [&]() noexcept
	{
		pLocal->SetAbsOrigin(start_pos);
		pLocal->m_vecOrigin()   = start_pos;
		pLocal->m_vecVelocity() = vLiveVelocity;
	};

	const Vec3 approach_normal = primary.plane.normal * -1.f;
	Vec3 angle_wall = Math::VectorAngles(approach_normal);
	Math::ClampAngles(angle_wall);

	// z of the wall face for the support line.
	float support_z = primary.endpos.z;
	{
		CTraceFilterWorldAndBrushOnly zfil;
		const Vec3 into = primary.plane.normal * -1.f * 32.f;
		float best_fr = 2.f; // > 1 so the primary z wins only by actually hitting
		const float z_cands[6] = { primary.endpos.z,
		                           start_pos.z + mins.z + 2.f,  start_pos.z + mins.z + 20.f,
		                           start_pos.z + mins.z + 41.f, start_pos.z + mins.z + 62.f,
		                           start_pos.z + maxs.z - 2.f };
		for (float zc : z_cands)
		{
			Ray_t ray; ray.Init(Vec3(start_pos.x, start_pos.y, zc),
			                    Vec3(start_pos.x + into.x, start_pos.y + into.y, zc),
			                    Vec3(0.f, 0.f, 0.f), Vec3(0.f, 0.f, 0.f));
			CGameTrace tr;
			I::EngineTrace->TraceRay(ray, MASK_ALL, &zfil, &tr);
			if (tr.fraction < 1.f && tr.fraction < best_fr && fabsf(tr.plane.normal.z) <= 0.05f)
				{ best_fr = tr.fraction; support_z = zc; }
		}
	}

	// support gap: a line trace along -normal minus the hull's projected extent
	const auto get_support_gap = [&]() -> float
	{
		const Vec3 minusplane = primary.plane.normal * -1.f * 32.f;
		const Vec3 origin = pLocal->GetAbsOrigin();
		Ray_t ray; ray.Init(Vec3(origin.x, origin.y, support_z),
		                    Vec3(origin.x + minusplane.x, origin.y + minusplane.y, support_z),
		                    Vec3(0.f, 0.f, 0.f), Vec3(0.f, 0.f, 0.f));
		CTraceFilterWorldAndBrushOnly wlft;
		CGameTrace trace_support;
		I::EngineTrace->TraceRay(ray, MASK_ALL, &wlft, &trace_support);
		const Vec3 wall_normal(primary.plane.normal.x, primary.plane.normal.y, 0.f);
		return trace_support.fraction * 32.f - TB_WallSupportDistance(pLocal, wall_normal);
	};

	const float live_gap = get_support_gap(); // gap before any sim (entity is still live here)

	float gap               = 1.f;
	float best_forward_move = pCmd->forwardmove;
	float best_side_move    = pCmd->sidemove;
	float best_yaw          = angle_wall.y;
	int   best_buttos       = pCmd->buttons;
	int   best_hit_streak   = 0;
	int   best_sustain      = -1; // sim look-ahead streak of the best pin so far (the ranker)
	bool  best_is_duck_extension = false;
	int   best_stand_buttons = pCmd->buttons;
	int   best_duck_buttons  = pCmd->buttons | IN_DUCK;
	const bool manual_duck_held = (original_buttons & IN_DUCK) != 0;

	const float vel_yaw = RAD2DEG(atan2f(vLiveVelocity.y, vLiveVelocity.x));
	const float rel     = Math::NormalizeAngle(vel_yaw - angle_wall.y);
	const float sign    = (rel >= 0.f) ? -1.f : 1.f;

	static constexpr float k_forward_move_logged_values[] = { 0.125f, 0.875f, 0.750f, 0.5f, 1.f, 2.125f, 2.875f, 5.f, 15.f, 30.f };
	constexpr float k_duck_extension_gap_tolerance = 0.05f;
	constexpr int   k_duck_extension_delay_ticks       = 0;

	// swap a move that predicts an air-stuck (2D speed < 1) for its reverse if that keeps more speed
	const auto fix_air_stuck_move = [&]() noexcept
	{
		const float inward_fwd  = pCmd->forwardmove;
		const float inward_side = pCmd->sidemove;
		if (fabsf(inward_fwd) < 0.01f && fabsf(inward_side) < 0.01f)
			return;

		RestoreOriginal();
		apply_predicted_state();

		CUserCmd test_cmd = *pCmd;
		sim(&test_cmd);
		const float inward_speed = pLocal->m_vecVelocity().Length2D();
		if (inward_speed >= 1.f) { RestoreOriginal(); apply_predicted_state(); return; }

		RestoreOriginal();
		apply_predicted_state();
		test_cmd = *pCmd;
		test_cmd.forwardmove = -inward_fwd;
		test_cmd.sidemove    = -inward_side;
		sim(&test_cmd);
		const float outward_speed = pLocal->m_vecVelocity().Length2D();
		// a committed pin has to survive the swap - reversing a stationary hold's wish can drift past
		// the eps band and drop the surf.
		const bool outward_pin = is_pinned(pLocal->m_vecVelocity().z, k_catch_eps);
		RestoreOriginal();
		apply_predicted_state();
		if (outward_speed > inward_speed + 0.01f && (!m_bTextureBugHit || outward_pin))
		{
			pCmd->forwardmove = -inward_fwd;
			pCmd->sidemove    = -inward_side;
		}
	};

	// edge stop: the surf ends along the ride direction -> brake so we stop on the brush instead of
	// sliding off. Brake rate = airaccel 10 * wishcap 30 = 300 u/s^2, so stop dist = v^2/600. Runs
	const auto apply_edge_stop = [&]()
	{
		if (!edge_stop)
			return;
		// near-zero speed the sweep can miss the pin (sims < 1 u/s are rejected) - a live pin plus the
		// brake state keeps the governor owning the cmd so the hold does not drift off the edge.
		const bool live_pinned = is_pinned(vLiveVelocity.z, k_catch_eps);
		if (!m_bTextureBugHit && !(edge_braking && live_pinned))
			return;

		const auto wall_at = [&](const Vec3& p) -> bool
		{
			const Vec3 into_wall = primary.plane.normal * -32.f;
			CTraceFilterWorldAndBrushOnly efil;
			Ray_t ray; ray.Init(Vec3(p.x, p.y, support_z),
			                    Vec3(p.x + into_wall.x, p.y + into_wall.y, support_z),
			                    Vec3(0.f, 0.f, 0.f), Vec3(0.f, 0.f, 0.f));
			CGameTrace tr;
			I::EngineTrace->TraceRay(ray, MASK_PLAYERSOLID, &efil, &tr);
			return tr.fraction < 1.f && fabsf(tr.plane.normal.z) <= 0.05f;
		};

		const auto move_keeps_pin = [&](float fwd, float side) -> bool
		{
			CUserCmd test = *pCmd;
			test.forwardmove = fwd;
			test.sidemove    = side;
			RestoreOriginal(); apply_predicted_state();
			sim(&test);
			const bool pinned = is_pinned(pLocal->m_vecVelocity().z, k_catch_eps);
			RestoreOriginal(); apply_predicted_state();
			return pinned;
		};

		const float speed_2d = vLiveVelocity.Length2D();

		if (speed_2d < 1.f)
		{
			// stopped at the edge: hold with a zero wish while the pin lasts
			if (!edge_braking)
				return;
			if (!move_keeps_pin(0.f, 0.f))
				return; // zero wish drops the pin -> the picked ride move stands
			pCmd->forwardmove = 0.f;
			pCmd->sidemove    = 0.f;
			SetHit(true);
			return;
		}

		const Vec3 ride_dir(vLiveVelocity.x / speed_2d, vLiveVelocity.y / speed_2d, 0.f);
		const float stop_dist = speed_2d * speed_2d / 600.f + speed_2d * I::GlobalVars->interval_per_tick * 3.f + 2.f;
		if (wall_at(start_pos + ride_dir * stop_dist))
		{
			edge_braking = false; // the surf continues past the stopping distance
			return;
		}

		// the edge is inside the stopping distance -> full brake against horizontal velocity
		const float brake_yaw  = RAD2DEG(atan2f(-vLiveVelocity.y, -vLiveVelocity.x));
		const float brake_fwd  = wall_to_player_fwd(brake_yaw, 450.f, 0.f);
		const float brake_side = wall_to_player_side(brake_yaw, 450.f, 0.f);
		if (!move_keeps_pin(brake_fwd, brake_side))
			return; // the brake drops the pin -> the ride move stands
		edge_braking = true;
		pCmd->forwardmove = brake_fwd;
		pCmd->sidemove    = brake_side;
		SetHit(true);
	};

	const auto simulate_variant = [&](float wall_yaw, float forward_move, bool force_duck,
	                                  float& out_gap, int& out_hit_streak, int& out_buttons, bool& out_is_headbounce,
	                                  bool allow_retry) -> bool
	{
		// reject defaults FIRST - an early return must not leave the caller's 0.0 in out_gap (a 0.0
		// wins the no-catch ranking and fakes the duck near-miss gate = a dead move gets committed).
		out_gap = FLT_MAX;
		out_hit_streak = 0;
		out_is_headbounce = false;

		RestoreOriginal();
		apply_predicted_state();

		pCmd->buttons = original_buttons;
		if (force_duck) pCmd->buttons |= IN_DUCK;
		pCmd->forwardmove = wall_to_player_fwd(wall_yaw, forward_move, 0.f);
		pCmd->sidemove    = wall_to_player_side(wall_yaw, forward_move, 0.f);

		const float pre_vel_z = pLocal->m_vecVelocity().z;

		sim(pCmd);

		float post_vel_z = pLocal->m_vecVelocity().z;
		const float predicted_speed_2d = pLocal->m_vecVelocity().Length2D();
		bool pinned = is_pinned(post_vel_z, k_catch_eps);
		// the dead-wedge guard only applies to NON-pins: a pinned stationary hold is a valid catch
		// (edge stop holds exactly this state).
		if (!pinned && predicted_speed_2d < 1.f)
			return false;
		out_gap = get_support_gap();
		bool delayed = false;
		if (allow_retry && !pinned && pre_vel_z < -1.0f)
		{
			// fast fall: the pin often lands one tick late - but retest only when the eps band is
			// reachable next tick, otherwise a blind retry doubles every failing sweep sim.
			const Vec3 v_post = pLocal->m_vecVelocity();
			const float v_in_post = std::max(0.f, v_post.x * approach_normal.x + v_post.y * approach_normal.y);
			if (out_gap <= 0.03125f + (v_in_post + 30.f) * I::GlobalVars->interval_per_tick)
			{
				sim(pCmd);
				post_vel_z = pLocal->m_vecVelocity().z;
				pinned = is_pinned(post_vel_z, k_catch_eps);
				delayed = pinned;
			}
		}

		if (pinned)
		{
			// first pin wins - ride depth comes from the fast path + ride hold, not a per-variant loop
			out_hit_streak = 1;
			// were rising and got pinned tight = the engine snapped us onto a top surface
			if (!delayed && pre_vel_z > 0.0f && F::EnginePrediction.IsTargetPredictZVelocity(post_vel_z, 0.1f))
				out_is_headbounce = true;
		}
		out_buttons = pCmd->buttons;
		return true;
	};

	// duck-extension replay owns the cmd - the old approach ran the full sweep first and threw it away
	if (!manual_duck_held && (duck_extension_holding || duck_extension_pending))
	{
		const bool started_falling_from_stand = !F::EnginePrediction.IsTargetPredictZVelocity(vLiveVelocity.z, k_catch_eps) && vLiveVelocity.z < -1.0f;
		SetHit(true);
		pCmd->forwardmove = wall_to_player_fwd(duck_extension_yaw, duck_extension_forward_move, duck_extension_side_move);
		pCmd->sidemove    = wall_to_player_side(duck_extension_yaw, duck_extension_forward_move, duck_extension_side_move);
		if (duck_extension_holding)
		{
			pCmd->buttons = started_falling_from_stand ? duck_extension_stand_buttons : duck_extension_duck_buttons;
			fix_air_stuck_move();
			arm_ride_hold(duck_extension_yaw, duck_extension_forward_move, duck_extension_side_move, pCmd->buttons);
			if (started_falling_from_stand) reset_duck_extension();
		}
		else
		{
			pCmd->buttons = duck_extension_stand_buttons;
			fix_air_stuck_move();
			arm_ride_hold(duck_extension_yaw, duck_extension_forward_move, duck_extension_side_move, pCmd->buttons);
			if (duck_extension_wait_ticks > 0) --duck_extension_wait_ticks;
			else { duck_extension_pending = false; duck_extension_holding = true; }
		}
		apply_edge_stop();
		return;
	}

	// live stick: the live vz is already pinned = a texture bug is in progress right now, however it
	// started (own jump, any combo, no combo). Hold it: the user's cmd if it sustains, else the tiny
	{
		const bool live_pin_now = F::EnginePrediction.IsTargetPredictZVelocity(vLiveVelocity.z, k_catch_eps)
		                       || fabsf(vLiveVelocity.z + flTargetVz) <= k_catch_eps;
		if (live_pin_now)
		{
			struct StickTry { float fwd; int buttons; bool user_cmd; };
			StickTry tries[5]; int n_tries = 0;
			// mid-ride: keep the current duck state on every try. A duck flip is a fresh-catch thing
			// only; flapping the hull mid-surf jittered rides.
			const bool riding = caught_last_tick;
			const int  duck_keep = (riding && ride_duck && auto_crouch && !manual_duck_held) ? IN_DUCK : 0;
			if (user_has_manual_movement_input)
				tries[n_tries++] = { 0.f, original_buttons | duck_keep, true };
			if (duck_keep)
				tries[n_tries++] = { 0.1f, original_buttons | IN_DUCK, false };
			tries[n_tries++] = { 0.1f, original_buttons, false };
			if (!duck_keep && !riding && !manual_duck_held && auto_crouch)
				tries[n_tries++] = { 0.1f, original_buttons | IN_DUCK, false };
			tries[n_tries++] = { 0.f, original_buttons | duck_keep, false };
			for (int i = 0; i < n_tries; ++i)
			{
				RestoreOriginal();
				apply_predicted_state();
				CUserCmd test = *pCmd;
				test.buttons = tries[i].buttons;
				if (!tries[i].user_cmd)
				{
					test.forwardmove = wall_to_player_fwd(angle_wall.y, tries[i].fwd, 0.f);
					test.sidemove    = wall_to_player_side(angle_wall.y, tries[i].fwd, 0.f);
				}
				sim(&test);
				const bool pin = is_pinned(pLocal->m_vecVelocity().z, k_catch_eps);
				RestoreOriginal();
				apply_predicted_state();
				if (!pin) continue;
				SetHit(true);
				reset_duck_extension();
				pCmd->forwardmove = test.forwardmove;
				pCmd->sidemove    = test.sidemove;
				pCmd->buttons     = test.buttons;
				if (tries[i].user_cmd)
				{
					fast_fwd = 0.f; // the user owns this ride
					arm_ride_hold(pCmd->viewangles.y, original_forward_move, original_side_move, test.buttons);
				}
				else
				{
					if (tries[i].fwd > 0.f)
					{
						fast_yaw  = angle_wall.y;
						fast_fwd  = tries[i].fwd;
						fast_duck = (test.buttons & IN_DUCK) != 0;
					}
					arm_ride_hold(angle_wall.y, tries[i].fwd, 0.f, test.buttons);
				}
				apply_edge_stop();
				return;
			}
		}
	}

	// natural hit: the user's own cmd already pins -> keep it, no bruteforce. Manual input only -
	// hands-off pins go to the sweep, which verifies durability; a 1-tick natural fluke here poisons
	if (!caught_last_tick && user_has_manual_movement_input)
	{
		RestoreOriginal();
		apply_predicted_state();
		CUserCmd natural = *pCmd;
		sim(&natural);
		const bool nat_pin = is_pinned(pLocal->m_vecVelocity().z, k_catch_eps);
		RestoreOriginal();
		apply_predicted_state();
		if (nat_pin)
		{
			SetHit(true);
			reset_duck_extension();
			fast_fwd = 0.f; // the user owns this ride - don't chase a stale sweep move next tick
			arm_ride_hold(pCmd->viewangles.y, original_forward_move, original_side_move, original_buttons);
			apply_edge_stop();
			return;
		}
	}

	// ride fast path: the last winning move still pins -> commit it and skip the sweep. As the band
	// drifts mid-ride: exact retest first, then a 2-tick retest (delayed pin), then small yaw nudges.
	if (fast_fwd > 0.f && caught_last_tick && !manual_duck_held)
	{
		static constexpr float k_fp_nudges[] = { 0.f, 0.f, -0.5f, 0.5f, -1.f, 1.f };
		const int n_fp_slots = (ride_ticks >= 2) ? 6 : 2; // nudge-chase only real rides
		for (int ni = 0; ni < n_fp_slots; ++ni)
		{
			const bool fp_retry = (ni == 1); // slot 1 = same yaw, 2-tick retest
			float fp_gap = 0.f; int fp_streak = 0; int fp_buttons = original_buttons; bool fp_hb = false;
			simulate_variant(fast_yaw + k_fp_nudges[ni], fast_fwd, fast_duck,
			                 fp_gap, fp_streak, fp_buttons, fp_hb, fp_retry);
			if (fp_streak > 0)
			{
				fast_yaw += k_fp_nudges[ni];
				SetHit(true);
				pCmd->forwardmove = wall_to_player_fwd(fast_yaw, fast_fwd, 0.f);
				pCmd->sidemove    = wall_to_player_side(fast_yaw, fast_fwd, 0.f);
				pCmd->buttons     = fp_buttons;
				fix_air_stuck_move();
				arm_ride_hold(fast_yaw, fast_fwd, 0.f, pCmd->buttons);
				apply_edge_stop();
				return;
			}
		}
	}

	// (mul x ang) grid, deduped by the effective wish against the wall: inward = mul*cos(ang) drives
	// the band stepping, along = mul*sin(ang) drives ride accel. Low-mul rows are near-identical
	struct TbVariant { float ang, mul; };
	static const auto build_variants = [](TbVariant* out, int cap, float ang_start, float ang_end, float ang_step) -> int
	{
		int n = 0, n_seen = 0;
		long long seen[96];
		for (float mul : k_forward_move_logged_values)
			for (float ang = ang_start; ang < ang_end && n < cap; ang += ang_step)
			{
				const float r = DEG2RAD(ang);
				const long long key = (long long)floorf(mul * cosf(r) / 0.03f) * 4096
				                    + (long long)floorf(mul * sinf(r) / 0.25f);
				bool dup = false;
				for (int i = 0; i < n_seen; i++) if (seen[i] == key) { dup = true; break; }
				if (dup) continue;
				if (n_seen < 96) seen[n_seen++] = key;
				out[n++] = { ang, mul };
			}
		return n;
	};
	static TbVariant s_main_vars[96], s_wig_vars[48];
	static const int s_n_main = build_variants(s_main_vars, 96, 85.999f, 90.f, 0.5f);
	static const int s_n_wig  = build_variants(s_wig_vars,  48, 90.5f, 94.f, 1.f);

	// streak-ranked picker: one sim per variant, but a pinning move is scored by how many consecutive
	// ticks it re-clips the seam in sim (each caught tick ends vz = -half, so sustain = a move whose
	static constexpr int k_streak_depth = 12; // sustain look-ahead ticks (tuning lever)
	static constexpr int k_early_accept = 12; // a full-depth streak stops the sweep - this is the ride
	float last_eval_gap = FLT_MAX;
	const auto eval_variant = [&](float current_yaw, float mul, bool force_duck, bool allow_retry) -> bool
	{
		float p = FLT_MAX; int streak = 0; int btn = original_buttons; bool hb = false;
		const bool ok = simulate_variant(current_yaw, mul, force_duck, p, streak, btn, hb, allow_retry);
		last_eval_gap = ok ? p : FLT_MAX;
		if (streak > 0)
		{
			// count consecutive sustain ticks (state + pCmd continue from the simulate)
			int sustain = 0;
			for (int t = 0; t < k_streak_depth && sims_left() > 0; ++t)
			{
				sim(pCmd);
				if (!is_pinned(pLocal->m_vecVelocity().z, k_catch_eps))
					break;
				++sustain;
			}
			// rank: longer sustain wins; equal sustain -> tighter gap
			if (sustain > best_sustain || (sustain == best_sustain && p < gap))
			{
				best_sustain       = sustain;
				gap                = p;
				best_hit_streak    = 1;
				best_forward_move  = mul;
				best_side_move     = 0.f;
				best_yaw           = current_yaw;
				best_buttos        = btn;
				best_stand_buttons = btn & ~IN_DUCK;
				best_duck_buttons  = btn | IN_DUCK;
			}
			return sustain >= k_early_accept; // fully sustaining -> abort the sweep
		}
		if (ok && !force_duck && best_hit_streak == 0 && p < gap)
		{
			gap                = p;
			best_forward_move  = mul;
			best_side_move     = 0.f;
			best_yaw           = current_yaw;
			best_buttos        = btn;
			best_stand_buttons = btn;
			best_duck_buttons  = btn | IN_DUCK;
		}
		return false;
	};

	// a catch needs the perpendicular speed < eps; the sweep pins within 2 sim ticks max, so gate on
	// the closable distance. Closing speed = existing velocity into the wall (uncapped) + accel cap 30.
	const float v_toward_wall = std::max(0.f, vLiveVelocity.x * approach_normal.x + vLiveVelocity.y * approach_normal.y);
	const bool sweep_viable = live_gap <= 1.0f + (v_toward_wall + 30.f) * 2.f * I::GlobalVars->interval_per_tick;
	bool winner = false;

	if (sweep_viable)
	{
		// pass 1: stand sweep, 1 sim each; pool the closest misses for the retry pass. Fast fall = the
		// pin nearly always lands one tick late (band crossed mid-tick), so pool more there.
		const int n_pool = (vLiveVelocity.z < -400.f) ? 12 : 8;
		int retry_idx[12]; float retry_gap[12]; int n_retry = 0;
		for (int vi = 0; vi < s_n_main && !winner && sims_left() > 0; ++vi)
		{
			winner = eval_variant(angle_wall.y + s_main_vars[vi].ang * sign, s_main_vars[vi].mul, false, false);
			if (!winner && last_eval_gap < FLT_MAX)
			{
				if (n_retry < n_pool || last_eval_gap < retry_gap[n_pool - 1])
				{
					int j = (n_retry < n_pool) ? n_retry++ : n_pool - 1;
					while (j > 0 && retry_gap[j - 1] > last_eval_gap)
						{ retry_gap[j] = retry_gap[j - 1]; retry_idx[j] = retry_idx[j - 1]; --j; }
					retry_gap[j] = last_eval_gap; retry_idx[j] = vi;
				}
			}
		}
		if (best_hit_streak > 0) winner = true;

		// pass 2: fast-fall pins land one tick late - 2-tick retest, closest misses only
		if (!winner && vLiveVelocity.z < -1.f)
			for (int i = 0; i < n_retry && !winner && sims_left() > 0; ++i)
				winner = eval_variant(angle_wall.y + s_main_vars[retry_idx[i]].ang * sign,
				                      s_main_vars[retry_idx[i]].mul, false, true);

		// pass 3: a rising headbounce needs duck (the stand hull can't snap onto the top side).
		// mid-ride only when already riding ducked - no stand-ride duck flips.
		if (!winner && !manual_duck_held && auto_crouch && vLiveVelocity.z > 0.f
			&& (!caught_last_tick || ride_duck))
			for (int vi = 0; vi < s_n_main && !winner && sims_left() > 0; ++vi)
				winner = eval_variant(angle_wall.y + s_main_vars[vi].ang * sign, s_main_vars[vi].mul, true, false);

		// pass 4: wiggle - flush at exactly eps never catches; a tiny outward drift re-enters the band.
		if (!winner && live_gap < 0.5f)
			for (int vi = 0; vi < s_n_wig && !winner && sims_left() > 0; ++vi)
				winner = eval_variant(angle_wall.y + s_wig_vars[vi].ang * sign, s_wig_vars[vi].mul, false,
				                      vLiveVelocity.z < -1.f);

		// pass 4.5: falling duck probe - short ledges: the duck hull (62 vs 82) shrinks player height
		// in the catch equation, and an air FinishDuck shifts the origin +20u instantly = a different
		if (!winner && !manual_duck_held && auto_crouch && vLiveVelocity.z < -1.f
			&& !caught_last_tick
			&& live_gap < 0.75f)
		{
			if (gap < 0.5f)
				winner = eval_variant(best_yaw, best_forward_move, true, true);
			for (int i = 0; i < n_retry && i < 4 && !winner && sims_left() > 0; ++i)
				winner = eval_variant(angle_wall.y + s_main_vars[retry_idx[i]].ang * sign,
				                      s_main_vars[retry_idx[i]].mul, true, true);
		}

		if (best_hit_streak > 0) winner = true;

		// pass 5: one duck probe of the stand winner -> a duck extension when duck also pins at about
		// the same gap. Fresh catch only: mid-ride re-plans re-ran the pending->holding choreography.
		if (winner && ride_ticks == 0 && !manual_duck_held && auto_crouch && !(best_buttos & IN_DUCK))
		{
			const float win_gap = gap;
			float dp = FLT_MAX; int ds = 0; int db = original_buttons; bool dh = false;
			simulate_variant(best_yaw, best_forward_move, true, dp, ds, db, dh, false);
			if (ds > 0 && dp <= win_gap + k_duck_extension_gap_tolerance)
			{
				best_is_duck_extension = true;
				best_duck_buttons = db;
			}
		}
	}

	// pin coast: a mid-ride sweep miss (band drift / seam) - replay the last catch move for up to the
	// hold ticks; the pin usually re-lands within 1-2 ticks and dropping to creep killed the ride.
	if (!winner && caught_last_tick && ride_ticks >= 2 && ride_hold_ticks_left > 0)
	{
		--ride_hold_ticks_left;
		SetHit(true);
		reset_duck_extension();
		pCmd->forwardmove = wall_to_player_fwd(ride_hold_yaw, ride_hold_forward_move, ride_hold_side_move);
		pCmd->sidemove    = wall_to_player_side(ride_hold_yaw, ride_hold_forward_move, ride_hold_side_move);
		pCmd->buttons     = ride_hold_buttons;
		apply_edge_stop();
		return;
	}

	if (manual_duck_held)
	{
		SetHit(best_hit_streak > 0);
		reset_duck_extension();
	}

	if (best_is_duck_extension && !manual_duck_held)
	{
		SetHit(best_hit_streak > 0);

		const bool plan_changed = fabsf(duck_extension_yaw - best_yaw) > 0.001f ||
		                          fabsf(duck_extension_forward_move - best_forward_move) > 0.001f ||
		                          fabsf(duck_extension_side_move - best_side_move) > 0.001f ||
		                          duck_extension_stand_buttons != best_stand_buttons || duck_extension_duck_buttons != best_duck_buttons;

		if (plan_changed || (!duck_extension_pending && !duck_extension_holding))
		{
			duck_extension_pending       = true;
			duck_extension_holding       = false;
			duck_extension_wait_ticks    = k_duck_extension_delay_ticks;
			duck_extension_yaw           = best_yaw;
			duck_extension_forward_move  = best_forward_move;
			duck_extension_side_move     = best_side_move;
			duck_extension_stand_buttons = best_stand_buttons;
			duck_extension_duck_buttons  = best_duck_buttons;
		}

		pCmd->forwardmove = wall_to_player_fwd(best_yaw, best_forward_move, best_side_move);
		pCmd->sidemove    = wall_to_player_side(best_yaw, best_forward_move, best_side_move);
		pCmd->buttons     = best_stand_buttons;
		fix_air_stuck_move();
		if (m_bTextureBugHit)
			arm_ride_hold(best_yaw, best_forward_move, best_side_move, pCmd->buttons);
		apply_edge_stop();
		return;
	}

	reset_duck_extension();
	SetHit(best_hit_streak > 0);

	// seam look-ahead: the sweep only sees 1-2 ticks, but falling fast the seam is often 3-7 ticks
	if (!m_bTextureBugHit && creep_lock_ticks == 0 && vLiveVelocity.z < -1.f
		&& live_gap < 0.5f && gap < 1.f && sims_left() >= 8)
	{
		RestoreOriginal();
		apply_predicted_state();
		CUserCmd look = *pCmd;
		look.buttons     = best_buttos;
		look.forwardmove = wall_to_player_fwd(best_yaw, best_forward_move, 0.f);
		look.sidemove    = wall_to_player_side(best_yaw, best_forward_move, 0.f);
		for (int k = 1; k <= 7 && sims_left() > 0; ++k)
		{
			sim(&look);
			if (is_pinned(pLocal->m_vecVelocity().z, k_catch_eps))
			{
				creep_lock_ticks   = k;
				creep_lock_yaw     = best_yaw;
				creep_lock_fwd     = best_forward_move;
				creep_lock_buttons = best_buttos;
				break;
			}
		}
		RestoreOriginal();
		apply_predicted_state();
	}
	if (!m_bTextureBugHit && creep_lock_ticks > 0)
	{
		--creep_lock_ticks;
		pCmd->forwardmove = wall_to_player_fwd(creep_lock_yaw, creep_lock_fwd, 0.f);
		pCmd->sidemove    = wall_to_player_side(creep_lock_yaw, creep_lock_fwd, 0.f);
		pCmd->buttons     = creep_lock_buttons;
		m_bTBOwnsCmd = true;
		return;
	}

	// no catch + user strafing + still far -> hand the cmd back, the user owns the approach. A NEAR
	// wall creep stays owned even against input: it is the multi-tick band setup (inside eps + tiny
	if (!m_bTextureBugHit && user_has_manual_movement_input && live_gap > 0.75f)
	{
		pCmd->forwardmove = original_forward_move;
		pCmd->sidemove    = original_side_move;
		pCmd->buttons     = original_buttons;
		return;
	}

	// no catch + still gapped + hands off -> drive straight at the wall (a near-tangent creep is too
	if (!m_bTextureBugHit && live_gap > 1.0f && !user_has_manual_movement_input && edge_stop)
	{
		const float drive_fwd = std::min(std::max(live_gap * 18.f, 120.f), 450.f);
		pCmd->forwardmove = wall_to_player_fwd(angle_wall.y, drive_fwd, 0.f);
		pCmd->sidemove    = wall_to_player_side(angle_wall.y, drive_fwd, 0.f);
		pCmd->buttons     = original_buttons;
		if (!manual_duck_held)
			pCmd->buttons &= ~IN_DUCK; // the catch was computed standing
		m_bTBOwnsCmd = true;
		return;
	}

	pCmd->forwardmove = wall_to_player_fwd(best_yaw, best_forward_move, best_side_move);
	pCmd->sidemove    = wall_to_player_side(best_yaw, best_forward_move, best_side_move);
	pCmd->buttons     = best_buttos;
	m_bTBOwnsCmd = true; // creep = the multi-tick band setup, AirStuck must not stomp it
	fix_air_stuck_move();
	if (m_bTextureBugHit)
	{
		arm_ride_hold(best_yaw, best_forward_move, best_side_move, pCmd->buttons);
		fast_yaw  = best_yaw;
		fast_fwd  = best_forward_move;
		fast_duck = (best_buttos & IN_DUCK) != 0;
	}
	apply_edge_stop();
}
  
// -- HeadSurf -----------------------------------------------------------------
  
// Runs as part of TextureBug (same toggle): when a brush bottom is just above your head, find
// the silent move that pins vz to +halfgravity.
void CMisc::HeadSurf(CTFPlayer* pLocal, CUserCmd* pCmd)
{
    EnginePredictionBatch_t tPredictionBatch;

    m_bHeadSurfHit = false;

    if (!Vars::Misc::Movement::HeadSurf.Value || !pLocal || !pLocal->IsAlive())
    {
        m_bHSLocked = false;
        m_flHSCachedForward = -1.f;
        return;
    }

    // TextureBug caught a wall surf this tick - its move owns the cmd, don't fight it.
    if (m_bTextureBugHit)
    {
        m_bHSLocked = false;
        return;
    }

    const float flHalfGrav = TB_HalfGravityPerTick();

    if (pLocal->m_fFlags() & FL_ONGROUND)
    {
        m_flHSCachedForward = -1.f;
        m_bHSSurfingNow     = false;
        m_bHSLocked         = false;
        return;
    }

    const int iMoveType = pLocal->m_MoveType();
    if (iMoveType == MOVETYPE_LADDER || iMoveType == MOVETYPE_NOCLIP
        || iMoveType == MOVETYPE_FLY || iMoveType == MOVETYPE_OBSERVER)
        return;

    const Vec3 vStartPos = pLocal->GetAbsOrigin();
    const Vec3 vMins     = pLocal->m_vecMins();
    const Vec3 vMaxs     = pLocal->m_vecMaxs();
    const float flReach  = m_bHSSurfingNow ? 10.f : 6.f;

    CGameTrace ceilTrace;
    if (!TB_DetectCeiling(vStartPos, vMins, vMaxs, flReach, ceilTrace))
    {
        m_flHSCachedForward = -1.f;
        m_bHSSurfingNow     = false;
        m_bHSLocked         = false;
        m_iHSColdSkip       = 0;
        return;
    }

    {
        const float flVZ    = pLocal->m_vecVelocity().z;
        const bool  bSurfVel = TB_IsPositiveHalfGravity(flVZ, flHalfGrav, 1.5f);
        m_bHSSurfingNow  = bSurfVel && m_bHSPrevSurfVel;
        m_bHSPrevSurfVel = bSurfVel;
    }

    const Vec3  vOriginalView     = pCmd->viewangles;
    const float flOriginalForward = pCmd->forwardmove;
    const float flOriginalSide    = pCmd->sidemove;
    const int   iOriginalButtons  = pCmd->buttons;

    // Locked: riding the head surf. Same zero-Simulate model as TextureBug - the engine pins the
    // real vz to +halfgravity while the surf holds, so test that and replay the move verbatim
    if (m_bHSLocked)
    {
        if (TB_IsPositiveHalfGravity(pLocal->m_vecVelocity().z, flHalfGrav, 3.f))
        {
            pCmd->viewangles  = vOriginalView;
            pCmd->forwardmove = m_flHSLockedForward;
            pCmd->sidemove    = m_flHSLockedSide;
            if (m_bHSLockedDuck) pCmd->buttons |= IN_DUCK;
            else                 pCmd->buttons &= ~IN_DUCK;
            m_bHeadSurfHit  = true;
            m_bHSSurfingNow = true;
            return;
        }
        m_bHSLocked = false; // slipped - fall through and re-catch this tick
    }

    Vec3 vCeilAngle = vOriginalView;
    {
        const Vec3 vNxy(ceilTrace.plane.normal.x, ceilTrace.plane.normal.y, 0.f);
        if (vNxy.Length2D() > 0.05f)
        {
            Vec3 vInto = vNxy * -1.f;
            vCeilAngle = Math::VectorAngles(vInto);
            Math::ClampAngles(vCeilAngle);
        }
    }

    // Pin the prediction frame (same fix as TextureBug/SimJumpReach): the bare
    // RestoreEntityToPredicted() drifts one slot forward per Simulate, so every candidate
    const int nBasePredicted = I::Prediction->m_nCommandsPredicted;
    const auto RestoreOriginal = [&]()
    {
        I::Prediction->RestoreEntityToPredictedFrame(nBasePredicted - 1);
    };

    constexpr int HS_SIM_BUDGET = 80;
    int iSimsUsed = 0;
    const auto TryConfig = [&](float flForward, float flAngleOffset, bool bDuck) -> bool
    {
        if (iSimsUsed >= HS_SIM_BUDGET) return false;
        iSimsUsed++;
        RestoreOriginal();
        Vec3 vWishAngle  = vOriginalView;
        vWishAngle.y     = vCeilAngle.y + flAngleOffset;
        pCmd->viewangles = vOriginalView;
        pCmd->buttons    = iOriginalButtons;
        if (bDuck) pCmd->buttons |= IN_DUCK;
        else       pCmd->buttons &= ~IN_DUCK;
        pCmd->forwardmove = flForward;
        pCmd->sidemove    = 0.f;
        CorrectMovement(pCmd, vWishAngle, vOriginalView);
        F::EnginePrediction.Simulate(pLocal, pCmd);
        // 0.5 tolerance to match TextureBug - the old 0.2 was tighter than the catch
        // velocity's own tick-to-tick jitter, so real catches were being rejected.
        return TB_IsPositiveHalfGravity(pLocal->m_vecVelocity().z, flHalfGrav, 0.5f);
    };

    const auto Commit = [&](float flCfgForward, float flCfgOffset, bool bDuck)
    {
        RestoreOriginal();                // pCmd already holds the winning silent move
        pCmd->viewangles        = vOriginalView;
        m_flHSCachedForward     = flCfgForward;
        m_flHSCachedAngleOffset = flCfgOffset;
        m_bHSCachedDuck         = bDuck;
        m_bHSLocked             = true;
        m_flHSLockedForward     = pCmd->forwardmove;
        m_flHSLockedSide        = pCmd->sidemove;
        m_bHSLockedDuck         = bDuck;
        m_bHeadSurfHit          = true;
        m_bHSSurfingNow         = true;
        m_iHSColdSkip           = 0;
    };

    // Fast path: last tick's winning config (1 Simulate).
    if (m_flHSCachedForward >= 0.f
        && TryConfig(m_flHSCachedForward, m_flHSCachedAngleOffset, m_bHSCachedDuck))
    {
        Commit(m_flHSCachedForward, m_flHSCachedAngleOffset, m_bHSCachedDuck);
        return;
    }

    // Throttle: a ceiling the whole sweep already missed stays un-surfable for a while; only
    // the cheap fast path above runs while cooling down (mirrors TextureBug's cold-skip).
    if (m_iHSColdSkip > 0)
    {
        m_iHSColdSkip--;
        RestoreOriginal();
        pCmd->viewangles  = vOriginalView;
        pCmd->forwardmove = flOriginalForward;
        pCmd->sidemove    = flOriginalSide;
        pCmd->buttons     = iOriginalButtons;
        return;
    }

    const float flVelYaw  = RAD2DEG(atan2f(pLocal->m_vecVelocity().y, pLocal->m_vecVelocity().x));
    const float flRelYaw  = Math::NormalizeAngle(flVelYaw - vCeilAngle.y);
    const float flSign    = (flRelYaw >= 0.f) ? -1.f : 1.f;

    static constexpr float k_hs_fwd[] = { 0.f, 0.1f, 0.25f, 0.5f, 0.75f, 1.f, 1.5f, 2.125f,
                                           2.875f, 5.f, 10.f, 20.f, 30.f };
    constexpr float flCoarseStep = 5.f, flFineStep = 1.f;

    for (float fwd : k_hs_fwd)
    {
        if (iSimsUsed >= HS_SIM_BUDGET) break;
        for (int iDuck = 0; iDuck <= 1 && iSimsUsed < HS_SIM_BUDGET; iDuck++)
        {
            const bool bDuck = (iDuck == 1);
            for (float ang = 0.f; ang < 90.f && iSimsUsed < HS_SIM_BUDGET; ang += flCoarseStep)
            {
                if (!TryConfig(fwd, ang * flSign, bDuck))
                    continue;

                // Coarse hit - refine around it for the steadiest offset, else keep the coarse one.
                for (float d = flFineStep; d <= flCoarseStep; d += flFineStep)
                {
                    for (int s = 0; s <= 1; s++)
                    {
                        const float fa = ang + (s ? d : -d);
                        if (fa < 0.f) continue;
                        if (TryConfig(fwd, fa * flSign, bDuck))
                        {
                            Commit(fwd, fa * flSign, bDuck);
                            return;
                        }
                    }
                }
                // Refinement found nothing better - re-derive the coarse hit and take it.
                if (TryConfig(fwd, ang * flSign, bDuck))
                {
                    Commit(fwd, ang * flSign, bDuck);
                    return;
                }
            }
        }
    }

    // Full miss: hand the player's movement back and cool down.
    RestoreOriginal();
    pCmd->viewangles    = vOriginalView;
    pCmd->forwardmove   = flOriginalForward;
    pCmd->sidemove      = flOriginalSide;
    pCmd->buttons       = iOriginalButtons;
    m_flHSCachedForward = -1.f;
    m_iHSColdSkip       = 4;
}
 
// -- WallClimb ----------------------------------------------------------------
 
void CMisc::WallClimb(CTFPlayer* pLocal, CUserCmd* pCmd)
{
    EnginePredictionBatch_t tPredictionBatch;

    m_bWallClimbHit = false;
 
    if (!Vars::Misc::Movement::WallClimb.Value)
    {
        m_flWallClimbCachedForward = -1.f;
        return;
    }
 
    if (!pLocal || !pLocal->IsAlive())
        return;

    // A texture-bug / head-surf catch owns this tick's move - grounding through it would drop the surf.
    if (m_bTextureBugHit || m_bHeadSurfHit)
        return;

    if (pLocal->m_fFlags() & FL_ONGROUND)
        return;

    const int iMoveType = pLocal->m_MoveType();
    if (iMoveType == MOVETYPE_LADDER || iMoveType == MOVETYPE_NOCLIP
        || iMoveType == MOVETYPE_FLY || iMoveType == MOVETYPE_OBSERVER)
        return;

    // Reuse the shared wall scan — free if TextureBug already ran it.
    TB_RefreshWallCache(pLocal);
    if (!m_TB_WallCache.valid)
    {
        m_flWallClimbCachedForward = -1.f;
        return;
    }
 
    const CGameTrace& primary = m_TB_WallCache.trace;
    const Vec3 vIntoWall      = primary.plane.normal * -1.f;
    Vec3 vWallAngle           = Math::VectorAngles(vIntoWall);
    Math::ClampAngles(vWallAngle);
 
    const Vec3  vOriginalView     = pCmd->viewangles;
    const float flOriginalForward = pCmd->forwardmove;
    const float flOriginalSide    = pCmd->sidemove;
    const int   iOriginalButtons  = pCmd->buttons;
 
    // Pin the prediction frame (same fix as TextureBug/SimJumpReach): a bare
    // RestoreEntityToPredicted() drifts one slot per Simulate, corrupting every sweep
    const int nBasePredicted = I::Prediction->m_nCommandsPredicted;
    const auto RestoreOriginal = [&]()
    {
        I::Prediction->RestoreEntityToPredictedFrame(nBasePredicted - 1);
    };

    int iSims = 0;
    const auto TryConfig = [&](float flForward, float flAngleOffset) -> bool
    {
        if (iSims >= TB_SIM_BUDGET) return false;
        iSims++;
        RestoreOriginal();
        Vec3 vWishAngle   = vOriginalView;
        vWishAngle.y      = vWallAngle.y + flAngleOffset;
        pCmd->viewangles  = vOriginalView;
        pCmd->buttons     = iOriginalButtons;
        pCmd->forwardmove = flForward;
        pCmd->sidemove    = 0.f;
        CorrectMovement(pCmd, vWishAngle, vOriginalView);
        F::EnginePrediction.Simulate(pLocal, pCmd);
        return (pLocal->m_fFlags() & FL_ONGROUND) != 0;
    };
 
    // Fast path.
    if (m_flWallClimbCachedForward >= 0.f
        && TryConfig(m_flWallClimbCachedForward, m_flWallClimbCachedAngleOffset))
    {
        RestoreOriginal();
        m_bWallClimbHit = true;
        return;
    }
 
    static constexpr float k_fwd[] = { 0.f, 0.25f, 0.5f, 0.75f, 1.f, 2.125f, 2.875f, 5.f, 15.f, 30.f };
    const Vec3  vVel     = pLocal->m_vecVelocity();
    const float flVelYaw = RAD2DEG(atan2f(vVel.y, vVel.x));
    const float flRelYaw = Math::NormalizeAngle(flVelYaw - vWallAngle.y);
    const float flSign   = (flRelYaw >= 0.f) ? -1.f : 1.f;
 
    bool  bFound      = false;
    float flBestForward = flOriginalForward, flBestSide = flOriginalSide;
    int   iBestButtons  = iOriginalButtons;
 
    for (float fwd : k_fwd)
    {
        if (iSims >= TB_SIM_BUDGET || bFound) break;
        for (float ang = 0.f; ang < 45.f && iSims < TB_SIM_BUDGET; ang += 2.f)
        {
            if (TryConfig(fwd, ang * flSign))
            {
                bFound      = true;
                flBestForward = pCmd->forwardmove;
                flBestSide    = pCmd->sidemove;
                iBestButtons  = pCmd->buttons;
                m_flWallClimbCachedForward     = fwd;
                m_flWallClimbCachedAngleOffset = ang * flSign;
                m_bWallClimbHit                = true;
                break;
            }
        }
    }
 
    RestoreOriginal();
    pCmd->viewangles = vOriginalView;

    if (bFound)
    {
        pCmd->forwardmove = flBestForward;
        pCmd->sidemove    = flBestSide;
        pCmd->buttons     = iBestButtons;
    }
    else
    {
        pCmd->forwardmove = flOriginalForward;
        pCmd->sidemove    = flOriginalSide;
        pCmd->buttons     = iOriginalButtons;
    }
}

bool CMisc::ConsumeTextureBugChoke()
{
	if (m_iTBChokeTicksLeft <= 0)
		return false;
	m_iTBChokeTicksLeft--;
	return true;
}

// -- Air stuck / Wall stuck ---------------------------------------------------
// air_stuck (any near-vertical wall) and wall_stuck (slanted-only) share one core; bSlantedOnly picks.
void CMisc::AirStuckCore(CTFPlayer* pLocal, CUserCmd* pCmd, bool bEnabled, bool bSlantedOnly,
                         bool& bHitFlag, AirStuckState_t& tState)
{
	EnginePredictionBatch_t tPredictionBatch; // the bruteforce runs many Simulates - hottest batch

	bHitFlag = false;

	const auto reset_state = [&]() { tState = AirStuckState_t{}; };

	if (!bEnabled)                         { reset_state(); return; }
	if (!pLocal || !pLocal->IsAlive())     { reset_state(); return; }

	const int iMoveType = pLocal->m_MoveType();
	if (iMoveType == MOVETYPE_LADDER || iMoveType == MOVETYPE_NOCLIP
		|| iMoveType == MOVETYPE_FLY || iMoveType == MOVETYPE_OBSERVER)
		{ reset_state(); return; }

	// Don't fight a TB/HeadSurf catch, or a TB creep that owns the cmd this tick.
	if (m_bTextureBugHit || m_bHeadSurfHit || m_bTBOwnsCmd) { reset_state(); return; }

	if (pLocal->m_fFlags() & FL_ONGROUND)  { reset_state(); return; }

	const Vec3 vOriginalView  = pCmd->viewangles;
	const Vec3 live_velocity  = pLocal->m_vecVelocity(); // backup_data.m_velocity
	const float step          = (3.14159265358979323846f * 2.f) / 16.f; // TB_DetectWall flStep (unused by it, kept for call sig)
	const Vec3  start_pos     = pLocal->GetAbsOrigin();
	const Vec3  mins          = pLocal->m_vecMins();
	const Vec3  maxs          = pLocal->m_vecMaxs();
	const int   cur_tick      = I::GlobalVars->tickcount;
	// -- Aggressiveness / perf levers --------------------------------------------------------------
	// All sliders now (Misc>Movement). Defaults match the old bumped constants.
	const float k_catch_eps    = Vars::Misc::Movement::AirStuckCatchEps.Value;  // raise = sticks more / easier
	const float k_detect_reach = Vars::Misc::Movement::AirStuckReach.Value;     // hull reach: engage walls this far out
	const float k_flush_tol    = Vars::Misc::Movement::AirStuckFlushTol.Value;  // gap > this -> drift only; <= -> catch sweep
	const float k_drift_gain   = Vars::Misc::Movement::AirStuckDriftGain.Value; // drift fwd = gap * gain (raise = faster drive-in)
	const int   k_sim_budget   = Vars::Misc::Movement::AirStuckSimBudget.Value; // hard cap on Simulate()s in the catch sweep
	// Detection now uses TB_DetectWall (the texture-bug path): ONE hull trace per angle (16 cold), and a
	// cached-angle fast path -> ~1 trace steady state, instead of the old per-Z line scan (~16x41 traces/tick)

	// Re-express a wall-frame move (forward into the wall, side along it) onto pCmd in the player's
	// real view frame so the engine produces that world wishdir without snapping the aim.
	const auto apply_move = [&](float wall_yaw, float fwd, float side)
	{
		Vec3 vWishAngle  = vOriginalView;
		vWishAngle.y     = wall_yaw;
		pCmd->viewangles = vOriginalView;
		pCmd->forwardmove = fwd;
		pCmd->sidemove    = side;
		CorrectMovement(pCmd, vWishAngle, vOriginalView);
	};

	// -- detect wall (TB_DetectWall: ONE hull trace/angle, cached-angle fast path) ------------------
	// This is the texture-bug detection path we target. Steady state is ~1 trace (re-test last
	CGameTrace primary{};
	float found_angle = tState.cached_wall_angle;
	bool wall_detected = TB_DetectWall(start_pos, mins, maxs, step, primary, found_angle,
	                                   tState.cached_wall_angle, k_detect_reach);

	// wall_stuck variant (bSlantedOnly): only slanted/diagonal vertical faces qualify - reject axis-aligned.
	const auto is_straight = [](float v) -> bool { return v == -1.f || v == 0.f || v == 1.f; };
	if (wall_detected && bSlantedOnly
		&& is_straight(primary.plane.normal.x) && is_straight(primary.plane.normal.y))
		wall_detected = false;

	if (!wall_detected)
	{
		tState.cached_wall_angle = -1.f;
		tState.start_circle      = 0.f;
		tState.stuck_hold_ticks  = 0;
		return; // not engaged - leave pCmd untouched
	}
	tState.cached_wall_angle = found_angle;

	const Vec3 approach_normal = primary.plane.normal * -1.f;
	Vec3 angle_wall = Math::VectorAngles(approach_normal);
	Math::ClampAngles(angle_wall);

	// Pin the base prediction frame; every probe restores to it (Simulate drifts m_nCommandsPredicted
	// forward, so a bare restore would creep). RAII guard restores on every exit path.
	const int nBasePredicted = I::Prediction->m_nCommandsPredicted;
	const auto RestoreOriginal = [&]() { I::Prediction->RestoreEntityToPredictedFrame(nBasePredicted - 1); };
	struct StateGuard { int base; ~StateGuard() { I::Prediction->RestoreEntityToPredictedFrame(base - 1); } } prediction_state_guard{ nBasePredicted };

	const auto apply_predicted_state = [&]() noexcept
	{
		pLocal->SetAbsOrigin(start_pos);
		pLocal->m_vecOrigin()   = start_pos;
		pLocal->m_vecVelocity() = live_velocity;
	};

	// the reference get_align: support-trace distance minus the hull's projected support distance.
	// > 0 == still a gap to the wall (approach); <= 0 == flush.
	const auto get_align = [&]() -> float
	{
		const Vec3 minusplane = primary.plane.normal * -1.f * 32.f;
		const Vec3 origin = pLocal->GetAbsOrigin();
		Ray_t ray2; ray2.Init(Vec3(origin.x, origin.y, primary.endpos.z),
		                      Vec3(origin.x + minusplane.x, origin.y + minusplane.y, primary.endpos.z),
		                      Vec3(0.f, 0.f, 0.f), Vec3(0.f, 0.f, 0.f)); // line trace
		CTraceFilterWorldAndBrushOnly wlft;
		CGameTrace trace_support;
		I::EngineTrace->TraceRay(ray2, MASK_ALL, &wlft, &trace_support);
		const Vec3 wall_normal(primary.plane.normal.x, primary.plane.normal.y, 0.f);
		return trace_support.fraction * 32.f - TB_WallSupportDistance(pLocal, wall_normal);
	};

	// Already stuck (live vz pinned to the catch target): hold last tick's winning move. Checked FIRST,
	// before any approach logic, so a successful stick is never interrupted by a re-approach.
	if (F::EnginePrediction.IsTargetPredictZVelocity(live_velocity.z, k_catch_eps))
	{
		bHitFlag = true;
		tState.stuck_hold_ticks++;
		const float hold_yaw  = (tState.stuck_last_yaw  != FLT_MAX) ? tState.stuck_last_yaw  : angle_wall.y;
		const float hold_move = (tState.stuck_last_move != FLT_MAX) ? tState.stuck_last_move : 5.f;
		const float hold_side = tState.stuck_last_side;
		apply_move(hold_yaw, hold_move, hold_side);
		return;
	}

	tState.stuck_hold_ticks = 0;

	// -- approach: still gapped -> can't catch un-flush, so just drive in HARD (0 Simulates) ---------
	// One cheap drive tick closes the gap (high gain so a fast fall reaches the wall before slipping past
	const float align = get_align();
	if (align > k_flush_tol)
	{
		const float drift_fwd = fminf(fmaxf(align * k_drift_gain, 120.f), 650.f); // [bumped min 60->120, cap 450->650: faster approach]
		apply_move(angle_wall.y, drift_fwd, 0.f);
		pCmd->buttons &= ~IN_DUCK;
		return;
	}

	// -- near flush: budgeted, coarse near-tangent catch sweep (runs EVERY tick = instant) ----------
	const float side_sign = (cur_tick % 2 == 0) ? 1.f : -1.f;

	constexpr float k_yaw_offsets[]   = { 88.0f, 88.5f, 89.0f, 89.5f, 89.9f }; // near-tangent (small |v_perp|)
	constexpr float k_forward_moves[] = { 2.f, 10.f, 60.f, 250.f, 650.f };     // tiny..slam [bumped for speed]

	int   sims_used      = 0;
	bool  found_catch    = false;
	bool  have_fallback  = false;
	float best_yaw       = angle_wall.y;
	float best_forward   = 2.f;
	float best_align_pred = FLT_MAX;

	for (const float rel_ang : k_yaw_offsets)
	{
		if (found_catch || sims_used >= k_sim_budget) break;
		for (const float fwd : k_forward_moves)
		{
			if (sims_used >= k_sim_budget) break;

			RestoreOriginal();
			apply_predicted_state();

			const float cand_yaw = Math::NormalizeAngle(angle_wall.y + rel_ang * side_sign);
			apply_move(cand_yaw, fwd, 0.f);
			pCmd->buttons &= ~IN_DUCK; // catch is computed standing (no duck-extension here)

			F::EnginePrediction.Simulate(pLocal, pCmd); sims_used++;

			if (F::EnginePrediction.IsTargetPredictZVelocity(pLocal->m_vecVelocity().z, k_catch_eps))
			{
				// Confirm: a real catch keeps pinning vz a second predicted tick (same cmd, no restore).
				F::EnginePrediction.Simulate(pLocal, pCmd); sims_used++;
				if (F::EnginePrediction.IsTargetPredictZVelocity(pLocal->m_vecVelocity().z, k_catch_eps))
				{
					best_yaw     = cand_yaw;
					best_forward = fwd;
					found_catch  = true;
					break;
				}
				continue; // single-tick blip, not a real catch
			}

			// No catch: rank by post-move flushness so the committed fallback hugs (and drives the gap shut).
			const float predicted_align = get_align();
			if (!have_fallback || predicted_align < best_align_pred)
			{
				best_align_pred = predicted_align;
				best_yaw        = cand_yaw;
				best_forward    = fwd;
				have_fallback   = true;
			}
		}
	}

	RestoreOriginal();

	// Commit the best move (the catching one if found, else the most-flush hug). Reality must match the
	// standing prediction, so clear the crouch here too.
	apply_move(best_yaw, best_forward, 0.f);
	pCmd->buttons &= ~IN_DUCK;

	if (found_catch)
	{
		bHitFlag               = true;
		tState.stuck_last_yaw  = best_yaw;
		tState.stuck_last_move = best_forward;
		tState.stuck_last_side = 0.f;
	}
}

void CMisc::AirStuck(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	// One toggle, all walls (bSlantedOnly=false). Flip to true for the wall_stuck variant (skip
	// axis-aligned faces) if a slanted-only mode is ever wanted again.
	AirStuckCore(pLocal, pCmd, Vars::Misc::Movement::AirStuck.Value, /*bSlantedOnly*/ false,
	             m_bAirStuckHit, m_AirStuckState);
}

// ----------------------------- Pixel finder -----------------------------
void CMisc::PixelFinder(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	EnginePredictionBatch_t tPredictionBatch; // column scan = probe ticks x test origins worth of Simulates

	const bool bHeld = Vars::Misc::Movement::PixelFinder.Value;
	const bool bRising = bHeld && !m_bPixelFinderHeld;
	const bool bFalling = !bHeld && m_bPixelFinderHeld;
	m_bPixelFinderHeld = bHeld;

	if (!bHeld && !bFalling)
		return;

	// Trace where we're aiming to find the wall + its normal.
	Vec3 vForward; Math::AngleVectors(pCmd->viewangles, &vForward);
	const Vec3 vEye = pLocal->GetEyePosition();

	CTraceFilterWorldAndPropsOnly filter;
	CGameTrace trace;
	SDK::Trace(vEye, vEye + vForward * 6000.f, MASK_PLAYERSOLID, &filter, &trace);
	const Vec3 vHit = trace.endpos;

	if (bRising)
	{
		// First press: clear old results, anchor the start of the scan column. Ignore walls you
		// can't actually interact with (trigger brushes) or empty aims, same as Auto Align.
		m_vPixelSurfPoints.clear();
		m_bPixelFinderFirst = false;
		if (trace.fraction >= 1.f || (trace.surface.flags & SURF_TRIGGER))
			return;
		m_vPixelFinderStart = vHit;
		m_vPixelFinderEnd = vHit;
		m_vPixelFinderWallNormal = trace.plane.normal;
		m_bPixelFinderFirst = true;
		return;
	}

	if (bHeld)
	{
		// While held: stretch the scan column to the current aim height.
		if (m_bPixelFinderFirst)
			m_vPixelFinderEnd.z = vHit.z;
		return;
	}

	// Release: brute-force the wall column for pixel-surf catch positions.
	if (!m_bPixelFinderFirst)
		return;
	m_bPixelFinderFirst = false;

	Vec3 vWallNormal = m_vPixelFinderWallNormal;
	vWallNormal.z = 0.f;
	if (vWallNormal.Length() < 0.1f)
		return;
	vWallNormal.Normalize();

	// Tick-rate-correct pixel-surf velocity: while riding a surf the engine pins vertical velocity
	// to -halfgravity-per-tick.
	static auto sv_gravity = H::ConVars.FindVar("sv_gravity");
	const float flHalfGrav = (sv_gravity ? sv_gravity->GetFloat() : 800.f) * I::GlobalVars->interval_per_tick * 0.5f;
	constexpr float flSurfTol = 1.5f;
	// Lateral hull offset that makes the probe actually CONTACT the lip. the reference working CSGO finder

	const float flMargin = Vars::Misc::Movement::PixelFinderScanMargin.Value;
	const float flLow = std::min(m_vPixelFinderStart.z, m_vPixelFinderEnd.z) - flMargin;
	const float flHigh = std::max(m_vPixelFinderStart.z, m_vPixelFinderEnd.z) + flMargin;
	const int iSteps = std::clamp(int(flHigh - flLow), 0, 600);

	const int iBackupButtons = pCmd->buttons;
	const float flBackupForward = pCmd->forwardmove;
	const float flBackupSide = pCmd->sidemove;
	const Vec3 vMins = pLocal->m_vecMins();
	const Vec3 vMaxs = pLocal->m_vecMaxs();
	const std::string sMap = I::EngineClient->GetLevelName() ? I::EngineClient->GetLevelName() : "";

	// Wall tangent (horizontal, perpendicular to the scan normal) for the lateral band sweep.
	Vec3 vTangent = Vec3(vWallNormal.y, -vWallNormal.x, 0.f);
	if (vTangent.Length() > 0.1f)
		vTangent.Normalize();

	// A pixel-surf lip occupies a narrow XY, so probing only the single aimed column means you have
	// to luck your crosshair onto it - that is the "scan 10 times before it finds one" problem.
	constexpr float flBandHalf = 16.f; // +/- units swept along the wall tangent at each height
	constexpr float flBandStep = 4.f;  // lateral sample spacing within the band

	// One probe per height. the reference finder (the accurate one) does NOT drop a falling probe and hope
	auto TryPin = [&](const Vec3& vTestOrigin, const Vec3& vN, bool bDuck, float& flPinZ) -> bool
	{
		const Vec3 vInto = Vec3(-vN.x, -vN.y, 0.f);
		Vec3 vIntoAngle = Math::VectorAngles(vInto);
		Math::ClampAngles(vIntoAngle);
		const float flRot = DEG2RAD(vIntoAngle.y - pCmd->viewangles.y);
		const float flCos = cosf(flRot), flSin = sinf(flRot);

		auto Probe = [&](float flFeetZ, float flSeedVz, int iTicks) -> bool
		{
			F::Misc.RestoreEntityToPredicted();

			const Vec3 vAt = Vec3(vTestOrigin.x, vTestOrigin.y, flFeetZ);
			pLocal->SetAbsOrigin(vAt);
			pLocal->m_vecOrigin() = vAt;
			pLocal->m_fFlags() &= ~FL_ONGROUND; // airborne, on (or falling onto) the lip

			// Light into-wall press only - a big into-wall speed pins the hull flat against the
			// main wall and free-falls past the lip.
			pLocal->m_vecVelocity() = vInto * 10.f + Vec3(0.f, 0.f, flSeedVz);

			pCmd->buttons = iBackupButtons & ~IN_JUMP;
			if (bDuck)
				pCmd->buttons |= IN_DUCK;
			else
				pCmd->buttons &= ~IN_DUCK;
			pCmd->forwardmove = flCos * 10.f;
			pCmd->sidemove = -flSin * 10.f;

			for (int t = 0; t < iTicks; t++)
			{
				F::EnginePrediction.Simulate(pLocal, pCmd);
				pCmd->buttons &= ~IN_JUMP; // never let the probe launch upward

				if (fabsf(pLocal->m_vecVelocity().z + flHalfGrav) < flSurfTol)
				{
					flPinZ = pLocal->m_vecOrigin().z; // the engine's own settled catch height
					return true;
				}
			}
			return false;
		};

		// Exact placement at the reference catch offset (1 tick is decisive), then the fallback drop
		// sweeping the rest of this 1u step (-8 vz = 0.12u per tick x 10 ticks covers ~1.2u).
		const float flT = truncf(vTestOrigin.z);
		const float flExact = vTestOrigin.z >= 0.f ? flT + 0.0287018f : flT - 0.972092f;
		if (Probe(flExact, -flHalfGrav, 2))
			return true;
		return Probe(flExact + 0.9f, -8.f, 10);
	};

	for (int i = 0; i <= iSteps; i++)
	{
		const float flZ = flLow + float(i);
		bool bFoundThisHeight = false;

		// Sweep the lateral band at this height. First spot that pins wins - the run-dedup below
		// already collapses the column of adjacent catches into one marker, so one per height is plenty.
		for (float flLat = -flBandHalf; flLat <= flBandHalf + 0.01f && !bFoundThisHeight; flLat += flBandStep)
		{
			// Re-trace the wall at THIS height and lateral offset instead of reusing the first-aim XY
			// for the whole column. A pixel-surf lip protrudes at its own height only, so probing per
			const Vec3 vProbeMid = Vec3(m_vPixelFinderStart.x, m_vPixelFinderStart.y, flZ) + vTangent * flLat;
			CGameTrace wallTrace;
			SDK::Trace(vProbeMid + vWallNormal * 12.f, vProbeMid - vWallNormal * 12.f, MASK_PLAYERSOLID, &filter, &wallTrace);

			Vec3 vN, vWallXY;
			bool bDispWall = false;
			if (wallTrace.fraction < 1.f && fabsf(wallTrace.plane.normal.z) <= 0.7f
				&& Vec3(wallTrace.plane.normal.x, wallTrace.plane.normal.y, 0.f).Length2D() > 0.1f)
			{
				vN = Vec3(wallTrace.plane.normal.x, wallTrace.plane.normal.y, 0.f);
				vN.Normalize();
				vWallXY = wallTrace.endpos;
				bDispWall = wallTrace.IsDispSurface();
			}
			else
			{
				// No clean re-acquire at this height (floor clutter, props, displacement noise in
				// the ray's path). Don't throw the height away - fall back to the ANCHORED wall
				vN = vWallNormal;
				vWallXY = vProbeMid;
			}

			// Feet origin: hull pressed up against the surface we just found here, at the AABB
			// support distance for THIS column's normal (face contact on axis-aligned walls, corner
			const float flSupport = fabsf(vMaxs.x) * fabsf(vN.x) + fabsf(vMaxs.y) * fabsf(vN.y);
			const float flGap = flSupport + (bDispWall ? 0.001f : -0.022f);
			const Vec3 vTestOrigin = Vec3(vWallXY.x, vWallXY.y, flZ) + vN * flGap;

			// Reject offsets where the hull spawns embedded in geometry - a real surf catches a lip
			// with the hull in open air; an embedded hull only produces junk clip velocities.
			const Vec3 vDuckMaxs = Vec3(vMaxs.x, vMaxs.y, 62.f);
			CGameTrace solidTrace;
			SDK::TraceHull(vTestOrigin, vTestOrigin, vMins, vDuckMaxs, MASK_PLAYERSOLID, &filter, &solidTrace);
			if (solidTrace.startsolid || solidTrace.allsolid)
				continue;

			// Test standing, then ducked (most TF2 pixel surfs are ducked).
			for (int iDuck = 0; iDuck < 2 && !bFoundThisHeight; iDuck++)
			{
				float flPinZ = flZ;
				if (TryPin(vTestOrigin, vN, iDuck != 0, flPinZ))
				{
					// Store the spot ON the wall surface (wall XY, not the hull feet origin that
					// sits ~half a hull-width off the wall) at the height the hull actually pinned -
					const Vec3 vWallPoint = Vec3(vWallXY.x, vWallXY.y, flPinZ);
					m_vPixelSurfPoints.push_back({ vWallPoint, sMap });
					bFoundThisHeight = true;
				}
			}
		}
	}

	// Restore everything we touched during the scan.
	pCmd->buttons = iBackupButtons;
	pCmd->forwardmove = flBackupForward;
	pCmd->sidemove = flBackupSide;
	F::Misc.RestoreEntityToPredicted();

	// The scan walks the column 1u at a time, so a single surfable lip yields a run of adjacent
	if (m_vPixelSurfPoints.size() > 1)
	{
		std::vector<PixelSurfPoint_t> vDeduped;
		vDeduped.push_back(m_vPixelSurfPoints.front());
		float flPrevZ = m_vPixelSurfPoints.front().m_vPos.z;

		for (size_t i = 1; i < m_vPixelSurfPoints.size(); i++)
		{
			const Vec3& vPos = m_vPixelSurfPoints[i].m_vPos;
			if (fabsf(vPos.z - flPrevZ) > 1.5f)
				vDeduped.push_back(m_vPixelSurfPoints[i]);
			flPrevZ = vPos.z;
		}
		m_vPixelSurfPoints = std::move(vDeduped);
	}
}

// ----------------------------- Pixel surf line -----------------------------
// "Show pixel surf line": trace your ACTUAL surf path. Every tick you're riding a pixel surf we append
void CMisc::PixelSurfLine(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	// Recording this tick? Feature on, alive, and actually riding a surf (PixelSurf set this earlier
	// in RunPost; it's the same signal the "ps" indicator uses, so it works with the auto-catcher off).
	const bool bSurfing = Vars::Misc::Movement::PixelSurfLine.Value
		&& pLocal && pLocal->IsAlive() && m_bPixelSurfingNow;

	if (bSurfing)
	{
		if (!m_bWasInSurfLine)
			m_vSurfLineCurrent.clear(); // new ride - start a fresh path
		m_bWasInSurfLine = true;
		m_iSurfLineGrace = 8; // tolerate a few non-surf ticks (detection blip) before closing the path

		// On-wall position: project the player origin onto the nearest near-vertical wall so the line
		// sits on the surface (a quick radial scan, no prediction); fall back to the origin if no wall
		const Vec3 vOrigin = pLocal->m_vecOrigin();
		Vec3 vPoint = vOrigin;
		{
			CTraceFilterWorldAndPropsOnly filter;
			constexpr int   iDirs = 16;
			constexpr float flProbe = 64.f;
			constexpr float flTwoPi = 6.28318530717958647692f;
			float flBest = 1.f;
			for (int i = 0; i < iDirs; i++)
			{
				const float flAng = float(i) / float(iDirs) * flTwoPi;
				const Vec3 vDir = Vec3(cosf(flAng), sinf(flAng), 0.f);
				CGameTrace t;
				SDK::Trace(vOrigin, vOrigin + vDir * flProbe, MASK_PLAYERSOLID, &filter, &t);
				if (t.fraction >= 1.f || t.fraction >= flBest)
					continue;
				if (fabsf(t.plane.normal.z) > 0.35f) // floor/ceiling, not a wall
					continue;
				flBest = t.fraction;
				vPoint = t.endpos;
			}
		}

		// Append, spaced out so a near-stationary pin doesn't pile up thousands of points.
		if (m_vSurfLineCurrent.empty() || m_vSurfLineCurrent.back().DistTo(vPoint) >= 4.f)
			m_vSurfLineCurrent.push_back(vPoint);
		return;
	}

	// Not surfing this tick. A brief blip in detection shouldn't end the path - ride it out for a few
	// ticks; if the surf resumes, the same path continues. Only when the grace runs out is the path
	if (m_bWasInSurfLine && m_iSurfLineGrace > 0)
	{
		m_iSurfLineGrace--;
		return;
	}

	// Grace expired: if we had a path open, commit the line we traced.
	if (m_bWasInSurfLine)
	{
		m_bWasInSurfLine = false;
		if (m_vSurfLineCurrent.size() >= 2)
		{
			const std::string sMap = I::EngineClient->GetLevelName() ? I::EngineClient->GetLevelName() : "";

			// Skip if this path basically retraces one already saved here (re-surfing the same line).
			// Sample every point of the new path against each saved segment; if most of the new points
			bool bDup = false;
			constexpr float flNearDist = 32.f;   // a new point this close to a saved point = overlapping
			constexpr float flDupRatio = 0.6f;    // this fraction of the path overlapping = duplicate
			for (const auto& seg : m_vSurfLineSegments)
			{
				if (seg.m_sMap != sMap)
					continue;
				int iOverlap = 0;
				for (const auto& np : m_vSurfLineCurrent)
				{
					for (const auto& p : seg.m_vPoints)
						if (p.DistTo(np) < flNearDist) { iOverlap++; break; }
				}
				if (iOverlap >= int(m_vSurfLineCurrent.size() * flDupRatio))
				{
					bDup = true;
					break;
				}
			}

			if (!bDup)
			{
				SurfLineSegment_t seg;
				seg.m_sMap = sMap;
				seg.m_vPoints = m_vSurfLineCurrent;
				m_vSurfLineSegments.push_back(std::move(seg));
				constexpr size_t iMaxSegments = 64; // bound memory: drop the oldest strip past the cap
				if (m_vSurfLineSegments.size() > iMaxSegments)
					m_vSurfLineSegments.erase(m_vSurfLineSegments.begin());
			}
		}
		m_vSurfLineCurrent.clear();
	}
}

// ----------------------------- Pixel surf assist -----------------------------
// Save finder-found surf points; while the assist key is held, jump onto the nearest saved point within

// SimJumpReach: sim one jump type's arc and return its feet-apex gain (or the height still left at a given
// flat distance when flAtFlatDist > 0), calibrated to the real landing. Side-effect free.
float CMisc::SimJumpReach(CTFPlayer* pLocal, CUserCmd* pCmd, int iJumpType, float flAtFlatDist)
{
	EnginePredictionBatch_t tPredictionBatch;

	const bool bMini = iJumpType == JT_MINI || iJumpType == JT_MINIDUCK;
	const bool bLong = iJumpType == JT_LONG || iJumpType == JT_LONGDUCK;
	const bool bHold = iJumpType == JT_CROUCH || iJumpType == JT_MINIDUCK || iJumpType == JT_LONGDUCK;

	// flAtFlatDist <= 0: apex feet-Z gain. > 0: the feet-Z still left once the arc has travelled that far
	// out (you arrive over a far target already descending, so the apex over-states reach).
	const bool bAtDist = flAtFlatDist > 0.f;

	const int iBtnBackup = pCmd->buttons;
	const float flFwdBackup = pCmd->forwardmove;
	const float flSideBackup = pCmd->sidemove;

	// Pin the base frame before Simulate: each Simulate advances m_nCommandsPredicted, so a bare restore
	// drifts and back-to-back candidate sims corrupt each other. Pinning keeps every sim independent.
	const int nBasePredicted = I::Prediction->m_nCommandsPredicted;
	I::Prediction->RestoreEntityToPredictedFrame(nBasePredicted - 1);
	const Vec3 vStart = pLocal->m_vecOrigin();
	const float flStartZ = vStart.z;
	float flPeakZ = flStartZ;

	// At-distance tracking: Z sampled at exactly flAtFlatDist of horizontal displacement, found by
	// interpolating across the tick where displacement first crosses that distance.
	float flAtDistZ = -FLT_MAX;
	float flPrevDisp = 0.f, flPrevZ = flStartZ;

	for (int t = 0; t < 50; t++)
	{
		pCmd->buttons = iBtnBackup & ~(IN_JUMP | IN_DUCK);
		if (t == 0)
			pCmd->buttons |= IN_JUMP; // launch on the first tick, same as the commit path

		// Real TF2 timings (user-measured apexes told the story: mini 28 / mini+duck 48 / crouch
		// 72 - the held-duck variants differ, so the mini's duck must ride ON the launch cmd).
		bool bDuck = false;
		if ((bMini || bLong) && t == 0)
			bDuck = true;             // mini/long: duck pressed WITH the jump
		if (bLong && t >= 1 && t <= 2)
			bDuck = true;             // long: keep it two more cmds
		if (bHold && t >= 1)
			bDuck = true;             // crouch/+duck: hold from the first airborne tick to landing
		if (bDuck)
			pCmd->buttons |= IN_DUCK;

		F::EnginePrediction.Simulate(pLocal, pCmd); // user's own forward/side carry the arc

		const Vec3 vNow = pLocal->m_vecOrigin();
		const float flZ = vNow.z;
		if (flZ > flPeakZ)
			flPeakZ = flZ;

		// Sample the arc's Z at the target's horizontal distance the first time we reach it.
		if (bAtDist && flAtDistZ == -FLT_MAX)
		{
			const float flDisp = Vec3(vNow.x - vStart.x, vNow.y - vStart.y, 0.f).Length2D();
			if (flDisp >= flAtFlatDist)
			{
				const float flSpan = flDisp - flPrevDisp;
				const float flFrac = flSpan > 0.01f ? (flAtFlatDist - flPrevDisp) / flSpan : 1.f;
				flAtDistZ = flPrevZ + (flZ - flPrevZ) * flFrac;
			}
			flPrevDisp = flDisp;
			flPrevZ = flZ;
		}

		if (t > 2 && pLocal->m_vecVelocity().z < 0.f && flZ <= flStartZ)
			break; // back down to launch height - arc finished
	}

	pCmd->buttons = iBtnBackup;
	pCmd->forwardmove = flFwdBackup;
	pCmd->sidemove = flSideBackup;
	// Restore back to the same pinned frame, not m_nCommandsPredicted - 1 (which has
	// drifted forward by however many Simulate calls the loop took).
	I::Prediction->RestoreEntityToPredictedFrame(nBasePredicted - 1);

	// Calibrated against real measured landings (chudhook\Debug\jumpstats.csv vs jumpsim.csv):
	// the raw per-tick simulated apex reads ~4u higher than the height the player actually gains
	constexpr float flSimOvershoot = 4.f;

	float flGain;
	if (bAtDist)
		// Height available where the target actually is. If the arc landed before ever travelling
		// that far horizontally, the target is out of this jump's range -> 0 (won't be picked).
		flGain = (flAtDistZ == -FLT_MAX) ? 0.f : (flAtDistZ - flStartZ);
	else
		flGain = flPeakZ - flStartZ; // apex

	const float flReach = flGain - flSimOvershoot;
	return flReach > 0.f ? flReach : 0.f;
}

// Simulates one jump type launched THIS tick (with the cmd's own steering held) and reports
static bool GridCheck(float a, float b)
{
	const int a1 = int(a), b1 = int(b);
	if (a1 != b1)
		return false;
	const int a2 = int((a - a1) * 100.f), b2 = int((b - b1) * 100.f);
	if (b < 0.f)
		return b2 == a2 || a2 + 1 == b2 || a2 + 2 == b2;
	return b2 == a2 || a2 == b2 - 1 || a2 == b2 - 2;
}

CMisc::JumpCatchResult_t CMisc::SimJumpCatch(CTFPlayer* pLocal, CUserCmd* pCmd, int iJumpType, const Vec3& vTarget, float flAtFlatDist)
{
	JumpCatchResult_t tResult;

	const bool bMini = iJumpType == JT_MINI || iJumpType == JT_MINIDUCK;
	const bool bLong = iJumpType == JT_LONG || iJumpType == JT_LONGDUCK;
	// mini+duck does NOT hold its duck here: live, the +20u at the catch comes from the descent
	// duck that the auto PixelSurf owns (you only minijump, and don't land crouched on a miss).
	const bool bHold = iJumpType == JT_CROUCH || iJumpType == JT_LONGDUCK;

	static auto sv_gravity = H::ConVars.FindVar("sv_gravity");
	const float flHalfGrav = (sv_gravity ? sv_gravity->GetFloat() : 800.f) * I::GlobalVars->interval_per_tick * 0.5f;
	constexpr float flSurfTol = 1.5f; // same pin tolerance PixelSurf uses
	constexpr float flZNear = 1.f;    // pin must happen AT our point - tight, so a neighbouring
	                                  // pixel on the same wall can't pass as a hit

	const int iBtnBackup = pCmd->buttons;
	const float flFwdBackup = pCmd->forwardmove;
	const float flSideBackup = pCmd->sidemove;

	// Pin the base prediction frame (see SimJumpReach for why a bare RestoreEntityToPredicted
	// drifts when several sims run back-to-back).
	const int nBasePredicted = I::Prediction->m_nCommandsPredicted;
	I::Prediction->RestoreEntityToPredictedFrame(nBasePredicted - 1);

	// Approach phase: when the sim starts airborne - descending from
	// the previous hop, running off a bump, mid bhop chain - it does NOT pretend a jump can fire
	constexpr int iMaxApproach = 40;
	int iLaunch = -1;          // sim tick the jump was pressed on
	Vec3 vLaunch = pLocal->m_vecOrigin();
	float flPeakZ = -FLT_MAX;  // tracked post-launch only

	// At-distance tracking, same interpolation as SimJumpReach: the height the arc still has
	float flAtDistZ = -FLT_MAX;
	float flPrevDisp = 0.f, flPrevZ = 0.f;

	// Analytic grid catch. Once the rising arc is past
	const float flGravPerTick2 = (sv_gravity ? sv_gravity->GetFloat() : 800.f)
		* I::GlobalVars->interval_per_tick * I::GlobalVars->interval_per_tick;
	// mini+duck rides the mini's grid: its +20 at the catch is PixelSurf's in-air descent duck,
	// an exact constant feet shift (standing hull 82 - duck hull 62), so its grid simply checks
	const float flGridDuckShift = iJumpType == JT_MINIDUCK ? 20.f : 0.f;
	// LONG releases its launch duck on t3 - a measurement before the resulting origin shift has
	// settled would bake +-20 into the delta and poison the whole extrapolation.
	const int iMinStable = bLong ? 5 : 3;
	int iRiseStable = 0;       // consecutive clean rising ticks
	bool bGridDone = false;
	float flGridPrevZ = 0.f;   // feet Z after the previous simulated tick

	bool bPrevPin = false;
	int iGroundTicks = 0; // consecutive grounded sim ticks (single bump contacts don't end the arc)
	for (int t = 0; t < iMaxApproach + 60; t++)
	{
		pCmd->buttons = iBtnBackup & ~(IN_JUMP | IN_DUCK);

		if (iLaunch < 0)
		{
			const bool bGroundedNow = pLocal->m_fFlags() & FL_ONGROUND;
			if (iJumpType == JT_AIRDASH || bGroundedNow)
			{
				iLaunch = t;
				vLaunch = pLocal->m_vecOrigin();
				flPeakZ = vLaunch.z;
				flPrevZ = vLaunch.z;
				pCmd->buttons |= IN_JUMP; // launch this cmd, same as the commit path
			}
			else if (t >= iMaxApproach)
				break; // never found ground to launch from inside the window
		}

		// Live choreography, real TF2 timings (see SimJumpReach), indexed from the LAUNCH tick:
		// mini/long press their duck WITH the jump (that's what cuts the apex), long keeps it 2
		const int iSince = iLaunch >= 0 ? t - iLaunch : -1;
		bool bDuck = false;
		if ((bMini || bLong) && iSince == 0)
			bDuck = true;
		if (bLong && iSince >= 1 && iSince <= 2)
			bDuck = true;
		if (bHold && iSince >= 1)
			bDuck = true;
		if (iSince >= 1 && pLocal->m_vecVelocity().z < 0.f)
			bDuck = true; // descending - PixelSurf will be holding the catch duck by now
		if (bDuck)
			pCmd->buttons |= IN_DUCK;

		const float flPreVz = pLocal->m_vecVelocity().z;
		const bool bPreDucked = pLocal->m_fFlags() & FL_DUCKING;

		F::EnginePrediction.Simulate(pLocal, pCmd); // user's own forward/side carry the arc into the wall

		if (iLaunch < 0)
			continue; // still approaching - the hop doesn't exist yet, nothing below applies

		const Vec3 vNow = pLocal->m_vecOrigin();
		const float flZ = vNow.z;
		const float flVz = pLocal->m_vecVelocity().z;
		const bool bAir = !(pLocal->m_fFlags() & FL_ONGROUND);
		if (flZ > flPeakZ)
			flPeakZ = flZ;

		// Sample the arc's Z at the target's horizontal distance the first time we reach it
		// (distance measured from the launch point, where the committed hop actually starts).
		if (flAtFlatDist > 0.f && flAtDistZ == -FLT_MAX)
		{
			const float flDisp = Vec3(vNow.x - vLaunch.x, vNow.y - vLaunch.y, 0.f).Length2D();
			if (flDisp >= flAtFlatDist)
			{
				const float flSpan = flDisp - flPrevDisp;
				const float flFrac = flSpan > 0.01f ? (flAtFlatDist - flPrevDisp) / flSpan : 1.f;
				flAtDistZ = flPrevZ + (flZ - flPrevZ) * flFrac;
			}
			flPrevDisp = flDisp;
			flPrevZ = flZ;
		}

		// Grid measurement: the first tick that is (a) past the launch choreography's duck
		if (!bGridDone && iSince >= 1)
		{
			const bool bPostDucked = pLocal->m_fFlags() & FL_DUCKING;
			const float flD = flGridPrevZ - flZ; // negative while rising (references' iCalcilate)
			const bool bClean = iSince >= iMinStable && flPreVz > 0.f && flVz > 0.f
				&& bPreDucked == bPostDucked && flD < 0.f && flD > -4.5f;
			if (!bClean)
				iRiseStable = 0;
			else if (++iRiseStable > 2)
			{
				bGridDone = true;
				// Anchor at the PRE-tick Z and replay the measured step as k=0 (exactly the
				// references): each later step then grows by gravity*dt^2 and every extrapolated
				float flGridZ = flGridPrevZ;
				for (int k = 0; k < 512; k++)
				{
					const float flStep = flD + flGravPerTick2 * float(k);
					if (flStep > 0.f) // apex passed - these are the landable descending ticks
					{
						if (GridCheck(flGridZ + flGridDuckShift, vTarget.z)
							|| (iJumpType == JT_AIRDASH && GridCheck(flGridZ + 20.f, vTarget.z)))
						{
							tResult.m_bGridCatch = true;
							break;
						}
						if (flGridZ < vTarget.z - 150.f)
							break; // fell far past the point - no tick aligned
					}
					flGridZ -= flStep;
				}
			}
		}
		flGridPrevZ = flZ;

		// Track how close the ducked descent got to the point's height, plus how fast it was
		// moving and how far from the point it still was (picker's slow-pass fallback + readout).
		if (bAir && flVz < 0.f)
		{
			const float flGap = fabsf(flZ - vTarget.z);
			if (flGap < tResult.m_flGap)
			{
				tResult.m_flGap = flGap;
				tResult.m_flGapVz = flVz;
				tResult.m_flGapFlat = Vec3(vTarget.x - vNow.x, vTarget.y - vNow.y, 0.f).Length2D();
			}
		}

		// The pin: free fall changes vz by a full gravity tick (~12), so a single tick can read
		const bool bPin = bAir && fabsf(flVz + flHalfGrav) < flSurfTol;
		if (bPin && fabsf(flZ - vTarget.z) < flZNear)
		{
			tResult.m_bGraze = true;
			if (bPrevPin)
			{
				tResult.m_bCatch = true;
				break;
			}
		}
		bPrevPin = bPin;

		// Landing check with displacement tolerance: launching from (or flying over) bumpy dirt,
		// the arc can clip a bump for a single grounded tick while still genuinely airborne -
		iGroundTicks = bAir ? 0 : iGroundTicks + 1;
		if (iSince > 2 && iGroundTicks >= 2)
			break; // landed without catching
		if (flVz < 0.f && flZ < vTarget.z - 30.f)
			break; // fell well past the point - no tick caught it
	}

	pCmd->buttons = iBtnBackup;
	pCmd->forwardmove = flFwdBackup;
	pCmd->sidemove = flSideBackup;
	I::Prediction->RestoreEntityToPredictedFrame(nBasePredicted - 1);

	tResult.m_iLaunchDelay = iLaunch;
	tResult.m_flLaunchZ = vLaunch.z;
	tResult.m_flApexGain = iLaunch >= 0 ? flPeakZ - vLaunch.z : 0.f;
	// If the arc never travelled that far horizontally, the point is out of this jump's range.
	tResult.m_flAtDistGain = (flAtDistZ == -FLT_MAX) ? 0.f : (flAtDistZ - vLaunch.z);
	return tResult;
}

void CMisc::PixelSurfAssist(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	m_bPixelSurfAssistActive = false;

	// Finished-hop bookkeeping must run UNCONDITIONALLY, before any early return. It used to sit
	// behind the key/radius gates below, so firing one assist jump and releasing the key mid-air
	if (m_iAssistJumpType != JT_NONE)
	{
		const int iNow = I::GlobalVars->tickcount;
		if (iNow < m_iAssistJumpStartTick
			|| ((m_iPrePredictionFlags & FL_ONGROUND) && iNow > m_iAssistJumpStartTick)
			// Catching the surf finishes the hop too. A pinned player is never FL_ONGROUND, so
			// without this the steer lock ran forever once you landed ON the point - still
			|| m_bPixelSurfingNow)
			m_iAssistJumpType = JT_NONE;
	}
	// Pending-launch sanity, same unconditional spot (the iteration-5 lesson: every tickcount
	if (m_iAssistPendingType != JT_NONE)
	{
		const int iNow = I::GlobalVars->tickcount;
		if (iNow > m_iAssistPendingUntil || m_iAssistPendingUntil > iNow + 120
			|| m_bPixelSurfingNow || !Vars::Misc::Movement::PixelSurfAssist.Value)
			m_iAssistPendingType = JT_NONE;
	}

	const std::string sMap = I::EngineClient->GetLevelName() ? I::EngineClient->GetLevelName() : "";
	const float flSnap = Vars::Misc::Movement::PixelSurfAssistSnapDist.Value;

	// Pull this map's saved targets from disk on the first run / whenever the map changes.
	if (sMap != m_sAssistPointsMap)
		LoadAssistPoints(sMap);

	// Where the crosshair hits the world (shared by set-point / delete-point).
	auto AimHit = [&](Vec3& vOut) -> bool
	{
		Vec3 vForward; Math::AngleVectors(pCmd->viewangles, &vForward);
		const Vec3 vEye = pLocal->GetEyePosition();
		CTraceFilterWorldAndPropsOnly filter;
		CGameTrace trace;
		SDK::Trace(vEye, vEye + vForward * 4000.f, MASK_PLAYERSOLID, &filter, &trace);
		vOut = trace.endpos;
		return trace.fraction < 1.f;
	};

	// Set-point: a target can only be placed on a pixel-surf spot the finder found - snap to the
	// nearest finder point on this map within the snap distance. Works any time.
	const bool bSetHeld = Vars::Misc::Movement::PixelSurfAssistSetPoint.Value;
	if (bSetHeld && !m_bAssistSetHeld)
	{
		Vec3 vHit;
		if (AimHit(vHit))
		{
			const PixelSurfPoint_t* pSnap = nullptr;
			float flBestSnap = flSnap;
			for (const auto& p : m_vPixelSurfPoints)
			{
				if (p.m_sMap != sMap)
					continue;
				const float flDist = vHit.DistTo(p.m_vPos);
				if (flDist < flBestSnap)
				{
					flBestSnap = flDist;
					pSnap = &p;
				}
			}
			if (pSnap)
			{
				m_vAssistPoints.push_back({ pSnap->m_vPos, sMap });
				WriteAssistPoints(sMap);
			}
		}
	}
	m_bAssistSetHeld = bSetHeld;

	// Delete-point: remove the saved target on this map nearest to where you're aiming.
	const bool bDelHeld = Vars::Misc::Movement::PixelSurfAssistDeletePoint.Value;
	if (bDelHeld && !m_bAssistDeleteHeld)
	{
		Vec3 vHit;
		if (AimHit(vHit))
		{
			int iErase = -1;
			float flBestDel = flSnap;
			for (int i = 0; i < int(m_vAssistPoints.size()); i++)
			{
				if (m_vAssistPoints[i].m_sMap != sMap)
					continue;
				const float flDist = vHit.DistTo(m_vAssistPoints[i].m_vPos);
				if (flDist < flBestDel)
				{
					flBestDel = flDist;
					iErase = i;
				}
			}
			if (iErase >= 0)
			{
				m_vAssistPoints.erase(m_vAssistPoints.begin() + iErase);
				WriteAssistPoints(sMap);
			}
		}
	}
	m_bAssistDeleteHeld = bDelHeld;

	if (!Vars::Misc::Movement::PixelSurfAssist.Value || m_vAssistPoints.empty())
		return;

	// Nearest ACTIVE saved point on this map inside its engage radius (per-point radius from the
	// ENTER menu wins over the global slider when set).
	const Vec3 vOrigin = pLocal->m_vecOrigin();
	const PixelSurfPoint_t* pPoint = nullptr;
	float flBestDist = 1e9f;
	for (const auto& p : m_vAssistPoints)
	{
		if (p.m_sMap != sMap || !p.m_bActive)
			continue;
		const float flRadius = p.m_flRadius > 0.f ? p.m_flRadius : Vars::Misc::Movement::PixelSurfAssistRadius.Value;
		const float flDist = vOrigin.DistTo(p.m_vPos);
		if (flDist > flRadius || flDist >= flBestDist)
			continue;
		flBestDist = flDist;
		pPoint = &p;
	}
	if (!pPoint)
		return;
	const Vec3* pTarget = &pPoint->m_vPos;

	m_bPixelSurfAssistActive = true;

	// Geometry to the target. Per request the assist does NOT steer your movement - it only
	// jumps; horizontal aiming/strafing toward the point is left entirely to you.
	const Vec3 vDelta = *pTarget - vOrigin;
	const float flFlat = Vec3(vDelta.x, vDelta.y, 0.f).Length2D();

	// --- Jump onto it: pick the cheapest jump that reaches the target, then drive its duck. ---
	// (Jump-type ids JT_* are the shared CMisc::EJumpType enum, declared in Misc.h.)

	// Real current ground contact is the pre-prediction flag (post-prediction already reflects
	// the cmd we're building); this matches how MiniJump/LongJump detect leaving the ground.
	const bool bOnGround = m_iPrePredictionFlags & FL_ONGROUND;

	// ---- Pending-launch service. ----
	// A pick armed mid-air fires on the REAL ground contact: the sim already validated the whole
	if (m_iAssistPendingType != JT_NONE && m_iAssistJumpType == JT_NONE && bOnGround)
	{
		const int iPick = m_iAssistPendingType;
		m_iAssistPendingType = JT_NONE;

		m_iAssistJumpType = iPick;
		m_iAssistJumpStartTick = I::GlobalVars->tickcount;
		pCmd->buttons |= IN_JUMP;
		const bool bLaunchDuck = iPick == JT_MINI || iPick == JT_MINIDUCK || iPick == JT_LONG || iPick == JT_LONGDUCK;
		if (bLaunchDuck)
			pCmd->buttons |= IN_DUCK;
		else
			pCmd->buttons &= ~IN_DUCK;
		// Steer lock stays as captured at ARM time - the sim flew that wish through the whole
		// approach, so recapturing here (possibly mid keyboard adjustment) would break the match.

		m_tAssistJumpBox.m_iJumpType = iPick;
		m_tAssistJumpBox.m_flTargetZ = pTarget->z;
		m_tAssistJumpBox.m_flCurTime = I::GlobalVars->curtime;
	}

	// The picker runs on the ground (launch now) AND while descending mid-air (plan the hop off
	// the predicted landing - this is where the previous jump's carried speed/duck state shapes
	const bool bDescending = !bOnGround && m_vPrePredictionVelocity.z < 0.f;

	if ((bOnGround || bDescending) && m_iAssistJumpType == JT_NONE && m_iAssistPendingType == JT_NONE)
	{
		// The jump box is NOT cleared here any more: hard-clearing on every picker re-entry meant

		// The pick is no longer "cheapest jump whose reach clears the height" - clearing is not
		// catching. Each type's whole arc is run through the real movement code from this exact
		struct Cand_t { int m_iType; JumpCatchResult_t m_tRes; };
		Cand_t aCands[] = // readout order: mini, regular, mini+duck, crouch, long, long+duck
		{
			{ JT_MINI,     {} },
			{ JT_REGULAR,  {} },
			{ JT_MINIDUCK, {} },
			{ JT_CROUCH,   {} },
			{ JT_LONG,     {} },
			{ JT_LONGDUCK, {} },
		};

		// Per-point jump-type allowance from the ENTER menu (bit k = aCands[k] allowed).
		auto Allowed = [&](int k) -> bool { return pPoint->m_iTypeMask & (1 << k); };

		// No jump gains ~90u even held-duck; skip the six full-arc sims when the point is
		// hopelessly above us (or we're outside the sane flat range the gate below enforces).
		const bool bWorthSimming = vDelta.z < 90.f && flFlat < 450.f;
		if (bWorthSimming)
			for (int k = 0; k < 6; k++)
				if (Allowed(k))
				{
					aCands[k].m_tRes = SimJumpCatch(pLocal, pCmd, aCands[k].m_iType, *pTarget, flFlat);
					// All six share the same approach (same held wish, no jump until contact) -
					// if this one never found ground inside the window, none of them will.
					if (aCands[k].m_tRes.m_iLaunchDelay < 0)
						break;
				}

		// Three tiers, best first. Pin-only firing turned out too strict in-game (the canned sim
		static constexpr int aPriority[] = { JT_REGULAR, JT_MINI, JT_LONG, JT_MINIDUCK, JT_CROUCH, JT_LONGDUCK };
		int iPick = JT_NONE;

		// Effective max height this jump can actually deliver, hoisted here so the grid-catch floor
		// below and the clears tier further down share one definition. flApexGain is the REAL
		constexpr float flSimHighBias = 4.f;  // sim reads ~4u high of real landings (calibrated)
		constexpr float flDuckShift = 20.f;   // VEC_HULL_MAX.z 82 - VEC_DUCK_HULL_MAX.z 62
		auto EffApex = [&](const Cand_t& c) -> float
		{
			float fl = c.m_tRes.m_flApexGain - flSimHighBias;
			// mini+duck's catch height comes from PixelSurf's descent duck, which its
			// (deliberately duck-free) sim apex doesn't carry - the descent samples do.
			if (c.m_iType == JT_MINIDUCK)
				fl += flDuckShift;
			return fl;
		};

		// Grid catch is only honest when the REAL arc actually reaches the point. The grid is pure
		auto GridCatchOk = [&](const Cand_t& c) -> bool
		{
			if (!c.m_tRes.m_bGridCatch)
				return false;
			const float flNeedK = pTarget->z - c.m_tRes.m_flLaunchZ;
			return EffApex(c) >= flNeedK - flSimHighBias;
		};

		// Tier 0 - the criterion, exact and strafe-proof: some future descending tick
		// of this launch lands in the point's sub-unit catch window (analytic grid). From a given
		for (int iWant : aPriority)
		{
			for (const auto& c : aCands)
			{
				if (c.m_iType == iWant && GridCatchOk(c))
				{
					iPick = iWant;
					break;
				}
			}
			if (iPick != JT_NONE)
				break;
		}

		for (int iPass = 0; iPass < 2 && iPick == JT_NONE; iPass++) // pass 0 = catch, pass 1 = graze
		{
			for (int iWant : aPriority)
			{
				for (const auto& c : aCands)
				{
					if (c.m_iType == iWant && (iPass == 0 ? c.m_tRes.m_bCatch : c.m_tRes.m_bGraze))
					{
						iPick = iWant;
						break;
					}
				}
				if (iPick != JT_NONE)
					break;
			}
		}

		// Displacement ground gets extra slack everywhere below: the launch height fraction
		// shifts with every step on dirt, so strict windows rarely line up there. (Trace reaches
		CGameTrace groundTrace;
		CTraceFilterWorldAndPropsOnly groundFilter;
		SDK::Trace(vOrigin, vOrigin - Vec3(0.f, 0.f, 80.f), MASK_PLAYERSOLID, &groundFilter, &groundTrace);
		const bool bDispGround = groundTrace.fraction < 1.f && groundTrace.IsDispSurface();

		// Tier-3 eligibility, shared with the readout's state column.
		const float flMaxOver = bDispGround ? 18.f : 10.f; // flSimHighBias/flDuckShift/EffApex hoisted above tier 0
		auto ClearsOk = [&](const Cand_t& c) -> bool
		{
			if (c.m_tRes.m_iLaunchDelay < 0)
				return false;
			const float flNeedK = pTarget->z - c.m_tRes.m_flLaunchZ; // need from the LAUNCH, not from here
			if (c.m_tRes.m_flAtDistGain - flSimHighBias < flNeedK + 1.f)
				return false; // doesn't clear the height where the point actually is
			const float flOver = EffApex(c) - flNeedK;
			return flOver >= 0.f && flOver <= flMaxOver;
		};

		if (iPick == JT_NONE)
		{
			float flBestOver = 1e9f;
			for (int k = 0; k < 6; k++)
			{
				const auto& c = aCands[k];
				if (!Allowed(k) || !ClearsOk(c))
					continue;
				const float flOver = EffApex(c) - (pTarget->z - c.m_tRes.m_flLaunchZ);
				if (flOver < flBestOver)
				{
					flBestOver = flOver;
					iPick = c.m_iType;
				}
			}
		}

		if (iPick != JT_NONE)
		{
			// Capture the commit-tick wish in WORLD space for the steer lock. SimJumpCatch held
			// this exact movement for the whole simulated approach + arc - if the player's keys
			Vec3 vFwd, vRight;
			Math::AngleVectors(Vec3(0.f, pCmd->viewangles.y, 0.f), &vFwd, &vRight, nullptr);
			Vec3 vWish = vFwd * pCmd->forwardmove + vRight * pCmd->sidemove;
			vWish.z = 0.f;
			m_flAssistWishMag = vWish.Length2D();
			if (m_flAssistWishMag > 1.f)
				m_vAssistWishDir = vWish * (1.f / m_flAssistWishMag);
			else
				m_flAssistWishMag = 0.f;

			const int iDelay = [&]{ for (const auto& c : aCands) if (c.m_iType == iPick) return c.m_tRes.m_iLaunchDelay; return 0; }();
			if (bOnGround && iDelay <= 0)
			{
				// Grounded with a zero-delay plan: fire now (the classic path).
				m_iAssistJumpType = iPick;
				m_iAssistJumpStartTick = I::GlobalVars->tickcount;
				pCmd->buttons |= IN_JUMP;
				// Mini/long variants press their duck WITH the jump - a fresh duck on the launch
				// cmd is taken (only an ALREADY-held duck, FL_DUCKING, eats the jump) and is what
				const bool bLaunchDuck = iPick == JT_MINI || iPick == JT_MINIDUCK || iPick == JT_LONG || iPick == JT_LONGDUCK;
				if (bLaunchDuck)
					pCmd->buttons |= IN_DUCK;
				else
					pCmd->buttons &= ~IN_DUCK;

				// Stamp the jump-box pop-up: which jump we just fired + the target's Z height.
				m_tAssistJumpBox.m_iJumpType = iPick;
				m_tAssistJumpBox.m_flTargetZ = pTarget->z;
				m_tAssistJumpBox.m_flCurTime = I::GlobalVars->curtime;
			}
			else
			{
				// Mid-air (or the launch sits a few ticks out): arm the pending launch. The hop
				m_iAssistPendingType = iPick;
				m_iAssistPendingUntil = I::GlobalVars->tickcount + iDelay + 8;
			}
		}
	}

	// Scout double jump: while airborne with a dash still in pocket, it can be fired at exactly
	// the height whose arc lands on the point - "jump from the perfect height". Re-evaluated

	// Dash budget, engine-faithful (CTFPlayer::CanAirDash, tf_player_shared.cpp): 1 by default,
	// 2 with the Atomizer's air_dash_count attribute ON THE ACTIVE WEAPON (the triple jump), 5
	auto CanAirDashNow = [&]() -> bool
	{
		if (pLocal->m_iClass() != TF_CLASS_SCOUT)
			return false;
		if (pLocal->InCond(TF_COND_HALLOWEEN_SPEED_BOOST))
			return true;
		if (pLocal->InCond(TF_COND_SODAPOPPER_HYPE))
			return pLocal->m_iAirDash() < 5;
		int iDashCount = 1;
		if (auto pWeapon = pLocal->m_hActiveWeapon()->As<CTFWeaponBase>())
			iDashCount = int(SDK::AttribHookValue(float(iDashCount), "air_dash_count", pWeapon));
		return pLocal->m_iAirDash() < iDashCount;
	};
	// An assist-fired dash that has provably missed - descending again with the feet already
	const bool bDashRetry = m_iAssistJumpType == JT_AIRDASH
		&& m_vPrePredictionVelocity.z < 0.f && vOrigin.z < pTarget->z;

	if (!bOnGround && (m_iAssistJumpType == JT_NONE || bDashRetry) && m_iAssistPendingType == JT_NONE
		&& CanAirDashNow()
		&& (pPoint->m_iTypeMask & (1 << 6))
		&& vDelta.z < 90.f && flFlat < 450.f)
	{
		const JumpCatchResult_t tDash = SimJumpCatch(pLocal, pCmd, JT_AIRDASH, *pTarget, flFlat);
		const float flDashOver = (tDash.m_flApexGain - 4.f) - vDelta.z; // same sim-high bias as grounded

		// Fast falls drop more than 8u per tick, so a fixed window gets skipped clean between
		// per-tick evaluations - widen it by the current per-tick fall so the perfect-height band
		const float flFallPerTick = std::max(0.f, -pLocal->m_vecVelocity().z) * I::GlobalVars->interval_per_tick;
		// Rising and still under the point: this is the moment to dash. Below the surf the apex
		const bool bRisingBelow = pLocal->m_vecVelocity().z > 0.f && vOrigin.z < pTarget->z;
		const bool bHeightWindow = flDashOver >= 0.5f && flDashOver <= std::max(8.f, flFallPerTick + 2.f)
			// Column clearance is only a VETO when the sim actually measured it.
			&& (bRisingBelow || tDash.m_flAtDistGain <= 0.f || tDash.m_flAtDistGain - 4.f >= vDelta.z + 1.f);

		// Same overhang/below-the-surf floor the grounded grid catch now uses: the dash's analytic
		// grid is free-flight math, so under a ledge it extrapolates past where the real hull bonked
		const bool bDashGridReaches = (tDash.m_flApexGain - 4.f) + 20.f >= vDelta.z;

		if ((tDash.m_bGridCatch && bDashGridReaches) || tDash.m_bCatch || tDash.m_bGraze || bHeightWindow)
		{
			m_iAssistJumpType = JT_AIRDASH;
			m_iAssistJumpStartTick = I::GlobalVars->tickcount;
			pCmd->buttons |= IN_JUMP;
			pCmd->buttons &= ~IN_DUCK; // launch the dash with the same clean buttons the sim flew

			// Steer lock capture, same as the grounded commit - but ONLY when the sim actually
			if (tDash.m_bCatch || tDash.m_bGraze)
			{
				Vec3 vFwd, vRight;
				Math::AngleVectors(Vec3(0.f, pCmd->viewangles.y, 0.f), &vFwd, &vRight, nullptr);
				Vec3 vWish = vFwd * pCmd->forwardmove + vRight * pCmd->sidemove;
				vWish.z = 0.f;
				m_flAssistWishMag = vWish.Length2D();
				if (m_flAssistWishMag > 1.f)
					m_vAssistWishDir = vWish * (1.f / m_flAssistWishMag);
				else
					m_flAssistWishMag = 0.f;
			}
			else
				m_flAssistWishMag = 0.f; // window-fired: your strafe, your path

			m_tAssistJumpBox.m_iJumpType = JT_AIRDASH;
			m_tAssistJumpBox.m_flTargetZ = pTarget->z;
			m_tAssistJumpBox.m_flCurTime = I::GlobalVars->curtime;
		}
	}

	// Post-launch duck choreography: reproduce live EXACTLY what SimJumpCatch validated the pick
	if ((m_iAssistJumpType != JT_NONE || m_iAssistPendingType != JT_NONE) && !bOnGround)
	{
		if (m_iAssistJumpType != JT_NONE)
		{
			const int iSinceLaunch = I::GlobalVars->tickcount - m_iAssistJumpStartTick;
			const bool bLong = m_iAssistJumpType == JT_LONG || m_iAssistJumpType == JT_LONGDUCK;
			const bool bHold = m_iAssistJumpType == JT_CROUCH || m_iAssistJumpType == JT_LONGDUCK;

			if (bHold || (bLong && iSinceLaunch >= 1 && iSinceLaunch <= 2))
				pCmd->buttons |= IN_DUCK;
		}

		// Steer lock: replay the commit-tick wish (the reference replays its recorded sim cmds the same
		// way). World-space, so turning the mouse mid-hop doesn't bend the path - the wish stays
		if (Vars::Misc::Movement::PixelSurfAssistSteer.Value && m_flAssistWishMag > 0.f)
		{
			Vec3 vWishAngle = Math::VectorAngles(m_vAssistWishDir);
			const float flRot = DEG2RAD(vWishAngle.y - pCmd->viewangles.y);
			pCmd->forwardmove = cosf(flRot) * m_flAssistWishMag;
			pCmd->sidemove = -sinf(flRot) * m_flAssistWishMag;
		}
	}
}


// Pixel finder markers + pixel surf assist target markers / aim line.
void CMisc::DrawPixelSurf(CTFPlayer* pLocal)
{
	if (!pLocal || !pLocal->IsAlive() || !I::EngineClient->IsInGame())
	{
		m_iAssistPointHover = -1; // never let a stale hover open the ENTER menu from a dead frame
		return;
	}

	const std::string sMap = I::EngineClient->GetLevelName() ? I::EngineClient->GetLevelName() : "";

	// Distance fade: points (and the finder line) past the assist engage radius + 200u stop
	// rendering, fading out over the last 100u so they don't pop. Distance is from the local origin.
	const Vec3 vLocalOrigin = pLocal->m_vecOrigin();
	const float flFadeCutoff = Vars::Misc::Movement::PixelSurfAssistRadius.Value + 200.f;
	constexpr float flFadeRange = 100.f;
	auto FadeAlpha = [&](const Vec3& vWorld) -> float
	{
		const float flD = vLocalOrigin.DistTo(vWorld);
		if (flD >= flFadeCutoff)
			return 0.f;
		return std::clamp((flFadeCutoff - flD) / flFadeRange, 0.f, 1.f);
	};

	// Pixel finder: every found surf spot on this map, plus the aim line while scanning.
	// The point under the crosshair grows + recolours (so you can see which one you're aiming at)
	if (Vars::Misc::Movement::PixelFinder.Value || !m_vPixelSurfPoints.empty())
	{
		const Color_t tPoint = Color_t(150, 150, 150, 255);     // unselected = gray
		const Color_t tHover = Vars::Menu::Theme::Accent.Value; // selected (crosshair) = accent

		const float flCenterX = H::Draw.m_nScreenW / 2.f;
		const float flCenterY = H::Draw.m_nScreenH / 2.f;
		const float flHoverRadius = H::Draw.Scale(16, Scale_Round); // crosshair pickup radius (px)
		const float flNormalSize = H::Draw.Scale(4, Scale_Round);
		const float flHoverSize = H::Draw.Scale(8, Scale_Round);
		const float flLerp = std::clamp(I::GlobalVars->frametime * 12.f, 0.f, 1.f);

		// Which on-screen point is the crosshair over? (nearest to screen centre within radius)
		int iHover = -1;
		float flHoverBest = flHoverRadius;
		Vec3 vScreen;
		for (int i = 0; i < int(m_vPixelSurfPoints.size()); i++)
		{
			if (m_vPixelSurfPoints[i].m_sMap != sMap)
				continue;
			if (!SDK::W2S(m_vPixelSurfPoints[i].m_vPos, vScreen))
				continue;
			const float flDx = vScreen.x - flCenterX, flDy = vScreen.y - flCenterY;
			const float flDist = sqrtf(flDx * flDx + flDy * flDy);
			if (flDist < flHoverBest)
			{
				flHoverBest = flDist;
				iHover = i;
			}
		}

		// Delete the hovered point on a fresh delete-key press.
		const bool bDel = Vars::Misc::Movement::PixelSurfAssistDeletePoint.Value;
		if (bDel && !m_bPixelDeleteHeldDraw && iHover >= 0)
		{
			m_vPixelSurfPoints.erase(m_vPixelSurfPoints.begin() + iHover);
			iHover = -1;
		}
		m_bPixelDeleteHeldDraw = bDel;

		for (int i = 0; i < int(m_vPixelSurfPoints.size()); i++)
		{
			auto& p = m_vPixelSurfPoints[i];
			if (p.m_sMap != sMap)
				continue;
			if (!SDK::W2S(p.m_vPos, vScreen))
				continue;

			const float flFade = FadeAlpha(p.m_vPos);
			if (flFade <= 0.f)
				continue;

			const bool bHover = (i == iHover);
			const float flTarget = bHover ? flHoverSize : flNormalSize;
			p.m_flSize += (flTarget - p.m_flSize) * flLerp; // smooth grow/shrink (and pop-in from 0)

			const Color_t tCol = bHover ? tHover : tPoint;
			H::Draw.FillCircle(int(vScreen.x), int(vScreen.y), p.m_flSize * 0.6f, 32, Color_t(tCol.r, tCol.g, tCol.b, int(90 * flFade)));
			H::Draw.LineCircle(int(vScreen.x), int(vScreen.y), p.m_flSize, 48, Color_t(tCol.r, tCol.g, tCol.b, int(tCol.a * flFade)));
		}

		if (Vars::Misc::Movement::PixelFinder.Value && m_bPixelFinderFirst)
		{
			Vec3 vA, vB;
			const float flFade = FadeAlpha(m_vPixelFinderEnd);
			if (flFade > 0.f && SDK::W2S(m_vPixelFinderStart, vA) && SDK::W2S(m_vPixelFinderEnd, vB))
				H::Draw.Line(int(vA.x), int(vA.y), int(vB.x), int(vB.y), Color_t(tPoint.r, tPoint.g, tPoint.b, int(tPoint.a * flFade)));
		}
	}

	// Pixel surf line: the path of the surf in progress (live) plus every saved surfable strip on this
	// map, drawn as a connected line in the configured colour. The saved strip nearest the crosshair
	if (Vars::Misc::Movement::PixelSurfLine.Value)
	{
		const Color_t tLine = Vars::Misc::Movement::PixelSurfLineColor.Value;
		// Surf lines get their own far fade - the assist-point cutoff (radius+200, ~500u) was clipping
		constexpr float flSurfFadeCutoff = 4096.f;
		constexpr float flSurfFadeRange = 256.f;
		auto SurfFade = [&](const Vec3& vWorld) -> float
		{
			const float flD = vLocalOrigin.DistTo(vWorld);
			if (flD >= flSurfFadeCutoff)
				return 0.f;
			return std::clamp((flSurfFadeCutoff - flD) / flSurfFadeRange, 0.f, 1.f);
		};
		const float flCenterX = H::Draw.m_nScreenW / 2.f;
		const float flCenterY = H::Draw.m_nScreenH / 2.f;
		const float flHoverRadius = H::Draw.Scale(20, Scale_Round); // crosshair pickup radius (px)

		// Which strip is the crosshair over? (its nearest on-screen point within the pickup radius)
		int iHoverSeg = -1;
		float flHoverBest = flHoverRadius;
		Vec3 vScreenPt;
		for (int s = 0; s < int(m_vSurfLineSegments.size()); s++)
		{
			if (m_vSurfLineSegments[s].m_sMap != sMap)
				continue;
			for (const auto& p : m_vSurfLineSegments[s].m_vPoints)
			{
				if (!SDK::W2S(p, vScreenPt))
					continue;
				const float flDx = vScreenPt.x - flCenterX, flDy = vScreenPt.y - flCenterY;
				const float flDist = sqrtf(flDx * flDx + flDy * flDy);
				if (flDist < flHoverBest)
				{
					flHoverBest = flDist;
					iHoverSeg = s;
				}
			}
		}

		// Delete the hovered strip on a fresh delete-key press (own edge so it doesn't race the finder's).
		const bool bDel = Vars::Misc::Movement::PixelSurfAssistDeletePoint.Value;
		if (bDel && !m_bSurfLineDeleteHeldDraw && iHoverSeg >= 0)
		{
			m_vSurfLineSegments.erase(m_vSurfLineSegments.begin() + iHoverSeg);
			iHoverSeg = -1;
		}
		m_bSurfLineDeleteHeldDraw = bDel;

		for (int s = 0; s < int(m_vSurfLineSegments.size()); s++)
		{
			const auto& seg = m_vSurfLineSegments[s];
			if (seg.m_sMap != sMap || seg.m_vPoints.size() < 2)
				continue;
			const Color_t tCol = tLine; // always configured colour - no hover highlight (delete still works)
			Vec3 vA, vB;
			for (size_t i = 1; i < seg.m_vPoints.size(); i++)
			{
				const float flFade = SurfFade(seg.m_vPoints[i]);
				if (flFade <= 0.f)
					continue;
				if (SDK::W2S(seg.m_vPoints[i - 1], vA) && SDK::W2S(seg.m_vPoints[i], vB))
					H::Draw.Line(int(vA.x), int(vA.y), int(vB.x), int(vB.y), Color_t(tCol.r, tCol.g, tCol.b, int(tCol.a * flFade)));
			}
		}

		// The surf in progress: draw the path traced so far this ride (it becomes a saved strip on landing).
		if (m_vSurfLineCurrent.size() >= 2)
		{
			Vec3 vA, vB;
			for (size_t i = 1; i < m_vSurfLineCurrent.size(); i++)
			{
				const float flFade = SurfFade(m_vSurfLineCurrent[i]);
				if (flFade <= 0.f)
					continue;
				if (SDK::W2S(m_vSurfLineCurrent[i - 1], vA) && SDK::W2S(m_vSurfLineCurrent[i], vB))
					H::Draw.Line(int(vA.x), int(vA.y), int(vB.x), int(vB.y), Color_t(tLine.r, tLine.g, tLine.b, int(tLine.a * flFade)));
			}
		}
	}

	// Pixel surf assist: saved target points on this map. The active target (nearest ACTIVE point
	m_iAssistPointHover = -1;
	if (Vars::Misc::Movement::PixelSurfAssistRender.Value && !m_vAssistPoints.empty())
	{
		const Color_t tSelected = Vars::Menu::Theme::Accent.Value;
		const Color_t tUnselected = Color_t(150, 150, 150, 255);
		const Color_t tInactive = Color_t(90, 90, 90, 255);

		const float flCenterX = H::Draw.m_nScreenW / 2.f;
		const float flCenterY = H::Draw.m_nScreenH / 2.f;
		const float flHoverRadius = H::Draw.Scale(16, Scale_Round); // crosshair pickup radius (px)

		// Which saved point is the active target? Same rules as the picker: active points only,
		// per-point radius (from the ENTER menu) beating the global slider when set.
		int iActive = -1;
		float flActiveBest = 1e9f;
		for (int i = 0; i < int(m_vAssistPoints.size()); i++)
		{
			const auto& p = m_vAssistPoints[i];
			if (p.m_sMap != sMap || !p.m_bActive)
				continue;
			const float flRadius = p.m_flRadius > 0.f ? p.m_flRadius : Vars::Misc::Movement::PixelSurfAssistRadius.Value;
			const float flDist = vLocalOrigin.DistTo(p.m_vPos);
			if (flDist > flRadius || flDist >= flActiveBest)
				continue;
			flActiveBest = flDist;
			iActive = i;
		}

		// First pass: find the hovered point (nearest to the crosshair inside the pickup radius).
		float flHoverBest = flHoverRadius;
		Vec3 vScreen;
		for (int i = 0; i < int(m_vAssistPoints.size()); i++)
		{
			const auto& p = m_vAssistPoints[i];
			if (p.m_sMap != sMap || FadeAlpha(p.m_vPos) <= 0.f || !SDK::W2S(p.m_vPos, vScreen))
				continue;
			const float flScrDist = sqrtf(powf(vScreen.x - flCenterX, 2.f) + powf(vScreen.y - flCenterY, 2.f));
			if (flScrDist < flHoverBest)
			{
				flHoverBest = flScrDist;
				m_iAssistPointHover = i;
			}
		}

		for (int i = 0; i < int(m_vAssistPoints.size()); i++)
		{
			const auto& p = m_vAssistPoints[i];
			if (p.m_sMap != sMap)
				continue;
			const float flFade = FadeAlpha(p.m_vPos);
			if (flFade <= 0.f)
				continue;
			if (SDK::W2S(p.m_vPos, vScreen))
			{
				const Color_t& tBase = !p.m_bActive ? tInactive : (i == iActive) ? tSelected : tUnselected;
				const Color_t tCol = Color_t(tBase.r, tBase.g, tBase.b, int(tBase.a * flFade));
				const bool bBig = i == m_iAssistPointHover || i == m_iAssistPointMenu;
				H::Draw.LineCircle(int(vScreen.x), int(vScreen.y), bBig ? 11.f : 7.f, 48, tCol);
			}
		}
	}

	// Jump-box pop-up is drawn in the ImGui pass (CMenu::DrawPixelSurfAssistBox) so it can share the
	// watermark's exact font + rounded background. See GetAssistJumpBox() for the text/fade/expiry.
}

// Feeds the ImGui-side jump-box draw: builds the "minijump (duck) to <z>" string and the fade
// alpha [0..1], and expires the box ~3s after the jump. Returns false when there's nothing to show.
bool CMisc::GetAssistJumpBox(std::string& sTextOut, float& flFadeOut)
{
	if (!Vars::Misc::Movement::PixelSurfAssistJumpBox.Value || m_tAssistJumpBox.m_iJumpType == JT_NONE)
		return false;

	constexpr float flShow = 3.f; // seconds the box stays up after the jump (timer is the ONLY expiry)
	const float flAge = I::GlobalVars->curtime - m_tAssistJumpBox.m_flCurTime;
	if (flAge >= flShow)
	{
		m_tAssistJumpBox.m_iJumpType = JT_NONE; // expired - stop drawing
		return false;
	}
	if (flAge < 0.f)
		return false;

	static const char* aszNames[] =
	{
		"",                  // JT_NONE
		"regular jump",      // JT_REGULAR
		"crouch jump",       // JT_CROUCH
		"minijump",          // JT_MINI
		"minijump (duck)",   // JT_MINIDUCK
		"longjump",          // JT_LONG
		"longjump (duck)",   // JT_LONGDUCK
		"double jump",       // JT_AIRDASH (scout)
	};
	const int iType = m_tAssistJumpBox.m_iJumpType;
	const char* szName = (iType >= JT_REGULAR && iType <= JT_AIRDASH) ? aszNames[iType] : "jump";

	sTextOut = std::format("{} to {:.3f}", szName, m_tAssistJumpBox.m_flTargetZ);
	flFadeOut = std::clamp((flShow - flAge) / 0.4f, 0.f, 1.f); // fade the last ~0.4s
	return true;
}

// ----------------------------- Pixel surf assist point persistence -----------------------------
// Saved assist targets persist to disk so they survive map reloads / restarts. One plain-text file
static std::string PointsDir()
{
	std::string sDir = F::Configs.m_sConfigPath + "Points\\";
	try { if (!std::filesystem::exists(sDir)) std::filesystem::create_directories(sDir); } catch (...) {}
	return sDir;
}
static std::string PointsFile(const std::string& sMap)
{
	// Map names can contain '/' (e.g. workshop/x.ugc123) - flatten to a safe filename.
	std::string sSafe = sMap.empty() ? "unknown" : sMap;
	for (auto& c : sSafe) if (c == '/' || c == '\\' || c == ':') c = '_';
	return PointsDir() + sSafe + ".txt";
}

void CMisc::LoadAssistPoints(const std::string& sMap)
{
	// Drop this map's points from the combined list, then reload them from disk.
	std::erase_if(m_vAssistPoints, [&](const PixelSurfPoint_t& p) { return p.m_sMap == sMap; });
	m_sAssistPointsMap = sMap;

	std::ifstream f(PointsFile(sMap));
	if (!f.is_open())
		return;

	// Line format: "x y z [active radius typemask]" - the per-point settings are optional so
	// files written before the on-point ENTER menu existed still load with defaults.
	std::string sLine;
	while (std::getline(f, sLine))
	{
		std::istringstream ss(sLine);
		PixelSurfPoint_t p;
		if (!(ss >> p.m_vPos.x >> p.m_vPos.y >> p.m_vPos.z))
			continue;
		int iActive = 1;
		if (ss >> iActive >> p.m_flRadius >> p.m_iTypeMask)
		{
			p.m_bActive = iActive != 0;
			if (p.m_iTypeMask == 63)
				p.m_iTypeMask = 127; // pre-air-dash default ("all types") gains the new dash bit
		}
		p.m_sMap = sMap;
		m_vAssistPoints.push_back(p);
	}
}

bool CMisc::WriteAssistPoints(const std::string& sMap)
{
	std::ofstream f(PointsFile(sMap), std::ios::trunc);
	if (!f.is_open())
		return false;

	for (const auto& p : m_vAssistPoints)
	{
		if (p.m_sMap != sMap)
			continue;
		f << p.m_vPos.x << ' ' << p.m_vPos.y << ' ' << p.m_vPos.z << ' '
			<< (p.m_bActive ? 1 : 0) << ' ' << p.m_flRadius << ' ' << p.m_iTypeMask << '\n';
	}
	return bool(f);
}

// --------------------------- On-point ENTER menu (per-point settings) ---------------------------
CMisc::PixelSurfPoint_t* CMisc::AssistPointMenuPoint()
{
	if (m_iAssistPointMenu < 0 || m_iAssistPointMenu >= int(m_vAssistPoints.size()))
		return nullptr;
	return &m_vAssistPoints[m_iAssistPointMenu];
}

void CMisc::CloseAssistPointMenu(bool bSave)
{
	if (bSave)
	{
		if (auto* p = AssistPointMenuPoint())
			WriteAssistPoints(p->m_sMap);
	}
	m_iAssistPointMenu = -1;
}

void CMisc::DeleteAssistPointMenuPoint()
{
	if (auto* p = AssistPointMenuPoint())
	{
		const std::string sMap = p->m_sMap;
		m_vAssistPoints.erase(m_vAssistPoints.begin() + m_iAssistPointMenu);
		WriteAssistPoints(sMap);
	}
	m_iAssistPointMenu = -1;
}

// ----------------------------- Movement recorder () -----------------------------
// Records the usercmd sent each tick and replays it verbatim. Each map gets ONE file that holds
static constexpr uint32_t REC_MAGIC = 0x33524D52; // "RMR3"

// The integer tickrate of the current session (66 on stock TF2, 64 on a 64-tick server). Truncated,
// NOT rounded, so 1/0.015 -> 66 /the reference (rounding would give a bogus 67). Per-tick
static int RecorderTickrate()
{
	const float flInterval = I::GlobalVars ? I::GlobalVars->interval_per_tick : 0.015f;
	const int iTick = (flInterval > 0.f) ? int(1.f / flInterval) : 66;
	return std::clamp(iTick, 1, 1000);
}

static std::string RecorderDir()
{
	std::string sDir = F::Configs.m_sConfigPath + "Recordings\\";
	try { if (!std::filesystem::exists(sDir)) std::filesystem::create_directories(sDir); } catch (...) {}
	return sDir;
}
static std::string RecorderFile(const std::string& sMap)
{
	// Map names can contain '/' (e.g. workshop/x.ugc123) - flatten to a safe filename.
	std::string sSafe = sMap.empty() ? "unknown" : sMap;
	for (auto& c : sSafe) if (c == '/' || c == '\\' || c == ':') c = '_';
	// e.g. "pl_upward_66.rec" - the tickrate suffix keeps 64- and 66-tick routes in separate files.
	return RecorderDir() + sSafe + "_" + std::to_string(RecorderTickrate()) + ".rec";
}

std::string CMisc::RecorderCurrentMap() const
{
	return I::EngineClient->GetLevelName() ? I::EngineClient->GetLevelName() : "";
}

void CMisc::LoadRoutes(const std::string& sMap)
{
	m_vRoutes.clear();
	m_sRoutesMap = sMap;
	m_iSelectedRoute = -1;

	std::ifstream f(RecorderFile(sMap), std::ios::binary);
	if (!f.is_open())
		return;

	uint32_t uMagic = 0, uRoutes = 0;
	f.read(reinterpret_cast<char*>(&uMagic), sizeof(uMagic));
	f.read(reinterpret_cast<char*>(&uRoutes), sizeof(uRoutes));
	if (!f || uMagic != REC_MAGIC || uRoutes == 0 || uRoutes > 4096)
		return;

	for (uint32_t r = 0; r < uRoutes; r++)
	{
		uint32_t uNameLen = 0;
		f.read(reinterpret_cast<char*>(&uNameLen), sizeof(uNameLen));
		if (!f || uNameLen > 256)
			return;
		Route_t tRoute;
		tRoute.m_sName.resize(uNameLen);
		if (uNameLen)
			f.read(tRoute.m_sName.data(), uNameLen);

		uint32_t uFrames = 0;
		f.read(reinterpret_cast<char*>(&uFrames), sizeof(uFrames));
		if (!f || uFrames == 0 || uFrames > 200000)
			return;
		tRoute.m_vFrames.resize(uFrames);
		f.read(reinterpret_cast<char*>(tRoute.m_vFrames.data()), std::streamsize(uFrames * sizeof(RecFrame_t)));
		if (!f)
			return;
		m_vRoutes.push_back(std::move(tRoute));
	}
	// Leave nothing selected: Play then defaults to the NEAREST route's start (move-to-start),
	// while the menu can still pick a specific route to force it.
}

bool CMisc::WriteRoutes(const std::string& sMap)
{
	std::ofstream f(RecorderFile(sMap), std::ios::binary | std::ios::trunc);
	if (!f.is_open())
		return false;

	const uint32_t uMagic = REC_MAGIC;
	const uint32_t uRoutes = uint32_t(m_vRoutes.size());
	f.write(reinterpret_cast<const char*>(&uMagic), sizeof(uMagic));
	f.write(reinterpret_cast<const char*>(&uRoutes), sizeof(uRoutes));
	for (const auto& tRoute : m_vRoutes)
	{
		const uint32_t uNameLen = uint32_t(tRoute.m_sName.size());
		const uint32_t uFrames = uint32_t(tRoute.m_vFrames.size());
		f.write(reinterpret_cast<const char*>(&uNameLen), sizeof(uNameLen));
		f.write(tRoute.m_sName.data(), uNameLen);
		f.write(reinterpret_cast<const char*>(&uFrames), sizeof(uFrames));
		f.write(reinterpret_cast<const char*>(tRoute.m_vFrames.data()), std::streamsize(uFrames * sizeof(RecFrame_t)));
	}
	return bool(f);
}

void CMisc::RecorderSaveActiveAs(const std::string& sName)
{
	if (m_vRecording.empty())
		return;
	const std::string sMap = RecorderCurrentMap();
	if (m_sRoutesMap != sMap)
		LoadRoutes(sMap);

	Route_t tRoute;
	tRoute.m_sName = sName.empty() ? std::format("route {}", m_vRoutes.size() + 1) : sName;
	tRoute.m_vFrames = m_vRecording;
	m_vRoutes.push_back(std::move(tRoute));
	m_iSelectedRoute = int(m_vRoutes.size()) - 1;
	WriteRoutes(sMap);
}

void CMisc::RecorderDeleteRoute(int i)
{
	if (i < 0 || i >= int(m_vRoutes.size()))
		return;
	m_vRoutes.erase(m_vRoutes.begin() + i);
	if (m_iSelectedRoute >= int(m_vRoutes.size()))
		m_iSelectedRoute = int(m_vRoutes.size()) - 1;
	WriteRoutes(m_sRoutesMap.empty() ? RecorderCurrentMap() : m_sRoutesMap);
}

void CMisc::RecorderReload()
{
	LoadRoutes(RecorderCurrentMap());
}

// Cut flStartSec off the front and flEndSec off the back of a saved route, then persist. Always
// leaves at least one frame so the route (and its start marker) survives.
void CMisc::RecorderTrimRoute(int i, float flStartSec, float flEndSec)
{
	if (i < 0 || i >= int(m_vRoutes.size()))
		return;
	auto& vFrames = m_vRoutes[i].m_vFrames;
	const int iTotal = int(vFrames.size());
	if (iTotal <= 1)
		return;

	int iCutFront = std::max(0, int(std::lround(flStartSec / TICK_INTERVAL)));
	int iCutBack = std::max(0, int(std::lround(flEndSec / TICK_INTERVAL)));
	// Keep at least one frame: never let the two cuts overlap into nothing.
	if (iCutFront + iCutBack >= iTotal)
		iCutBack = std::max(0, iTotal - 1 - iCutFront);
	if (iCutFront >= iTotal)
		iCutFront = iTotal - 1;

	if (iCutBack > 0)
		vFrames.erase(vFrames.end() - iCutBack, vFrames.end());
	if (iCutFront > 0)
		vFrames.erase(vFrames.begin(), vFrames.begin() + iCutFront);

	WriteRoutes(m_sRoutesMap.empty() ? RecorderCurrentMap() : m_sRoutesMap);
}

// Velocity-aware "arrive" steer (replaces the reference plain proportional goingtostart).
static void RecorderSteerToward(CUserCmd* pCmd, const Vec3& vFrom, const Vec3& vVel, const Vec3& vTarget)
{
	const Vec3 vRem = { vTarget.x - vFrom.x, vTarget.y - vFrom.y, 0.f };
	const float flR = vRem.Length2D();
	const float flDt = std::max(TICK_INTERVAL, 0.0001f);

	// Velocity we'd want this tick: just enough to land on the target in one tick, capped to a sane
	// max. As flR -> 0 the wanted speed -> 0, so the controller eases to a dead stop on the point.
	Vec3 vWish;
	if (flR > 1e-5f)
	{
		const float flDesired = std::min(flR / flDt, 450.f);
		const Vec3 vDir = vRem / flR;
		vWish = { vDir.x * flDesired - vVel.x, vDir.y * flDesired - vVel.y, 0.f }; // desired velocity - current
	}
	else
		vWish = { -vVel.x, -vVel.y, 0.f }; // dead on target: just cancel any residual drift

	// Rotate the world wish into forward/side (same yaw convention as the old steer).
	const float flYawRad = DEG2RAD(pCmd->viewangles.y);
	const float flCos = cosf(flYawRad), flSin = sinf(flYawRad);
	pCmd->forwardmove = std::clamp(vWish.x * flCos + vWish.y * flSin, -450.f, 450.f);
	pCmd->sidemove = std::clamp(vWish.x * flSin - vWish.y * flCos, -450.f, 450.f);
}

// ----------------------------- Checkpoints -----------------------------
// KZ-practice checkpoints, /the reference EXACTLY: ONE saved spot. The Save key
void CMisc::Checkpoints(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	if (!Vars::Misc::Movement::Checkpoints.Value || !pLocal)
		return;

	const int iSaveKey = Vars::Misc::Movement::CheckpointSaveKey.Value;
	const int iTpKey = Vars::Misc::Movement::CheckpointTeleportKey.Value;
	const int iNoclipKey = Vars::Misc::Movement::CheckpointNoclipKey.Value;

	// Refresh the key state every frame (even with the menu open) so no stale press leaks an action
	// the moment the menu closes; act on the release edge below.
	const bool bSave = iSaveKey && U::KeyHandler.Released(iSaveKey, true);
	const bool bTp = iTpKey && U::KeyHandler.Released(iTpKey, true);
	const bool bNoclip = iNoclipKey && U::KeyHandler.Released(iNoclipKey, true);

	// Don't fire while the cheat menu is open (so binding a key doesn't also trigger its action), and
	// only with sv_cheats (setpos/setang/noclip need it - CheatsBypass can force it on).
	static auto sv_cheats = H::ConVars.FindVar("sv_cheats");
	if (!sv_cheats || !sv_cheats->GetBool())
		return;

	const auto fnChat = [&](const std::string& sMsg)
	{
		if (I::ClientModeShared && I::ClientModeShared->m_pChatElement)
			I::ClientModeShared->m_pChatElement->ChatPrintf(0, std::format("{}{} \x07" "FFFFFF" "| {}",
				Vars::Menu::Theme::Accent.Value.ToHex(), Vars::Menu::CheatTitle.Value, sMsg).c_str());
	};

	// Save: store the current origin + view.
	if (bSave)
	{
		if (!(pLocal->m_fFlags() & FL_ONGROUND))
			fnChat("you need to be onground to set a checkpoint.");
		else
		{
			m_vCheckpointPos = pLocal->m_vecOrigin();
			m_vCheckpointAng = I::EngineClient->GetViewAngles();
			m_bHasCheckpoint = true;
			fnChat("saved checkpoint.");
		}
	}

	// Teleport: setpos + setang back to the saved spot.
	if (bTp && m_bHasCheckpoint)
	{
		I::EngineClient->ClientCmd_Unrestricted(std::format("setpos {} {} {};setang {} {} {}",
			m_vCheckpointPos.x, m_vCheckpointPos.y, m_vCheckpointPos.z,
			m_vCheckpointAng.x, m_vCheckpointAng.y, m_vCheckpointAng.z).c_str());
	}

	// Noclip: toggle (the third row in the reference window's build).
	if (bNoclip)
		I::EngineClient->ClientCmd_Unrestricted("noclip");
}

void CMisc::MovementRecorder(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	if (!Vars::Misc::Movement::MovementRecorder.Value || !pLocal || !pLocal->IsAlive())
		return;

	const std::string sMap = RecorderCurrentMap();
	if (sMap != m_sRoutesMap) // map changed (or first run): pull this map's saved routes from disk
		LoadRoutes(sMap);

	const Vec3 vOrigin = pLocal->m_vecOrigin();

	const bool bRec = Vars::Misc::Movement::MovementRecorderRecord.Value;
	const bool bPlay = Vars::Misc::Movement::MovementRecorderPlay.Value;
	const bool bStop = Vars::Misc::Movement::MovementRecorderStop.Value;
	const bool bSave = Vars::Misc::Movement::MovementRecorderSave.Value;
	const bool bLoad = Vars::Misc::Movement::MovementRecorderLoad.Value;
	const bool bClear = Vars::Misc::Movement::MovementRecorderClear.Value;

	const bool bBusy = m_bPlaying || m_bMovingToStart || m_bWishToStart;

	if (bRec && !m_bRecKeyHeld) // record toggle (cancels any playback)
	{
		m_bRecording = !m_bRecording;
		m_bPlaying = m_bMovingToStart = m_bSmoothToStart = m_bWishToStart = false;
		if (m_bRecording) { m_vRecording.clear(); m_iPlayRoute = -1; }
	}
	m_bRecKeyHeld = bRec;

	if (bPlay && !m_bPlayKeyHeld && !m_bRecording && !bBusy) // begin a route: move-to-start then replay
	{
		// Prefer the menu-selected route; else the nearest route start to the player.
		int iRoute = -1;
		if (m_iSelectedRoute >= 0 && m_iSelectedRoute < int(m_vRoutes.size()) && !m_vRoutes[m_iSelectedRoute].m_vFrames.empty())
			iRoute = m_iSelectedRoute;
		else
		{
			float flBest = float(Vars::Misc::Movement::MovementRecorderMoveToStartDist.Value);
			if (flBest <= 0.f) flBest = 99999.f;
			for (int i = 0; i < int(m_vRoutes.size()); i++)
			{
				if (m_vRoutes[i].m_vFrames.empty())
					continue;
				const float flDist = vOrigin.DistTo(m_vRoutes[i].m_vFrames.front().m_vOrigin);
				if (flDist < flBest) { flBest = flDist; iRoute = i; }
			}
		}

		if (iRoute >= 0)
		{
			m_iPlayRoute = iRoute;
			m_vRecording = m_vRoutes[iRoute].m_vFrames; // active buffer = the route being replayed
			m_iPlaybackIdx = 0;

			// A route that started while already moving fast can't be matched by creeping in (the arrive
			// controller decelerates to a stop on the point), so its trick won't reproduce reliably - flag
			m_bRouteUnreliable = m_vRecording.front().m_vVelocity.Length2D() > 30.f;

			const Vec3 vStart = m_vRecording.front().m_vOrigin;
			const float flTol = Vars::Misc::Movement::MovementRecorderStartTolerance.Value;
			const float flSpd = float(Vars::Misc::Movement::MovementRecorderStartSpeed.Value);
			// Only skip the approach if we're ALREADY at the exact start AND near standstill; otherwise
			// creep in (move-to-start) so position + velocity match the recording before frame 0 runs.
			if (Vars::Misc::Movement::MovementRecorderMoveToStart.Value
				&& (vOrigin.DistTo(vStart) > flTol || pLocal->m_vecVelocity().Length2D() > flSpd))
			{
				m_bMovingToStart = true;
				m_bSmoothToStart = Vars::Misc::Movement::MovementRecorderLockView.Value;
				m_bWishToStart = true;
				m_iSettleTicks = m_iStallTicks = 0;
				m_flAlignBest = FLT_MAX;
			}
			else
				m_bPlaying = true; // already at the start: replay immediately
		}
	}
	m_bPlayKeyHeld = bPlay;

	if (bStop && !m_bStopKeyHeld) // stop playback / move-to-start (leaves recording alone)
	{
		m_bPlaying = m_bMovingToStart = m_bSmoothToStart = m_bWishToStart = false;
		m_iPlaybackIdx = 0;
	}
	m_bStopKeyHeld = bStop;

	if (bSave && !m_bSaveKeyHeld) RecorderSaveActiveAs(""); // quick-save the active recording (auto name)
	m_bSaveKeyHeld = bSave;

	if (bLoad && !m_bLoadKeyHeld) LoadRoutes(sMap);
	m_bLoadKeyHeld = bLoad;

	if (bClear && !m_bClearKeyHeld) { m_vRecording.clear(); m_bRecording = m_bPlaying = m_bMovingToStart = m_bWishToStart = false; m_iPlaybackIdx = 0; }
	m_bClearKeyHeld = bClear;

	// --- Shadowplay: the save bind dumps the rolling buffer into the active recording, so you can
	// keep a moment after it happened (then Save it as a route, or Play it back immediately). ---
	const bool bShadowSave = Vars::Misc::Movement::MovementRecorderShadowSave.Value;
	if (bShadowSave && !m_bShadowSaveHeld && !m_dqShadow.empty())
	{
		m_vRecording.assign(m_dqShadow.begin(), m_dqShadow.end()); // last N seconds become the active recording
		m_bRecording = m_bPlaying = m_bMovingToStart = m_bWishToStart = m_bSmoothToStart = false;
		m_iPlaybackIdx = 0;
		m_iPlayRoute = -1;
	}
	m_bShadowSaveHeld = bShadowSave;

	// While idle, keep a rolling buffer of the last N seconds of movement. Game thread only - the
	// render thread never reads m_dqShadow, so there's no race. Freed when shadowplay is off.
	if (Vars::Misc::Movement::MovementRecorderShadowPlay.Value
		&& !m_bRecording && !m_bPlaying && !m_bMovingToStart && !m_bWishToStart)
	{
		RecFrame_t tShadow;
		tShadow.m_vViewAngles = pCmd->viewangles;
		tShadow.m_flForward = pCmd->forwardmove;
		tShadow.m_flSide = pCmd->sidemove;
		tShadow.m_flUp = pCmd->upmove;
		tShadow.m_iButtons = pCmd->buttons;
		tShadow.m_vOrigin = vOrigin;
		tShadow.m_vVelocity = pLocal->m_vecVelocity();
		m_dqShadow.push_back(tShadow);
		const size_t iMax = std::max<size_t>(1, size_t(Vars::Misc::Movement::MovementRecorderShadowSeconds.Value / TICK_INTERVAL));
		while (m_dqShadow.size() > iMax)
			m_dqShadow.pop_front();
	}
	else if (!Vars::Misc::Movement::MovementRecorderShadowPlay.Value && !m_dqShadow.empty())
		m_dqShadow.clear();

	// --- Recording: capture the final command + position each tick ---
	if (m_bRecording)
	{
		RecFrame_t tFrame;
		tFrame.m_vViewAngles = pCmd->viewangles;
		tFrame.m_flForward = pCmd->forwardmove;
		tFrame.m_flSide = pCmd->sidemove;
		tFrame.m_flUp = pCmd->upmove;
		tFrame.m_iButtons = pCmd->buttons;
		tFrame.m_vOrigin = vOrigin;
		tFrame.m_vVelocity = pLocal->m_vecVelocity(); // pre-sim velocity - the start frame's is the gate target
		m_vRecording.push_back(tFrame);
		if (m_vRecording.size() > 120000) // ~30 min @ 66 tick safety cap
			m_bRecording = false;
		return;
	}

	if (m_vRecording.empty())
		return;
	const Vec3 vStart = m_vRecording.front().m_vOrigin;

	// --- Move-to-start: creep to the EXACT recorded start, align the view, then gate the replay ---
	// Playback only begins once position and velocity match the recorded start.
	if (m_bMovingToStart)
	{
		const Vec3 vVel = pLocal->m_vecVelocity();
		const float flDist = vOrigin.DistTo(vStart);
		const float flSpeed = vVel.Length2D();
		m_flAlignDist = flDist;   // HUD feedback
		m_flAlignSpeed = flSpeed;

		// Track the closest we've gotten. Once the distance stops improving we've hit the floor that
		// ground physics allows for this point, so there's no value in waiting out the full timeout.
		if (flDist < m_flAlignBest - 0.003f) { m_flAlignBest = flDist; m_iStallTicks = 0; }
		else m_iStallTicks++;

		// Velocity-aware arrive: decelerate onto the EXACT point and keep shimmying it tighter.
		RecorderSteerToward(pCmd, vOrigin, vVel, vStart);

		// Optionally rotate the view onto the recorded start angle while approaching.
		bool bViewOK = true;
		if (Vars::Misc::Movement::MovementRecorderLockView.Value)
		{
			pCmd->mousedx = pCmd->mousedy = 0;
			const Vec3& vWant = m_vRecording.front().m_vViewAngles;
			Vec3 vDelta = { vWant.x - pCmd->viewangles.x, Math::NormalizeAngle(vWant.y - pCmd->viewangles.y), 0.f };
			bViewOK = vDelta.Length2D() < 1.f;
			if (!bViewOK)
			{
				pCmd->viewangles.x += vDelta.x / 8.f;
				pCmd->viewangles.y += vDelta.y / 8.f;
				I::EngineClient->SetViewAngles(pCmd->viewangles);
			}
		}

		// Gate: exact position + settled speed (+ view aligned). If the route was recorded mid-motion
		// (start speed already high), we can't match that by creeping in, so gate on position only.
		const float flTol = Vars::Misc::Movement::MovementRecorderStartTolerance.Value;
		const float flSpd = float(Vars::Misc::Movement::MovementRecorderStartSpeed.Value);
		const Vec3& vRecStartVel = m_vRecording.front().m_vVelocity;
		const float flRecStartSpeed = vRecStartVel.Length2D();
		const bool bPosOK = flDist <= flTol;
		// Match the recorded start VELOCITY as a vector (not just our raw speed): a route recorded at rest
		const Vec3 vVelErr = { vVel.x - vRecStartVel.x, vVel.y - vRecStartVel.y, 0.f };
		const bool bSpeedOK = (flRecStartSpeed > 30.f) ? true : (vVelErr.Length2D() <= flSpd);
		// Settled: the shimmy hasn't improved for ~0.25s and we're already close - that's the physical
		// floor, so commit there rather than burning the rest of the timeout chasing a decimal physics
		const bool bSettled = m_iStallTicks > int(0.25f / TICK_INTERVAL) && flDist <= flTol * 4.f;
		// Safety timeout (~3s) so an unreachable start can never hard-lock.
		const bool bTimeout = ++m_iSettleTicks > int(3.f / TICK_INTERVAL);

		if (((bPosOK || bSettled) && bSpeedOK && bViewOK) || bTimeout)
		{
			m_bMovingToStart = m_bSmoothToStart = m_bWishToStart = false;
			m_bPlaying = true;
			m_iPlaybackIdx = 0;
			// fall through to playback so frame 0 runs from the matched state THIS tick
		}
	}

	// --- Playback: replay the route verbatim ---
	if (m_bPlaying)
	{
		if (m_iPlaybackIdx >= m_vRecording.size())
		{
			m_bPlaying = false;
		}
		else
		{
			const RecFrame_t& tFrame = m_vRecording[m_iPlaybackIdx++];
			pCmd->viewangles = tFrame.m_vViewAngles;
			pCmd->forwardmove = tFrame.m_flForward;
			pCmd->sidemove = tFrame.m_flSide;
			pCmd->upmove = tFrame.m_flUp;
			pCmd->buttons = tFrame.m_iButtons;

			// --- Drift correction (closed loop) ---
			bool bDoDrift = Vars::Misc::Movement::MovementRecorderDriftCorrect.Value
				&& !Vars::Misc::Movement::MovementRecorderVerbatim.Value
				&& (pLocal->m_fFlags() & FL_ONGROUND);
			if (bDoDrift)
			{
				// Freeze correction for a window around any jump/duck transition - those are the trick's own
				// setup ticks (e.g. the duck that arms a pixelsurf, the jump out of an edgebug).
				const size_t iCur = m_iPlaybackIdx - 1; // tFrame's index (idx was already incremented)
				constexpr int iEdgeWin = 4;
				const int iButtonMask = IN_JUMP | IN_DUCK;
				const size_t iLo = (iCur > size_t(iEdgeWin)) ? iCur - iEdgeWin : 0;
				const size_t iHi = std::min(iCur + iEdgeWin, m_vRecording.size() - 1);
				for (size_t i = iLo; i < iHi; i++)
				{
					if ((m_vRecording[i].m_iButtons & iButtonMask) != (m_vRecording[i + 1].m_iButtons & iButtonMask))
					{
						bDoDrift = false;
						break;
					}
				}
			}
			if (bDoDrift)
			{
				const Vec3 vErr = { tFrame.m_vOrigin.x - vOrigin.x, tFrame.m_vOrigin.y - vOrigin.y, 0.f };
				const float flErr = vErr.Length2D();
				if (flErr > 0.5f) // deadzone (u): below this, replay verbatim
				{
					const float flCap = Vars::Misc::Movement::MovementRecorderDriftStrength.Value;
					const float flDesired = std::min(flErr / std::max(TICK_INTERVAL, 0.0001f), flCap);
					const Vec3 vDir = vErr / flErr;
					const float flYawRad = DEG2RAD(pCmd->viewangles.y);
					const float flCos = cosf(flYawRad), flSin = sinf(flYawRad);
					const float flFwd = vDir.x * flCos + vDir.y * flSin;
					const float flSide = vDir.x * flSin - vDir.y * flCos;
					pCmd->forwardmove = std::clamp(pCmd->forwardmove + flFwd * flDesired, -450.f, 450.f);
					pCmd->sidemove = std::clamp(pCmd->sidemove + flSide * flDesired, -450.f, 450.f);
				}
			}

			I::EngineClient->SetViewAngles(pCmd->viewangles); // keep the view following the replayed route
		}
	}
}

void CMisc::DrawRecorder()
{
	if (!Vars::Misc::Movement::MovementRecorder.Value || !Vars::Misc::Movement::MovementRecorderHud.Value)
		return;
	if (!m_bRecording && !m_bPlaying && !m_bMovingToStart)
		return;

	// Title + value-in-seconds + tint per state. Ticks -> seconds via the engine tick interval.
	std::string sTitle, sValue;
	Color_t tCol;
	if (m_bRecording)
	{
		sTitle = "recording";
		sValue = std::format("{:.1f}", m_vRecording.size() * TICK_INTERVAL); // elapsed seconds
		tCol = Color_t(255, 80, 80, 255);
	}
	else if (m_bMovingToStart)
	{
		// "aligning" while still walking in; "settling" once on the point but waiting for speed to
		// bleed off (so frame 0 starts from the recorded standstill). Show the live distance.
		const float flTol = Vars::Misc::Movement::MovementRecorderStartTolerance.Value;
		sTitle = (m_flAlignDist <= flTol) ? "settling" : "aligning";
		sValue = std::format("{:.2f}u", m_flAlignDist);
		tCol = Color_t(255, 210, 90, 255);
	}
	else // playing
	{
		sTitle = (m_iPlayRoute >= 0 && m_iPlayRoute < int(m_vRoutes.size())) ? m_vRoutes[m_iPlayRoute].m_sName : "route";
		if (sTitle.empty())
			sTitle = "route";
		sValue = std::format("{} / {}", int(m_iPlaybackIdx * TICK_INTERVAL), int(m_vRecording.size() * TICK_INTERVAL));
		tCol = Color_t(120, 255, 140, 255);
	}
	// This route was recorded mid-motion, so creeping to its start can't reproduce its velocity - the
	// trick won't replay reliably. Amber-tint + prefix so you know to re-record it from a dead standstill.
	if (m_bRouteUnreliable && !m_bRecording)
	{
		sTitle = "! " + sTitle;
		tCol = Color_t(255, 150, 60, 255);
	}
	const bool bHasValue = !sValue.empty();

	const auto& tFont = H::Fonts.GetFont(FONT_INDICATORS);
	const int iLineH = std::max(int(H::Draw.GetTextSize("0", tFont).y), int(H::Draw.Scale(11)));
	const int iPad = int(H::Draw.Scale(8));
	const int iGap = int(H::Draw.Scale(6));
	const int iW = int(H::Draw.Scale(160));

	// Box height: pad + title (+ separator + value when present) + pad.
	const int iH = iPad * 2 + iLineH + (bHasValue ? iGap + 1 + iGap + iLineH : 0);

	// DragBox stores center-x / top-y (see CMenu::AddDraggable); centre the fixed-width box on it.
	// Never positioned (0,0) -> default to the upper centre instead of half off the left edge.
	int iCX = Vars::Menu::RecorderDisplay.Value.x;
	int iTop = Vars::Menu::RecorderDisplay.Value.y;
	if (iCX == 0 && iTop == 0)
	{
		iCX = H::Draw.m_nScreenW / 2;
		iTop = int(H::Draw.m_nScreenH * 0.18f);
	}
	const int iLeft = iCX - iW / 2;

	H::Draw.StyledPanel(iLeft, iTop, iW, iH); // shared panel bg+outline (matches watermark/menu/spec list)

	int iY = iTop + iPad;
	H::Draw.String(tFont, iCX, iY, tCol, ALIGN_TOP, sTitle.c_str());
	iY += iLineH;

	if (bHasValue)
	{
		// Faded separator (minimal spectator-list style): transparent edges -> accent centre.
		iY += iGap;
		const int iLineW = iW - iPad * 2;
		const int iHalfW = iLineW / 2;
		const Color_t tEdge = { 0, 0, 0, 0 };
		const Color_t tAccent = Vars::Menu::Theme::Accent.Value;
		H::Draw.GradientRect(iLeft + iPad, iY, iHalfW, 1, tEdge, tAccent, true);
		H::Draw.GradientRect(iLeft + iPad + iHalfW, iY, iLineW - iHalfW, 1, tAccent, tEdge, true);
		iY += 1 + iGap;
		H::Draw.String(tFont, iCX, iY, tCol, ALIGN_TOP, sValue.c_str());
	}
}

// Short, upper-case key label for the on-screen key boxes (matches the compact codes in the
// reference window, e.g. capslock -> "CAP", letters/digits as-is, mouse buttons -> M1..M5).
std::string CMisc::CheckpointKeyName(int iKey)
{
	if (iKey == 0)
		return "-";
	if (iKey >= 0x30 && iKey <= 0x39) return std::string(1, char('0' + (iKey - 0x30))); // 0-9
	if (iKey >= 0x41 && iKey <= 0x5A) return std::string(1, char(iKey));                 // A-Z
	if (iKey >= VK_F1 && iKey <= VK_F12) return std::format("F{}", iKey - VK_F1 + 1);
	if (iKey >= VK_NUMPAD0 && iKey <= VK_NUMPAD9) return std::format("N{}", iKey - VK_NUMPAD0);
	switch (iKey)
	{
	case VK_LBUTTON: return "M1";
	case VK_RBUTTON: return "M2";
	case VK_MBUTTON: return "M3";
	case VK_XBUTTON1: return "M4";
	case VK_XBUTTON2: return "M5";
	case VK_SPACE: return "SPC";
	case VK_SHIFT: case VK_LSHIFT: return "SFT";
	case VK_RSHIFT: return "RSF";
	case VK_CONTROL: case VK_LCONTROL: return "CTL";
	case VK_RCONTROL: return "RCT";
	case VK_MENU: case VK_LMENU: return "ALT";
	case VK_RMENU: return "RAL";
	case VK_CAPITAL: return "CAP";
	case VK_TAB: return "TAB";
	case VK_RETURN: return "ENT";
	case VK_BACK: return "BSP";
	case VK_ESCAPE: return "ESC";
	case VK_INSERT: return "INS";
	case VK_DELETE: return "DEL";
	case VK_HOME: return "HOM";
	case VK_END: return "END";
	case VK_PRIOR: return "PGU";
	case VK_NEXT: return "PGD";
	case VK_OEM_3: return "~";
	}
	return "?";
}

// On-screen "Checkpoints" window - styled EXACTLY practice window (the reference
void CMisc::DrawCheckpoints()
{
	if (!Vars::Misc::Movement::Checkpoints.Value || !I::EngineClient->IsInGame())
		return;
	auto pLocal = H::Entities.GetLocal();
	if (!pLocal || !pLocal->IsAlive())
		return;

	// the reference constants (scaled to the menu scale).
	const int iW          = int(H::Draw.Scale(170));
	const int iTitleTop   = int(H::Draw.Scale(5));
	const int iFirstRow   = int(H::Draw.Scale(25));
	const int iLeftMargin = int(H::Draw.Scale(10));
	const int iBox        = int(H::Draw.Scale(20));
	const int iBoxRound   = int(H::Draw.Scale(3));
	const int iRowH       = int(H::Draw.Scale(24));
	const int iRowGap     = int(H::Draw.Scale(3));
	const int iBottom     = int(H::Draw.Scale(5));

	const Color_t tBoxLine  = Color_t(22, 22, 22, 255);
	const Color_t tLabel    = Color_t(100, 100, 100, 255);
	const Color_t tKeyText  = Color_t(100, 100, 100, 255);
	const Color_t tTitle    = Color_t(255, 255, 255, 255);

	const auto& tFont = H::Fonts.GetFont(FONT_SPECTATOR);

	struct Row_t { const char* sLabel; int iKey; };
	const Row_t aRows[3] = {
		{ "Save",     Vars::Misc::Movement::CheckpointSaveKey.Value },
		{ "Teleport", Vars::Misc::Movement::CheckpointTeleportKey.Value },
		{ "Noclip",   Vars::Misc::Movement::CheckpointNoclipKey.Value },
	};
	constexpr int iRows = 3;

	const int iH = iFirstRow + iRows * iRowH + (iRows - 1) * iRowGap + iBottom;

	// DragBox stores centre-x / top-y (see CMenu::AddDraggable); centre the fixed-width panel on it.
	int iCX = Vars::Menu::CheckpointsDisplay.Value.x;
	int iTop = Vars::Menu::CheckpointsDisplay.Value.y;
	if (iCX == 0 && iTop == 0)
	{
		iCX = int(H::Draw.m_nScreenW * 0.12f);
		iTop = int(H::Draw.m_nScreenH * 0.32f);
	}
	const int iLeft = iCX - iW / 2;

	H::Draw.StyledPanel(iLeft, iTop, iW, iH); // shared panel bg+outline (matches watermark/menu/spec list)

	// Centred title.
	H::Draw.String(tFont, iLeft + iW / 2, iTop + iTitleTop, tTitle, ALIGN_TOP, "Checkpoints");

	// Rows: grey label on the left, bordered key box (with the bound key) on the right.
	for (int i = 0; i < iRows; i++)
	{
		const int iRowTop = iTop + iFirstRow + i * (iRowH + iRowGap);
		const int iRowMidY = iRowTop + iRowH / 2;

		H::Draw.String(tFont, iLeft + iLeftMargin, iRowMidY, tLabel, ALIGN_LEFT, aRows[i].sLabel);

		const int iBoxX = iLeft + iW - iLeftMargin - iBox;
		const int iBoxY = iRowMidY - iBox / 2;
		H::Draw.LineRoundRect(iBoxX, iBoxY, iBox, iBox, iBoxRound, tBoxLine);

		const std::string sKey = CheckpointKeyName(aRows[i].iKey);
		const int iKeyTextH = int(H::Draw.GetTextSize(sKey.c_str(), tFont).y);
		H::Draw.String(tFont, iBoxX + iBox / 2, iBoxY + (iBox - iKeyTextH) / 2, tKeyText, ALIGN_TOP, sKey.c_str());
	}
}

// Start circles + path line on the world. Draws every saved route's start
// when idle; the active route's path while recording/playing.
void CMisc::DrawRoutes(CTFPlayer* pLocal)
{
	if (!Vars::Misc::Movement::MovementRecorder.Value || !Vars::Misc::Movement::MovementRecorderShowRoutes.Value)
		return;
	if (!pLocal || !pLocal->IsAlive() || !I::EngineClient->IsInGame())
		return;

	const Color_t tAccent = Vars::Menu::Theme::Accent.Value;
	const Vec3 vOrigin = pLocal->m_vecOrigin();

	const bool bSolid = Vars::Misc::Movement::MovementRecorderRouteSolid.Value;
	const float flRingU = Vars::Misc::Movement::MovementRecorderRouteRadius.Value;
	const int iPoints = std::max(3, Vars::Misc::Movement::MovementRecorderRoutePoints.Value);

	// A flat ring on the floor around a route start, route name centred in the middle. Solid =
	auto DrawGroundRing = [&](const Vec3& vCenter, const Color_t& tBase, int iAlpha, const std::string& sName)
	{
		const int iSeg = bSolid ? std::max(iPoints, 24) : iPoints;
		std::vector<Vec3> vScreens;
		vScreens.reserve(iSeg);
		bool bAny = false;
		for (int i = 0; i < iSeg; i++)
		{
			const float flA = 2.f * PI * float(i) / float(iSeg);
			Vec3 vScr;
			if (SDK::W2S(vCenter + Vec3(cosf(flA) * flRingU, sinf(flA) * flRingU, 0.f), vScr))
			{
				vScreens.push_back(vScr);
				bAny = true;
			}
			else
				vScreens.emplace_back(FLT_MAX, FLT_MAX, 0.f); // off-screen / behind camera
		}
		if (!bAny)
			return;

		const Color_t tCol = Color_t(tBase.r, tBase.g, tBase.b, int(tBase.a * iAlpha / 255));
		if (bSolid)
		{
			for (int i = 0; i < iSeg; i++)
			{
				const Vec3& a = vScreens[i];
				const Vec3& b = vScreens[(i + 1) % iSeg];
				if (a.x == FLT_MAX || b.x == FLT_MAX)
					continue;
				H::Draw.Line(int(a.x), int(a.y), int(b.x), int(b.y), tCol);
			}
		}
		else
		{
			const float flDot = H::Draw.Scale(2.5f, Scale_Round);
			for (const Vec3& s : vScreens)
			{
				if (s.x == FLT_MAX)
					continue;
				H::Draw.FillCircle(int(s.x), int(s.y), flDot, 16, tCol);
			}
		}

		// Route name in the middle, small font.
		Vec3 vMid;
		if (!sName.empty() && SDK::W2S(vCenter, vMid))
		{
			const auto& tFont = H::Fonts.GetFont(FONT_SPECTATOR);
			H::Draw.String(tFont, int(vMid.x), int(vMid.y), Color_t(255, 255, 255, iAlpha), ALIGN_CENTER, sName.c_str());
		}
	};

	// Active route path while replaying: connect the frames with an accent line + start ring.
	// Only while playing/moving-to-start - the buffer is immutable then; during recording it grows
	if ((m_bPlaying || m_bMovingToStart) && m_vRecording.size() > 1)
	{
		if (Vars::Misc::Movement::MovementRecorderShowPath.Value)
		{
			Vec3 vPrev;
			bool bHavePrev = false;
			// Sample every few frames so long routes stay cheap (~66 segments/sec is plenty).
			const int iStep = std::max(1, int(m_vRecording.size()) / 600);
			for (int i = 0; i < int(m_vRecording.size()); i += iStep)
			{
				Vec3 vScreen;
				if (SDK::W2S(m_vRecording[i].m_vOrigin, vScreen))
				{
					if (bHavePrev)
						H::Draw.Line(int(vPrev.x), int(vPrev.y), int(vScreen.x), int(vScreen.y), tAccent);
					vPrev = vScreen;
					bHavePrev = true;
				}
				else
					bHavePrev = false;
			}
		}

		const std::string sActive = (m_iPlayRoute >= 0 && m_iPlayRoute < int(m_vRoutes.size())) ? m_vRoutes[m_iPlayRoute].m_sName : std::string();
		DrawGroundRing(m_vRecording.front().m_vOrigin, tAccent, 255, sActive);
		return;
	}

	// Idle: a ground ring at each saved route's start, name in the middle, faded by distance.
	const Color_t tRouteCol = Vars::Misc::Movement::MovementRecorderRouteColor.Value;
	for (const auto& tRoute : m_vRoutes)
	{
		if (tRoute.m_vFrames.empty())
			continue;
		const Vec3 vStart = tRoute.m_vFrames.front().m_vOrigin;
		const float flDist = vOrigin.DistTo(vStart);
		int iAlpha = 255;
		if (flDist > 900.f)
			iAlpha = int(std::clamp((1400.f - flDist) / 500.f, 0.f, 1.f) * 255.f);
		if (iAlpha <= 0)
			continue;
		DrawGroundRing(vStart, tRouteCol, iAlpha, tRoute.m_sName);
	}
}

// ----------------------------- Jump stats () -----------------------------
// Tracked from RunPre with the REAL (pre-prediction) ground flags + velocity, so takeoff/landing
void CMisc::JumpStats(CTFPlayer* pLocal, CUserCmd* pCmd)
{
	if (!Vars::Misc::Movement::JumpStats.Value || !pLocal || !pLocal->IsAlive())
		return;

	const bool bOnGround = (m_iPrePredictionFlags & FL_ONGROUND) != 0;
	const Vec3 vVel = pLocal->m_vecVelocity();
	const float flSpeed = vVel.Length2D();
	const Vec3 vOrigin = pLocal->m_vecOrigin();
	const bool bDuckHeld = pCmd && (pCmd->buttons & IN_DUCK) != 0;

	// Ground-streak + bhop-chain bookkeeping.
	if (bOnGround)
		m_iJumpTicksOnGround++;
	else
		m_iJumpTicksOnGround = 0;
	if (m_iJumpTicksOnGround > 2) // stood on the ground too long -> the bhop chain is broken
		m_iJumpBhopChain = 0;

	// Snapshot the launch state on every grounded tick.
	if (bOnGround)
	{
		m_vJumpLastGroundOrigin = vOrigin;
		m_flJumpLastGroundSpeed = flSpeed;
		m_iJumpLastGroundStreak = m_iJumpTicksOnGround;
	}

	if (m_bJumpStatsOnGround && !bOnGround) // takeoff (first airborne tick)
	{
		m_vJumpTakeoffPos = m_vJumpLastGroundOrigin;    // real launch edge (accuracy fix)
		m_flJumpTakeoffSpeed = m_flJumpLastGroundSpeed; // speed at launch (= "pre")
		m_flJumpPeakSpeed = flSpeed;
		m_flJumpPeakZ = m_vJumpTakeoffPos.z;
		m_bJumpDuckedTakeoff = pLocal->IsDucking();
		m_bBreakJumpThisAir = false; // re-arm; BreakJump sets it again if it ducks during this airtime
		m_iJumpTakeoffTick = I::GlobalVars->tickcount; // for airtime in the debug log

		// A jump off the very first grounded tick (streak == 1) continues a bhop chain.
		if (m_iJumpLastGroundStreak == 1)
			m_iJumpBhopChain++;

		// Base label from the bhop chain; MJ/LJ/JB refine it below across the airtime.
		if (m_iJumpBhopChain >= 2)      m_sLastJumpType = "MBH";
		else if (m_iJumpBhopChain == 1) m_sLastJumpType = "BH";
		else                            m_sLastJumpType = "Jump";

		// Re-arm the duck (MJ/LJ) tracker for this airtime.
		m_bJumpRecordDuck = bDuckHeld;
		m_iJumpDuckTicks = 0;

		m_iJumpStrafes = 0; // start counting this airtime's strafes fresh
	}
	else if (!bOnGround) // airborne - track peaks
	{
		if (flSpeed > m_flJumpPeakSpeed) m_flJumpPeakSpeed = flSpeed;
		if (vOrigin.z > m_flJumpPeakZ) m_flJumpPeakZ = vOrigin.z;
	}

	// the reference MJ/LJ: count ticks the duck is held continuously from takeoff (1 = MJ, 2 = LJ). Only
	// applies to non-chained jumps; JB (jumpbug) always wins. The label persists once set.
	if (m_bJumpRecordDuck && bDuckHeld)
		m_iJumpDuckTicks++;
	else
	{
		m_iJumpDuckTicks = 0;
		m_bJumpRecordDuck = false;
	}
	if (m_iJumpBhopChain == 0 && m_bJumpRecordDuck)
	{
		if (m_iJumpDuckTicks == 1)      m_sLastJumpType = "MJ";
		else if (m_iJumpDuckTicks == 2) m_sLastJumpType = "LJ";
	}
	if (m_bJumpBugThisAir)
		m_sLastJumpType = "JB";

	// the reference strafe count: each time the mouse turn direction (mousedx sign) flips L<->R we count
	const int iMouseDx = pCmd ? pCmd->mousedx : 0;
	if (iMouseDx < 0 && !m_bJumpStrafeRight && !m_bJumpStrafeLeft)
		m_bJumpStrafeRight = true;
	else if (iMouseDx > 0 && !m_bJumpStrafeRight && !m_bJumpStrafeLeft)
		m_bJumpStrafeLeft = true;
	else if (iMouseDx < 0 && !m_bJumpStrafeRight && m_bJumpStrafeLeft)
	{
		m_bJumpStrafeRight = true;
		m_bJumpStrafeLeft = false;
		if (!bOnGround) m_iJumpStrafes++;
	}
	else if (iMouseDx > 0 && m_bJumpStrafeRight && !m_bJumpStrafeLeft)
	{
		m_bJumpStrafeRight = false;
		m_bJumpStrafeLeft = true;
		if (!bOnGround) m_iJumpStrafes++;
	}

	if (!m_bJumpStatsOnGround && bOnGround) // landing - finalize
	{
		// Distance is edge-to-edge: horizontal launch->land displacement + the 32u standing hull width
		// (matches TF2 jump servers/plugins and the reference).
		m_flLastJumpDistance = Vec3(vOrigin.x - m_vJumpTakeoffPos.x, vOrigin.y - m_vJumpTakeoffPos.y, 0.f).Length2D() + 32.f;
		m_flLastJumpTakeoffSpeed = m_flJumpTakeoffSpeed;
		m_flLastJumpMaxSpeed = m_flJumpPeakSpeed;
		m_flLastJumpHeight = m_flJumpPeakZ - m_vJumpTakeoffPos.z;
		m_iLastJumpStrafes = m_iJumpStrafes;

		const float flLandDeltaZ = vOrigin.z - m_vJumpTakeoffPos.z; // ground-height change over the jump

		const int iNow = I::GlobalVars->tickcount;
		if (m_iJumpLandTick != 0 && iNow - m_iJumpLandTick <= 12) // quick re-land = continued bhop chain
			m_iJumpChain++;
		else
			m_iJumpChain = 1;
		m_iJumpLandTick = iNow;

		// Flat-ground gate.
		constexpr float flFlatZTol = 4.f;   // |takeoff z - land z| within this == flat ground
		constexpr float flMinDist  = 200.f; // ignore tiny hops
		constexpr float flMaxDist  = 400.f; // sanity bound
		const bool bFlat = fabsf(flLandDeltaZ) <= flFlatZTol;

		// Print to chat, not on the screen.
		if (bFlat && m_flLastJumpDistance >= flMinDist && m_flLastJumpDistance <= flMaxDist
			&& I::ClientModeShared && I::ClientModeShared->m_pChatElement)
		{
			// Accent-coloured cheat tag + white stat line (ToHex() emits the \x07RRGGBB chat code).
			I::ClientModeShared->m_pChatElement->ChatPrintf(0, std::format(
				"{}{} \x07" "FFFFFF" "| {} \x07" "FFFFFF" "- dist {:.0f}u, speed {:.0f} (max {:.0f}), height {:.0f}u, {} strafe",
				Vars::Menu::Theme::Accent.Value.ToHex(), Vars::Menu::CheatTitle.Value,
				m_sLastJumpType, m_flLastJumpDistance, m_flLastJumpTakeoffSpeed,
				m_flLastJumpMaxSpeed, m_flLastJumpHeight, m_iLastJumpStrafes).c_str());
		}

		m_bJumpBugThisAir = false; // consume the jumpbug flag so it doesn't bleed into the next airtime
	}

	m_bJumpStatsOnGround = bOnGround;
}

void CMisc::Draw(CTFPlayer* pLocal)
{
	DrawPixelSurf(pLocal);
	DrawRecorder();
	// DrawCheckpoints() moved to IEngineVGui_Paint AFTER CALL_ORIGINAL so the window sits on top of the HUD
	DrawRoutes(pLocal);

	if (!Vars::Misc::Movement::EdgeBug.Value || !Vars::Misc::Movement::EdgeBugVisualize.Value)
	{
		m_flEdgeBugDrawAlpha = 0.f;
		return;
	}

	if (!pLocal || !pLocal->IsAlive() || !I::EngineClient->IsInGame())
		return;

	// Smoothly fade the predicted path in/out
	const float flTarget = (m_bEdgeBugDetected && m_bEdgeBugHasPath) ? 1.f : 0.f;
	m_flEdgeBugDrawAlpha += (flTarget - m_flEdgeBugDrawAlpha) * std::clamp(I::GlobalVars->frametime * 8.f, 0.f, 1.f);

	if (m_flEdgeBugDrawAlpha <= 0.01f || m_vEdgeBugPath.size() < 2)
		return;

	Color_t tColor = Vars::Colors::EdgeBugCircle.Value;
	tColor.a = static_cast<byte>(std::clamp(m_flEdgeBugDrawAlpha, 0.f, 1.f) * tColor.a);

	// Predicted path
	Vec3 vCur, vNext;
	for (size_t i = 0; i + 1 < m_vEdgeBugPath.size(); i++)
	{
		if (SDK::W2S(m_vEdgeBugPath[i], vCur) && SDK::W2S(m_vEdgeBugPath[i + 1], vNext))
			H::Draw.Line(int(vCur.x), int(vCur.y), int(vNext.x), int(vNext.y), tColor);
	}

	// Landing marker square
	const Vec3& vLand = m_vEdgeBugLandPos;
	Vec3 vC[4];
	if (SDK::W2S({ vLand.x - 16.f, vLand.y - 16.f, vLand.z }, vC[0])
		&& SDK::W2S({ vLand.x - 16.f, vLand.y + 16.f, vLand.z }, vC[1])
		&& SDK::W2S({ vLand.x + 16.f, vLand.y + 16.f, vLand.z }, vC[2])
		&& SDK::W2S({ vLand.x + 16.f, vLand.y - 16.f, vLand.z }, vC[3]))
	{
		for (int i = 0; i < 4; i++)
		{
			const Vec3& a = vC[i];
			const Vec3& b = vC[(i + 1) % 4];
			H::Draw.Line(int(a.x), int(a.y), int(b.x), int(b.y), tColor);
		}
	}
}
