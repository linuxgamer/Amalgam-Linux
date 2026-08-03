#pragma once
#include "../../SDK/SDK.h"
#include <deque>

class CMisc
{
private:
	void AutoJump(CTFPlayer* pLocal, CUserCmd* pCmd);
	void AutoJumpbug(CTFPlayer* pLocal, CUserCmd* pCmd);
	void AutoStrafe(CTFPlayer* pLocal, CUserCmd* pCmd);
	void StrafeOptimizer(CTFPlayer* pLocal, CUserCmd* pCmd);
	void MovementLock(CTFPlayer* pLocal, CUserCmd* pCmd);
	void BreakJump(CTFPlayer* pLocal, CUserCmd* pCmd);
	void FlipWorldInput(CUserCmd* pCmd); // mirror controls to match the flipped world view
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
	bool EdgeBugCheck(CTFPlayer* pLocal, CUserCmd* pCmd, const Vec3& vUnpredictedVel, const Vec3& vUnpredictedOrigin, bool& bBreak, int& iSimsUsed, int iSimCap);
	void RestoreEntityToPredicted();
	void CorrectMovement(CUserCmd* pCmd, Vec3 vWishAngle, Vec3 vOldAngles);
	void AutoStrafeEdgeBug(CUserCmd* pCmd, CTFPlayer* pLocal);

	void LongJump(CTFPlayer* pLocal, CUserCmd* pCmd);
	void MiniJump(CTFPlayer* pLocal, CUserCmd* pCmd);
	void MiniJumpPre(CTFPlayer* pLocal, CUserCmd* pCmd); // re-presses a parked jump before prediction
	void AutoAlign(CTFPlayer* pLocal, CUserCmd* pCmd);
	void PixelSurf(CTFPlayer* pLocal, CUserCmd* pCmd);
	void TextureBug(CTFPlayer* pLocal, CUserCmd* pCmd);
	void HeadSurf(CTFPlayer* pLocal, CUserCmd* pCmd);
	void WallClimb(CTFPlayer* pLocal, CUserCmd* pCmd);
	// Air stuck. Persistent state held here rather than
	// function-static so the core stays re-entrant. bSlantedOnly switches to the wall_stuck variant
	struct AirStuckState_t
	{
		float start_circle    = 0.f;
		int   stuck_hold_ticks = 0;
		float stuck_last_yaw  = 3.402823466e+38f; // FLT_MAX sentinel = "no committed move yet"
		float stuck_last_move = 3.402823466e+38f;
		float stuck_last_side = 0.f;
		float cached_wall_angle = -1.f;           // last tick's winning TB_DetectWall angle (-1 = none) -> 1-trace steady state
	};
	void AirStuckCore(CTFPlayer* pLocal, CUserCmd* pCmd, bool bEnabled, bool bSlantedOnly, bool& bHitFlag, AirStuckState_t& tState);
	void AirStuck(CTFPlayer* pLocal, CUserCmd* pCmd);
	void PixelFinder(CTFPlayer* pLocal, CUserCmd* pCmd);
	void PixelSurfLine(CTFPlayer* pLocal, CUserCmd* pCmd); // the reference: record the surf strip on entry
	void PixelSurfAssist(CTFPlayer* pLocal, CUserCmd* pCmd);
	void LoadAssistPoints(const std::string& sMap);  // load this map's saved targets from Points\<map>.txt
	bool WriteAssistPoints(const std::string& sMap); // (re)write this map's saved targets to Points\<map>.txt
	float SimJumpReach(CTFPlayer* pLocal, CUserCmd* pCmd, int iJumpType, float flAtFlatDist = -1.f); // feet-Z gain of one jump type: apex when flAtFlatDist<=0, else the height the arc still has once it has travelled that far horizontally

	// Result of simulating one jump type against a pixel-surf point: catch/graze, apex gain, and gap.
	struct JumpCatchResult_t
	{
		bool m_bCatch = false;       // sustained pin at the point's height
		bool m_bGraze = false;       // single pinned tick at the point's height
		bool m_bGridCatch = false;   // analytic tick grid: a future descending tick of this launch
		                             // lands in the point's sub-unit catch window (exact, strafe-proof)
		float m_flApexGain = 0.f;    // feet-Z gain at the arc's apex, measured from the LAUNCH position
		float m_flAtDistGain = 0.f;  // feet-Z gain the arc still has at the target's flat distance
		float m_flGap = 1e9f;        // closest the ducked descent came to the point's Z
		float m_flGapVz = 0.f;       // vertical speed at that closest pass (small = slow, catchable band)
		float m_flGapFlat = 1e9f;    // horizontal distance still left to the point at that pass
		float m_flLaunchZ = 0.f;     // feet Z the sim launched from (the predicted ground on slopes/chains)
		int m_iLaunchDelay = -1;     // sim ticks flown before launch (0 = grounded now, -1 = never found ground)
	};
	JumpCatchResult_t SimJumpCatch(CTFPlayer* pLocal, CUserCmd* pCmd, int iJumpType, const Vec3& vTarget, float flAtFlatDist);
	void DrawPixelSurf(CTFPlayer* pLocal);

	// movement recorder (records/replays per-map named routes).
	void MovementRecorder(CTFPlayer* pLocal, CUserCmd* pCmd);
	void LoadRoutes(const std::string& sMap);      // load this map's routes file into m_vRoutes
	bool WriteRoutes(const std::string& sMap);      // (re)write this map's routes file from m_vRoutes
	void DrawRecorder();                            // REC/PLAY text HUD
	void DrawRoutes(CTFPlayer* pLocal);             // start circles + path line on the world

	// Checkpoints (KZ practice; kz_practice): save origin+view, teleport back.
	void Checkpoints(CTFPlayer* pLocal, CUserCmd* pCmd);

	// jump stats (takeoff/landing tracking; prints to chat on landing).
	void JumpStats(CTFPlayer* pLocal, CUserCmd* pCmd);

	// edge bug sound (plays the configured sound when an edge bug is hit).
	void PlayEdgeBugSound();

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
	EdgeBugCmd_t m_EdgeBugCmds[128] = {};
	int m_iEdgeBugSuccessCounter = 0; // bumps the instant an edgebug actually completes (not when detected)

	// Advanced edgebug
	Vec3 m_vEdgeBugTargetAngles = {};
	Vec3 m_vEdgeBugSmoothAngles = {};
	Vec3 m_vEdgeBugOriginalAngles = {};
	float m_flEdgeBugOriginalForward = 0.f;
	float m_flEdgeBugOriginalSide = 0.f;
	int m_iEdgeBugPredictionTimestamp = 0;
	int m_iEdgeBugMouseOffset = 0;

	// Visualization (predicted edgebug path + landing point of the winning search attempt)
	std::vector<Vec3> m_vEdgeBugPath = {};
	std::vector<Vec3> m_vEdgeBugScratchPath = {}; // reused per-attempt during the search (avoids per-CreateMove heap churn)

	// One search attempt's input recipe. Held in a reused member buffer so the per-CreateMove
	// fan-out build (crouch/stand still + strafe scans + auto-strafe) does not heap-allocate.
	struct EdgeBugAttempt_t { bool bDuck; int iMode; float flYawStep; };
	std::vector<EdgeBugAttempt_t> m_vEdgeBugAttempts = {};
	Vec3 m_vEdgeBugLandPos = {};
	bool m_bEdgeBugHasPath = false;
	float m_flEdgeBugDrawAlpha = 0.f;

	// LongJump data
	int m_iLongJumpSavedTick = 0;
	bool m_bLongJumpDetected = false;

	// MiniJump data
	bool m_bMiniJumpShouldDuck = false;
	bool m_bMiniJumpDetected = false;
	int m_iPrePredictionFlags = 0; // Flags before prediction for mini jump detection
	Vec3 m_vPrePredictionVelocity = {}; // velocity before prediction (pixel-surf ride detection)
	// Flip world: tracks whether we've negated the m_yaw cvar (mouse-look mirror) so we flip it back
	// exactly once when the feature turns off, instead of fighting the engine per tick.
	bool m_bFlipWorldYawNegated = false;
	int m_iMiniJumpDuckTick = 0;
	// jump -> minijump repair: when a grounded jump press gets eaten by the engine (still
	// ducked / mid-unduck from the previous hop, or jump not released long enough), we park
	int m_iMiniJumpPendingUntil = 0;   // tickcount deadline for the parked jump (0 = none)
	int m_iMiniJumpLastJumpCmdTick = 0; // last tick the outgoing cmd carried IN_JUMP
	int m_iMiniJumpLandTick = 0;        // tick we last touched ground (bhop delay reference)
	bool m_bMiniJumpPrevGround = false; // ground state last tick (landing edge detect)

	// AutoAlign data
	bool m_bWallDetected = false;
	float m_flAutoAlignStartCircle = 0.f;

	// PixelSurf data
	bool m_bShouldPixelSurf = false;
	int m_iPixelSurfTicks = 0;
	bool m_bPixelSurfingNow = false; // actually riding a surf this tick (drives the indicator)
	bool m_bPrevSurfVel = false;     // surf velocity last tick (2-tick sustain filter)

	// TextureBug / choke data.
	float m_flTBCachedForward = -1.f;      // last winning forward magnitude (<0 == no cache)
	float m_flTBCachedAngleOffset = 0.f;   // last winning yaw offset from the wall (incl. sign)
	float m_flTBCachedWallAngle = -1.f;    // last winning wall sweep angle (<0 == no cache)
	bool m_bTBCachedDuck = false;          // last winning config needed an auto-crouch to catch the bug
	int m_iTBChokeTicksLeft = 0;           // packets to choke after catching the bug
struct WallCache_t { bool valid = false; CGameTrace trace; float foundAngle = 0.f; };
WallCache_t m_TB_WallCache;
void TB_RefreshWallCache(CTFPlayer* pLocal);

	// TB-specific lock
	bool m_bTBLocked = false;
	bool m_bTBArrestLock = false;   // the lock is a fall-arrest hold (absorbed AirStuck), not a half-grav
	                                // catch: held verbatim every tick, bypasses the band / z-pin ride checks
	float m_flTBLockedForward = 0.f;
	float m_flTBLockedSide = 0.f;
	bool m_bTBLockedDuck = false;   // the locked catch needed IN_DUCK - re-assert it every replay tick
	                                // (flip-flopping the duck mid-ride is what used to drop the bug)
	int m_iTBSteerAwayTicks = 0;    // consecutive hard outward-steer ticks while locked; ride drops at 2
	int m_iTBPendingCatchTicks = 0; // replay ticks left while a committed lookahead catch falls into place
	                                // (the candidate sims showed the catch landing a few ticks ahead, so the
	                                // live vz isn't banded yet - keep holding the move instead of unlocking)
	float m_flTBPrevVz = 0.f;       // live vz from the previous TB tick (passive-catch "fall snapped" signature)
	float m_flTBLockedZ = 0.f;      // z where the locked ride engaged - z still pinned == still on the bug, 0 sims
	bool m_bTBMissValid = false;    // miss memo below is live
	bool m_bTBMissSettled = false;  // BOTH sweep halves missed at the memo spot - skip the sweeps until z moves
	float m_flTBMissZ = 0.f;        // z of the last full-sweep miss
	float m_flTBMissWallYaw = 0.f;  // wall yaw of that miss
	int m_iTBSweepPhase = 0;        // alternates the two expensive sweep halves across consecutive miss ticks

	// --- Paper-grounded TB improvement state (/Review.pdf §9.4.2; each behind its own toggle) ---
	// #1 brush-face measurement, cached per wall+z (the catch is a SPATIAL event - the head crossing
	bool  m_bTBBrushValid = false;   // the measurement below is live for the current wall+spot
	float m_flTBBrushBottomZ = 0.f;  // world z of the brush bottom edge the head falls past (the window)
	float m_flTBBrushHeight = 0.f;   // measured height of the brush face at the head (Eq.10 H_brush)
	float m_flTBBrushMeasZ = 0.f;    // player z when measured (re-measure once z drifts)
	float m_flTBBrushMeasYaw = 0.f;  // wall yaw when measured (re-measure when the wall changes)
	// #3 sequenced flip: the tangent sign hugged during the fall, flipped once at the brush bottom.
	int   m_iTBHugSign = 0;          // -1 / +1 once chosen, 0 = not chosen yet

	// WallClimb data (same config cache idea as TextureBug, but targets FL_ONGROUND)
	float m_flWallClimbCachedForward = -1.f;
	float m_flWallClimbCachedAngleOffset = 0.f;
	float m_flWallClimbCachedWallAngle = -1.f;
	bool m_bWallClimbCachedDuck = false;   // last winning config grounded us ducked
	int m_iWallClimbColdSkip = 0;          // ticks to skip the expensive bruteforce after a full miss (unclimbable-wall guard)

	// HeadSurf data — "pixelsurf with your head against a brush bottom".
	// Targets vz == +halfgravity (+6.25), the mirror of a normal pixelsurf.
	float m_flHSCachedForward     = -1.f;  // last winning forward magnitude (<0 == no cache)
	float m_flHSCachedAngleOffset = 0.f;   // last winning yaw offset from ceiling alignment (incl. sign)
	bool  m_bHSCachedDuck         = false; // last winning config had IN_DUCK set
	bool  m_bHSSurfingNow         = false; // actually riding a head surf this tick
	bool  m_bHSPrevSurfVel        = false; // +halfgravity vz last tick (2-tick sustain filter)
	bool  m_bHSLocked             = false; // riding a head surf - replay the locked move verbatim (0 sims)
	float m_flHSLockedForward     = 0.f;
	float m_flHSLockedSide        = 0.f;
	bool  m_bHSLockedDuck         = false;
	int   m_iHSColdSkip           = 0;     // ticks to skip the bruteforce after a full miss (dead-ceiling guard)

	// Indicator detection states (transient per-tick highlight flags)
	bool m_bTextureBugHit = false;
	bool m_bTBOwnsCmd = false;   // TB steered pCmd this tick without a hit (creep/drive/probe) - AirStuck yields
	bool m_bHeadSurfHit = false;
	bool m_bWallClimbHit = false;
	bool m_bAirStuckHit = false;
	AirStuckState_t m_AirStuckState{};
	bool m_bJumpBugHit = false;
	bool m_bRawJumpHeld = false; // player's real IN_JUMP at the top of RunPre, captured BEFORE AutoJump
	                             // (Bunnyhop) strips it in the air - the gate AutoJumpbug must trust
	bool m_bAutoStrafeActive = false; // drives the auto-strafe keybind indicator (id 11)
	bool m_bAutoStrafeWeaveLeft = false; // latched S-curve weave side (steer-to-look auto strafe)

	// A world point tagged with the map it belongs to (so points don't bleed across maps).
	// m_flSize is a render-only animated radius (grows when the crosshair is over the point).
	struct PixelSurfPoint_t
	{
		Vec3 m_vPos = {};
		std::string m_sMap;
		float m_flSize = 0.f;
		bool m_bActive = true;
		float m_flRadius = 0.f;
		int m_iTypeMask = 127;
	};

	// Pixel finder data: the aimed wall column to scan + the catch points found on release.
	bool m_bPixelFinderHeld = false;        // finder key state last tick (edge detect)
	bool m_bPixelFinderFirst = false;       // a start point has been anchored this hold
	Vec3 m_vPixelFinderStart = {};          // wall hit when the key was first pressed
	Vec3 m_vPixelFinderEnd = {};            // wall hit at the current aim (defines the z range)
	Vec3 m_vPixelFinderWallNormal = {};     // normal of the aimed wall
	std::vector<PixelSurfPoint_t> m_vPixelSurfPoints = {}; // on-wall surf spots found on release
	bool m_bPixelDeleteHeldDraw = false;    // delete-key edge state for the draw-thread hover delete

	// Pixel surf line ("show pixel surf line"): trace your ACTUAL surf path.
	struct SurfLineSegment_t
	{
		std::vector<Vec3> m_vPoints; // wall-surface points, drawn as a connected line
		std::string m_sMap;
	};
	std::vector<SurfLineSegment_t> m_vSurfLineSegments = {};
	std::vector<Vec3> m_vSurfLineCurrent = {}; // path being recorded for the surf in progress
	bool m_bWasInSurfLine = false;          // a path is currently open (recording)
	int m_iSurfLineGrace = 0;               // non-surf ticks tolerated before the path closes (detection blip guard)
	bool m_bSurfLineDeleteHeldDraw = false; // delete-key edge state for crosshair-hover strip delete

	// Pixel surf assist data: saved target points + key edges + transient engaged flag.
	// Targets persist to disk per map (Points\<map>.txt); m_sAssistPointsMap tracks which map is loaded.
	std::vector<PixelSurfPoint_t> m_vAssistPoints = {};
	std::string m_sAssistPointsMap = "";
	bool m_bAssistSetHeld = false;
	bool m_bAssistDeleteHeld = false;
	bool m_bPixelSurfAssistActive = false;
	int m_iAssistJumpType = 0;       // jump type chosen for the in-progress hop (0 = none)
	int m_iAssistJumpStartTick = 0;  // tickcount when we left the ground on the current hop
	// Steer lock: the world-space wish from the commit tick, replayed
	// for the whole hop so the live arc matches the arc the sim validated the pick on.
	Vec3 m_vAssistWishDir = {};      // normalized world wish direction at commit
	float m_flAssistWishMag = 0.f;   // wish magnitude at commit (0 = nothing to replay)
	// Pending launch: while airborne the picker sims the approach to
	// the predicted landing and the jump from THERE; a hit arms this, the wish is replayed through
	int m_iAssistPendingType = 0;    // EJumpType armed for the upcoming ground contact (0 = none)
	int m_iAssistPendingUntil = 0;   // tickcount deadline (predicted landing + slack); stale = dropped

	// Jump-box pop-up: set the tick the assist actually fires a jump for you, naming the jump type
	// and the target's Z height (the reference-style "minijump (duck) to <z>"). Lingers a moment after.
	struct AssistJumpBox_t
	{
		int m_iJumpType = 0;         // EJumpType that was fired
		float m_flTargetZ = 0.f;     // world Z of the surf point we jumped toward
		float m_flCurTime = 0.f;     // I::GlobalVars->curtime when the jump fired (for fade/expiry)
	} m_tAssistJumpBox;

	// movement recorder data. One frame per command; replayed verbatim on playback.
	// m_vOrigin is captured so we can draw the start circle and steer to the route start. m_vVelocity
	struct RecFrame_t { Vec3 m_vViewAngles = {}; float m_flForward = 0.f; float m_flSide = 0.f; float m_flUp = 0.f; int m_iButtons = 0; Vec3 m_vOrigin = {}; Vec3 m_vVelocity = {}; };
	// A named route = a finished recording. Many routes live in one per-map file.
	struct Route_t { std::string m_sName; std::vector<RecFrame_t> m_vFrames; };

	std::vector<RecFrame_t> m_vRecording = {};   // the in-progress / active recording buffer
	std::vector<Route_t> m_vRoutes = {};         // all saved routes for the currently-loaded map
	std::string m_sRoutesMap = "";               // which map m_vRoutes was loaded for
	int m_iSelectedRoute = -1;                   // menu selection (also the route Play prefers)

	bool m_bRecording = false;
	bool m_bPlaying = false;
	size_t m_iPlaybackIdx = 0;
	int m_iPlayRoute = -1;                        // index into m_vRoutes being replayed (-1 = active buffer)

	// Move-to-start state machine: creep to the EXACT start, optionally
	// smooth the view onto the start angle, then begin playback only once the position + velocity
	bool m_bMovingToStart = false;
	bool m_bSmoothToStart = false;
	bool m_bWishToStart = false;
	int m_iSettleTicks = 0;          // ticks spent creeping to the start (safety timeout so it can't hard-lock)
	float m_flAlignDist = 0.f;       // live distance to the start while approaching (HUD feedback)
	float m_flAlignSpeed = 0.f;      // live horizontal speed while approaching (HUD feedback)
	float m_flAlignBest = 0.f;       // closest distance achieved so far (the shimmy hunts this toward 0)
	int m_iStallTicks = 0;           // ticks since the distance last improved - once it stops improving we've
	                                 // reached the physical floor, so commit there instead of waiting out the timeout
	bool m_bRouteUnreliable = false; // set when the active route was recorded mid-motion (start speed too high to
	                                 // reproduce by creeping in) - HUD warns so you re-record it from a standstill

	bool m_bRecKeyHeld = false, m_bPlayKeyHeld = false, m_bStopKeyHeld = false, m_bSaveKeyHeld = false, m_bLoadKeyHeld = false, m_bClearKeyHeld = false;

	// Checkpoints (KZ practice; the reference/the reference model): ONE saved spot - origin + view angles.
	// Save key stores it (on ground), Teleport key setpos+setangs back to it. Keys are edge-detected
	Vec3 m_vCheckpointPos = {};
	Vec3 m_vCheckpointAng = {};
	bool m_bHasCheckpoint = false;

	// Shadowplay: a rolling buffer of the last N seconds of movement, captured while idle. The
	// shadow-save bind dumps it into m_vRecording so you can keep a moment after it happened.
	std::deque<RecFrame_t> m_dqShadow = {};
	bool m_bShadowSaveHeld = false;

	// jump stats data. Real (pre-prediction) ground transitions drive takeoff/landing.
	bool m_bJumpStatsOnGround = true;
	Vec3 m_vJumpTakeoffPos = {};
	float m_flJumpTakeoffSpeed = 0.f;
	float m_flJumpPeakSpeed = 0.f;
	float m_flJumpPeakZ = 0.f;
	bool m_bJumpDuckedTakeoff = false;
	bool m_bBreakJumpThisAir = false; // BreakJump forced a duck during this airtime -> classify as "Break jump"
	int m_iJumpChain = 0;            // consecutive low-airtime hops (bhop chain)
	int m_iJumpLandTick = 0;         // tick we last landed (for chain detection)
	// the reference-style jump-type classification (BH/MBH/LJ/MJ/Jump/JB). Snapshots are taken on the
	// last grounded tick so the takeoff edge is the REAL launch position (the old code read the first
	Vec3 m_vJumpLastGroundOrigin = {};   // origin on the most recent grounded tick (= launch edge)
	float m_flJumpLastGroundSpeed = 0.f; // horizontal speed on the most recent grounded tick (= "pre")
	int m_iJumpTicksOnGround = 0;        // consecutive grounded ticks (reset airborne)
	int m_iJumpLastGroundStreak = 0;     // streak length captured on the last grounded tick
	int m_iJumpBhopChain = 0;            // the reference BhopCount: consecutive 1-tick-ground hops
	bool m_bJumpRecordDuck = false;      // duck was held at takeoff and has stayed held since
	int m_iJumpDuckTicks = 0;            // ticks duck has been continuously held since takeoff (MJ=1, LJ=2)
	bool m_bJumpBugThisAir = false;      // AutoJumpbug fired the unduck+jump for this airtime -> "JB"
	int m_iJumpStrafes = 0;              // strafes (mouse L<->R flips) counted during the current airtime
	int m_iLastJumpStrafes = 0;          // strafe count captured at landing, for the chat line
	bool m_bJumpStrafeRight = false;     // the reference strafe-direction latches (track mousedx sign flips)
	bool m_bJumpStrafeLeft = false;
	float m_flLastJumpDistance = 0.f;
	float m_flLastJumpTakeoffSpeed = 0.f;
	float m_flLastJumpMaxSpeed = 0.f;
	float m_flLastJumpHeight = 0.f;
	std::string m_sLastJumpType = "";
	float m_flJumpStatsDrawTime = 0.f; // curtime until which to keep the panel visible

	int m_iJumpTakeoffTick = 0;          // tick we left the ground

	// Shared jump-type ids for the pixel-surf-assist picker.
	// JT_AIRDASH = Scout's double jump, fired MID-AIR at the height whose dash arc pins the point.
	enum EJumpType { JT_NONE = 0, JT_REGULAR, JT_CROUCH, JT_MINI, JT_MINIDUCK, JT_LONG, JT_LONGDUCK, JT_AIRDASH };

public:
	void RunPre(CTFPlayer* pLocal, CUserCmd* pCmd);
	void RunPost(CTFPlayer* pLocal, CUserCmd* pCmd, bool pSendPacket);

	void Event(IGameEvent* pEvent, uint32_t uNameHash);
	int AntiBackstab(CTFPlayer* pLocal, CUserCmd* pCmd, bool bSendPacket);

	void PingReducer();
	void UnlockAchievements();
	void LockAchievements();

	void Draw(CTFPlayer* pLocal);
	void DrawCheckpoints();                 // legacy surface draw (unused; checkpoints now drawn ImGui in CMenu::DrawCheckpoints)
	std::string CheckpointKeyName(int iKey); // short key label for the checkpoint key boxes (used by the ImGui draw)
	void EdgeBugMouseLock(float& x, float& y);
	void EdgeBugPostPrediction(CTFPlayer* pLocal, CUserCmd* pCmd);

	// Movement indicator state (consumed by CIndicators)
	bool IsEdgeBugDetected() const { return m_bEdgeBugDetected; }
	// Monotonic counter bumped at the moment an edgebug *succeeds* (countdown finished), so callers can
	// fire one-shot effects on the actual completion rather than on the early detection flag.
	int EdgeBugSuccessCounter() const { return m_iEdgeBugSuccessCounter; }
	bool IsLongJumpDetected() const { return m_bLongJumpDetected; }
	bool IsMiniJumpDetected() const { return m_bMiniJumpDetected; }
	bool IsWallDetected() const { return m_bWallDetected; }
	// "ps" indicator: ONLY when actually riding a surf (vz pinned, 2-tick sustain) - the auto
	// catcher's crouch decision (m_bShouldPixelSurf) no longer lights it (per user).
	bool IsPixelSurfing() const { return m_bPixelSurfingNow; }
	bool IsTextureBugHit() const { return m_bTextureBugHit; }
	bool IsHeadSurfHit() const { return m_bHeadSurfHit; }
	bool IsWallClimbHit() const { return m_bWallClimbHit; }
	bool IsAirStuckHit() const { return m_bAirStuckHit; }
	bool IsJumpBugHit() const { return m_bJumpBugHit; }
	bool IsPixelFinderActive() const { return !m_vPixelSurfPoints.empty(); }
	bool IsPixelSurfAssistActive() const { return m_bPixelSurfAssistActive; }
	bool IsAutoStrafeActive() const { return m_bAutoStrafeActive; }

	// Pixel-surf-assist jump-box: builds the pop-up text + fade [0..1] for the ImGui-side draw
	// (CMenu::DrawPixelSurfAssistBox), and expires the box. Returns false when nothing to show.
	bool GetAssistJumpBox(std::string& sTextOut, float& flFadeOut);

	// On-point ENTER menu: aim at a saved assist point + ENTER opens a small
	int m_iAssistPointHover = -1; // assist point under the crosshair this frame (-1 = none)
	int m_iAssistPointMenu = -1;  // point whose settings window is open (-1 = closed)
	bool AssistPointMenuOpen() const { return m_iAssistPointMenu >= 0; }
	void OpenAssistPointMenu(int i) { m_iAssistPointMenu = i; }
	PixelSurfPoint_t* AssistPointMenuPoint(); // nullptr when closed/stale (callers use auto*)
	void CloseAssistPointMenu(bool bSave);    // optionally persists the point file on close
	void DeleteAssistPointMenuPoint();        // erase the open point + persist + close

	// Called from CHLClient_CreateMove after RunPost: returns true while a texture-bug
	// catch wants the current command choked (decrements the remaining choke ticks).
	bool ConsumeTextureBugChoke();

	// recorder route API consumed by the Movement > RECORDER menu tab.
	int  RecorderRouteCount() const { return int(m_vRoutes.size()); }
	std::string RecorderRouteName(int i) const { return (i >= 0 && i < int(m_vRoutes.size())) ? m_vRoutes[i].m_sName : ""; }
	int  RecorderRouteFrames(int i) const { return (i >= 0 && i < int(m_vRoutes.size())) ? int(m_vRoutes[i].m_vFrames.size()) : 0; }
	float RecorderRouteSeconds(int i) const { return RecorderRouteFrames(i) * TICK_INTERVAL; }
	void RecorderTrimRoute(int i, float flStartSec, float flEndSec); // cut N seconds off the start/end of a route + persist
	int  RecorderSelected() const { return m_iSelectedRoute; }
	void RecorderSelect(int i) { m_iSelectedRoute = (i >= 0 && i < int(m_vRoutes.size())) ? i : -1; }
	void RecorderSaveActiveAs(const std::string& sName);   // save m_vRecording as a new route + write file
	void RecorderDeleteRoute(int i);                        // erase a route + rewrite the file
	void RecorderReload();                                  // reload the current map's routes file
	bool RecorderIsRecording() const { return m_bRecording; }
	bool RecorderIsPlaying() const { return m_bPlaying || m_bMovingToStart; }
	int  RecorderActiveFrames() const { return int(m_vRecording.size()); }
	int  RecorderPlaybackIdx() const { return int(m_iPlaybackIdx); }
	std::string RecorderCurrentMap() const;

	int m_iWishCmdrate = -1;
	int m_iWishUpdaterate = -1;
};

ADD_FEATURE(CMisc, Misc);