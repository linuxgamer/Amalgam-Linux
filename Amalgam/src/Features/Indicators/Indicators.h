#pragma once
#include "../../Utils/Macros/Macros.h"
#include "../../SDK/Definitions/Types.h"
#include <string>
#include <vector>
#include <deque>
#include <unordered_map>

class CTFPlayer;

struct MovementIndicator_t
{
    int m_iId = -1;             // stable feature id (animation state survives label edits)
    std::string m_sLabel;       // displayed text (customizable per feature)
    bool m_bAccent = false;     // draw the text in the accent color (edge-bug "name")
    float m_flAlpha = 0.f;      // 0..1 eased fade (derived from m_flEnter)
    float m_flEnter = 0.f;      // 0..1 raw entry/exit progress (easing input)
    float m_flPosX = 0.f;       // current X (animated)
    float m_flPosY = 0.f;       // current Y (animated, vertical/the reference layouts)
    float m_flTargetX = 0.f;    // target X (layout)
    float m_flTargetY = 0.f;    // target Y (layout)
    float m_flDetect = 0.f;     // 0..1 detect-colour intensity
    bool m_bActive = false;     // feature currently enabled
    bool m_bRemoving = false;
    bool m_bWasDetected = false; // previous raw detect state (rising-edge -> particle burst)
    bool m_bSpawnBurst = false;  // a detection just fired; spawn particles once the label is laid out
};

// A single detection particle: a shape launched from the label centre that drifts outward and fades.
struct IndicatorParticle_t
{
    float m_flX = 0.f, m_flY = 0.f;       // screen position (px)
    float m_flVelX = 0.f, m_flVelY = 0.f; // velocity (px/s)
    float m_flLife = 0.f;                 // remaining life (s)
    float m_flMaxLife = 1.f;              // initial life (s) -> alpha fade
    float m_flScale = 3.f;                // radius (px)
    float m_flRot = 0.f;                  // current rotation (rad)
    float m_flRotSpeed = 0.f;             // spin (rad/s)
    int m_iShape = 0;                     // resolved shape (Random picks one at spawn)
    Color_t m_cColor = {};
};

// A single expanding ring. Spawned in a small staggered cluster so the burst reads as a few ripples.
struct IndicatorRipple_t
{
    float m_flX = 0.f, m_flY = 0.f; // centre (label centre at spawn)
    float m_flAge = 0.f;            // seconds since spawn; negative = staggered start delay
    float m_flDuration = 0.7f;      // expand + fade time
    float m_flMaxRadius = 22.f;     // radius reached at end of life
    float m_flThickness = 1.5f;     // ring line thickness (px)
    Color_t m_cColor = {};
};

class CIndicators
{
public:
    void Draw(CTFPlayer* pLocal);

private:
    void DrawVelocity(CTFPlayer* pLocal);
    void DrawKeybinds();
    // Detection effects (spawned from a keybind label when the action actually fires).
    void SpawnParticleBurst(float flCenterX, float flCenterY, Color_t cColor);
    void UpdateDrawParticles();
    void DrawParticleShape(int iShape, float flX, float flY, float flScale, float flRot, Color_t cColor);
    void SpawnRippleBurst(float flCenterX, float flCenterY, Color_t cColor);
    void UpdateDrawRipples();
    void UpdateDrawEffects(); // advance + draw particles and ripples together

    float Lerp(float flCurrent, float flTarget, float flSpeed);
    float LerpSmooth(float flCurrent, float flTarget, float flSpeed); // frame-rate independent
    // Critically-damped spring (matches the SpringToward feel).
    void SpringToward(float& flVal, float& flVel, float flTarget, float flDt, float flOmega = 22.f);
    // Speed -> color gradient (slow/mid/fast) with master alpha applied.
    Color_t ResolveSpeedColor(float flSpeed, float flMasterAlpha);

    static constexpr int m_iHistMax = 185; // graph sample count

    // Velocity number state
    float m_flDisplayVal = -1.f;   // smoothed displayed number (-1 = snap on first frame)
    float m_flDisplayV = 0.f;
    float m_flMainSlotW = 0.f;     // animated main-number slot width (0 = snap)
    float m_flMainSlotWV = 0.f;
    float m_flMainPosX = -1.f;     // animated left edge of main number (-1 = snap)
    float m_flAlpha = 0.f;         // number fade alpha
    float m_flAlphaV = 0.f;
    bool m_bWasOnGround = false;

    // Pre-jump (takeoff) speed state
    float m_flPreVal = 0.f;
    float m_flPreTimer = 0.f;
    float m_flPreAlpha = 0.f;
    float m_flPreAlphaV = 0.f;
    float m_flPreXOff = 0.f;       // slide/expand offset
    float m_flPreXOffV = 0.f;
    float m_flPreSlotW = 0.f;
    float m_flPreSlotWV = 0.f;
    float m_flCenterDelay = 0.f;

    // Graph state
    std::deque<float> m_dqHistory; // newest at front
    float m_flGraphAlpha = 0.f;
    float m_flGraphAlphaV = 0.f;

    // Movement indicators with animation
    std::vector<MovementIndicator_t> m_vKeybindIndicators;

    // Live detection effects (shared pools; they outlive the label that spawned them).
    std::vector<IndicatorParticle_t> m_vParticles;
    std::vector<IndicatorRipple_t> m_vRipples;

    // Edge bug fires its effect on the *completion* counter (not the early detect flag); -1 = uninitialised.
    int m_iLastEdgeBugSuccess = -1;
};

ADD_FEATURE(CIndicators, Indicators);
