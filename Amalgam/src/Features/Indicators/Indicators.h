#pragma once
#include "../../Utils/Macros/Macros.h"
#include <string>
#include <vector>
#include <unordered_map>

class CTFPlayer;

struct KeybindIndicator_t
{
    std::string m_sName;
    std::string m_sShortName;
    float m_flAlpha = 0.f;
    float m_flPosX = 0.f;
    float m_flTargetX = 0.f;
    bool m_bActive = false;
    bool m_bRemoving = false;
};

class CIndicators
{
public:
    void Draw(CTFPlayer* pLocal);

private:
    void DrawVelocity(CTFPlayer* pLocal);
    void DrawKeybinds();
    
    std::string GetShortName(const std::string& sName);
    float Lerp(float flCurrent, float flTarget, float flSpeed);

    // Velocity tracking
    int m_iLastVelocity = 0;
    int m_iTakeoffVelocity = 0;
    float m_flTakeoffTime = 0.f;
    bool m_bWasOnGround = true;
    float m_flTakeoffAlpha = 0.f;

    // Keybind indicators with animation
    std::vector<KeybindIndicator_t> m_vKeybindIndicators;
};

ADD_FEATURE(CIndicators, Indicators);
