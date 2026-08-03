#include "Indicators.h"
#include "../../SDK/SDK.h"
#include "../Binds/Binds.h"
#include <algorithm>
#include <cmath>

float CIndicators::Lerp(float flCurrent, float flTarget, float flSpeed)
{
    float flDelta = flTarget - flCurrent;
    float flStep = flDelta * flSpeed * I::GlobalVars->frametime;
    
    if (std::abs(flDelta) < 0.01f)
        return flTarget;
    
    return flCurrent + flStep;
}

std::string CIndicators::GetShortName(const std::string& sName)
{
    // Common abbreviations
    std::string sLower = sName;
    std::transform(sLower.begin(), sLower.end(), sLower.begin(), ::tolower);
    
    if (sLower.find("edgebug") != std::string::npos || sLower.find("edge bug") != std::string::npos)
        return "eb";
    if (sLower.find("minijump") != std::string::npos || sLower.find("mini jump") != std::string::npos)
        return "mj";
    if (sLower.find("longjump") != std::string::npos || sLower.find("long jump") != std::string::npos)
        return "lj";
    if (sLower.find("pixelsurf") != std::string::npos || sLower.find("pixel surf") != std::string::npos)
        return "px";
    
    // Not in whitelist - return empty to skip
    return "";
}

void CIndicators::Draw(CTFPlayer* pLocal)
{
    if (!pLocal || !pLocal->IsAlive())
        return;

    if (Vars::Visuals::UI::VelocityIndicator.Value)
        DrawVelocity(pLocal);

    if (Vars::Visuals::UI::KeybindIndicator.Value)
        DrawKeybinds();
}

void CIndicators::DrawVelocity(CTFPlayer* pLocal)
{
    const Vec3 vVelocity = pLocal->m_vecVelocity();
    const int iVelocity = static_cast<int>(vVelocity.Length2D());
    const bool bOnGround = pLocal->OnSolid();

    // Track takeoff velocity
    if (m_bWasOnGround && !bOnGround)
    {
        m_iTakeoffVelocity = m_iLastVelocity;
        m_flTakeoffTime = I::GlobalVars->curtime + 2.f;
    }
    m_bWasOnGround = bOnGround;
    m_iLastVelocity = iVelocity;

    // Animate takeoff alpha
    bool bShowTakeoff = Vars::Visuals::UI::VelocityTakeoff.Value && m_flTakeoffTime > I::GlobalVars->curtime && !bOnGround;
    m_flTakeoffAlpha = Lerp(m_flTakeoffAlpha, bShowTakeoff ? 1.f : 0.f, 15.f);

    // Colors based on velocity
    Color_t clrVelocity = { 255, 255, 255, 255 };
    if (iVelocity > 300)
        clrVelocity = Vars::Menu::Theme::Accent.Value;

    // Position
    const int iScreenW = H::Draw.m_nScreenW;
    const int iScreenH = H::Draw.m_nScreenH;
    const int iPosY = iScreenH - Vars::Visuals::UI::VelocityPos.Value;

    // Main velocity text
    std::string sVelocity = std::to_string(iVelocity);
    
    const auto& tFont = H::Fonts.GetFont(FONT_INDICATORS);
    Vec2 vTextSize = H::Draw.GetTextSize(sVelocity.c_str(), tFont);

    // Takeoff velocity
    std::string sTakeoff;
    int iTakeoffW = 0;
    if (m_flTakeoffAlpha > 0.01f)
    {
        sTakeoff = "(" + std::to_string(m_iTakeoffVelocity) + ")";
        Vec2 vTakeoffSize = H::Draw.GetTextSize(sTakeoff.c_str(), tFont);
        iTakeoffW = static_cast<int>((vTakeoffSize.x + 8) * m_flTakeoffAlpha);
    }

    const int iTotalW = static_cast<int>(vTextSize.x) + iTakeoffW;
    const int iPosX = (iScreenW - iTotalW) / 2;

    // Draw shadow + text
    H::Draw.String(tFont, iPosX + 1, iPosY + 1, { 0, 0, 0, 200 }, ALIGN_TOPLEFT, sVelocity.c_str());
    H::Draw.String(tFont, iPosX, iPosY, clrVelocity, ALIGN_TOPLEFT, sVelocity.c_str());

    // Draw takeoff with fade
    if (m_flTakeoffAlpha > 0.01f)
    {
        int iAlpha = static_cast<int>(180 * m_flTakeoffAlpha);
        Color_t clrTakeoff = { 180, 180, 180, static_cast<byte>(iAlpha) };
        Color_t clrShadow = { 0, 0, 0, static_cast<byte>(static_cast<int>(200 * m_flTakeoffAlpha)) };
        int iTakeoffX = iPosX + static_cast<int>(vTextSize.x) + 8;
        H::Draw.String(tFont, iTakeoffX + 1, iPosY + 1, clrShadow, ALIGN_TOPLEFT, sTakeoff.c_str());
        H::Draw.String(tFont, iTakeoffX, iPosY, clrTakeoff, ALIGN_TOPLEFT, sTakeoff.c_str());
    }
}

void CIndicators::DrawKeybinds()
{
    const float flAnimSpeed = 12.f;
    const float flSpacing = 12.f;
    
    // Collect current active binds
    std::vector<std::string> vActiveBinds;
    for (size_t i = 0; i < F::Binds.m_vBinds.size(); i++)
    {
        auto& tBind = F::Binds.m_vBinds[i];
        
        if (tBind.m_sName.empty())
            continue;
        
        if (tBind.m_iVisibility == BindVisibilityEnum::Hidden)
            continue;

        if (tBind.m_bActive)
            vActiveBinds.push_back(tBind.m_sName);
    }

    // Update indicators - mark inactive ones for removal
    for (auto& ind : m_vKeybindIndicators)
    {
        auto it = std::find(vActiveBinds.begin(), vActiveBinds.end(), ind.m_sName);
        if (it == vActiveBinds.end())
        {
            ind.m_bActive = false;
            ind.m_bRemoving = true;
        }
        else
        {
            ind.m_bActive = true;
            ind.m_bRemoving = false;
            vActiveBinds.erase(it);
        }
    }

    // Add new indicators
    for (const auto& sName : vActiveBinds)
    {
        std::string sShort = GetShortName(sName);
        if (sShort.empty())
            continue; // Skip if not in whitelist
            
        KeybindIndicator_t ind;
        ind.m_sName = sName;
        ind.m_sShortName = sShort;
        ind.m_flAlpha = 0.f;
        ind.m_bActive = true;
        ind.m_bRemoving = false;
        ind.m_flPosX = static_cast<float>(H::Draw.m_nScreenW + 50);
        m_vKeybindIndicators.push_back(ind);
    }

    // Animate and remove dead indicators
    for (auto it = m_vKeybindIndicators.begin(); it != m_vKeybindIndicators.end();)
    {
        auto& ind = *it;
        
        // Animate alpha
        float flTargetAlpha = ind.m_bRemoving ? 0.f : 1.f;
        ind.m_flAlpha = Lerp(ind.m_flAlpha, flTargetAlpha, flAnimSpeed);
        
        // Remove if fully faded
        if (ind.m_bRemoving && ind.m_flAlpha < 0.01f)
        {
            it = m_vKeybindIndicators.erase(it);
            continue;
        }
        ++it;
    }

    if (m_vKeybindIndicators.empty())
        return;

    const int iScreenW = H::Draw.m_nScreenW;
    const int iScreenH = H::Draw.m_nScreenH;
    const int iPosY = iScreenH - Vars::Visuals::UI::KeybindPos.Value;

    const auto& tFont = H::Fonts.GetFont(FONT_INDICATORS);

    // Calculate total width and target positions
    float flTotalW = 0.f;
    std::vector<float> vWidths;
    for (auto& ind : m_vKeybindIndicators)
    {
        Vec2 vSize = H::Draw.GetTextSize(ind.m_sShortName.c_str(), tFont);
        float flWidth = vSize.x * ind.m_flAlpha;
        vWidths.push_back(vSize.x);
        flTotalW += flWidth + (ind.m_flAlpha > 0.01f ? flSpacing * ind.m_flAlpha : 0.f);
    }
    if (flTotalW > 0.f)
        flTotalW -= flSpacing * m_vKeybindIndicators.back().m_flAlpha;

    // Calculate target positions
    float flStartX = (iScreenW - flTotalW) / 2.f;
    float flCurrentX = flStartX;
    for (size_t i = 0; i < m_vKeybindIndicators.size(); i++)
    {
        auto& ind = m_vKeybindIndicators[i];
        ind.m_flTargetX = flCurrentX;
        flCurrentX += vWidths[i] * ind.m_flAlpha + flSpacing * ind.m_flAlpha;
    }

    // Animate positions and draw
    for (size_t i = 0; i < m_vKeybindIndicators.size(); i++)
    {
        auto& ind = m_vKeybindIndicators[i];
        
        // Animate position
        ind.m_flPosX = Lerp(ind.m_flPosX, ind.m_flTargetX, flAnimSpeed);
        
        if (ind.m_flAlpha < 0.01f)
            continue;

        int iAlpha = static_cast<int>(255 * ind.m_flAlpha);
        int iShadowAlpha = static_cast<int>(200 * ind.m_flAlpha);
        
        Color_t clr = Vars::Menu::Theme::Accent.Value;
        clr.a = static_cast<byte>(iAlpha);
        Color_t clrShadow = { 0, 0, 0, static_cast<byte>(iShadowAlpha) };

        int iPosX = static_cast<int>(ind.m_flPosX);
        H::Draw.String(tFont, iPosX + 1, iPosY + 1, clrShadow, ALIGN_TOPLEFT, ind.m_sShortName.c_str());
        H::Draw.String(tFont, iPosX, iPosY, clr, ALIGN_TOPLEFT, ind.m_sShortName.c_str());
    }
}
