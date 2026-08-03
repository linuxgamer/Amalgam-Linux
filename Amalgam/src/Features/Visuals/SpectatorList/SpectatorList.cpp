#include "SpectatorList.h"

#include "../../Players/PlayerUtils.h"
#include "../../Spectate/Spectate.h"
#include "../../ImGui/Render.h" // F::Render style-effect surface buffer (Balatro/Blur bg)
#include <algorithm>
#include <cctype>

bool CSpectatorList::GetSpectators(CTFPlayer* pTarget)
{
	m_vSpectators.clear();

	auto pResource = H::Entities.GetResource();
	if (!pResource)
		return false;

	int iTarget = pTarget->entindex();
	for (int n = 1; n <= I::EngineClient->GetMaxClients(); n++)
	{
		auto pPlayer = I::ClientEntityList->GetClientEntity(n)->As<CTFPlayer>();
		bool bLocal = n == I::EngineClient->GetLocalPlayer();

		if (pResource->m_bValid(n) && !pResource->IsFakePlayer(n)
			&& pResource->m_iTeam(I::EngineClient->GetLocalPlayer()) != TEAM_SPECTATOR && pResource->m_iTeam(n) == TEAM_SPECTATOR)
		{
			m_vSpectators.emplace_back(F::PlayerUtils.GetPlayerName(n, pResource->GetName(n)), "possible", -1.f, false, n, pResource->m_iAccountID(n));
			continue;
		}

		if (pTarget->entindex() == n || pResource->IsFakePlayer(n)
			|| !pPlayer || !pPlayer->IsPlayer() || pPlayer->IsAlive()
			|| pTarget->IsDormant() != pPlayer->IsDormant()
			|| pResource->m_iTeam(iTarget) != pResource->m_iTeam(n))
		{
			if (m_mRespawnCache.contains(n))
				m_mRespawnCache.erase(n);
			continue;
		}

		int iObserverTarget = !pPlayer->IsDormant() ? pPlayer->m_hObserverTarget().GetEntryIndex() : iTarget;
		int iObserverMode = pPlayer->m_iObserverMode();
		if (bLocal && F::Spectate.m_iTarget != -1)
		{
			iObserverTarget = F::Spectate.m_hOriginalTarget.GetEntryIndex();
			iObserverMode = F::Spectate.m_iOriginalMode;
		}
		if (iObserverTarget != iTarget || bLocal && !I::EngineClient->IsPlayingDemo() && F::Spectate.m_iTarget == -1)
		{
			if (m_mRespawnCache.contains(n))
				m_mRespawnCache.erase(n);
			continue;
		}

		const char* sMode = "possible";
		if (!pPlayer->IsDormant())
		{
			switch (iObserverMode)
			{
			case OBS_MODE_FIRSTPERSON: sMode = "1st"; break;
			case OBS_MODE_THIRDPERSON: sMode = "3rd"; break;
			default: continue;
			}
		}

		float flRespawnTime = 0.f, flRespawnIn = -1.f;
		bool bRespawnTimeIncreased = false;
		if (pPlayer->IsInValidTeam())
		{
			flRespawnTime = pResource->m_flNextRespawnTime(n);
			flRespawnIn = std::max(floorf(flRespawnTime - TICKS_TO_TIME(I::ClientState->m_ClockDriftMgr.m_nServerTick)), 0.f);
			if (!m_mRespawnCache.contains(n))
				m_mRespawnCache[n] = flRespawnTime;
			else if (m_mRespawnCache[n] + 0.5f < flRespawnTime)
				bRespawnTimeIncreased = true;
		}

		m_vSpectators.emplace_back(F::PlayerUtils.GetPlayerName(n, pResource->GetName(n)), sMode, flRespawnIn, bRespawnTimeIncreased, n, pResource->m_iAccountID(n));
	}

	return !m_vSpectators.empty();
}

bool CSpectatorList::PrepareMinimal(CTFPlayer* pLocal)
{
	// Render-side gate ONLY - the rows are gathered on the main thread in Draw() (see header note: the
	// Present thread must not read entity netvars). Just report whether the Minimal list should draw.
	return pLocal
		&& (Vars::Menu::Indicators.Value & Vars::Menu::IndicatorsEnum::Spectators)
		&& Vars::Menu::SpectatorListStyle.Value == Vars::Menu::SpectatorListStyleEnum::Minimal;
}

void CSpectatorList::Draw(CTFPlayer* pLocal)
{
	if (!(Vars::Menu::Indicators.Value & Vars::Menu::IndicatorsEnum::Spectators))
	{
		m_mRespawnCache.clear();
		return;
	}

	auto pTarget = pLocal;
	switch (pLocal->m_iObserverMode())
	{
	case OBS_MODE_FIRSTPERSON:
	case OBS_MODE_THIRDPERSON:
		pTarget = pLocal->m_hObserverTarget()->As<CTFPlayer>();
	}

	// Minimal style RENDERS in the ImGui pass (CMenu::DrawSpectatorListMinimal) so it matches the menu's
	// exact styled background + rounding. But the GATHER must run here on the main thread - reading entity
	if (Vars::Menu::SpectatorListStyle.Value == Vars::Menu::SpectatorListStyleEnum::Minimal)
	{
		if (!(pTarget && pTarget->IsPlayer() && GetSpectators(pTarget)))
			m_vSpectators.clear();
		return;
	}

	const int iStyle = Vars::Menu::SpectatorListStyle.Value;
	const bool bInterwebz = iStyle == Vars::Menu::SpectatorListStyleEnum::Interwebz;
	// most "panel" styles are always visible, even with nobody spectating you - the
	// box/header stays up. Amalgam/Default AND Clarity bail when empty (clarity-source draws
	const bool bAlwaysVisible = bInterwebz
		|| iStyle == Vars::Menu::SpectatorListStyleEnum::Minimal
		|| iStyle == Vars::Menu::SpectatorListStyleEnum::Kamidere;

	// User panel scale on top of the global menu scale (the spectator font tracks this too).
	const float flListScale = Vars::Menu::SpectatorListScale.Value;
	auto SL = [&](float v) { return int(H::Draw.Scale(v) * flListScale); };

	// Gather the spectators if we have a valid target (this also fills m_vSpectators).
	const bool bHasSpectators = pTarget && pTarget->IsPlayer() && GetSpectators(pTarget);
	if (!bHasSpectators)
	{
		m_vSpectators.clear();
		if (!bAlwaysVisible)
			return;
	}

	// the reference "default" look: a minimal right-aligned list pinned to the top-right corner.
	// 1st-person rows draw in the accent colour, 3rd-person (and "possible") draw white.
	if (iStyle == Vars::Menu::SpectatorListStyleEnum::Default)
	{
		const auto& fFont = H::Fonts.GetFont(FONT_SPECTATOR);
		const int nTall = fFont.m_nTall + H::Draw.Scale(3);
		const int xr = H::Draw.m_nScreenW - H::Draw.Scale(6);
		int yr = H::Draw.Scale(6);
		for (auto& tSpectator : m_vSpectators)
		{
			const Color_t tRowText = FNV1A::Hash32(tSpectator.m_sMode) == FNV1A::Hash32Const("1st")
				? Vars::Menu::Theme::Accent.Value : Color_t{ 255, 255, 255, 255 };
			H::Draw.StringOutlined(fFont, xr, yr, tRowText, Vars::Menu::Theme::Background.Value, ALIGN_TOPRIGHT,
				std::format("{} ({})", tSpectator.m_sName, tSpectator.m_sMode).c_str());
			yr += nTall;
		}
		return;
	}

	// the reference "interwebz" (delnew) look: a right-leaning slanted dark-grey header parallelogram
	// titled SPECTATOR LIST, sitting on top of a column of zebra-striped grey rows (one per
	if (Vars::Menu::SpectatorListStyle.Value == Vars::Menu::SpectatorListStyleEnum::Interwebz)
	{
		const auto& fFont = H::Fonts.GetFont(FONT_SPECTATOR);
		const int iRowH = SL(20);   // delnew's fixed 20px band height (header + each row)
		const int iSkew = iRowH;    // header slants right by exactly one band height
		const int iPad = SL(4);

		// delnew uses a fixed 195px panel; keep that as a minimum but grow it for long names so text
		// never spills out of the box.
		int iWidth = SL(195);
		std::vector<std::string> vLines;
		for (auto& tSpectator : m_vSpectators)
		{
			std::string sLine = std::format("{} ({})", tSpectator.m_sName, tSpectator.m_sMode);
			const int iLineW = int(H::Draw.GetTextSize(sLine.c_str(), fFont).x) + iPad * 2;
			if (iLineW > iWidth)
				iWidth = iLineW;
			vLines.push_back(std::move(sLine));
		}
		const int iCount = int(vLines.size());

		const int bx = Vars::Menu::SpectatorsDisplay.Value.x;
		const int by = Vars::Menu::SpectatorsDisplay.Value.y;

		const Color_t tHeaderBg = { 59, 59, 59, 255 };
		const Color_t tOutline = { 85, 85, 85, 255 };
		// optional background. When off, the fills AND the outlines are skipped - only the
		// text remains.
		const bool bBackground = Vars::Menu::SpectatorListBackground.Value;
		// Per-row text colour is decided in the loop below: 1st-person spectators use the accent colour,
		// 3rd-person (and "possible") are drawn white.

		// Right-leaning header parallelogram fill: the top edge is pushed right by iSkew over its height.
		// (Outline is drawn once below as a single perimeter so the header + rows borders can't misalign.)
		if (bBackground)
		{
			std::vector<Vertex_t> vHeader =
			{
				{ { float(bx + iSkew),          float(by) } },
				{ { float(bx + iWidth + iSkew), float(by) } },
				{ { float(bx + iWidth),         float(by + iRowH) } },
				{ { float(bx),                  float(by + iRowH) } }
			};
			H::Draw.FillPolygon(vHeader, tHeaderBg);
		}
		H::Draw.StringOutlined(fFont, bx + iWidth / 2 + iSkew / 2, by + iRowH / 2, Vars::Menu::Theme::Accent.Value, Vars::Menu::Theme::Background.Value, ALIGN_CENTER, "SPECTATOR LIST");

		// Zebra-striped rectangular rows below the header.
		for (int i = 0; i < iCount; i++)
		{
			const int iRowY = by + iRowH * (i + 1);
			if (bBackground)
				H::Draw.FillRect(bx, iRowY, iWidth, iRowH, (i % 2) ? Color_t{ 75, 75, 75, 255 } : Color_t{ 45, 45, 45, 255 });
			const Color_t tRowText = FNV1A::Hash32(m_vSpectators[i].m_sMode) == FNV1A::Hash32Const("1st")
				? Vars::Menu::Theme::Accent.Value : Color_t{ 255, 255, 255, 255 };
			H::Draw.StringOutlined(fFont, bx + iPad, iRowY + iRowH / 2, tRowText, Vars::Menu::Theme::Background.Value, ALIGN_LEFT, vLines[i].c_str());
		}

		// Single continuous outline tracing the whole shape.
		if (bBackground)
		{
			const float fBottom = float(by + iRowH * (iCount + 1)); // rows extend iCount bands below the header
			std::vector<Vertex_t> vPerimeter =
			{
				{ { float(bx + iSkew),          float(by) } },          // header top-left
				{ { float(bx + iWidth + iSkew), float(by) } },          // header top-right
				{ { float(bx + iWidth),         float(by + iRowH) } },  // header bottom-right == rows top-right
				{ { float(bx + iWidth),         fBottom } },            // rows bottom-right
				{ { float(bx),                  fBottom } },            // rows bottom-left
				{ { float(bx),                  float(by + iRowH) } },  // header bottom-left == rows top-left
				{ { float(bx + iSkew),          float(by) } }           // close the loop
			};
			H::Draw.LinePolygon(vPerimeter, tOutline);
		}
		return;
	}

	// Minimal (ex-Arbuzebra port): a 200px-wide rounded dark panel with a centred "Spectators"
	// title, a faded separator line under it and left-aligned avatar + "name | mode" rows. Always
	if (iStyle == Vars::Menu::SpectatorListStyleEnum::Minimal)
	{
		const auto& fFont = H::Fonts.GetFont(FONT_SPECTATOR_MINIMAL); // thin player rows (weight 300)
		const auto& fTitle = H::Fonts.GetFont(FONT_SPECTATOR_MINIMAL_TITLE); // semibold "SPECTATORS" = watermark
		const int iWidth = SL(200);
		const int iTitleTop = SL(5);
		const int iTitleToList = SL(15);
		const int iLeftMargin = SL(10);
		const int iRowStride = SL(24); // item height + spacing
		const int iBottomPad = SL(10);
		const int nTall = fFont.m_nTall;

		// Avatar sizing: radius fits inside the row, matching Kamidere's approach.
		const int iAvatarR = std::max(2, iRowStride / 2 - SL(3));
		const int iAvatarGap = SL(6);

		const int bx = Vars::Menu::SpectatorsDisplay.Value.x;
		const int by = Vars::Menu::SpectatorsDisplay.Value.y;
		const int iCount = int(m_vSpectators.size());
		const int iRows = std::max(iCount, 1); // empty list still shows the "No spectators" row

		const int iListTop = iTitleTop + nTall + iTitleToList;
		const int iHeight = iListTop + iRows * iRowStride - (iRowStride - nTall) + iBottomPad;

		// Background: the shared menu/panel style (Visuals > MENU > Settings) - the exact menu bg colour
		// + rounding. A single FillRoundRect, identical to the checkpoints + model-preview panels, so all
		H::Draw.StyledPanel(bx, by, iWidth, iHeight); // shared panel bg+outline (matches watermark/menu)
		H::Draw.StringOutlined(fTitle, bx + iWidth / 2, by + iTitleTop, Vars::Menu::Theme::Accent.Value, Vars::Menu::Theme::Background.Value, ALIGN_TOP, "SPECTATORS");

		// Faded separator line under the title: brightest in the middle, fading out to both edges.
		{
			const int iLineY = by + iTitleTop + nTall + SL(4);
			const int iLineW = iWidth - iLeftMargin * 2;
			const int iHalfW = iLineW / 2;
			const Color_t tEdge = { 0, 0, 0, 0 };
			H::Draw.GradientRect(bx + iLeftMargin, iLineY, iHalfW, 1, tEdge, Vars::Menu::Theme::Accent.Value, true);
			H::Draw.GradientRect(bx + iLeftMargin + iHalfW, iLineY, iLineW - iHalfW, 1, Vars::Menu::Theme::Accent.Value, tEdge, true);
		}

		const Color_t tText = Vars::Menu::SpectatorListTextColor.Value;
		if (iCount == 0)
		{
			H::Draw.StringOutlined(fFont, bx + iWidth / 2, by + iListTop, Vars::Menu::Theme::Inactive.Value, Vars::Menu::Theme::Background.Value, ALIGN_TOP, "no spectators");
		}
		else
		{
			auto pMinResource = H::Entities.GetResource();
			// Text starts after the avatar circle and a small gap.
			const int iTextX = bx + iLeftMargin + iAvatarR * 2 + iAvatarGap;
			for (int i = 0; i < iCount; i++)
			{
				// Row vertical centre for avatar alignment.
				const int iRowCenterY = by + iListTop + i * iRowStride + nTall / 2;
				if (pMinResource)
					H::Draw.AvatarCircle(bx + iLeftMargin + iAvatarR, iRowCenterY, float(iAvatarR), pMinResource->m_iAccountID(m_vSpectators[i].m_iIndex));

				// First-person spectators draw in the accent colour (matching the other styles); the
				// rest use the configured text colour.
				const bool b1st = FNV1A::Hash32(m_vSpectators[i].m_sMode) == FNV1A::Hash32Const("1st");
				H::Draw.StringOutlined(fFont, iTextX, by + iListTop + i * iRowStride, b1st ? Vars::Menu::Theme::Accent.Value : tText, Vars::Menu::Theme::Background.Value, ALIGN_TOPLEFT,
					std::format("{} | {}", m_vSpectators[i].m_sName, m_vSpectators[i].m_sMode).c_str());
			}
		}
		return;
	}

	// Clarity style: a subtle dark vertical-gradient box drawn at the draggable Spectators display
	// position, with a centred white "spectators" title and one evenly-spaced row per spectator showing
	if (iStyle == Vars::Menu::SpectatorListStyleEnum::Clarity)
	{
		const auto& fFont = H::Fonts.GetFont(FONT_SPECTATOR_CLARITY); // clarity-source watermark_font (Tahoma 12, weight 500, aliased)
		const int iCount = int(m_vSpectators.size());
		if (iCount == 0)
			return;

		const int nTall = fFont.m_nTall;
		const int iPad = SL(10);          // horizontal text padding
		const int iLineH = nTall + SL(6); // uniform vertical rhythm shared by the title and every row
		const int iTopPad = SL(6);
		const int iBotPad = SL(6);

		// Build "name (mode)" once per spectator, tracking the widest line so the box never clips.
		std::vector<std::string> vLines;
		vLines.reserve(iCount);
		int iWidth = int(H::Draw.GetTextSize("spectators", H::Fonts.GetFont(FONT_SPECTATOR_CLARITY_TITLE)).x) + iPad * 2; // bold title sets the floor
		for (auto& tSpectator : m_vSpectators)
		{
			std::string sLine = std::format("{} ({})", tSpectator.m_sName, tSpectator.m_sMode);
			const int iLineW = int(H::Draw.GetTextSize(sLine.c_str(), fFont).x) + iPad * 2;
			if (iLineW > iWidth)
				iWidth = iLineW;
			vLines.push_back(std::move(sLine));
		}

		// Title takes the first band, then one band per row; bottom padding closes the box.
		const int iHeight = iTopPad + iCount * iLineH + nTall + iBotPad;
		// Draggable like the other panel styles - top-left pinned to the Spectators display position.
		const int bx = Vars::Menu::SpectatorsDisplay.Value.x;
		const int by = Vars::Menu::SpectatorsDisplay.Value.y;

		// Subtle vertical gradient background: a slightly-lit top fading down to a darker bottom. Built
		// the clarity way — an opaque base fill with the second colour faded in vertically over it.
		auto ClarityVGradient = [](int gx, int gy, int gw, int gh, Color_t tTop, Color_t tBottom)
		{
			H::Draw.FillRect(gx, gy, gw, gh, tTop);
			I::MatSystemSurface->DrawSetColor(tBottom.r, tBottom.g, tBottom.b, tBottom.a);
			I::MatSystemSurface->DrawFilledRectFade(gx, gy, gx + gw, gy + gh, 0, tBottom.a, false);
		};
		ClarityVGradient(bx, by, iWidth, iHeight, Color_t{ 32, 32, 32, 255 }, Color_t{ 15, 15, 15, 255 });

		// Title: "spectators", white, centred in the first band. Bold scheme font (a bit thicker than the
		// rows); plain draw (aliased glyph + font dropshadow).
		H::Draw.String(H::Fonts.GetFont(FONT_SPECTATOR_CLARITY_TITLE), bx + iWidth / 2, by + iTopPad, Color_t{ 255, 255, 255, 255 }, ALIGN_TOP, "spectators");

		// Rows: grey, or the accent colour for first-person spectators — one even band apart.
		const Color_t tAccent = Vars::Menu::Theme::Accent.Value;
		const Color_t tGrey = Color_t{ 158, 158, 158, 255 };
		for (int i = 0; i < iCount; i++)
		{
			const bool b1st = FNV1A::Hash32(m_vSpectators[i].m_sMode) == FNV1A::Hash32Const("1st");
			H::Draw.String(fFont, bx + iPad, by + iTopPad + (i + 1) * iLineH, b1st ? tAccent : tGrey, ALIGN_TOPLEFT, vLines[i].c_str());
		}
		return;
	}

	// Kamidere: fixed-width rounded panel - near-black header bar with "Spectators" on the left,
	// opaque grey body with one row per spectator: circular steam avatar, then "name | mode".
	if (iStyle == Vars::Menu::SpectatorListStyleEnum::Kamidere)
	{
		const auto& fFont = H::Fonts.GetFont(FONT_SPECTATOR_KAMIDERE); // menu body face, smooth AA
		const int nTall = fFont.m_nTall;
		const int iPad = SL(12);
		const int iRadius = SL(7);
		const int iHeaderH = std::max(SL(28), nTall + SL(6));
		const int iRowH = std::max(SL(32), nTall + SL(6));
		const int iAvatarR = std::max(2, iRowH / 2 - SL(3));
		const int iAvatarGap = SL(8);
		const int iBottomPad = SL(5);

		const int bx = Vars::Menu::SpectatorsDisplay.Value.x;
		const int by = Vars::Menu::SpectatorsDisplay.Value.y;
		const int iCount = int(m_vSpectators.size());

		// Width grows to the widest "name | mode" row (avatar + gaps + padding) so names never clip;
		// SL(190) stays as the minimum. The header label must fit too.
		const int iTextOffset = iPad + iAvatarR * 2 + iAvatarGap; // x from bx where row text starts
		int iWidth = std::max(SL(190), int(H::Draw.GetTextSize("Spectators", fFont).x) + iPad * 2);
		for (auto& tSpectator : m_vSpectators)
		{
			const std::string sLine = std::format("{} | {}", tSpectator.m_sName, tSpectator.m_sMode);
			const int iNeed = iTextOffset + int(H::Draw.GetTextSize(sLine.c_str(), fFont).x) + iPad;
			if (iNeed > iWidth)
				iWidth = iNeed;
		}

		const int iBodyH = iCount ? iCount * iRowH + iBottomPad : SL(14);
		const int iHeight = iHeaderH + iBodyH;
		const Color_t tHeader = { 26, 26, 26, 255 };
		const Color_t tBody = { 26, 26, 26, 175 };

		// Grey rounded panel first, then the dark header on top (its bottom edge squared off so
		// only the panel's outer corners stay rounded).
		H::Draw.FillRoundRect(bx, by, iWidth, iHeight, iRadius, tBody);
		H::Draw.FillRoundRect(bx, by, iWidth, iHeaderH, iRadius, tHeader);
		H::Draw.FillRect(bx, by + iHeaderH - iRadius, iWidth, iRadius, tHeader);

		// Header label on the left.
		H::Draw.StringOutlined(fFont, bx + iPad, by + iHeaderH / 2, Vars::Menu::SpectatorListTextColor.Value, Vars::Menu::Theme::Background.Value, ALIGN_LEFT, "Spectators");

		if (iCount == 0)
			return;

		const Color_t tText = Vars::Menu::SpectatorListTextColor.Value;
		const int iTextX = bx + iPad + iAvatarR * 2 + iAvatarGap;
		const int iMaxTextW = bx + iWidth - iPad - iTextX;
		for (int i = 0; i < iCount; i++)
		{
			const int iRowCenterY = by + iHeaderH + i * iRowH + iRowH / 2;
			// Use the account-id resolved at gather time (Minimal's pattern) - re-reading the resource netvar
			// here would race the sim on the Present thread. AvatarCircle no-ops on id 0.
			H::Draw.AvatarCircle(bx + iPad + iAvatarR, iRowCenterY, float(iAvatarR), m_vSpectators[i].m_uAccountID);

			std::string sLine = std::format("{} | {}", m_vSpectators[i].m_sName, m_vSpectators[i].m_sMode);
			if (int(H::Draw.GetTextSize(sLine.c_str(), fFont).x) > iMaxTextW)
			{
				// Trim the name (not the mode) until "name... | mode" fits the fixed panel width.
				std::string sName = m_vSpectators[i].m_sName;
				while (sName.size() > 1)
				{
					// Drop a whole UTF-8 codepoint (continuation bytes are 10xxxxxx).
					sName.pop_back();
					while (sName.size() > 1 && (sName.back() & 0xC0) == 0x80)
						sName.pop_back();
					sLine = std::format("{}... | {}", sName, m_vSpectators[i].m_sMode);
					if (int(H::Draw.GetTextSize(sLine.c_str(), fFont).x) <= iMaxTextW)
						break;
				}
			}
			H::Draw.StringOutlined(fFont, iTextX, iRowCenterY, tText, Vars::Menu::Theme::Background.Value, ALIGN_LEFT, sLine.c_str());
		}
		return;
	}

	int x = Vars::Menu::SpectatorsDisplay.Value.x;
	int y = Vars::Menu::SpectatorsDisplay.Value.y + 8;
	int iconOffset = 0;
	const auto& fFont = H::Fonts.GetFont(FONT_SPECTATOR);
	const int nTall = fFont.m_nTall + H::Draw.Scale(3);

	EAlign align = ALIGN_TOP;
	if (x <= 100 + H::Draw.Scale(50, Scale_Round))
	{
		x -= H::Draw.Scale(42, Scale_Round);
		align = ALIGN_TOPLEFT;
	}
	else if (x >= H::Draw.m_nScreenW - 100 + H::Draw.Scale(50, Scale_Round))
	{
		x += H::Draw.Scale(42, Scale_Round);
		align = ALIGN_TOPRIGHT;
	}

	auto pResource = H::Entities.GetResource();
	int iIndex = pTarget->entindex();
	const char* sName = pTarget != pLocal ? F::PlayerUtils.GetPlayerName(iIndex, pResource->GetName(iIndex)) : "You";
	H::Draw.StringOutlined(fFont, x, y, Vars::Menu::Theme::Accent.Value, Vars::Menu::Theme::Background.Value, align, std::format("Spectating {}:", sName).c_str());
	for (auto& tSpectator : m_vSpectators)
	{
		y += nTall;

		Color_t tColor = Vars::Menu::Theme::Active.Value;
		if (H::Entities.IsFriend(tSpectator.m_iIndex))
			tColor = F::PlayerUtils.m_vTags[F::PlayerUtils.TagToIndex(FRIEND_TAG)].m_tColor;
		else if (H::Entities.InParty(tSpectator.m_iIndex))
			tColor = F::PlayerUtils.m_vTags[F::PlayerUtils.TagToIndex(PARTY_TAG)].m_tColor;
		else if (tSpectator.m_bRespawnTimeIncreased)
			tColor = F::PlayerUtils.m_vTags[F::PlayerUtils.TagToIndex(CHEATER_TAG)].m_tColor;
		else if (FNV1A::Hash32(tSpectator.m_sMode) == FNV1A::Hash32Const("1st"))
			tColor = tColor.Lerp({ 255, 150, 0, 255 }, 0.5f);

		if (tSpectator.m_flRespawnIn != -1.f)
			H::Draw.StringOutlined(fFont, x + iconOffset, y, tColor, Vars::Menu::Theme::Background.Value, align, std::format("{} ({} - respawn {}s)", tSpectator.m_sName, tSpectator.m_sMode, tSpectator.m_flRespawnIn).c_str());
		else
			H::Draw.StringOutlined(fFont, x + iconOffset, y, tColor, Vars::Menu::Theme::Background.Value, align, std::format("{} ({})", tSpectator.m_sName, tSpectator.m_sMode).c_str());
	}
}