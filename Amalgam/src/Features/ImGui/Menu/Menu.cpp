#include "Menu.h"

#include "Components.h"
#include "../../Configs/Configs.h"
#include "../../Binds/Binds.h"
#include "../../Visuals/Groups/Groups.h"
#include "../../Players/PlayerUtils.h"
#include "../../Spectate/Spectate.h"
#include "../../Resolver/Resolver.h"
#include "../../Visuals/Visuals.h"
#include "../../Visuals/ModelPreview/ModelPreview.h"
#include "../../Visuals/SpectatorList/SpectatorList.h"
#include "../../Misc/Misc.h"
#include "../../Output/Output.h"
#include "../media_player.h"

// Current logo spin angle (radians) from the Visuals > Logo rotation-speed slider (deg/sec).
// Returns 0 when disabled so the logo draws axis-aligned. Shared by the menu header + watermark.
static float LogoSpinAngle()
{
	const float flSpeed = Vars::Menu::LogoRotateSpeed.Value;
	if (flSpeed == 0.f) return 0.f;
	return static_cast<float>(ImGui::GetTime()) * flSpeed * (IM_PI / 180.f);
}

static bool LogoSpinning() { return Vars::Menu::LogoRotateSpeed.Value != 0.f; }

// A spinning square's corners trace a circle ~1.41x its side. When spinning we keep the drawn quad
static constexpr float kLogoSpinSlot = 1.30f; // reserved slot multiplier while spinning
static constexpr float kLogoSpinDraw = 1.00f; // drawn quad as a fraction of the static logo size

// Draws a square logo texture centred at vCenter, rotated flAngle radians, tinted col. AddImageQuad
// keeps the rotation cheap and is identical to AddImage at flAngle == 0.
static void DrawLogoRotated(ImDrawList* pDraw, ImTextureID tex, const ImVec2& vCenter, float flSize, float flAngle, ImU32 col)
{
	const float c = ImCos(flAngle), s = ImSin(flAngle);
	const float h = flSize * 0.5f;
	const ImVec2 vOff[4] = { ImVec2(-h, -h), ImVec2(h, -h), ImVec2(h, h), ImVec2(-h, h) };
	ImVec2 p[4];
	for (int i = 0; i < 4; ++i)
		p[i] = ImVec2(vCenter.x + vOff[i].x * c - vOff[i].y * s,
		              vCenter.y + vOff[i].x * s + vOff[i].y * c);
	pDraw->AddImageQuad(tex, p[0], p[1], p[2], p[3],
		ImVec2(0, 0), ImVec2(1, 0), ImVec2(1, 1), ImVec2(0, 1), col);
}

// Stacked outline layers (Visuals > MENU > Settings) drawn just outside the panel edge, each layer
// starting outside the previous. Shared by every styled panel so they all carry the same border/outline
static void DrawStyledOutline(ImDrawList* pDraw, const ImVec2& vMin, const ImVec2& vMax, float flAlpha = 1.f)
{
	using namespace ImGui;
	flAlpha = ImClamp(flAlpha, 0.f, 1.f);
	const float flRound = H::Draw.Scale(Vars::Menu::Style::Rounding.Value);
	const int iLayers = ImClamp(Vars::Menu::Style::OutlineCount.Value, 0, 4);

	// Antialiased outline: one smooth thick stroke per layer (AA on), each layer stacking just outside
	// the previous.
	const ImDrawListFlags flSavedFlags = pDraw->Flags;
	pDraw->Flags |= ImDrawListFlags_AntiAliasedLines;

	// The outline expands OUTWARD past the panel rect, so without a full-screen clip the owning ImGui
	// window clips away everything outside its rect - which made tightly-fit windows (the model preview)
	pDraw->PushClipRectFullScreen();

	float flOffset = 0.f; // accumulated thickness of layers already drawn (each stacks outside the last)
	for (int i = 0; i < iLayers; ++i)
	{
		Color_t oc; float flThickRaw, flBlurRaw;
		if (!GetStyleOutlineLayer(i, oc, flThickRaw, flBlurRaw))
			break;
		const float flThick = ImMax(1.f, H::Draw.Scale(flThickRaw));

		// Main stroke: one AA'd stroke of the requested thickness, centred just outside the panel edge.
		const int iMainA = ImClamp(int(oc.a * flAlpha), 0, 255);
		const float e = flOffset + flThick * 0.5f;
		pDraw->AddRect({ vMin.x - e, vMin.y - e }, { vMax.x + e, vMax.y + e },
			IM_COL32(oc.r, oc.g, oc.b, iMainA), flRound > 0.f ? flRound + e : 0.f, 0, flThick);
		flOffset += flThick;
	}

	pDraw->PopClipRect();
	pDraw->Flags = flSavedFlags;
}

// Shared background renderer for all overlay panels (main menu, watermark, media player, pixel-surf
static void DrawStyledBackground(ImDrawList* pDraw, const ImVec2& vMin, const ImVec2& vMax, float flAlpha = 1.f)
{
	using namespace ImGui;
	flAlpha = ImClamp(flAlpha, 0.f, 1.f);
	const float flRound = H::Draw.Scale(Vars::Menu::Style::Rounding.Value);
	const Color_t c = Vars::Menu::Style::Color.Value;
	pDraw->AddRectFilled(vMin, vMax, IM_COL32(c.r, c.g, c.b, int(c.a * flAlpha)), flRound);
	DrawStyledOutline(pDraw, vMin, vMax, flAlpha);
}

// Main menu window rect, captured each frame in CMenu::Draw so RenderModelPreviewWindow can match the
// menu's height and anchor itself to the menu's right edge.
static ImVec2 g_vMainMenuPos  = {};
static ImVec2 g_vMainMenuSize = {};

void CMenu::DrawMenu()
{
	using namespace ImGui;

	static bool bSetPosition = false;
	if (!bSetPosition)
	{
		SetNextWindowPos((GetIO().DisplaySize - ImVec2(H::Draw.Scale(750), H::Draw.Scale(620))) / 2, ImGuiCond_Always);
		SetNextWindowSize({ H::Draw.Scale(750), H::Draw.Scale(620) }, ImGuiCond_Always);
		bSetPosition = true;
	}

	PushStyleVar(ImGuiStyleVar_WindowMinSize, { H::Draw.Scale(750), H::Draw.Scale(620) });
	
	if (Begin("Main", nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoBackground))
	{
		ImVec2 vWindowPos = GetWindowPos();
		ImVec2 vWindowSize = GetWindowSize();
		g_vMainMenuPos = vWindowPos; g_vMainMenuSize = vWindowSize; // for the model preview to match height/anchor
		ImVec2 vDrawPos = GetDrawPos();
		auto pDrawList = GetWindowDrawList();

		// Draw main background (shared style: rounding + colour from Config > STYLE, no outline)
		float flInset = H::Draw.Scale();
		DrawStyledBackground(pDrawList, { vDrawPos.x + flInset, vDrawPos.y + flInset }, { vDrawPos.x - flInset + vWindowSize.x, vDrawPos.y - flInset + vWindowSize.y });

		// Horizontal accent line that fades out toward both ends.
		auto DrawFadeLineH = [&](float x1, float x2, float yTop, float h, ImU32 col)
			{
				const ImU32 colT = col & 0x00FFFFFFu; // transparent (same RGB, 0 alpha)
				const float xm = (x1 + x2) * 0.5f;
				pDrawList->AddRectFilledMultiColor({ x1, yTop }, { xm, yTop + h }, colT, col, col, colT);
				pDrawList->AddRectFilledMultiColor({ xm, yTop }, { x2, yTop + h }, col, colT, colT, col);
			};

		// Draw title centered (with optional logo to its left)
		float flTitleHeight = H::Draw.Scale(40);
		PushFont(F::Render.FontBold);
		PushStyleColor(ImGuiCol_Text, F::Render.Accent.Value);
		const char* sTitle = Vars::Menu::CheatTitle.Value.c_str();
		ImVec2 vTitleSize = CalcTextSize(sTitle);
		const float flLogoBase = H::Draw.Scale(22); // a touch bigger (was 20)
		IDirect3DTexture9* pLogo = F::Render.GetActiveLogoTex(); // selected built-in or custom logo
		// Bigger reserved slot + smaller drawn quad while spinning so the rotating logo never clips.
		const bool bSpinLogo = LogoSpinning();
		const float flLogoSlot = flLogoBase * (bSpinLogo ? kLogoSpinSlot : 1.f);
		const float flLogoDraw = flLogoBase * (bSpinLogo ? kLogoSpinDraw : 1.f);
		const float flLogoGap = pLogo ? H::Draw.Scale(5) : 0.f;
		const float flBlockW = vTitleSize.x + (pLogo ? flLogoSlot + flLogoGap : 0.f);
		const float flBlockX = (vWindowSize.x - flBlockW) / 2;
		const float flCenterY = H::Draw.Scale(11) - (flLogoSlot - flLogoBase) * 0.5f; // keep the logo centre fixed as the slot grows
		if (pLogo)
		{
			const ImVec2 vLogoMin(vDrawPos.x + flBlockX, vDrawPos.y + flCenterY);
			const ImVec2 vCenter(vLogoMin.x + flLogoSlot * 0.5f, vLogoMin.y + flLogoSlot * 0.5f);
			DrawLogoRotated(pDrawList, reinterpret_cast<ImTextureID>(pLogo), vCenter, flLogoDraw, LogoSpinAngle(), F::Render.Accent); // accent-tinted logo
		}
		SetCursorPos({ flBlockX + (pLogo ? flLogoSlot + flLogoGap : 0.f), flCenterY + (flLogoSlot - vTitleSize.y) * 0.5f });
		FText(sTitle);
		PopStyleColor();
		PopFont();

		// Draw horizontal line under title (fades out toward both ends)
		DrawFadeLineH(vDrawPos.x + H::Draw.Scale(10), vDrawPos.x + vWindowSize.x - H::Draw.Scale(10), vDrawPos.y + flTitleHeight, H::Draw.Scale(2), (ImU32)ImColor(F::Render.Accent));

		// BRIMGUI layout - main tabs sit at the BOTTOM of the window, subtabs under the title.
		static int iTab = 0, iAimbotTab = 0, iVisualsTab = 0, iMiscTab = 0, iMovementTab = 0, iConfigTab = 0;

		float flTabHeight = H::Draw.Scale(40);
		float flMainTabY = vWindowSize.y - flTabHeight - H::Draw.Scale(8); // bottom of the window

		// Derive per-tab width from the window so the 5 tabs always span the bar (x=20.. width-20) and never get cut off.
		float flMainTabW = (vWindowSize.x - H::Draw.Scale(40)) / 5.f;

		PushFont(F::Render.FontBold);
		SetCursorPos({ H::Draw.Scale(20), flMainTabY });
		FTabs(
			{
				{ "AIM" },
				{ "VISUALS" },
				{ "MISC" },
				{ "MOVEMENT" },
				{ "CONFIG" }
			},
			{ &iTab },
			{ flMainTabW, flTabHeight },
			{ H::Draw.Scale(20), flMainTabY },
			FTabsEnum::Horizontal | FTabsEnum::AlignCenter | FTabsEnum::BarTop, // bar on top of bottom tabs
			{},
			{}, {},
			{}, {},
			0.f, 2.f
		);
		PopFont();

		// Draw subtabs for current tab (directly under the title line)
		float flSubTabStartY = flTitleHeight + H::Draw.Scale(10);
		PushFont(F::Render.FontBold);
		SetCursorPos({ H::Draw.Scale(20), flSubTabStartY });
		
		switch (iTab)
		{
		case 0: // Aim
			FTabs(
				{ "GENERAL", "DRAW", "KNIFE" },
				&iAimbotTab,
				{ H::Draw.Scale(80), H::Draw.Scale(30) },
				{ H::Draw.Scale(20), flSubTabStartY },
				FTabsEnum::Horizontal | FTabsEnum::AlignCenter | FTabsEnum::BarBottom,
				{},
				{}, {},
				{}, {},
				0.f, 2.f
			);
			break;
		case 1: // Visuals
			FTabs(
				{ "ESP", "MISC##", "MENU" },
				&iVisualsTab,
				{ H::Draw.Scale(80), H::Draw.Scale(30) },
				{ H::Draw.Scale(20), flSubTabStartY },
				FTabsEnum::Horizontal | FTabsEnum::AlignCenter | FTabsEnum::BarBottom,
				{},
				{}, {},
				{}, {},
				0.f, 2.f
			);
			break;
		case 2: // Misc (single page - HvH removed)
			FTabs(
				{ "MAIN" },
				&iMiscTab, // Misc has a single page (MenuMisc always renders case 0)
				{ H::Draw.Scale(80), H::Draw.Scale(30) },
				{ H::Draw.Scale(20), flSubTabStartY },
				FTabsEnum::Horizontal | FTabsEnum::AlignCenter | FTabsEnum::BarBottom,
				{},
				{}, {},
				{}, {},
				0.f, 2.f
			);
			break;
		case 3: // Movement
			FTabs(
				{ "MOVEMENT##", "INDICATORS", "RECORDER", "FAKE POV" },
				&iMovementTab,
				{ H::Draw.Scale(90), H::Draw.Scale(30) },
				{ H::Draw.Scale(20), flSubTabStartY },
				FTabsEnum::Horizontal | FTabsEnum::AlignCenter | FTabsEnum::BarBottom,
				{},
				{}, {},
				{}, {},
				0.f, 2.f
			);
			break;
		case 4: // Config (old Movement pages + old Cfg pages)
			FTabs(
				{ "CONFIG##", "BINDS", "PLAYERS", "LOGGING", "OUTPUT", "MATERIALS", "EXTRA" },
				&iConfigTab,
				{ H::Draw.Scale(80), H::Draw.Scale(30) },
				{ H::Draw.Scale(20), flSubTabStartY },
				FTabsEnum::Horizontal | FTabsEnum::AlignCenter | FTabsEnum::BarBottom,
				{},
				{}, {},
				{}, {},
				0.f, 2.f
			);
			break;
		}
		PopFont();

		// Content area: between the subtabs (top) and the bottom main-tab bar.
		float flContentStartY = flSubTabStartY + H::Draw.Scale(40);
		float flContentHeight = flMainTabY - flContentStartY - H::Draw.Scale(8);
		SetCursorPos({ H::Draw.Scale(10), flContentStartY });
		PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0);
		PushStyleVar(ImGuiStyleVar_WindowPadding, { H::Draw.Scale(8), H::Draw.Scale(8) });
		if (BeginChild("Page", { vWindowSize.x - H::Draw.Scale(20), flContentHeight }, ImGuiChildFlags_AlwaysUseWindowPadding))
		{
			switch (iTab)
			{
			case 0: MenuAimbot(iAimbotTab); break;
			case 1: MenuVisuals(iVisualsTab); break;
			case 2: MenuMisc(0); break; // single page
			case 3: MenuMovement(iMovementTab); break;
			case 4: MenuConfig(iConfigTab); break;
			}
		} EndChild();
		PopStyleVar(2);

		End();
	}
	PopStyleVar();
}

#pragma region Tabs
void CMenu::MenuAimbot(int iTab)
{
	using namespace ImGui;

	switch (iTab)
	{
	// General
	case 0:
	{
		if (BeginTable("AimbotTable", 2))
		{
			/* Column 1 */
			TableNextColumn();
			{
				if (Section("General"))
				{
					FDropdown(Vars::Aimbot::General::AimType, FDropdownEnum::Left);
					FDropdown(Vars::Aimbot::General::TargetSelection, FDropdownEnum::Right);
					FDropdown(Vars::Aimbot::General::Target, FDropdownEnum::Left);
					FDropdown(Vars::Aimbot::General::Ignore, FDropdownEnum::Right);
					FSlider(Vars::Aimbot::General::AimFOV);
					FSlider(Vars::Aimbot::General::MaxTargets, FSliderEnum::Left);
					PushTransparent(!(Vars::Aimbot::General::Ignore.Value & Vars::Aimbot::General::IgnoreEnum::Invisible));
					{
						FSlider(Vars::Aimbot::General::IgnoreInvisible, FSliderEnum::Right);
					}
					PopTransparent();
					FSlider(Vars::Aimbot::General::AssistStrength, FSliderEnum::Left);
					PushTransparent(!(Vars::Aimbot::General::Ignore.Value & Vars::Aimbot::General::IgnoreEnum::Unsimulated));
					{
						FSlider(Vars::Aimbot::General::TickTolerance, FSliderEnum::Right);
					}
					PopTransparent();
					FColorPicker(Vars::Colors::FOVCircle);
					FToggle(Vars::Aimbot::General::AutoShoot, FToggleEnum::Left);
					FToggle(Vars::Aimbot::General::FOVCircle, FToggleEnum::Right);
					FToggle(Vars::CritHack::ForceCrits, FToggleEnum::Left);
					FToggle(Vars::CritHack::AvoidRandomCrits, FToggleEnum::Right);
					FToggle(Vars::CritHack::AlwaysMeleeCrit, FToggleEnum::Left);
					FToggle(Vars::Aimbot::General::NoSpread, FToggleEnum::Right);
				} EndSection();
				if (Vars::Debug::Options.Value)
				{
					if (Section("##Debug Aimbot", -8))
					{
						FDropdown(Vars::Aimbot::General::AimHoldsFire);
						FSlider(Vars::Aimbot::General::NoSpreadOffset);
						FSlider(Vars::Aimbot::General::NoSpreadAverage);
						FSlider(Vars::Aimbot::General::NoSpreadInterval);
						FSlider(Vars::Aimbot::General::NoSpreadBackupInterval);
					} EndSection();
				}
				if (Section("Backtrack", 8))
				{
					FSlider(Vars::Backtrack::Latency);
					FSlider(Vars::Backtrack::Interp);
					FSlider(Vars::Backtrack::Window);
					//FToggle(Vars::Backtrack::PreferOnShot);
				} EndSection();
				if (Vars::Debug::Options.Value)
				{
					if (Section("##Debug Backtrack"))
					{
						FSlider(Vars::Backtrack::Offset);
					} EndSection();
				}
				if (Section("Healing"))
				{
					FDropdown(Vars::Aimbot::Healing::HealPriority);
					FToggle(Vars::Aimbot::Healing::AutoHeal, FToggleEnum::Left);
					FToggle(Vars::Aimbot::Healing::AutoArrow, FToggleEnum::Right);
					FToggle(Vars::Aimbot::Healing::AutoRepair, FToggleEnum::Left);
					FToggle(Vars::Aimbot::Healing::AutoSandvich, FToggleEnum::Right);
					FToggle(Vars::Aimbot::Healing::AutoVaccinator, FToggleEnum::Left);
					FToggle(Vars::Aimbot::Healing::ActivateOnVoice, FToggleEnum::Right);
				} EndSection();
				if (Vars::Debug::Options.Value)
				{
					if (Section("##Debug Healing"))
					{
						FSlider(Vars::Aimbot::Healing::AutoVaccinatorBulletScale);
						FSlider(Vars::Aimbot::Healing::AutoVaccinatorBlastScale);
						FSlider(Vars::Aimbot::Healing::AutoVaccinatorFireScale);
						FToggle(Vars::Aimbot::Healing::AutoVaccinatorFlamethrowerDamageOnly);
					} EndSection();
				}
			}
			/* Column 2 */
			TableNextColumn();
			{
				if (Section("Hitscan"))
				{
					FDropdown(Vars::Aimbot::Hitscan::Hitboxes, FDropdownEnum::Left);
					FDropdown(Vars::Aimbot::Hitscan::MultipointHitboxes, FDropdownEnum::Right);
					FDropdown(Vars::Aimbot::Hitscan::Modifiers);
					FSlider(Vars::Aimbot::Hitscan::MultipointScale);
					PushTransparent(!(Vars::Aimbot::Hitscan::Modifiers.Value & Vars::Aimbot::Hitscan::ModifiersEnum::Tapfire));
					{
						FSlider(Vars::Aimbot::Hitscan::TapfireDistance);
					}
					PopTransparent();
				} EndSection();
				if (Vars::Debug::Options.Value)
				{
					if (Section("##Debug Hitscan", -8))
					{
						FDropdown(Vars::Aimbot::Hitscan::PeekCheck, FDropdownEnum::None, 0, &Hovered); FTooltip("This should stay as doubletap only or off if you want to be able to target hitboxes other than the highest priority one", Hovered);
						FSlider(Vars::Aimbot::Hitscan::PeekAmount);
						FSlider(Vars::Aimbot::Hitscan::BoneSizeSubtract);
						FSlider(Vars::Aimbot::Hitscan::BoneSizeMinimumScale);
					} EndSection();
				}
				if (Section("Projectile"))
				{
					FDropdown(Vars::Aimbot::Projectile::StrafePrediction, FDropdownEnum::Left);
					FDropdown(Vars::Aimbot::Projectile::SplashPrediction, FDropdownEnum::Right);
					FDropdown(Vars::Aimbot::Projectile::AutoDetonate, FDropdownEnum::Left);
					FDropdown(Vars::Aimbot::Projectile::AutoAirblast, FDropdownEnum::Right);
					FDropdown(Vars::Aimbot::Projectile::Hitboxes, FDropdownEnum::Left);
					FDropdown(Vars::Aimbot::Projectile::Modifiers, FDropdownEnum::Right);
					FSlider(Vars::Aimbot::Projectile::MaxSimulationTime, FSliderEnum::Left);
					PushTransparent(!Vars::Aimbot::Projectile::StrafePrediction.Value);
					{
						FSlider(Vars::Aimbot::Projectile::HitChance, FSliderEnum::Right);
					}
					PopTransparent();
					FSlider(Vars::Aimbot::Projectile::AutodetRadius, FSliderEnum::Left);
					FSlider(Vars::Aimbot::Projectile::SplashRadius, FSliderEnum::Right);
					PushTransparent(!Vars::Aimbot::Projectile::AutoRelease.Value);
					{
						FSlider(Vars::Aimbot::Projectile::AutoRelease);
					}
					PopTransparent();
				} EndSection();
				if (Vars::Debug::Options.Value)
				{
					if (Section("##Debug Projectile"))
					{
						FText("Ground");
						FSlider(Vars::Aimbot::Projectile::GroundSamples, FSliderEnum::Left);
						FSlider(Vars::Aimbot::Projectile::GroundStraightFuzzyValue, FSliderEnum::Right);
						FSlider(Vars::Aimbot::Projectile::GroundLowMinimumSamples, FSliderEnum::Left);
						FSlider(Vars::Aimbot::Projectile::GroundHighMinimumSamples, FSliderEnum::Right);
						FSlider(Vars::Aimbot::Projectile::GroundLowMinimumDistance, FSliderEnum::Left);
						FSlider(Vars::Aimbot::Projectile::GroundHighMinimumDistance, FSliderEnum::Right);
						FSlider(Vars::Aimbot::Projectile::GroundMaxChanges, FSliderEnum::Left);
						FSlider(Vars::Aimbot::Projectile::GroundMaxChangeTime, FSliderEnum::Right);

						FText("\nAir");
						FSlider(Vars::Aimbot::Projectile::AirSamples, FSliderEnum::Left);
						FSlider(Vars::Aimbot::Projectile::AirStraightFuzzyValue, FSliderEnum::Right);
						FSlider(Vars::Aimbot::Projectile::AirLowMinimumSamples, FSliderEnum::Left);
						FSlider(Vars::Aimbot::Projectile::AirHighMinimumSamples, FSliderEnum::Right);
						FSlider(Vars::Aimbot::Projectile::AirLowMinimumDistance, FSliderEnum::Left);
						FSlider(Vars::Aimbot::Projectile::AirHighMinimumDistance, FSliderEnum::Right);
						FSlider(Vars::Aimbot::Projectile::AirMaxChanges, FSliderEnum::Left);
						FSlider(Vars::Aimbot::Projectile::AirMaxChangeTime, FSliderEnum::Right);

						FText("");
						FSlider(Vars::Aimbot::Projectile::VelocityAverageCount, FSliderEnum::Left);
						FSlider(Vars::Aimbot::Projectile::VerticalShift, FSliderEnum::Right);

						FSlider(Vars::Aimbot::Projectile::DragOverride, FSliderEnum::Left);
						FSlider(Vars::Aimbot::Projectile::TimeOverride, FSliderEnum::Right);
						FSlider(Vars::Aimbot::Projectile::HuntsmanLerp, FSliderEnum::Left);
						FSlider(Vars::Aimbot::Projectile::HuntsmanLerpLow, FSliderEnum::Right);
						FSlider(Vars::Aimbot::Projectile::HuntsmanAdd, FSliderEnum::Left);
						FSlider(Vars::Aimbot::Projectile::HuntsmanAddLow, FSliderEnum::Right);
						FSlider(Vars::Aimbot::Projectile::HuntsmanClamp, FSliderEnum::Left);
						FToggle(Vars::Aimbot::Projectile::HuntsmanPullPoint, FToggleEnum::Right);
						SetCursorPosY(GetCursorPosY() + 8);

						FSlider(Vars::Aimbot::Projectile::SplashPointsDirect, FSliderEnum::Left);
						FSlider(Vars::Aimbot::Projectile::SplashPointsArc, FSliderEnum::Right);
						FSlider(Vars::Aimbot::Projectile::SplashCountDirect, FSliderEnum::Left);
						FSlider(Vars::Aimbot::Projectile::SplashCountArc, FSliderEnum::Right);
						FSlider(Vars::Aimbot::Projectile::SplashRotateX, FSliderEnum::Left, Vars::Aimbot::Projectile::SplashRotateX[DEFAULT_BIND] < 0.f ? "random" : "%g");
						FSlider(Vars::Aimbot::Projectile::SplashRotateY, FSliderEnum::Right, Vars::Aimbot::Projectile::SplashRotateY[DEFAULT_BIND] < 0.f ? "random" : "%g");
						FSlider(Vars::Aimbot::Projectile::SplashTraceInterval, FSliderEnum::Left);
						FSlider(Vars::Aimbot::Projectile::SplashNormalSkip, FSliderEnum::Right);
						FDropdown(Vars::Aimbot::Projectile::SplashMode, FDropdownEnum::Left);
						FDropdown(Vars::Aimbot::Projectile::RocketSplashMode, FDropdownEnum::Right, 0, &Hovered); FTooltip("Special splash type for rockets, more expensive", Hovered);
						FToggle(Vars::Aimbot::Projectile::SplashGrates);
						SetCursorPosY(GetCursorPosY() + 8);

						FSlider(Vars::Aimbot::Projectile::DeltaCount, FSliderEnum::Left);
						FDropdown(Vars::Aimbot::Projectile::DeltaMode, FDropdownEnum::Right);
						FDropdown(Vars::Aimbot::Projectile::MovesimFrictionFlags);
					} EndSection();
				}
				// Auto backstab and all backstab-specific options live only in the KNIFE subtab now.
				if (Section("Melee", 8))
				{
					FToggle(Vars::Aimbot::Melee::SwingPrediction, FToggleEnum::Left);
					FToggle(Vars::Aimbot::Melee::WhipTeam, FToggleEnum::Right);
				} EndSection();
				if (Vars::Debug::Options.Value)
				{
					if (Section("##Debug Melee"))
					{
						FSlider(Vars::Aimbot::Melee::SwingOffset, FSliderEnum::Left);
						FToggle(Vars::Aimbot::Melee::SwingPredictLag, FToggleEnum::Right);
					} EndSection();
				}
			}
			EndTable();
		}
		break;
	}
	// Draw
	case 1:
	{
		if (BeginTable("DrawTable", 2))
		{
			/* Column 1 */
			TableNextColumn();
			{
				if (Section("Line", 8))
				{
					FColorPicker(Vars::Colors::Line, FColorPickerEnum::None, { -H::Draw.Scale(12), 0 });
					FColorPicker(Vars::Colors::LineIgnoreZ);
					FToggle(Vars::Visuals::Line::Enabled);
					FSlider(Vars::Visuals::Line::DrawDuration);
				} EndSection();
				if (Section("Hitbox"))
				{
					FDropdown(Vars::Visuals::Hitbox::BonesEnabled, FDropdownEnum::None, -50);
					FColorPicker(Vars::Colors::BoneHitboxEdge, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(10), H::Draw.Scale(20) });
					FColorPicker(Vars::Colors::BoneHitboxEdgeIgnoreZ, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(10), H::Draw.Scale(20) });
					FColorPicker(Vars::Colors::BoneHitboxFace, FColorPickerEnum::SameLine, { H::Draw.Scale(10), 0 }, { H::Draw.Scale(10), H::Draw.Scale(20) });
					FColorPicker(Vars::Colors::BoneHitboxFaceIgnoreZ, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(10), H::Draw.Scale(20) });
					FColorPicker(Vars::Colors::TargetHitboxEdge, FColorPickerEnum::SameLine, { -H::Draw.Scale(50), H::Draw.Scale(20) }, { H::Draw.Scale(10), H::Draw.Scale(20) });
					FColorPicker(Vars::Colors::TargetHitboxEdgeIgnoreZ, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(10), H::Draw.Scale(20) });
					FColorPicker(Vars::Colors::TargetHitboxFace, FColorPickerEnum::SameLine, { H::Draw.Scale(10), 0 }, { H::Draw.Scale(10), H::Draw.Scale(20) });
					FColorPicker(Vars::Colors::TargetHitboxFaceIgnoreZ, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(10), H::Draw.Scale(20) });

					FDropdown(Vars::Visuals::Hitbox::BoundsEnabled, FDropdownEnum::None, -50);
					FColorPicker(Vars::Colors::BoundHitboxEdge, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(10), H::Draw.Scale(40) });
					FColorPicker(Vars::Colors::BoundHitboxEdgeIgnoreZ, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(10), H::Draw.Scale(40) });
					FColorPicker(Vars::Colors::BoundHitboxFace, FColorPickerEnum::SameLine, { H::Draw.Scale(10), 0 }, { H::Draw.Scale(10), H::Draw.Scale(40) });
					FColorPicker(Vars::Colors::BoundHitboxFaceIgnoreZ, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(10), H::Draw.Scale(40) });

					FSlider(Vars::Visuals::Hitbox::DrawDuration);
				} EndSection();
			}
			/* Column 2 */
			TableNextColumn();
			{
				if (Section("Simulation"))
				{
					FDropdown(Vars::Visuals::Simulation::PlayerPath, FDropdownEnum::Left, -20);
					FColorPicker(Vars::Colors::PlayerPath, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(10), H::Draw.Scale(40) });
					FColorPicker(Vars::Colors::PlayerPathIgnoreZ, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(10), H::Draw.Scale(40) });
					FDropdown(Vars::Visuals::Simulation::ProjectilePath, FDropdownEnum::Right, -20);
					FColorPicker(Vars::Colors::ProjectilePath, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(10), H::Draw.Scale(40) });
					FColorPicker(Vars::Colors::ProjectilePathIgnoreZ, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(10), H::Draw.Scale(40) });
					FDropdown(Vars::Visuals::Simulation::TrajectoryPath, FDropdownEnum::Left, -20);
					FColorPicker(Vars::Colors::TrajectoryPath, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(10), H::Draw.Scale(40) });
					FColorPicker(Vars::Colors::TrajectoryPathIgnoreZ, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(10), H::Draw.Scale(40) });
					FDropdown(Vars::Visuals::Simulation::ShotPath, FDropdownEnum::Right, -20);
					FColorPicker(Vars::Colors::ShotPath, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(10), H::Draw.Scale(40) });
					FColorPicker(Vars::Colors::ShotPathIgnoreZ, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(10), H::Draw.Scale(40) });
					FDropdown(Vars::Visuals::Simulation::SplashRadius, FDropdownEnum::None, -20);
					FColorPicker(Vars::Colors::SplashRadius, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(10), H::Draw.Scale(40) });
					FColorPicker(Vars::Colors::SplashRadiusIgnoreZ, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(10), H::Draw.Scale(40) });
					FToggle(Vars::Visuals::Simulation::Timed, FToggleEnum::Left);
					FToggle(Vars::Visuals::Simulation::Box, FToggleEnum::Right);
					FToggle(Vars::Visuals::Simulation::ProjectileCamera, FToggleEnum::Left);
					FToggle(Vars::Visuals::Simulation::SwingLines, FToggleEnum::Right);
					FSlider(Vars::Visuals::Simulation::DrawDuration);
				} EndSection();
				if (Vars::Debug::Options.Value)
				{
					if (Section("##Debug Part1", -8))
					{
						FDropdown(Vars::Visuals::Simulation::RealPath, FDropdownEnum::None, -20);
						FColorPicker(Vars::Colors::RealPath, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(10), H::Draw.Scale(40) });
						FColorPicker(Vars::Colors::RealPathIgnoreZ, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(10), H::Draw.Scale(40) });

						FSlider(Vars::Visuals::Simulation::SeparatorSpacing, FSliderEnum::Left);
						FSlider(Vars::Visuals::Simulation::SeparatorLength, FSliderEnum::Right);
					} EndSection();
					if (Section("##Debug Part2"))
					{
						FToggle(Vars::Visuals::Trajectory::Override);
						FSlider(Vars::Visuals::Trajectory::OffsetX);
						FSlider(Vars::Visuals::Trajectory::OffsetY);
						FSlider(Vars::Visuals::Trajectory::OffsetZ);
						FToggle(Vars::Visuals::Trajectory::Pipes);
						FSlider(Vars::Visuals::Trajectory::Hull);
						FSlider(Vars::Visuals::Trajectory::Speed);
						FSlider(Vars::Visuals::Trajectory::Gravity);
						FSlider(Vars::Visuals::Trajectory::LifeTime);
						FSlider(Vars::Visuals::Trajectory::UpVelocity);
						FSlider(Vars::Visuals::Trajectory::AngularVelocityX);
						FSlider(Vars::Visuals::Trajectory::AngularVelocityY);
						FSlider(Vars::Visuals::Trajectory::AngularVelocityZ);
						FSlider(Vars::Visuals::Trajectory::Drag);
						FSlider(Vars::Visuals::Trajectory::DragX);
						FSlider(Vars::Visuals::Trajectory::DragY);
						FSlider(Vars::Visuals::Trajectory::DragZ);
						FSlider(Vars::Visuals::Trajectory::AngularDragX);
						FSlider(Vars::Visuals::Trajectory::AngularDragY);
						FSlider(Vars::Visuals::Trajectory::AngularDragZ);
						FSlider(Vars::Visuals::Trajectory::MaxVelocity);
						FSlider(Vars::Visuals::Trajectory::MaxAngularVelocity);
					} EndSection();
				}
			}
			EndTable();
		}
		break;
	}
	// Knife
	case 2:
	{
		if (BeginTable("KnifeTable", 2))
		{
			TableNextColumn();
			{
				if (Section("Backstab"))
				{
					FToggle(Vars::Aimbot::Melee::AutoBackstab);
					FDropdown(Vars::Aimbot::Melee::BackstabAimType, FDropdownEnum::Left);
					// FOV slider only applies to Silent; Legit is fixed at 180 and never moves the view.
					if (Vars::Aimbot::Melee::BackstabAimType.Value == Vars::Aimbot::Melee::BackstabAimTypeEnum::Silent)
						FSlider(Vars::Aimbot::Melee::BackstabFOV, FSliderEnum::Right);
					FToggle(Vars::Aimbot::Melee::BackstabAutoShoot, FToggleEnum::Left);
					// Backstab-only ignore list (invulnerable, invisible, dead ringer, etc.), separate
					// from the General aim Ignore. Invisible threshold shares the General slider.
					FDropdown(Vars::Aimbot::Melee::BackstabIgnore);
					PushTransparent(!(Vars::Aimbot::Melee::BackstabIgnore.Value & Vars::Aimbot::Melee::BackstabIgnoreEnum::Invisible));
					{
						FSlider(Vars::Aimbot::General::IgnoreInvisible);
					}
					PopTransparent();
				} EndSection();
			}
			TableNextColumn();
			{
				if (Section("Reliability", 8))
				{
					FToggle(Vars::Aimbot::Melee::BackstabAccountPing, FToggleEnum::Left);
					FToggle(Vars::Aimbot::Melee::BackstabDoubleTest, FToggleEnum::Right);
					// Timing: predict one move-tick so the stab fires the exact tick it becomes valid;
					// strict range trades a hair of speed for fewer "chasing" whiffs into the air.
					FToggle(Vars::Aimbot::Melee::BackstabPredictMovement, FToggleEnum::Left);
					FToggle(Vars::Aimbot::Melee::BackstabStrictRange, FToggleEnum::Right);
					FToggle(Vars::Aimbot::Melee::BackstabRemoveInterp, FToggleEnum::Left);
					// Greedy: swing whenever the back is exposed + in range + clear line, even if our own
					// trace didn't confirm. Catches backstabs our (slightly stricter) trace would miss.
					FToggle(Vars::Aimbot::Melee::BackstabAggressive, FToggleEnum::Right);
					// Finer aim-point search: also test points between the centre axis and the nearest box
					// point, so awkward height/side stabs that only line up in between still fire.
					FToggle(Vars::Aimbot::Melee::BackstabMultiPoint, FToggleEnum::Left);
					// Rear-arc extrapolation: test the whole current..ping-extrapolated yaw span and stab if
					// any of it exposes the back - the principled side/rear-stab fix. Non-turning target = no
					FToggle(Vars::Aimbot::Melee::BackstabExtrapolateYaw, FToggleEnum::Right);
					// Bad-net help: widen the behind test into an OR-spread of plausible (stale) yaws and try
					// the live position too, both scaled by measured ping/loss. LAN = unchanged. Max-angle caps
					FToggle(Vars::Aimbot::Melee::BackstabUnstableConnection, FToggleEnum::Left);
					PushTransparent(!Vars::Aimbot::Melee::BackstabUnstableConnection.Value);
					{
						FSlider(Vars::Aimbot::Melee::BackstabUnstableMaxAngle, FSliderEnum::Right);
					}
					PopTransparent();
				} EndSection();
				if (Vars::Debug::Options.Value)
				{
					if (Section("##Debug Knife"))
					{
						FToggle(Vars::Aimbot::Melee::SwingPredictLag);
						FSlider(Vars::Aimbot::Melee::SwingOffset);
					} EndSection();
				}
			}
			EndTable();
		}
		break;
	}
	}
}

// Renders one ESP section as an always-visible flat 2-column list of option rows (the reference /
// dna / the reference style). Every recolorable element shows a color swatch next to it when enabled;
static void RenderESPSection(int iSection)
{
	using namespace ImGui;
	if (iSection < 0 || iSection >= SectionEnum::Count)
		return;

	auto& tGroup = F::Groups.m_vGroups[iSection];
	auto LB = [iSection](const char* s) { return std::format("{}##sec{}", s, iSection); }; // unique per-section label

	static const char* sNames[SectionEnum::Count] =
	{ "Enemies", "Enemy buildings", "Teammates", "Teammate buildings", "Viewmodel weapon", "Viewmodel arms", "Projectiles", "Pickups", "Intel" };

	const bool bPlayer = iSection == SectionEnum::Enemies || iSection == SectionEnum::Teammates;
	const bool bBuilding = iSection == SectionEnum::EnemyBuildings || iSection == SectionEnum::TeammateBuildings;
	const bool bViewmodel = iSection == SectionEnum::ViewmodelWeapon || iSection == SectionEnum::ViewmodelArms;
	const bool bProjectile = iSection == SectionEnum::Projectiles;
	const bool bPickup = iSection == SectionEnum::Pickups;
	const bool bIntel = iSection == SectionEnum::Intel;

	if (Section(sNames[iSection]))
	{
		FToggle(LB("Enable").c_str(), &tGroup.m_bEnabled, FToggleEnum::Left);
		if (!tGroup.m_bEnabled) { EndSection(); return; } // hide options while disabled

		// 2-column element rows: toggles alternate left/right, color swatch beside each enabled toggle.
		int iCol = 0;
		auto Element = [&](const char* sLabel, int iBit, bool bRecolor = true)
		{
			int iSide = (iCol++ % 2 == 0) ? FToggleEnum::Left : FToggleEnum::Right;
			FToggle(LB(sLabel).c_str(), &tGroup.m_iESP, iBit, iSide);
			if (bRecolor && tGroup.m_iESP & iBit)
				FColorPicker(std::format("##c{}_{}", iSection, iBit).c_str(), &tGroup.ElementColor(iBit), FColorPickerEnum::SameLine | FColorPickerEnum::NoTooltip, {}, { H::Draw.Scale(11), H::Draw.Scale(11) });
		};

		if (bPlayer)
		{
			Element("Name", ESPEnum::Name);
			Element("Box", ESPEnum::Box);
			Element("Skeleton", ESPEnum::Bones);
			Element("Health bar", ESPEnum::HealthBar);
			Element("Health text", ESPEnum::HealthText);
			Element("Ubercharge bar", ESPEnum::UberBar);
			Element("Ubercharge text", ESPEnum::UberText);
			Element("Class text", ESPEnum::ClassText);
			Element("Weapon icon", ESPEnum::WeaponIcon);
			Element("Weapon text", ESPEnum::WeaponText);
			Element("Class icon", ESPEnum::ClassIcon, false);
			Element("Flags", ESPEnum::Labels, false);
			Element("Avatar", ESPEnum::Avatar, false);
		}
		else if (bBuilding)
		{
			Element("Name", ESPEnum::Name);
			Element("Box", ESPEnum::Box);
			Element("Health bar", ESPEnum::HealthBar);
			Element("Health text", ESPEnum::HealthText);
			Element("Owner", ESPEnum::Owner);
			Element("Level", ESPEnum::Level);
			Element("Ammo bars", ESPEnum::AmmoBars);
			Element("Ammo text", ESPEnum::AmmoText);
		}
		else if (bProjectile)
		{
			Element("Name", ESPEnum::Name);
			Element("Box", ESPEnum::Box);
			Element("Owner", ESPEnum::Owner);
		}
		else if (bPickup)
		{
			Element("Name", ESPEnum::Name);
			Element("Box", ESPEnum::Box);
		}
		else if (bIntel)
		{
			Element("Name", ESPEnum::Name);
			Element("Box", ESPEnum::Box);
			Element("Return time", ESPEnum::IntelReturnTime);
		}

		// optional black outline ring on the avatar circle
		if (bPlayer && tGroup.m_iESP & ESPEnum::Avatar)
			FToggle(LB("Avatar outline").c_str(), &tGroup.m_bAvatarOutline, FToggleEnum::Left);

		// skeleton line thickness — global Vars::ESP::SkeletonThickness; PushID(iSection) keeps the
		// widget id unique across the two player sections that share this global.
		if (bPlayer && tGroup.m_iESP & ESPEnum::Bones)
		{
			PushID(iSection);
			FSlider(Vars::ESP::SkeletonThickness, FSliderEnum::Left);
			PopID();
		}

		// per-group box style (shown when this section draws a box)
		if (!bViewmodel && tGroup.m_iESP & ESPEnum::Box)
		{
			FToggle(LB("Corner box").c_str(), &tGroup.m_bCornerBox, FToggleEnum::Left);
			FToggle(LB("Box outline").c_str(), &tGroup.m_bBoxOutline, FToggleEnum::Right);
			if (tGroup.m_bCornerBox)
				FSlider(LB("Corner length").c_str(), &tGroup.m_flCornerLength, 0.1f, 0.5f, 0.05f, "%.2f", FSliderEnum::Left | FSliderEnum::Precision);
		}
		// per-group health-bar style
		if (!bViewmodel && tGroup.m_iESP & ESPEnum::HealthBar)
		{
			FToggle(LB("Health bar outline").c_str(), &tGroup.m_bHealthBarOutline, FToggleEnum::Left);
			FToggle(LB("Health bar background").c_str(), &tGroup.m_bHealthBarBackground, FToggleEnum::Right);
			if (tGroup.m_bHealthBarBackground)
				FColorPicker(std::format("##hpbg{}", iSection).c_str(), &tGroup.m_tHealthBarBackgroundColor, FColorPickerEnum::SameLine | FColorPickerEnum::NoTooltip, {}, { H::Draw.Scale(11), H::Draw.Scale(11) });
			// width + position share a row; overheal color sits beside the position dropdown.
			FSlider(LB("Health bar width").c_str(), &tGroup.m_flHealthBarWidth, 1.f, 8.f, 0.5f, "%g", FSliderEnum::Left | FSliderEnum::Precision);
			FDropdown(LB("Health bar position").c_str(), &tGroup.m_iHealthBarPosition, { "Left", "Right", "Above", "Below" }, {}, FDropdownEnum::Right);
			// buildings can't be overhealed -> no overheal color
			if (!bBuilding)
				FColorPicker(LB("Overheal color").c_str(), &tGroup.m_tHealthBarOverhealColor, FColorPickerEnum::Left);
		}

		// per-group name position
		if ((bPlayer || bBuilding) && tGroup.m_iESP & ESPEnum::Name)
			FDropdown(LB("Name position").c_str(), &tGroup.m_iNamePosition, { "Above", "Below" }, {}, FDropdownEnum::Left);

		// per-group class icon scale/position + simple (flat SVG silhouette) icon
		if (bPlayer && tGroup.m_iESP & ESPEnum::ClassIcon)
		{
			FSlider(LB("Class icon scale").c_str(), &tGroup.m_flClassIconScale, 0.5f, 3.f, 0.1f, "%.1f", FSliderEnum::Left | FSliderEnum::Precision);
			FDropdown(LB("Class icon position").c_str(), &tGroup.m_iClassIconPosition, { "Above", "Below", "Left", "Right" }, {}, FDropdownEnum::Right);
			FToggle(LB("Scale with distance").c_str(), &tGroup.m_bClassIconDistanceScale, FToggleEnum::Left);
			FToggle(LB("Simple class icon").c_str(), &tGroup.m_bSimpleClassIcon, FToggleEnum::Right);
			if (tGroup.m_bSimpleClassIcon)
			{
				FColorPicker(std::format("##simpleci{}", iSection).c_str(), &tGroup.m_tSimpleClassIconColor, FColorPickerEnum::SameLine | FColorPickerEnum::NoTooltip, {}, { H::Draw.Scale(11), H::Draw.Scale(11) });
				FToggle(LB("Icon outline").c_str(), &tGroup.m_bSimpleClassIconOutline, FToggleEnum::Left);
			}
		}

		// "Flags" status labels — which effects show + their size/position. These are global (one shared
		// FONT_ESP_LABEL); PushID(iSection) keeps the widget id unique across the two player sections.
		if (bPlayer && tGroup.m_iESP & ESPEnum::Labels)
		{
			PushID(iSection);
			FDropdown(Vars::ESP::FlagEffects, FDropdownEnum::Left);
			FDropdown(Vars::ESP::FlagPosition, FDropdownEnum::Right);
			FSlider(Vars::ESP::FlagScale, FSliderEnum::Left);
			PopID();
		}

		// Chams
		if (bViewmodel)
			FMDropdown(LB("Material").c_str(), &tGroup.m_tChams.Visible);
		else
		{
			FMDropdown(LB("Visible material").c_str(), &tGroup.m_tChams.Visible, FDropdownEnum::Left);
			FMDropdown(LB("Occluded material").c_str(), &tGroup.m_tChams.Occluded, FDropdownEnum::Right);
		}

		// Glow (with its own color). Player/building sections can split visible vs occluded colors.
		if (bPlayer || bBuilding)
		{
			FColorPicker(LB("Visible glow").c_str(), &tGroup.m_tColor, FColorPickerEnum::Left);
			FToggle(LB("Occluded color").c_str(), &tGroup.m_bGlowVisCheck, FToggleEnum::Right);
			if (tGroup.m_bGlowVisCheck)
				FColorPicker(LB("Invisible glow").c_str(), &tGroup.m_tGlowInvisibleColor, FColorPickerEnum::Left);
		}
		else
			FColorPicker(LB("Glow color").c_str(), &tGroup.m_tColor, FColorPickerEnum::Left);
		FSlider(LB("Stencil scale").c_str(), &tGroup.m_tGlow.Stencil, 0, 10, 1, "%i", FSliderEnum::Left | FSliderEnum::Min);
		FSlider(LB("Blur scale").c_str(), &tGroup.m_tGlow.Blur, 0.f, 10.f, 1.f, "%g", FSliderEnum::Right | FSliderEnum::Min | FSliderEnum::Precision);

		// extra per-section toggles
		if (bPlayer)
		{
			FToggle(LB("Backtrack").c_str(), &tGroup.m_bBacktrack, FToggleEnum::Left);
			SameLine(GetWindowWidth() - H::Draw.Scale(33));
			if (IconButton(ICON_MD_KEYBOARD_ARROW_DOWN))
				OpenPopup(std::format("Backtrack##{}", iSection).c_str());
		}
		if (bPlayer || bBuilding || bProjectile || bPickup)
		{
			FToggle(LB("Offscreen arrows").c_str(), &tGroup.m_bOffscreenArrows, FToggleEnum::Left);
			SameLine(GetWindowWidth() - H::Draw.Scale(33));
			if (IconButton(ICON_MD_KEYBOARD_ARROW_DOWN))
				OpenPopup(std::format("OffscreenArrows##{}", iSection).c_str());
		}
		if (bPickup)
			FToggle(LB("Pickup timer").c_str(), &tGroup.m_bPickupTimer, FToggleEnum::Left);

		SetNextWindowSize({ H::Draw.Scale(300), 0 });
		if (FBeginPopup(std::format("Backtrack##{}", iSection).c_str()))
		{
			SetCursorPosY(GetCursorPosY() - H::Draw.Scale(8));
			FDropdown(LB("##Draw").c_str(), &tGroup.m_iBacktrackDraw, { "Last", "First", "##Divider", "Always" }, {}, FDropdownEnum::Multi, 0, "All");
			FMDropdown(LB("Material##bt").c_str(), &tGroup.m_vBacktrackChams, FDropdownEnum::Left);
			SetCursorPos({ GetWindowWidth() / 2 + GetStyle().WindowPadding.x / 2, GetCursorPosY() - H::Draw.Scale(32) });
			FToggle(LB("Ignore Z##bt").c_str(), &tGroup.m_iBacktrackDraw, BacktrackEnum::IgnoreZ, FToggleEnum::Left);
			SetCursorPosY(GetCursorPosY() + H::Draw.Scale(8));
			FSlider(LB("Stencil scale##bt").c_str(), &tGroup.m_tBacktrackGlow.Stencil, 0, 10, 1, "%i", FSliderEnum::Left | FSliderEnum::Min);
			FSlider(LB("Blur scale##bt").c_str(), &tGroup.m_tBacktrackGlow.Blur, 0.f, 10.f, 1.f, "%g", FSliderEnum::Right | FSliderEnum::Min | FSliderEnum::Precision);
			EndPopup();
		}
		SetNextWindowSize({ H::Draw.Scale(300), 0 });
		if (FBeginPopup(std::format("OffscreenArrows##{}", iSection).c_str()))
		{
			FSlider(LB("Offset").c_str(), &tGroup.m_iOffscreenArrowsOffset, 0, 1000, 25, "%i", FSliderEnum::Precision);
			FSlider(LB("Max distance").c_str(), &tGroup.m_flOffscreenArrowsMaxDistance, 0.f, 5000.f, 50.f, "%g", FSliderEnum::Min | FSliderEnum::Precision);
			EndPopup();
		}
	} EndSection();
}

// Floating window that frames the 3D heavy model rendered (in the VGui paint pass) for the ESP
// preview. It shows what your player ESP/chams/glow look like in-game on a real model. The window
static void RenderModelPreviewWindow()
{
	using namespace ImGui;
	if (!Vars::Visuals::ModelPreview::Enabled.Value)
		return;

	// Match the main menu's height; width = the rendered heavy's width + 80px padding (40 each side).
	// Both size and position track the main menu rect (captured in CMenu::Draw) so the panels stay paired.
	const float flScreenH = float(H::Draw.m_nScreenH > 0 ? H::Draw.m_nScreenH : int(GetIO().DisplaySize.y));

	// Latch the last-good main-menu rect. Ingame a frame can report a stale/zero size; without this the
	// preview height fell back to flScreenH*0.55 on those frames, so the panel flickered between two sizes.
	static ImVec2 s_vMenuPos = {}, s_vMenuSize = {};
	if (g_vMainMenuSize.y > 1.f) { s_vMenuPos = g_vMainMenuPos; s_vMenuSize = g_vMainMenuSize; }
	const bool bHaveMenu = s_vMenuSize.y > 1.f;

	// Match the menu's VISIBLE bg, which DrawStyledBackground insets by flInset (see CMenu::DrawMenu), so the
	// two panels are exactly the same height instead of the preview sitting a couple px taller.
	const float flInset = H::Draw.Scale();
	const float flWinH = bHaveMenu ? s_vMenuSize.y - flInset * 2.f : flScreenH * 0.55f;
	const float flHeaderForFit = H::Draw.Scale(40);      // keep in sync with flHeaderH below
	const float flFooterForFit = H::Draw.Scale(40 + 8);  // bottom buttons + pad (keep in sync with footer below)
	const float flWinW = F::ModelPreview.GetModelDrawWidth(flWinH - flHeaderForFit - flFooterForFit) + 110.f;

	// CMenu::Draw pushes WindowMinSize = 750x620 for the main window and only pops it after this runs, so
	// without our own override the preview gets clamped to 750px wide (the "still too wide" bug). Override it.
	PushStyleVar(ImGuiStyleVar_WindowMinSize, ImVec2(1.f, 1.f));
	SetNextWindowSize(ImVec2(flWinW, flWinH), ImGuiCond_Always);
	// Anchor to the menu's right edge, top-aligned, so the two panels sit side by side at the same height.
	const float flAnchorX = (bHaveMenu ? s_vMenuPos.x + s_vMenuSize.x + H::Draw.Scale(8) : GetIO().DisplaySize.x - flWinW - H::Draw.Scale(20));
	const float flAnchorY = (bHaveMenu ? s_vMenuPos.y + flInset : (GetIO().DisplaySize.y - flWinH) * 0.5f);
	SetNextWindowPos(ImVec2(flAnchorX, flAnchorY), ImGuiCond_Always);

	PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.f, 0.f, 0.f, 0.f)); // transparent: model is blitted behind it
	if (Begin("Model Preview##ESPModelPreview", nullptr,
		ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoBringToFrontOnFocus))
	{
		const ImVec2 vMin = GetWindowPos();
		const ImVec2 vSize = GetWindowSize();
		const ImVec2 vMax(vMin.x + vSize.x, vMin.y + vSize.y);
		ImDrawList* pDraw = GetWindowDrawList();

		// The panel background (header + body) is drawn as one rounded SURFACE rect under the model in
		// ModelPreview::DrawBackground (VGui pass); here we only draw the tabs / accent line / outline over it.
		const ImU32 colAccent= (ImU32)ImColor(F::Render.Accent);

		const float flHeaderH = H::Draw.Scale(40);
		const float flBodyTop = vMin.y + flHeaderH;

		// --- Footer: enemy/teammate buttons sit at the BOTTOM, same height/pad as the main menu's tab bar ---
		const float flFooterH   = H::Draw.Scale(40);
		const float flFooterPad = H::Draw.Scale(8);
		const float flFooterTop = vMax.y - flFooterH - flFooterPad;

		// --- "Preview" title: same font (FontBold) + ypos as the main menu title at the top ---
		{
			PushFont(F::Render.FontBold);
			PushStyleColor(ImGuiCol_Text, F::Render.Accent.Value);
			const char* sTitle = "preview";
			const ImVec2 vTitleSize = CalcTextSize(sTitle);
			SetCursorPos({ (vSize.x - vTitleSize.x) * 0.5f, H::Draw.Scale(11) + (H::Draw.Scale(22) - vTitleSize.y) * 0.5f });
			FText(sTitle);
			PopStyleColor();
			PopFont();
		}

		// --- Body: hand its rect (header bottom.. footer top) to the VGui paint pass (model blits there) ---
		F::ModelPreview.SetRect(int(vMin.x), int(flBodyTop), int(vSize.x), int(flFooterTop - flBodyTop));
		F::ModelPreview.SetPanelTop(int(vMin.y));    // whole-panel top, so the surface bg covers the header
		F::ModelPreview.SetPanelBottom(int(vMax.y)); // whole-panel bottom, so the bg covers the footer buttons
		F::ModelPreview.SetWantRender(true);

		// Drag inside the body (not the header/footer) to spin the model.
		SetCursorScreenPos(ImVec2(vMin.x, flBodyTop));
		InvisibleButton("##mp_rotate", ImVec2(vSize.x, flFooterTop - flBodyTop));
		if (IsItemActive())
			F::ModelPreview.AddYaw(GetIO().MouseDelta.x * 0.5f);
		// Scroll the wheel over the body to zoom the camera in/out.
		if (IsItemHovered() && GetIO().MouseWheel != 0.f)
			F::ModelPreview.AddZoom(GetIO().MouseWheel);

		// --- enemy / teammate buttons (bottom bar, FTabs BarTop = gradient accent bar above the active one,
		// exactly like the main menu's bottom tab bar) ---
		static int iPreviewSection = 0; // 0 = enemy, 1 = teammate
		const float flBtnW = (vSize.x - H::Draw.Scale(40)) / 2.f; // two buttons span x=20.. width-20
		PushFont(F::Render.FontBold);
		FTabs(
			{ "ENEMY", "TEAMMATE" },
			&iPreviewSection,
			{ flBtnW, flFooterH },
			{ H::Draw.Scale(20), flFooterTop - vMin.y },
			FTabsEnum::Horizontal | FTabsEnum::AlignCenter | FTabsEnum::BarTop,
			{}, {}, {}, {}, {}, 0.f, 2.f
		);
		PopFont();
		F::ModelPreview.SetSection(iPreviewSection);

		// The 2D ESP overlay (box/bones/health/uber/name/class+weapon icon+text) is drawn separately as a
		// SURFACE pass in IEngineVGui_Paint (F::ModelPreview.DrawOverlay), not here - that context can draw

		// --- Accent line under the title, fading out toward both ends (same level as the menu's title line) ---
		{
			const float x1 = vMin.x + H::Draw.Scale(12), x2 = vMax.x - H::Draw.Scale(12);
			// Line at header bottom (vMin.y + flHeaderH = Scale(40)) so it sits at the SAME level as the main
			// menu's title line (vDrawPos.y + flTitleHeight, also Scale(40)). h matches the menu's Scale(2).
			const float yT = vMin.y + flHeaderH, h = H::Draw.Scale(2);
			const ImU32 colT = colAccent & 0x00FFFFFFu;
			const float xm = (x1 + x2) * 0.5f;
			pDraw->AddRectFilledMultiColor({ x1, yT }, { xm, yT + h }, colT, colAccent, colAccent, colT);
			pDraw->AddRectFilledMultiColor({ xm, yT }, { x2, yT + h }, colAccent, colT, colT, colAccent);
		}

		// --- Shared menu outline stack around the whole panel (same color/rounding/outlines as the menu) ---
		DrawStyledOutline(pDraw, vMin, vMax);
	}
	End();
	PopStyleColor();
	PopStyleVar(); // WindowMinSize override
}

void CMenu::MenuVisuals(int iTab)
{
	using namespace ImGui;

	switch (iTab)
	{
	// ESP
	case 0:
	{
		if (BeginTable("VisualsESPTable", 2))
		{
			TableNextColumn();
			{
				// Global text styling for every group's name/labels/text (they all share FONT_ESP).
				if (Section("Text"))
				{
					FSDropdown(Vars::ESP::FontFamily, FDropdownEnum::Left);
					FSlider(Vars::ESP::FontSize, FSliderEnum::Right);
					FDropdown(Vars::ESP::TextStyle, FDropdownEnum::Left);
					FToggle(Vars::ESP::FontNoAntialias, FToggleEnum::Right);
				} EndSection();
				RenderESPSection(SectionEnum::Enemies);
				RenderESPSection(SectionEnum::EnemyBuildings);
				RenderESPSection(SectionEnum::Projectiles);
				RenderESPSection(SectionEnum::Pickups);
				RenderESPSection(SectionEnum::Intel);
			}
			TableNextColumn();
			{
				RenderESPSection(SectionEnum::Teammates);
				RenderESPSection(SectionEnum::TeammateBuildings);
				RenderESPSection(SectionEnum::ViewmodelWeapon);
				RenderESPSection(SectionEnum::ViewmodelArms);

				if (Section("Model preview"))
				{
					FToggle(Vars::Visuals::ModelPreview::Enabled);
				} EndSection();
			}
			EndTable();
		}

		RenderModelPreviewWindow();
		break;
	}
	// Misc
	case 1:
	{
		if (BeginTable("VisualsMiscTable", 2))
		{
			/* Column 1 */
			TableNextColumn();
			{
				if (Section("UI"))
				{
					FDropdown(Vars::Visuals::UI::ChatTags, FDropdownEnum::Left, -10);
					FColorPicker(Vars::Colors::Local, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(10), H::Draw.Scale(40) });
					// the reference player name changer (exact NAMES tab).

					// Per-cycler row editor: one input row per name with an X remove button and an "+ Add name"
					// button (max 16 = the reference k_maxNames).
					struct NameListUI_t { std::vector<std::string> vNames; std::string sCache = "\n"; };
					auto RenderNameList = [](const char* sID, NameListUI_t& st, std::string& sList)
					{
						if (sList != st.sCache)
						{
							st.vNames.clear();
							size_t iStart = 0;
							while (iStart <= sList.size() && !sList.empty())
							{
								size_t iEnd = sList.find('\n', iStart);
								if (iEnd == std::string::npos) iEnd = sList.size();
								st.vNames.push_back(sList.substr(iStart, iEnd - iStart));
								if (iEnd == sList.size()) break;
								iStart = iEnd + 1;
							}
							st.sCache = sList;
						}
						PushID(sID);
						bool bChanged = false;
						for (int i = 0; i < (int)st.vNames.size(); i++)
						{
							PushID(i);
							if (FInputText("Name...", st.vNames[i], H::Draw.Scale(240)))
								bChanged = true;
							if (FButton("X", FButtonEnum::Fit | FButtonEnum::SameLine))
							{
								st.vNames.erase(st.vNames.begin() + i--);
								bChanged = true;
							}
							PopID();
						}
						if ((int)st.vNames.size() < 16 && FButton("+ Add name"))
						{
							st.vNames.emplace_back();
							bChanged = true;
						}
						PopID();
						if (bChanged)
						{
							std::string sJoined;
							for (size_t i = 0; i < st.vNames.size(); i++)
							{
								if (i) sJoined += '\n';
								sJoined += st.vNames[i];
							}
							sList = st.sCache = sJoined;
						}
					};

					// (1) the reference g_streamerMode: change other player names (class / team role / everyone).
					// Right half-width so it sits to the RIGHT of the Chat tags dropdown on the same row.
					FDropdown(Vars::Visuals::UI::StreamerMode, FDropdownEnum::Right);
					if (Vars::Visuals::UI::StreamerMode.Value == Vars::Visuals::UI::StreamerModeEnum::TeamRole)
					{
						{   // Teammate label (applied as you type, ).
							static std::string sTeamLabelInput = "";
							if (FInputText("Teammate label...", sTeamLabelInput, H::Draw.Scale(284)))
								FSet(Vars::Visuals::UI::StreamerTeamLabel, sTeamLabelInput);
							if (!IsItemFocused())
								sTeamLabelInput = FGet(Vars::Visuals::UI::StreamerTeamLabel);
						}
						FToggle(Vars::Visuals::UI::TeamLabelCycle);
						if (Vars::Visuals::UI::TeamLabelCycle.Value)
						{
							static NameListUI_t stTeam;
							RenderNameList("teamlabel", stTeam, Vars::Visuals::UI::TeamLabelCycleList.Value);
						}
						{   // Enemy label.
							static std::string sEnemyLabelInput = "";
							if (FInputText("Enemy label...", sEnemyLabelInput, H::Draw.Scale(284)))
								FSet(Vars::Visuals::UI::StreamerEnemyLabel, sEnemyLabelInput);
							if (!IsItemFocused())
								sEnemyLabelInput = FGet(Vars::Visuals::UI::StreamerEnemyLabel);
						}
						FToggle(Vars::Visuals::UI::EnemyLabelCycle);
						if (Vars::Visuals::UI::EnemyLabelCycle.Value)
						{
							static NameListUI_t stEnemy;
							RenderNameList("enemylabel", stEnemy, Vars::Visuals::UI::EnemyLabelCycleList.Value);
						}
					}
					else if (Vars::Visuals::UI::StreamerMode.Value == Vars::Visuals::UI::StreamerModeEnum::Everyone)
					{
						{   // Everyone label.
							static std::string sEveryoneLabelInput = "";
							if (FInputText("Everyone label...", sEveryoneLabelInput, H::Draw.Scale(284)))
								FSet(Vars::Visuals::UI::StreamerEveryoneLabel, sEveryoneLabelInput);
							if (!IsItemFocused())
								sEveryoneLabelInput = FGet(Vars::Visuals::UI::StreamerEveryoneLabel);
						}
						FToggle(Vars::Visuals::UI::EveryoneLabelCycle);
						if (Vars::Visuals::UI::EveryoneLabelCycle.Value)
						{
							static NameListUI_t stEveryone;
							RenderNameList("everyonelabel", stEveryone, Vars::Visuals::UI::EveryoneLabelCycleList.Value);
						}
					}

					// (2) the reference g_useCustomUsername: change your own name.
					FToggle(Vars::Visuals::UI::ChangeUsername);
					if (Vars::Visuals::UI::ChangeUsername.Value)
					{
						{   // the reference g_customUsername: type your name to apply live (empty = off).
							// Commits on every keystroke (same as the streamer-label boxes above) so no
							static std::string sCustomNameInput = "";
							if (FInputText("Display name...", sCustomNameInput, H::Draw.Scale(284)))
								FSet(Vars::Visuals::UI::CustomLocalName, sCustomNameInput);
							if (!IsItemFocused())
								sCustomNameInput = FGet(Vars::Visuals::UI::CustomLocalName);
						}
						FToggle(Vars::Visuals::UI::NameCycle);
						if (Vars::Visuals::UI::NameCycle.Value)
						{
							static NameListUI_t stLocal;
							RenderNameList("localname", stLocal, Vars::Visuals::UI::NameCycleList.Value);
						}
					}

					// Shared cycle trigger + interval (the reference shares g_nameTrigger / g_nameInterval across
					// every cycler); shown whenever any cycle above is enabled.
					{
						const bool bAnyCycle =
							(Vars::Visuals::UI::ChangeUsername.Value && Vars::Visuals::UI::NameCycle.Value)
							|| (Vars::Visuals::UI::StreamerMode.Value == Vars::Visuals::UI::StreamerModeEnum::TeamRole
								&& (Vars::Visuals::UI::TeamLabelCycle.Value || Vars::Visuals::UI::EnemyLabelCycle.Value))
							|| (Vars::Visuals::UI::StreamerMode.Value == Vars::Visuals::UI::StreamerModeEnum::Everyone
								&& Vars::Visuals::UI::EveryoneLabelCycle.Value);
						if (bAnyCycle)
						{
							FDropdown(Vars::Visuals::UI::NameCycleTrigger, FDropdownEnum::Left);
							if (Vars::Visuals::UI::NameCycleTrigger.Value == Vars::Visuals::UI::NameCycleTriggerEnum::Timer)
								FSlider(Vars::Visuals::UI::NameCycleInterval, FSliderEnum::Right);
						}
					}
					PushTransparent(!Vars::Visuals::UI::FieldOfView.Value);
					{
						FSlider(Vars::Visuals::UI::FieldOfView);
					}
					PopTransparent();
					PushTransparent(!Vars::Visuals::UI::ZoomFieldOfView.Value);
					{
						FSlider(Vars::Visuals::UI::ZoomFieldOfView);
					}
					PopTransparent();
					/*
					PushTransparent(!Vars::Visuals::UI::AspectRatio.Value);
					{
						FSlider(Vars::Visuals::UI::AspectRatio);
					}
					PopTransparent();
					*/
					FToggle(Vars::Visuals::UI::RevealScoreboard, FToggleEnum::Left);
					FToggle(Vars::Visuals::UI::ScoreboardUtility, FToggleEnum::Right);
					FToggle(Vars::Visuals::UI::ScoreboardColors, FToggleEnum::Left);
					FToggle(Vars::Visuals::UI::CleanScreenshots, FToggleEnum::Right);
					// the reference custom crosshair (full port).
					FToggle(Vars::Visuals::UI::Crosshair, FToggleEnum::Left);
					FColorPicker(Vars::Colors::Crosshair, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(14), H::Draw.Scale(14) });
					if (Vars::Visuals::UI::Crosshair.Value) // hide sub-options when off
					{
						FToggle(Vars::Visuals::UI::CrosshairAntialias, FToggleEnum::Left);
						FSlider(Vars::Visuals::UI::CrosshairThickness);
						FSlider(Vars::Visuals::UI::CrosshairLength);
						FSlider(Vars::Visuals::UI::CrosshairGap);
						FSlider(Vars::Visuals::UI::CrosshairAspect);
						FSlider(Vars::Visuals::UI::CrosshairGapAspect);
						FSlider(Vars::Visuals::UI::CrosshairRotation);
						FDropdown(Vars::Visuals::UI::CrosshairArms);
						// Dot
						FToggle(Vars::Visuals::UI::CrosshairDot, FToggleEnum::Left);
						FToggle(Vars::Visuals::UI::CrosshairDotCircle, FToggleEnum::Right);
						FToggle(Vars::Visuals::UI::CrosshairDotCustomColor, FToggleEnum::Left);
						FColorPicker(Vars::Colors::CrosshairDot, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(14), H::Draw.Scale(14) });
						// Dynamic spread
						FToggle(Vars::Visuals::UI::CrosshairDynamic, FToggleEnum::Left);
						if (Vars::Visuals::UI::CrosshairDynamic.Value)
						{
							FSlider(Vars::Visuals::UI::CrosshairDynamicMin);
							FSlider(Vars::Visuals::UI::CrosshairDynamicMax);
							FSlider(Vars::Visuals::UI::CrosshairDynamicRef);
						}
						// Gradient
						FToggle(Vars::Visuals::UI::CrosshairGradient, FToggleEnum::Left);
						FColorPicker(Vars::Colors::CrosshairGradCenter, FColorPickerEnum::Left); // own labeled rows so the two swatches stay separate
						FColorPicker(Vars::Colors::CrosshairGradOuter, FColorPickerEnum::Right);
						FSlider(Vars::Visuals::UI::CrosshairGradStart);
						FSlider(Vars::Visuals::UI::CrosshairGradEnd);
						// Taper
						FToggle(Vars::Visuals::UI::CrosshairTaper, FToggleEnum::Left);
						FSlider(Vars::Visuals::UI::CrosshairTaperNear);
						FSlider(Vars::Visuals::UI::CrosshairTaperFar);
						// Outline
						FToggle(Vars::Visuals::UI::CrosshairOutline, FToggleEnum::Left);
						FColorPicker(Vars::Colors::CrosshairOutline, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(14), H::Draw.Scale(14) });
						FSlider(Vars::Visuals::UI::CrosshairOutlineThick);
						FToggle(Vars::Visuals::UI::CrosshairOutlineOnLines, FToggleEnum::Left);
						FToggle(Vars::Visuals::UI::CrosshairOutlineOnDot, FToggleEnum::Right);
						FToggle(Vars::Visuals::UI::CrosshairOutlineBlur, FToggleEnum::Left);
						FSlider(Vars::Visuals::UI::CrosshairOutlineBlurRadius);
						FDropdown(Vars::Visuals::UI::CrosshairOutlineSides);
					}
					// Cheat title moved to the Visuals > MENU subtab (next to the cheat tag).
				} EndSection();
				if (Section("Thirdperson", 8))
				{
					FToggle(Vars::Visuals::Thirdperson::Enabled, FToggleEnum::Left);
					FToggle(Vars::Visuals::Thirdperson::Crosshair, FToggleEnum::Right);
					FSlider(Vars::Visuals::Thirdperson::Distance);
					FSlider(Vars::Visuals::Thirdperson::Right);
					FSlider(Vars::Visuals::Thirdperson::Up);
				} EndSection();
				if (Vars::Debug::Options.Value)
				{
					if (Section("##Debug"))
					{
						FToggle(Vars::Visuals::Thirdperson::Scale, FToggleEnum::Left);
						FToggle(Vars::Visuals::Thirdperson::Collide, FToggleEnum::Right);
					} EndSection();
				}
			}
			/* Column 2 */
			TableNextColumn();
			{
				if (Section("Removals", 8))
				{
					FToggle(Vars::Visuals::Removals::Interpolation, FToggleEnum::Left);
					PushTransparent(Vars::Visuals::Removals::Interpolation.Value);
					{
						FToggle(Vars::Visuals::Removals::Lerp, FToggleEnum::Right);
					}
					PopTransparent();
					FToggle(Vars::Visuals::Removals::Disguises, FToggleEnum::Left);
					FToggle(Vars::Visuals::Removals::Taunts, FToggleEnum::Right);
					FToggle(Vars::Visuals::Removals::Scope, FToggleEnum::Left);
					FToggle(Vars::Visuals::Removals::PostProcessing, FToggleEnum::Right);
					FToggle(Vars::Visuals::Removals::ScreenOverlays, FToggleEnum::Left);
					FToggle(Vars::Visuals::Removals::ScreenEffects, FToggleEnum::Right);
					FToggle(Vars::Visuals::Removals::ViewPunch, FToggleEnum::Left);
					FToggle(Vars::Visuals::Removals::AngleForcing, FToggleEnum::Right);
					FToggle(Vars::Visuals::Removals::Ragdolls, FToggleEnum::Left);
					FToggle(Vars::Visuals::Removals::Gibs, FToggleEnum::Right);
					FToggle(Vars::Visuals::Removals::MOTD, FToggleEnum::Left);
					FToggle(Vars::Visuals::Removals::Cosmetics, FToggleEnum::Right);
				} EndSection();
				if (Section("Viewmodel", 8))
				{
					FToggle(Vars::Visuals::Viewmodel::CrosshairAim, FToggleEnum::Left);
					FToggle(Vars::Visuals::Viewmodel::ViewmodelAim, FToggleEnum::Right);
					FSlider(Vars::Visuals::Viewmodel::OffsetX, FSliderEnum::Left);
					FSlider(Vars::Visuals::Viewmodel::Pitch, FSliderEnum::Right);
					FSlider(Vars::Visuals::Viewmodel::OffsetY, FSliderEnum::Left);
					FSlider(Vars::Visuals::Viewmodel::Yaw, FSliderEnum::Right);
					FSlider(Vars::Visuals::Viewmodel::OffsetZ, FSliderEnum::Left);
					FSlider(Vars::Visuals::Viewmodel::Roll, FSliderEnum::Right);
					PushTransparent(!Vars::Visuals::Viewmodel::SwayScale.Value || !Vars::Visuals::Viewmodel::SwayInterp.Value);
					{
						FSlider(Vars::Visuals::Viewmodel::SwayScale, FSliderEnum::Left);
						FSlider(Vars::Visuals::Viewmodel::SwayInterp, FSliderEnum::Right);
					}
					PopTransparent();
				} EndSection();
				if (Section("World"))
				{
					FDropdown(Vars::Visuals::World::Modulations);
					FSDropdown(Vars::Visuals::World::WorldTexture, FDropdownEnum::Left);
					FSDropdown(Vars::Visuals::World::SkyboxChanger, FDropdownEnum::Right);
					PushTransparent(!(Vars::Visuals::World::Modulations.Value & Vars::Visuals::World::ModulationsEnum::World));
					{
						FColorPicker(Vars::Colors::WorldModulation, FColorPickerEnum::Left);
					}
					PopTransparent();
					PushTransparent(!(Vars::Visuals::World::Modulations.Value & Vars::Visuals::World::ModulationsEnum::Sky));
					{
						FColorPicker(Vars::Colors::SkyModulation, FColorPickerEnum::Right);
					}
					PopTransparent();
					PushTransparent(!(Vars::Visuals::World::Modulations.Value & Vars::Visuals::World::ModulationsEnum::Prop));
					{
						FColorPicker(Vars::Colors::PropModulation, FColorPickerEnum::Left);
					}
					PopTransparent();
					PushTransparent(!(Vars::Visuals::World::Modulations.Value & Vars::Visuals::World::ModulationsEnum::Particle));
					{
						FColorPicker(Vars::Colors::ParticleModulation, FColorPickerEnum::Right);
					}
					PopTransparent();
					PushTransparent(!(Vars::Visuals::World::Modulations.Value & Vars::Visuals::World::ModulationsEnum::Fog));
					{
						FColorPicker(Vars::Colors::FogModulation, FColorPickerEnum::Left);
					}
					PopTransparent();
					
					FToggle(Vars::Visuals::World::NearPropFade, FToggleEnum::Left);
					FToggle(Vars::Visuals::World::NoPropFade, FToggleEnum::Right);
					FToggle(Vars::Visuals::World::FlipWorld, FToggleEnum::Left); // mirror world view + controls
				} EndSection();
				if (Section("Weather"))
				{
					FToggle(Vars::Visuals::Weather::Fog, FToggleEnum::Left);
					FToggle(Vars::Visuals::Weather::FogLinkSkybox, FToggleEnum::Right);
					PushTransparent(!Vars::Visuals::Weather::Fog.Value);
					{
						FSlider(Vars::Visuals::Weather::FogStart, FSliderEnum::Left);
						FSlider(Vars::Visuals::Weather::FogEnd, FSliderEnum::Right);
						FSlider(Vars::Visuals::Weather::FogDensity, FSliderEnum::Left);
						FColorPicker(Vars::Visuals::Weather::FogColor, FColorPickerEnum::Right);
						PushTransparent(Vars::Visuals::Weather::FogLinkSkybox.Value);
						{
							FSlider(Vars::Visuals::Weather::FogSkyStart, FSliderEnum::Left);
							FSlider(Vars::Visuals::Weather::FogSkyEnd, FSliderEnum::Right);
							FSlider(Vars::Visuals::Weather::FogSkyDensity, FSliderEnum::Left);
						}
						PopTransparent();
					}
					PopTransparent();

					FDropdown(Vars::Visuals::Weather::Precipitation);
					PushTransparent(Vars::Visuals::Weather::Precipitation.Value != Vars::Visuals::Weather::PrecipitationEnum::Rain);
					{
						FSlider(Vars::Visuals::Weather::RainAlpha, FSliderEnum::Left);
						FSlider(Vars::Visuals::Weather::RainSpeed, FSliderEnum::Right);
						FSlider(Vars::Visuals::Weather::RainWidth, FSliderEnum::Left);
						FSlider(Vars::Visuals::Weather::RainLength, FSliderEnum::Right);
						FSlider(Vars::Visuals::Weather::RainRadius, FSliderEnum::Left);
						FSlider(Vars::Visuals::Weather::RainSideVel, FSliderEnum::Right);
					}
					PopTransparent();
					PushTransparent(Vars::Visuals::Weather::Precipitation.Value != Vars::Visuals::Weather::PrecipitationEnum::Snow);
					{
						FSlider(Vars::Visuals::Weather::SnowAlpha, FSliderEnum::Left);
						FSlider(Vars::Visuals::Weather::SnowSpeed, FSliderEnum::Right);
						FSlider(Vars::Visuals::Weather::SnowWidth, FSliderEnum::Left);
						FSlider(Vars::Visuals::Weather::SnowLength, FSliderEnum::Right);
						FSlider(Vars::Visuals::Weather::SnowRadius, FSliderEnum::Left);
						FSlider(Vars::Visuals::Weather::SnowSideVel, FSliderEnum::Right);
						FSlider(Vars::Visuals::Weather::SnowDensity, FSliderEnum::Left);
						FSDropdown(Vars::Visuals::Weather::SnowSprite, FDropdownEnum::Right);
					}
					PopTransparent();
				} EndSection();
				if (Section("Effects"))
				{
					// https://developer.valvesoftware.com/wiki/Team_Fortress_2/Particles
					// https://forums.alliedmods.net/showthread.php?t=127111
					FSDropdown(Vars::Visuals::Effects::BulletTracer, FDropdownEnum::Left);
					FSDropdown(Vars::Visuals::Effects::CritTracer, FDropdownEnum::Right);
					FSDropdown(Vars::Visuals::Effects::MedigunBeam, FDropdownEnum::Left);
					FSDropdown(Vars::Visuals::Effects::MedigunCharge, FDropdownEnum::Right);
					FSDropdown(Vars::Visuals::Effects::ProjectileTrail, FDropdownEnum::Left);
					FDropdown(Vars::Visuals::Effects::SpellFootsteps, FDropdownEnum::Right, -10);
					FColorPicker(Vars::Colors::SpellFootstep, FColorPickerEnum::SameLine | FColorPickerEnum::NoTooltip, {}, { H::Draw.Scale(10), H::Draw.Scale(40) });
					FDropdown(Vars::Visuals::Effects::WeaponSheen, FDropdownEnum::Left);
					FToggle(Vars::Visuals::Effects::WeaponSheenCustomColor);
					FColorPicker(Vars::Colors::WeaponSheen, FColorPickerEnum::SameLine | FColorPickerEnum::NoTooltip, {}, { H::Draw.Scale(14), H::Draw.Scale(14) });
					FDropdown(Vars::Visuals::Effects::RagdollEffects);
					FToggle(Vars::Visuals::Effects::DrawIconsThroughWalls);
					FToggle(Vars::Visuals::Effects::DrawDamageNumbersThroughWalls);
				} EndSection();
			}
			EndTable();
		}
		break;
	}
	// Menu
	case 2:
	{
		if (BeginTable("MenuTable", 2))
		{
			/* Column 1 */
			TableNextColumn();
			{
				if (Section("Settings", 8))
				{
					FColorPicker(Vars::Menu::Theme::Accent, FColorPickerEnum::Left);
					FColorPicker(Vars::Menu::Theme::Background, FColorPickerEnum::Right);
					FColorPicker(Vars::Menu::Theme::Active, FColorPickerEnum::Left);
					FColorPicker(Vars::Menu::Theme::Inactive, FColorPickerEnum::Right);

					// Shared menu/panel background style.
					FColorPicker(Vars::Menu::Style::Color, FColorPickerEnum::Full);
					FSlider(Vars::Menu::Style::Rounding);

					// Stacked outlines around every styled panel (menu, watermark, media player,
					// checkpoints, Minimal spectator list, pixel-surf point). "Outline layers" is how
					FSlider(Vars::Menu::Style::OutlineCount);
					const int iOutlines = Vars::Menu::Style::OutlineCount.Value;
					if (iOutlines >= 1)
					{
						FColorPicker(Vars::Menu::Style::OutlineColor1, FColorPickerEnum::Full);
						FSlider(Vars::Menu::Style::OutlineThickness1);
					}
					if (iOutlines >= 2)
					{
						FColorPicker(Vars::Menu::Style::OutlineColor2, FColorPickerEnum::Full);
						FSlider(Vars::Menu::Style::OutlineThickness2);
					}
					if (iOutlines >= 3)
					{
						FColorPicker(Vars::Menu::Style::OutlineColor3, FColorPickerEnum::Full);
						FSlider(Vars::Menu::Style::OutlineThickness3);
					}
					if (iOutlines >= 4)
					{
						FColorPicker(Vars::Menu::Style::OutlineColor4, FColorPickerEnum::Full);
						FSlider(Vars::Menu::Style::OutlineThickness4);
					}

					// Main menu font: an installed font family name ("Tahoma", "Segoe UI") or a file
					// name/stem; blank = built-in. Press Enter to apply - the watcher in CRender::Render
					FDropdown(Vars::Menu::Style::FontPreset);
					if (Vars::Menu::Style::FontPreset.Value == Vars::Menu::Style::FontPresetEnum::Custom)
						FSDropdown(Vars::Menu::Style::Font); // typed family name / file; Enter rebuilds the atlas
					FSlider(Vars::Menu::Style::FontScale);
					FToggle(Vars::Menu::Style::FontAntiAlias); // off = crisp non-antialiased glyphs (FreeType builds)

					// Single label input: this name replaces every name, becomes the cheat title, and the
					// chat tag is auto-bracketed ([name]) so all three stay consistent. Derive the other two
					FSDropdown(Vars::Menu::CheatTitle);
					{
						const std::string& sLabel = Vars::Menu::CheatTitle.Value;
						const std::string sBracketed = "[" + sLabel + "]";
						Vars::Menu::CheatTag.Value = sBracketed;
						Vars::Menu::CheatTag.Map[DEFAULT_BIND] = sBracketed;
						Vars::Visuals::UI::Name.Value = sLabel;
						Vars::Visuals::UI::Name.Map[DEFAULT_BIND] = sLabel;
					}
					FKeybind(Vars::Menu::PrimaryKey, FButtonEnum::Left, { Vars::Menu::SecondaryKey[DEFAULT_BIND], VK_LBUTTON, VK_RBUTTON });
					FKeybind(Vars::Menu::SecondaryKey, FButtonEnum::Right | FButtonEnum::SameLine, { Vars::Menu::PrimaryKey[DEFAULT_BIND], VK_LBUTTON, VK_RBUTTON });
				} EndSection();
				// watermark - dedicated section (was previously crammed into the Binds column).
				if (Section("Watermark", 8))
				{
					FToggle(Vars::Menu::Watermark);
					if (Vars::Menu::Watermark.Value) // hide sub-options when off
					{
						FDropdown(Vars::Menu::WmPosition);
						FToggle(Vars::Menu::WmShowFPS, FToggleEnum::Left);
						FToggle(Vars::Menu::WmShowTime, FToggleEnum::Right);
						FToggle(Vars::Menu::WmShowDate, FToggleEnum::Left);
						FToggle(Vars::Menu::WmShowPing, FToggleEnum::Right);
						FToggle(Vars::Menu::WmShowLoss, FToggleEnum::Left);
						FToggle(Vars::Menu::WmShowLogo, FToggleEnum::Right);
						FToggle(Vars::Menu::WmBackground, FToggleEnum::Left);
						FDropdown(Vars::Menu::WmSepGlyph); // separator style: Text, a built-in glyph, or a pasted custom SVG
						if (Vars::Menu::WmSepGlyph.Value == Vars::Menu::WmSepGlyphEnum::WmSepText)
							FSDropdown(Vars::Menu::WmSeparator); // customizable separator string (default "|")
						else if (Vars::Menu::WmSepGlyph.Value == Vars::Menu::WmSepGlyphEnum::WmSepCustom)
							FSDropdown(Vars::Menu::WmSepCustomSvg); // paste raw SVG markup
					}
				} EndSection();
				// Logo style/rotation - applies to the menu header logo and the (optional) watermark logo.
				if (Section("Logo", 8))
				{
					FDropdown(Vars::Menu::LogoStyle);
					FSlider(Vars::Menu::LogoRotateSpeed); // deg/sec, 0 = static
					if (Vars::Menu::LogoStyle.Value == Vars::Menu::LogoStyleEnum::LogoCustom)
						FSDropdown(Vars::Menu::LogoCustomSvg); // paste raw SVG markup
				} EndSection();
			}
			/* Column 2 */
			TableNextColumn();
			{
				if (Section("Indicators"))
				{
					FDropdown(Vars::Menu::Indicators);
					FDropdown(Vars::Menu::SpectatorListStyle);
					// sub-options: interwebz background toggle + row text colour for the panel ports.
					if (Vars::Menu::SpectatorListStyle.Value == Vars::Menu::SpectatorListStyleEnum::Interwebz)
						FToggle(Vars::Menu::SpectatorListBackground);
					if (Vars::Menu::SpectatorListStyle.Value == Vars::Menu::SpectatorListStyleEnum::Minimal
						|| Vars::Menu::SpectatorListStyle.Value == Vars::Menu::SpectatorListStyleEnum::Clarity
						|| Vars::Menu::SpectatorListStyle.Value == Vars::Menu::SpectatorListStyleEnum::Kamidere)
						FColorPicker(Vars::Menu::SpectatorListTextColor, FColorPickerEnum::Left); // own labeled row, not a swatch over the scale slider
					if (Vars::Menu::SpectatorListStyle.Value == Vars::Menu::SpectatorListStyleEnum::Minimal)
					{   // Minimal-style header label.
						static std::string sSpecTitleInput = "";
						if (FInputText("Title...", sSpecTitleInput, H::Draw.Scale(284)))
							FSet(Vars::Menu::SpectatorListTitle, sSpecTitleInput);
						if (!IsItemFocused())
							sSpecTitleInput = FGet(Vars::Menu::SpectatorListTitle);
					}
					FSlider(Vars::Menu::SpectatorListScale); // panel + font scale (font rebuilds via the watcher below)
					if (FSlider(Vars::Menu::Scale))
						H::Fonts.Reload(Vars::Menu::Scale[DEFAULT_BIND]);
					FToggle(Vars::Menu::CheapText);
				} EndSection();
				if (Vars::Debug::Options.Value)
				{
					if (Section("##Debug"))
					{
						FColorPicker(Vars::Colors::IndicatorGood, FColorPickerEnum::Left);
						FColorPicker(Vars::Colors::IndicatorTextGood, FColorPickerEnum::Right);
						FColorPicker(Vars::Colors::IndicatorBad, FColorPickerEnum::Left);
						FColorPicker(Vars::Colors::IndicatorTextBad, FColorPickerEnum::Right);
						FColorPicker(Vars::Colors::IndicatorMid, FColorPickerEnum::Left);
						FColorPicker(Vars::Colors::IndicatorTextMid, FColorPickerEnum::Right);
						FColorPicker(Vars::Colors::IndicatorMisc, FColorPickerEnum::Left);
						FColorPicker(Vars::Colors::IndicatorTextMisc, FColorPickerEnum::Right);
					}
					EndSection();
				}
				// media player - dedicated section (was previously buried in Visuals > MISC).
				if (Section("Media Player", 8))
				{
					FToggle(Vars::Menu::MediaPlayer, FToggleEnum::Left);
					if (Vars::Menu::MediaPlayer.Value) // hide sub-options (incl. card/strip style) when off
					{
						FDropdown(Vars::Menu::MediaPlayerMode); // style: card / text strip
						FDropdown(Vars::Menu::MediaPlayerPosition); // anchor preset
						FSlider(Vars::Menu::MediaPlayerPosX, FSliderEnum::Left);
						FSlider(Vars::Menu::MediaPlayerPosY, FSliderEnum::Right);
						FSlider(Vars::Menu::MediaPlayerScale); // rounding/colour now come from Config > STYLE
						FToggle(Vars::Menu::MediaPlayerThumb, FToggleEnum::Left);
						FToggle(Vars::Menu::MediaPlayerThumbCircle, FToggleEnum::Right);
						FSlider(Vars::Menu::MediaPlayerThumbSize, FSliderEnum::Left);
						FToggle(Vars::Menu::MediaPlayerProgress, FToggleEnum::Right);
						FToggle(Vars::Menu::MediaPlayerBackground, FToggleEnum::Left); // bg now uses the menu theme background
						FColorPicker(Vars::Menu::MediaPlayerTitleColor, FColorPickerEnum::Left); // own labeled rows so the two swatches don't overlap
						FColorPicker(Vars::Menu::MediaPlayerArtistColor, FColorPickerEnum::Right);
					}
				} EndSection();
			}
			EndTable();
		}
		break;
	}
	}
}

void CMenu::MenuMisc(int iTab)
{
	using namespace ImGui;

	switch (iTab)
	{
	// Main
	case 0:
	{
		if (BeginTable("MiscTable", 2))
		{
			/* Column 1 */
			TableNextColumn();
			{
				if (Section("Automation"))
				{
					FDropdown(Vars::Misc::Automation::AntiBackstab); // pitch/fake _might_ slip up some auto backstabs
					FToggle(Vars::Misc::Automation::AntiAFK, FToggleEnum::Left);
					FToggle(Vars::Misc::Automation::AntiAutobalance, FToggleEnum::Right);
					FToggle(Vars::Misc::Automation::TauntControl, FToggleEnum::Left);
					FToggle(Vars::Misc::Automation::KartControl, FToggleEnum::Right);
					FToggle(Vars::Misc::Automation::AutoF2Ignored, FToggleEnum::Left);
					FToggle(Vars::Misc::Automation::AutoF1Priority, FToggleEnum::Right);
					FToggle(Vars::Misc::Automation::AcceptItemDrops);
				} EndSection();
				if (Section("Mann vs. Machine", 8))
				{
					FToggle(Vars::Misc::MannVsMachine::InstantRespawn, FToggleEnum::Left);
					FToggle(Vars::Misc::MannVsMachine::InstantRevive, FToggleEnum::Right);
					FToggle(Vars::Misc::MannVsMachine::AllowInspect);
				} EndSection();
				if (Section("Doubletap", 8)) // moved from the Movement tab (kept HvH feature, all other HvH removed)
				{
					FToggle(Vars::Doubletap::Doubletap, FToggleEnum::Left);
					FToggle(Vars::Doubletap::Warp, FToggleEnum::Right);
					FToggle(Vars::Doubletap::RechargeTicks, FToggleEnum::Left);
					FToggle(Vars::Doubletap::AntiWarp, FToggleEnum::Right);
					FSlider(Vars::Doubletap::TickLimit, FSliderEnum::Left);
					FSlider(Vars::Doubletap::WarpRate, FSliderEnum::Right);
					FSlider(Vars::Doubletap::RechargeLimit, FSliderEnum::Left);
					FSlider(Vars::Doubletap::PassiveRecharge, FSliderEnum::Right);
				} EndSection();
			}
			/* Column 2 */
			TableNextColumn();
			{
				if (Section("Exploits", 8))
				{
					FToggle(Vars::Misc::Exploits::PureBypass, FToggleEnum::Left);
					FToggle(Vars::Misc::Exploits::CheatsBypass, FToggleEnum::Right);
					FToggle(Vars::Misc::Exploits::EquipRegionUnlock, FToggleEnum::Left);
					FToggle(Vars::Misc::Exploits::BackpackExpander, FToggleEnum::Right);
					FToggle(Vars::Misc::Exploits::PingReducer, FToggleEnum::Left);
					FToggle(Vars::Misc::Exploits::NoisemakerSpam, FToggleEnum::Right);
					PushTransparent(!Vars::Misc::Exploits::PingReducer.Value);
					{
						FSlider(Vars::Misc::Exploits::PingTarget);
					}
					PopTransparent();
				} EndSection();
				if (Section("Game", 8))
				{
					FToggle(Vars::Misc::Game::AntiCheatCompatibility, FToggleEnum::Left);
					FToggle(Vars::Misc::Game::F2PChatBypass, FToggleEnum::Right);
					FToggle(Vars::Misc::Game::NetworkFix, FToggleEnum::Left);
					FToggle(Vars::Misc::Game::SetupBonesOptimization, FToggleEnum::Right);
					FToggle(Vars::Misc::Game::CleanPOVDemos, FToggleEnum::Left);
					FToggle(Vars::Misc::Game::NoAmbyCrosshairResize, FToggleEnum::Right);
				} EndSection();
				if (Vars::Debug::Options.Value)
				{
					if (Section("##Debug AntiCheat"))
					{
						FToggle(Vars::Misc::Game::AntiCheatCritHack);
					} EndSection();
				}
				if (Section("Queueing"))
				{
					FDropdown(Vars::Misc::Queueing::ForceRegions);
					FToggle(Vars::Misc::Queueing::FreezeQueue, FToggleEnum::Left);
					FToggle(Vars::Misc::Queueing::AutoCasualQueue, FToggleEnum::Right);
				} EndSection();
				if (Section("Sound"))
				{
					FDropdown(Vars::Misc::Sound::Block);
					FToggle(Vars::Misc::Sound::HitsoundAlways, FToggleEnum::Left);
					FToggle(Vars::Misc::Sound::RemoveDSP, FToggleEnum::Right);
					FToggle(Vars::Misc::Sound::GiantWeaponSounds);
				} EndSection();
			}
			EndTable();
		}
		break;
	}
	}
}

// Tasks 8/19: dedicated Movement tab. MOVEMENT subtab = all movement features (moved out of Misc) plus
// the kept Doubletap; INDICATORS subtab = velocity + keybind indicators (moved out of Misc Visuals).
void CMenu::MenuMovement(int iTab)
{
	using namespace ImGui;

	switch (iTab)
	{
	// Movement
	case 0:
	{
		if (BeginTable("MovementTable", 2))
		{
			/* Column 1 */
			TableNextColumn();
			{
				if (FlatSection("Strafe", 8))
				{
					FToggle(Vars::Misc::Movement::AutoStrafe, FToggleEnum::Left);     // single perfect-strafe checkbox
					FToggle(Vars::Misc::Movement::StrafeOptimizer, FToggleEnum::Right); // manual strafe perfecter
					if (Vars::Misc::Movement::StrafeOptimizer.Value)
					{
						FSlider(Vars::Misc::Movement::StrafeOptimizerGain, FSliderEnum::Left);
						FSlider(Vars::Misc::Movement::StrafeOptimizerMinSpeed, FSliderEnum::Right);
					}
				} EndSection();
				if (FlatSection("Jumps", 8))
				{
					FToggle(Vars::Misc::Movement::Bunnyhop, FToggleEnum::Left);
					FToggle(Vars::Misc::Movement::EdgeJump, FToggleEnum::Right);
					FToggle(Vars::Misc::Movement::AutoJumpbug, FToggleEnum::Left);
					FToggle(Vars::Misc::Movement::AutoCTap, FToggleEnum::Right);
					FToggle(Vars::Misc::Movement::AutoRocketJump, FToggleEnum::Left);
					FToggle(Vars::Misc::Movement::LongJump, FToggleEnum::Right);
					FToggle(Vars::Misc::Movement::MiniJump, FToggleEnum::Left);
					FToggle(Vars::Misc::Movement::BreakJump, FToggleEnum::Right);
					if (Vars::Misc::Movement::MiniJump.Value)
					{
						FToggle(Vars::Misc::Movement::MiniJumpHoldDuck, FToggleEnum::Left);
						FToggle(Vars::Misc::Movement::MiniJumpQueue, FToggleEnum::Right);
					}
				} EndSection();
				if (FlatSection("Movement Tweaks", 8))
				{
					FToggle(Vars::Misc::Movement::FastStop, FToggleEnum::Left);
					FToggle(Vars::Misc::Movement::FastAccelerate, FToggleEnum::Right);
					FToggle(Vars::Misc::Movement::DuckSpeed, FToggleEnum::Left);
					FToggle(Vars::Misc::Movement::MovementLock, FToggleEnum::Right);
					FToggle(Vars::Misc::Movement::NoPush, FToggleEnum::Left);
					FToggle(Vars::Misc::Movement::ShieldTurnRate, FToggleEnum::Right);
				} EndSection();
				if (FlatSection("Edge Bug", 8))
				{
					FToggle(Vars::Misc::Movement::EdgeBug);
					if (Vars::Misc::Movement::EdgeBug.Value)
					{
						FSlider(Vars::Misc::Movement::EdgeBugLockTicks, FSliderEnum::Left);
						FToggle(Vars::Misc::Movement::EdgeBugAdvancedSearch, FToggleEnum::Right);
						FToggle(Vars::Misc::Movement::EdgeBugAutoStrafe, FToggleEnum::Left);
						FToggle(Vars::Misc::Movement::EdgeBugChatPrint, FToggleEnum::Right);
						if (Vars::Misc::Movement::EdgeBugAdvancedSearch.Value)
						{
							FSlider(Vars::Misc::Movement::EdgeBugScanRadius, FSliderEnum::Left);
							FSlider(Vars::Misc::Movement::EdgeBugAngleLimit, FSliderEnum::Right);
						}
						FToggle(Vars::Misc::Movement::EdgeBugMouseLock, FToggleEnum::Left);
						if (Vars::Misc::Movement::EdgeBugMouseLock.Value)
						{
							FDropdown(Vars::Misc::Movement::EdgeBugMouseLockType, FDropdownEnum::Right);
							FSlider(Vars::Misc::Movement::EdgeBugLockAmount, FSliderEnum::Left);
						}
						FToggle(Vars::Misc::Movement::EdgeBugVisualize, FToggleEnum::Left);
						FColorPicker(Vars::Colors::EdgeBugCircle, FColorPickerEnum::SameLine);
						FDropdown(Vars::Misc::Movement::EdgeBugSound, FDropdownEnum::Left);
						if (Vars::Misc::Movement::EdgeBugSound.Value == Vars::Misc::Movement::EdgeBugSoundEnum::Custom)
						{	// path relative to tf/sound/, e.g. "ui/hitsound.wav"
							static std::string sEdgeBugSoundInput = "";
							bool bEnter = FInputText("Custom sound path...", sEdgeBugSoundInput, H::Draw.Scale(284), ImGuiInputTextFlags_EnterReturnsTrue);
							if (!IsItemFocused())
								sEdgeBugSoundInput = FGet(Vars::Misc::Movement::EdgeBugSoundCustom);
							if (bEnter)
								FSet(Vars::Misc::Movement::EdgeBugSoundCustom, sEdgeBugSoundInput);
						}
					}
				} EndSection();
			}
			/* Column 2 */
			TableNextColumn();
			{
				if (FlatSection("Pixel Surf"))
				{
					FToggle(Vars::Misc::Movement::PixelSurf, FToggleEnum::Left);
					FToggle(Vars::Misc::Movement::AutoAlign, FToggleEnum::Right);
					FToggle(Vars::Misc::Movement::PixelFinder, FToggleEnum::Left);
					FToggle(Vars::Misc::Movement::PixelSurfAssist, FToggleEnum::Right);
					if (Vars::Misc::Movement::PixelFinder.Value)
						FSlider(Vars::Misc::Movement::PixelFinderScanMargin);
					FToggle(Vars::Misc::Movement::PixelSurfLine, FToggleEnum::Left);
					if (Vars::Misc::Movement::PixelSurfLine.Value)
						FColorPicker(Vars::Misc::Movement::PixelSurfLineColor, FColorPickerEnum::SameLine);
					if (Vars::Misc::Movement::PixelSurfAssist.Value)
					{
						FToggle(Vars::Misc::Movement::PixelSurfAssistSetPoint, FToggleEnum::Left);
						FToggle(Vars::Misc::Movement::PixelSurfAssistDeletePoint, FToggleEnum::Right);
						FToggle(Vars::Misc::Movement::PixelSurfAssistRender, FToggleEnum::Left);
						FToggle(Vars::Misc::Movement::PixelSurfAssistJumpBox, FToggleEnum::Right);
						if (Vars::Misc::Movement::PixelSurfAssistJumpBox.Value)
							FSlider(Vars::Misc::Movement::PixelSurfAssistJumpBoxPosY, FSliderEnum::Left);
						FSlider(Vars::Misc::Movement::PixelSurfAssistSnapDist, FSliderEnum::Left);
						FSlider(Vars::Misc::Movement::PixelSurfAssistRadius, FSliderEnum::Right);
						// Steer lock = hold the commit-tick movement for the whole hop so the live
						// arc matches the simulated pick. (The old per-type reach sliders are gone -
						FToggle(Vars::Misc::Movement::PixelSurfAssistSteer, FToggleEnum::Left);
					}
				} EndSection();
				if (FlatSection("Texture Bug / Wall Climb", 8))
				{
					FToggle(Vars::Misc::Movement::TextureBug, FToggleEnum::Left);
					FToggle(Vars::Misc::Movement::WallClimb, FToggleEnum::Right);
					if (Vars::Misc::Movement::TextureBug.Value)
					{
						FToggle(Vars::Misc::Movement::TextureBugChokeTick, FToggleEnum::Left);
						FSlider(Vars::Misc::Movement::TextureBugChokeTicks, FSliderEnum::Right);
						FToggle(Vars::Misc::Movement::TextureBugAutoCrouch, FToggleEnum::Left);
						FToggle(Vars::Misc::Movement::TextureBugEdgeStop, FToggleEnum::Right);
						FSlider(Vars::Misc::Movement::TextureBugHoldTicks, FSliderEnum::Left);
						FSlider(Vars::Misc::Movement::TextureBugCatchEps, FSliderEnum::Right);
						FSlider(Vars::Misc::Movement::TextureBugReach, FSliderEnum::Left);
						FSlider(Vars::Misc::Movement::TextureBugScanStep, FSliderEnum::Right);
					}
					FToggle(Vars::Misc::Movement::HeadSurf, FToggleEnum::Left);
					FToggle(Vars::Misc::Movement::AirStuck, FToggleEnum::Right);
					if (Vars::Misc::Movement::AirStuck.Value)
					{
						FSlider(Vars::Misc::Movement::AirStuckCatchEps, FSliderEnum::Left);
						FSlider(Vars::Misc::Movement::AirStuckReach, FSliderEnum::Right);
						FSlider(Vars::Misc::Movement::AirStuckFlushTol, FSliderEnum::Left);
						FSlider(Vars::Misc::Movement::AirStuckDriftGain, FSliderEnum::Right);
						FSlider(Vars::Misc::Movement::AirStuckSimBudget, FSliderEnum::Left);
					}
				} EndSection();
				/* Doubletap section moved to the Misc tab (below Mann vs. Machine) */
				/* Recorder moved to its own Movement > RECORDER subtab */
				if (FlatSection("Jump Stats", 8))
				{
					// Jump stats print to chat on landing; no on-screen panel.
					FToggle(Vars::Misc::Movement::JumpStats);
				} EndSection();
				if (FlatSection("Checkpoints", 8)) // KZ practice
				{
					// Bind a key to each action; the bound keys show in the on-screen Checkpoints window.
					// Needs sv_cheats for setpos/setang/noclip (CheatsBypass can supply it).
					FToggle(Vars::Misc::Movement::Checkpoints);
					if (Vars::Misc::Movement::Checkpoints.Value)
					{
						FKeybind(Vars::Misc::Movement::CheckpointSaveKey, FButtonEnum::Left);
						FKeybind(Vars::Misc::Movement::CheckpointTeleportKey, FButtonEnum::Right | FButtonEnum::SameLine);
						FKeybind(Vars::Misc::Movement::CheckpointNoclipKey);
					}
				} EndSection();
				if (Vars::Debug::Options.Value)
				{
					if (Section("##Debug Movement"))
					{
						FSlider(Vars::Misc::Movement::TimingOffset);
						FSlider(Vars::Misc::Movement::ChokeCount);
						FSlider(Vars::Misc::Movement::ApplyAbove);
					} EndSection();
				}
			}
			EndTable();
		}
		break;
	}
	// Indicators (moved out of Misc Visuals)
	case 1:
	{
		if (BeginTable("IndicatorsTable", 2))
		{
			/* Column 1 - velocity */
			TableNextColumn();
			{
				if (Section("Velocity Indicator"))
				{
					FToggle(Vars::Visuals::UI::VelocityIndicator, FToggleEnum::Left);
					// Shared thin-font option for both indicators - visible if either is on.
					if (Vars::Visuals::UI::VelocityIndicator.Value || Vars::Visuals::UI::KeybindIndicator.Value)
						FToggle(Vars::Visuals::UI::IndicatorThinFont, FToggleEnum::Right);
					if (Vars::Visuals::UI::VelocityIndicator.Value)
					{
						// per-indicator font override (empty = use default/thin toggle).
						FSDropdown(Vars::Visuals::UI::VelocityFontFamily, FDropdownEnum::Left);
						FSlider(Vars::Visuals::UI::VelocityFontSize, FSliderEnum::Right);
						FSlider(Vars::Visuals::UI::VelocityPosX, FSliderEnum::Left);
						FSlider(Vars::Visuals::UI::VelocityPosY, FSliderEnum::Right);
						FToggle(Vars::Visuals::UI::VelocityColorBySpeed, FToggleEnum::Left);
						FToggle(Vars::Visuals::UI::VelocityTakeoff, FToggleEnum::Right);
						FColorPicker(Vars::Visuals::UI::VelocityColorLow, FColorPickerEnum::Left); // labeled rows so the swatches don't stack
						if (Vars::Visuals::UI::VelocityColorBySpeed.Value)
						{
							FColorPicker(Vars::Visuals::UI::VelocityColorMid, FColorPickerEnum::Right);
							FColorPicker(Vars::Visuals::UI::VelocityColorHigh, FColorPickerEnum::Full);
							FSlider(Vars::Visuals::UI::VelocitySpeedLow, FSliderEnum::Left);
							FSlider(Vars::Visuals::UI::VelocitySpeedHigh, FSliderEnum::Right);
						}
						if (Vars::Visuals::UI::VelocityTakeoff.Value)
						{
							FSlider(Vars::Visuals::UI::VelocityTakeoffHold);
							FDropdown(Vars::Visuals::UI::VelocityTakeoffPos, FDropdownEnum::Left);
							FDropdown(Vars::Visuals::UI::VelocityTakeoffAnim, FDropdownEnum::Right);
							if (Vars::Visuals::UI::VelocityTakeoffAnim.Value == Vars::Visuals::UI::VelocityTakeoffAnimEnum::Static)
								FSlider(Vars::Visuals::UI::VelocityTakeoffFade); // 0 = appears instantly
						}
						FToggle(Vars::Visuals::UI::VelocityShadow, FToggleEnum::Left);
						if (Vars::Visuals::UI::VelocityShadow.Value) // colour box sits inline next to the label
							FColorPicker(Vars::Visuals::UI::VelocityShadowColor, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(14), H::Draw.Scale(14) });
						FToggle(Vars::Visuals::UI::VelocityOutline, FToggleEnum::Left);
						if (Vars::Visuals::UI::VelocityShadow.Value)
						{
							FSlider(Vars::Visuals::UI::VelocityShadowOffX, FSliderEnum::Left);
							FSlider(Vars::Visuals::UI::VelocityShadowOffY, FSliderEnum::Right);
							FToggle(Vars::Visuals::UI::VelocityShadowBlur);
						}
						if (Vars::Visuals::UI::VelocityOutline.Value)
						{
							FColorPicker(Vars::Visuals::UI::VelocityOutlineColor, FColorPickerEnum::Left); // pair with thickness so the swatch isn't covered by the slider
							FSlider(Vars::Visuals::UI::VelocityOutlineThick, FSliderEnum::Right);
						}
						FToggle(Vars::Visuals::UI::VelocityFadeIn, FToggleEnum::Left);
						if (Vars::Visuals::UI::VelocityFadeIn.Value)
							FSlider(Vars::Visuals::UI::VelocityFadeThreshold, FSliderEnum::Right);
					}
				} EndSection();
				if (Section("Velocity Graph", 8))
				{
					FToggle(Vars::Visuals::UI::VelocityGraph, FToggleEnum::Left);
					if (Vars::Visuals::UI::VelocityGraph.Value)
					{
						FSlider(Vars::Visuals::UI::VelocityScale, FSliderEnum::Right);
						FSlider(Vars::Visuals::UI::VelocityGraphPosX, FSliderEnum::Left);
						FSlider(Vars::Visuals::UI::VelocityGraphPosY, FSliderEnum::Right);
						FSlider(Vars::Visuals::UI::VelocityGraphW, FSliderEnum::Left);
						FSlider(Vars::Visuals::UI::VelocityGraphH, FSliderEnum::Right);
						FSlider(Vars::Visuals::UI::VelocityGraphMaxSpeed);
						FColorPicker(Vars::Visuals::UI::VelocityGraphColor, FColorPickerEnum::Left); // pair with fade so the swatch isn't hidden behind the dropdown
						FDropdown(Vars::Visuals::UI::VelocityGraphFade, FDropdownEnum::Right);
						FSlider(Vars::Visuals::UI::VelocityGraphLineH);
					}
				} EndSection();
			}
			/* Column 2 - keybinds */
			TableNextColumn();
			{
				if (Section("Keybind Indicator"))
				{
					FToggle(Vars::Visuals::UI::KeybindIndicator);
					if (Vars::Visuals::UI::KeybindIndicator.Value)
					{
						FSlider(Vars::Visuals::UI::KeybindPosX, FSliderEnum::Left);
						FSlider(Vars::Visuals::UI::KeybindPosY, FSliderEnum::Right);
						// per-indicator font override.
						FSDropdown(Vars::Visuals::UI::KeybindFontFamily, FDropdownEnum::Left);
						FSlider(Vars::Visuals::UI::KeybindFontSize, FSliderEnum::Right);
						/* cheat name moved to Visuals > MENU subtab, next to the cheat tag */
						// - layout / animation / label colours.
						FDropdown(Vars::Visuals::UI::KeybindLayout, FDropdownEnum::Left);
						FDropdown(Vars::Visuals::UI::KeybindEntryAnim, FDropdownEnum::Right);
						FDropdown(Vars::Visuals::UI::KeybindEasing); // smoothing: own full-width row so entry-offset never overlaps it
							if (Vars::Visuals::UI::KeybindLayout.Value != Vars::Visuals::UI::KeybindLayoutEnum::Horizontal)
								FDropdown(Vars::Visuals::UI::KeybindVerticalDir); // grow up/down from the anchor (no centering)
						// keybind text colour grouped with the detect/glow colours below
						if (Vars::Visuals::UI::KeybindEntryAnim.Value != Vars::Visuals::UI::KeybindEntryAnimEnum::Static)
							FSlider(Vars::Visuals::UI::KeybindEntryOffset, FSliderEnum::Left), FSlider(Vars::Visuals::UI::KeybindFadeDuration, FSliderEnum::Right);
						else // static: no entry offset; keep fade duration on its own row so it never stacks onto the text-colour row
							FSlider(Vars::Visuals::UI::KeybindFadeDuration, FSliderEnum::Left);
						FToggle(Vars::Visuals::UI::KeybindShadow, FToggleEnum::Left);
						if (Vars::Visuals::UI::KeybindShadow.Value) FColorPicker(Vars::Visuals::UI::KeybindShadowColor, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(14), H::Draw.Scale(14) }); // colour box inline next to the label
						FToggle(Vars::Visuals::UI::KeybindDetectGlow, FToggleEnum::Left);
						if (Vars::Visuals::UI::KeybindShadow.Value)
						{
							// shadow colour is shown inline next to the toggle above
							FSlider(Vars::Visuals::UI::KeybindShadowOffX, FSliderEnum::Left);
							FSlider(Vars::Visuals::UI::KeybindShadowOffY, FSliderEnum::Right);
							FToggle(Vars::Visuals::UI::KeybindShadowBlur);
						}
						// Labelled detect (success) + glow colours, paired so they don't overlap each other or the label dropdowns below.
						FColorPicker(Vars::Visuals::UI::KeybindTextColor, FColorPickerEnum::Left); FColorPicker(Vars::Visuals::UI::KeybindDetectColor, FColorPickerEnum::Right);
						if (Vars::Visuals::UI::KeybindDetectGlow.Value)
							FColorPicker(Vars::Visuals::UI::KeybindDetectGlowColor, FColorPickerEnum::Left),
								FSlider(Vars::Visuals::UI::KeybindDetectGlowBlur, FSliderEnum::Left),       // gaussian skirt softness (sigma px)
								FSlider(Vars::Visuals::UI::KeybindDetectGlowStencil, FSliderEnum::Right);   // bold/dilate the core before blurring
							// Foreground label recolour on detection (separate from the shadow detect tint).
							FToggle(Vars::Visuals::UI::KeybindDetectText, FToggleEnum::Left);
							if (Vars::Visuals::UI::KeybindDetectText.Value) FColorPicker(Vars::Visuals::UI::KeybindDetectTextColor, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(14), H::Draw.Scale(14) });
						// Detection particles: a burst of shapes that erupts from the label on detection.
						FToggle(Vars::Visuals::UI::KeybindDetectParticles, FToggleEnum::Left);
						if (Vars::Visuals::UI::KeybindDetectParticles.Value) FColorPicker(Vars::Visuals::UI::KeybindDetectParticleColor, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(14), H::Draw.Scale(14) });
						if (Vars::Visuals::UI::KeybindDetectParticles.Value)
						{
							FDropdown(Vars::Visuals::UI::KeybindDetectParticleShape, FDropdownEnum::Left);
							FSlider(Vars::Visuals::UI::KeybindDetectParticleCount, FSliderEnum::Right);
							FSlider(Vars::Visuals::UI::KeybindDetectParticleSpeed, FSliderEnum::Left);
							FSlider(Vars::Visuals::UI::KeybindDetectParticleLifetime, FSliderEnum::Right);
							FSlider(Vars::Visuals::UI::KeybindDetectParticleScale, FSliderEnum::Left);
							FSlider(Vars::Visuals::UI::KeybindDetectParticleSpread, FSliderEnum::Right);
							FSlider(Vars::Visuals::UI::KeybindDetectParticleGravity, FSliderEnum::Left);
							FToggle(Vars::Visuals::UI::KeybindDetectParticleAntiAlias, FToggleEnum::Right);
						}
						// Detection ripple: expanding rings that pulse out of the label a few times when it fires.
						FToggle(Vars::Visuals::UI::KeybindDetectRipple, FToggleEnum::Left);
						if (Vars::Visuals::UI::KeybindDetectRipple.Value) FColorPicker(Vars::Visuals::UI::KeybindDetectRippleColor, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(14), H::Draw.Scale(14) });
						if (Vars::Visuals::UI::KeybindDetectRipple.Value)
						{
							FSlider(Vars::Visuals::UI::KeybindDetectRippleCount, FSliderEnum::Left);
							FSlider(Vars::Visuals::UI::KeybindDetectRippleRadius, FSliderEnum::Right);
							FSlider(Vars::Visuals::UI::KeybindDetectRippleDuration, FSliderEnum::Left);
							FSlider(Vars::Visuals::UI::KeybindDetectRippleThickness, FSliderEnum::Right);
						}
						// Which feature indicators actually fire the detection effects (particles + ripple).
						if (Vars::Visuals::UI::KeybindDetectParticles.Value || Vars::Visuals::UI::KeybindDetectRipple.Value)
							FDropdown(Vars::Visuals::UI::KeybindDetectEffectMask, { "Edge bug", "Long jump", "Mini jump", "Auto align", "Pixel surf", "Texture bug", "Wall climb", "Jump bug", "Pixel finder", "Pixel surf assist", "Auto strafe", "Air stuck" }, {}, FDropdownEnum::Multi);
						// Order matches the indicator mask dropdowns (Edge bug first). 12 labels = 6 clean
						// Left/Right pairs.
						FSDropdown(Vars::Visuals::UI::KeybindLabelEdgeBug, FDropdownEnum::Left);
						FSDropdown(Vars::Visuals::UI::KeybindLabelLongJump, FDropdownEnum::Right);
						FSDropdown(Vars::Visuals::UI::KeybindLabelMiniJump, FDropdownEnum::Left);
						FSDropdown(Vars::Visuals::UI::KeybindLabelAutoAlign, FDropdownEnum::Right);
						FSDropdown(Vars::Visuals::UI::KeybindLabelPixelSurf, FDropdownEnum::Left);
						FSDropdown(Vars::Visuals::UI::KeybindLabelTextureBug, FDropdownEnum::Right);
						FSDropdown(Vars::Visuals::UI::KeybindLabelWallClimb, FDropdownEnum::Left);
						FSDropdown(Vars::Visuals::UI::KeybindLabelJumpBug, FDropdownEnum::Right);
						FSDropdown(Vars::Visuals::UI::KeybindLabelPixelFinder, FDropdownEnum::Left);
						FSDropdown(Vars::Visuals::UI::KeybindLabelPixelSurfAssist, FDropdownEnum::Right);
						FSDropdown(Vars::Visuals::UI::KeybindLabelAutoStrafe, FDropdownEnum::Left);
							FSDropdown(Vars::Visuals::UI::KeybindLabelAirStuck, FDropdownEnum::Right); // air stuck / wall stuck (combined)
						FDropdown(Vars::Visuals::UI::KeybindIndicatorMask, { "Edge bug", "Long jump", "Mini jump", "Auto align", "Pixel surf", "Texture bug", "Wall climb", "Jump bug", "Pixel finder", "Pixel surf assist", "Auto strafe", "Air stuck" }, {}, FDropdownEnum::Multi);
					}
				} EndSection();
			}
			EndTable();
		}
		break;
	}
	// Recorder (per-map named routes)
	case 2:
	{
		if (BeginTable("RecorderTable", 2))
		{
			/* Column 1 - settings + binds */
			TableNextColumn();
			{
				if (Section("Recorder"))
				{
					FToggle(Vars::Misc::Movement::MovementRecorder, FToggleEnum::Left);
					FToggle(Vars::Misc::Movement::MovementRecorderHud, FToggleEnum::Right);
					if (Vars::Misc::Movement::MovementRecorder.Value)
					{
						FToggle(Vars::Misc::Movement::MovementRecorderShowRoutes, FToggleEnum::Left);
						FToggle(Vars::Misc::Movement::MovementRecorderShowPath, FToggleEnum::Right);
						FToggle(Vars::Misc::Movement::MovementRecorderMoveToStart, FToggleEnum::Left);
						FToggle(Vars::Misc::Movement::MovementRecorderLockView, FToggleEnum::Right);
						if (Vars::Misc::Movement::MovementRecorderMoveToStart.Value)
						{
							FSlider(Vars::Misc::Movement::MovementRecorderMoveToStartDist);
							// How exactly playback must line up before it starts - tight numbers are what
							// let edgebugs/pixelsurfs reproduce (raise tolerance if the creep stalls short).
							FSlider(Vars::Misc::Movement::MovementRecorderStartTolerance, FSliderEnum::Left);
							FSlider(Vars::Misc::Movement::MovementRecorderStartSpeed, FSliderEnum::Right);
						}
						// Verbatim replay: no drift correction at all (pure byte-for-byte). Most faithful for
						// clean standstill-recorded routes; hard-overrides drift correct below when on.
						FToggle(Vars::Misc::Movement::MovementRecorderVerbatim, FToggleEnum::Left);
						// Closed-loop drift correction: pulls playback back onto the recorded path each tick
						// so it stays accurate over the whole route. Now ground-only and frozen around jump/
						if (!Vars::Misc::Movement::MovementRecorderVerbatim.Value)
						{
							FToggle(Vars::Misc::Movement::MovementRecorderDriftCorrect, FToggleEnum::Right);
							if (Vars::Misc::Movement::MovementRecorderDriftCorrect.Value)
								FSlider(Vars::Misc::Movement::MovementRecorderDriftStrength, FSliderEnum::Left);
						}
						// Route start markers: a ring of dots (or a solid ground circle) + the name.
						if (Vars::Misc::Movement::MovementRecorderShowRoutes.Value)
						{
							FToggle(Vars::Misc::Movement::MovementRecorderRouteSolid, FToggleEnum::Left);
							FColorPicker(Vars::Misc::Movement::MovementRecorderRouteColor, FColorPickerEnum::Right);
							FSlider(Vars::Misc::Movement::MovementRecorderRoutePoints, FSliderEnum::Left);
							FSlider(Vars::Misc::Movement::MovementRecorderRouteRadius, FSliderEnum::Right);
						}
						// Shadowplay: keep the last N seconds rolling so the save bind can grab them.
						FToggle(Vars::Misc::Movement::MovementRecorderShadowPlay, FToggleEnum::Left);
						if (Vars::Misc::Movement::MovementRecorderShadowPlay.Value)
							FSlider(Vars::Misc::Movement::MovementRecorderShadowSeconds, FSliderEnum::Right);
					}
				} EndSection();
				if (Vars::Misc::Movement::MovementRecorder.Value)
				{
					if (Section("Binds", 8)) // bind these to keys (Config > BINDS)
					{
						FToggle(Vars::Misc::Movement::MovementRecorderRecord, FToggleEnum::Left);
						FToggle(Vars::Misc::Movement::MovementRecorderPlay, FToggleEnum::Right);
						FToggle(Vars::Misc::Movement::MovementRecorderStop, FToggleEnum::Left);
						FToggle(Vars::Misc::Movement::MovementRecorderClear, FToggleEnum::Right);
						FToggle(Vars::Misc::Movement::MovementRecorderSave, FToggleEnum::Left);
						FToggle(Vars::Misc::Movement::MovementRecorderLoad, FToggleEnum::Right);
						FToggle(Vars::Misc::Movement::MovementRecorderShadowSave, FToggleEnum::Left);
					} EndSection();
				}
			}
			/* Column 2 - routes for the current map */
			TableNextColumn();
			{
				if (Section("Routes"))
				{
					// Status: recording tick count / playback progress / saved route count.
					std::string sStatus = F::Misc.RecorderIsRecording()
						? std::format("Recording... {} ticks", F::Misc.RecorderActiveFrames())
						: F::Misc.RecorderIsPlaying()
							? std::format("Playing {}/{}", F::Misc.RecorderPlaybackIdx(), F::Misc.RecorderActiveFrames())
							: std::format("{} route(s) on this map", F::Misc.RecorderRouteCount());
					FText(sStatus.c_str());

					// Save the active recording as a new named route (Enter or the button).
					static std::string sRouteName = "";
					bool bEnter = FInputText("Route name...", sRouteName, H::Draw.Scale(190), ImGuiInputTextFlags_EnterReturnsTrue);
					PushDisabled(F::Misc.RecorderActiveFrames() == 0);
					if (FButton("Save", FButtonEnum::SameLine | FButtonEnum::Fit) || (bEnter && F::Misc.RecorderActiveFrames() > 0))
					{
						F::Misc.RecorderSaveActiveAs(sRouteName);
						sRouteName.clear();
					}
					PopDisabled();

					// The saved routes for this map - click to select (Play uses the selection).
					const int iCount = F::Misc.RecorderRouteCount();
					if (BeginChild("RouteList", { 0, H::Draw.Scale(160) }, ImGuiChildFlags_Borders))
					{
						for (int i = 0; i < iCount; i++)
						{
							const bool bSel = (F::Misc.RecorderSelected() == i);
							std::string sLabel = std::format("{}  [{:.1f}s]##route{}", F::Misc.RecorderRouteName(i), F::Misc.RecorderRouteSeconds(i), i);
							if (Selectable(sLabel.c_str(), bSel))
								F::Misc.RecorderSelect(i);
						}
					} EndChild();

					PushDisabled(F::Misc.RecorderSelected() < 0);
					if (FButton("Delete route", FButtonEnum::Left))
						F::Misc.RecorderDeleteRoute(F::Misc.RecorderSelected());
					PopDisabled();
					if (FButton("Reload from disk", FButtonEnum::Right | FButtonEnum::SameLine))
						F::Misc.RecorderReload();

					// Cut seconds off the selected route's start/end (e.g. trim a shadowplay grab).
					const int iSel = F::Misc.RecorderSelected();
					const float flLen = iSel >= 0 ? F::Misc.RecorderRouteSeconds(iSel) : 0.f;
					if (iSel >= 0 && flLen > 0.5f)
					{
						static float flTrimStart = 0.f, flTrimEnd = 0.f;
						FSlider("Trim start (s)", &flTrimStart, 0.f, flLen, 0.5f, "%.1f", FSliderEnum::Left);
						FSlider("Trim end (s)", &flTrimEnd, 0.f, flLen, 0.5f, "%.1f", FSliderEnum::Right);
						if (FButton("Apply cut"))
						{
							F::Misc.RecorderTrimRoute(iSel, flTrimStart, flTrimEnd);
							flTrimStart = flTrimEnd = 0.f;
						}
					}
				} EndSection();
			}
			EndTable();
		}
		break;
	}
	// Fake POV: bake a fake demo camera angle (right/left/up/bottom/backwards/spinning) into recorded
	// POV demos without touching live aim, with a smooth turn-in and an on-screen direction arrow.
	case 3:
	{
		if (BeginTable("FakePOVTable", 2))
		{
			/* Column 1 - the fake demo angle */
			TableNextColumn();
			{
				if (Section("Fake Demo Angle"))
				{
					FToggle(Vars::Misc::Movement::FakePOV);
					if (Vars::Misc::Movement::FakePOV.Value)
					{
						FDropdown(Vars::Misc::Movement::FakePOVMode);
						if (Vars::Misc::Movement::FakePOVMode.Value == Vars::Misc::Movement::FakePOVModeEnum::Spinning)
							FSlider(Vars::Misc::Movement::FakePOVSpinSpeed);
						// How fast the camera turns into/out of position (mouse-turn feel).
						FSlider(Vars::Misc::Movement::FakePOVSmoothSpeed);
						// Snap real aim onto the fake direction when turning off (while recording) for a seamless cut.
						FToggle(Vars::Misc::Movement::FakePOVSnapView);
					}
					FText("Affects recorded POV demos only - live aim moves solely on the snap-on-disable cut.");
				} EndSection();
			}
			/* Column 2 - the on-screen indicator arrow */
			TableNextColumn();
			{
				if (Section("Indicator Arrow"))
				{
					FToggle(Vars::Misc::Movement::FakePOVArrow);
					if (Vars::Misc::Movement::FakePOVArrow.Value)
					{
						FSlider(Vars::Misc::Movement::FakePOVArrowSize, FSliderEnum::Left);
						FSlider(Vars::Misc::Movement::FakePOVArrowDist, FSliderEnum::Right);
						FColorPicker(Vars::Misc::Movement::FakePOVArrowColor, FColorPickerEnum::Left); // labeled row, not an orphan swatch
					}
				} EndSection();
			}
			EndTable();
		}
		break;
	}
	}
}

// Config tab consolidates the old Movement-tab pages (players/logging/output) with the old
// Cfg-tab pages (config/binds/materials/extra). Routes each subtab index to the existing handlers.
void CMenu::MenuConfig(int iTab)
{
	switch (iTab)
	{
	case 0: MenuSettings(0); break; // Config
	case 1: MenuSettings(1); break; // Binds
	case 2: MenuLogs(0);     break; // Players
	case 3: MenuLogs(1);     break; // Logging
	case 4: MenuLogs(2);     break; // Output
	case 5: MenuSettings(2); break; // Materials
	case 6: MenuSettings(3); break; // Extra
	}
}

void CMenu::MenuLogs(int iTab)
{
	using namespace ImGui;

	switch (iTab)
	{
	// PlayerList
	case 0:
	{
		if (Section("Players"))
		{
			if (I::EngineClient->IsInGame())
			{
				std::lock_guard tLock(m_tMutex);
				const auto& vPlayers = F::PlayerUtils.m_vPlayerCache;

				std::unordered_map<uint64_t, std::vector<const ListPlayer*>> mParties = {};
				int iPartyCount = 0;
				for (auto& tPlayer : vPlayers)
				{
					if (tPlayer.m_iParty)
					{
						mParties[tPlayer.m_iParty].push_back(&tPlayer);
						iPartyCount = std::max(iPartyCount, tPlayer.m_iParty);
					}
				}

				auto getTeamColor = [&](int iTeam, bool bAlive)
					{
						switch (iTeam)
						{
						case 3: return Color_t(100, 150, 200, bAlive ? 255 : 127).Lerp(Vars::Menu::Theme::Background.Value, 0.5f, LerpEnum::NoAlpha);
						case 2: return Color_t(255, 100, 100, bAlive ? 255 : 127).Lerp(Vars::Menu::Theme::Background.Value, 0.5f, LerpEnum::NoAlpha);
						}
						return Color_t(127, 127, 127, 255).Lerp(Vars::Menu::Theme::Background.Value, 0.5f, LerpEnum::NoAlpha);
					};
				auto drawPlayer = [&](const ListPlayer& tPlayer, int x, int y)
					{
						ImVec2 vOriginalPos = { !x ? GetStyle().WindowPadding.x : GetWindowWidth() / 2 + GetStyle().WindowPadding.x / 2, H::Draw.Scale(35 + 36 * y) };

						// background
						float flWidth = GetWindowWidth() / 2 - GetStyle().WindowPadding.x * 1.5f;
						float flHeight = H::Draw.Scale(28);
						ImColor tColor = ColorToVec(getTeamColor(tPlayer.m_iTeam, tPlayer.m_bAlive));
						ImVec2 vDrawPos = GetDrawPos() + vOriginalPos;
						GetWindowDrawList()->AddRectFilled(vDrawPos, { vDrawPos.x + flWidth, vDrawPos.y + flHeight }, tColor, H::Draw.Scale(4));

						// text + icons
						int lOffset = H::Draw.Scale(10);
						if (tPlayer.m_bLocal || F::Spectate.m_iIntendedTarget == tPlayer.m_iUserID || tPlayer.m_bFriend || tPlayer.m_bParty)
						{
							lOffset += H::Draw.Scale(19);
							SetCursorPos({ vOriginalPos.x + H::Draw.Scale(7), vOriginalPos.y + H::Draw.Scale(6) });
							if (tPlayer.m_bLocal)
								IconImage(ICON_MD_PERSON);
							else if (F::Spectate.m_iIntendedTarget == tPlayer.m_iUserID)
								IconImage(ICON_MD_VISIBILITY);
							else if (tPlayer.m_bFriend)
								IconImage(ICON_MD_GROUP);
							else if (tPlayer.m_bParty)
								IconImage(ICON_MD_GROUPS);
						}
						SetCursorPos({ vOriginalPos.x + lOffset, vOriginalPos.y + H::Draw.Scale(7) });
						auto sName = TruncateText(tPlayer.m_sName, flWidth / 2 - lOffset);
						FText(sName.c_str());
						lOffset += FCalcTextSize(sName.c_str()).x + H::Draw.Scale(8);

						// buttons
						bool bPopup = false;

						if (!tPlayer.m_bFake)
						{
							// tag bar
							SetCursorPos({ vOriginalPos.x + lOffset, vOriginalPos.y });
							if (BeginChild(std::format("TagBar{}", tPlayer.m_iUserID).c_str(), { flWidth - lOffset - H::Draw.Scale(4), flHeight }, ImGuiWindowFlags_None, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoBackground))
							{
								std::vector<PriorityLabel_t> vLabels = {};
								std::vector<std::pair<PriorityLabel_t*, int>> vTags = {};
								if (int iParty = tPlayer.m_iParty)
								{
									auto pTag = &F::PlayerUtils.m_vTags[F::PlayerUtils.TagToIndex(PARTY_TAG)];
									if (!--iParty)
										vTags.emplace_back(pTag, 0);
									else
										vLabels.emplace_back(std::format("{}: {}", pTag->m_sName, iParty), pTag->m_tColor.HueShift(iParty * 360.f / iPartyCount));
								}
								if (tPlayer.m_bF2P)
								{
									auto pTag = &F::PlayerUtils.m_vTags[F::PlayerUtils.TagToIndex(F2P_TAG)];
									vTags.emplace_back(pTag, 0);
								}
								for (auto& iID : F::PlayerUtils.GetPlayerTags(tPlayer.m_uAccountID))
								{
									if (auto pTag = F::PlayerUtils.GetTag(iID))
										vTags.emplace_back(pTag, iID);
								}

								PushFont(F::Render.FontSmall);
								const auto vDrawPos = GetDrawPos();
								float flTagOffset = 0;
								auto drawTag = [&](PriorityLabel_t& tTag, int iID)
									{
										ImColor tTagColor = ColorToVec(tTag.m_tColor);
										float flTagWidth = FCalcTextSize(tTag.m_sName.c_str()).x + H::Draw.Scale(!iID ? 10 : 25);
										float flTagHeight = H::Draw.Scale(20);
										ImVec2 vTagPos = { flTagOffset, H::Draw.Scale(4) };

										GetWindowDrawList()->AddRectFilled(vDrawPos + vTagPos, { vDrawPos.x + vTagPos.x + flTagWidth, vDrawPos.y + vTagPos.y + flTagHeight }, tTagColor, H::Draw.Scale(4));
										SetCursorPos({ vTagPos.x + H::Draw.Scale(5), vTagPos.y + H::Draw.Scale(3) });
										TextColored(VecToColor(tColor).Blend(tTag.m_tColor).IsColorBright() ? ImVec4(0, 0, 0, 1) : ImVec4(1, 1, 1, 1), tTag.m_sName.c_str());
										if (iID)
										{
											SetCursorPos({ vTagPos.x + flTagWidth - H::Draw.Scale(22), vTagPos.y - H::Draw.Scale(2) });
											if (IconButton(ICON_MD_CANCEL))
												F::PlayerUtils.RemoveTag(tPlayer.m_uAccountID, iID, true, tPlayer.m_sName.c_str());
										}

										flTagOffset += flTagWidth + H::Draw.Scale(4);
									};

								for (auto& tTag : vLabels)
									drawTag(tTag, 0);
								for (auto& [pTag, iID] : vTags)
									drawTag(*pTag, iID);
								PopFont();
							} EndChild();

							bPopup = IsItemHovered() && IsMouseReleased(ImGuiMouseButton_Right);
						}
						SetCursorPos(vOriginalPos);
						Button(std::format("##{}", tPlayer.m_iUserID).c_str(), { flWidth, flHeight });
						bPopup = bPopup || IsItemHovered() && IsMouseReleased(ImGuiMouseButton_Right);

						// popups
						if (bPopup)
							OpenPopup(std::format("RightClicked{}", tPlayer.m_iUserID).c_str());

						if (FBeginPopup(std::format("RightClicked{}", tPlayer.m_iUserID).c_str()))
						{
							PushStyleVar(ImGuiStyleVar_ItemSpacing, { H::Draw.Scale(8), H::Draw.Scale(8) });

							if (!tPlayer.m_bFake)
							{
								if (FSelectable("Profile"))
									I::SteamFriends->ActivateGameOverlayToUser("steamid", CSteamID(tPlayer.m_uAccountID, k_EUniversePublic, k_EAccountTypeIndividual));
								if (FSelectable("History"))
									I::SteamFriends->ActivateGameOverlayToWebPage(std::format("https://steamhistory.net/id/{}", CSteamID(tPlayer.m_uAccountID, k_EUniversePublic, k_EAccountTypeIndividual).ConvertToUint64()).c_str());
							}

							if (FSelectable(F::Spectate.m_iIntendedTarget == tPlayer.m_iUserID ? "Unspectate" : "Spectate"))
								F::Spectate.SetTarget(tPlayer.m_iUserID);

							if (!I::EngineClient->IsPlayingDemo() && FBeginMenu("Votekick"))
							{
								if (FSelectable("No reason"))
									I::ClientState->SendStringCmd(std::format("callvote Kick \"{} other\"", tPlayer.m_iUserID).c_str());
								if (FSelectable("Cheating"))
									I::ClientState->SendStringCmd(std::format("callvote Kick \"{} cheating\"", tPlayer.m_iUserID).c_str());
								if (FSelectable("Idle"))
									I::ClientState->SendStringCmd(std::format("callvote Kick \"{} idle\"", tPlayer.m_iUserID).c_str());
								if (FSelectable("Scamming"))
									I::ClientState->SendStringCmd(std::format("callvote Kick \"{} scamming\"", tPlayer.m_iUserID).c_str());

								ImGui::EndMenu();
							}

							if (!tPlayer.m_bFake)
							{
								if (FBeginMenu("Add tag"))
								{
									for (auto it = F::PlayerUtils.m_vTags.begin(); it != F::PlayerUtils.m_vTags.end(); it++)
									{
										int iID = std::distance(F::PlayerUtils.m_vTags.begin(), it);
										auto& tTag = *it;
										if (!tTag.m_bAssignable || F::PlayerUtils.HasTag(tPlayer.m_uAccountID, iID))
											continue;

										auto imColor = ColorToVec(tTag.m_tColor);
										PushStyleColor(ImGuiCol_Text, imColor);
										imColor.x /= 3; imColor.y /= 3; imColor.z /= 3;
										if (FSelectable(tTag.m_sName.c_str(), imColor))
											F::PlayerUtils.AddTag(tPlayer.m_uAccountID, iID, true, tPlayer.m_sName.c_str());
										PopStyleColor();
									}

									ImGui::EndMenu();
								}
								if (FBeginMenu("Alias"))
								{
									bool bHasAlias = F::PlayerUtils.m_mPlayerAliases.contains(tPlayer.m_uAccountID);
									static std::string sInput = "";

									bool bEnter = FInputText("Alias...", sInput, H::Draw.Scale(150), ImGuiInputTextFlags_EnterReturnsTrue);
									if (!IsItemFocused())
										sInput = bHasAlias ? F::PlayerUtils.m_mPlayerAliases[tPlayer.m_uAccountID] : "";
									if (bEnter)
									{
										if (sInput.empty() && bHasAlias)
										{
											F::Output.AliasChanged(tPlayer.m_sName.c_str(), "Removed", F::PlayerUtils.m_mPlayerAliases[tPlayer.m_uAccountID].c_str());

											F::PlayerUtils.m_mPlayerAliases.erase(tPlayer.m_uAccountID);
											F::PlayerUtils.m_bSave = true;
										}
										else if (!sInput.empty())
										{
											F::PlayerUtils.m_mPlayerAliases[tPlayer.m_uAccountID] = sInput;
											F::PlayerUtils.m_bSave = true;

											F::Output.AliasChanged(tPlayer.m_sName.c_str(), bHasAlias ? "Changed" : "Added", sInput.c_str());
										}
									}

									ImGui::EndMenu();
								}
							}

							if (Vars::Resolver::Enabled.Value && !tPlayer.m_bLocal && !I::EngineClient->IsPlayingDemo())
							{
								if (FBeginMenu("Set yaw"))
								{
									static std::vector<std::pair<const char*, float>> vYaws = {
										{ "Auto", 0.f },
										{ "Forward", 0.f },
										{ "Left", 90.f },
										{ "Right", -90.f },
										{ "Backwards", 180.f }
									};
									for (auto& [sYaw, flValue] : vYaws)
									{
										if (FSelectable(sYaw))
										{
											switch (FNV1A::Hash32(sYaw))
											{
											case FNV1A::Hash32Const("Auto"):
												F::Resolver.SetYaw(tPlayer.m_iUserID, 0.f, true);
												break;
											default:
												F::Resolver.SetYaw(tPlayer.m_iUserID, flValue);
											}
										}
									}

									ImGui::EndMenu();
								}
								if (FBeginMenu("Set pitch"))
								{
									static std::vector<std::pair<const char*, float>> vPitches = {
										{ "Auto", 0.f },
										{ "Up", -90.f },
										{ "Down", 90.f },
										{ "Zero", 0.f },
										{ "Inverse", 0.f }
									};
									for (auto& [sPitch, flValue] : vPitches)
									{
										if (FSelectable(sPitch))
										{
											switch (FNV1A::Hash32(sPitch))
											{
											case FNV1A::Hash32Const("Auto"):
												F::Resolver.SetPitch(tPlayer.m_iUserID, 0.f, false, true);
												break;
											case FNV1A::Hash32Const("Inverse"):
												F::Resolver.SetPitch(tPlayer.m_iUserID, 0.f, true);
												break;
											default:
												F::Resolver.SetPitch(tPlayer.m_iUserID, flValue);
											}
										}
									}

									ImGui::EndMenu();
								}
								if (FBeginMenu("Set view"))
								{
									static std::vector<std::pair<const char*, bool>> vPitches = {
										{ "Offset from static view", true },
										{ "Offset from view to local", false }
									};
									for (auto& [sPitch, bValue] : vPitches)
									{
										if (FSelectable(sPitch))
											F::Resolver.SetView(tPlayer.m_iUserID, bValue);
									}

									ImGui::EndMenu();
								}
								if (FBeginMenu("Set minwalk"))
								{
									static std::vector<std::pair<const char*, bool>> vPitches = {
										{ "Minwalk on", true },
										{ "Minwalk off", false }
									};
									for (auto& [sPitch, bValue] : vPitches)
									{
										if (FSelectable(sPitch))
											F::Resolver.SetMinwalk(tPlayer.m_iUserID, bValue);
									}

									ImGui::EndMenu();
								}
							}

							if (mParties.contains(tPlayer.m_iParty))
							{
								Divider(H::Draw.Scale(), H::Draw.Scale(1), 0);

								TextColored(F::Render.Inactive.Value, "Partied:");
								for (auto& pPlayer2 : mParties[tPlayer.m_iParty])
									TextColored(F::Render.Inactive.Value, pPlayer2->m_sName.c_str());
							}

							if (tPlayer.m_iLevel != -2)
							{
								Divider(H::Draw.Scale(), H::Draw.Scale(1), 0);

								std::string sLevel = "T? L?";
								if (tPlayer.m_iLevel != -1)
								{
									int iTier = std::max(std::ceil(tPlayer.m_iLevel / 150.f), 1.f);
									int iLevel = ((tPlayer.m_iLevel - 1) % 150) + 1;
									sLevel = std::format("T{} L{}", iTier, iLevel);
								}
								TextColored(F::Render.Inactive.Value, sLevel.c_str());
							}

							PopStyleVar();
							EndPopup();
						}
					};

				// display players
				std::vector<ListPlayer> vBlu, vRed, vOther;
				for (auto& tPlayer : vPlayers)
				{
					switch (tPlayer.m_iTeam)
					{
					case 3: vBlu.push_back(tPlayer); break;
					case 2: vRed.push_back(tPlayer); break;
					default: vOther.push_back(tPlayer); break;
					}
				}

				int iBlu = 0, iRed = 0;
				for (size_t i = 0; i < vBlu.size(); i++)
				{
					drawPlayer(vBlu[i], 0, int(i));
					iBlu++;
				}
				for (size_t i = 0; i < vRed.size(); i++)
				{
					drawPlayer(vRed[i], 1, int(i));
					iRed++;
				}
				if (vOther.empty())
				{
					SetCursorPos({ 0, H::Draw.Scale(36 * std::max(iBlu, iRed) - 1) }); DebugDummy({ 0, H::Draw.Scale(28) });
				}
				else
				{
					size_t iMax = std::max(iBlu, iRed);
					for (size_t i = 0; i < vOther.size(); i++)
						drawPlayer(vOther[i], i % 2, int(iMax + i / 2));
				}
			}
			else
			{
				SetCursorPos({ H::Draw.Scale(15), H::Draw.Scale(40) });
				FText("Not ingame");
				DebugDummy({ 0, H::Draw.Scale(8) });
			}
		} EndSection();
		if (Section("Tags"))
		{
			static int iID = -1;
			static PriorityLabel_t tTag = {};

			auto vTable = WidgetTable(3, H::Draw.Scale(56), { GetWindowWidth() / 2, GetWindowWidth() / 2 - H::Draw.Scale(90) - GetStyle().WindowPadding.x });

			if (BeginWidgetTable(0, vTable))
			{
				FSDropdown("Name", &tTag.m_sName, {}, FDropdownEnum::Left | FSDropdownEnum::AutoUpdate, -10);
				FColorPicker("Color", &tTag.m_tColor, FColorPickerEnum::SameLine, {}, { H::Draw.Scale(10), H::Draw.Scale(40) });

				PushDisabled(iID == DEFAULT_TAG || iID == IGNORED_TAG);
				{
					int iLabel = Disabled ? 0 : tTag.m_bLabel;
					FDropdown("Type", &iLabel, { "Priority", "Label" }, {}, FDropdownEnum::Right);
					tTag.m_bLabel = iLabel;
					if (Disabled)
						tTag.m_bLabel = false;
				}
				PopDisabled();
			} EndChild();

			if (BeginWidgetTable(1, vTable))
			{
				PushTransparent(tTag.m_bLabel); // transparent if we want a label, user can still use to sort
				{
					SetCursorPosY(GetCursorPos().y + H::Draw.Scale(12));
					FSlider("Priority", &tTag.m_iPriority, -10, 10);
				}
				PopTransparent();
			} EndChild();

			if (BeginWidgetTable(2, vTable))
			{
				// create/modify button
				bool bCreate = false, bClear = false;

				SetCursorPos({ GetWindowWidth() - H::Draw.Scale(95), H::Draw.Scale(8) });
				PushDisabled(tTag.m_sName.empty());
				{
					bCreate = FButton(iID != -1 ? ICON_MD_SETTINGS : ICON_MD_ADD, FButtonEnum::None, { 40, 40 }, 0, F::Render.IconFont);
				}
				PopDisabled();

				// clear button
				SetCursorPos({ GetWindowWidth() - H::Draw.Scale(47), H::Draw.Scale(8) });
				bClear = FButton(ICON_MD_CLEAR, FButtonEnum::None, { 40, 40 }, 0, F::Render.IconFont);

				if (bCreate)
				{
					F::PlayerUtils.m_bSave = true;
					if (iID > -1 || iID < F::PlayerUtils.m_vTags.size())
					{
						F::PlayerUtils.m_vTags[iID].m_sName = tTag.m_sName;
						F::PlayerUtils.m_vTags[iID].m_tColor = tTag.m_tColor;
						F::PlayerUtils.m_vTags[iID].m_iPriority = tTag.m_iPriority;
						F::PlayerUtils.m_vTags[iID].m_bLabel = tTag.m_bLabel;
					}
					else
						F::PlayerUtils.m_vTags.push_back(tTag);
				}
				if (bCreate || bClear)
				{
					iID = -1;
					tTag = {};
				}
			} EndChild();

			auto drawTag = [](std::vector<PriorityLabel_t>::iterator it, PriorityLabel_t& _tTag, int y)
				{
					int _iID = std::distance(F::PlayerUtils.m_vTags.begin(), it);

					ImVec2 vOriginalPos = { !_tTag.m_bLabel ? GetStyle().WindowPadding.x : GetWindowWidth() * 2 / 3 + GetStyle().WindowPadding.x / 2, H::Draw.Scale(96 + 36 * y) };

					// background
					float flWidth = GetWindowWidth() * (_tTag.m_bLabel ? 1.f / 3 : 2.f / 3) - GetStyle().WindowPadding.x * 1.5f;
					float flHeight = H::Draw.Scale(28);
					ImColor tColor = ColorToVec(_tTag.m_tColor.Lerp(Vars::Menu::Theme::Background.Value, 0.5f, LerpEnum::NoAlpha));
					ImVec2 vDrawPos = GetDrawPos() + vOriginalPos;
					if (iID != _iID)
						GetWindowDrawList()->AddRectFilled(vDrawPos, { vDrawPos.x + flWidth, vDrawPos.y + flHeight }, tColor, H::Draw.Scale(4));
					else
					{
						ImColor tColor2 = { tColor.Value.x * 1.1f, tColor.Value.y * 1.1f, tColor.Value.z * 1.1f, tColor.Value.w };
						GetWindowDrawList()->AddRectFilled(vDrawPos, { vDrawPos.x + flWidth, vDrawPos.y + flHeight }, tColor2, H::Draw.Scale(4));

						tColor2 = ColorToVec(_tTag.m_tColor.Lerp(Vars::Menu::Theme::Background.Value, 0.25f, LerpEnum::NoAlpha));
						float flInset = H::Draw.Scale(0.5f) - 0.5f;
						GetWindowDrawList()->AddRect({ vDrawPos.x + flInset, vDrawPos.y + flInset }, { vDrawPos.x - flInset + flWidth, vDrawPos.y - flInset + flHeight }, tColor2, H::Draw.Scale(4), ImDrawFlags_None, H::Draw.Scale());
					}

					// text
					SetCursorPos({ vOriginalPos.x + H::Draw.Scale(9), vOriginalPos.y + H::Draw.Scale(7) });
					FText(TruncateText(_tTag.m_sName, _tTag.m_bLabel ? flWidth - H::Draw.Scale(38) : flWidth / 2 - H::Draw.Scale(20)).c_str());

					if (!_tTag.m_bLabel)
					{
						SetCursorPos({ vOriginalPos.x + flWidth / 2, vOriginalPos.y + H::Draw.Scale(7) });
						FText(std::format("{}", _tTag.m_iPriority).c_str());
					}

					// buttons / icons
					bool bDelete = false;
					if (!_tTag.m_bLocked)
					{
						SetCursorPos({ vOriginalPos.x + flWidth - H::Draw.Scale(26), vOriginalPos.y + H::Draw.Scale(2) });
						bDelete = IconButton(ICON_MD_DELETE);
					}
					else
					{
						SetCursorPos({ vOriginalPos.x + flWidth - H::Draw.Scale(22), vOriginalPos.y + H::Draw.Scale(6) });
						switch (F::PlayerUtils.IndexToTag(_iID))
						{
						//case DEFAULT_TAG: // no image
						case IGNORED_TAG: IconImage(ICON_MD_DO_NOT_DISTURB); break;
						case CHEATER_TAG: IconImage(ICON_MD_FLAG); break;
						case FRIEND_TAG: IconImage(ICON_MD_GROUP); break;
						case PARTY_TAG: IconImage(ICON_MD_GROUPS); break;
						case F2P_TAG: IconImage(ICON_MD_MONEY_OFF); break;
						}
					}

					SetCursorPos(vOriginalPos);
					bool bClicked = Button(std::format("##{}", _tTag.m_sName).c_str(), { flWidth, flHeight });
					bool bPopup = IsItemClicked(ImGuiMouseButton_Right);

					if (bClicked)
					{
						iID = _iID;
						tTag.m_sName = _tTag.m_sName;
						tTag.m_tColor = _tTag.m_tColor;
						tTag.m_iPriority = _tTag.m_iPriority;
						tTag.m_bLabel = _tTag.m_bLabel;
					}
					else if (bPopup)
						OpenPopup(std::format("RightClicked{}", _iID).c_str());
					else if (bDelete)
						OpenPopup(std::format("DeleteTag{}", _iID).c_str());

					if (FBeginPopup(std::format("RightClicked{}", _iID).c_str()))
					{
						PushStyleVar(ImGuiStyleVar_ItemSpacing, { H::Draw.Scale(8), 0 });

						auto& _tTag2 = *it;
						bool bSave = false;

						{
							static std::string sInput = "";

							bool bEnter = FInputText("Name...", sInput, H::Draw.Scale(284), ImGuiInputTextFlags_EnterReturnsTrue);
							if (!IsItemFocused())
								sInput = _tTag2.m_sName;
							if (bEnter)
							{
								_tTag2.m_sName = sInput;
								bSave = true;
							}
						}

						PushDisabled(_iID == DEFAULT_TAG || _iID == IGNORED_TAG);
						{
							int iLabel = Disabled ? 0 : _tTag2.m_bLabel;
							if (FDropdown("Type##", &iLabel, { "Priority", "Label" }))
								bSave = true;
							_tTag2.m_bLabel = iLabel;
							if (Disabled)
								_tTag2.m_bLabel = false;
						}
						PopDisabled();
						if (FSlider("Priority##", &_tTag2.m_iPriority, -10, 10))
							bSave = true;

						if (bSave)
							F::PlayerUtils.m_bSave = true;

						PopStyleVar();
						EndPopup();
					}
					else if (FBeginPopupModal(std::format("DeleteTag{}", _iID).c_str(), nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysUseWindowPadding))
					{
						FText(std::format("Do you really want to delete '{}'?", _tTag.m_sName).c_str());

						if (FButton("Yes", FButtonEnum::Left))
						{
							F::PlayerUtils.m_vTags.erase(it);
							F::PlayerUtils.m_bSave = F::PlayerUtils.m_bSave = true;

							for (auto& [_, vTags] : F::PlayerUtils.m_mPlayerTags)
							{
								for (auto it = vTags.begin(); it != vTags.end();)
								{
									if (_iID == *it)
										vTags.erase(it);
									else
									{
										if (_iID < *it)
											(*it)--;
										it++;
									}
								}
							}

							if (iID == _iID)
							{
								iID = -1;
								tTag = {};
							}
							else if (iID > _iID)
								iID--;

							CloseCurrentPopup();
						}
						if (FButton("No", FButtonEnum::Right | FButtonEnum::SameLine))
							CloseCurrentPopup();

						EndPopup();
					}
				};

			PushStyleColor(ImGuiCol_Text, F::Render.Inactive.Value);
			SetCursorPos({ H::Draw.Scale(13), H::Draw.Scale(80) }); FText("Priorities");
			SetCursorPos({ GetWindowWidth() * 2 / 3 + H::Draw.Scale(9), H::Draw.Scale(80) }); FText("Labels");
			PopStyleColor();

			std::vector<std::pair<std::vector<PriorityLabel_t>::iterator, PriorityLabel_t>> vPriorities = {}, vLabels = {};
			for (auto it = F::PlayerUtils.m_vTags.begin(); it != F::PlayerUtils.m_vTags.end(); it++)
			{
				auto& _tTag = *it;

				if (!_tTag.m_bLabel)
					vPriorities.emplace_back(it, _tTag);
				else
					vLabels.emplace_back(it, _tTag);
			}

			std::sort(vPriorities.begin(), vPriorities.end(), [&](const auto& a, const auto& b) -> bool
				{
					// override for default tag
					if (std::distance(F::PlayerUtils.m_vTags.begin(), a.first) == DEFAULT_TAG)
						return true;
					if (std::distance(F::PlayerUtils.m_vTags.begin(), b.first) == DEFAULT_TAG)
						return false;

					// sort by priority if unequal
					if (a.second.m_iPriority != b.second.m_iPriority)
						return a.second.m_iPriority > b.second.m_iPriority;

					return a.second.m_sName < b.second.m_sName;
				});
			std::sort(vLabels.begin(), vLabels.end(), [&](const auto& a, const auto& b) -> bool
				{
					// sort by priority if unequal
					if (a.second.m_iPriority != b.second.m_iPriority)
						return a.second.m_iPriority > b.second.m_iPriority;

					return a.second.m_sName < b.second.m_sName;
				});

			// display tags
			int iPriorities = 0, iLabels = 0;
			for (auto& pair : vPriorities)
			{
				drawTag(pair.first, pair.second, iPriorities);
				iPriorities++;
			}
			for (auto& pair : vLabels)
			{
				drawTag(pair.first, pair.second, iLabels);
				iLabels++;
			}
			SetCursorPos({ 0, H::Draw.Scale(60 + 36 * std::max(iPriorities, iLabels)) }); DebugDummy({ 0, H::Draw.Scale(28) });
		} EndSection();
		{
			PushDisabled(F::PlayerUtils.m_bLoad);
			{
				SetCursorPosY(GetCursorPosY() - H::Draw.Scale(8));
				if (FButton(ICON_MD_SYNC, FButtonEnum::None, { 30, 30 }, 0, F::Render.IconFont))
					F::PlayerUtils.m_bLoad = true;

				if (FButton("Export", FButtonEnum::Fit | FButtonEnum::SameLine))
				{
					// this should be up2date anyways
					std::ifstream fStream(F::Configs.m_sCorePath + "Players.json", std::ios_base::app);
					if (fStream.is_open())
					{
						std::string sString;
						{
							std::string line;
							while (std::getline(fStream, line))
								sString += line + "\n";
							if (!sString.empty())
								sString.pop_back();
						}
						fStream.close();

						SDK::SetClipboard(sString);
						SDK::Output("chudhook", "Copied playerlist to clipboard", DEFAULT_COLOR, OUTPUT_CONSOLE | OUTPUT_TOAST | OUTPUT_MENU | OUTPUT_DEBUG);
					}
				}

				{
					static std::vector<PriorityLabel_t> vTags = {};
					static std::unordered_map<uint32_t, std::vector<int>> mPlayerTags = {};
					static std::unordered_map<uint32_t, std::string> mPlayerAliases = {};
					static std::unordered_map<int, int> mAs = {};

					if (FButton("Import", FButtonEnum::Fit | FButtonEnum::SameLine))
					{
						try
						{
							// will not directly support older tag systems
							boost::property_tree::ptree tRead;
							std::stringstream ssStream;
							ssStream << SDK::GetClipboard();
							read_json(ssStream, tRead);

							mPlayerTags.clear();
							mPlayerAliases.clear();
							mAs.clear();
							vTags = {
								{ "Default", { 200, 200, 200, 255 }, 0, false, false, true },
								{ "Ignored", { 200, 200, 200, 255 }, -1, false, true, true },
								{ "Cheater", { 255, 100, 100, 255 }, 1, false, true, true },
								{ "Friend", { 100, 255, 100, 255 }, 0, true, false, true },
								{ "Party", { 100, 100, 255, 255 }, 0, true, false, true },
								{ "F2P", { 255, 255, 255, 255 }, 0, true, false, true }
							};

							if (auto tSub = tRead.get_child_optional("Config"))
							{
								for (auto& [sName, tChild] : *tSub)
								{
									PriorityLabel_t tTag = {};
									F::Configs.LoadJson(tChild, "Name", tTag.m_sName);
									F::Configs.LoadJson(tChild, "Color", tTag.m_tColor);
									F::Configs.LoadJson(tChild, "Priority", tTag.m_iPriority);
									F::Configs.LoadJson(tChild, "Label", tTag.m_bLabel);

									int iID = F::PlayerUtils.TagToIndex(std::stoi(sName));
									if (iID > -1 && iID < vTags.size())
									{
										vTags[iID].m_sName = tTag.m_sName;
										vTags[iID].m_tColor = tTag.m_tColor;
										vTags[iID].m_iPriority = tTag.m_iPriority;
										vTags[iID].m_bLabel = tTag.m_bLabel;
									}
									else
										vTags.push_back(tTag);
								}
							}

							if (auto tSub = tRead.get_child_optional("Tags"))
							{
								for (auto& [sName, tChild] : *tSub)
								{
									uint32_t uAccountID = std::stoul(sName);
									for (auto& [_, tTag] : tChild)
									{
										const std::string& sTag = tTag.data();

										int iID = F::PlayerUtils.TagToIndex(std::stoi(sTag));
										auto pTag = F::PlayerUtils.GetTag(iID);
										if (!pTag || !pTag->m_bAssignable)
											continue;

										if (!F::PlayerUtils.HasTag(uAccountID, iID, mPlayerTags))
											F::PlayerUtils.AddTag(uAccountID, iID, false, "", mPlayerTags);
									}
								}
							}

							if (auto tSub = tRead.get_child_optional("Aliases"))
							{
								for (auto& [sName, tAlias] : *tSub)
								{
									uint32_t uAccountID = std::stoul(sName);
									const std::string& sAlias = tAlias.data();

									if (!sAlias.empty())
										mPlayerAliases[uAccountID] = sAlias;
								}
							}

							for (int i = 0; i < vTags.size(); i++)
							{
								if (vTags[i].m_bAssignable)
								{
									if (F::PlayerUtils.IndexToTag(i) <= 0)
										mAs[i] = i;
									else
										mAs[i] = -1;
								}
							}
							OpenPopup("ImportPlayerlist");
						}
						catch (...)
						{
							SDK::Output("chudhook", "Failed to import playerlist", ALTERNATE_COLOR, OUTPUT_CONSOLE | OUTPUT_TOAST | OUTPUT_MENU | OUTPUT_DEBUG);
						}
					}

					SetNextWindowSize({ H::Draw.Scale(300), 0 });
					if (FBeginPopupModal("ImportPlayerlist", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysUseWindowPadding))
					{
						FText("Import");
						FText("As", FTextEnum::Right | FTextEnum::SameLine);

						for (int i = 0; i < vTags.size(); i++)
						{
							if (!vTags[i].m_bAssignable)
								continue;

							auto& iIDTo = mAs[i];

							ImVec2 vOriginalPos = GetCursorPos();
							PushStyleColor(ImGuiCol_Text, ColorToVec(vTags[i].m_tColor));
							SetCursorPos(vOriginalPos + ImVec2(H::Draw.Scale(8), H::Draw.Scale(5)));
							FText(vTags[i].m_sName.c_str());
							PopStyleColor();
							SetCursorPos(vOriginalPos - ImVec2(0, H::Draw.Scale(8))); DebugDummy({ GetWindowWidth() - GetStyle().WindowPadding.x * 2, H::Draw.Scale(32) });

							std::vector<const char*> vEntries = { "None" };
							std::vector<int> vValues = { 0 };
							for (int i = 0; i < F::PlayerUtils.m_vTags.size(); i++)
							{
								if (F::PlayerUtils.m_vTags[i].m_bAssignable)
								{
									vEntries.push_back(F::PlayerUtils.m_vTags[i].m_sName.c_str());
									vValues.push_back(i + 1);
								}
							}
							PushTransparent(iIDTo == -1);
							{
								int iTo = iIDTo + 1;
								FDropdown(std::format("##{}", i).c_str(), &iTo, vEntries, vValues, FSliderEnum::Right);
								iIDTo = iTo - 1;
							}
							PopTransparent();
						}

						if (FButton("Import", FButtonEnum::Left))
						{
							for (auto& [uAccountID, vTags] : mPlayerTags)
							{
								for (auto& iTag : vTags)
								{
									int iID = mAs.contains(iTag) ? mAs[iTag] : -1;
									if (iID != -1 && !F::PlayerUtils.HasTag(uAccountID, iID))
										F::PlayerUtils.AddTag(uAccountID, iID, false);
								}
							}
							for (auto& [uAccountID, sAlias] : mPlayerAliases)
							{
								if (!F::PlayerUtils.m_mPlayerAliases.contains(uAccountID))
									F::PlayerUtils.m_mPlayerAliases[uAccountID] = sAlias;
							}

							F::PlayerUtils.m_bSave = true;
							SDK::Output("chudhook", "Imported playerlist", DEFAULT_COLOR, OUTPUT_CONSOLE | OUTPUT_TOAST | OUTPUT_MENU | OUTPUT_DEBUG);

							CloseCurrentPopup();
						}
						if (FButton("Cancel", FButtonEnum::Right | FButtonEnum::SameLine))
							CloseCurrentPopup();

						EndPopup();
					}
				}

				if (FButton("Backup", FButtonEnum::Fit | FButtonEnum::SameLine))
				{
					try
					{
						int iBackupCount = 0;
						for (auto& entry : std::filesystem::directory_iterator(F::Configs.m_sCorePath))
						{
							if (!entry.is_regular_file() || entry.path().extension() != F::Configs.m_sConfigExtension)
								continue;

							std::string sConfigName = entry.path().filename().string();
							sConfigName.erase(sConfigName.end() - F::Configs.m_sConfigExtension.size(), sConfigName.end());
							if (sConfigName.find("Backup") != std::string::npos)
								iBackupCount++;
						}
						std::filesystem::copy(
							F::Configs.m_sCorePath + "Players.json",
							F::Configs.m_sCorePath + std::format("Backup{}.json", iBackupCount + 1),
							std::filesystem::copy_options::overwrite_existing
						);
						SDK::Output("chudhook", "Saved backup playerlist", DEFAULT_COLOR, OUTPUT_CONSOLE | OUTPUT_TOAST | OUTPUT_MENU | OUTPUT_DEBUG);
					}
					catch (...)
					{
						SDK::Output("chudhook", "Failed to backup playerlist", ALTERNATE_COLOR, OUTPUT_CONSOLE | OUTPUT_TOAST | OUTPUT_MENU | OUTPUT_DEBUG);
					}
				}
			}
			PopDisabled();

			if (FButton("Folder", FButtonEnum::Fit | FButtonEnum::SameLine))
				ShellExecuteA(NULL, NULL, F::Configs.m_sCorePath.c_str(), NULL, NULL, SW_SHOWNORMAL);
		}
		break;
	}
	// Settings
	case 1:
	{
		if (BeginTable("ConfigSettingsTable", 2))
		{
			/* Column 1 */
			TableNextColumn();
			{
				if (Section("Logging"))
				{
					FDropdown(Vars::Logging::Logs);
					FDropdown(Vars::Logging::NotificationPosition);
					FSlider(Vars::Logging::Lifetime);
				} EndSection();
				if (Section("Vote Start"))
				{
					PushTransparent(!(Vars::Logging::Logs.Value & Vars::Logging::LogsEnum::VoteStart));
					{
						FDropdown(Vars::Logging::VoteStart::LogTo);
					}
					PopTransparent();
				} EndSection();
				if (Section("Vote Cast"))
				{
					PushTransparent(!(Vars::Logging::Logs.Value & Vars::Logging::LogsEnum::VoteCast));
					{
						FDropdown(Vars::Logging::VoteCast::LogTo);
					}
					PopTransparent();
				} EndSection();
				if (Section("Class Change"))
				{
					PushTransparent(!(Vars::Logging::Logs.Value & Vars::Logging::LogsEnum::ClassChanges));
					{
						FDropdown(Vars::Logging::ClassChange::LogTo);
					}
					PopTransparent();
				} EndSection();
			}
			/* Column 2 */
			TableNextColumn();
			{
				if (Section("Damage"))
				{
					PushTransparent(!(Vars::Logging::Logs.Value & Vars::Logging::LogsEnum::Damage));
					{
						FDropdown(Vars::Logging::Damage::LogTo);
					}
					PopTransparent();
				} EndSection();
				if (Section("Cheat Detection"))
				{
					PushTransparent(!(Vars::Logging::Logs.Value & Vars::Logging::LogsEnum::CheatDetection));
					{
						FDropdown(Vars::Logging::CheatDetection::LogTo);
					}
					PopTransparent();
				} EndSection();
				if (Section("Tags"))
				{
					PushTransparent(!(Vars::Logging::Logs.Value & Vars::Logging::LogsEnum::Tags));
					{
						FDropdown(Vars::Logging::Tags::LogTo);
					}
					PopTransparent();
				} EndSection();
				if (Section("Aliases"))
				{
					PushTransparent(!(Vars::Logging::Logs.Value & Vars::Logging::LogsEnum::Aliases));
					{
						FDropdown(Vars::Logging::Aliases::LogTo);
					}
					PopTransparent();
				} EndSection();
				if (Section("Resolver"))
				{
					PushTransparent(!(Vars::Logging::Logs.Value & Vars::Logging::LogsEnum::Resolver));
					{
						FDropdown(Vars::Logging::Resolver::LogTo);
					}
					PopTransparent();
				} EndSection();
			}
			EndTable();
		}
		break;
	}
	// Output
	case 2:
	{
		if (Section("##Output", false, GetWindowHeight() - GetStyle().WindowPadding.y * 2))
		{
			for (auto& tOutput : m_vOutput)
			{
				ImVec2 vOriginalPos = GetCursorPos();
				size_t iLines = 1;

				float flWidth = GetWindowWidth() - GetStyle().WindowPadding.x * 2;
				if (tOutput.m_sFunction != "")
				{
					float flTitleWidth = 0.f;

					PushStyleColor(ImGuiCol_Text, ColorToVec(tOutput.tAccent));

					auto vWrapped = WrapText(tOutput.m_sFunction, flWidth);
					for (size_t i = 0; i < vWrapped.size(); i++)
					{
						FText(vWrapped[i].c_str());
						if (i == vWrapped.size() - 1)
							flTitleWidth = FCalcTextSize(vWrapped[i].c_str()).x + H::Draw.Scale(4);
					}
					iLines = vWrapped.size();

					PopStyleColor();

					vWrapped = WrapText(tOutput.m_sLog, flWidth - flTitleWidth);
					if (!vWrapped.empty())
					{
						SameLine(flTitleWidth + GetStyle().WindowPadding.x);
						FText(vWrapped.front().c_str());

						if (vWrapped.size() > 1)
						{
							std::string sLog = "";
							for (size_t i = 1; i < vWrapped.size(); i++)
							{
								sLog += vWrapped[i].c_str();
								if (i != vWrapped.size() - 1)
									sLog += " ";
							}
							vWrapped = WrapText(sLog, flWidth);
							for (size_t i = 0; i < vWrapped.size(); i++)
							{
								FText(vWrapped[i].c_str());
								if (i == vWrapped.size() - 1)
									flTitleWidth = FCalcTextSize(vWrapped[i].c_str()).x + H::Draw.Scale(4);
							}
							iLines += vWrapped.size();
						}
					}
				}
				else
				{
					PushStyleColor(ImGuiCol_Text, ColorToVec(tOutput.tAccent));

					auto vWrapped = WrapText(tOutput.m_sLog, flWidth);
					for (size_t i = 0; i < vWrapped.size(); i++)
						FText(vWrapped[i].c_str());
					iLines = vWrapped.size();

					PopStyleColor();
				}

				SetCursorPos(vOriginalPos); DebugDummy({ flWidth, H::Draw.Scale(13) * iLines + GetStyle().WindowPadding.y });

				if (IsItemHovered() && IsMouseReleased(ImGuiMouseButton_Right))
					OpenPopup(std::format("Output{}", tOutput.m_iID).c_str());
				if (FBeginPopup(std::format("Output{}", tOutput.m_iID).c_str()))
				{
					PushStyleVar(ImGuiStyleVar_ItemSpacing, { H::Draw.Scale(8), H::Draw.Scale(8) });

					if (FSelectable("Copy"))
						SDK::SetClipboard(std::format("{}{}{}", tOutput.m_sFunction, tOutput.m_sFunction != "" ? " " : "", tOutput.m_sLog));

					PopStyleVar();
					EndPopup();
				}
			}
		} EndSection();
		break;
	}
	}
}

void CMenu::MenuSettings(int iTab)
{
	using namespace ImGui;

	switch (iTab)
	{
	// Settings
	case 0:
	{
		if (BeginTable("ConfigSettingsTable", 2))
		{
			/*
			if (Section("Config"))
			{
				static int iCurrentType = 0;
				PushFont(F::Render.FontBold);
				FTabs({ "GENERAL", "VISUAL", }, &iCurrentType, { H::Draw.Scale(20), H::Draw.Scale(28) }, { GetWindowWidth(), 0 }, FTabsEnum::AlignReverse | FTabsEnum::Fit);
				SetCursorPosY(GetCursorPosY() - H::Draw.Scale());
				PopFont();

				switch (iCurrentType)
			*/

			auto drawConfigs = [](std::string& sStaticName, bool bVisual = false)
				{
					auto& sPath = !bVisual ? F::Configs.m_sConfigPath : F::Configs.m_sVisualsPath;
					auto& sConfig = !bVisual ? F::Configs.m_sCurrentConfig : F::Configs.m_sCurrentVisuals;
					auto sType = !bVisual ? "Config" : "Visual";
					bool bNoSave = GetAsyncKeyState(VK_SHIFT) & 0x8000;

					FSDropdown("Name", &sStaticName, {}, FSDropdownEnum::AutoUpdate, -H::Draw.Unscale(FCalcTextSize("CREATE").x + FCalcTextSize("FOLDER").x) - 72);
					PushDisabled(sStaticName.empty());
					{
						if (FButton("Create", FButtonEnum::Fit | FButtonEnum::SameLine, { 0, 40 }))
						{
							if (!std::filesystem::exists(sPath + sStaticName))
							{
								if (!bVisual)
									F::Configs.SaveConfig(sStaticName);
								else
									F::Configs.SaveVisual(sStaticName);
							}
							sStaticName.clear();
						}
					}
					PopDisabled();
					if (FButton("Folder", FButtonEnum::Fit | FButtonEnum::SameLine, { 0, 40 }))
						ShellExecuteA(NULL, NULL, sPath.c_str(), NULL, NULL, SW_SHOWNORMAL);
					vRowSizes.clear();

					std::vector<std::pair<std::filesystem::directory_entry, std::string>> vConfigs = {};
					bool bDefaultFound = false;
					for (auto& tEntry : std::filesystem::directory_iterator(sPath))
					{
						if (!tEntry.is_regular_file() || tEntry.path().extension() != F::Configs.m_sConfigExtension)
							continue;

						std::string sName = tEntry.path().filename().string();
						sName.erase(sName.end() - F::Configs.m_sConfigExtension.size(), sName.end());
						if (FNV1A::Hash32(sName.c_str()) == FNV1A::Hash32Const("default"))
							bDefaultFound = true;

						vConfigs.emplace_back(tEntry, sName);
					}
					if (!bVisual)
					{
						if (!bDefaultFound)
							F::Configs.SaveConfig("default");
						std::sort(vConfigs.begin(), vConfigs.end(), [&](const auto& a, const auto& b) -> bool
							{
								// override for default config
								if (FNV1A::Hash32(a.second.c_str()) == FNV1A::Hash32Const("default"))
									return true;
								if (FNV1A::Hash32(b.second.c_str()) == FNV1A::Hash32Const("default"))
									return false;

								return a.second < b.second;
							});
					}

					for (auto& [entry, sConfigName] : vConfigs)
					{
						bool bCurrentConfig = FNV1A::Hash32(sConfigName.c_str()) == FNV1A::Hash32(sConfig.c_str());
						ImVec2 vOriginalPos = GetCursorPos();

						SetCursorPos({ vOriginalPos.x + H::Draw.Scale(2), vOriginalPos.y + H::Draw.Scale(9) });
						bool bLoad = IconButton(bCurrentConfig ? ICON_MD_REFRESH : ICON_MD_DOWNLOAD);
						FTooltip(ICON_MD_ADD ICON_MD_FILE_UPLOAD_OFF, bNoSave && IsItemHovered(), 300.f, F::Render.IconFont);

						SetCursorPos({ H::Draw.Scale(43), vOriginalPos.y + H::Draw.Scale(14) });
						TextColored(bCurrentConfig ? F::Render.Active.Value : F::Render.Inactive.Value, TruncateText(sConfigName, GetWindowWidth() - GetStyle().WindowPadding.x * 2 - H::Draw.Scale(80)).c_str());

						int iOffset = 9;
						SetCursorPos({ GetWindowWidth() - H::Draw.Scale(iOffset += 25), vOriginalPos.y + H::Draw.Scale(9) });
						bool bDelete = IconButton(ICON_MD_DELETE);

						SetCursorPos({ GetWindowWidth() - H::Draw.Scale(iOffset += 25), vOriginalPos.y + H::Draw.Scale(9) });
						bool bSave = IconButton(ICON_MD_SAVE);
						FTooltip(ICON_MD_ADD ICON_MD_FILE_DOWNLOAD_OFF, bNoSave && IsItemHovered(), 300.f, F::Render.IconFont);

						if (bLoad)
						{
							if (!bVisual)
								F::Configs.LoadConfig(sConfigName);
							else
								F::Configs.LoadVisual(sConfigName);
						}
						else if (bSave)
						{
							if (!bCurrentConfig || !bVisual && !F::Configs.m_sCurrentVisuals.empty())
								OpenPopup(std::format("Save{}{}", sType, sConfigName).c_str());
							else if (!bVisual)
								F::Configs.SaveConfig(sConfigName);
							else
								F::Configs.SaveVisual(sConfigName);
						}
						else if (bDelete)
							OpenPopup(std::format("Remove{}{}", sType, sConfigName).c_str());

						if (FBeginPopupModal(std::format("Save{}{}", sType, sConfigName).c_str(), nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysUseWindowPadding))
						{
							FText(std::format("Do you really want to override '{}'?", sConfigName).c_str());

							if (FButton("Yes, override", FButtonEnum::Left))
							{
								if (!bVisual)
									F::Configs.SaveConfig(sConfigName);
								else
									F::Configs.SaveVisual(sConfigName);
								CloseCurrentPopup();
							}
							if (FButton("No", FButtonEnum::Right | FButtonEnum::SameLine))
								CloseCurrentPopup();

							EndPopup();
						}
						else if (FBeginPopupModal(std::format("Remove{}{}", sType, sConfigName).c_str(), nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysUseWindowPadding))
						{
							FText(std::format("Do you really want to remove '{}'?", sConfigName).c_str());

							PushDisabled(!bVisual && FNV1A::Hash32(sConfigName.c_str()) == FNV1A::Hash32Const("default"));
							{
								if (FButton("Yes, delete", FButtonEnum::Fit))
								{
									if (!bVisual)
										F::Configs.DeleteConfig(sConfigName);
									else
										F::Configs.DeleteVisual(sConfigName);
									CloseCurrentPopup();
								}
							}
							PopDisabled();
							if (FButton("Yes, reset", FButtonEnum::Fit | FButtonEnum::SameLine))
							{
								if (!bVisual)
									F::Configs.ResetConfig(sConfigName);
								else
									F::Configs.ResetVisual(sConfigName);
								CloseCurrentPopup();
							}
							if (FButton("No", FButtonEnum::Fit | FButtonEnum::SameLine))
								CloseCurrentPopup();

							EndPopup();
						}

						SetCursorPos(vOriginalPos); DebugDummy({ 0, H::Draw.Scale(28) });
					}
					DebugDummy({ 0, H::Draw.Scale(7) });
				};

			/* Column 1 */
			TableNextColumn();
			if (Section("Config"))
			{
				static std::string sStaticName;

				drawConfigs(sStaticName);
			} EndSection();
			PushStyleColor(ImGuiCol_Text, F::Render.Inactive.Value);
			SetCursorPosX(GetCursorPosX() + GetStyle().WindowPadding.x);
			FText("Built on " __DATE__ ", " __TIME__);
			//SetCursorPosX(GetCursorPosX() + GetStyle().WindowPadding.x);
			//FText(std::format("Time @ {}, {}", SDK::GetDate(), SDK::GetTime()).c_str());
			PopStyleColor();

			/* Column 2 */
			TableNextColumn();
			if (Section("Visuals"))
			{
				static std::string sStaticName;

				drawConfigs(sStaticName, true);
			} EndSection();

			EndTable();
		}
		break;
	}
	// Binds
	case 1:
	{
		if (Section("Settings", 8))
		{
			auto vTable = WidgetTable(4, H::Draw.Scale(24));

			if (BeginWidgetTable(0, vTable))
			{
				FToggle(Vars::Menu::BindWindow);
			} EndChild();

			if (BeginWidgetTable(1, vTable))
			{
				FToggle(Vars::Menu::BindWindowTitle);
			} EndChild();

			if (BeginWidgetTable(2, vTable))
			{
				FToggle(Vars::Menu::MenuShowsBinds);
			} EndChild();

			if (BeginWidgetTable(3, vTable))
			{
				// Watermark + Media Player options moved to the Visuals > MENU subtab
				// (they were unusable squeezed into this bind-window widget column).
			} EndChild();
		} EndSection();
		if (Section("Binds"))
		{
			static int iBind = DEFAULT_BIND;
			static Bind_t tBind = {};

			static int bParent = false;
			if (bParent)
				SetMouseCursor(ImGuiMouseCursor_Hand);

			auto vTable = WidgetTable(2, H::Draw.Scale(104));

			if (BeginWidgetTable(0, vTable))
			{
				FSDropdown("Name", &tBind.m_sName, {}, FDropdownEnum::Left | FSDropdownEnum::AutoUpdate);
				{
					auto sParent = bParent ? "..." : tBind.m_iParent != DEFAULT_BIND && tBind.m_iParent < F::Binds.m_vBinds.size() ? F::Binds.m_vBinds[tBind.m_iParent].m_sName : "None";
					if (FButton(std::format("Parent: {}", sParent).c_str(), FButtonEnum::Right | FButtonEnum::SameLine | FButtonEnum::NoUpper, { 0, 40 }))
						bParent = 2;
				}
				FDropdown("Type", &tBind.m_iType, { "Key", "Class", "Weapon type", "Item slot" }, {}, FDropdownEnum::Left);
				switch (tBind.m_iType)
				{
				case BindEnum::Key: tBind.m_iInfo = std::clamp(tBind.m_iInfo, 0, 2); FDropdown("Behavior", &tBind.m_iInfo, { "Hold", "Toggle", "Double click" }, {}, FDropdownEnum::Right); break;
				case BindEnum::Class: tBind.m_iInfo = std::clamp(tBind.m_iInfo, 0, 8); FDropdown("Class", &tBind.m_iInfo, { "Scout", "Soldier", "Pyro", "Demoman", "Heavy", "Engineer", "Medic", "Sniper", "Spy" }, {}, FDropdownEnum::Right); break;
				case BindEnum::WeaponType: tBind.m_iInfo = std::clamp(tBind.m_iInfo, 0, 3); FDropdown("Weapon type", &tBind.m_iInfo, { "Hitscan", "Projectile", "Melee", "Throwable" }, {}, FDropdownEnum::Right); break;
				case BindEnum::ItemSlot: tBind.m_iInfo = std::max(tBind.m_iInfo, 0); FDropdown("Item slot", &tBind.m_iInfo, { "1", "2", "3", "4", "5", "6", "7", "8", "9" }, {}, FDropdownEnum::Right); break;
				}
			} EndChild();

			if (BeginWidgetTable(1, vTable))
			{
				int iNot = tBind.m_bNot;
				FDropdown("While", &iNot, { "Active", "Not active" }, {}, FDropdownEnum::Left);
				tBind.m_bNot = iNot;
				FDropdown("Visibility", &tBind.m_iVisibility, { "Always", "While active", "Hidden" }, {}, FDropdownEnum::Right);
				if (tBind.m_iType == 0)
					FKeybind("Key", tBind.m_iKey, FButtonEnum::None, { Vars::Menu::PrimaryKey[DEFAULT_BIND], Vars::Menu::SecondaryKey[DEFAULT_BIND] }, { 0, 40 }, -96);

				// create/modify button
				bool bCreate = false, bClear = false, bParent = true;
				if (tBind.m_iParent != DEFAULT_BIND)
					bParent = F::Binds.m_vBinds.size() > tBind.m_iParent;

				SetCursorPos({ GetWindowWidth() - H::Draw.Scale(96), H::Draw.Scale(56) });
				PushDisabled(!bParent || !(tBind.m_iType == BindEnum::Key ? tBind.m_iKey : true));
				{
					bool bMatch = iBind != DEFAULT_BIND && F::Binds.m_vBinds.size() > iBind;
					bCreate = FButton(bMatch ? ICON_MD_SETTINGS : ICON_MD_ADD, FButtonEnum::None, { 40, 40 }, 0, F::Render.IconFont);
				}
				PopDisabled();

				// clear button
				SetCursorPos({ GetWindowWidth() - H::Draw.Scale(48), H::Draw.Scale(56) });
				bClear = FButton(ICON_MD_CLEAR, FButtonEnum::None, { 40, 40 }, 0, F::Render.IconFont);

				if (bCreate)
					F::Binds.AddBind(iBind, tBind);
				if (bCreate || bClear)
				{
					iBind = DEFAULT_BIND;
					tBind = {};
				}
			} EndChild();

			PushStyleColor(ImGuiCol_Text, F::Render.Inactive.Value);
			SetCursorPos({ H::Draw.Scale(13), H::Draw.Scale(128) });
			FText("Binds");
			SetCursorPosY(GetCursorPosY() - H::Draw.Scale(5));
			PopStyleColor();

			auto numberToIndex = [](int iNumber, int iLayer)
				{
					int iIndex = -1, i = -1;

					std::unordered_map<int, bool> mBinds = {};
					std::function<void(int)> getBinds = [&](int iParent)
						{
							for (int _iBind = 0; _iBind < F::Binds.m_vBinds.size(); _iBind++)
							{
								auto& _tBind = F::Binds.m_vBinds[_iBind];
								if (iParent != _tBind.m_iParent || mBinds.contains(_iBind))
									continue;

								mBinds[_iBind];

								i++;
								//if (iParent == iLayer && iNumber >= i)
								if (iIndex == -1 && iParent == iLayer && iNumber <= i)
									iIndex = _iBind;
								getBinds(_iBind);
							}
						};
					getBinds(DEFAULT_BIND);

					return iIndex;
				};
			auto positionToIndex = [&](ImVec2 vPos, int iLayer = DEFAULT_BIND)
				{
					int iIndex = floorf((vPos.y - GetCursorPosY() - H::Draw.Scale(4)) / H::Draw.Scale(36));
					iIndex = std::clamp(iIndex, 0, int(F::Binds.m_vBinds.size() - 1));
					iIndex = numberToIndex(iIndex, iLayer);
					return iIndex;
				};

			static int iDragging = -1, iLayer = DEFAULT_BIND;
			if (!IsMouseDown(ImGuiMouseButton_Left))
				iDragging = -1;
			else if (iDragging != -1)
			{
				int iTo = positionToIndex(GetMousePos() - GetDrawPos(), iLayer);
				if (iTo != -1 && iDragging != iTo)
				{
					F::Binds.Move(iDragging, iTo);
					iDragging = iTo;
				}
			}

			std::unordered_map<int, bool> mBinds = {};
			std::function<void(int, int)> getBinds = [&](int iParent, int x)
				{
					for (int _iBind = 0; _iBind < F::Binds.m_vBinds.size(); _iBind++)
					{
						auto& _tBind = F::Binds.m_vBinds[_iBind];
						if (iParent != DEFAULT_BIND - 1 && iParent != _tBind.m_iParent || mBinds.contains(_iBind))
							continue;

						mBinds[_iBind];

						std::string sType; std::string sInfo;
						switch (_tBind.m_iType)
						{
						case BindEnum::Key:
							switch (_tBind.m_iInfo)
							{
							case BindEnum::KeyEnum::Hold: { sType = "hold"; break; }
							case BindEnum::KeyEnum::Toggle: { sType = "toggle"; break; }
							case BindEnum::KeyEnum::DoubleClick: { sType = "double"; break; }
							}
							sInfo = VK2STR(_tBind.m_iKey);
							break;
						case BindEnum::Class:
							sType = "class";
							switch (_tBind.m_iInfo)
							{
							case BindEnum::ClassEnum::Scout: { sInfo = "scout"; break; }
							case BindEnum::ClassEnum::Soldier: { sInfo = "soldier"; break; }
							case BindEnum::ClassEnum::Pyro: { sInfo = "pyro"; break; }
							case BindEnum::ClassEnum::Demoman: { sInfo = "demoman"; break; }
							case BindEnum::ClassEnum::Heavy: { sInfo = "heavy"; break; }
							case BindEnum::ClassEnum::Engineer: { sInfo = "engineer"; break; }
							case BindEnum::ClassEnum::Medic: { sInfo = "medic"; break; }
							case BindEnum::ClassEnum::Sniper: { sInfo = "sniper"; break; }
							case BindEnum::ClassEnum::Spy: { sInfo = "spy"; break; }
							}
							break;
						case BindEnum::WeaponType:
							sType = "weapon";
							switch (_tBind.m_iInfo)
							{
							case BindEnum::WeaponTypeEnum::Hitscan: { sInfo = "hitscan"; break; }
							case BindEnum::WeaponTypeEnum::Projectile: { sInfo = "projectile"; break; }
							case BindEnum::WeaponTypeEnum::Melee: { sInfo = "melee"; break; }
							case BindEnum::WeaponTypeEnum::Throwable: { sInfo = "throwable"; break; }
							}
							break;
						case BindEnum::ItemSlot:
							sType = "slot";
							sInfo = std::format("{}", _tBind.m_iInfo + 1);
							break;
						}
						if (_tBind.m_bNot && (_tBind.m_iType != BindEnum::Key || _tBind.m_iInfo == BindEnum::KeyEnum::Hold))
							sType = std::format("not {}", sType);

						ImVec2 vOriginalPos = { H::Draw.Scale(8) + H::Draw.Scale(28) * std::min(x, 3), GetCursorPosY() + H::Draw.Scale(8) };

						// background
						float flWidth = GetWindowWidth() - GetStyle().WindowPadding.x * 2 - H::Draw.Scale(28) * std::min(x, 3);
						float flHeight = H::Draw.Scale(28);
						ImVec2 vDrawPos = GetDrawPos() + vOriginalPos;
						if (iBind != _iBind)
							GetWindowDrawList()->AddRectFilled(vDrawPos, { vDrawPos.x + flWidth, vDrawPos.y + flHeight }, F::Render.Background1p5, H::Draw.Scale(4));
						else
						{
							ImColor tColor = F::Render.Background1p5L;
							GetWindowDrawList()->AddRectFilled(vDrawPos, { vDrawPos.x + flWidth, vDrawPos.y + flHeight }, tColor, H::Draw.Scale(4));

							tColor = ColorToVec((VecToColor(F::Render.Background1p5)).Lerp({ 127, 127, 127 }, 1.f / 9, LerpEnum::NoAlpha));
							float flInset = H::Draw.Scale(0.5f) - 0.5f;
							GetWindowDrawList()->AddRect({ vDrawPos.x + flInset, vDrawPos.y + flInset }, { vDrawPos.x - flInset + flWidth, vDrawPos.y - flInset + flHeight }, tColor, H::Draw.Scale(4), ImDrawFlags_None, H::Draw.Scale());
						}

						// text
						if (x > 3)
						{	// don't indent too much
							auto sText = std::format("-> {}", x);
							SetCursorPos({ vOriginalPos.x - FCalcTextSize(sText.c_str()).x - H::Draw.Scale(10), vOriginalPos.y + H::Draw.Scale(7) });
							FText(sText.c_str());
						}

						float flTextWidth = flWidth - H::Draw.Scale(127);
						PushTransparent(!F::Binds.WillBeEnabled(_iBind), true);

						SetCursorPos({ vOriginalPos.x + H::Draw.Scale(9), vOriginalPos.y + H::Draw.Scale(7) });
						FText(TruncateText(_tBind.m_sName, flTextWidth * (1.f / 3) - H::Draw.Scale(20)).c_str());

						SetCursorPos({ vOriginalPos.x + flTextWidth * (1.f / 3), vOriginalPos.y + H::Draw.Scale(7) });
						FText(sType.c_str());

						SetCursorPos({ vOriginalPos.x + flTextWidth * (2.f / 3), vOriginalPos.y + H::Draw.Scale(7) });
						FText(sInfo.c_str());

						// buttons
						int iOffset = 1;

						SetCursorPos({ vOriginalPos.x + flWidth - H::Draw.Scale(iOffset += 25), vOriginalPos.y + H::Draw.Scale(2) });
						bool bDelete = IconButton(ICON_MD_DELETE);

						SetCursorPos({ vOriginalPos.x + flWidth - H::Draw.Scale(iOffset += 25), vOriginalPos.y + H::Draw.Scale(2) });
						if (IconButton(ICON_MD_EDIT))
							CurrentBind = CurrentBind != _iBind ? _iBind : DEFAULT_BIND;

						SetCursorPos({ vOriginalPos.x + flWidth - H::Draw.Scale(iOffset += 25), vOriginalPos.y + H::Draw.Scale(2) });
						if (IconButton(!_tBind.m_bNot ? ICON_MD_CODE : ICON_MD_CODE_OFF))
							_tBind.m_bNot = !_tBind.m_bNot;

						PushTransparent(Transparent || _tBind.m_iVisibility == BindVisibilityEnum::Hidden, true);
						SetCursorPos({ vOriginalPos.x + flWidth - H::Draw.Scale(iOffset += 25), vOriginalPos.y + H::Draw.Scale(2) });
						if (IconButton(_tBind.m_iVisibility == BindVisibilityEnum::Always ? ICON_MD_VISIBILITY : ICON_MD_VISIBILITY_OFF))
							_tBind.m_iVisibility = (_tBind.m_iVisibility + 1) % 3;
						PopTransparent(1, 1);

						SetCursorPos({ vOriginalPos.x + flWidth - H::Draw.Scale(iOffset += 25), vOriginalPos.y + H::Draw.Scale(2) });
						if (IconButton(_tBind.m_bEnabled ? ICON_MD_TOGGLE_ON : ICON_MD_TOGGLE_OFF))
							_tBind.m_bEnabled = !_tBind.m_bEnabled;

						SetCursorPos(vOriginalPos);
						bool bClicked = Button(std::format("##{}", _iBind).c_str(), { flWidth, flHeight });
						bool bPopup = IsItemClicked(ImGuiMouseButton_Right);

						PopTransparent(1, 1);

						if (bClicked)
						{
							if (!bParent)
							{
								iBind = _iBind;
								tBind = _tBind;
							}
							else
							{
								bParent = false;
								tBind.m_iParent = _iBind;

								// make sure bind can't be parented to itself or any of its children
								int _iBind2 = _iBind;
								Bind_t _tBind2;
								while (F::Binds.GetBind(_iBind2, &_tBind2))
								{
									if (_iBind2 == iBind)
										tBind.m_iParent = DEFAULT_BIND;
									_iBind2 = _tBind2.m_iParent;
								}
							}
						}
						else if (bPopup)
							OpenPopup(std::format("RightClicked{}", _iBind).c_str());
						else if (iDragging == -1 && IsItemHovered() && IsMouseDown(ImGuiMouseButton_Left))
							iDragging = _iBind, iLayer = iParent;
						else if (bDelete)
						{
							if (U::KeyHandler.Down(VK_SHIFT)) // allow user to quickly remove binds
								F::Binds.RemoveBind(_iBind);
							else
								OpenPopup(std::format("DeleteBind{}", _iBind).c_str());
						}

						if (FBeginPopup(std::format("RightClicked{}", _iBind).c_str()))
						{
							PushStyleVar(ImGuiStyleVar_ItemSpacing, { H::Draw.Scale(8), 0 });

							{
								static std::string sInput = "";

								bool bEnter = FInputText("Name...", sInput, H::Draw.Scale(284), ImGuiInputTextFlags_EnterReturnsTrue);
								if (!IsItemFocused())
									sInput = _tBind.m_sName;
								if (bEnter)
									_tBind.m_sName = sInput;
							}

							FDropdown("Type", &_tBind.m_iType, { "Key", "Class", "Weapon type", "Item slot" }, {}, FDropdownEnum::Left);
							switch (_tBind.m_iType)
							{
							case BindEnum::Key: _tBind.m_iInfo = std::clamp(_tBind.m_iInfo, 0, 2); FDropdown("Behavior", &_tBind.m_iInfo, { "Hold", "Toggle", "Double click" }, {}, FDropdownEnum::Right); break;
							case BindEnum::Class: _tBind.m_iInfo = std::clamp(_tBind.m_iInfo, 0, 8); FDropdown("Class", &_tBind.m_iInfo, { "Scout", "Soldier", "Pyro", "Demoman", "Heavy", "Engineer", "Medic", "Sniper", "Spy" }, {}, FDropdownEnum::Right); break;
							case BindEnum::WeaponType: _tBind.m_iInfo = std::clamp(_tBind.m_iInfo, 0, 3); FDropdown("Weapon type", &_tBind.m_iInfo, { "Hitscan", "Projectile", "Melee", "Throwable" }, {}, FDropdownEnum::Right); break;
							case BindEnum::ItemSlot: _tBind.m_iInfo = std::max(_tBind.m_iInfo, 0); FDropdown("Item slot", &_tBind.m_iInfo, { "1", "2", "3", "4", "5", "6", "7", "8", "9" }, {}, FDropdownEnum::Right); break;
							}
							if (_tBind.m_iType == BindEnum::Key)
								FKeybind("Key", _tBind.m_iKey);

							PopStyleVar();
							EndPopup();
						}
						else if (FBeginPopupModal(std::format("DeleteBind{}", _iBind).c_str(), nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysUseWindowPadding))
						{
							FText(std::format("Do you really want to delete '{}'{}?", _tBind.m_sName, F::Binds.HasChildren(_iBind) ? " and all of its children" : "").c_str());

							if (FButton("Yes", FButtonEnum::Left))
							{
								F::Binds.RemoveBind(_iBind);
								CloseCurrentPopup();

								iBind = DEFAULT_BIND;
								tBind = {};
							}
							if (FButton("No", FButtonEnum::Right | FButtonEnum::SameLine))
								CloseCurrentPopup();

							EndPopup();
						}

						if (iParent != DEFAULT_BIND - 1)
							getBinds(_iBind, x + 1);
					}
				};
			getBinds(DEFAULT_BIND, 0);

			// this should ideally never happen, but failsafe
			if (F::Binds.m_vBinds.size() > mBinds.size())
			{
				PushStyleColor(ImGuiCol_Text, F::Render.Inactive.Value);
				SetCursorPos({ H::Draw.Scale(13), GetCursorPosY() + H::Draw.Scale(5) });
				FText("Dangling");
				SetCursorPosY(GetCursorPosY() - H::Draw.Scale(5));
				PopStyleColor();

				getBinds(DEFAULT_BIND - 1, 0);
			}

			if (bParent == 2) // dumb
				bParent = 1;
			else if (bParent && IsMouseReleased(ImGuiMouseButton_Left))
			{
				bParent = false;
				tBind.m_iParent = DEFAULT_BIND;
			}
		} EndSection();
		break;
	}
	// Materials
	case 2:
	{
		static TextEditor TextEditor;
		static std::string CurrentMaterial;
		static bool LockedMaterial;

		bool bTable = false;
		if (!CurrentMaterial.empty())
			bTable = BeginTable("MaterialsTable", 2);
		{
			if (bTable)
			{
				TableSetupColumn("MaterialsTable1", ImGuiTableColumnFlags_WidthFixed, H::Draw.Scale(288));
				TableSetupColumn("MaterialsTable2", ImGuiTableColumnFlags_WidthFixed, GetWindowWidth());

				/* Column 1 */
				TableNextColumn();
			}

			if (Section("Manager"))
			{
				static std::string sStaticName;

				FSDropdown("Name", &sStaticName, {}, FSDropdownEnum::AutoUpdate, -H::Draw.Unscale(FCalcTextSize("CREATE").x + FCalcTextSize("FOLDER").x) - 72);
				PushDisabled(sStaticName.empty());
				{
					if (FButton("Create", FButtonEnum::Fit | FButtonEnum::SameLine, { 0, 40 }))
					{
						F::Materials.AddMaterial(sStaticName.c_str());
						sStaticName.clear();
					}
				}
				PopDisabled();
				if (FButton("Folder", FButtonEnum::Fit | FButtonEnum::SameLine, { 0, 40 }))
					ShellExecuteA(NULL, NULL, F::Configs.m_sMaterialsPath.c_str(), NULL, NULL, SW_SHOWNORMAL);

				std::vector<Material_t> vMaterials;
				for (auto& [_, mat] : F::Materials.m_mMaterials)
					vMaterials.push_back(mat);

				std::sort(vMaterials.begin(), vMaterials.end(), [&](const auto& a, const auto& b) -> bool
					{
						// override for none material
						if (FNV1A::Hash32(a.m_sName.c_str()) == FNV1A::Hash32Const("None"))
							return true;
						if (FNV1A::Hash32(b.m_sName.c_str()) == FNV1A::Hash32Const("None"))
							return false;

						// keep locked materials higher
						if (a.m_bLocked && !b.m_bLocked)
							return true;
						if (!a.m_bLocked && b.m_bLocked)
							return false;

						return a.m_sName < b.m_sName;
					});

				for (auto& tMaterial : vMaterials)
				{
					ImVec2 vOriginalPos = GetCursorPos();

					SetCursorPos({ H::Draw.Scale(17), vOriginalPos.y + H::Draw.Scale(14) });
					TextColored(tMaterial.m_bLocked ? F::Render.Inactive.Value : F::Render.Active.Value, TruncateText(tMaterial.m_sName, GetWindowWidth() - GetStyle().WindowPadding.x * 2 - H::Draw.Scale(56)).c_str());

					int iOffset = 9;
					if (!tMaterial.m_bLocked)
					{
						SetCursorPos({ GetWindowWidth() - H::Draw.Scale(iOffset += 25), vOriginalPos.y + H::Draw.Scale(9) });
						if (IconButton(ICON_MD_DELETE))
							OpenPopup(std::format("DeleteMat{}", tMaterial.m_sName).c_str());
						if (FBeginPopupModal(std::format("DeleteMat{}", tMaterial.m_sName).c_str(), nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysUseWindowPadding))
						{
							FText(std::format("Do you really want to delete '{}'?", tMaterial.m_sName).c_str());

							if (FButton("Yes", FButtonEnum::Left))
							{
								F::Materials.RemoveMaterial(tMaterial.m_sName.c_str());
								CloseCurrentPopup();
							}
							if (FButton("No", FButtonEnum::Right | FButtonEnum::SameLine))
								CloseCurrentPopup();

							EndPopup();
						}
					}

					SetCursorPos({ GetWindowWidth() - H::Draw.Scale(iOffset += 25), vOriginalPos.y + H::Draw.Scale(9) });
					if (IconButton(ICON_MD_EDIT))
					{
						CurrentMaterial = tMaterial.m_sName;
						LockedMaterial = tMaterial.m_bLocked;

						TextEditor.SetText(F::Materials.GetVMT(FNV1A::Hash32(CurrentMaterial.c_str())));
						TextEditor.SetReadOnlyEnabled(LockedMaterial);
					}

					SetCursorPos(vOriginalPos); DebugDummy({ 0, H::Draw.Scale(28) });
				}
				DebugDummy({ 0, H::Draw.Scale(7) });
			}
			else
				SetScrollY(0);
			EndSection();

			/* Column 2 */
			if (bTable)
			{
				TableNextColumn();
				if (CurrentMaterial.length())
				{
					SetCursorPosY(GetScrollY() + GetStyle().WindowPadding.y);
					if (Section("Editor", 0, GetWindowHeight() - GetStyle().WindowPadding.y * 2, true))
					{
						// Toolbar
						if (!LockedMaterial)
						{
							if (FButton("Save", FButtonEnum::Fit))
							{
								auto sText = TextEditor.GetText();
								F::Materials.EditMaterial(CurrentMaterial.c_str(), sText.c_str());
							}
							SameLine();
						}
						if (FButton("Close", FButtonEnum::Fit))
							CurrentMaterial = "";
						SetCursorPosY(H::Draw.Scale(52));
						PushStyleColor(ImGuiCol_Text, F::Render.Inactive.Value);
						FText(std::format("{}: {}", LockedMaterial ? "Viewing" : "Editing", CurrentMaterial).c_str(), FTextEnum::Right);
						PopStyleColor();

						// Text editor
						DebugDummy({ 0, H::Draw.Scale(8) });

						TextEditor.SetLanguageDefinition(TextEditor::LanguageDefinitionId::Cpp);
						TextEditor.SetPaletteIndex(TextEditor::PaletteIndex::Background, F::Render.Background1);
						TextEditor.SetPaletteIndex(TextEditor::PaletteIndex::Default, F::Render.Active);
						TextEditor.SetPaletteIndex(TextEditor::PaletteIndex::Identifier, F::Render.Active);
						TextEditor.SetPaletteIndex(TextEditor::PaletteIndex::Cursor, F::Render.Active);
						TextEditor.SetPaletteIndex(TextEditor::PaletteIndex::LineNumber, F::Render.Inactive);
						TextEditor.SetPaletteIndex(TextEditor::PaletteIndex::Comment, F::Render.Inactive);
						TextEditor.SetPaletteIndex(TextEditor::PaletteIndex::MultiLineComment, F::Render.Inactive);
						TextEditor.SetPaletteIndex(TextEditor::PaletteIndex::Punctuation, F::Render.Inactive);
						TextEditor.SetPaletteIndex(TextEditor::PaletteIndex::ControlCharacter, ImColor(F::Render.Inactive.Value.x, F::Render.Inactive.Value.y, F::Render.Inactive.Value.z, 0.1f));
						TextEditor.SetPaletteIndex(TextEditor::PaletteIndex::String, F::Render.Accent);
						
						PushFont(F::Render.FontMono);
						ImVec2 vDrawPos = GetDrawPos() + GetCursorPos();
						TextEditor.Render("TextEditor");
						ImVec2 vSize = GetItemRectSize();
						float flInset = H::Draw.Scale(0.5f) - 0.5f;
						GetWindowDrawList()->AddRect({ vDrawPos.x + flInset, vDrawPos.y + flInset }, { vDrawPos.x - flInset + vSize.x, vDrawPos.y - flInset + vSize.y }, F::Render.Background2, H::Draw.Scale(4), ImDrawFlags_None, H::Draw.Scale());
						PopFont();
					} EndSection();
				}

				EndTable();
			}
		}
		break;
	}
	// Extra
	case 3:
	{
		if (Section("Debug", 8))
		{
			FToggle(Vars::Debug::Info, FToggleEnum::Left);
			FToggle(Vars::Debug::Logging, FToggleEnum::Right);
			FToggle(Vars::Debug::Options, FToggleEnum::Left);
			FToggle(Vars::Debug::DrawHitboxes, FToggleEnum::Right);
			FToggle(Vars::Debug::AntiAimLines, FToggleEnum::Left);
			FToggle(Vars::Debug::CrashLogging, FToggleEnum::Right);
#ifdef DEBUG_TRACES
			FToggle(Vars::Debug::VisualizeTraces, FToggleEnum::Left);
			FToggle(Vars::Debug::VisualizeTraceHits, FToggleEnum::Right);
#endif
		} EndSection();
		if (Section("Buttons"))
		{
			if (FButton("cl_fullupdate", FButtonEnum::Left))
				I::EngineClient->ClientCmd_Unrestricted("cl_fullupdate");
			if (FButton("retry", FButtonEnum::Right | FButtonEnum::SameLine))
				I::EngineClient->ClientCmd_Unrestricted("retry");
			if (FButton("Console", FButtonEnum::Left))
				I::EngineClient->ClientCmd_Unrestricted("toggleconsole");
			if (FButton("Fix materials", FButtonEnum::Right | FButtonEnum::SameLine) && F::Materials.m_bLoaded)
				F::Materials.ReloadMaterials();

			if (!I::EngineClient->IsConnected())
			{
				if (FButton("Unlock achievements", FButtonEnum::Left))
					OpenPopup("UnlockAchievements");
				if (FButton("Lock achievements", FButtonEnum::Right | FButtonEnum::SameLine))
					OpenPopup("LockAchievements");

				if (FBeginPopupModal("UnlockAchievements", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysUseWindowPadding))
				{
					FText("Do you really want to unlock all achievements?");

					if (FButton("Yes, unlock", FButtonEnum::Left))
					{
						F::Misc.UnlockAchievements();
						CloseCurrentPopup();
					}
					if (FButton("No", FButtonEnum::Right | FButtonEnum::SameLine))
						CloseCurrentPopup();

					EndPopup();
				}
				else if (FBeginPopupModal("LockAchievements", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysUseWindowPadding))
				{
					FText("Do you really want to lock all achievements?");

					if (FButton("Yes, lock", FButtonEnum::Left))
					{
						F::Misc.LockAchievements();
						CloseCurrentPopup();
					}
					if (FButton("No", FButtonEnum::Right | FButtonEnum::SameLine))
						CloseCurrentPopup();

					EndPopup();
				}
			}

		} EndSection();
		if (Vars::Debug::Options.Value && I::EngineClient->IsConnected())
		{
			if (Section("##Debug", -8))
			{
				if (FButton("Restore lines", FButtonEnum::Left))
				{
					for (auto& tLine : G::LineStorage)
						tLine.m_flTime = I::GlobalVars->curtime + 60.f;
				}
				if (FButton("Restore paths", FButtonEnum::Right | FButtonEnum::SameLine))
				{
					for (auto& tPath : G::PathStorage)
						tPath.m_flTime = I::GlobalVars->curtime + 60.f;
				}
				if (FButton("Restore boxes", FButtonEnum::Left))
				{
					for (auto& tBox : G::BoxStorage)
						tBox.m_flTime = I::GlobalVars->curtime + 60.f;
				}
				if (FButton("Clear visuals", FButtonEnum::Right | FButtonEnum::SameLine))
				{
					G::LineStorage.clear();
					G::PathStorage.clear();
					G::BoxStorage.clear();
					G::SphereStorage.clear();
					G::SweptStorage.clear();
				}
			} EndSection();
		}
		/*
		if (Vars::Debug::Options.Value)
		{
			if (Section("Convar spoofer"))
			{
				static std::string sName = "", sValue = "";

				FSDropdown("Convar", &sName, {}, FDropdownEnum::Left);
				FSDropdown("Value", &sValue, {}, FDropdownEnum::Right);
				if (FButton("Send"))
				{
					if (auto pNetChan = static_cast<CNetChannel*>(I::EngineClient->GetNetChannelInfo()))
					{
						SDK::Output("Convar", std::format("Sent {} as {}", sName, sValue).c_str(), Vars::Menu::Theme::Accent.Value);
						NET_SetConVar cmd(sName.c_str(), sValue.c_str());
						pNetChan->SendNetMsg(cmd);

						sName = sValue = "";
					}
				}
			} EndSection();
		}
		*/
#ifdef DEBUG_HOOKS
		if (Section("Hooks", 8))
		{
			int i = 0; for (auto& pBase : G::Vars)
			{
				if (pBase->m_sName.find("Vars::Hooks::") == std::string::npos)
					continue;

				FToggle(*pBase->As<bool>(), !(i % 2) ? FToggleEnum::Left : FToggleEnum::Right);
				i++;
			}
		} EndSection();
#endif
		break;
	}
	}
}

void CMenu::MenuSearch(std::string sSearch)
{
	using namespace ImGui;

	if (sSearch.empty())
		return;

	static std::vector<BaseVar*> vVars = {}; // don't string search every single frame

	static uint32_t uStaticHash = 0;
	if (const uint32_t uCurrHash = FNV1A::Hash32(sSearch.c_str());
		uCurrHash != uStaticHash)
	{
		std::string sSearch2 = sSearch;
		std::transform(sSearch2.begin(), sSearch2.end(), sSearch2.begin(), ::tolower);

		vVars.clear();
		for (auto& pBase : G::Vars)
		{
			if (!Vars::Debug::Options[DEFAULT_BIND] && pBase->m_iFlags & DEBUGVAR)
				continue;

			std::vector<const char*> vSearch = { pBase->m_sName.c_str(), pBase->m_sSection };
			vSearch.insert(vSearch.end(), pBase->m_vTitle.begin(), pBase->m_vTitle.end());
			vSearch.insert(vSearch.end(), pBase->m_vValues.begin(), pBase->m_vValues.end());
			for (auto pSearch : vSearch)
			{
				std::string sSearch3 = pSearch;
				if (auto iFind = sSearch3.find("Vars::"); iFind != std::string::npos)
					sSearch3 = sSearch3.replace(iFind, strlen("Vars::"), "");
				if (auto iFind = sSearch3.find("##"); iFind != std::string::npos)
					sSearch3 = sSearch3.replace(iFind, strlen("##"), "");
				std::transform(sSearch3.begin(), sSearch3.end(), sSearch3.begin(), ::tolower);
				if (sSearch3.find(sSearch2) != std::string::npos)
				{
					vVars.push_back(pBase);
					break;
				}
			}
		}

		uStaticHash = uCurrHash;
	}

	if (vVars.empty())
		return;

	uint32_t uLastSection = 0;
	int i = 0; for (auto pBase : vVars) // possibly implement tablelike visuals, do away with left right, just switch if current side is higher than other?
	{
		int iWidgetEnum = WidgetEnum::Invalid, iTypeEnum = WidgetEnum::Invalid;
		if (auto pVar = pBase->As<bool>())
			iWidgetEnum = iTypeEnum = WidgetEnum::FToggle;
		else if (auto pVar = pBase->As<int>())
		{
			if (!pVar->m_vValues.empty()
				|| FNV1A::Hash32(pVar->m_sName.c_str()) == FNV1A::Hash32Const("Vars::ESP::ActiveGroups"))
				iWidgetEnum = iTypeEnum = WidgetEnum::FDropdown;
			else if (pVar->m_sExtra)
				iWidgetEnum = WidgetEnum::FISlider, iTypeEnum = WidgetEnum::FSlider;
			else
				iWidgetEnum = iTypeEnum = WidgetEnum::FKeybind;
		}
		else if (auto pVar = pBase->As<float>())
			iWidgetEnum = WidgetEnum::FFSlider, iTypeEnum = WidgetEnum::FSlider;
		else if (auto pVar = pBase->As<IntRange_t>())
			iWidgetEnum = WidgetEnum::FIRSlider, iTypeEnum = WidgetEnum::FSlider;
		else if (auto pVar = pBase->As<FloatRange_t>())
			iWidgetEnum = WidgetEnum::FFRSlider, iTypeEnum = WidgetEnum::FSlider;
		else if (auto pVar = pBase->As<std::string>())
			iWidgetEnum = WidgetEnum::FSDropdown, iTypeEnum = WidgetEnum::FDropdown;
		else if (auto pVar = pBase->As<std::vector<std::pair<std::string, Color_t>>>())
			iWidgetEnum = WidgetEnum::FMDropdown, iTypeEnum = WidgetEnum::FDropdown;
		else if (auto pVar = pBase->As<Color_t>())
			iWidgetEnum = WidgetEnum::FColorPicker, iTypeEnum = WidgetEnum::FToggle;
		else if (auto pVar = pBase->As<Gradient_t>())
			iWidgetEnum = WidgetEnum::FGColorPicker, iTypeEnum = WidgetEnum::FToggle;
		else
			continue;

		uint32_t uSection = FNV1A::Hash32(pBase->m_sSection);
		if (uSection != uLastSection)
		{
			if (uLastSection)
				EndSection();
			Section(std::format("{}## {}", pBase->m_sSection, pBase->m_sName).c_str());
			i = 0;
		}
		uLastSection = uSection;

		static int iLastEnum = WidgetEnum::Invalid;
		if (!i || iTypeEnum != iLastEnum)
		{
			if (!i)
			{
				switch (iWidgetEnum)
				{
				case WidgetEnum::FToggle:
				case WidgetEnum::FISlider:
				case WidgetEnum::FFSlider:
				case WidgetEnum::FIRSlider:
				case WidgetEnum::FFRSlider:
				case WidgetEnum::FColorPicker:
				case WidgetEnum::FGColorPicker:
					DebugDummy({ 0, H::Draw.Scale(8) });
				}
				i = 2;
			}
			else if (iTypeEnum == WidgetEnum::FToggle && iLastEnum == WidgetEnum::FSlider && (i % 2))
			{
				SetCursorPos({ GetWindowWidth() / 2 + GetStyle().WindowPadding.x / 2, GetRowPos() + H::Draw.Scale(8) });
				i = 0;
			}
			else if (iTypeEnum == WidgetEnum::FSlider && iLastEnum == WidgetEnum::FDropdown && (i % 2) && !vRowSizes.empty())
			{
				auto& tRow = vRowSizes.front();
				tRow.m_vPos.y += H::Draw.Scale(13), tRow.m_vSize.y -= H::Draw.Scale(13);
				SetCursorPos({ GetWindowWidth() / 2 + GetStyle().WindowPadding.x / 2, GetRowPos() });
				i = 0;
			}
			else
				i = 2;
		}
		iLastEnum = iTypeEnum;

		int iOverride = -1;
		switch (iWidgetEnum)
		{
		case WidgetEnum::FToggle:
		{
			auto pVar = pBase->As<bool>();
			if (FToggle(*pVar, !(i % 2) ? FToggleEnum::Left : FToggleEnum::Right, nullptr, iOverride/*, iOverride*/))
			{
				if (FNV1A::Hash32(pVar->m_sName.c_str()) == FNV1A::Hash32Const("Vars::Debug::Options"))
					uStaticHash = 0;
			}
			break;
		}
		case WidgetEnum::FISlider:
		{
			auto pVar = pBase->As<int>();
			FSlider(*pVar, !(i % 2) ? FSliderEnum::Left : FSliderEnum::Right, nullptr, nullptr, iOverride/*, iOverride*/);
			break;
		}
		case WidgetEnum::FFSlider:
		{
			auto pVar = pBase->As<float>();
			const char* sFormat = pVar->m_sExtra;
			switch (FNV1A::Hash32(pVar->m_sName.c_str()))
			{
			case FNV1A::Hash32Const("Vars::Aimbot::Projectile::SplashRotateX"):
			case FNV1A::Hash32Const("Vars::Aimbot::Projectile::SplashRotateY"):
				if (pVar->Map[DEFAULT_BIND] < 0.f)
					sFormat = "random";
			}
			FSlider(*pVar, !(i % 2) ? FSliderEnum::Left : FSliderEnum::Right, sFormat, nullptr, iOverride/*, iOverride*/);
			break;
		}
		case WidgetEnum::FIRSlider:
		{
			auto pVar = pBase->As<IntRange_t>();
			FSlider(*pVar, !(i % 2) ? FSliderEnum::Left : FSliderEnum::Right, nullptr, nullptr, iOverride/*, iOverride*/);
			break;
		}
		case WidgetEnum::FFRSlider:
		{
			auto pVar = pBase->As<FloatRange_t>();
			FSlider(*pVar, !(i % 2) ? FSliderEnum::Left : FSliderEnum::Right, nullptr, nullptr, iOverride/*, iOverride*/);
			break;
		}
		case WidgetEnum::FDropdown:
		{
			auto pVar = pBase->As<int>();
			FDropdown(*pVar, !(i % 2) ? FDropdownEnum::Left : FDropdownEnum::Right, 0, nullptr, iOverride/*, iOverride*/);
			break;
		}
		case WidgetEnum::FSDropdown:
		{
			auto pVar = pBase->As<std::string>();
			FSDropdown(*pVar, !(i % 2) ? FDropdownEnum::Left : FDropdownEnum::Right, 0, nullptr, iOverride/*, iOverride*/);
			break;
		}
		case WidgetEnum::FMDropdown:
		{
			auto pVar = pBase->As<std::vector<std::pair<std::string, Color_t>>>();
			FMDropdown(*pVar, !(i % 2) ? FDropdownEnum::Left : FDropdownEnum::Right, 0, nullptr, iOverride/*, iOverride*/);
			break;
		}
		case WidgetEnum::FColorPicker:
		{
			auto pVar = pBase->As<Color_t>();
			FColorPicker(*pVar, !(i % 2) ? FColorPickerEnum::Left : FColorPickerEnum::Right, {}, { H::Draw.Scale(12), H::Draw.Scale(12) }, {}, nullptr, iOverride, iOverride);
			break;
		}
		case WidgetEnum::FGColorPicker:
		{
			auto pVar = pBase->As<Gradient_t>();
			FColorPicker(*pVar, true, !(i % 2) ? FColorPickerEnum::Left : FColorPickerEnum::Right, {}, { H::Draw.Scale(12), H::Draw.Scale(12) }, {}, nullptr, iOverride/*, iOverride*/);
			FColorPicker(*pVar, false, !(++i % 2) ? FColorPickerEnum::Left : FColorPickerEnum::Right, {}, { H::Draw.Scale(12), H::Draw.Scale(12) }, {}, nullptr, iOverride/*, iOverride*/);
			break;
		}
		case WidgetEnum::FKeybind:
		{
			auto pVar = pBase->As<int>();
			std::vector<int> vIgnore;
			switch (FNV1A::Hash32(pVar->m_sName.c_str()))
			{
			case FNV1A::Hash32Const("Vars::Menu::PrimaryKey"):
				vIgnore = { Vars::Menu::SecondaryKey[DEFAULT_BIND], VK_LBUTTON, VK_RBUTTON };
				break;
			case FNV1A::Hash32Const("Vars::Menu::SecondaryKey"):
				vIgnore = { Vars::Menu::PrimaryKey[DEFAULT_BIND], VK_LBUTTON, VK_RBUTTON };
				break;
			default:
				vIgnore = { Vars::Menu::PrimaryKey[DEFAULT_BIND], Vars::Menu::SecondaryKey[DEFAULT_BIND] };
			}
			FKeybind(iOverride != -1 ? pVar->m_vTitle[iOverride] : pVar->m_vTitle.front(), pVar->Map[DEFAULT_BIND], !(i % 2) ? FButtonEnum::Left : FButtonEnum::Right | FButtonEnum::SameLine, vIgnore);
			break;
		}
		}

		if (iOverride != -2)
			i += i > 1 ? 1 : 2;
	}
	if (uLastSection)
		EndSection();
}
#pragma endregion

struct DragBoxStorage_t
{
	DragBox_t m_tDragBox;
	float m_flScale;
};
static std::unordered_map<uint32_t, DragBoxStorage_t> s_mDragBoxStorage = {};
void CMenu::AddDraggable(const char* sLabel, ConfigVar<DragBox_t>& var, bool bShouldDraw, ImVec2 vSize)
{
	using namespace ImGui;

	if (!bShouldDraw)
		return;

	auto tDragBox = FGet(var, true);
	auto uHash = FNV1A::Hash32(sLabel);

	bool bContains = s_mDragBoxStorage.contains(uHash);
	auto& tStorage = s_mDragBoxStorage[uHash];

	SetNextWindowSize(vSize, ImGuiCond_Always);
	if (!bContains || tDragBox != tStorage.m_tDragBox || H::Draw.Scale() != tStorage.m_flScale)
		SetNextWindowPos({ float(tDragBox.x - vSize.x / 2), float(tDragBox.y) }, ImGuiCond_Always);

	PushStyleColor(ImGuiCol_WindowBg, {});
	PushStyleColor(ImGuiCol_Border, F::Render.Active.Value);
	PushStyleVar(ImGuiStyleVar_WindowRounding, H::Draw.Scale(3));
	PushStyleVar(ImGuiStyleVar_WindowBorderSize, H::Draw.Scale(1));
	PushStyleVar(ImGuiStyleVar_WindowMinSize, vSize);
	if (Begin(sLabel, nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoFocusOnAppearing))
	{
		ImVec2 vWindowPos = GetWindowPos();

		tDragBox.x = vWindowPos.x + vSize.x / 2, tDragBox.y = vWindowPos.y;
		tStorage = { tDragBox, H::Draw.Scale() };
		FSet(var, tDragBox);

		PushFont(F::Render.FontBold);
		ImVec2 vTextSize = FCalcTextSize(sLabel);
		SetCursorPos({ (vSize.x - vTextSize.x) * 0.5f, (vSize.y - vTextSize.y) * 0.5f });
		FText(sLabel);
		PopFont();

		End();
	}
	PopStyleVar(3);
	PopStyleColor(2);
}

struct WindowBoxStorage_t
{
	WindowBox_t m_tWindowBox;
	float m_flScale;
};
static std::unordered_map<uint32_t, WindowBoxStorage_t> s_mWindowBoxStorage = {};
void CMenu::AddResizableDraggable(const char* sLabel, ConfigVar<WindowBox_t>& var, bool bShouldDraw, ImVec2 vMinSize, ImVec2 vMaxSize, ImGuiSizeCallback fCustomCallback)
{
	using namespace ImGui;

	if (!bShouldDraw)
		return;

	auto tWindowBox = FGet(var, true);
	auto uHash = FNV1A::Hash32(sLabel);

	bool bContains = s_mWindowBoxStorage.contains(uHash);
	auto& tStorage = s_mWindowBoxStorage[uHash];

	SetNextWindowSizeConstraints(vMinSize, vMaxSize, fCustomCallback);
	if (!bContains || tWindowBox != tStorage.m_tWindowBox || H::Draw.Scale() != tStorage.m_flScale)
	{
		SetNextWindowPos({ float(tWindowBox.x - tWindowBox.w / 2), float(tWindowBox.y) }, ImGuiCond_Always);
		SetNextWindowSize({ float(tWindowBox.w), float(tWindowBox.h) }, ImGuiCond_Always);
	}

	PushStyleColor(ImGuiCol_WindowBg, {});
	PushStyleColor(ImGuiCol_Border, F::Render.Active.Value);
	PushStyleVar(ImGuiStyleVar_WindowRounding, H::Draw.Scale(3));
	PushStyleVar(ImGuiStyleVar_WindowBorderSize, H::Draw.Scale(1));
	if (Begin(sLabel, nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoFocusOnAppearing))
	{
		ImVec2 vWindowPos = GetWindowPos();
		ImVec2 vWinSize = GetWindowSize();

		tWindowBox.w = vWinSize.x, tWindowBox.h = vWinSize.y;
		tWindowBox.x = vWindowPos.x + tWindowBox.w / 2, tWindowBox.y = vWindowPos.y;
		tStorage = { tWindowBox, H::Draw.Scale() };
		FSet(var, tWindowBox);

		PushFont(F::Render.FontBold);
		ImVec2 vTextSize = FCalcTextSize(sLabel);
		SetCursorPos({ (vWinSize.x - vTextSize.x) * 0.5f, (vWinSize.y - vTextSize.y) * 0.5f });
		FText(sLabel);
		PopFont();

		End();
	}
	PopStyleVar(2);
	PopStyleColor(2);
}

struct BindInfo_t
{
	const char* sName;
	std::string sInfo;
	std::string sState;

	int iBind;
	Bind_t& tBind;
};
void CMenu::DrawBinds()
{
	using namespace ImGui;

	if (!Vars::Menu::BindWindow.Value) // disabled: never draw, even while the menu is open
		return;
	if (!m_bIsOpen && (I::EngineVGui->IsGameUIVisible() || I::MatSystemSurface->IsCursorVisible() && !I::EngineClient->IsPlayingDemo()))
		return;

	std::vector<BindInfo_t> vInfo;
	std::function<void(int)> getBinds = [&](int iParent)
		{
			for (int iBind = 0; iBind < F::Binds.m_vBinds.size(); iBind++)
			{
				auto& tBind = F::Binds.m_vBinds[iBind];
				if (iParent != tBind.m_iParent || !tBind.m_bEnabled && !m_bIsOpen)
					continue;

				if (tBind.m_iVisibility == BindVisibilityEnum::Always || tBind.m_iVisibility == BindVisibilityEnum::WhileActive && tBind.m_bActive || m_bIsOpen)
				{
					std::string sType; std::string sInfo;
					switch (tBind.m_iType)
					{
					case BindEnum::Key:
						switch (tBind.m_iInfo)
						{
						case BindEnum::KeyEnum::Hold: { sType = "hold"; break; }
						case BindEnum::KeyEnum::Toggle: { sType = "toggle"; break; }
						case BindEnum::KeyEnum::DoubleClick: { sType = "double"; break; }
						}
						sInfo = VK2STR(tBind.m_iKey);
						break;
					case BindEnum::Class:
						sType = "class";
						switch (tBind.m_iInfo)
						{
						case BindEnum::ClassEnum::Scout: { sInfo = "scout"; break; }
						case BindEnum::ClassEnum::Soldier: { sInfo = "soldier"; break; }
						case BindEnum::ClassEnum::Pyro: { sInfo = "pyro"; break; }
						case BindEnum::ClassEnum::Demoman: { sInfo = "demoman"; break; }
						case BindEnum::ClassEnum::Heavy: { sInfo = "heavy"; break; }
						case BindEnum::ClassEnum::Engineer: { sInfo = "engineer"; break; }
						case BindEnum::ClassEnum::Medic: { sInfo = "medic"; break; }
						case BindEnum::ClassEnum::Sniper: { sInfo = "sniper"; break; }
						case BindEnum::ClassEnum::Spy: { sInfo = "spy"; break; }
						}
						break;
					case BindEnum::WeaponType:
						sType = "weapon";
						switch (tBind.m_iInfo)
						{
						case BindEnum::WeaponTypeEnum::Hitscan: { sInfo = "hitscan"; break; }
						case BindEnum::WeaponTypeEnum::Projectile: { sInfo = "projectile"; break; }
						case BindEnum::WeaponTypeEnum::Melee: { sInfo = "melee"; break; }
						case BindEnum::WeaponTypeEnum::Throwable: { sInfo = "throwable"; break; }
						}
						break;
					case BindEnum::ItemSlot:
						sType = "slot";
						sInfo = std::format("{}", tBind.m_iInfo + 1);
						break;
					}
					if (tBind.m_bNot && (tBind.m_iType != BindEnum::Key || tBind.m_iInfo == BindEnum::KeyEnum::Hold))
						sInfo = std::format("not {}", sInfo);

					vInfo.emplace_back(tBind.m_sName.c_str(), sType, sInfo, iBind, tBind);
				}

				if (tBind.m_bActive || m_bIsOpen)
					getBinds(iBind);
			}
		};
	getBinds(DEFAULT_BIND);
	if (vInfo.empty())
		return;

	static DragBox_t tOld = { -2147483648, -2147483648 };
	DragBox_t tDragBox = m_bIsOpen ? FGet(Vars::Menu::BindsDisplay, true) : Vars::Menu::BindsDisplay.Value;
	if (tDragBox != tOld)
		SetNextWindowPos({ float(tDragBox.x), float(tDragBox.y) }, ImGuiCond_Always);

	float flNameWidth = 0, flInfoWidth = 0, flStateWidth = 0;
	PushFont(F::Render.FontSmall);
	for (auto& [sName, sInfo, sState, iBind, tBind] : vInfo)
	{
		flNameWidth = std::max(flNameWidth, FCalcTextSize(sName).x);
		flInfoWidth = std::max(flInfoWidth, FCalcTextSize(sInfo.c_str()).x);
		flStateWidth = std::max(flStateWidth, FCalcTextSize(sState.c_str()).x);
	}
	PopFont();
	flNameWidth += H::Draw.Scale(9), flInfoWidth += H::Draw.Scale(9), flStateWidth += H::Draw.Scale(9);

	float flWidth = flNameWidth + flInfoWidth + flStateWidth + (m_bIsOpen ? H::Draw.Scale(113) : H::Draw.Scale(14));
	float flHeight = H::Draw.Scale(18 * vInfo.size() + (Vars::Menu::BindWindowTitle.Value ? 42 : 12));
	SetNextWindowSize({ flWidth, flHeight });
	PushStyleVar(ImGuiStyleVar_WindowMinSize, { H::Draw.Scale(40), H::Draw.Scale(40) });
	if (Begin("Binds", nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoFocusOnAppearing))
	{
		ImVec2 vWindowPos = GetWindowPos();

		if (Vars::Menu::BindWindowTitle.Value)
			RenderTwoToneBackground(H::Draw.Scale(28), F::Render.Background0, F::Render.Background0p5, F::Render.Background2);
		else
			RenderBackground(F::Render.Background0p5, F::Render.Background2);

		tDragBox.x = vWindowPos.x; tDragBox.y = vWindowPos.y; tOld = tDragBox;
		if (m_bIsOpen)
			FSet(Vars::Menu::BindsDisplay, tDragBox);

		int iListStart = 8;
		if (Vars::Menu::BindWindowTitle.Value)
		{
			SetCursorPos({ H::Draw.Scale(8), H::Draw.Scale(6) });
			IconImage(ICON_MD_KEYBOARD);
			PushFont(F::Render.FontLarge);
			SetCursorPos({ H::Draw.Scale(30), H::Draw.Scale(7) });
			FText("Binds");
			PopFont();

			iListStart = 36;
		}

		PushFont(F::Render.FontSmall);
		int i = 0; for (auto& [sName, sInfo, sState, iBind, tBind] : vInfo)
		{
			float flPosX = 0;

			if (m_bIsOpen)
				PushTransparent(!F::Binds.WillBeEnabled(iBind), true);

			SetCursorPos({ flPosX += H::Draw.Scale(12), H::Draw.Scale(iListStart + 18 * i) });
			PushStyleColor(ImGuiCol_Text, tBind.m_bActive ? F::Render.Accent.Value : F::Render.Inactive.Value);
			FText(sName);
			PopStyleColor();

			SetCursorPos({ flPosX += flNameWidth, H::Draw.Scale(iListStart + 18 * i) });
			PushStyleColor(ImGuiCol_Text, tBind.m_bActive ? F::Render.Active.Value : F::Render.Inactive.Value);
			FText(sInfo.c_str());

			SetCursorPos({ flPosX += flInfoWidth, H::Draw.Scale(iListStart + 18 * i) });
			FText(sState.c_str());
			PopStyleColor();

			if (m_bIsOpen)
			{	// buttons
				SetCursorPos({ flWidth - H::Draw.Scale(26), H::Draw.Scale(iListStart - 2 + 18 * i) });
				bool bDelete = IconButton(ICON_MD_DELETE, H::Draw.Scale(18));

				SetCursorPos({ flWidth - H::Draw.Scale(51), H::Draw.Scale(iListStart - 2 + 18 * i) });
				bool bNot = IconButton(!tBind.m_bNot ? ICON_MD_CODE : ICON_MD_CODE_OFF, H::Draw.Scale(18));

				PushTransparent(Transparent || tBind.m_iVisibility == BindVisibilityEnum::Hidden, true);
				SetCursorPos({ flWidth - H::Draw.Scale(76), H::Draw.Scale(iListStart - 2 + 18 * i) });
				bool bVisibility = IconButton(tBind.m_iVisibility == BindVisibilityEnum::Always ? ICON_MD_VISIBILITY : ICON_MD_VISIBILITY_OFF, H::Draw.Scale(18));
				PopTransparent(1, 1);

				SetCursorPos({ flWidth - H::Draw.Scale(101), H::Draw.Scale(iListStart - 2 + 18 * i) });
				bool bEnable = IconButton(tBind.m_bEnabled ? ICON_MD_TOGGLE_ON : ICON_MD_TOGGLE_OFF, H::Draw.Scale(18));

				PopTransparent(1, 1);

				PushFont(F::Render.FontRegular);
				PushStyleVar(ImGuiStyleVar_WindowPadding, { H::Draw.Scale(8), H::Draw.Scale(8) });

				if (bEnable)
					tBind.m_bEnabled = !tBind.m_bEnabled;
				else if (bVisibility)
					tBind.m_iVisibility = (tBind.m_iVisibility + 1) % 3;
				else if (bNot)
					tBind.m_bNot = !tBind.m_bNot;
				else if (bDelete)
				{
					if (U::KeyHandler.Down(VK_SHIFT)) // allow user to quickly remove binds
						F::Binds.RemoveBind(iBind);
					else
						OpenPopup(std::format("DeleteBind{}", iBind).c_str());
				}

				if (FBeginPopupModal(std::format("DeleteBind{}", iBind).c_str(), nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysUseWindowPadding))
				{
					FText(std::format("Do you really want to delete '{}'{}?", tBind.m_sName, F::Binds.HasChildren(iBind) ? " and all of its children" : "").c_str());

					SetCursorPosY(GetCursorPosY() - 8); // stupid and i don't know why this is needed here
					if (FButton("Yes", FButtonEnum::Left))
					{
						F::Binds.RemoveBind(iBind);
						CloseCurrentPopup();
					}
					if (FButton("No", FButtonEnum::Right | FButtonEnum::SameLine))
						CloseCurrentPopup();

					EndPopup();
				}

				PopStyleVar();
				PopFont();
			}

			i++;
		}
		PopFont();

		End();
	}
	PopStyleVar();
}

static inline void SquareConstraints(ImGuiSizeCallbackData* data)
{
	//data->DesiredSize.x = data->DesiredSize.y = std::max(data->DesiredSize.x, data->DesiredSize.y);
	data->DesiredSize.x = data->DesiredSize.y = (data->DesiredSize.x + data->DesiredSize.y) / 2;
}

static inline void ManageVars()
{
	Vars::ESP::ActiveGroups.m_vValues = {};
	for (auto& tGroup : F::Groups.m_vGroups)
		Vars::ESP::ActiveGroups.m_vValues.push_back(tGroup.m_sName.c_str());
}

void CMenu::Render()
{
	using namespace ImGui;

	if (!(ImGui::GetIO().DisplaySize.x > 160.f && ImGui::GetIO().DisplaySize.y > 28.f))
		return;

	m_bInKeybind = m_bWindowHovered = false;
	// Default the model preview off every frame; the ESP tab turns it back on while visible.
	F::ModelPreview.SetWantRender(false);
	if (m_bIsOpen)
	{
		for (int iKey = 0; iKey < 256; iKey++)
			U::KeyHandler.StoreKey(iKey);
	}
	else
	{
		U::KeyHandler.StoreKey(Vars::Menu::PrimaryKey.Value);
		U::KeyHandler.StoreKey(Vars::Menu::SecondaryKey.Value);
		U::KeyHandler.StoreKey(VK_F11);
	}
	if (U::KeyHandler.Pressed(Vars::Menu::PrimaryKey.Value) || U::KeyHandler.Pressed(Vars::Menu::SecondaryKey.Value))
		I::MatSystemSurface->SetCursorAlwaysVisible(m_bIsOpen = !m_bIsOpen);

	// rebuild ISurface font atlas when user changes keybind/velocity font family or size
	// (or the spectator-list scale, whose font tracks it).
	{
		static std::string sPrevKbFam, sPrevVelFam, sPrevEspFam, sPrevMenuFam;
		static float flPrevKbSz = -1.f, flPrevVelSz = -1.f, flPrevSpecScale = -1.f, flPrevEspSz = -1.f, flPrevFlagSc = -1.f;
		static int iPrevThin = -1;
		static int iPrevEspAA = -1;
		const auto& sKb  = Vars::Visuals::UI::KeybindFontFamily.Value;
		const auto& sVel = Vars::Visuals::UI::VelocityFontFamily.Value;
		const auto& sEsp = Vars::ESP::FontFamily.Value;
		const auto& sMenu = Vars::Menu::Style::Font.Value; // the Minimal spec-list surface font tracks the menu font
		const float fKb  = Vars::Visuals::UI::KeybindFontSize.Value;
		const float fVel = Vars::Visuals::UI::VelocityFontSize.Value;
		const float fEsp = Vars::ESP::FontSize.Value;
		const float fFlagSc = Vars::ESP::FlagScale.Value; // FONT_ESP_LABEL tracks this
		const float fSpec = Vars::Menu::SpectatorListScale.Value;
		// FONT_KEYBIND's default family/weight/size tracks the thin-font toggle, so rebuild on it too.
		const int iThin = Vars::Visuals::UI::IndicatorThinFont.Value ? 1 : 0;
		const int iEspAA = Vars::ESP::FontNoAntialias.Value ? 1 : 0;
		if (sKb != sPrevKbFam || sVel != sPrevVelFam || sEsp != sPrevEspFam || sMenu != sPrevMenuFam || fKb != flPrevKbSz || fVel != flPrevVelSz || fEsp != flPrevEspSz || fFlagSc != flPrevFlagSc || fSpec != flPrevSpecScale || iThin != iPrevThin || iEspAA != iPrevEspAA)
		{
			sPrevKbFam = sKb; sPrevVelFam = sVel; sPrevEspFam = sEsp; sPrevMenuFam = sMenu; flPrevKbSz = fKb; flPrevVelSz = fVel; flPrevEspSz = fEsp; flPrevFlagSc = fFlagSc; flPrevSpecScale = fSpec; iPrevThin = iThin; iPrevEspAA = iEspAA;
			H::Fonts.Reload(Vars::Menu::Scale[DEFAULT_BIND]);
		}
	}

	PushFont(F::Render.FontRegular);

	DrawWatermark();
	DrawCheckpoints();
	DrawSpectatorListMinimal();
	DrawPixelSurfAssistBox();
	DrawAssistPointMenu();
	DrawMediaPlayer();
	DrawBinds();
	if (m_bIsOpen)
	{
		ManageVars();
		DrawMenu();

		AddDraggable("Ticks", Vars::Menu::TicksDisplay, FGet(Vars::Menu::Indicators) & Vars::Menu::IndicatorsEnum::Ticks);
		AddDraggable("Crit hack", Vars::Menu::CritsDisplay, FGet(Vars::Menu::Indicators) & Vars::Menu::IndicatorsEnum::CritHack);
		AddDraggable("Spectators", Vars::Menu::SpectatorsDisplay, FGet(Vars::Menu::Indicators) & Vars::Menu::IndicatorsEnum::Spectators);
		AddDraggable("Ping", Vars::Menu::PingDisplay, FGet(Vars::Menu::Indicators) & Vars::Menu::IndicatorsEnum::Ping);
		AddDraggable("Conditions", Vars::Menu::ConditionsDisplay, FGet(Vars::Menu::Indicators) & Vars::Menu::IndicatorsEnum::Conditions);
		AddDraggable("Seed prediction", Vars::Menu::SeedPredictionDisplay, FGet(Vars::Menu::Indicators) & Vars::Menu::IndicatorsEnum::SeedPrediction);
		AddDraggable("Recorder", Vars::Menu::RecorderDisplay, Vars::Misc::Movement::MovementRecorder.Value && Vars::Misc::Movement::MovementRecorderHud.Value, { H::Draw.Scale(160), H::Draw.Scale(50) });
		AddDraggable("Checkpoints", Vars::Menu::CheckpointsDisplay, Vars::Misc::Movement::Checkpoints.Value, { H::Draw.Scale(170), H::Draw.Scale(108) });
		AddResizableDraggable("Camera", Vars::Visuals::Simulation::ProjectileWindow, FGet(Vars::Visuals::Simulation::ProjectileCamera));

		// Don't switch to the hand cursor when hovering buttons/widgets; keep the normal arrow.
		// Force ImGui's internal cursor (not just the captured copy) so the Win32 backend doesn't
		if (GetMouseCursor() == ImGuiMouseCursor_Hand)
			SetMouseCursor(ImGuiMouseCursor_Arrow);
		F::Render.Cursor = GetMouseCursor();
		m_bWindowHovered = IsWindowHovered(ImGuiHoveredFlags_AnyWindow | ImGuiHoveredFlags_AllowWhenBlockedByPopup | ImGuiHoveredFlags_AllowWhenBlockedByActiveItem);

		if (!vDisabled.empty())
		{
			IM_ASSERT_USER_ERROR(0, "Calling PopDisabled() too little times: stack overflow.");
			Disabled = false;
			vDisabled.clear();
		}
		if (!vTransparent.empty())
		{
			IM_ASSERT_USER_ERROR(0, "Calling PopTransparent() too little times: stack overflow.");
			Transparent = false;
			vTransparent.clear();
		}
	}
	else
		mActiveMap.clear();

	PopFont();
}

void CMenu::AddOutput(const char* sFunction, const char* sLog, Color_t tColor)
{
	static size_t iID = 0;

	m_vOutput.emplace_back(sFunction, sLog, iID++, tColor);
	while (m_vOutput.size() > m_iMaxOutputSize)
		m_vOutput.pop_front();
}

void CMenu::DrawWatermark()
{
	using namespace ImGui;

	if (!Vars::Menu::Watermark.Value)
		return;

	static float flAnimProgress = 0.f;
	flAnimProgress = ImClamp(flAnimProgress + GetIO().DeltaTime * 5.f, 0.f, 1.f);

	const float flDt = GetIO().DeltaTime;
	const float flSW = GetIO().DisplaySize.x;
	const float flSH = GetIO().DisplaySize.y;
	if (flSW <= 0.f)
		return;

	// Cache FPS every 0.5s
	static float flFpsAccum = 0.f;
	static int iFpsCache = 0;
	flFpsAccum += flDt;
	if (flFpsAccum >= 0.5f) { iFpsCache = static_cast<int>(GetIO().Framerate); flFpsAccum = 0.f; }

	const int iAlpha = static_cast<int>(255 * flAnimProgress);
	ImColor tAccent = F::Render.Accent;
	tAccent.Value.w = flAnimProgress;
	const ImU32 uAccentU32 = static_cast<ImU32>(tAccent);
	const ImU32 uDimCol    = IM_COL32(140, 140, 140, iAlpha);
	const ImU32 uWhiteCol  = IM_COL32(255, 255, 255, iAlpha);
	const ImU32 uRedCol    = IM_COL32(255, 80,  80,  iAlpha);

	// Build item list (all text pointers must outlive this function call).
	// glyph >= 0 => draw the accent-tinted separator glyph texture m_pSepTex[glyph] instead of text.
	struct WmItem { const char* text; ImU32 col; float w = 0.f; int glyph = -1; IDirect3DTexture9* customTex = nullptr; }; // w cached in the measure pass below
	std::vector<WmItem> vItems;

	const std::string& sTag = Vars::Visuals::UI::Name.Value;
	vItems.push_back({ sTag.c_str(), uAccentU32 });

	// Separator: "Text" mode uses the customizable WmSeparator string; any other mode draws an
	// accent-tinted glyph texture. iSepGlyph 0 == Text, so the texture index is iSepGlyph - 1.
	const int iSepGlyph = Vars::Menu::WmSepGlyph.Value;
	const bool bCustomSep = iSepGlyph == Vars::Menu::WmSepGlyphEnum::WmSepCustom
		&& F::Render.m_pSepCustomTex; // pasted custom SVG
	const bool bBuiltinSep = iSepGlyph != Vars::Menu::WmSepGlyphEnum::WmSepText && !bCustomSep
		&& (iSepGlyph - 1) >= 0 && (iSepGlyph - 1) < CRender::kMaxSepGlyphs
		&& F::Render.m_pSepTex[iSepGlyph - 1];
	const bool bGlyphSep = bCustomSep || bBuiltinSep;

	// Customizable separator string (default "|"). Built once so the pointer stays valid for the draw loop.
	const std::string sSep = " " + Vars::Menu::WmSeparator.Value + " ";
	auto sep = [&]()
	{
		if (bCustomSep)
			vItems.push_back({ nullptr, uAccentU32, 0.f, -1, F::Render.m_pSepCustomTex });
		else if (bBuiltinSep)
			vItems.push_back({ nullptr, uAccentU32, 0.f, iSepGlyph - 1 });
		else
			vItems.push_back({ sSep.c_str(), uAccentU32 }); // Text separator also accent-tinted
	};

	static char szFps[16], szTime[16], szDate[16], szPing[16], szLoss[16];

	if (Vars::Menu::WmShowFPS.Value)
	{
		snprintf(szFps, sizeof(szFps), "%d fps", iFpsCache);
		sep(); vItems.push_back({ szFps, uWhiteCol });
	}
	if (Vars::Menu::WmShowTime.Value)
	{
		time_t now_t = time(nullptr);
		if (tm* pTm = localtime(&now_t))
			strftime(szTime, sizeof(szTime), "%H:%M", pTm);
		sep(); vItems.push_back({ szTime, uWhiteCol });
	}
	if (Vars::Menu::WmShowDate.Value)
	{
		time_t now_t = time(nullptr);
		if (tm* pTm = localtime(&now_t))
			strftime(szDate, sizeof(szDate), "%d/%m/%Y", pTm);
		sep(); vItems.push_back({ szDate, uWhiteCol });
	}

	const bool bConnected = I::EngineClient->IsConnected() && I::EngineClient->IsInGame();
	auto* pNetChan = bConnected ? I::EngineClient->GetNetChannelInfo() : nullptr;
	if (Vars::Menu::WmShowPing.Value)
	{
		if (pNetChan)
		{
			int iPing = static_cast<int>((pNetChan->GetAvgLatency(FLOW_OUTGOING) + pNetChan->GetAvgLatency(FLOW_INCOMING)) * 1000.f);
			snprintf(szPing, sizeof(szPing), "%d ms", iPing);
			sep(); vItems.push_back({ szPing, iPing > 120 ? uRedCol : uWhiteCol });
		}
		else { sep(); vItems.push_back({ "-- ms", uDimCol }); }
	}
	if (Vars::Menu::WmShowLoss.Value)
	{
		if (pNetChan)
		{
			int iLoss = static_cast<int>(pNetChan->GetAvgLoss(FLOW_OUTGOING) * 100.f);
			snprintf(szLoss, sizeof(szLoss), "%d%% loss", iLoss);
			sep(); vItems.push_back({ szLoss, iLoss > 5 ? uRedCol : uWhiteCol });
		}
		else { sep(); vItems.push_back({ "--% loss", uDimCol }); }
	}

	PushFont(F::Render.FontBold);

	// Square glyph separators are sized to the text cap height with a little breathing room each
	// side (mirrors the " | " spacing of the text separator). 1.25x to make them a touch bigger.
	const float flGlyphSz = CalcTextSize("X").y * 1.25f;
	const float flGlyphPad = 4.f;

	// Slash/Line glyphs (and their doubled variants) read thin/small at the shared box size, so give
	// them an extra bump on top of the base size.
	auto glyphScale = [](int g) -> float
	{
		if (g == Vars::Menu::WmSepGlyphEnum::WmSepSlash       - 1
		 || g == Vars::Menu::WmSepGlyphEnum::WmSepDoubleSlash - 1
		 || g == Vars::Menu::WmSepGlyphEnum::WmSepLine        - 1
		 || g == Vars::Menu::WmSepGlyphEnum::WmSepDoubleLine  - 1)
			return 1.35f;
		return 1.f;
	};

	// The dot/slash/line glyphs are thin shapes floating in a square canvas with wide empty
	// margins; crop equal slices off the left/right (UV + box width) so they don't eat space.
	auto glyphCropX = [](int g) -> float
	{
		if (g == Vars::Menu::WmSepGlyphEnum::WmSepSlash         - 1
		 || g == Vars::Menu::WmSepGlyphEnum::WmSepDoubleSlash   - 1
		 || g == Vars::Menu::WmSepGlyphEnum::WmSepLine          - 1
		 || g == Vars::Menu::WmSepGlyphEnum::WmSepDoubleLine    - 1
		 || g == Vars::Menu::WmSepGlyphEnum::WmSepDot           - 1
		 || g == Vars::Menu::WmSepGlyphEnum::WmSepEllipsis      - 1
		 || g == Vars::Menu::WmSepGlyphEnum::WmSepChevronRight  - 1
		 || g == Vars::Menu::WmSepGlyphEnum::WmSepChevronLeft   - 1
		 || g == Vars::Menu::WmSepGlyphEnum::WmSepChevronsRight - 1
		 || g == Vars::Menu::WmSepGlyphEnum::WmSepChevronsLeft  - 1
		 || g == Vars::Menu::WmSepGlyphEnum::WmSepAsterisk      - 1)
			return 0.5f; // keep only the centre 50% horizontally
		return 1.f;
	};

	// "Double slash"/"Double line" are special-cased: instead of cramming two strokes into one square
	// (which looks squished) we draw the single Slash/Line glyph twice, offset by flDoubleOffset.
	const float flDoubleOffset = 6.f;
	// Map a doubled glyph -> its single-stroke source glyph index (or -1 if not a doubled glyph).
	auto doubledSrc = [](int g) -> int
	{
		if (g == Vars::Menu::WmSepGlyphEnum::WmSepDoubleSlash - 1) return Vars::Menu::WmSepGlyphEnum::WmSepSlash - 1;
		if (g == Vars::Menu::WmSepGlyphEnum::WmSepDoubleLine  - 1) return Vars::Menu::WmSepGlyphEnum::WmSepLine  - 1;
		return -1;
	};

	// Compute total content width (cache each item's width for the draw pass)
	float flContentW = 0.f;
	for (auto& it : vItems)
	{
		if (it.customTex)
			it.w = flGlyphSz + flGlyphPad * 2.f; // custom glyph: plain square box, no scale/crop/double
		else
			it.w = it.glyph >= 0
				? flGlyphSz * glyphScale(it.glyph) * glyphCropX(it.glyph) + flGlyphPad * 2.f + (doubledSrc(it.glyph) >= 0 ? flDoubleOffset : 0.f)
				: CalcTextSize(it.text).x;
		flContentW += it.w;
	}

	const float flPadX = 7.f;
	const float flPadY = 5.5f;
	const float flH = CalcTextSize("X").y + flPadY * 2.f;

	IDirect3DTexture9* pWmLogo = F::Render.GetActiveLogoTex(); // selected built-in or custom logo
	const bool bShowLogo = Vars::Menu::WmShowLogo.Value && pWmLogo;
	const bool bSpinLogo = LogoSpinning();
	const float flLogoBase = bShowLogo ? (flH - flPadY) * 1.25f : 0.f; // 1.25x boost (was 1.1x)
	// Bigger reserved slot + smaller drawn quad while spinning so the rotating logo stays in the bar.
	const float flLogoSlot = flLogoBase * (bSpinLogo ? kLogoSpinSlot : 1.f);
	const float flLogoDraw = flLogoBase * (bSpinLogo ? kLogoSpinDraw : 1.f);
	const float flLogoGap = bShowLogo ? 5.f : 0.f; // match the menu header gap, give the bigger logo room
	if (bShowLogo) flContentW += flLogoSlot + flLogoGap;

	// Smooth width
	static float flCurrentW = 0.f;
	const float flTargetW = flContentW + flPadX * 2.f;
	if (flCurrentW < 1.f) flCurrentW = flTargetW;
	flCurrentW = ImLerp(flCurrentW, flTargetW, ImMin(1.f, flDt * 10.f));

	// Position
	const float kEdge = 10.f;
	float bx, by;
	switch (Vars::Menu::WmPosition.Value)
	{
	case Vars::Menu::WmPositionEnum::WmTopLeft:      bx = kEdge;                       by = kEdge;               break;
	case Vars::Menu::WmPositionEnum::WmTopCenter:    bx = (flSW - flCurrentW) * 0.5f; by = kEdge;               break;
	case Vars::Menu::WmPositionEnum::WmBottomLeft:   bx = kEdge;                       by = flSH - flH - kEdge;  break;
	case Vars::Menu::WmPositionEnum::WmBottomCenter: bx = (flSW - flCurrentW) * 0.5f; by = flSH - flH - kEdge;  break;
	case Vars::Menu::WmPositionEnum::WmBottomRight:  bx = flSW - flCurrentW - kEdge;  by = flSH - flH - kEdge;  break;
	default: /* WmTopRight */        bx = flSW - flCurrentW - kEdge;  by = kEdge;               break;
	}

	ImDrawList* pDraw = GetBackgroundDrawList();
	const ImVec2 vMin(bx, by);
	const ImVec2 vMax(bx + flCurrentW, by + flH);

	// Background (optional) - shared style box (Config > STYLE)
	if (Vars::Menu::WmBackground.Value)
		DrawStyledBackground(pDraw, vMin, vMax, flAnimProgress);

	// Draw items left-to-right
	float tx = bx + flPadX;
	const float ty = by + flPadY - 1.f;
	if (bShowLogo)
	{
		const Color_t tLogo = Vars::Menu::Theme::Accent.Value; // accent-tinted logo (keep fade alpha)
		const ImVec2 vCenter(tx + flLogoSlot * 0.5f, by + flH * 0.5f); // centre in the slot + bar; spin about it
		DrawLogoRotated(pDraw, reinterpret_cast<ImTextureID>(pWmLogo),
			vCenter, flLogoDraw, LogoSpinAngle(),
			IM_COL32(tLogo.r, tLogo.g, tLogo.b, iAlpha));
		tx += flLogoSlot + flLogoGap;
	}
	for (auto& it : vItems)
	{
		if (it.customTex)
		{
			// Pasted custom separator: plain square box, accent-tinted, vertically centred on the bar.
			const float gx = tx + flGlyphPad;
			const float gy = by + (flH - flGlyphSz) * 0.5f;
			pDraw->AddImage(reinterpret_cast<ImTextureID>(it.customTex),
				ImVec2(gx, gy), ImVec2(gx + flGlyphSz, gy + flGlyphSz),
				ImVec2(0.f, 0.f), ImVec2(1.f, 1.f), it.col);
		}
		else if (it.glyph >= 0 && it.glyph < CRender::kMaxSepGlyphs && F::Render.m_pSepTex[it.glyph])
		{
			// Accent-tinted glyph separator, vertically centred on the bar (keeps the fade alpha).
			const float gsz = flGlyphSz * glyphScale(it.glyph);
			const float flCrop = glyphCropX(it.glyph);
			const float gw = gsz * flCrop;               // drawn (cropped) width; full height stays gsz
			const float flUvL = (1.f - flCrop) * 0.5f;   // trim equal margins off the left/right UVs
			const ImVec2 vUvMin(flUvL, 0.f), vUvMax(1.f - flUvL, 1.f);
			const float gx = tx + flGlyphPad;
			const float gy = by + (flH - gsz) * 0.5f;
			const int iSrc = doubledSrc(it.glyph);
			if (iSrc >= 0 && iSrc < CRender::kMaxSepGlyphs && F::Render.m_pSepTex[iSrc])
			{
				// Two single strokes side-by-side (unsquished) with a small horizontal offset.
				IDirect3DTexture9* pSingle = F::Render.m_pSepTex[iSrc];
				pDraw->AddImage(reinterpret_cast<ImTextureID>(pSingle),
					ImVec2(gx, gy), ImVec2(gx + gw, gy + gsz),
					vUvMin, vUvMax, it.col);
				pDraw->AddImage(reinterpret_cast<ImTextureID>(pSingle),
					ImVec2(gx + flDoubleOffset, gy), ImVec2(gx + flDoubleOffset + gw, gy + gsz),
					vUvMin, vUvMax, it.col);
			}
			else
				pDraw->AddImage(
					reinterpret_cast<ImTextureID>(F::Render.m_pSepTex[it.glyph]),
					ImVec2(gx, gy), ImVec2(gx + gw, gy + gsz),
					vUvMin, vUvMax, it.col);
		}
		else if (it.text)
			pDraw->AddText(ImVec2(tx, ty), it.col, it.text);
		tx += it.w;
	}

	PopFont();
}

// On-screen "Checkpoints" key window. Drawn here (ImGui, GetBackgroundDrawList) - not in CMisc - so it
// shares the menu/watermark's EXACT styled background (Style colour + rounding + outline via
void CMenu::DrawCheckpoints()
{
	using namespace ImGui;

	if (!Vars::Misc::Movement::Checkpoints.Value || !I::EngineClient->IsInGame())
		return;
	auto pLocal = H::Entities.GetLocal();
	if (!pLocal || !pLocal->IsAlive())
		return;

	// the reference layout (scaled), mirroring the old CMisc::DrawCheckpoints.
	const float flW        = H::Draw.Scale(170);
	const float flTitleTop = H::Draw.Scale(5);
	const float flFirstRow = H::Draw.Scale(25);
	const float flLeftMrg  = H::Draw.Scale(10);
	const float flBox      = H::Draw.Scale(20);
	const float flBoxRound = H::Draw.Scale(3);
	const float flRowH     = H::Draw.Scale(24);
	const float flRowGap   = H::Draw.Scale(3);
	const float flBottom   = H::Draw.Scale(5);

	struct Row_t { const char* sLabel; int iKey; };
	const Row_t aRows[3] = {
		{ "Save",     Vars::Misc::Movement::CheckpointSaveKey.Value },
		{ "Teleport", Vars::Misc::Movement::CheckpointTeleportKey.Value },
		{ "Noclip",   Vars::Misc::Movement::CheckpointNoclipKey.Value },
	};
	constexpr int iRows = 3;
	const float flH = flFirstRow + iRows * flRowH + (iRows - 1) * flRowGap + flBottom;

	// DragBox stores centre-x / top-y (CMenu::AddDraggable); centre the fixed-width panel on it.
	int iCX = Vars::Menu::CheckpointsDisplay.Value.x;
	int iTop = Vars::Menu::CheckpointsDisplay.Value.y;
	if (iCX == 0 && iTop == 0)
	{
		iCX = int(H::Draw.m_nScreenW * 0.12f);
		iTop = int(H::Draw.m_nScreenH * 0.32f);
	}
	const float flLeft = float(iCX) - flW * 0.5f;
	const float flTopY = float(iTop);

	ImDrawList* pDraw = GetBackgroundDrawList();
	DrawStyledBackground(pDraw, { flLeft, flTopY }, { flLeft + flW, flTopY + flH });

	const ImU32 uTitle = IM_COL32(255, 255, 255, 255);
	const ImU32 uLabel = IM_COL32(100, 100, 100, 255);
	const ImU32 uKey   = IM_COL32(100, 100, 100, 255);
	const ImU32 uBox   = IM_COL32(22, 22, 22, 255);

	// Centred title.
	PushFont(F::Render.FontBold);
	{
		const char* szT = "Checkpoints";
		const ImVec2 vSz = CalcTextSize(szT);
		pDraw->AddText({ flLeft + (flW - vSz.x) * 0.5f, flTopY + flTitleTop }, uTitle, szT);
	}
	PopFont();

	// Rows: grey label on the left, bordered key box (with the bound key) on the right.
	PushFont(F::Render.FontRegular);
	for (int i = 0; i < iRows; i++)
	{
		const float flRowTop  = flTopY + flFirstRow + i * (flRowH + flRowGap);
		const float flRowMidY = flRowTop + flRowH * 0.5f;

		const ImVec2 vL = CalcTextSize(aRows[i].sLabel);
		pDraw->AddText({ flLeft + flLeftMrg, flRowMidY - vL.y * 0.5f }, uLabel, aRows[i].sLabel);

		const float flBoxX = flLeft + flW - flLeftMrg - flBox;
		const float flBoxY = flRowMidY - flBox * 0.5f;
		pDraw->AddRect({ flBoxX, flBoxY }, { flBoxX + flBox, flBoxY + flBox }, uBox, flBoxRound, 0, H::Draw.Scale(1.f));

		const std::string sKey = F::Misc.CheckpointKeyName(aRows[i].iKey);
		const ImVec2 vK = CalcTextSize(sKey.c_str());
		pDraw->AddText({ flBoxX + (flBox - vK.x) * 0.5f, flBoxY + (flBox - vK.y) * 0.5f }, uKey, sKey.c_str());
	}
	PopFont();
}

// Minimal spectator list (ex-Arbuzebra port). Drawn here (ImGui) - not in CSpectatorList - so the panel
void CMenu::DrawSpectatorListMinimal()
{
	using namespace ImGui;

	auto pLocal = H::Entities.GetLocal();
	if (!pLocal || !F::SpectatorList.PrepareMinimal(pLocal))
		return;
	const auto& vRows = F::SpectatorList.Rows();

	const float flScale = Vars::Menu::SpectatorListScale.Value;
	auto SL = [&](float v) { return H::Draw.Scale(v) * flScale; };

	// Row font height drives the vertical rhythm (matches the old surface layout, which used the row
	// font's height for the title band too).
	PushFont(F::Render.FontRegular); const float flRowTextH = GetFontSize(); PopFont();

	const float flTitleTop    = SL(5.f);
	const float flTitleToList = SL(15.f);
	const float flLeftMargin  = SL(10.f);
	const float flRowStride   = SL(24.f);
	const float flBottomPad   = SL(10.f);
	// Avatar size is fixed (DPI only) - it does NOT follow the spectator-list scale, only the panel does.
	const float flAvatarR     = ImMax(2.f, H::Draw.Scale(24.f) * 0.5f - H::Draw.Scale(3.f));
	const float flAvatarGap   = SL(6.f);

	const int iCount = int(vRows.size());
	const int iRows  = ImMax(iCount, 1); // empty list still shows the "no spectators" row

	const std::string& sTitle = Vars::Menu::SpectatorListTitle.Value;

	// Width grows to the widest title/row so long names never clip; SL(200) stays as the minimum.
	const float flTextOffset = flLeftMargin + flAvatarR * 2.f + flAvatarGap; // x from bx where row text starts
	float flW = SL(200.f);
	{
		PushFont(F::Render.FontBold);
		flW = ImMax(flW, CalcTextSize(sTitle.c_str()).x + flLeftMargin * 2.f);
		PopFont();
		PushFont(F::Render.FontRegular);
		if (iCount == 0)
			flW = ImMax(flW, CalcTextSize("no spectators").x + flLeftMargin * 2.f);
		for (int i = 0; i < iCount; i++)
		{
			const std::string sLine = std::format("{} | {}", vRows[i].m_sName, vRows[i].m_sMode);
			flW = ImMax(flW, flTextOffset + CalcTextSize(sLine.c_str()).x + flLeftMargin);
		}
		PopFont();
	}

	const float flListTop = flTitleTop + flRowTextH + flTitleToList;
	const float flH = flListTop + iRows * flRowStride - (flRowStride - flRowTextH) + flBottomPad;

	const float bx = float(Vars::Menu::SpectatorsDisplay.Value.x);
	const float by = float(Vars::Menu::SpectatorsDisplay.Value.y);

	ImDrawList* pDraw = GetBackgroundDrawList();
	DrawStyledBackground(pDraw, { bx, by }, { bx + flW, by + flH });

	const ImU32 uAccent = (ImU32)F::Render.Accent;

	// Centred title (accent), user-customizable (Vars::Menu::SpectatorListTitle).
	PushFont(F::Render.FontBold);
	{
		const char* szT = sTitle.c_str();
		const ImVec2 vSz = CalcTextSize(szT);
		pDraw->AddText({ bx + (flW - vSz.x) * 0.5f, by + flTitleTop }, uAccent, szT);
	}
	PopFont();

	// Faded separator under the title (brightest in the middle).
	{
		const float flLineY = by + flTitleTop + flRowTextH + SL(4.f);
		const float x1 = bx + flLeftMargin;
		const float x2 = bx + flW - flLeftMargin;
		const float xm = (x1 + x2) * 0.5f;
		const float flLineH = ImMax(1.f, H::Draw.Scale(1.f));
		const ImU32 uT = uAccent & 0x00FFFFFFu; // same RGB, 0 alpha
		pDraw->AddRectFilledMultiColor({ x1, flLineY }, { xm, flLineY + flLineH }, uT, uAccent, uAccent, uT);
		pDraw->AddRectFilledMultiColor({ xm, flLineY }, { x2, flLineY + flLineH }, uAccent, uT, uT, uAccent);
	}

	PushFont(F::Render.FontRegular);
	if (iCount == 0)
	{
		const char* szNone = "no spectators";
		const ImVec2 vSz = CalcTextSize(szNone);
		pDraw->AddText({ bx + (flW - vSz.x) * 0.5f, by + flListTop }, (ImU32)F::Render.Inactive, szNone);
	}
	else
	{
		const Color_t cText = Vars::Menu::SpectatorListTextColor.Value;
		const ImU32 uText = IM_COL32(cText.r, cText.g, cText.b, cText.a);
		const float flTextX = bx + flLeftMargin + flAvatarR * 2.f + flAvatarGap;
		for (int i = 0; i < iCount; i++)
		{
			const float flRowY = by + flListTop + i * flRowStride;
			const float flRowCenterY = flRowY + flRowTextH * 0.5f;

			// Account-id was resolved on the main thread at gather time (m_uAccountID); the Present thread
			// only uploads/binds the texture. Reading the resource netvar here raced the sim -> 0 -> no avatar.
			IDirect3DTexture9* pAvatar = F::Render.GetAvatarTex(vRows[i].m_uAccountID);
			if (pAvatar)
			{
				const float cx = bx + flLeftMargin + flAvatarR;
				pDraw->AddImageRounded(reinterpret_cast<ImTextureID>(pAvatar),
					{ cx - flAvatarR, flRowCenterY - flAvatarR }, { cx + flAvatarR, flRowCenterY + flAvatarR },
					{ 0.f, 0.f }, { 1.f, 1.f }, IM_COL32_WHITE, flAvatarR, ImDrawFlags_RoundCornersAll);
			}

			// First-person spectators draw in the accent colour (matches the other styles).
			const bool b1st = FNV1A::Hash32(vRows[i].m_sMode) == FNV1A::Hash32Const("1st");
			const std::string sLine = std::format("{} | {}", vRows[i].m_sName, vRows[i].m_sMode);
			pDraw->AddText({ flTextX, flRowY }, b1st ? uAccent : uText, sLine.c_str());
		}
	}
	PopFont();
}

// Pixel-surf-assist jump-box: small pop-up naming the jump the assist fired + the target Z height.
// Drawn here (not in CMisc) so it shares the watermark's exact font + flat rounded background.
void CMenu::DrawPixelSurfAssistBox()
{
	using namespace ImGui;

	std::string sText;
	float flFade = 0.f;
	if (!F::Misc.GetAssistJumpBox(sText, flFade))
		return;

	PushFont(F::Render.FontBold);

	const float flSW = GetIO().DisplaySize.x;
	const float flSH = GetIO().DisplaySize.y;

	const float flPadX = 6.f;
	const float flPadY = 4.f;
	const ImVec2 vTextSz = CalcTextSize(sText.c_str());
	const float flW = vTextSz.x + flPadX * 2.f;
	const float flH = vTextSz.y + flPadY * 2.f;

	const float bx = (flSW - flW) * 0.5f;
	const float by = flSH * Vars::Misc::Movement::PixelSurfAssistJumpBoxPosY.Value - flH * 0.5f;

	ImDrawList* pDraw = GetBackgroundDrawList();

	// Shared style box (Config > STYLE), faded with the pop-up.
	DrawStyledBackground(pDraw, ImVec2(bx, by), ImVec2(bx + flW, by + flH), flFade);

	pDraw->AddText(ImVec2(bx + flPadX, by + flPadY), IM_COL32(255, 255, 255, int(255 * flFade)), sText.c_str());

	PopFont();
}

// Per-point settings pop-up: aim at a saved pixel-surf-assist point and press
void CMenu::DrawAssistPointMenu()
{
	using namespace ImGui;

	// Edge-detected keys via GetAsyncKeyState so they work regardless of input routing.
	static bool bEnterWas = false, bBackWas = false;
	const bool bEnterDown = GetAsyncKeyState(VK_RETURN) & 0x8000;
	const bool bEnterPressed = bEnterDown && !bEnterWas;
	bEnterWas = bEnterDown;
	const bool bBackDown = GetAsyncKeyState(VK_BACK) & 0x8000;
	const bool bBackPressed = bBackDown && !bBackWas;
	bBackWas = bBackDown;

	const bool bInGame = I::EngineClient->IsInGame() && !I::EngineVGui->IsGameUIVisible();

	// We own the cursor only while the pop-up is open; the full menu manages its own.
	// (Opening itself happens in WndProc - the ENTER that opens is eaten there so it can't reach
	static bool bCursorOwned = false;
	static double dOpenedAt = 0.0;
	auto* pPoint = F::Misc.AssistPointMenuPoint();

	if (!pPoint || !bInGame || m_bIsOpen)
	{
		if (pPoint)
			F::Misc.CloseAssistPointMenu(true); // main menu opened over us / left game: save + close
		if (bCursorOwned)
		{
			if (!m_bIsOpen)
				I::MatSystemSurface->SetCursorAlwaysVisible(false);
			bCursorOwned = false;
		}
		return;
	}

	if (!bCursorOwned)
	{
		I::MatSystemSurface->SetCursorAlwaysVisible(true);
		bCursorOwned = true;
		dOpenedAt = GetTime(); // debounce: the press that opened us must not instantly close us
	}

	if (GetTime() - dOpenedAt > 0.25)
	{
		if (bEnterPressed || GetAsyncKeyState(VK_ESCAPE) & 0x8000)
		{
			F::Misc.CloseAssistPointMenu(true);
			return;
		}
		if (bBackPressed)
		{
			F::Misc.DeleteAssistPointMenuPoint();
			return;
		}
	}

	const float flSW = GetIO().DisplaySize.x;
	const float flSH = GetIO().DisplaySize.y;
	SetNextWindowPos(ImVec2(flSW * 0.5f + H::Draw.Scale(48), flSH * 0.5f - H::Draw.Scale(24)), ImGuiCond_Appearing);

	// Same chrome as the main menu/watermark: flat rounded theme background, accent title,
	// theme-colored widgets (the global style leaves WindowBg/Button transparent for the menu's
	PushStyleVar(ImGuiStyleVar_WindowRounding, H::Draw.Scale(Vars::Menu::Style::Rounding.Value)); // shared style (Config > STYLE)
	PushStyleVar(ImGuiStyleVar_WindowPadding, { H::Draw.Scale(12), H::Draw.Scale(10) });
	PushStyleVar(ImGuiStyleVar_FramePadding, { H::Draw.Scale(4), H::Draw.Scale(3) });
	PushStyleColor(ImGuiCol_WindowBg, ImVec4(0, 0, 0, 0)); // transparent: we draw the shared style bg ourselves below
	PushStyleColor(ImGuiCol_CheckMark, F::Render.Accent.Value);
	PushStyleColor(ImGuiCol_SliderGrab, F::Render.Accent.Value);
	PushStyleColor(ImGuiCol_SliderGrabActive, F::Render.Accent.Value);
	PushStyleColor(ImGuiCol_Button, F::Render.Background1p5.Value);
	PushStyleColor(ImGuiCol_ButtonHovered, F::Render.Background1p5L.Value);
	PushStyleColor(ImGuiCol_ButtonActive, F::Render.Background1p5.Value);
	PushStyleColor(ImGuiCol_Separator, F::Render.Background2.Value);
	PushStyleColor(ImGuiCol_TextDisabled, F::Render.Inactive.Value);

	bool bDeleteClicked = false;

	PushFont(F::Render.FontRegular);
	if (Begin("ps point settings", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoTitleBar))
	{
		// Shared style background.
		{
			ImDrawList* pBgDL = GetWindowDrawList();
			const ImVec2 vWPos = GetWindowPos();
			const ImVec2 vWSize = GetWindowSize();
			DrawStyledBackground(pBgDL, vWPos, { vWPos.x + vWSize.x, vWPos.y + vWSize.y });
		}

		// Centered accent title + fading underline, mirroring the main menu's title bar
		PushFont(F::Render.FontBold);
		const char* szTitle = "pixel surf point";
		const float flWinW = GetWindowWidth();
		SetCursorPosX(ImMax(GetStyle().WindowPadding.x, (flWinW - CalcTextSize(szTitle).x) * 0.5f));
		TextColored(F::Render.Accent.Value, "%s", szTitle);
		PopFont();
		{
			ImDrawList* pDL = GetWindowDrawList();
			const ImVec2 vWin = GetWindowPos();
			const float x1 = vWin.x + H::Draw.Scale(10);
			const float x2 = vWin.x + flWinW - H::Draw.Scale(10);
			const float xm = (x1 + x2) * 0.5f;
			const float yTop = vWin.y + GetCursorPosY() + H::Draw.Scale(2);
			const float flLineH = H::Draw.Scale(2);
			const ImU32 uCol = (ImU32)ImColor(F::Render.Accent);
			const ImU32 uColT = uCol & 0x00FFFFFFu;
			pDL->AddRectFilledMultiColor({ x1, yTop }, { xm, yTop + flLineH }, uColT, uCol, uCol, uColT);
			pDL->AddRectFilledMultiColor({ xm, yTop }, { x2, yTop + flLineH }, uCol, uColT, uColT, uCol);
			Dummy({ 0.f, flLineH + H::Draw.Scale(6) });
		}

		TextDisabled("z %.3f", pPoint->m_vPos.z);

		Checkbox("active", &pPoint->m_bActive);

		SetNextItemWidth(H::Draw.Scale(170));
		SliderFloat("radius", &pPoint->m_flRadius, 0.f, 1000.f, pPoint->m_flRadius <= 0.f ? "global" : "%.0f");

		Separator();
		TextDisabled("allowed jumps");
		static const char* aszLabels[6] = { "mini", "regular", "mini+duck", "crouch", "long", "long+duck" };
		for (int k = 0; k < 6; k++)
		{
			bool bOn = pPoint->m_iTypeMask & (1 << k);
			if (Checkbox(aszLabels[k], &bOn))
				pPoint->m_iTypeMask = bOn ? (pPoint->m_iTypeMask | (1 << k)) : (pPoint->m_iTypeMask & ~(1 << k));
			if (k % 2 == 0)
				SameLine(H::Draw.Scale(120));
		}
		{
			bool bOn = pPoint->m_iTypeMask & (1 << 6);
			if (Checkbox("double jump (scout)", &bOn))
				pPoint->m_iTypeMask = bOn ? (pPoint->m_iTypeMask | (1 << 6)) : (pPoint->m_iTypeMask & ~(1 << 6));
		}

		Separator();
		if (Button("delete point", ImVec2(-1.f, 0.f)))
			bDeleteClicked = true;
		TextDisabled("enter: save + close   backspace: delete");
	}
	End();
	PopFont();
	PopStyleColor(9);
	PopStyleVar(3);

	if (bDeleteClicked)
		F::Misc.DeleteAssistPointMenuPoint();
}

void CMenu::DrawMediaPlayer()
{
	using namespace ImGui;

	// Check if media player is enabled
	if (!Vars::Menu::MediaPlayer.Value)
		return;

	// Don't draw if no media is playing
	if (strtitle.empty())
		return;

	static float flAnimProgress = 0.f;
	flAnimProgress = ImClamp(flAnimProgress + GetIO().DeltaTime * 5.f, 0.f, 1.f);

	// Calculate progress
	float progress = 0.0f;
	if (trackDuration > 0)
	{
		progress = static_cast<float>(trackPosition) / static_cast<float>(trackDuration);
		if (progress > 1.0f)
			progress = 1.0f;
	}

	// Smooth progress interpolation
	static float smoothProgress = 0.0f;
	if (progress < smoothProgress)
		smoothProgress = progress;
	smoothProgress += (progress - smoothProgress) * 0.1f;

	// FontMedia is re-baked at MediaPlayerScale (see CRender::LoadFonts), so draw at its native pixel
	// size for crisp text. flScale still drives layout (padding, thumbnail, bar) only.
	ImFont* pFont = F::Render.FontMedia ? F::Render.FontMedia : F::Render.FontBold;
	PushFont(pFont);

	const float flScale = std::clamp(Vars::Menu::MediaPlayerScale.Value, 0.5f, 2.5f);
	const float flFontSize = pFont->FontSize;

	ImDrawList* pDraw = GetBackgroundDrawList();
	ImColor tAccent = F::Render.Accent;
	tAccent.Value.w = flAnimProgress;

	auto ToCol = [&](Color_t c) { return ImColor((int)c.r, (int)c.g, (int)c.b, (int)(c.a * flAnimProgress)); };
	auto Measure = [&](const char* s) { return pFont->CalcTextSizeA(flFontSize, FLT_MAX, 0.f, s); };

	const float flScreenW = GetIO().DisplaySize.x;
	const float flScreenH = GetIO().DisplaySize.y;

	// Compute the box top-left from the chosen anchor preset (+ fine X/Y offsets)
	const float flPadEdge = 14.f;
	auto AnchorTopLeft = [&](float flW, float flH) -> ImVec2
	{
		float x, y;
		switch (Vars::Menu::MediaPlayerPosition.Value)
		{
		case Vars::Menu::MediaPlayerPositionEnum::TopLeft:      x = flPadEdge;                    y = flPadEdge; break;
		case Vars::Menu::MediaPlayerPositionEnum::TopCenter:    x = (flScreenW - flW) * 0.5f;     y = flPadEdge; break;
		case Vars::Menu::MediaPlayerPositionEnum::BottomLeft:   x = flPadEdge;                    y = flScreenH - flH - flPadEdge; break;
		case Vars::Menu::MediaPlayerPositionEnum::BottomCenter: x = (flScreenW - flW) * 0.5f;     y = flScreenH - flH - flPadEdge; break;
		case Vars::Menu::MediaPlayerPositionEnum::BottomRight:  x = flScreenW - flW - flPadEdge;  y = flScreenH - flH - flPadEdge; break;
		case Vars::Menu::MediaPlayerPositionEnum::TopRight:
		default:                                                x = flScreenW - flW - flPadEdge;  y = flPadEdge; break;
		}
		// Drop below the watermark when both sit at a Top anchor on the same side, so the two boxes
		// don't overlap. Watermark height mirrors DrawWatermark: kEdge(10) + textH + flPadY*2(11) + gap.
		auto MediaSide = [](int p) -> int {
			switch (p) {
			case Vars::Menu::MediaPlayerPositionEnum::TopLeft:   case Vars::Menu::MediaPlayerPositionEnum::BottomLeft:   return 0;
			case Vars::Menu::MediaPlayerPositionEnum::TopCenter: case Vars::Menu::MediaPlayerPositionEnum::BottomCenter: return 1;
			default: return 2; }
		};
		auto WmSide = [](int p) -> int {
			switch (p) {
			case Vars::Menu::WmPositionEnum::WmTopLeft:   case Vars::Menu::WmPositionEnum::WmBottomLeft:   return 0;
			case Vars::Menu::WmPositionEnum::WmTopCenter: case Vars::Menu::WmPositionEnum::WmBottomCenter: return 1;
			default: return 2; }
		};
		const int iMP = Vars::Menu::MediaPlayerPosition.Value;
		const int iWP = Vars::Menu::WmPosition.Value;
		const bool bMediaTop = iMP == Vars::Menu::MediaPlayerPositionEnum::TopLeft
			|| iMP == Vars::Menu::MediaPlayerPositionEnum::TopCenter
			|| iMP == Vars::Menu::MediaPlayerPositionEnum::TopRight;
		const bool bWmTop = iWP == Vars::Menu::WmPositionEnum::WmTopLeft
			|| iWP == Vars::Menu::WmPositionEnum::WmTopCenter
			|| iWP == Vars::Menu::WmPositionEnum::WmTopRight;
		if (Vars::Menu::Watermark.Value && bMediaTop && bWmTop && MediaSide(iMP) == WmSide(iWP))
		{
			const float flWmFontH = F::Render.FontBold ? F::Render.FontBold->FontSize : 16.f;
			y += 10.f + flWmFontH + 2.f; // watermark edge + height + gap
			// Align x to the watermark edge (media inset 14 vs wm 10): Right +, Left -, Center none.
			const int iSide = MediaSide(iMP);
			if (iSide == 2)      x += 0.0025f * flScreenW;
			else if (iSide == 0) x -= 0.0025f * flScreenW;
		}
		x += Vars::Menu::MediaPlayerPosX.Value * flScreenW;
		y += Vars::Menu::MediaPlayerPosY.Value * flScreenH;
		return ImVec2(x, y);
	};

	// ---- Text strip mode: "title - artist" on a watermark-style bar ----
	if (Vars::Menu::MediaPlayerMode.Value == Vars::Menu::MediaPlayerModeEnum::TextStrip)
	{
		const std::string sLine = strtitle + "  -  " + strartist;
		const ImVec2 vSz = Measure(sLine.c_str());
		const float flPad = 8.f * flScale;
		const float flW = vSz.x + flPad * 2.f;
		const float flH = vSz.y + flPad * 2.f;
		const ImVec2 vPos = AnchorTopLeft(flW, flH);
		const ImVec2 vMax(vPos.x + flW, vPos.y + flH);
		if (Vars::Menu::MediaPlayerBackground.Value)
			DrawStyledBackground(pDraw, vPos, vMax, flAnimProgress); // shared style box (Config > STYLE)
		pDraw->AddText(pFont, flFontSize, ImVec2(vPos.x + flPad, vPos.y + flPad), ToCol(Vars::Menu::MediaPlayerTitleColor.Value), sLine.c_str());
		PopFont();
		return;
	}

	// ---- Card mode ----
	const bool bThumb = Vars::Menu::MediaPlayerThumb.Value;
	const bool bCircle = Vars::Menu::MediaPlayerThumbCircle.Value;
	const bool bProgress = Vars::Menu::MediaPlayerProgress.Value;
	const float flPadding = 10.f * flScale;
	const float flImageSize = bThumb ? Vars::Menu::MediaPlayerThumbSize.Value * flScale : 0.f;

	std::string sArtist = strartist;

	const ImVec2 vSizeTitle = Measure(strtitle.c_str());
	const ImVec2 vSizeArtist = Measure(sArtist.c_str());
	const float flThumbGap = bThumb ? (flPadding + flImageSize) : 0.f;
	const float flWidth = std::max(vSizeTitle.x, vSizeArtist.x) + flPadding * 2.f + flThumbGap;
	const float flHeight = std::max(45.f * flScale, flImageSize + flPadding * 2.f);

	const ImVec2 vPos = AnchorTopLeft(flWidth, flHeight);
	const ImVec2 vPosMax(vPos.x + flWidth, vPos.y + flHeight);

	// Background (optional) - shared style box (Config > STYLE)
	if (Vars::Menu::MediaPlayerBackground.Value)
		DrawStyledBackground(pDraw, vPos, vPosMax, flAnimProgress);

	// Album art
	if (bThumb)
	{
		const ImVec2 vImagePos(vPos.x + flPadding, vPos.y + (flHeight - flImageSize) / 2.f);
		const ImVec2 vImageMax(vImagePos.x + flImageSize, vImagePos.y + flImageSize);
		const float flArtRound = bCircle ? flImageSize / 2.f : 4.f * flScale;
		if (albumArtTexture)
			pDraw->AddImageRounded((ImTextureID)albumArtTexture, vImagePos, vImageMax, ImVec2(0, 0), ImVec2(1, 1), ImColor(255, 255, 255, (int)(255 * flAnimProgress)), flArtRound);
		else
			pDraw->AddRectFilled(vImagePos, vImageMax, ImColor(60, 60, 60, (int)(255 * flAnimProgress)), flArtRound);

		if (bCircle)
			pDraw->AddCircle(ImVec2(vImagePos.x + flImageSize / 2.f, vImagePos.y + flImageSize / 2.f), flImageSize / 2.f, tAccent, 0, 2.f);
		else
			pDraw->AddRect(vImagePos, vImageMax, tAccent, flArtRound, 0, 2.f);
	}

	// Text
	const float flTextX = vPos.x + flPadding + flThumbGap;
	const float flTextY = vPos.y + (bProgress ? 8.f * flScale : (flHeight - vSizeTitle.y - vSizeArtist.y) / 2.f);
	pDraw->AddText(pFont, flFontSize, ImVec2(flTextX, flTextY), ToCol(Vars::Menu::MediaPlayerTitleColor.Value), strtitle.c_str());
	pDraw->AddText(pFont, flFontSize, ImVec2(flTextX, flTextY + vSizeTitle.y + 2.f), ToCol(Vars::Menu::MediaPlayerArtistColor.Value), sArtist.c_str());

	// Progress bar
	if (bProgress)
	{
		const float flBarW = flWidth - flPadding * 2.f;
		const float flBarH = 2.f * flScale;
		const ImVec2 vBar(vPos.x + flPadding, vPosMax.y - 4.f * flScale - flBarH);
		pDraw->AddRectFilled(vBar, ImVec2(vBar.x + flBarW, vBar.y + flBarH), ImColor(60, 60, 60, (int)(255 * flAnimProgress)), 1.f);
		pDraw->AddRectFilled(vBar, ImVec2(vBar.x + flBarW * smoothProgress, vBar.y + flBarH), tAccent, 1.f);
	}

	PopFont();
}
