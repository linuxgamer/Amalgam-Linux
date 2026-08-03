#include "../SDK/SDK.h"

#include "../Features/Visuals/Visuals.h"
#include "../Features/Ticks/Ticks.h"
#include "../Features/CritHack/CritHack.h"
#include "../Features/Visuals/SpectatorList/SpectatorList.h"
#include "../Features/Backtrack/Backtrack.h"
#include "../Features/Visuals/PlayerConditions/PlayerConditions.h"
#include "../Features/NoSpread/NoSpreadHitscan/NoSpreadHitscan.h"
#include "../Features/Aimbot/Aimbot.h"
#include "../Features/Visuals/ESP/ESP.h"
#include "../Features/Visuals/OffscreenArrows/OffscreenArrows.h"
#include "../Features/Visuals/CameraWindow/CameraWindow.h"
#include "../Features/Visuals/Notifications/Notifications.h"
#include "../Features/Aimbot/AutoHeal/AutoHeal.h"
#include "../Features/Misc/Misc.h"
#include "../Features/Indicators/Indicators.h"
#include "../Features/Visuals/FakePOV/FakePOV.h"
#include "../Features/Visuals/ModelPreview/ModelPreview.h"

// The ESP-tab model preview, in its render order: rounded surface bg UNDER, the model RT render+blit, then
// the 2D ESP overlay over it. Three separate H::Draw batches so the surface draws flush in the right order
static void DrawModelPreviewPanel()
{
	H::Draw.UpdateScreenSize();

	H::Draw.Start();
	{
		F::ModelPreview.DrawBackground();
	}
	H::Draw.End();

	F::ModelPreview.Run(); // render the model into the RT + blit it (transparent bg) over the rounded bg

	H::Draw.Start();
	{
		F::ModelPreview.DrawOverlay(); // 2D ESP (box/bones/health/uber/name/class+weapon) over the blit
	}
	H::Draw.End();
}

MAKE_HOOK(IEngineVGui_Paint, U::Memory.GetVirtual(I::EngineVGui, 14), void,
	void* rcx, int iMode)
{
#ifdef DEBUG_HOOKS
	if (!Vars::Hooks::IEngineVGui_Paint[DEFAULT_BIND])
		return CALL_ORIGINAL(rcx, iMode);
#endif

	if (G::Unload)
		return CALL_ORIGINAL(rcx, iMode);

	if (iMode & PAINT_INGAMEPANELS && !SDK::CleanScreenshot())
	{
		H::Draw.UpdateScreenSize();
		H::Draw.UpdateW2SMatrix();
		H::Draw.Start(true);
		if (auto pLocal = H::Entities.GetLocal())
		{
			F::CameraWindow.Draw();
			F::Visuals.DrawAntiAim(pLocal);

			F::Visuals.DrawPickupTimers();
			F::ESP.Draw();
			F::Arrows.Draw(pLocal);
			F::Aimbot.Draw(pLocal);
			F::Misc.Draw(pLocal);

#ifdef DEBUG_VACCINATOR
			F::AutoHeal.Draw(pLocal);
#endif
			F::NoSpreadHitscan.Draw(pLocal);
			F::PlayerConditions.Draw(pLocal);
			F::Backtrack.Draw(pLocal);
			F::SpectatorList.Draw(pLocal);
			F::CritHack.Draw(pLocal);
			F::Ticks.Draw(pLocal);
			F::Visuals.DrawDebugInfo(pLocal);
			F::Indicators.Draw(pLocal);
			F::FakePOV.Draw(pLocal);
		}
		H::Draw.End();
	}

	CALL_ORIGINAL(rcx, iMode);

	// In-game model preview drawn AFTER CALL_ORIGINAL but still on the PAINT_INGAMEPANELS call: that call
	// fires every frame in a match (so no blink) AND CALL_ORIGINAL just painted the in-game HUD, so drawing
	if (iMode & PAINT_INGAMEPANELS && !SDK::CleanScreenshot() && I::EngineClient->IsInGame())
		DrawModelPreviewPanel();

	if (iMode & PAINT_UIPANELS && !SDK::CleanScreenshot())
	{
		H::Draw.UpdateScreenSize();

		H::Draw.Start();
		{
			F::Notifications.Draw();
		}
		H::Draw.End();

		// Main-menu (not connected): PAINT_INGAMEPANELS doesn't fire, so draw the preview here (after
		// CALL_ORIGINAL so the full-screen menu bg doesn't cover it).
		if (!I::EngineClient->IsConnected())
			DrawModelPreviewPanel();
	}
}