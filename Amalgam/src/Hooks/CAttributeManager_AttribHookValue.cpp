#include "../SDK/SDK.h"

MAKE_SIGNATURE(CAttributeManager_AttribHookInt, "client.dll", "4C 8B DC 49 89 5B ? 49 89 6B ? 49 89 73 ? 57 41 54 41 55 41 56 41 57 48 83 EC ? 48 8B 3D ? ? ? ? 4C 8D 35", 0x0);
MAKE_SIGNATURE(CTFPlayer_FireEvent_AttribHookValue_Call, "client.dll", "8B F8 83 BE", 0x0);

static inline int ColorToInt(Color_t col)
{
    return col.r << 16 | col.g << 8 | col.b;
}

// Set by CAttributeManager_AttribHookInt while the sheen proxy (below) is reading
// killstreak_idleeffect on a local-owned weapon during its OnBind. Reset at the top
static bool g_bSheenBindLocal = false;

MAKE_HOOK(CAttributeManager_AttribHookInt, S::CAttributeManager_AttribHookInt(), int,
	int value, const char* name, void* econent, void* buffer, bool isGlobalConstString)
{
#ifdef DEBUG_HOOKS
	if (!Vars::Hooks::CAttributeManager_AttribHookValue[DEFAULT_BIND])
		return CALL_ORIGINAL(value, name, econent, buffer, isGlobalConstString);
#endif

	const auto dwDesired = S::CTFPlayer_FireEvent_AttribHookValue_Call();
	const auto dwRetAddr = uintptr_t(_ReturnAddress());

	if (dwRetAddr == dwDesired && Vars::Visuals::Effects::SpellFootsteps.Value
		&& econent == H::Entities.GetLocal() && FNV1A::Hash32(name) == FNV1A::Hash32Const("halloween_footstep_type"))
	{
		switch (Vars::Visuals::Effects::SpellFootsteps.Value)
		{
		case Vars::Visuals::Effects::SpellFootstepsEnum::Color: return ColorToInt(Vars::Colors::SpellFootstep.Value);
		case Vars::Visuals::Effects::SpellFootstepsEnum::Team: return 1;
		case Vars::Visuals::Effects::SpellFootstepsEnum::Halloween: return 2;
		}
	}

	// Force a killstreak sheen on all of the local player's weapons (the reference "sheen on all
	// weapons"). Ported the Amalgam-native way: override the killstreak attribute reads here
	if (Vars::Visuals::Effects::WeaponSheen.Value && econent)
	{
		const auto uName = FNV1A::Hash32(name);
		if (uName == FNV1A::Hash32Const("killstreak_idleeffect") || uName == FNV1A::Hash32Const("killstreak_tier"))
		{
			// econent is the weapon (CEconEntity) for these reads; only touch the local player's own.
			auto pEntity = reinterpret_cast<CBaseEntity*>(econent);
			if (pEntity->m_hOwnerEntity().Get() == H::Entities.GetLocal())
			{
				if (uName == FNV1A::Hash32Const("killstreak_tier"))
					return 3; // professional tier so the sheen proxy renders on every weapon
				// idleeffect read by CProxyAnimatedWeaponSheen::OnBind for OUR weapon: flag it so the
				// OnBind override knows this bind is local and may recolour it.
				g_bSheenBindLocal = true;
				return Vars::Visuals::Effects::WeaponSheen.Value; // 1..7 = sheen colour index / anim style
			}
		}
	}

	return CALL_ORIGINAL(value, name, econent, buffer, isGlobalConstString);
}

// ===== Custom weapon-sheen colour ========================================================================
// TF2 only has 7 fixed sheen colours (the killstreak_idleeffect palette). To get an arbitrary RGB

using SheenOnBind_t = void(__fastcall*)(void*, void*);
static SheenOnBind_t o_SheenOnBind = nullptr;

// $sheenmaptint IMaterialVar* offset inside CProxyAnimatedWeaponSheen (x64 TF2). Confirmed via
// the shipped hook (m_pSheenIndexVar@0x20, m_pTintVar@0x28).
static constexpr ptrdiff_t kSheenTintVarOffset = 0x28;

__declspec(noinline) static void SheenOnBind_OverrideTint(uintptr_t base)
{
	__try
	{
		auto pTint = *reinterpret_cast<IMaterialVar**>(base + kSheenTintVarOffset);
		if (pTint)
		{
			const Color_t c = Vars::Colors::WeaponSheen.Value;
			// Match the reference: 0-255 colour -> 0-2.55 HDR tint (component / 100) so the sheen pops.
			float col[4] = { c.r / 100.f, c.g / 100.f, c.b / 100.f, 1.f };
			pTint->SetVecValue(col, 4);
		}
	}
	__except (EXCEPTION_EXECUTE_HANDLER) {}
}

static void __fastcall h_SheenOnBind(void* rcx, void* pEntity)
{
	g_bSheenBindLocal = false;
	if (o_SheenOnBind)
		o_SheenOnBind(rcx, pEntity);

	// Only recolour when enabled, custom colour is on, and the original just bound OUR weapon.
	if (rcx && g_bSheenBindLocal
		&& Vars::Visuals::Effects::WeaponSheen.Value
		&& Vars::Visuals::Effects::WeaponSheenCustomColor.Value)
		SheenOnBind_OverrideTint(reinterpret_cast<uintptr_t>(rcx));
}

void InstallSheenColorHook()
{
	static bool s_bAttempted = false;
	// Don't pay the scan until the user actually turns the sheen on; latch after the first attempt.
	if (s_bAttempted || !Vars::Visuals::Effects::WeaponSheen.Value)
		return;
	s_bAttempted = true;

	static const char* const sPatterns[] = {
		"48 89 54 24 ? 55 57 41 54 48 8D 6C 24",
		"48 89 5C 24 ? 48 89 74 24 ? 57 48 83 EC ? 49",
		"40 55 53 56 57 41 54 41 55 41 56 41 57 48 8D 6C 24 ? 48 81 EC ? ? ? ? 4C 8B EA",
	};

	uintptr_t addr = 0;
	for (auto pPattern : sPatterns)
		if ((addr = U::Memory.FindSignature("client.dll", pPattern)))
			break;
	if (!addr)
		return; // build drift: custom sheen colour silently unavailable, everything else fine

	if (MH_CreateHook(reinterpret_cast<void*>(addr), reinterpret_cast<void*>(&h_SheenOnBind),
		reinterpret_cast<void**>(&o_SheenOnBind)) == MH_OK)
		MH_EnableHook(reinterpret_cast<void*>(addr));
}