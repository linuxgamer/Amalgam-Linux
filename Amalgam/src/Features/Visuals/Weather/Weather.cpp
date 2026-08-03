#include "Weather.h"

#include <Psapi.h>
#pragma comment(lib, "Psapi.lib")

// ===================================================================================================
// Fog changer
void CWeather::ApplyFog()
{
	static auto fog_override     = H::ConVars.FindVar("fog_override");
	static auto fog_enable       = H::ConVars.FindVar("fog_enable");
	static auto fog_enableskybox = H::ConVars.FindVar("fog_enableskybox");
	static auto fog_start            = H::ConVars.FindVar("fog_start");
	static auto fog_end              = H::ConVars.FindVar("fog_end");
	static auto fog_startskybox      = H::ConVars.FindVar("fog_startskybox");
	static auto fog_endskybox        = H::ConVars.FindVar("fog_endskybox");
	static auto fog_maxdensity       = H::ConVars.FindVar("fog_maxdensity");
	static auto fog_maxdensityskybox = H::ConVars.FindVar("fog_maxdensityskybox");
	static auto fog_color            = H::ConVars.FindVar("fog_color");
	static auto fog_colorskybox      = H::ConVars.FindVar("fog_colorskybox");

	static bool bWasEnabled = false;

	if (Vars::Visuals::Weather::Fog.Value)
	{
		const float flSkyStart   = Vars::Visuals::Weather::FogLinkSkybox.Value ? Vars::Visuals::Weather::FogStart.Value   : Vars::Visuals::Weather::FogSkyStart.Value;
		const float flSkyEnd     = Vars::Visuals::Weather::FogLinkSkybox.Value ? Vars::Visuals::Weather::FogEnd.Value     : Vars::Visuals::Weather::FogSkyEnd.Value;
		const float flSkyDensity = Vars::Visuals::Weather::FogLinkSkybox.Value ? Vars::Visuals::Weather::FogDensity.Value : Vars::Visuals::Weather::FogSkyDensity.Value;

		if (fog_override)      fog_override->SetValue(1);
		if (fog_enable)        fog_enable->SetValue(1);
		if (fog_enableskybox)  fog_enableskybox->SetValue(1);
		if (fog_start)            fog_start->SetValue(Vars::Visuals::Weather::FogStart.Value);
		if (fog_end)              fog_end->SetValue(Vars::Visuals::Weather::FogEnd.Value);
		if (fog_startskybox)      fog_startskybox->SetValue(flSkyStart);
		if (fog_endskybox)        fog_endskybox->SetValue(flSkyEnd);
		if (fog_maxdensity)       fog_maxdensity->SetValue(Vars::Visuals::Weather::FogDensity.Value);
		if (fog_maxdensityskybox) fog_maxdensityskybox->SetValue(flSkyDensity);

		const auto& tColor = Vars::Visuals::Weather::FogColor.Value;
		char szColor[32];
		snprintf(szColor, sizeof(szColor), "%d %d %d", tColor.r, tColor.g, tColor.b);
		if (fog_color)       fog_color->SetValue(szColor);
		if (fog_colorskybox) fog_colorskybox->SetValue(szColor);

		bWasEnabled = true;
	}
	else if (bWasEnabled)
	{
		if (fog_override) fog_override->SetValue(0);
		bWasEnabled = false;
	}
}

// ===================================================================================================
// Precipitation
void CWeather::FindOffsets()
{
	// GetNetVar returns 0 on failure, so keep -1 until we get a real (positive) offset and retry.
	if (m_iPrecipTypeOffset > 0 && m_iMinsOffset > 0 && m_iMaxsOffset > 0)
		return;

	if (m_iPrecipTypeOffset <= 0)
	{ int o = U::NetVars.GetNetVar("CPrecipitation", "m_nPrecipType"); if (o > 0) m_iPrecipTypeOffset = o; }
	if (m_iMinsOffset <= 0)
	{ int o = U::NetVars.GetNetVar("CBaseEntity", "m_vecMins"); if (o > 0) m_iMinsOffset = o; }
	if (m_iMaxsOffset <= 0)
	{ int o = U::NetVars.GetNetVar("CBaseEntity", "m_vecMaxs"); if (o > 0) m_iMaxsOffset = o; }
}

// IClientNetworkable update order (matches the SDK): set m_nPrecipType -> PreDataUpdate ->
// OnPreDataChanged -> set mins/maxs -> OnDataChanged -> PostDataUpdate.
void CWeather::UpdateChain(void* pNetVoid, void* pEnt, int iPrecipType, bool bSetBounds)
{
	if (!pNetVoid || !pEnt)
		return;

	auto pNet = static_cast<IClientNetworkable*>(pNetVoid);
	__try
	{
		if (m_iPrecipTypeOffset > 0)
			*reinterpret_cast<int*>(reinterpret_cast<uintptr_t>(pEnt) + m_iPrecipTypeOffset) = iPrecipType;

		pNet->PreDataUpdate(DATA_UPDATE_CREATED);
		pNet->OnPreDataChanged(DATA_UPDATE_CREATED);

		if (m_iMinsOffset > 0 && m_iMaxsOffset > 0)
		{
			float* mins = reinterpret_cast<float*>(reinterpret_cast<uintptr_t>(pEnt) + m_iMinsOffset);
			float* maxs = reinterpret_cast<float*>(reinterpret_cast<uintptr_t>(pEnt) + m_iMaxsOffset);
			if (bSetBounds)
			{
				mins[0] = mins[1] = mins[2] = -8192.f; // covers any TF2 map
				maxs[0] = maxs[1] = maxs[2] =  8192.f;
			}
			else
			{
				memset(mins, 0, 12); memset(maxs, 0, 12);
			}
		}

		pNet->OnDataChanged(DATA_UPDATE_CREATED);
		pNet->PostDataUpdate(DATA_UPDATE_CREATED);
	}
	__except (EXCEPTION_EXECUTE_HANDLER)
	{
		m_pPrecipNetworkable = nullptr;
		m_iLastMode = -99;
	}
}

void CWeather::ApplyPrecipConVars()
{
	static auto r_rainalpha   = H::ConVars.FindVar("r_rainalpha");
	static auto r_rainspeed   = H::ConVars.FindVar("r_rainspeed");
	static auto r_rainwidth   = H::ConVars.FindVar("r_rainwidth");
	static auto r_rainlength  = H::ConVars.FindVar("r_rainlength");
	static auto r_RainRadius  = H::ConVars.FindVar("r_RainRadius");
	static auto r_RainSideVel = H::ConVars.FindVar("r_RainSideVel");

	const int iMode = Vars::Visuals::Weather::Precipitation.Value;
	if (iMode == Vars::Visuals::Weather::PrecipitationEnum::Rain)
	{
		if (r_rainalpha)   r_rainalpha->SetValue(Vars::Visuals::Weather::RainAlpha.Value);
		if (r_rainspeed)   r_rainspeed->SetValue(Vars::Visuals::Weather::RainSpeed.Value);
		if (r_rainwidth)   r_rainwidth->SetValue(Vars::Visuals::Weather::RainWidth.Value);
		if (r_rainlength)  r_rainlength->SetValue(Vars::Visuals::Weather::RainLength.Value);
		if (r_RainRadius)  r_RainRadius->SetValue(Vars::Visuals::Weather::RainRadius.Value);
		if (r_RainSideVel) r_RainSideVel->SetValue(Vars::Visuals::Weather::RainSideVel.Value);
	}
	else if (iMode == Vars::Visuals::Weather::PrecipitationEnum::Snow)
	{
		if (r_rainalpha)   r_rainalpha->SetValue(Vars::Visuals::Weather::SnowAlpha.Value);
		if (r_rainspeed)   r_rainspeed->SetValue(Vars::Visuals::Weather::SnowSpeed.Value);
		if (r_rainwidth)   r_rainwidth->SetValue(Vars::Visuals::Weather::SnowWidth.Value);
		if (r_rainlength)  r_rainlength->SetValue(Vars::Visuals::Weather::SnowLength.Value);
		if (r_RainRadius)  r_RainRadius->SetValue(Vars::Visuals::Weather::SnowRadius.Value * std::max(0.1f, Vars::Visuals::Weather::SnowDensity.Value));
		if (r_RainSideVel) r_RainSideVel->SetValue(Vars::Visuals::Weather::SnowSideVel.Value);
	}
}

void CWeather::TearDown()
{
	void* pEnt = I::ClientEntityList->GetClientEntity(kSlot);
	void* pNet = I::ClientEntityList->GetClientNetworkable(kSlot);
	if (pEnt && pNet)
		UpdateChain(pNet, pEnt, 0, false);
	m_pPrecipNetworkable = nullptr;
}

// Scan client.dll for s_pMaterial_Rain: find the "particle/rain" literal, the LEA that loads it,
// then the adjacent RIP-relative word-store that writes the FindMaterial result back.
void CWeather::FindMatHandleAddr()
{
	if (m_bMatHandleSearched)
		return;
	m_bMatHandleSearched = true;

	HMODULE hClient = GetModuleHandleA("client.dll");
	if (!hClient)
	{
		m_bMatHandleSearched = false; // retry next time once client.dll is loaded
		return;
	}

	MODULEINFO mi{};
	GetModuleInformation(GetCurrentProcess(), hClient, &mi, sizeof(mi));
	uint8_t* base = static_cast<uint8_t*>(mi.lpBaseOfDll);
	size_t   sz   = mi.SizeOfImage;

	__try
	{
		const char* kStr = "particle/rain";
		const size_t kStrLen = 13;
		uint8_t* strAddr = nullptr;
		for (size_t i = 0; i + kStrLen + 1 <= sz; ++i)
		{
			if (base[i] == 'p' && memcmp(base + i, kStr, kStrLen) == 0 && base[i + kStrLen] == '\0')
			{
				strAddr = base + i;
				break;
			}
		}
		if (!strAddr)
			return;

		// lea rdx/r8/rcx, [rip+rel32]: 48/4C 8D 15/05/0D rel32
		for (size_t i = 0; i + 7 <= sz; ++i)
		{
			uint8_t* p = base + i;
			if (!((p[0] == 0x48 || p[0] == 0x4C) && p[1] == 0x8D && (p[2] == 0x15 || p[2] == 0x05 || p[2] == 0x0D)))
				continue;

			int32_t rel = *reinterpret_cast<int32_t*>(p + 3);
			if (p + 7 + rel != strAddr)
				continue;

			// within +/-128 bytes, a RIP-relative word store: 66 89 05/0D/15 rel32
			for (int j = -128; j < 128; ++j)
			{
				uint8_t* q = p + j;
				if (q < base || q + 7 > base + sz)
					continue;
				if (q[0] != 0x66 || q[1] != 0x89)
					continue;
				if (q[2] != 0x05 && q[2] != 0x0D && q[2] != 0x15)
					continue;

				int32_t rel2 = *reinterpret_cast<int32_t*>(q + 3);
				unsigned short* addr = reinterpret_cast<unsigned short*>(q + 7 + rel2);
				MEMORY_BASIC_INFORMATION mbi{};
				if (VirtualQuery(addr, &mbi, sizeof(mbi)) && mbi.State == MEM_COMMIT &&
					(mbi.Protect & (PAGE_READWRITE | PAGE_WRITECOPY | PAGE_EXECUTE_READWRITE | PAGE_EXECUTE_WRITECOPY)))
				{
					m_pMatHandle = addr;
					return;
				}
			}
		}
	}
	__except (EXCEPTION_EXECUTE_HANDLER) {}
}

void CWeather::InvalidateMatHandle()
{
	if (!m_pMatHandle)
		return;
	__try { *m_pMatHandle = 0xFFFF; /* INVALID_MATERIAL_HANDLE */ }
	__except (EXCEPTION_EXECUTE_HANDLER) {}
}

void CWeather::RunPrecipitation()
{
	const int iMode = Vars::Visuals::Weather::Precipitation.Value;

	// Off / disabled.
	if (iMode == Vars::Visuals::Weather::PrecipitationEnum::Off)
	{
		if (m_iLastMode == 0)
			return;
		TearDown();
		m_iLastMode = 0;
		return;
	}

	// Throttle entity management to every 6 frames; push convars every frame (cheap).
	bool bFullTick = (++m_iTickSkip >= 6);
	if (bFullTick)
		m_iTickSkip = 0;

	if (!bFullTick)
	{
		if (m_iLastMode == iMode)
			ApplyPrecipConVars();
		return;
	}

	// Find the CPrecipitation ClientClass lazily.
	if (!m_pPrecipClass)
	{
		for (auto* p = I::Client->GetAllClasses(); p; p = p->m_pNext)
		{
			if (p->m_pNetworkName && !strcmp(p->m_pNetworkName, "CPrecipitation"))
			{
				m_pPrecipClass = p;
				break;
			}
		}
		if (!m_pPrecipClass)
			return;
	}

	FindOffsets();
	if (m_iPrecipTypeOffset <= 0 || m_iMinsOffset <= 0 || m_iMaxsOffset <= 0)
		return;

	// Spawn the sprite entity at the spare slot if absent.
	void* pEnt = I::ClientEntityList->GetClientEntity(kSlot);
	if (!pEnt)
	{
		auto* cls = static_cast<ClientClass*>(m_pPrecipClass);
		if (!cls->m_pCreateFn)
			return;
		__try
		{
			m_pPrecipNetworkable = cls->m_pCreateFn(kSlot, 0);
		}
		__except (EXCEPTION_EXECUTE_HANDLER)
		{
			m_pPrecipNetworkable = nullptr;
			return;
		}
		pEnt = I::ClientEntityList->GetClientEntity(kSlot);
		if (!pEnt)
		{
			m_pPrecipNetworkable = nullptr;
			return;
		}
		m_pPrecipNetworkable = I::ClientEntityList->GetClientNetworkable(kSlot);
		m_iLastMode = -99;
	}
	else if (!m_pPrecipNetworkable)
	{
		m_pPrecipNetworkable = I::ClientEntityList->GetClientNetworkable(kSlot);
	}

	ApplyPrecipConVars();
	FindMatHandleAddr();

	// Snow sprite name changed while running -> force a re-lookup.
	if (iMode == Vars::Visuals::Weather::PrecipitationEnum::Snow)
	{
		uint32_t uHash = FNV1A::Hash32(Vars::Visuals::Weather::SnowSprite.Value.c_str());
		if (uHash != m_uLastSpriteHash)
		{
			m_uLastSpriteHash = uHash;
			m_iLastMode = -99;
		}
	}

	if (m_iLastMode != iMode)
	{
		// Reset the cached material handle so OnDataChanged re-calls FindMaterial (our hook redirects).
		InvalidateMatHandle();
		const int iPrecipType = (iMode == Vars::Visuals::Weather::PrecipitationEnum::Snow) ? 1 : 0;
		UpdateChain(m_pPrecipNetworkable, pEnt, iPrecipType, true);
		m_iLastMode = iMode;
	}
}

void CWeather::Run()
{
	if (G::Unload)
		return;

	ApplyFog();

	if (!I::EngineClient->IsConnected() || !I::EngineClient->IsInGame())
	{
		// Map change / disconnect destroys the entity; forget it so we respawn cleanly.
		m_pPrecipNetworkable = nullptr;
		m_iLastMode = -99;
		return;
	}

	RunPrecipitation();
}

void CWeather::Unload()
{
	if (auto fog_override = H::ConVars.FindVar("fog_override"))
		fog_override->SetValue(0);

	if (m_iLastMode == 1 || m_iLastMode == 2)
		TearDown();
	m_iLastMode = -99;
}
