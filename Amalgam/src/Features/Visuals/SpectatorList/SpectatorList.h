#pragma once
#include "../../../SDK/SDK.h"

class CSpectatorList
{
public:
	struct Spectator_t
	{
		std::string m_sName;
		const char* m_sMode;
		float m_flRespawnIn;
		bool m_bRespawnTimeIncreased;
		int m_iIndex;
		uint32_t m_uAccountID; // resolved on the main thread at gather time (Present-thread reads race -> 0)
	};

private:
	std::vector<Spectator_t> m_vSpectators = {};
	std::unordered_map<int, float> m_mRespawnCache = {};

	// Minimal-style background: a surface texture mirroring the Config > STYLE effect (Balatro/Blur),
	// re-uploaded each frame. Recreated when the effect buffer's size changes (Balatro vs Blur).
	int m_iStyleTex = -1, m_iStyleTexW = 0, m_iStyleTexH = 0;

public:
	bool GetSpectators(CTFPlayer* pTarget);
	void Draw(CTFPlayer* pLocal);

	// Minimal style is drawn in the ImGui pass (CMenu::DrawSpectatorListMinimal) so it matches the
	// menu's exact styled background. The actual GATHER runs on the main thread in Draw() (the rows +
	bool PrepareMinimal(CTFPlayer* pLocal);
	const std::vector<Spectator_t>& Rows() const { return m_vSpectators; }
};

ADD_FEATURE(CSpectatorList, SpectatorList);