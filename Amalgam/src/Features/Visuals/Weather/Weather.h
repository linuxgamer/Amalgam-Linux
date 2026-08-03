#pragma once
#include "../../../SDK/SDK.h"

// Fog changer + native precipitation (rain / snow),.
class CWeather
{
private:
	static constexpr int kSlot = 2047; // spare client entity slot for the CPrecipitation sprite

	void* m_pPrecipClass = nullptr;       // ClientClass*
	void* m_pPrecipNetworkable = nullptr; // IClientNetworkable*
	int m_iPrecipTypeOffset = -1;
	int m_iMinsOffset = -1;
	int m_iMaxsOffset = -1;
	int m_iLastMode = -99;                // last applied precip mode (0 off / 1 rain / 2 snow)
	int m_iTickSkip = 0;

	unsigned short* m_pMatHandle = nullptr; // client.dll s_pMaterial_Rain static
	bool m_bMatHandleSearched = false;
	uint32_t m_uLastSpriteHash = 0;         // detect snow-sprite name changes

	void ApplyFog();
	void RunPrecipitation();
	void FindOffsets();
	void UpdateChain(void* pNet, void* pEnt, int iPrecipType, bool bSetBounds);
	void ApplyPrecipConVars();
	void TearDown();
	void FindMatHandleAddr();
	void InvalidateMatHandle();

public:
	void Run();    // every frame, FRAME_RENDER_START
	void Unload(); // restore fog convars + kill the precip entity
};

ADD_FEATURE(CWeather, Weather);
