#pragma once
#include "../../../SDK/SDK.h"

// Flip world (Visuals > World > Flip world). Mirrors the rendered world frame horizontally in screen
// space: from the CClientModeShared::DoPostScreenSpaceEffects hook (which fires AFTER the world + ESP
class CFlipWorld
{
private:
	IMaterial* m_pFlipMaterial = nullptr;
	ITexture* m_pFlipTexture = nullptr;

public:
	// Called from the CClientModeShared_DoPostScreenSpaceEffects hook (main view only).
	void Render(const CViewSetup* pView);

	void Initialize();
	void Unload();
};

ADD_FEATURE(CFlipWorld, FlipWorld);
