#include "FlipWorld.h"

#include "../Materials/Materials.h"
#include "../CameraWindow/CameraWindow.h"

// Horizontally mirror the rendered 3D frame. Reversing the source X texcoords (right edge -> left
// edge) flips the captured image; the world ends up mirrored, the pre-flipped viewmodel ends up
void CFlipWorld::Render(const CViewSetup* pView)
{
	// G::FlipWorldActive already folds in the feature toggle + local-alive gate (set in CMisc::RunPre).
	// Skip the CameraWindow's re-entrant render-to-texture pass so we only mirror the real screen.
	if (!pView || !G::FlipWorldActive || G::Unload || SDK::CleanScreenshot() || F::CameraWindow.m_bDrawing)
		return;
	if (!m_pFlipMaterial || !m_pFlipTexture)
		return;

	auto pRenderContext = I::MaterialSystem->GetRenderContext();
	if (!pRenderContext)
		return;

	const int iWidth = m_pFlipTexture->GetActualWidth();
	const int iHeight = m_pFlipTexture->GetActualHeight();

	// Copy the current world frame into our texture, then draw it back over the screen mirrored.
	pRenderContext->CopyRenderTargetToTexture(m_pFlipTexture);
	pRenderContext->DrawScreenSpaceRectangle(
		m_pFlipMaterial,
		pView->x, pView->y, pView->width, pView->height,
		float(iWidth), 0.f,   // top-right source corner
		0.f, float(iHeight),  // bottom-left source corner -> reversed X = horizontal mirror
		iWidth, iHeight,
		nullptr, 1, 1);
	pRenderContext->Release();
}

void CFlipWorld::Initialize()
{
	if (!m_pFlipMaterial)
	{
		KeyValues* kv = new KeyValues("UnlitGeneric");
		kv->SetString("$basetexture", "m_pFlipWorldTexture");
		kv->SetInt("$ignorez", 1); // full-screen overlay, draw over the scene regardless of depth
		m_pFlipMaterial = F::Materials.Create("FlipWorldMaterial", kv);
	}

	if (!m_pFlipTexture)
	{
		m_pFlipTexture = I::MaterialSystem->CreateNamedRenderTargetTextureEx(
			"m_pFlipWorldTexture",
			1,
			1,
			RT_SIZE_FULL_FRAME_BUFFER,
			IMAGE_FORMAT_RGB888,
			MATERIAL_RT_DEPTH_SHARED,
			TEXTUREFLAGS_CLAMPS | TEXTUREFLAGS_CLAMPT,
			CREATERENDERTARGETFLAGS_HDR
		);
		m_pFlipTexture->IncrementReferenceCount();
	}
}

void CFlipWorld::Unload()
{
	if (m_pFlipMaterial)
	{
		m_pFlipMaterial->DecrementReferenceCount();
		m_pFlipMaterial->DeleteIfUnreferenced();
		m_pFlipMaterial = nullptr;
	}

	if (m_pFlipTexture)
	{
		m_pFlipTexture->DecrementReferenceCount();
		m_pFlipTexture->DeleteIfUnreferenced();
		m_pFlipTexture = nullptr;
	}
}
