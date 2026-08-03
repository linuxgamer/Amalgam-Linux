#include "../SDK/SDK.h"

#include "../Features/Visuals/ModelPreview/ModelPreview.h"

// While in the main menu the engine's model lookup returns null even after we point
// the entity at a model, which crashes SetModelByIndex / DrawModel. Return our preview
MAKE_HOOK(CModelInfoClient_GetModel,
	U::Memory.FindSignature("engine.dll", "83 FA ? 0F 8C ? ? ? ? 48 8D 0D ? ? ? ? E9 ? ? ? ? CC CC CC CC CC CC CC CC CC CC CC 48 89 5C 24"),
	void*, void* rcx, int iModelIndex)
{
	// In the menu the engine's model lookup returns null, so SetModelByIndex stores a null
	if (!I::EngineClient->IsConnected() && F::ModelPreview.WantRender())
	{
		if (auto pModel = F::ModelPreview.GetModelPtr())
			return const_cast<void*>(reinterpret_cast<const void*>(pModel));
	}

	return CALL_ORIGINAL(rcx, iModelIndex);
}
