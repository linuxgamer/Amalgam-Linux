#include "../SDK/SDK.h"

#include "../Features/Players/PlayerUtils.h"

MAKE_SIGNATURE(GetPlayerNameForSteamID_GetFriendPersonaName_Call, "client.dll", "41 B9 ? ? ? ? 44 8B C3 48 8B C8", 0x0);

MAKE_HOOK(ISteamFriends_GetFriendPersonaName, U::Memory.GetVirtual(I::SteamFriends, 7), const char*,
	void* rcx, CSteamID steamIDFriend)
{
#ifdef DEBUG_HOOKS
	if (!Vars::Hooks::ISteamFriends_GetFriendPersonaName[DEFAULT_BIND])
		return CALL_ORIGINAL(rcx, steamIDFriend);
#endif

	const auto dwDesired = S::GetPlayerNameForSteamID_GetFriendPersonaName_Call();
	const auto dwRetAddr = uintptr_t(_ReturnAddress());

	if (dwRetAddr == dwDesired)
	{
		// the reference player name changer: return the overridden name - the streamer-mode mask for
		// other players (Player), your own custom/cycled display name or a playerlist alias (Custom)
		int iType; const char* sName = F::PlayerUtils.GetPlayerName(steamIDFriend.GetAccountID(), nullptr, &iType);
		if (sName && (iType == NameTypeEnum::Player || iType == NameTypeEnum::Custom))
			return sName;
	}

	return CALL_ORIGINAL(rcx, steamIDFriend);
}