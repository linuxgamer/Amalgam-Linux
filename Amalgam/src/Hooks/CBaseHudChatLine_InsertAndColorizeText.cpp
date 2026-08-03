#include "../SDK/SDK.h"

#include "../Features/Players/PlayerUtils.h"

MAKE_SIGNATURE(CBaseHudChatLine_InsertAndColorizeText, "client.dll", "44 89 44 24 ? 55 53 56 57", 0x0);

MAKE_HOOK(CBaseHudChatLine_InsertAndColorizeText, S::CBaseHudChatLine_InsertAndColorizeText(), void,
	void* rcx, wchar_t* buf, int clientIndex)
{
#ifdef DEBUG_HOOKS
	if (!Vars::Hooks::CBaseHudChatLine_InsertAndColorizeText[DEFAULT_BIND])
		return CALL_ORIGINAL(rcx, buf, clientIndex);
#endif

	std::string sMessage = SDK::ConvertWideToUTF8(buf);

	if (clientIndex)
	{
		auto pResource = H::Entities.GetResource();
		if (!pResource)
			return CALL_ORIGINAL(rcx, buf, clientIndex);

		const char* sName = pResource->GetName(clientIndex);
		auto iFind = sMessage.find(sName);

		int iType;
		if (const char* sReplace = F::PlayerUtils.GetPlayerName(clientIndex, nullptr, &iType))
		{
			if (iFind != std::string::npos)
				sMessage = sMessage.replace(std::max(iFind - 1, 0ui64), strlen(sName) + 1, std::format("\x3{}\x1", sReplace));
			sName = sReplace;
		}

		if (Vars::Visuals::UI::ChatTags.Value && !(iType & NameTypeEnum::Privacy))
		{
			std::string sTag, cColor;
			if (Vars::Visuals::UI::ChatTags.Value & Vars::Visuals::UI::ChatTagsEnum::Local && clientIndex == I::EngineClient->GetLocalPlayer())
				sTag = "You", cColor = Vars::Colors::Local.Value.ToHexA();
			else if (Vars::Visuals::UI::ChatTags.Value & Vars::Visuals::UI::ChatTagsEnum::Friends && H::Entities.IsFriend(clientIndex))
				sTag = "Friend", cColor = F::PlayerUtils.m_vTags[F::PlayerUtils.TagToIndex(FRIEND_TAG)].m_tColor.ToHexA();
			else if (Vars::Visuals::UI::ChatTags.Value & Vars::Visuals::UI::ChatTagsEnum::Party && H::Entities.InParty(clientIndex))
				sTag = "Party", cColor = F::PlayerUtils.m_vTags[F::PlayerUtils.TagToIndex(PARTY_TAG)].m_tColor.ToHexA();
			else if (Vars::Visuals::UI::ChatTags.Value & Vars::Visuals::UI::ChatTagsEnum::Assigned)
			{
				if (auto pTag = F::PlayerUtils.GetSignificantTag(clientIndex, 0))
					sTag = pTag->m_sName, cColor = pTag->m_tColor.ToHexA();
			}

			if (!sTag.empty())
			{
				if (iFind != std::string::npos)
					sMessage.insert(iFind + strlen(sName), "\x1");
				sMessage.insert(0, std::format("{}[{}] \x3", cColor, sTag));
			}
		}
	}

	CALL_ORIGINAL(rcx, const_cast<wchar_t*>(SDK::ConvertUtf8ToWide(sMessage).c_str()), clientIndex);
}