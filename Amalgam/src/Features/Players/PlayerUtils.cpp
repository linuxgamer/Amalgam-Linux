#include "PlayerUtils.h"

#include "../ImGui/Menu/Menu.h"
#include "../Output/Output.h"
#include "../../SDK/Definitions/Types.h"

uint32_t CPlayerlistUtils::GetAccountID(int iIndex)
{
	auto pResource = H::Entities.GetResource();
	if (pResource && pResource->m_bValid(iIndex) && !pResource->IsFakePlayer(iIndex))
		return pResource->m_iAccountID(iIndex);
	return 0;
}

int CPlayerlistUtils::GetIndex(uint32_t uAccountID)
{
	auto pResource = H::Entities.GetResource();
	if (!pResource)
		return 0;
	for (int n = 1; n <= I::EngineClient->GetMaxClients(); n++)
	{
		if (pResource->m_bValid(n) && !pResource->IsFakePlayer(n) && pResource->m_iAccountID(n) == uAccountID)
			return n;
	}
	return 0;
}

PriorityLabel_t* CPlayerlistUtils::GetTag(int iID)
{
	if (iID > -1 && iID < m_vTags.size())
		return &m_vTags[iID];

	return nullptr;
}

int CPlayerlistUtils::GetTag(const std::string& sTag)
{
	auto uHash = FNV1A::Hash32(sTag.c_str());

	int iID = -1;
	for (auto& tTag : m_vTags)
	{
		iID++;
		if (uHash == FNV1A::Hash32(tTag.m_sName.c_str()))
			return iID;
	}

	return -1;
}



void CPlayerlistUtils::AddTag(uint32_t uAccountID, int iID, bool bSave, const char* sName, std::unordered_map<uint32_t, std::vector<int>>& mPlayerTags)
{
	if (!uAccountID)
		return;

	if (!HasTag(uAccountID, iID))
	{
		mPlayerTags[uAccountID].push_back(iID);
		m_bSave = bSave;
		if (auto pTag = GetTag(iID); pTag && sName)
			F::Output.TagsChanged(sName, "Added", pTag->m_tColor.ToHexA().c_str(), pTag->m_sName.c_str());
	}
}
void CPlayerlistUtils::AddTag(int iIndex, int iID, bool bSave, const char* sName, std::unordered_map<uint32_t, std::vector<int>>& mPlayerTags)
{
	AddTag(GetAccountID(iIndex), iID, bSave, sName, mPlayerTags);
}
void CPlayerlistUtils::AddTag(uint32_t uAccountID, int iID, bool bSave, const char* sName)
{
	AddTag(uAccountID, iID, bSave, sName, m_mPlayerTags);
}
void CPlayerlistUtils::AddTag(int iIndex, int iID, bool bSave, const char* sName)
{
	AddTag(iIndex, iID, bSave, sName, m_mPlayerTags);
}

void CPlayerlistUtils::RemoveTag(uint32_t uAccountID, int iID, bool bSave, const char* sName, std::unordered_map<uint32_t, std::vector<int>>& mPlayerTags)
{
	if (!uAccountID)
		return;

	auto& vTags = mPlayerTags[uAccountID];
	for (auto it = vTags.begin(); it != vTags.end(); it++)
	{
		if (iID == *it)
		{
			vTags.erase(it);
			m_bSave = bSave;
			if (auto pTag = GetTag(iID); pTag && sName)
				F::Output.TagsChanged(sName, "Removed", pTag->m_tColor.ToHexA().c_str(), pTag->m_sName.c_str());
			break;
		}
	}
	if (vTags.empty())
		mPlayerTags.erase(uAccountID);
}
void CPlayerlistUtils::RemoveTag(int iIndex, int iID, bool bSave, const char* sName, std::unordered_map<uint32_t, std::vector<int>>& mPlayerTags)
{
	RemoveTag(GetAccountID(iIndex), iID, bSave, sName, mPlayerTags);
}
void CPlayerlistUtils::RemoveTag(uint32_t uAccountID, int iID, bool bSave, const char* sName)
{
	RemoveTag(uAccountID, iID, bSave, sName, m_mPlayerTags);
}
void CPlayerlistUtils::RemoveTag(int iIndex, int iID, bool bSave, const char* sName)
{
	RemoveTag(iIndex, iID, bSave, sName, m_mPlayerTags);
}

bool CPlayerlistUtils::HasTags(uint32_t uAccountID, std::unordered_map<uint32_t, std::vector<int>>& mPlayerTags)
{
	if (!uAccountID)
		return false;

	return !mPlayerTags[uAccountID].empty();
}
bool CPlayerlistUtils::HasTags(int iIndex, std::unordered_map<uint32_t, std::vector<int>>& mPlayerTags)
{
	return HasTags(GetAccountID(iIndex), mPlayerTags);
}
bool CPlayerlistUtils::HasTags(uint32_t uAccountID)
{
	return HasTags(uAccountID, m_mPlayerTags);
}
bool CPlayerlistUtils::HasTags(int iIndex)
{
	return HasTags(iIndex, m_mPlayerTags);
}

bool CPlayerlistUtils::HasTag(uint32_t uAccountID, int iID, std::unordered_map<uint32_t, std::vector<int>>& mPlayerTags)
{
	if (!uAccountID)
		return false;

	auto it = std::ranges::find_if(mPlayerTags[uAccountID], [iID](const auto& _iID) { return iID == _iID; });
	return it != mPlayerTags[uAccountID].end();
}
bool CPlayerlistUtils::HasTag(int iIndex, int iID, std::unordered_map<uint32_t, std::vector<int>>& mPlayerTags)
{
	return HasTag(GetAccountID(iIndex), iID, mPlayerTags);
}
bool CPlayerlistUtils::HasTag(uint32_t uAccountID, int iID)
{
	return HasTag(uAccountID, iID, m_mPlayerTags);
}
bool CPlayerlistUtils::HasTag(int iIndex, int iID)
{
	return HasTag(iIndex, iID, m_mPlayerTags);
}



int CPlayerlistUtils::GetPriority(uint32_t uAccountID, bool bCache)
{
	if (bCache)
		return H::Entities.GetPriority(uAccountID);

	const int iDefault = m_vTags[TagToIndex(DEFAULT_TAG)].m_iPriority;
	if (!uAccountID)
		return iDefault;

	if (HasTag(uAccountID, TagToIndex(IGNORED_TAG)))
		return m_vTags[TagToIndex(IGNORED_TAG)].m_iPriority;

	std::vector<int> vPriorities;
	if (m_mPlayerTags.contains(uAccountID))
	{
		for (auto& iID : m_mPlayerTags[uAccountID])
		{
			auto pTag = GetTag(iID);
			if (pTag && !pTag->m_bLabel)
				vPriorities.push_back(pTag->m_iPriority);
		}
	}
	if (H::Entities.IsFriend(uAccountID))
	{
		auto& tTag = m_vTags[TagToIndex(FRIEND_TAG)];
		if (!tTag.m_bLabel)
			vPriorities.push_back(tTag.m_iPriority);
	}
	if (H::Entities.InParty(uAccountID))
	{
		auto& tTag = m_vTags[TagToIndex(PARTY_TAG)];
		if (!tTag.m_bLabel)
			vPriorities.push_back(tTag.m_iPriority);
	}
	if (H::Entities.IsF2P(uAccountID))
	{
		auto& tTag = m_vTags[TagToIndex(F2P_TAG)];
		if (!tTag.m_bLabel)
			vPriorities.push_back(tTag.m_iPriority);
	}
	if (vPriorities.empty())
		return iDefault;

	std::sort(vPriorities.begin(), vPriorities.end(), std::greater<int>());
	return vPriorities.front();
}
int CPlayerlistUtils::GetPriority(int iIndex, bool bCache)
{
	if (bCache)
		return H::Entities.GetPriority(iIndex);

	return GetPriority(GetAccountID(iIndex));
}

PriorityLabel_t* CPlayerlistUtils::GetSignificantTag(uint32_t uAccountID, int iMode)
{
	if (!uAccountID)
		return nullptr;

	std::vector<PriorityLabel_t*> vTags;
	if (!iMode || iMode == 1)
	{
		if (HasTag(uAccountID, TagToIndex(IGNORED_TAG)))
			return &m_vTags[TagToIndex(IGNORED_TAG)];

		if (m_mPlayerTags.contains(uAccountID))
		{
			for (auto& iID : m_mPlayerTags[uAccountID])
			{
				PriorityLabel_t* _pTag = GetTag(iID);
				if (_pTag && !_pTag->m_bLabel)
					vTags.push_back(_pTag);
			}
		}
		if (H::Entities.IsFriend(uAccountID))
		{
			auto _pTag = &m_vTags[TagToIndex(FRIEND_TAG)];
			if (!_pTag->m_bLabel)
				vTags.push_back(_pTag);
		}
		if (H::Entities.InParty(uAccountID))
		{
			auto _pTag = &m_vTags[TagToIndex(PARTY_TAG)];
			if (!_pTag->m_bLabel)
				vTags.push_back(_pTag);
		}
		if (H::Entities.IsF2P(uAccountID))
		{
			auto _pTag = &m_vTags[TagToIndex(F2P_TAG)];
			if (!_pTag->m_bLabel)
				vTags.push_back(_pTag);
		}
	}
	if ((!iMode || iMode == 2) && !vTags.size())
	{
		if (m_mPlayerTags.contains(uAccountID))
		{
			for (auto& iID : m_mPlayerTags[uAccountID])
			{
				PriorityLabel_t* _pTag = GetTag(iID);
				if (_pTag && _pTag->m_bLabel)
					vTags.push_back(_pTag);
			}
		}
		if (H::Entities.IsFriend(uAccountID))
		{
			auto _pTag = &m_vTags[TagToIndex(FRIEND_TAG)];
			if (_pTag->m_bLabel)
				vTags.push_back(_pTag);
		}
		if (H::Entities.InParty(uAccountID))
		{
			auto _pTag = &m_vTags[TagToIndex(PARTY_TAG)];
			if (_pTag->m_bLabel)
				vTags.push_back(_pTag);
		}
		if (H::Entities.IsF2P(uAccountID))
		{
			auto _pTag = &m_vTags[TagToIndex(F2P_TAG)];
			if (_pTag->m_bLabel)
				vTags.push_back(_pTag);
		}
	}
	if (vTags.empty())
		return nullptr;

	std::sort(vTags.begin(), vTags.end(), [&](const PriorityLabel_t* a, const PriorityLabel_t* b) -> bool
		{
			// sort by priority if unequal
			if (a->m_iPriority != b->m_iPriority)
				return a->m_iPriority > b->m_iPriority;

			return a->m_sName < b->m_sName;
		});
	return vTags.front();
}
PriorityLabel_t* CPlayerlistUtils::GetSignificantTag(int iIndex, int iMode)
{
	return GetSignificantTag(GetAccountID(iIndex), iMode);
}

bool CPlayerlistUtils::IsIgnored(uint32_t uAccountID)
{
	const int iPriority = GetPriority(uAccountID);
	const int iIgnored = m_vTags[TagToIndex(IGNORED_TAG)].m_iPriority;
	return iPriority <= iIgnored;
}
bool CPlayerlistUtils::IsIgnored(int iIndex)
{
	return IsIgnored(GetAccountID(iIndex));
}

bool CPlayerlistUtils::IsPrioritized(uint32_t uAccountID)
{
	if (!uAccountID)
		return false;

	const int iPriority = GetPriority(uAccountID);
	const int iDefault = m_vTags[TagToIndex(DEFAULT_TAG)].m_iPriority;
	return iPriority > iDefault;
}
bool CPlayerlistUtils::IsPrioritized(int iIndex)
{
	return IsPrioritized(GetAccountID(iIndex));
}



// the reference name changer: TF2 class index (TF_CLASS_*) -> display word. Index 0 is undefined.
static const char* const s_StreamerClassNames[10] = {
	"unknown", "scout", "sniper", "soldier", "demoman",
	"medic",   "heavy", "pyro",   "spy",     "engineer"
};

// Generic newline-list cycler shared by the local-name cycler and the three streamer label
// cyclers - mirrors the NameEntry_t list cyclers (g_nameList, g_teamLabelList,...).
struct NameCycler_t
{
	std::vector<std::string> m_vEntries;
	std::string m_sCache = "\n"; // sentinel: never equals a real list value, forces the first parse
	std::string m_sCurrent;
	int m_iIdx = 0;
	ULONGLONG m_ullLast = 0;

	void Rebuild(const std::string& sList)
	{
		if (sList == m_sCache)
			return;
		m_sCache = sList;
		m_vEntries.clear();
		size_t iStart = 0;
		while (iStart <= sList.size() && !sList.empty())
		{
			size_t iEnd = sList.find('\n', iStart);
			if (iEnd == std::string::npos) iEnd = sList.size();
			std::string sLine = sList.substr(iStart, iEnd - iStart);
			if (!sLine.empty() && sLine.back() == '\r') sLine.pop_back();
			if (!sLine.empty()) m_vEntries.push_back(sLine);
			if (iEnd == sList.size()) break;
			iStart = iEnd + 1;
		}
		if (m_iIdx >= (int)m_vEntries.size()) m_iIdx = 0;
	}

	// Returns the current entry's c_str(), or nullptr when the list is empty. Advances on the
	// shared interval timer when bTimer is set.
	const char* Current(const std::string& sList, bool bTimer, float flInterval)
	{
		Rebuild(sList);
		if (m_vEntries.empty())
			return nullptr;
		if (bTimer)
		{
			const ULONGLONG ullNow = GetTickCount64();
			const ULONGLONG ullInterval = ULONGLONG(flInterval * 1000.f);
			if (m_ullLast == 0) m_ullLast = ullNow;
			if (ullNow - m_ullLast >= ullInterval)
			{
				m_iIdx = (m_iIdx + 1) % (int)m_vEntries.size();
				m_ullLast = ullNow;
			}
		}
		if (m_iIdx < 0 || m_iIdx >= (int)m_vEntries.size()) m_iIdx = 0;
		m_sCurrent = m_vEntries[m_iIdx];
		return m_sCurrent.c_str();
	}

	void Advance(const std::string& sList)
	{
		Rebuild(sList);
		if (m_vEntries.empty())
			return;
		m_iIdx = (m_iIdx + 1) % (int)m_vEntries.size();
	}
};

static NameCycler_t s_LocalNameCycler;
static NameCycler_t s_TeamLabelCycler;
static NameCycler_t s_EnemyLabelCycler;
static NameCycler_t s_EveryoneLabelCycler;

// Produce the masked label for a non-local player given the active streamer mode.
static const char* GetStreamerMaskLabel(int iIndex)
{
	const bool bTimer = Vars::Visuals::UI::NameCycleTrigger.Value == Vars::Visuals::UI::NameCycleTriggerEnum::Timer;
	const float flInterval = Vars::Visuals::UI::NameCycleInterval.Value;

	switch (Vars::Visuals::UI::StreamerMode.Value)
	{
	case Vars::Visuals::UI::StreamerModeEnum::Class:
	{
		// the reference case 1: TF2 class word; a class index outside 1..9 (unknown / out of PVS) -> "unknown".
		int iClass = 0;
		if (auto pResource = H::Entities.GetResource(); pResource && pResource->m_bValid(iIndex))
			iClass = pResource->m_iPlayerClass(iIndex);
		return s_StreamerClassNames[(iClass >= 1 && iClass <= 9) ? iClass : 0];
	}
	case Vars::Visuals::UI::StreamerModeEnum::TeamRole:
	{
		// the reference case 2: teammate / enemy label. It returns the literal "player" whenever the teams
		// can't be compared - not in game (localIdx < 1), no resource, or either player unassigned (team 0).
		const int iLocal = I::EngineClient->GetLocalPlayer();
		if (iLocal < 1)
			return "player";
		auto pResource = H::Entities.GetResource();
		if (!pResource || !pResource->m_bValid(iLocal) || !pResource->m_bValid(iIndex))
			return "player";
		const int iLocalTeam = pResource->m_iTeam(iLocal);
		const int iPlayerTeam = pResource->m_iTeam(iIndex);
		if (iLocalTeam == 0 || iPlayerTeam == 0)
			return "player";
		if (iPlayerTeam == iLocalTeam)
		{
			if (Vars::Visuals::UI::TeamLabelCycle.Value)
				if (const char* s = s_TeamLabelCycler.Current(Vars::Visuals::UI::TeamLabelCycleList.Value, bTimer, flInterval))
					return s;
			return Vars::Visuals::UI::StreamerTeamLabel.Value.c_str();
		}
		if (Vars::Visuals::UI::EnemyLabelCycle.Value)
			if (const char* s = s_EnemyLabelCycler.Current(Vars::Visuals::UI::EnemyLabelCycleList.Value, bTimer, flInterval))
				return s;
		return Vars::Visuals::UI::StreamerEnemyLabel.Value.c_str();
	}
	case Vars::Visuals::UI::StreamerModeEnum::Everyone:
	default:
		// the reference case 3: a single shared label for everyone else.
		if (Vars::Visuals::UI::EveryoneLabelCycle.Value)
			if (const char* s = s_EveryoneLabelCycler.Current(Vars::Visuals::UI::EveryoneLabelCycleList.Value, bTimer, flInterval))
				return s;
		return Vars::Visuals::UI::StreamerEveryoneLabel.Value.c_str();
	}
}

const char* CPlayerlistUtils::GetCycledLocalName()
{
	if (!Vars::Visuals::UI::ChangeUsername.Value || !Vars::Visuals::UI::NameCycle.Value)
		return nullptr;
	const bool bTimer = Vars::Visuals::UI::NameCycleTrigger.Value == Vars::Visuals::UI::NameCycleTriggerEnum::Timer;
	return s_LocalNameCycler.Current(Vars::Visuals::UI::NameCycleList.Value, bTimer, Vars::Visuals::UI::NameCycleInterval.Value);
}

void CPlayerlistUtils::AdvanceNameCycle()
{
	// the reference h_FireEvent on-kill block: advance every active cycler when the trigger is OnKill.
	if (Vars::Visuals::UI::NameCycleTrigger.Value != Vars::Visuals::UI::NameCycleTriggerEnum::OnKill)
		return;
	if (Vars::Visuals::UI::ChangeUsername.Value && Vars::Visuals::UI::NameCycle.Value)
		s_LocalNameCycler.Advance(Vars::Visuals::UI::NameCycleList.Value);
	if (Vars::Visuals::UI::StreamerMode.Value == Vars::Visuals::UI::StreamerModeEnum::TeamRole)
	{
		if (Vars::Visuals::UI::TeamLabelCycle.Value)
			s_TeamLabelCycler.Advance(Vars::Visuals::UI::TeamLabelCycleList.Value);
		if (Vars::Visuals::UI::EnemyLabelCycle.Value)
			s_EnemyLabelCycler.Advance(Vars::Visuals::UI::EnemyLabelCycleList.Value);
	}
	if (Vars::Visuals::UI::StreamerMode.Value == Vars::Visuals::UI::StreamerModeEnum::Everyone
		&& Vars::Visuals::UI::EveryoneLabelCycle.Value)
		s_EveryoneLabelCycler.Advance(Vars::Visuals::UI::EveryoneLabelCycleList.Value);
}

int CPlayerlistUtils::GetNameType(int iIndex)
{
	// the reference streamer modes mask every OTHER player (the local player is handled separately by
	// the change-your-username override in GetPlayerName). Aliases take priority over the mask.
	if (const uint32_t uAccountID = GetAccountID(iIndex); uAccountID && GetPlayerAlias(uAccountID))
		return NameTypeEnum::Custom;
	if (Vars::Visuals::UI::StreamerMode.Value && iIndex != I::EngineClient->GetLocalPlayer())
		return NameTypeEnum::Player;
	return NameTypeEnum::None;
}

int CPlayerlistUtils::GetNameType(uint32_t uAccountID)
{
	if (GetPlayerAlias(uAccountID))
		return NameTypeEnum::Custom;
	if (Vars::Visuals::UI::StreamerMode.Value && uAccountID != I::SteamUser->GetSteamID().GetAccountID())
		return NameTypeEnum::Player;
	return NameTypeEnum::None;
}

const char* CPlayerlistUtils::GetPlayerName(int iIndex, const char* sDefault, int* pType)
{
	// the reference player name changer: override your own displayed name. A non-empty name cycle
	// entry takes priority over the single CustomLocalName, exactly names.cpp.
	if (Vars::Visuals::UI::ChangeUsername.Value && iIndex == I::EngineClient->GetLocalPlayer())
	{
		if (const char* sCycle = GetCycledLocalName())
		{
			if (pType) *pType = NameTypeEnum::Custom;
			return sCycle;
		}
		if (!Vars::Visuals::UI::CustomLocalName.Value.empty())
		{
			if (pType) *pType = NameTypeEnum::Custom;
			return Vars::Visuals::UI::CustomLocalName.Value.c_str();
		}
	}

	int iType = GetNameType(iIndex);
	if (pType) *pType = iType;

	switch (iType)
	{
	case NameTypeEnum::Custom:
		if (auto sAlias = GetPlayerAlias(GetAccountID(iIndex)))
			return sAlias->c_str();
		break;
	case NameTypeEnum::Player:
		return GetStreamerMaskLabel(iIndex);
	}
	return sDefault;
}

const char* CPlayerlistUtils::GetPlayerName(uint32_t uAccountID, const char* sDefault, int* pType)
{
	// the reference player name changer: override your own displayed name. A non-empty name cycle
	// entry takes priority over the single CustomLocalName, exactly names.cpp.
	if (Vars::Visuals::UI::ChangeUsername.Value && uAccountID == I::SteamUser->GetSteamID().GetAccountID())
	{
		if (const char* sCycle = GetCycledLocalName())
		{
			if (pType) *pType = NameTypeEnum::Custom;
			return sCycle;
		}
		if (!Vars::Visuals::UI::CustomLocalName.Value.empty())
		{
			if (pType) *pType = NameTypeEnum::Custom;
			return Vars::Visuals::UI::CustomLocalName.Value.c_str();
		}
	}

	int iType = GetNameType(uAccountID);
	if (pType) *pType = iType;

	switch (iType)
	{
	case NameTypeEnum::Custom:
		if (auto sAlias = GetPlayerAlias(uAccountID))
			return sAlias->c_str();
		break;
	case NameTypeEnum::Player:
		if (int iPlayerIndex = GetIndex(uAccountID))
			return GetStreamerMaskLabel(iPlayerIndex);
		return Vars::Visuals::UI::StreamerEveryoneLabel.Value.c_str();
	}
	return sDefault;
}



void CPlayerlistUtils::Store()
{
	static Timer tTimer = {};
	if (!tTimer.Run(1.f))
		return;

	std::lock_guard tLock(F::Menu.m_tMutex);
	m_vPlayerCache.clear();

	auto pResource = H::Entities.GetResource();
	if (!pResource)
		return;

	for (int n = 1; n <= I::EngineClient->GetMaxClients(); n++)
	{
		if (!pResource->m_bValid(n) || !pResource->m_bConnected(n))
			continue;

		uint32_t uAccountID = pResource->m_iAccountID(n);
		m_vPlayerCache.emplace_back(
			pResource->GetName(n),
			uAccountID,
			pResource->m_iUserID(n),
			pResource->m_iTeam(n),
			pResource->m_bAlive(n),
			n == I::EngineClient->GetLocalPlayer(),
			pResource->IsFakePlayer(n),
			H::Entities.IsFriend(uAccountID),
			H::Entities.InParty(uAccountID),
			H::Entities.IsF2P(uAccountID),
			H::Entities.GetLevel(uAccountID),
			H::Entities.GetParty(uAccountID)
		);
	}
}