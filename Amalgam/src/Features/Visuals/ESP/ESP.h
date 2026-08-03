#pragma once
#include "../../../SDK/SDK.h"

struct Text_t
{
	int m_iMode = ALIGN_TOP;
	std::string m_sText = "";
	Color_t m_tColor = {};
	Color_t m_tOutline = {};
};

struct Bar_t
{
	int m_iMode = ALIGN_TOP;
	float flPercent = 1.f;
	Color_t m_tColor = {};
	Color_t m_tOverfill = {};
	bool m_bAdjust = true;
};

struct EntityCache_t
{
	float m_flAlpha = 1.f;
	std::vector<Text_t> m_vText = {};
	Color_t m_tColor = {};
	bool m_bBox = false;
	Color_t m_tBoxColor = { 255, 255, 255, 255 };
	bool m_bCornerBox = false;
	bool m_bBoxOutline = true;
	float m_flCornerLength = 0.25f;
	int m_iNameMode = ALIGN_TOP; // ALIGN_TOP (above box) or ALIGN_BOTTOM (below box)
};

struct BuildingCache_t : EntityCache_t
{
	std::vector<Bar_t> m_vBars = {};
	float m_flHealth = 1.f;
	bool m_bHealthBarOutline = true;
	bool m_bHealthBarBackground = false;
	Color_t m_tHealthBarBackgroundColor = { 0, 0, 0, 255 };
	float m_flHealthBarWidth = 2.f;
	int m_iHealthBarPosition = 0; // 0 Left, 1 Right, 2 Above, 3 Below
};

struct PlayerCache_t : BuildingCache_t
{
	bool m_bBones = false;
	Color_t m_tBonesColor = { 255, 255, 255, 255 };
	int m_iClassIcon = 0;
	int m_iClassIconPosition = 0; // 0 Above name, 1 Below box, 2 Left, 3 Right
	float m_flClassIconScale = 1.f;
	bool m_bClassIconDistanceScale = false;
	bool m_bSimpleClassIcon = false;
	Color_t m_tSimpleClassIconColor = { 255, 255, 255, 255 };
	bool m_bSimpleClassIconOutline = false;
	CHudTexture* m_pWeaponIcon = nullptr;
	Color_t m_tWeaponIconColor = { 255, 255, 255, 255 };
	bool m_bAvatar = false;
	bool m_bAvatarOutline = false;
	uint32_t m_uAccountID = 0;
	std::vector<Text_t> m_vLabels = {}; // status "flags" labels (Invuln/Invis/Crit/Disguised), drawn with FONT_ESP_LABEL
};

class CESP
{
private:
	void DrawPlayers();
	void DrawBuildings();
	void DrawWorld();
	
	void DrawBox(int x, int y, int w, int h, const EntityCache_t& tCache);
	bool GetDrawBounds(CBaseEntity* pEntity, float& x, float& y, float& w, float& h);
	void DrawBones(CTFPlayer* pPlayer, matrix3x4* aBones, std::vector<int> vecBones, Color_t clr);

	std::unordered_map<CBaseEntity*, PlayerCache_t> m_mPlayerCache = {};
	std::unordered_map<CBaseEntity*, BuildingCache_t> m_mBuildingCache = {};
	std::unordered_map<CBaseEntity*, EntityCache_t> m_mEntityCache = {};

public:
	void Store(CTFPlayer* pLocal);
	void Draw();
};

ADD_FEATURE(CESP, ESP);