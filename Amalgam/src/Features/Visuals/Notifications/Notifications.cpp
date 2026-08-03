#include "Notifications.h"

static inline float EaseInOutCubic(float x)
{
	return x < 0.5f ? 4 * powf(x, 3) : 1 - powf(-2 * x + 2, 3) / 2;
}

void CNotifications::Add(const std::string& sText, Color_t tColor, float flLifeTime, float flPanTime)
{
	m_vNotifications.emplace_back(sText, float(SDK::PlatFloatTime()), flLifeTime, flPanTime, tColor);
	while (m_vNotifications.size() > m_iMaxNotifySize)
		m_vNotifications.pop_front();
}

static inline bool ShouldReverseX()
{
	switch (Vars::Logging::NotificationPosition.Value)
	{
	case Vars::Logging::NotificationPositionEnum::TopLeft:
	case Vars::Logging::NotificationPositionEnum::BottomLeft:
		return false;
	case Vars::Logging::NotificationPositionEnum::TopRight:
	case Vars::Logging::NotificationPositionEnum::BottomRight:
		return true;
	}
	return false;
}

static inline bool ShouldReverseY()
{
	switch (Vars::Logging::NotificationPosition.Value)
	{
	case Vars::Logging::NotificationPositionEnum::TopLeft:
	case Vars::Logging::NotificationPositionEnum::TopRight:
		return false;
	case Vars::Logging::NotificationPositionEnum::BottomLeft:
	case Vars::Logging::NotificationPositionEnum::BottomRight:
		return true;
	}
	return false;
}

void CNotifications::Draw()
{
	for (auto it = m_vNotifications.begin(); it != m_vNotifications.end();)
	{
		if (it->m_flCreateTime + it->m_flLifeTime + it->m_flPanTime < SDK::PlatFloatTime())
			it = m_vNotifications.erase(it);
		else
			++it;
	}
	if (m_vNotifications.empty())
		return;

	const auto& fFont = H::Fonts.GetFont(FONT_NOTIFY);

	// the reference v2 style: plain stacked text lines in the corner, no box / no progress bar
	const bool bReverseX = ShouldReverseX();
	const bool bReverseY = ShouldReverseY();
	const EAlign eAlign = bReverseX ? ALIGN_TOPRIGHT : ALIGN_TOPLEFT;

	const int iLineH = H::Draw.GetTextSize("Q", fFont).y + H::Draw.Scale(4, Scale_Round);
	const int iPad = H::Draw.Scale(8, Scale_Round);
	const int iEdgeX = !bReverseX ? iPad : H::Draw.m_nScreenW - iPad;

	int y = !bReverseY ? iPad : H::Draw.m_nScreenH - iPad - iLineH;

	const float flTime = SDK::PlatFloatTime();
	const Color_t tOutline = Vars::Menu::Theme::Background.Value;

	for (auto& tNotification : m_vNotifications)
	{
		const float flCreate = tNotification.m_flCreateTime;
		const float flLife = tNotification.m_flLifeTime;
		const float flPan = tNotification.m_flPanTime;

		// fade in at spawn, fade out before death
		float flAlpha = 1.f;
		if (flPan > 0.f)
		{
			const float flIn = flTime - flCreate;
			const float flOut = flCreate + flLife + flPan - flTime;
			flAlpha = std::min(flIn, flOut) / flPan;
			flAlpha = EaseInOutCubic(std::clamp(flAlpha, 0.f, 1.f));
		}

		// slight slide-in from the screen edge
		const int iSlide = static_cast<int>((iPad + H::Draw.Scale(6, Scale_Round)) * (1.f - flAlpha));
		const int x = iEdgeX - (!bReverseX ? iSlide : -iSlide);

		const Color_t tText = tNotification.m_tColor.Alpha(static_cast<byte>(255 * flAlpha));
		H::Draw.StringOutlined(fFont, x, y, tText, tOutline.Alpha(static_cast<byte>(255 * flAlpha)), eAlign, tNotification.m_sText.c_str());

		y += iLineH * (!bReverseY ? 1 : -1);
	}
}