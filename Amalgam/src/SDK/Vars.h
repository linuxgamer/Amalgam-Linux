#pragma once
#include "../SDK/Definitions/Types.h"
#include "../Utils/Macros/Macros.h"
#include <windows.h>
#include <unordered_map>
#include <map>
#include <typeinfo>

#define DEFAULT_BIND -1

// forward declartion of ConfigVar
template <class T>
class ConfigVar;

class BaseVar
{
public:
	size_t m_iType;
	std::string m_sName;
	int m_iFlags = 0;

	std::vector<const char*> m_vTitle;
	const char* m_sSection;
	union {
		int i = 0;
		float f;
	} m_unMin;
	union {
		int i = 0;
		float f;
	} m_unMax;
	union {
		int i = 0;
		float f;
	} m_unStep;
	std::vector<const char*> m_vValues = {};
	const char* m_sExtra = nullptr;

	// getter for ConfigVar
	template <class T>
	inline ConfigVar<T>* As()
	{
		if (typeid(T).hash_code() != m_iType)
			return nullptr;

		return reinterpret_cast<ConfigVar<T>*>(this);
	}
};

namespace G
{
	inline std::vector<BaseVar*> Vars = {};
};

template <class T>
class ConfigVar : public BaseVar
{
public:
	T Default;
	T Value;
	std::unordered_map<int, T> Map = {};
	ConfigVar(T tValue, std::string sName, const char* sSection, std::vector<const char*> vTitle, int iFlags = 0, std::vector<const char*> vValues = {}, const char* sNone = nullptr)
	{
		Default = tValue;
		Value = tValue;
		Map[DEFAULT_BIND] = tValue;

		m_iType = typeid(T).hash_code();
		m_sName = sName;
		m_iFlags = iFlags;

		m_vTitle = vTitle;
		m_sSection = sSection;
		m_vValues = vValues;
		m_sExtra = sNone;

		G::Vars.push_back(this);
	}
	ConfigVar(T tValue, std::string sName, const char* sSection, std::vector<const char*> vTitle, int iFlags, int iMin, int iMax, int iStep = 1, const char* sFormat = "%i")
	{
		Default = tValue;
		Value = tValue;
		Map[DEFAULT_BIND] = tValue;

		m_iType = typeid(T).hash_code();
		m_sName = sName;
		m_iFlags = iFlags;

		m_vTitle = vTitle;
		m_sSection = sSection;
		m_unMin.i = iMin;
		m_unMax.i = iMax;
		m_unStep.i = iStep;
		m_sExtra = sFormat;

		G::Vars.push_back(this);
	}
	ConfigVar(T tValue, std::string sName, const char* sSection, std::vector<const char*> vTitle, int iFlags, float flMin, float flMax, float flStep = 1.f, const char* sFormat = "%g")
	{
		Default = tValue;
		Value = tValue;
		Map[DEFAULT_BIND] = tValue;

		m_iType = typeid(T).hash_code();
		m_sName = sName;
		m_iFlags = iFlags;

		m_vTitle = vTitle;
		m_sSection = sSection;
		m_unMin.f = flMin;
		m_unMax.f = flMax;
		m_unStep.f = flStep;
		m_sExtra = sFormat;

		G::Vars.push_back(this);
	}

	inline T& operator[](int i)
	{
		return Map[i];
	}
	inline bool contains(int i) const
	{
		return Map.contains(i);
	}
};

#define NAMESPACE_BEGIN(name, ...)\
	namespace name {\
		inline const char* GetNamespace() { return "Vars::"#name"::"; }\
		inline const char* GetSubname() { return ""; }\
		inline const char* GetSection() { return !std::string(#__VA_ARGS__).empty() ? ""#__VA_ARGS__ : #name; }

#define SUBNAMESPACE_BEGIN(name, ...)\
	namespace name {\
		inline const char* GetSubname() { return #name"::"; }\
		inline const char* GetSection() { return !std::string(#__VA_ARGS__).empty() ? ""#__VA_ARGS__ : #name; }

#define NAMESPACE_END(name)\
	}
#define SUBNAMESPACE_END(name)\
	}

#define CVar(name, title, value, ...)\
	inline ConfigVar<decltype(value)> name = { value, std::format("{}{}{}", GetNamespace(), GetSubname(), #name), GetSection(), { title }, __VA_ARGS__ }
#define CVarValues(name, title, value, flags, none, ...)\
	inline ConfigVar<decltype(value)> name = { value, std::format("{}{}{}", GetNamespace(), GetSubname(), #name), GetSection(), { title }, flags, { __VA_ARGS__ }, none }
#define Enum(name, ...)\
	namespace name##Enum { enum name##Enum { __VA_ARGS__ }; }
#define CVarEnum(name, title, value, flags, none, values, ...)\
	CVarValues(name, title, value, flags, none, values);\
	Enum(name, __VA_ARGS__);

#define NONE 0
#define VISUAL (1 << 31)
#define NOSAVE (1 << 30)
#define NOBIND (1 << 29)
#define DEBUGVAR (1 << 28)

// flags to be automatically used in widgets. keep these as the same values as the flags in components, do not include visual flags
#define SLIDER_CLAMP (1 << 2)
#define SLIDER_MIN (1 << 3)
#define SLIDER_MAX (1 << 4)
#define SLIDER_PRECISION (1 << 5)
#define SLIDER_NOAUTOUPDATE (1 << 6)
#define DROPDOWN_MULTI (1 << 2)
#define DROPDOWN_MODIFIABLE (1 << 3)
#define DROPDOWN_CUSTOM (1 << 2)
#define DROPDOWN_AUTOUPDATE (1 << 3)

namespace Vars
{
	NAMESPACE_BEGIN(Menu)
		// Single source-of-truth label. CheatTag and Visuals::UI::Name are derived from this each
		// frame in the menu (tag = "[name]", name replaces every name) so all three stay consistent.
		CVar(CheatTitle, "Cheat name", std::string("chudhook"), VISUAL | DROPDOWN_AUTOUPDATE);
		CVar(CheatTag, "Cheat tag", std::string("[chudhook]"), VISUAL);
		CVar(PrimaryKey, "Primary key", VK_INSERT, NOBIND);
		CVar(SecondaryKey, "Secondary key", VK_F3, NOBIND);

		CVar(BindWindow, "Bind window", true);
		CVar(BindWindowTitle, "Bind window title", true);
		CVar(MenuShowsBinds, "Menu shows binds", false, NOBIND);
		CVar(Watermark, "Watermark", true, VISUAL);
		CVar(WmShowFPS, "Watermark show fps", true, VISUAL);
		CVar(WmShowTime, "Watermark show time", false, VISUAL);
		CVar(WmShowPing, "Watermark show ping", false, VISUAL);
		CVar(WmShowLoss, "Watermark show loss", false, VISUAL);
		CVar(WmShowDate, "Watermark show date", false, VISUAL);
		CVar(WmBackground, "Watermark background", true, VISUAL);
		CVarEnum(WmPosition, "Watermark position", 2, VISUAL, nullptr,
			VA_LIST("Top left", "Top center", "Top right", "Bottom left", "Bottom center", "Bottom right"),
			WmTopLeft, WmTopCenter, WmTopRight, WmBottomLeft, WmBottomCenter, WmBottomRight);
		CVar(WmShowLogo, "Watermark show logo", false, VISUAL);
		CVar(WmSeparator, "Watermark separator", std::string("|"), VISUAL);
		// Separator style: "Text" uses the WmSeparator string above; every other entry draws an
		// accent-tinted glyph texture (order MUST match g_WmSepGlyphs in wm_sep_svgs.h, offset by 1).
		CVarEnum(WmSepGlyph, "Watermark separator glyph", 0, VISUAL, nullptr,
			VA_LIST("Text", "Slash", "Double slash", "Line", "Double line", "Plus", "Minus", "Loader", "Lens",
				"Italic", "Hash", "Ellipsis", "Chevron right", "Chevron left", "Chevrons right", "Chevrons left",
				"Asterisk", "Activity", "Dot", "Stars", "X", "Claude", "Snowflakes", "Custom"),
			WmSepText, WmSepSlash, WmSepDoubleSlash, WmSepLine, WmSepDoubleLine, WmSepPlus, WmSepMinus, WmSepLoader,
			WmSepLens, WmSepItalic, WmSepHash, WmSepEllipsis, WmSepChevronRight, WmSepChevronLeft, WmSepChevronsRight,
			WmSepChevronsLeft, WmSepAsterisk, WmSepActivity, WmSepDot, WmSepStars, WmSepX, WmSepClaude, WmSepSnowflakes,
			WmSepCustom);
		CVar(WmSepCustomSvg, "Watermark separator custom svg", std::string(""), VISUAL); // paste raw SVG markup; used when glyph == Custom
		// Logo (menu header + optional watermark logo). Style picks a built-in glyph or a pasted custom
		// SVG; order MUST match g_LogoGlyphs in logo_svgs.h, offset by 1 (Chud is the chud_svg.h blob).
		CVarEnum(LogoStyle, "Logo style", 0, VISUAL, nullptr,
			VA_LIST("Chud", "Star of David", "Star", "Heart", "Clarity", "Custom"),
			LogoChud, LogoStarOfDavid, LogoStar, LogoHeart, LogoClarity, LogoCustom);
		CVar(LogoRotateSpeed, "Logo rotation speed", 0.f, VISUAL | SLIDER_PRECISION, 0.f, 360.f, 1.f, "%.0f"); // deg/sec, 0 = static
		CVar(LogoCustomSvg, "Logo custom svg", std::string(""), VISUAL); // paste raw SVG markup; used when style == Custom
		CVar(MediaPlayer, "Media player", true, VISUAL);
		CVar(MediaPlayerBackground, "Media player background##show", true, VISUAL);
		CVarEnum(MediaPlayerMode, "Media player mode", 0, VISUAL, nullptr,
			VA_LIST("Card", "Text strip"),
			Card, TextStrip);
		CVarEnum(MediaPlayerPosition, "Media player position", 2, VISUAL, nullptr,
			VA_LIST("Top left", "Top center", "Top right", "Bottom left", "Bottom center", "Bottom right"),
			TopLeft, TopCenter, TopRight, BottomLeft, BottomCenter, BottomRight);
		CVar(MediaPlayerPosX, VA_LIST("X offset## MediaPlayerPosX", "Media player X offset"), 0.f, VISUAL | SLIDER_PRECISION, -1.f, 1.f, 0.005f, "%.3f");
		CVar(MediaPlayerPosY, VA_LIST("Y offset## MediaPlayerPosY", "Media player Y offset"), 0.f, VISUAL | SLIDER_PRECISION, -1.f, 1.f, 0.005f, "%.3f");
		CVar(MediaPlayerScale, "Media player scale", 1.f, VISUAL | SLIDER_PRECISION, 0.5f, 2.5f, 0.05f, "%.2f");
		CVar(MediaPlayerThumb, "Media player album art", true, VISUAL);
		CVar(MediaPlayerThumbCircle, "Media player circle art", true, VISUAL);
		CVar(MediaPlayerThumbSize, "Media player art size", 30.f, VISUAL | SLIDER_PRECISION, 16.f, 96.f, 1.f, "%.0f");
		CVar(MediaPlayerProgress, "Media player progress bar", true, VISUAL);
		CVar(MediaPlayerRounding, "Media player rounding", 5.f, VISUAL | SLIDER_PRECISION, 0.f, 16.f, 0.5f, "%.1f");
		CVar(MediaPlayerBgColor, "Media player background", Color_t(26, 26, 26, 255), VISUAL);
		CVar(MediaPlayerTitleColor, "Media player title color", Color_t(255, 255, 255, 255), VISUAL);
		CVar(MediaPlayerArtistColor, "Media player artist color", Color_t(160, 160, 160, 255), VISUAL);

		CVarEnum(Indicators, "Indicators", 0b00000, VISUAL | DROPDOWN_MULTI, nullptr,
			VA_LIST("Ticks", "Crit hack", "Spectators", "Ping", "Conditions", "Seed prediction"),
			Ticks = 1 << 0, CritHack = 1 << 1, Spectators = 1 << 2, Ping = 1 << 3, Conditions = 1 << 4, SeedPrediction = 1 << 5);

		CVar(BindsDisplay, "Binds display", DragBox_t(100, 100), VISUAL | NOBIND);
		CVar(TicksDisplay, "Ticks display", DragBox_t(), VISUAL | NOBIND);
		CVar(CritsDisplay, "Crits display", DragBox_t(), VISUAL | NOBIND);
		CVar(SpectatorsDisplay, "Spectators display", DragBox_t(), VISUAL | NOBIND);
		// Spectator list look: Amalgam = the native style, Default/Interwebz reproduce the reference,
		// Minimal (ex-Arbuzebra) / Clarity ported from those refs, Kamidere = rounded avatar panel.
		CVarEnum(SpectatorListStyle, "Spectator list style", 0, VISUAL, nullptr,
			VA_LIST("Amalgam", "Default", "Interwebz", "Minimal", "Clarity", "Kamidere"),
			Amalgam, Default, Interwebz, Minimal, Clarity, Kamidere);
		CVar(SpectatorListBackground, "Spectator list background", true, VISUAL); // interwebz bg toggle
		CVar(SpectatorListTextColor, "Spectator list text color", Color_t(255, 255, 255, 255), VISUAL); // minimal/clarity/kamidere rows
		CVar(SpectatorListTitle, "Spectator list title", std::string("SPECTATORS"), VISUAL); // minimal-style header label
		CVar(SpectatorListScale, "Spectator list scale", 1.f, VISUAL | SLIDER_PRECISION, 0.5f, 2.f, 0.05f, "%.2f"); // scales panel + font
		CVar(PingDisplay, "Ping display", DragBox_t(), VISUAL | NOBIND);
		CVar(ConditionsDisplay, "Conditions display", DragBox_t(), VISUAL | NOBIND);
		CVar(SeedPredictionDisplay, "Seed prediction display", DragBox_t(), VISUAL | NOBIND);
		CVar(RecorderDisplay, "Recorder display", DragBox_t(), VISUAL | NOBIND); // movement recorder HUD box position
		CVar(CheckpointsDisplay, "Checkpoints display", DragBox_t(), VISUAL | NOBIND); // KZ checkpoint list HUD box position

		CVar(Scale, "Scale", 1.f, NOBIND | SLIDER_MIN | SLIDER_PRECISION | SLIDER_NOAUTOUPDATE, 0.75f, 2.f, 0.25f);
		CVar(CheapText, "Cheap text", false);

		SUBNAMESPACE_BEGIN(Theme)
			CVar(Accent, "Accent color", Color_t(200, 25, 25, 255), VISUAL);
			CVar(Background, "Drop down bg color", Color_t(0, 0, 0, 250), VISUAL);
			CVar(Active, "Active color", Color_t(255, 255, 255, 255), VISUAL);
			CVar(Inactive, "Inactive color", Color_t(150, 150, 150, 255), VISUAL);
		SUBNAMESPACE_END(Theme);

		// One shared background style for every overlay panel (main menu, watermark, media player,
		// pixel-surf point boxes, the Minimal spectator list and the Checkpoints window): a solid
		SUBNAMESPACE_BEGIN(Style)
			CVar(Rounding, "Corner rounding", 6.f, VISUAL | SLIDER_PRECISION, 0.f, 24.f, 0.5f, "%.1f");
			CVar(Color, "Background color", Color_t(0, 0, 0, 250), VISUAL);
			// Stacked panel outlines (up to 4), each drawn just outside the previous - port of
			// the OutlineLayer stack. OutlineCount = how many layers are active (0 = off).
			CVar(OutlineCount, "Outline layers", 0, VISUAL | SLIDER_MIN, 0, 4);
			CVar(OutlineColor1, "Outline 1 color", Color_t(0, 0, 0, 255), VISUAL);
			CVar(OutlineThickness1, "Outline 1 thickness", 1.f, VISUAL | SLIDER_PRECISION, 0.5f, 8.f, 0.5f, "%.1f");
			CVar(OutlineBlur1, "Outline 1 blur", 0.f, VISUAL | SLIDER_PRECISION, 0.f, 16.f, 0.5f, "%.1f");
			CVar(OutlineColor2, "Outline 2 color", Color_t(0, 0, 0, 200), VISUAL);
			CVar(OutlineThickness2, "Outline 2 thickness", 1.f, VISUAL | SLIDER_PRECISION, 0.5f, 8.f, 0.5f, "%.1f");
			CVar(OutlineBlur2, "Outline 2 blur", 0.f, VISUAL | SLIDER_PRECISION, 0.f, 16.f, 0.5f, "%.1f");
			CVar(OutlineColor3, "Outline 3 color", Color_t(0, 0, 0, 150), VISUAL);
			CVar(OutlineThickness3, "Outline 3 thickness", 1.f, VISUAL | SLIDER_PRECISION, 0.5f, 8.f, 0.5f, "%.1f");
			CVar(OutlineBlur3, "Outline 3 blur", 0.f, VISUAL | SLIDER_PRECISION, 0.f, 16.f, 0.5f, "%.1f");
			CVar(OutlineColor4, "Outline 4 color", Color_t(0, 0, 0, 100), VISUAL);
			CVar(OutlineThickness4, "Outline 4 thickness", 1.f, VISUAL | SLIDER_PRECISION, 0.5f, 8.f, 0.5f, "%.1f");
			CVar(OutlineBlur4, "Outline 4 blur", 0.f, VISUAL | SLIDER_PRECISION, 0.f, 16.f, 0.5f, "%.1f");
			// Menu/overlay font preset. The first entries bake from embedded faces (see the Fonts/
			// folder + tools/font_to_header.py); Custom uses the typed name in Font below. Order MUST
			CVarEnum(FontPreset, "Menu font", 0, VISUAL, nullptr,
				VA_LIST("Kodchasan", "SF Pro Display", "SN Pro", "Open Sans", "Montserrat", "Custom"),
				Kodchasan, SFProDisplay, SNPro, OpenSans, Montserrat, Custom);
			// Custom font (used when FontPreset == Custom): an installed family name ("Tahoma",
			// "Segoe UI") or a file name/stem from the Fonts folders; blank = falls back to Kodchasan.
			CVar(Font, "Custom font", std::string(""), VISUAL);
			CVar(FontScale, "Font scale", 1.f, VISUAL | SLIDER_PRECISION, 0.5f, 2.f, 0.05f, "%.2f");
			// Off = crisp non-antialiased (monochrome) glyphs. Only the FreeType build configs can
			// disable AA; the stb builds always antialias. See CRender::LoadFonts.
			CVar(FontAntiAlias, "Font anti-aliasing", true, VISUAL);
		SUBNAMESPACE_END(Style);
	NAMESPACE_END(Menu);

	NAMESPACE_BEGIN(Colors)
		CVar(FOVCircle, "FOV circle color", Color_t(255, 255, 255, 100), VISUAL);
		CVar(EdgeBugCircle, "Edge bug circle color", Color_t(255, 255, 255, 255), VISUAL);
		CVar(Local, "Local color", Color_t(255, 255, 255, 0), VISUAL);

		CVar(IndicatorGood, "Indicator good", Color_t(0, 255, 100, 255), NOSAVE | DEBUGVAR);
		CVar(IndicatorMid, "Indicator mid", Color_t(255, 200, 0, 255), NOSAVE | DEBUGVAR);
		CVar(IndicatorBad, "Indicator bad", Color_t(255, 0, 0, 255), NOSAVE | DEBUGVAR);
		CVar(IndicatorMisc, "Indicator misc", Color_t(75, 175, 255, 255), NOSAVE | DEBUGVAR);
		CVar(IndicatorTextGood, "Indicator text good", Color_t(150, 255, 150, 255), NOSAVE | DEBUGVAR);
		CVar(IndicatorTextMid, "Indicator text mid", Color_t(255, 200, 0, 255), NOSAVE | DEBUGVAR);
		CVar(IndicatorTextBad, "Indicator text bad", Color_t(255, 150, 150, 255), NOSAVE | DEBUGVAR);
		CVar(IndicatorTextMisc, "Indicator text misc", Color_t(100, 255, 255, 255), NOSAVE | DEBUGVAR);

		CVar(WorldModulation, VA_LIST("World modulation", "World modulation color"), Color_t(255, 255, 255, 255), VISUAL);
		CVar(SkyModulation, VA_LIST("Sky modulation", "Sky modulation color"), Color_t(255, 255, 255, 255), VISUAL);
		CVar(PropModulation, VA_LIST("Prop modulation", "Prop modulation color"), Color_t(255, 255, 255, 255), VISUAL);
		CVar(ParticleModulation, VA_LIST("Particle modulation", "Particle modulation color"), Color_t(255, 255, 255, 255), VISUAL);
		CVar(FogModulation, VA_LIST("Fog modulation", "Fog modulation color"), Color_t(255, 255, 255, 255), VISUAL);

		CVar(Line, "Line color", Color_t(255, 255, 255, 255), VISUAL);
		CVar(LineIgnoreZ, "Line ignore Z color", Color_t(255, 255, 255, 0), VISUAL);

		CVar(PlayerPath, "Player path color", Color_t(255, 255, 255, 0), VISUAL);
		CVar(PlayerPathIgnoreZ, "Player path ignore Z color", Color_t(255, 255, 255, 255), VISUAL);
		CVar(ProjectilePath, "Projectile path color", Color_t(255, 255, 255, 0), VISUAL);
		CVar(ProjectilePathIgnoreZ, "Projectile path ignore Z color", Color_t(255, 255, 255, 255), VISUAL);
		CVar(TrajectoryPath, "Trajectory path color", Color_t(255, 255, 255, 0), VISUAL);
		CVar(TrajectoryPathIgnoreZ, "Trajectory path ignore Z color", Color_t(255, 255, 255, 255), VISUAL);
		CVar(ShotPath, "Shot path color", Color_t(255, 255, 255, 0), VISUAL);
		CVar(ShotPathIgnoreZ, "Shot path ignore Z color", Color_t(255, 255, 255, 255), VISUAL);
		CVar(SplashRadius, "Splash radius color", Color_t(255, 255, 255, 0), VISUAL);
		CVar(SplashRadiusIgnoreZ, "Splash radius ignore Z color", Color_t(255, 255, 255, 255), VISUAL);
		CVar(RealPath, "Real path color", Color_t(255, 255, 255, 0), NOSAVE | DEBUGVAR);
		CVar(RealPathIgnoreZ, "Real path ignore Z color", Color_t(255, 255, 255, 255), NOSAVE | DEBUGVAR);

		CVar(BoneHitboxEdge, "Bone hitbox edge color", Color_t(255, 255, 255, 255), VISUAL);
		CVar(BoneHitboxEdgeIgnoreZ, "Bone hitbox edge ignore Z color", Color_t(255, 255, 255, 0), VISUAL);
		CVar(BoneHitboxFace, "Bone hitbox face color", Color_t(255, 255, 255, 0), VISUAL);
		CVar(BoneHitboxFaceIgnoreZ, "Bone hitbox face ignore Z color", Color_t(255, 255, 255, 0), VISUAL);
		CVar(TargetHitboxEdge, "Target hitbox edge color", Color_t(255, 150, 150, 255), VISUAL);
		CVar(TargetHitboxEdgeIgnoreZ, "Target hitbox edge ignore Z color", Color_t(255, 150, 150, 0), VISUAL);
		CVar(TargetHitboxFace, "Target hitbox face color", Color_t(255, 150, 150, 0), VISUAL);
		CVar(TargetHitboxFaceIgnoreZ, "Target hitbox face ignore Z color", Color_t(255, 150, 150, 0), VISUAL);
		CVar(BoundHitboxEdge, "Bound hitbox edge color", Color_t(255, 255, 255, 255), VISUAL);
		CVar(BoundHitboxEdgeIgnoreZ, "Bound hitbox edge ignore Z color", Color_t(255, 255, 255, 0), VISUAL);
		CVar(BoundHitboxFace, "Bound hitbox face color", Color_t(255, 255, 255, 0), VISUAL);
		CVar(BoundHitboxFaceIgnoreZ, "Bound hitbox face ignore Z color", Color_t(255, 255, 255, 0), VISUAL);

		CVar(SpellFootstep, "Spell footstep color", Color_t(255, 255, 255, 255), VISUAL);
		CVar(WeaponSheen, "Weapon sheen color", Color_t(0, 200, 255, 255), VISUAL);
		CVar(Crosshair, "Crosshair color", Color_t(0, 255, 0, 255), VISUAL);
		CVar(CrosshairOutline, "Crosshair outline color", Color_t(0, 0, 0, 255), VISUAL);
		CVar(CrosshairGradCenter, "Crosshair gradient center color", Color_t(0, 255, 0, 255), VISUAL);
		CVar(CrosshairGradOuter, "Crosshair gradient outer color", Color_t(0, 255, 0, 0), VISUAL);
		CVar(CrosshairDot, "Crosshair dot color", Color_t(0, 255, 0, 255), VISUAL);
	NAMESPACE_END(Colors);

	NAMESPACE_BEGIN(Aimbot)
		SUBNAMESPACE_BEGIN(General, Aimbot)
			CVarEnum(AimType, "Aim type", 0, NONE, nullptr,
				VA_LIST("Off", "Plain", "Smooth", "Silent", "Locking", "Assistive"),
				Off, Plain, Smooth, Silent, Locking, Assistive);
			CVarEnum(TargetSelection, "Target selection", 0, NONE, nullptr,
				VA_LIST("FOV", "Distance"),
				FOV, Distance);
			CVarEnum(Target, "Target", 0b0000001, DROPDOWN_MULTI, nullptr,
				VA_LIST("Players", "Sentries", "Dispensers", "Teleporters", "Stickies", "NPCs", "Bombs"),
				Players = 1 << 0, Sentry = 1 << 1, Dispenser = 1 << 2, Teleporter = 1 << 3, Stickies = 1 << 4, NPCs = 1 << 5, Bombs = 1 << 6,
				Building = Sentry | Dispenser | Teleporter);
			CVarEnum(Ignore, "Ignore", 0b00000001000, DROPDOWN_MULTI, nullptr,
				VA_LIST("Friends", "Party", "Unprioritized", "Invulnerable", "Invisible", "Unsimulated", "Dead ringer", "Vaccinator", "Disguised", "Taunting", "Team"),
				Friends = 1 << 0, Party = 1 << 1, Unprioritized = 1 << 2, Invulnerable = 1 << 3, Invisible = 1 << 4, Unsimulated = 1 << 5, DeadRinger = 1 << 6, Vaccinator = 1 << 7, Disguised = 1 << 8, Taunting = 1 << 9, Team = 1 << 10);
			CVar(AimFOV, "Aim FOV", 30.f, SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 180.f);
			CVar(MaxTargets, "Max targets", 2, SLIDER_MIN, 1, 6);
			CVar(IgnoreInvisible, "Ignore invisible", 50.f, SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 100.f, 10.f, "%g%%");
			CVar(AssistStrength, "Assist strength", 25.f, SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 100.f, 1.f, "%g%%");
			CVar(TickTolerance, "Tick tolerance", 4, SLIDER_CLAMP, 0, 21);
			CVar(AutoShoot, "Auto shoot", false);
			CVar(FOVCircle, "FOV Circle", true, VISUAL);
			CVar(NoSpread, "No spread", false);

			CVarEnum(AimHoldsFire, "Aim holds fire", 2, NOSAVE | DEBUGVAR, nullptr,
				VA_LIST("False", "Minigun only", "Always"),
				False, MinigunOnly, Always);
			CVar(NoSpreadOffset, "No spread offset", 0.f, NOSAVE | DEBUGVAR | SLIDER_PRECISION, -1.f, 1.f, 0.1f);
			CVar(NoSpreadAverage, "No spread average", 5, NOSAVE | DEBUGVAR | SLIDER_MIN, 1, 25);
			CVar(NoSpreadInterval, "No spread interval", 0.1f, NOSAVE | DEBUGVAR | SLIDER_MIN, 0.05f, 5.f, 0.1f, "%gs");
			CVar(NoSpreadBackupInterval, "No spread backup interval", 2.f, NOSAVE | DEBUGVAR | SLIDER_MIN, 2.f, 10.f, 0.1f, "%gs");
		SUBNAMESPACE_END(Global);

		SUBNAMESPACE_BEGIN(Hitscan)
			CVarEnum(Hitboxes, VA_LIST("Hitboxes", "Hitscan hitboxes"), 0b000111, DROPDOWN_MULTI, nullptr,
				VA_LIST("Head", "Body", "Pelvis", "Arms", "Legs", "##Divider", "Bodyaim if lethal", "Headshot only"),
				Head = 1 << 0, Body = 1 << 1, Pelvis = 1 << 2, Arms = 1 << 3, Legs = 1 << 4, BodyaimIfLethal = 1 << 5, HeadshotOnly = 1 << 6);
			CVarValues(MultipointHitboxes, "Multipoint hitboxes", 0b00000, DROPDOWN_MULTI, "All",
				VA_LIST("Head", "Body", "Pelvis", "Arms", "Legs"));
			CVarEnum(Modifiers, VA_LIST("Modifiers", "Hitscan modifiers"), 0b0100000, DROPDOWN_MULTI, nullptr,
				VA_LIST("Tapfire", "Wait for headshot", "Wait for charge", "Scoped only", "Auto scope", "Auto rev minigun", "Extinguish team"),
				Tapfire = 1 << 0, WaitForHeadshot = 1 << 1, WaitForCharge = 1 << 2, ScopedOnly = 1 << 3, AutoScope = 1 << 4, AutoRev = 1 << 5, ExtinguishTeam = 1 << 6);
			CVar(MultipointScale, "Multipoint scale", 0.f, SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 100.f, 5.f, "%g%%");
			CVar(TapfireDistance, "Tapfire distance", 1000.f, SLIDER_MIN | SLIDER_PRECISION, 250.f, 1000.f, 50.f);

			CVarEnum(PeekCheck, "Peek check", 1, NOSAVE | DEBUGVAR, nullptr,
				VA_LIST("Off", "Doubletap only", "Always"),
				Off, DoubletapOnly, Always);
			CVar(PeekAmount, "Peek amount", 1, NOSAVE | DEBUGVAR, 0, 5);
			CVar(BoneSizeSubtract, "Bone size subtract", 1.f, NOSAVE | DEBUGVAR | SLIDER_MIN, 0.f, 4.f, 0.25f);
			CVar(BoneSizeMinimumScale, "Bone size minimum scale", 1.f, NOSAVE | DEBUGVAR | SLIDER_CLAMP, 0.f, 1.f, 0.1f);
		SUBNAMESPACE_END(HITSCAN);

		SUBNAMESPACE_BEGIN(Projectile)
			CVarEnum(StrafePrediction, VA_LIST("Predict", "Strafe prediction"), 0b11, DROPDOWN_MULTI, "Off",
				VA_LIST("Air strafing", "Ground strafing"),
				Air = 1 << 0, Ground = 1 << 1);
			CVarEnum(SplashPrediction, VA_LIST("Splash", "Splash prediction"), 0, NONE, nullptr,
				VA_LIST("Off", "Include", "Prefer", "Only"),
				Off, Include, Prefer, Only);
			CVarEnum(AutoDetonate, "Auto detonate", 0b00, DROPDOWN_MULTI, "Off",
				VA_LIST("Stickies", "Flares", "##Divider", "Prevent self damage", "Ignore invisible"),
				Stickies = 1 << 0, Flares = 1 << 1, PreventSelfDamage = 1 << 2, IgnoreInvisible = 1 << 3);
			CVarEnum(AutoAirblast, "Auto airblast", 0b000, DROPDOWN_MULTI, "Off", // todo: implement advanced redirect!!
				VA_LIST("Enabled", "##Divider", "Redirect", "Ignore FOV"),
				Enabled = 1 << 0, Redirect = 1 << 1, IgnoreFOV = 1 << 2);
			CVarEnum(Hitboxes, VA_LIST("Hitboxes", "Projectile hitboxes"), 0b001111, DROPDOWN_MULTI, nullptr,
				VA_LIST("Auto", "##Divider", "Head", "Body", "Feet", "##Divider", "Bodyaim if lethal", "Prioritize feet"),
				Auto = 1 << 0, Head = 1 << 1, Body = 1 << 2, Feet = 1 << 3, BodyaimIfLethal = 1 << 4, PrioritizeFeet = 1 << 5);
			CVarEnum(Modifiers, VA_LIST("Modifiers", "Projectile modifiers"), 0b1010, DROPDOWN_MULTI, nullptr,
				VA_LIST("Charge shot", "Cancel charge", "Use prime time"),
				ChargeWeapon = 1 << 0, CancelCharge = 1 << 1, UsePrimeTime = 1 << 2);
			CVar(MaxSimulationTime, "Max simulation time", 2.f, SLIDER_MIN | SLIDER_PRECISION, 0.1f, 2.5f, 0.25f, "%gs");
			CVar(HitChance, "Hit chance", 0.f, SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 100.f, 10.f, "%g%%");
			CVar(AutodetRadius, "Autodet radius", 90.f, SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 100.f, 10.f, "%g%%");
			CVar(SplashRadius, "Splash radius", 90.f, SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 100.f, 10.f, "%g%%");
			CVar(AutoRelease, "Auto release", 0.f, SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 100.f, 5.f, "%g%%");

			CVar(GroundSamples, "Samples", 33, NOSAVE | DEBUGVAR, 3, 66);
			CVar(GroundStraightFuzzyValue, "Straight fuzzy value", 100.f, NOSAVE | DEBUGVAR | SLIDER_PRECISION, 0.f, 500.f, 25.f);
			CVar(GroundLowMinimumSamples, "Low min samples", 16, NOSAVE | DEBUGVAR, 3, 66);
			CVar(GroundHighMinimumSamples, "High min samples", 33, NOSAVE | DEBUGVAR, 3, 66);
			CVar(GroundLowMinimumDistance, "Low min distance", 0.f, NOSAVE | DEBUGVAR | SLIDER_MIN | SLIDER_PRECISION, 0.f, 2500.f, 100.f);
			CVar(GroundHighMinimumDistance, "High min distance", 1000.f, NOSAVE | DEBUGVAR | SLIDER_MIN | SLIDER_PRECISION, 0.f, 2500.f, 100.f);
			CVar(GroundMaxChanges, "Max changes", 0, NOSAVE | DEBUGVAR | SLIDER_MIN | SLIDER_PRECISION, 0, 5);
			CVar(GroundMaxChangeTime, "Max change time", 0, NOSAVE | DEBUGVAR | SLIDER_MIN | SLIDER_PRECISION, 0, 66);

			CVar(AirSamples, "Samples", 33, NOSAVE | DEBUGVAR, 3, 66);
			CVar(AirStraightFuzzyValue, "Straight fuzzy value", 0.f, NOSAVE | DEBUGVAR | SLIDER_PRECISION, 0.f, 500.f, 25.f);
			CVar(AirLowMinimumSamples, "Low min samples", 16, NOSAVE | DEBUGVAR, 3, 66);
			CVar(AirHighMinimumSamples, "High min samples", 16, NOSAVE | DEBUGVAR, 3, 66);
			CVar(AirLowMinimumDistance, "Low min distance", 100000.f, NOSAVE | DEBUGVAR | SLIDER_MIN | SLIDER_PRECISION, 0.f, 2500.f, 100.f);
			CVar(AirHighMinimumDistance, "High min distance", 100000.f, NOSAVE | DEBUGVAR | SLIDER_MIN | SLIDER_PRECISION, 0.f, 2500.f, 100.f);
			CVar(AirMaxChanges, "Max changes", 2, NOSAVE | DEBUGVAR | SLIDER_MIN | SLIDER_PRECISION, 0, 5);
			CVar(AirMaxChangeTime, "Max change time", 16, NOSAVE | DEBUGVAR | SLIDER_MIN | SLIDER_PRECISION, 0, 66);

			CVar(VelocityAverageCount, "Velocity average count", 5, NOSAVE | DEBUGVAR, 1, 10);
			CVar(VerticalShift, "Vertical shift", 5.f, NOSAVE | DEBUGVAR | SLIDER_MIN | SLIDER_PRECISION, 0.f, 10.f, 0.5f);
			CVar(DragOverride, "Drag override", 0.f, NOSAVE | DEBUGVAR | SLIDER_MIN | SLIDER_PRECISION, 0.f, 1.f, 0.01f);
			CVar(TimeOverride, "Time override", 0.f, NOSAVE | DEBUGVAR | SLIDER_MIN | SLIDER_PRECISION, 0.f, 2.f, 0.01f);
			CVar(HuntsmanLerp, "Huntsman lerp", 50.f, NOSAVE | DEBUGVAR | SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 100.f, 1.f, "%g%%");
			CVar(HuntsmanLerpLow, "Huntsman lerp low", 100.f, NOSAVE | DEBUGVAR | SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 100.f, 1.f, "%g%%");
			CVar(HuntsmanAdd, "Huntsman add", 0.f, NOSAVE | DEBUGVAR | SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 20.f);
			CVar(HuntsmanAddLow, "Huntsman add low", 0.f, NOSAVE | DEBUGVAR | SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 20.f);
			CVar(HuntsmanClamp, "Huntsman clamp", 5.f, NOSAVE | DEBUGVAR | SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 10.f, 0.5f);
			CVar(HuntsmanPullPoint, "Huntsman pull point", false, NOSAVE | DEBUGVAR);

			CVar(SplashPointsDirect, "Direct splash points", 100, NOSAVE | DEBUGVAR | SLIDER_MIN, 0, 400, 5);
			CVar(SplashPointsArc, "Arc splash points", 100, NOSAVE | DEBUGVAR | SLIDER_MIN, 0, 400, 5);
			CVar(SplashCountDirect, "Direct splash count", 100, NOSAVE | DEBUGVAR | SLIDER_MIN, 1, 100);
			CVar(SplashCountArc, "Arc splash count", 5, NOSAVE | DEBUGVAR | SLIDER_MIN, 1, 100);
			CVar(SplashRotateX, "Splash Rx", -1.f, NOSAVE | DEBUGVAR | SLIDER_MIN | SLIDER_PRECISION, -1.f, 360.f);
			CVar(SplashRotateY, "Splash Ry", -1.f, NOSAVE | DEBUGVAR | SLIDER_MIN | SLIDER_PRECISION, -1.f, 360.f);
			CVar(SplashTraceInterval, "Splash trace interval", 10, NOSAVE | DEBUGVAR, 1, 10);
			CVar(SplashNormalSkip, "Splash normal skip", 1, NOSAVE | DEBUGVAR | SLIDER_MIN, 1, 10);
			CVarEnum(SplashMode, "Splash mode", 0, NOSAVE | DEBUGVAR, nullptr,
				VA_LIST("Multi", "Single"),
				Multi, Single);
			CVarEnum(RocketSplashMode, "Rocket splash mode", 0, NOSAVE | DEBUGVAR, nullptr,
				VA_LIST("Regular", "Special light", "Special heavy"),
				Regular, SpecialLight, SpecialHeavy);
			CVar(SplashGrates, "Splash grates", true, NOSAVE | DEBUGVAR);

			CVar(DeltaCount, "Delta count", 5, NOSAVE | DEBUGVAR, 1, 5);
			CVarEnum(DeltaMode, "Delta mode", 0, NOSAVE | DEBUGVAR, nullptr,
				VA_LIST("Average", "Max"),
				Average, Max);
			CVarEnum(MovesimFrictionFlags, "Movesim friction flags", 0b01, NOSAVE | DEBUGVAR | DROPDOWN_MULTI, nullptr,
				VA_LIST("Run reduce", "Calculate increase"),
				RunReduce = 1 << 0, CalculateIncrease = 1 << 1);
		SUBNAMESPACE_END(Projectile);

		SUBNAMESPACE_BEGIN(Melee)
			CVar(AutoBackstab, "Auto backstab", false);
			// Backstab gets its own FOV + aim type so the knife can be tuned independently of the
			// general aimbot (values mirror General::AimType so they feed the same Aim() methods).
			CVarEnum(BackstabAimType, "Backstab aim type", 0, NONE, nullptr,
				VA_LIST("Silent", "Legit"),
				Silent, Legit);
			CVar(BackstabFOV, "Backstab FOV", 180.f, SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 180.f);
			CVar(BackstabAutoShoot, "Backstab auto shoot", false);
			// Backstab-specific copy of the General aim Ignore dropdown so the knife can skip
			// invulnerable / invisible / dead-ringer / vaccinator-shielded / disguised / taunting
			CVarEnum(BackstabIgnore, "Backstab ignore", 0b100000001000, DROPDOWN_MULTI, nullptr,
				VA_LIST("Friends", "Party", "Unprioritized", "Invulnerable", "Invisible", "Unsimulated", "Dead ringer", "Vaccinator", "Disguised", "Taunting", "Team", "Razorback"),
				Friends = 1 << 0, Party = 1 << 1, Unprioritized = 1 << 2, Invulnerable = 1 << 3, Invisible = 1 << 4, Unsimulated = 1 << 5, DeadRinger = 1 << 6, Vaccinator = 1 << 7, Disguised = 1 << 8, Taunting = 1 << 9, Team = 1 << 10, Razorback = 1 << 11);
			CVar(SwingPrediction, "Swing prediction", false);
			CVar(WhipTeam, "Whip team", false);

			CVar(SwingOffset, "Swing offset", -1, NOSAVE | DEBUGVAR, -1, 1);
			CVar(SwingPredictLag, "Swing predict lag", true, NOSAVE | DEBUGVAR);
			// Exposed in the KNIFE subtab so backstab false positives/negatives can be tuned without testing builds.
			CVar(BackstabAccountPing, "Backstab account ping", false);
			CVar(BackstabDoubleTest, "Backstab double test", false);
			// Remove client interpolation from the target when checking a backstab (no-backtrack path) so it
			// is judged against the enemy's real-time position instead of the ~interp-delayed rendered one.
			CVar(BackstabRemoveInterp, "Backstab remove interp", false);
			// Only auto-swing once the direct (non-hull) trace connects. The 18u swing hull otherwise lets the
			// auto-backstab fire a touch too far out while chasing, so the knife whiffs into the air. With this
			CVar(BackstabStrictRange, "Backstab strict range", false);
			// Predict ONE tick of local movement before judging the swing. The server runs ProcessMovement
			CVar(BackstabPredictMovement, "Backstab predict movement", false);
			// Fire whenever the back is exposed, even when our own swing trace doesn't confirm. Our trace
			CVar(BackstabAggressive, "Backstab aggressive", false);
			// Finer aim-point search (ref the reference melee multi-point sweep). Normally the knife tests two points
			// on the target box - the centre axis (best owner-facing dot) and the nearest reachable point.
			CVar(BackstabMultiPoint, "Backstab multi point", false);
			// Rear-arc yaw extrapolation (the principled side-stab fix). The target's networked yaw is always a
			// little stale, and the server lag-comps them to somewhere between their current yaw and where they
			CVar(BackstabExtrapolateYaw, "Backstab extrapolate yaw", false);
			// Bad-connection backstab help: high ping / loss leaves the target's networked yaw + position
			// stale, so a normally-valid backstab gets dropped.
			CVar(BackstabUnstableConnection, "Backstab unstable connection", false);
			// Ceiling (degrees) on how far unstable-connection mode may widen the rear arc, so a fast-spinning
			// target under heavy loss can't open the behind check all the way round to a facestab. Tunable
			CVar(BackstabUnstableMaxAngle, "Backstab unstable max angle", 25.f, SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 60.f);
		SUBNAMESPACE_END(Melee);

		SUBNAMESPACE_BEGIN(Healing)
			CVarEnum(HealPriority, "Heal Priority", 0, NONE, nullptr,
				VA_LIST("None", "Prioritize team", "Prioritize friends", "Friends only"),
				None, PrioritizeTeam, PrioritizeFriends, FriendsOnly);
			CVar(AutoHeal, "Auto heal", false);
			CVar(AutoArrow, "Auto arrow", false);
			CVar(AutoRepair, "Auto repair", false);
			CVar(AutoSandvich, "Auto sandvich", false);
			CVar(AutoVaccinator, "Auto vaccinator", false);
			CVar(ActivateOnVoice, "Activate on voice", false);

			CVar(AutoVaccinatorBulletScale, "Auto vaccinator bullet scale", 100.f, NOSAVE | DEBUGVAR | SLIDER_MIN | SLIDER_PRECISION, 0.f, 200.f, 10.f, "%g%%");
			CVar(AutoVaccinatorBlastScale, "Auto vaccinator blast scale", 100.f, NOSAVE | DEBUGVAR | SLIDER_MIN | SLIDER_PRECISION, 0.f, 200.f, 10.f, "%g%%");
			CVar(AutoVaccinatorFireScale, "Auto vaccinator fire scale", 100.f, NOSAVE | DEBUGVAR | SLIDER_MIN | SLIDER_PRECISION, 0.f, 200.f, 10.f, "%g%%");
			CVar(AutoVaccinatorFlamethrowerDamageOnly, "Auto vaccinator flamethrower damage only", false, NOSAVE | DEBUGVAR);
		SUBNAMESPACE_END(Healing);
	NAMESPACE_END(Aimbot);
	
	NAMESPACE_BEGIN(CritHack, Crit Hack)
		CVar(ForceCrits, "Force crits", false);
		CVar(AvoidRandomCrits, "Avoid random crits", false);
		CVar(AlwaysMeleeCrit, "Always melee crit", false);
	NAMESPACE_END(CritHack);

	NAMESPACE_BEGIN(Backtrack)
		CVar(Latency, "Fake latency", 0, SLIDER_CLAMP, 0, 1000, 5);
		CVar(Interp, "Fake interp", 0, SLIDER_CLAMP | SLIDER_PRECISION, 0, 1000, 5);
		CVar(Window, VA_LIST("Window", "Backtrack window"), 0, SLIDER_CLAMP | SLIDER_PRECISION, 0, 200, 5);
		CVar(PreferOnShot, "Prefer on shot", false);

		CVar(Offset, "Offset", 0, NOSAVE | DEBUGVAR, -1, 1);
	NAMESPACE_END(Backtrack);

	NAMESPACE_BEGIN(Doubletap)
		CVar(Doubletap, "Doubletap", false);
		CVar(Warp, "Warp", false);
		CVar(RechargeTicks, "Recharge ticks", false);
		CVar(AntiWarp, "Anti-warp", false);
		CVar(TickLimit, "Tick limit", 22, SLIDER_CLAMP, 2, 22);
		CVar(WarpRate, "Warp rate", 22, SLIDER_CLAMP, 2, 22);
		CVar(RechargeLimit, "Recharge limit", 24, SLIDER_MIN, 1, 24);
		CVar(PassiveRecharge, "Passive recharge", 0, SLIDER_CLAMP, 0, 67);
	NAMESPACE_END(DoubleTap)

	NAMESPACE_BEGIN(Fakelag)
		CVarEnum(Fakelag, "Fakelag", 0, NONE, nullptr,
			VA_LIST("Off", "Plain", "Random", "Adaptive"),
			Off, Plain, Random, Adaptive);
		CVarEnum(Options, VA_LIST("Options", "Fakelag options"), 0b000, DROPDOWN_MULTI, nullptr,
			VA_LIST("Only moving", "On unduck", "Not airborne"),
			OnlyMoving = 1 << 0, OnUnduck = 1 << 1, NotAirborne = 1 << 2);
		CVar(PlainTicks, "Plain ticks", 12, SLIDER_CLAMP, 1, 22);
		CVar(RandomTicks, "Random ticks", IntRange_t(14, 18), SLIDER_CLAMP, 1, 22, 1, "%i - %i");
		CVar(UnchokeOnAttack, "Unchoke on attack", false);
		CVar(RetainBlastJump, "Retain blastjump", false);

		CVar(RetainSoldierOnly, "Retain blastjump soldier only", true, NOSAVE | DEBUGVAR);
	NAMESPACE_END(FakeLag);

	NAMESPACE_BEGIN(AutoPeek, Auto Peek)
		CVar(Enabled, VA_LIST("Enabled", "Auto peek"), false);
	NAMESPACE_END(AutoPeek);

	NAMESPACE_BEGIN(Speedhack)
		CVar(Enabled, VA_LIST("Enabled", "Speedhack enabled"), false);
		CVar(Amount, VA_LIST("Amount", "SpeedHack amount"), 1, NONE, 1, 50);
	NAMESPACE_END(Speedhack);

	NAMESPACE_BEGIN(AntiAim, Antiaim)
		CVar(Enabled, VA_LIST("Enabled", "Antiaim enabled"), false);
		CVarEnum(PitchReal, "Real pitch", 0, NONE, nullptr,
			VA_LIST("None", "Up", "Down", "Zero", "Jitter", "Reverse jitter"),
			None, Up, Down, Zero, Jitter, ReverseJitter);
		CVarEnum(PitchFake, "Fake pitch", 0, NONE, nullptr,
			VA_LIST("None", "Up", "Down", "Jitter", "Reverse jitter"),
			None, Up, Down, Jitter, ReverseJitter);
		Enum(Yaw, Forward, Left, Right, Backwards, Edge, Jitter, Spin);
		CVarValues(YawReal, "Real yaw", 0, NONE, nullptr,
			"Forward", "Left", "Right", "Backwards", "Edge", "Jitter", "Spin");
		CVarValues(YawFake, "Fake yaw", 0, NONE, nullptr,
			"Forward", "Left", "Right", "Backwards", "Edge", "Jitter", "Spin");
		Enum(YawMode, View, Target);
		CVarValues(RealYawBase, "Real base", 0, NONE, nullptr,
			"View", "Target");
		CVarValues(FakeYawBase, "Fake base", 0, NONE, nullptr,
			"View", "Target");
		CVar(RealYawOffset, "Real offset", 0.f, SLIDER_CLAMP | SLIDER_PRECISION, -180.f, 180.f, 5.f);
		CVar(FakeYawOffset, "Fake offset", 0.f, SLIDER_CLAMP | SLIDER_PRECISION, -180.f, 180.f, 5.f);
		CVar(RealYawValue, "Real value", 90.f, SLIDER_CLAMP | SLIDER_PRECISION, -180.f, 180.f, 5.f);
		CVar(FakeYawValue, "Fake value", -90.f, SLIDER_CLAMP | SLIDER_PRECISION, -180.f, 180.f, 5.f);
		CVar(SpinSpeed, "Spin speed", 15.f, SLIDER_PRECISION, -30.f, 30.f);
		CVar(MinWalk, "Minwalk", false);
		CVar(AntiOverlap, "Anti-overlap", false);
		CVar(InvalidShootPitch, "Hide pitch on shot", false);
	NAMESPACE_END(AntiAim);

	NAMESPACE_BEGIN(Resolver)
		CVar(Enabled, VA_LIST("Enabled", "Resolver enabled"), false);
		CVar(AutoResolve, "Auto resolve", false);
		CVar(AutoResolveCheatersOnly, "Auto resolve cheaters only", false);
		CVar(AutoResolveHeadshotOnly, "Auto resolve headshot only", false);
		CVar(AutoResolveYawAmount, "Auto resolve yaw", 90.f, SLIDER_CLAMP | SLIDER_PRECISION, -180.f, 180.f, 45.f);
		CVar(AutoResolvePitchAmount, "Auto resolve pitch", 90.f, SLIDER_CLAMP, -180.f, 180.f, 90.f);
		CVar(CycleYaw, "Cycle yaw", 0.f, SLIDER_CLAMP | SLIDER_PRECISION, -180.f, 180.f, 45.f);
		CVar(CyclePitch, "Cycle pitch", 0.f, SLIDER_CLAMP, -180.f, 180.f, 90.f);
		CVar(CycleView, "Cycle view", false);
		CVar(CycleMinwalk, "Cycle minwalk", false);
	NAMESPACE_END(Resolver);

	NAMESPACE_BEGIN(CheaterDetection, Cheater Detection)
		CVarEnum(Methods, "Detection methods", 0b0000, DROPDOWN_MULTI, nullptr,
			VA_LIST("Invalid pitch", "Packet choking", "Aim flicking", "Duck Speed"),
			InvalidPitch = 1 << 0, PacketChoking = 1 << 1, AimFlicking = 1 << 2, DuckSpeed = 1 << 3);
		CVar(DetectionsRequired, "Detections required", 10, SLIDER_MIN, 0, 50);
		CVar(MinimumChoking, "Minimum choking", 20, SLIDER_MIN, 4, 22);
		CVar(MinimumFlick, "Minimum flick angle", 20.f, SLIDER_PRECISION, 10.f, 30.f); // min flick size to suspect
		CVar(MaximumNoise, "Maximum flick noise", 1.f, SLIDER_PRECISION, 1.f, 10.f); // max difference between angles before and after flick
	NAMESPACE_END(CheaterDetection);

	NAMESPACE_BEGIN(ESP)
		CVarValues(ActiveGroups, "Active groups", int(0b11111111111111111111111111111111), VISUAL | DROPDOWN_MULTI, nullptr);
		// - health bar tweaks (apply to every group's ESP health bar).
		CVar(HealthBarGroupColor, "Health bar group color", false, VISUAL);
		CVar(HealthBarWidth, "Health bar width", 2.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 1.f, 8.f, 0.5f);
		CVar(HealthBarOutline, "Health bar outline", true, VISUAL);
		CVar(HealthBarBackground, "Health bar background", false, VISUAL);
		// Box style (global; applies to every group that draws a box).
		CVar(CornerBox, "Corner box", false, VISUAL);
		CVar(BoxOutline, "Box outline", true, VISUAL);
		CVar(CornerLength, "Corner length", 0.25f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 0.1f, 0.5f, 0.05f, "%.2f");
		// Text styling (global; applies to every group's name/labels/text — they all share FONT_ESP).
		// Family = GDI font name (e.g. "Verdana", "Arial", "Tahoma"). Empty = default (Verdana).
		CVar(FontFamily, "ESP font family", std::string(""), VISUAL | DROPDOWN_AUTOUPDATE);
		CVar(FontSize, "ESP font size", 0.f, VISUAL | SLIDER_PRECISION, 0.f, 64.f, 1.f, "%.0f");
		// Drop antialias on the ESP font (crisp pixel edges, like the menu font).
		CVar(FontNoAntialias, "ESP font no antialias", false, VISUAL);
		// Backing behind name/labels/text: None (flat), Shadow (single offset drop), Outline (full surround).
		// Default Outline preserves the original look.
		CVarEnum(TextStyle, "ESP text style", 2, VISUAL, nullptr,
			VA_LIST("None", "Shadow", "Outline"),
			TextNone, TextShadow, TextOutline);
		// "Flags" status labels (repurposed from the old player-tag labels). The per-section ESP "Flags"
		// toggle (ESPEnum::Labels bit) turns these on; these globals pick WHICH effects show, their size
		CVarEnum(FlagEffects, "Flags", 0b1111, VISUAL | DROPDOWN_MULTI, nullptr,
			VA_LIST("Invuln", "Invis", "Crit", "Disguised"),
			Invuln = 1 << 0, Invis = 1 << 1, Crit = 1 << 2, Disguised = 1 << 3);
		CVar(FlagScale, "Flags scale", 1.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 0.5f, 3.f, 0.1f, "%.1f");
		CVarEnum(FlagPosition, "Flags position", 0, VISUAL, nullptr,
			VA_LIST("Right", "Left", "Above", "Below"),
			Right, Left, Above, Below);
		// Skeleton (bones ESP) line thickness in pixels.
		CVar(SkeletonThickness, "Skeleton thickness", 1.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 1.f, 10.f, 0.5f, "%.1f");
	NAMESPACE_END(ESP);

	NAMESPACE_BEGIN(Visuals)
		SUBNAMESPACE_BEGIN(UI)
			// the reference player name changer (exact port of names.cpp). Two parts:
			// (1) OTHER players: mask everyone else's displayed name.
			CVarEnum(StreamerMode, "Change other player names", 0, VISUAL, nullptr,
				VA_LIST("Off", "Class", "Team", "Everyone"),
				Off, Class, TeamRole, Everyone);
			CVar(StreamerTeamLabel, "Teammate label", std::string("teammate"), VISUAL);
			CVar(StreamerEnemyLabel, "Enemy label", std::string("enemy"), VISUAL);
			CVar(StreamerEveryoneLabel, "Everyone label", std::string("player"), VISUAL);
			// Per-label cyclers.
			// Each cycles its label through a newline-separated list; they share NameCycleTrigger/Interval.
			CVar(TeamLabelCycle, "Cycle teammate label", false, VISUAL);
			CVar(TeamLabelCycleList, "Teammate label list", std::string(""), VISUAL | NOBIND);
			CVar(EnemyLabelCycle, "Cycle enemy label", false, VISUAL);
			CVar(EnemyLabelCycleList, "Enemy label list", std::string(""), VISUAL | NOBIND);
			CVar(EveryoneLabelCycle, "Cycle everyone label", false, VISUAL);
			CVar(EveryoneLabelCycleList, "Everyone label list", std::string(""), VISUAL | NOBIND);
			// (2) YOUR OWN name: display-only override of your name
			// everywhere the cheat renders names - it does not touch the server name.
			CVar(ChangeUsername, "Change your username", false, VISUAL);
			CVar(CustomLocalName, "Display name", std::string(""), VISUAL);
			// Name cycler: cycles your displayed name through the
			// list (newline-separated storage; per-entry rows in the menu, max 16 like k_maxNames).
			CVar(NameCycle, "Cycle display name", false, VISUAL);
			CVar(NameCycleList, "Name cycle list", std::string(""), VISUAL | NOBIND);
			CVarEnum(NameCycleTrigger, "Name cycle trigger", 0, VISUAL, nullptr,
				VA_LIST("Timer", "On kill"),
				Timer, OnKill);
			CVar(NameCycleInterval, "Interval", 5.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 0.1f, 60.f, 0.1f);
			// the reference custom crosshair (full port). Sizes are in "units" (1 unit = 2px), matching
			// the reference; thickness < 1.0 draws a 1px sub-pixel cross.
			CVar(Crosshair, "Crosshair", false, VISUAL);
			CVar(CrosshairAntialias, "Crosshair antialias", false, VISUAL);
			CVar(CrosshairThickness, "Crosshair thickness", 1.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 10.f, 0.1f);
			CVar(CrosshairLength, "Crosshair length", 4.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 40.f, 0.5f);
			CVar(CrosshairGap, "Crosshair gap", 2.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 40.f, 0.5f);
			CVar(CrosshairAspect, "Crosshair aspect", 1.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 0.1f, 5.f, 0.05f);
			CVar(CrosshairGapAspect, "Crosshair gap aspect", 1.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 0.1f, 5.f, 0.05f);
			CVar(CrosshairRotation, "Crosshair rotation", 0.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, -180.f, 180.f, 1.f);
			CVar(CrosshairUseSheen, "Crosshair track sheen color", false, VISUAL);
			// Which arms draw (clear bits to hide arms - e.g. clear Top for a T style). dirs 0-3.
			CVarEnum(CrosshairArms, "Crosshair arms", 0b1111, VISUAL | DROPDOWN_MULTI, nullptr,
				VA_LIST("Left", "Right", "Bottom", "Top"),
				Left = 1 << 0, Right = 1 << 1, Bottom = 1 << 2, Top = 1 << 3);
			// Dot.
			CVar(CrosshairDot, "Crosshair dot", false, VISUAL);
			CVar(CrosshairDotCircle, "Crosshair dot circle", false, VISUAL);
			CVar(CrosshairDotCustomColor, "Crosshair dot custom color", false, VISUAL);
			// Dynamic spread by speed.
			CVar(CrosshairDynamic, "Crosshair dynamic", false, VISUAL);
			CVar(CrosshairDynamicMin, "Crosshair dynamic min speed", 0.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 600.f, 10.f);
			CVar(CrosshairDynamicMax, "Crosshair dynamic spread", 3.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 20.f, 0.5f);
			CVar(CrosshairDynamicRef, "Crosshair dynamic ref speed", 400.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 1.f, 600.f, 10.f);
			// Gradient (center -> outer along each arm).
			CVar(CrosshairGradient, "Crosshair gradient", false, VISUAL);
			CVar(CrosshairGradStart, "Crosshair gradient start", 0.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 1.f, 0.05f);
			CVar(CrosshairGradEnd, "Crosshair gradient end", 1.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 1.f, 0.05f);
			// Taper (half-thickness scale near the gap -> at the tip).
			CVar(CrosshairTaper, "Crosshair taper", false, VISUAL);
			CVar(CrosshairTaperNear, "Crosshair taper near", 1.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 2.f, 0.05f);
			CVar(CrosshairTaperFar, "Crosshair taper far", 0.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 2.f, 0.05f);
			// Outline.
			CVar(CrosshairOutline, "Crosshair outline", false, VISUAL);
			CVar(CrosshairOutlineThick, "Crosshair outline thickness", 0.5f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 0.1f, 5.f, 0.1f);
			CVar(CrosshairOutlineOnLines, "Crosshair outline on lines", true, VISUAL);
			CVar(CrosshairOutlineOnDot, "Crosshair outline on dot", true, VISUAL);
			CVar(CrosshairOutlineBlur, "Crosshair outline blur", false, VISUAL);
			CVar(CrosshairOutlineBlurRadius, "Crosshair outline blur radius", 3.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 10.f, 0.5f);
			CVarEnum(CrosshairOutlineSides, "Crosshair outline sides", 0b1111, VISUAL | DROPDOWN_MULTI, nullptr,
				VA_LIST("Left", "Right", "Top", "Bottom"),
				Left = 1 << 0, Right = 1 << 1, Top = 1 << 2, Bottom = 1 << 3);
			CVarEnum(ChatTags, "Chat tags", 0b000, VISUAL | DROPDOWN_MULTI, nullptr,
				VA_LIST("Local", "Friends", "Party", "Assigned"),
				Local = 1 << 0, Friends = 1 << 1, Party = 1 << 2, Assigned = 1 << 3);
			CVar(FieldOfView, "Field of view## FOV", 0.f, VISUAL | SLIDER_MIN | SLIDER_PRECISION, 0.f, 160.f, 5.f);
			CVar(ZoomFieldOfView, "Zoomed field of view## Zoomed FOV", 0.f, VISUAL | SLIDER_MIN | SLIDER_PRECISION, 0.f, 160.f, 5.f);
			CVar(AspectRatio, "Aspect ratio", 0.f, VISUAL | SLIDER_MIN | SLIDER_PRECISION, 0.f, 5.f, 0.05f);
			CVar(RevealScoreboard, "Reveal scoreboard", false, VISUAL);
			CVar(ScoreboardUtility, "Scoreboard utility", false);
			CVar(ScoreboardColors, "Scoreboard colors", false, VISUAL);
			CVar(CleanScreenshots, "Clean screenshots", false);
			// Shared by both indicators: swap the default bold Verdana for the embedded thin "Doctrine" font.
			CVar(IndicatorThinFont, "Indicator thin font", false, VISUAL);
			CVar(VelocityIndicator, "Velocity indicator", false, VISUAL);
			CVar(VelocityPosX, VA_LIST("X## VelocityPosX", "Velocity position X"), 0.5f, VISUAL | SLIDER_PRECISION, 0.f, 1.f, 0.005f, "%.3f");
			CVar(VelocityPosY, VA_LIST("Y## VelocityPosY", "Velocity position Y"), 0.65f, VISUAL | SLIDER_PRECISION, 0.f, 1.f, 0.005f, "%.3f");
			CVar(VelocityScale, "Velocity graph scale", 1.f, VISUAL | SLIDER_PRECISION, 0.3f, 5.f, 0.05f, "%.2f");
			// number color
			CVar(VelocityColorBySpeed, "Velocity speed-based color", true, VISUAL);
			CVar(VelocityColorLow, "Velocity slow color", Color_t(255, 255, 255, 255), VISUAL);
			CVar(VelocityColorMid, "Velocity mid color", Color_t(102, 229, 102, 255), VISUAL);
			CVar(VelocityColorHigh, "Velocity fast color", Color_t(255, 76, 76, 255), VISUAL);
			CVar(VelocitySpeedLow, "Velocity slow threshold", 200.f, VISUAL | SLIDER_PRECISION, 0.f, 800.f, 5.f, "%.0f");
			CVar(VelocitySpeedHigh, "Velocity fast threshold", 450.f, VISUAL | SLIDER_PRECISION, 0.f, 800.f, 5.f, "%.0f");
			// shadow
			CVar(VelocityShadow, "Velocity shadow", true, VISUAL);
			CVar(VelocityShadowColor, "Velocity shadow color", Color_t(0, 0, 0, 140), VISUAL);
			CVar(VelocityShadowOffX, "Velocity shadow offset X", 1.5f, VISUAL | SLIDER_PRECISION, -20.f, 20.f, 0.5f, "%.1f");
			CVar(VelocityShadowOffY, "Velocity shadow offset Y", 1.5f, VISUAL | SLIDER_PRECISION, -20.f, 20.f, 0.5f, "%.1f");
			CVar(VelocityShadowBlur, "Velocity shadow blur", false, VISUAL);
			// outline
			CVar(VelocityOutline, "Velocity outline", false, VISUAL);
			CVar(VelocityOutlineColor, "Velocity outline color", Color_t(0, 0, 0, 255), VISUAL);
			CVar(VelocityOutlineThick, "Velocity outline thickness", 1.f, VISUAL | SLIDER_PRECISION, 0.5f, 6.f, 0.5f, "%.1f");
			// fade
			CVar(VelocityFadeIn, "Velocity fade in/out", true, VISUAL);
			CVar(VelocityFadeThreshold, "Velocity show above", 0.f, VISUAL | SLIDER_PRECISION, 0.f, 600.f, 5.f, "%.0f");
			// pre-jump (takeoff) speed
			CVar(VelocityTakeoff, "Velocity pre-jump speed", true, VISUAL);
			CVar(VelocityTakeoffHold, "Velocity pre-jump hold time", 3.f, VISUAL | SLIDER_PRECISION, 0.5f, 10.f, 0.1f, "%.1f");
			// Where the pre-jump number sits relative to the main speed number.
			CVarEnum(VelocityTakeoffPos, "Velocity pre-jump position", 0, VISUAL, nullptr,
				VA_LIST("Beside", "Above", "Below"),
				Beside, Above, Below);
			// Slide = slide in from the right, Expand = expand from center, Static = no motion,
			// pure alpha fade over the fade-time slider (0 = appears instantly).
			CVarEnum(VelocityTakeoffAnim, "Velocity pre-jump animation", 0, VISUAL, nullptr,
				VA_LIST("Slide", "Expand center", "Static"),
				Slide, Expand, Static);
			CVar(VelocityTakeoffFade, "Velocity pre-jump fade time", 0.15f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 1.f, 0.05f, "%.2f");
			// graph
			CVar(VelocityGraph, "Velocity graph", false, VISUAL);
			CVar(VelocityGraphPosX, VA_LIST("X## VelocityGraphPosX", "Velocity graph position X"), 0.5f, VISUAL | SLIDER_PRECISION, 0.f, 1.f, 0.005f, "%.3f");
			CVar(VelocityGraphPosY, VA_LIST("Y## VelocityGraphPosY", "Velocity graph position Y"), 0.72f, VISUAL | SLIDER_PRECISION, 0.f, 1.f, 0.005f, "%.3f");
			CVar(VelocityGraphW, "Velocity graph width", 185.f, VISUAL | SLIDER_PRECISION, 20.f, 1000.f, 5.f, "%.0f");
			CVar(VelocityGraphH, "Velocity graph height", 60.f, VISUAL | SLIDER_PRECISION, 10.f, 400.f, 5.f, "%.0f");
			CVar(VelocityGraphColor, "Velocity graph color", Color_t(102, 229, 102, 204), VISUAL);
			CVar(VelocityGraphMaxSpeed, "Velocity graph max speed", 500.f, VISUAL | SLIDER_PRECISION, 10.f, 2000.f, 10.f, "%.0f");
			CVarEnum(VelocityGraphFade, "Velocity graph fade", 1, VISUAL, nullptr,
				VA_LIST("None", "Both ends", "By velocity", "End", "Start"),
				None, BothEnds, ByVelocity, End, Start);
			CVar(VelocityGraphLineH, "Velocity graph line height", 1.f, VISUAL | SLIDER_PRECISION, 0.5f, 10.f, 0.5f, "%.1f");
			CVar(KeybindIndicator, "Keybind indicator", false, VISUAL);
			CVar(KeybindPos, "Keybind position", 130, VISUAL | SLIDER_CLAMP, 50, 500, 10);
			CVar(KeybindPosX, VA_LIST("X## KeybindPosX", "Keybind position X"), 0.5f, VISUAL | SLIDER_PRECISION, 0.f, 1.f, 0.005f, "%.3f");
			CVar(KeybindPosY, VA_LIST("Y## KeybindPosY", "Keybind position Y"), 0.9f, VISUAL | SLIDER_PRECISION, 0.f, 1.f, 0.005f, "%.3f");
			CVar(KeybindShadow, "Keybind shadow", true, VISUAL);
			CVar(KeybindShadowBlur, "Keybind shadow blur", false, VISUAL);
			CVar(KeybindShadowColor, "Keybind shadow color", Color_t(0, 0, 0, 220), VISUAL);
			CVar(KeybindShadowOffX, "Keybind shadow offset X", 1.f, VISUAL | SLIDER_PRECISION, -20.f, 20.f, 0.5f, "%.1f");
			CVar(KeybindShadowOffY, "Keybind shadow offset Y", 1.f, VISUAL | SLIDER_PRECISION, -20.f, 20.f, 0.5f, "%.1f");
			CVar(KeybindDetectColor, "Keybind detect color", Color_t(175, 210, 170, 255), VISUAL);
				// Recolour the main label text on detection (foreground), separate from the shadow tint.
				CVar(KeybindDetectText, "Keybind detect text", false, VISUAL);
				CVar(KeybindDetectTextColor, "Keybind detect text color", Color_t(175, 25, 20, 255), VISUAL);
			// - keybind indicator styles / animations.
			CVarEnum(KeybindLayout, "Keybind layout", 0, VISUAL, nullptr,
				VA_LIST("Horizontal", "Vertical"),
				Horizontal, Vertical);
			// Vertical stacking direction from the anchor (no centering, desktop list).
				CVarEnum(KeybindVerticalDir, "Keybind vertical direction", 0, VISUAL, nullptr,
					VA_LIST("Down", "Up"),
					Down, Up);
				CVar(KeybindTextColor, "Keybind text color", Color_t(255, 255, 255, 255), VISUAL);
			CVarEnum(KeybindEntryAnim, "Keybind entry animation", 0, VISUAL, nullptr,
				VA_LIST("Slide right", "Slide left", "Slide center", "Static"),
				SlideRight, SlideLeft, SlideCenter, Static);
			CVar(KeybindEntryOffset, "Keybind entry offset", 150.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 600.f, 5.f, "%.0f");
			CVar(KeybindFadeDuration, "Keybind fade duration", 0.25f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 0.05f, 2.f, 0.05f, "%.2f");
			// Easing shapes for the entry/exit slide+fade. Smooth = cubic in-out, Fast = sharp expo
			// snap, Bounce = bounces into place, Overshoot = shoots slightly past then settles.
			CVarEnum(KeybindEasing, "Keybind easing", 0, VISUAL, nullptr,
				VA_LIST("Smooth", "Linear", "Fast", "Bounce", "Overshoot"),
				Smooth, Linear, Fast, Bounce, Overshoot);
			CVar(KeybindDetectGlow, "Keybind detection glow", false, VISUAL);
			CVar(KeybindDetectGlowColor, "Keybind detection glow color", Color_t(175, 25, 20, 200), VISUAL);
				// Gaussian glow controls. Stencil = how much the glyph is boldened (dilated, px) BEFORE
				// blurring -> a solid/fat glow core; Blur = gaussian spread/softness (sigma px) of the skirt.
				CVar(KeybindDetectGlowBlur, "Keybind glow blur", 4.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 0.5f, 16.f, 0.5f, "%.1f");
				CVar(KeybindDetectGlowStencil, "Keybind glow stencil", 2, VISUAL | SLIDER_CLAMP, 0, 8, 1);
				// Detection particles: a little burst of shapes that erupts from inside the indicator label
				// the moment a feature is detected (edge bug, pixel surf, texture bug,...) and scatters in
				CVar(KeybindDetectParticles, "Keybind detection particles", false, VISUAL);
				CVar(KeybindDetectParticleColor, "Keybind detection particle color", Color_t(175, 210, 170, 255), VISUAL);
				CVarEnum(KeybindDetectParticleShape, "Keybind particle shape", 0, VISUAL, nullptr,
					VA_LIST("Square", "Circle", "Triangle", "Diamond", "Star", "Random"),
					Square, Circle, Triangle, Diamond, Star, Random);
				CVar(KeybindDetectParticleCount, "Keybind particle count", 12, VISUAL | SLIDER_CLAMP, 1, 60, 1);
				CVar(KeybindDetectParticleSpeed, "Keybind particle speed", 140.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 10.f, 600.f, 10.f, "%.0f");
				CVar(KeybindDetectParticleLifetime, "Keybind particle lifetime", 0.6f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 0.1f, 3.f, 0.1f, "%.1f");
				CVar(KeybindDetectParticleScale, "Keybind particle scale", 3.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 1.f, 12.f, 0.5f, "%.1f");
				CVar(KeybindDetectParticleSpread, "Keybind particle spread", 6.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 40.f, 1.f, "%.0f");
				CVar(KeybindDetectParticleGravity, "Keybind particle gravity", 90.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, -300.f, 800.f, 10.f, "%.0f");
				CVar(KeybindDetectParticleAntiAlias, "Keybind particle antialias", true, VISUAL);
				// Detection ripple: concentric rings that expand out of the label and fade, a few staggered
				// pulses per detection (the classic "drop in water" ring).
				CVar(KeybindDetectRipple, "Keybind detection ripple", false, VISUAL);
				CVar(KeybindDetectRippleColor, "Keybind detection ripple color", Color_t(175, 210, 170, 255), VISUAL);
				CVar(KeybindDetectRippleCount, "Keybind ripple count", 3, VISUAL | SLIDER_CLAMP, 1, 8, 1);
				CVar(KeybindDetectRippleRadius, "Keybind ripple radius", 22.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 4.f, 80.f, 1.f, "%.0f");
				CVar(KeybindDetectRippleDuration, "Keybind ripple duration", 0.7f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 0.1f, 2.f, 0.1f, "%.1f");
				CVar(KeybindDetectRippleThickness, "Keybind ripple thickness", 1.5f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, 1.f, 6.f, 0.5f, "%.1f");
				// Which feature indicators actually fire the detection effects (particles + ripple). 12 bits, all on (ids 0..11).
				CVar(KeybindDetectEffectMask, "Keybind detection effects", 4095, VISUAL);
			// "Name" doubles as the on-screen watermark name and the edge-bug indicator label (accent-coloured).
			CVar(Name, "Name", std::string("chudhook"), VISUAL | DROPDOWN_AUTOUPDATE);
			CVar(KeybindLabelLongJump, "Long jump label", std::string("lj"), VISUAL | DROPDOWN_AUTOUPDATE);
			CVar(KeybindLabelMiniJump, "Mini jump label", std::string("mj"), VISUAL | DROPDOWN_AUTOUPDATE);
			CVar(KeybindLabelAutoAlign, "Auto align label", std::string("aa"), VISUAL | DROPDOWN_AUTOUPDATE);
			CVar(KeybindLabelPixelSurf, "Pixel surf label", std::string("px"), VISUAL | DROPDOWN_AUTOUPDATE);
			CVar(KeybindLabelTextureBug, "Texture bug label", std::string("tb"), VISUAL | DROPDOWN_AUTOUPDATE);
			CVar(KeybindLabelWallClimb, "Wall climb label", std::string("wc"), VISUAL | DROPDOWN_AUTOUPDATE);
			CVar(KeybindLabelJumpBug, "Jump bug label", std::string("jb"), VISUAL | DROPDOWN_AUTOUPDATE);
			CVar(KeybindLabelPixelFinder, "Pixel finder label", std::string("pf"), VISUAL | DROPDOWN_AUTOUPDATE);
			CVar(KeybindLabelPixelSurfAssist, "Pixel surf assist label", std::string("ast"), VISUAL | DROPDOWN_AUTOUPDATE);
			CVar(KeybindLabelEdgeBug, "Edge bug label", std::string("eb"), VISUAL | DROPDOWN_AUTOUPDATE);
			CVar(KeybindLabelAutoStrafe, "Auto strafe label", std::string("as"), VISUAL | DROPDOWN_AUTOUPDATE);
				CVar(KeybindLabelAirStuck, "Air stuck label", std::string("air"), VISUAL | DROPDOWN_AUTOUPDATE); // id 11 ("as" taken by Auto strafe)
			// Bitmask of which keybind indicators are shown (bit per stable id, see Indicators.cpp).
			// Default 4095 = all 12 on (ids 0..11). Driven by a Multi dropdown in the menu.
			CVar(KeybindIndicatorMask, "Keybind indicators", 4095, VISUAL); // 12 bits (ids 0..11)
			// configurable font for keybind + velocity indicators.
			// Family = GDI font name (e.g. "Verdana", "Arial", "Tahoma"). Empty = default (Verdana).
			CVar(KeybindFontFamily, "Keybind font family", std::string(""), VISUAL | DROPDOWN_AUTOUPDATE);
			CVar(KeybindFontSize, "Keybind font size", 0.f, VISUAL | SLIDER_PRECISION, 0.f, 64.f, 1.f, "%.0f");
			CVar(VelocityFontFamily, "Velocity font family", std::string(""), VISUAL | DROPDOWN_AUTOUPDATE);
			CVar(VelocityFontSize, "Velocity font size", 0.f, VISUAL | SLIDER_PRECISION, 0.f, 64.f, 1.f, "%.0f");
		SUBNAMESPACE_END(UI);

		SUBNAMESPACE_BEGIN(Thirdperson)
			CVar(Enabled, "Thirdperson", false, VISUAL);
			CVar(Crosshair, VA_LIST("Crosshair", "Thirdperson crosshair"), false, VISUAL);
			CVar(Distance, "Thirdperson distance", 150.f, VISUAL | SLIDER_PRECISION, 0.f, 400.f, 10.f);
			CVar(Right, "Thirdperson right", 0.f, VISUAL | SLIDER_PRECISION, -100.f, 100.f, 5.f);
			CVar(Up, "Thirdperson up", 0.f, VISUAL | SLIDER_PRECISION, -100.f, 100.f, 5.f);

			CVar(Scale, "Thirdperson scales", true, NOSAVE | DEBUGVAR);
			CVar(Collide, "Thirdperson collides", true, NOSAVE | DEBUGVAR);
		SUBNAMESPACE_END(ThirdPerson);

		SUBNAMESPACE_BEGIN(Removals)
			CVar(Interpolation, VA_LIST("Interpolation", "Remove interpolation"), false);
			CVar(Lerp, VA_LIST("Lerp", "Remove lerp"), false);
			CVar(Disguises, VA_LIST("Disguises", "Remove disguises"), false, VISUAL);
			CVar(Taunts, VA_LIST("Taunts", "Remove taunts"), false, VISUAL);
			CVar(Scope, VA_LIST("Scope", "Remove scope"), false, VISUAL);
			CVar(PostProcessing, VA_LIST("Post processing", "Remove post processing"), false, VISUAL);
			CVar(ScreenOverlays, VA_LIST("Screen overlays", "Remove screen overlays"), false, VISUAL);
			CVar(ScreenEffects, VA_LIST("Screen effects", "Remove screen effects"), false, VISUAL);
			CVar(ViewPunch, VA_LIST("View punch", "Remove view punch"), false, VISUAL);
			CVar(AngleForcing, VA_LIST("Angle forcing", "Remove angle forcing"), false, VISUAL);
			CVar(Ragdolls, VA_LIST("Ragdolls", "Remove ragdoll"), false, VISUAL);
			CVar(Gibs, VA_LIST("Gibs", "Remove gibs"), false, VISUAL);
			CVar(MOTD, VA_LIST("MOTD", "Remove MOTD"), false, VISUAL);
			CVar(Cosmetics, VA_LIST("Cosmetics", "Remove cosmetics / hats"), false, VISUAL);
		SUBNAMESPACE_END(Removals);

		SUBNAMESPACE_BEGIN(Effects)
			CVarValues(BulletTracer, "Bullet tracer", std::string("Default"), VISUAL | DROPDOWN_CUSTOM, nullptr,
				"Default", "None", "Big nasty", "Distortion trail", "Machina", "Sniper rail", "Short circuit", "C.A.P.P.E.R", "Merasmus ZAP", "Merasmus ZAP 2", "Black ink", "Line", "Line ignore Z", "Beam");
			CVarValues(CritTracer, "Crit tracer", std::string("Default"), VISUAL | DROPDOWN_CUSTOM, nullptr,
				"Default", "None", "Big nasty", "Distortion trail", "Machina", "Sniper rail", "Short circuit", "C.A.P.P.E.R", "Merasmus ZAP", "Merasmus ZAP 2", "Black ink", "Line", "Line ignore Z", "Beam");
			CVarValues(MedigunBeam, "Medigun beam", std::string("Default"), VISUAL | DROPDOWN_CUSTOM, nullptr,
				"Default", "None", "Uber", "Dispenser", "Passtime", "Bombonomicon", "White", "Orange");
			CVarValues(MedigunCharge, "Medigun charge", std::string("Default"), VISUAL | DROPDOWN_CUSTOM, nullptr,
				"Default", "None", "Electrocuted", "Halloween", "Fireball", "Teleport", "Burning", "Scorching", "Purple energy", "Green energy", "Nebula", "Purple stars", "Green stars", "Sunbeams", "Spellbound", "Purple sparks", "Yellow sparks", "Green zap", "Yellow zap", "Plasma", "Frostbite", "Time warp", "Purple souls", "Green souls", "Bubbles", "Hearts");
			CVarValues(ProjectileTrail, "Projectile trail", std::string("Default"), VISUAL | DROPDOWN_CUSTOM, nullptr,
				"Default", "None", "Rocket", "Critical", "Energy", "Charged", "Ray", "Fireball", "Teleport", "Fire", "Flame", "Sparks", "Flare", "Trail", "Health", "Smoke", "Bubbles", "Halloween", "Monoculus", "Sparkles", "Rainbow");
			CVarEnum(SpellFootsteps, "Spell footsteps", 0, VISUAL, nullptr,
				VA_LIST("Off", "Color", "Team", "Halloween"),
				Off, Color, Team, Halloween);
			// Force a killstreak sheen on all of the local player's weapons.
			// The enum index doubles as the TF2 "killstreak_idleeffect" value (1-7), so it is read
			CVarEnum(WeaponSheen, "Weapon sheen", 0, VISUAL, nullptr,
				VA_LIST("Off", "Team Shine", "Deadly Daffodil", "Manndarin", "Mean Green", "Agonizing Emerald", "Villainous Violet", "Hot Rod"),
				Off, TeamShine, DeadlyDaffodil, Manndarin, MeanGreen, AgonizingEmerald, VillainousViolet, HotRod);
			// When on, the picked preset above only selects the sheen *animation* style; the actual
			// tint is replaced with Vars::Colors::WeaponSheen by the CProxyAnimatedWeaponSheen::OnBind
			CVar(WeaponSheenCustomColor, "Weapon sheen custom color", false, VISUAL);
			CVarEnum(RagdollEffects, "Ragdoll effects", 0b000000, VISUAL | DROPDOWN_MULTI, nullptr,
				VA_LIST("Burning", "Electrocuted", "Ash", "Dissolve", "##Divider", "Gold", "Ice"),
				Burning = 1 << 0, Electrocuted = 1 << 1, Ash = 1 << 2, Dissolve = 1 << 3, Gold = 1 << 4, Ice = 1 << 5);
			CVar(DrawIconsThroughWalls, "Draw icons through walls", false, VISUAL);
			CVar(DrawDamageNumbersThroughWalls, "Draw damage numbers through walls", false, VISUAL);
		SUBNAMESPACE_END(Tracers);

		SUBNAMESPACE_BEGIN(Viewmodel)
			CVar(CrosshairAim, "Crosshair aim position", false, VISUAL);
			CVar(ViewmodelAim, "Viewmodel aim position", false, VISUAL);
			CVar(OffsetX, VA_LIST("Offset X", "Viewmodel offset X"), 0.f, VISUAL | SLIDER_PRECISION, -45.f, 45.f, 5.f);
			CVar(OffsetY, VA_LIST("Offset Y", "Viewmodel offset Y"), 0.f, VISUAL | SLIDER_PRECISION, -45.f, 45.f, 5.f);
			CVar(OffsetZ, VA_LIST("Offset Z", "Viewmodel offset Z"), 0.f, VISUAL | SLIDER_PRECISION, -45.f, 45.f, 5.f);
			CVar(Pitch, VA_LIST("Pitch", "Viewmodel pitch"), 0.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, -180.f, 180.f, 5.f);
			CVar(Yaw, VA_LIST("Yaw", "Viewmodel yaw"), 0.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, -180.f, 180.f, 5.f);
			CVar(Roll, VA_LIST("Roll", "Viewmodel roll"), 0.f, VISUAL | SLIDER_CLAMP | SLIDER_PRECISION, -180.f, 180.f, 5.f);
			CVar(SwayScale, VA_LIST("Sway scale", "Viewmodel sway scale"), 0.f, VISUAL | SLIDER_MIN | SLIDER_PRECISION, 0.f, 5.f, 0.5f);
			CVar(SwayInterp, VA_LIST("Sway interp", "Viewmodel sway interp"), 0.f, VISUAL | SLIDER_MIN | SLIDER_PRECISION, 0.f, 1.f, 0.1f);
		SUBNAMESPACE_END(Viewmodel);

		SUBNAMESPACE_BEGIN(World)
			CVarEnum(Modulations, "Modulations", 0b00000, VISUAL | DROPDOWN_MULTI, nullptr,
				VA_LIST("World", "Sky", "Prop", "Particle", "Fog", "Rain", "Snow", "Storm", "Sandstorm", "Ash"),
				World = 1 << 0, Sky = 1 << 1, Prop = 1 << 2, Particle = 1 << 3, Fog = 1 << 4, Rain = 1 << 5, Snow = 1 << 6, Storm = 1 << 7, Sandstorm = 1 << 8, Ash = 1 << 9);
			CVarValues(SkyboxChanger, "Skybox changer", std::string("Off"), VISUAL | DROPDOWN_CUSTOM, nullptr,
				VA_LIST("Off"));
			CVarValues(WorldTexture, "World texture", std::string("Default"), VISUAL | DROPDOWN_CUSTOM, nullptr,
				"Default", "Dev", "Camo", "Black", "White", "Gray", "Flat");
			CVar(NearPropFade, "Near prop fade", false, VISUAL);
			CVar(NoPropFade, "No prop fade", false, VISUAL);
			// Flip world: horizontally mirror the world view (screen-space, in the DoPostScreenSpaceEffects
			CVar(FlipWorld, "Flip world", false, VISUAL);
		SUBNAMESPACE_END(World);

		// Weather: fog changer + native precipitation (rain / snow),. See CWeather.
		SUBNAMESPACE_BEGIN(Weather)
			// Fog: forces fog_override + the fog_* convars to override the map's fog.
			CVar(Fog, "Fog changer", false, VISUAL);
			CVar(FogStart, "Fog start", 0.f, VISUAL | SLIDER_PRECISION, 0.f, 10000.f, 50.f, "%.0f");
			CVar(FogEnd, "Fog end", 1000.f, VISUAL | SLIDER_PRECISION, 0.f, 30000.f, 50.f, "%.0f");
			CVar(FogDensity, "Fog density", 1.f, VISUAL | SLIDER_PRECISION, 0.f, 1.f, 0.05f, "%.2f");
			CVar(FogColor, "Fog color", Color_t(128, 128, 128, 255), VISUAL);
			CVar(FogLinkSkybox, "Fog link skybox", true, VISUAL);
			CVar(FogSkyStart, "Fog sky start", 0.f, VISUAL | SLIDER_PRECISION, 0.f, 10000.f, 50.f, "%.0f");
			CVar(FogSkyEnd, "Fog sky end", 1000.f, VISUAL | SLIDER_PRECISION, 0.f, 30000.f, 50.f, "%.0f");
			CVar(FogSkyDensity, "Fog sky density", 1.f, VISUAL | SLIDER_PRECISION, 0.f, 1.f, 0.05f, "%.2f");
			// Precipitation: spawns a client-side CPrecipitation sprite and drives r_rain*. Snow redirects the sprite.
			CVarEnum(Precipitation, "Precipitation", 0, VISUAL, nullptr,
				VA_LIST("Off", "Rain", "Snow"),
				Off, Rain, Snow);
			CVar(RainAlpha, "Rain alpha", 0.4f, VISUAL | SLIDER_PRECISION, 0.f, 1.f, 0.05f, "%.2f");
			CVar(RainSpeed, "Rain speed", 600.f, VISUAL | SLIDER_PRECISION, 0.f, 2000.f, 10.f, "%.0f");
			CVar(RainWidth, "Rain width", 0.5f, VISUAL | SLIDER_PRECISION, 0.f, 10.f, 0.1f, "%.2f");
			CVar(RainLength, "Rain length", 0.1f, VISUAL | SLIDER_PRECISION, 0.f, 10.f, 0.05f, "%.2f");
			CVar(RainRadius, "Rain radius", 1500.f, VISUAL | SLIDER_PRECISION, 0.f, 4000.f, 50.f, "%.0f");
			CVar(RainSideVel, "Rain side velocity", 130.f, VISUAL | SLIDER_PRECISION, 0.f, 500.f, 5.f, "%.0f");
			CVar(SnowAlpha, "Snow alpha", 1.f, VISUAL | SLIDER_PRECISION, 0.f, 1.f, 0.05f, "%.2f");
			CVar(SnowSpeed, "Snow speed", 80.f, VISUAL | SLIDER_PRECISION, 0.f, 2000.f, 10.f, "%.0f");
			CVar(SnowWidth, "Snow width", 3.f, VISUAL | SLIDER_PRECISION, 0.f, 10.f, 0.1f, "%.2f");
			CVar(SnowLength, "Snow length", 0.02f, VISUAL | SLIDER_PRECISION, 0.f, 10.f, 0.01f, "%.2f");
			CVar(SnowRadius, "Snow radius", 1500.f, VISUAL | SLIDER_PRECISION, 0.f, 4000.f, 50.f, "%.0f");
			CVar(SnowSideVel, "Snow side velocity", 15.f, VISUAL | SLIDER_PRECISION, 0.f, 500.f, 5.f, "%.0f");
			CVar(SnowDensity, "Snow density", 0.35f, VISUAL | SLIDER_PRECISION, 0.1f, 1.f, 0.05f, "%.2f");
			CVar(SnowSprite, "Snow sprite", std::string("particle/snow"), VISUAL | DROPDOWN_AUTOUPDATE);
		SUBNAMESPACE_END(Weather);

		SUBNAMESPACE_BEGIN(Beams) // as of now, these will stay out of the menu
			CVar(Model, "Model", std::string("sprites/physbeam.vmt"), VISUAL);
			CVar(Life, "Life", 2.f, VISUAL | SLIDER_MIN | SLIDER_PRECISION, 0.f, 10.f);
			CVar(Width, "Width", 2.f, VISUAL | SLIDER_MIN | SLIDER_PRECISION, 0.f, 10.f);
			CVar(EndWidth, "End width", 2.f, VISUAL | SLIDER_MIN | SLIDER_PRECISION, 0.f, 10.f);
			CVar(FadeLength, "Fade length", 10.f, VISUAL | SLIDER_MIN | SLIDER_PRECISION, 0.f, 30.f);
			CVar(Amplitude, "Amplitude", 2.f, VISUAL | SLIDER_MIN | SLIDER_PRECISION, 0.f, 10.f);
			CVar(Brightness, "Brightness", 255.f, VISUAL | SLIDER_MIN | SLIDER_PRECISION, 0.f, 255.f);
			CVar(Speed, "Speed", 0.2f, VISUAL | SLIDER_MIN | SLIDER_PRECISION, 0.f, 5.f);
			CVar(Segments, "Segments", 2, VISUAL | SLIDER_MIN, 1, 10);
			CVar(Color, "Color", Color_t(255, 255, 255, 255), VISUAL);
			CVarEnum(Flags, "Flags", 0b10000000100000000, VISUAL | DROPDOWN_MULTI, nullptr,
				VA_LIST("Start entity", "End entity", "Fade in", "Fade out", "Sine noise", "Solid", "Shade in", "Shade out", "Only noise once", "No tile", "Use hitboxes", "Start visible", "End visible", "Is active", "Forever", "Halobeam", "Reverse"),
				StartEntity = 1 << 0, EndEntity = 1 << 1, FadeIn = 1 << 2, FadeOut = 1 << 3, SineNoise = 1 << 4, Solid = 1 << 5, ShadeIn = 1 << 6, ShadeOut = 1 << 7, OnlyNoiseOnce = 1 << 8, NoTile = 1 << 9, UseHitboxes = 1 << 10, StartVisible = 1 << 11, EndVisible = 1 << 12, IsActive = 1 << 13, Forever = 1 << 14, Halobeam = 1 << 15, Reverse = 1 << 16);
		SUBNAMESPACE_END(Beams);

		SUBNAMESPACE_BEGIN(Line)
			CVar(Enabled, "Line tracers", false, VISUAL);
			CVar(DrawDuration, VA_LIST("Draw duration", "Line draw duration"), 5.f, VISUAL | SLIDER_MIN | SLIDER_PRECISION, 0.f, 10.f);
		SUBNAMESPACE_END(Line);

		SUBNAMESPACE_BEGIN(Hitbox)
			CVarEnum(BonesEnabled, VA_LIST("Bones enabled", "Hitbox bones enabled"), 0b00, VISUAL | DROPDOWN_MULTI, "Off",
				VA_LIST("On shot", "On hit"),
				OnShot = 1 << 0, OnHit = 1 << 1);
			CVarEnum(BoundsEnabled, VA_LIST("Bounds enabled", "Hitbox bounds enabled"), 0b000, VISUAL | DROPDOWN_MULTI, "Off",
				VA_LIST("On shot", "On hit", "Aim point"),
				OnShot = 1 << 0, OnHit = 1 << 1, AimPoint = 1 << 2);
			CVar(DrawDuration, VA_LIST("Draw duration", "Hitbox draw duration"), 5.f, VISUAL | SLIDER_MIN | SLIDER_PRECISION, 0.f, 10.f);
		SUBNAMESPACE_END(Hitbox);

		SUBNAMESPACE_BEGIN(Simulation)
			Enum(Style, Off, Line, Separators, Spaced, Arrows, Boxes);
			CVarValues(PlayerPath, "Player path", 0, VISUAL, nullptr,
				"Off", "Line", "Separators", "Spaced", "Arrows", "Boxes");
			CVarValues(ProjectilePath, "Projectile path", 0, VISUAL, nullptr,
				"Off", "Line", "Separators", "Spaced", "Arrows", "Boxes");
			CVarValues(TrajectoryPath, "Trajectory path", 0, VISUAL, nullptr,
				"Off", "Line", "Separators", "Spaced", "Arrows", "Boxes");
			CVarValues(ShotPath, "Shot path", 0, VISUAL, nullptr,
				"Off", "Line", "Separators", "Spaced", "Arrows", "Boxes");
			CVarEnum(SplashRadius, "Splash radius", 0b0, VISUAL | DROPDOWN_MULTI, "Off",
				VA_LIST("Simulation", "##Divider", "Priority", "Enemy", "Team", "Local", "Friends", "Party", "##Divider", "Rockets", "Stickies", "Pipes", "Scorch shot", "##Divider", "Trace"),
				Simulation = 1 << 0, Priority = 1 << 1, Enemy = 1 << 2, Team = 1 << 3, Local = 1 << 4, Friends = 1 << 5, Party = 1 << 6, Rockets = 1 << 7, Stickies = 1 << 8, Pipes = 1 << 9, ScorchShot = 1 << 10, Trace = 1 << 11);
			CVar(Timed, VA_LIST("Timed", "Timed path"), false, VISUAL);
			CVar(Box, VA_LIST("Box", "Path box"), true, VISUAL);
			CVar(ProjectileCamera, "Projectile camera", false, VISUAL);
			CVar(ProjectileWindow, "Projectile window", WindowBox_t(), VISUAL | NOBIND);
			CVar(SwingLines, "Swing lines", false, VISUAL);
			CVar(DrawDuration, VA_LIST("Draw duration", "Simulation draw duration"), 5.f, VISUAL | SLIDER_MIN | SLIDER_PRECISION, 0.f, 10.f);

			CVarValues(RealPath, "Real path", 0, NOSAVE | DEBUGVAR, nullptr,
				"Off", "Line", "Separators", "Spaced", "Arrows", "Boxes");
			CVar(SeparatorSpacing, "Separator spacing", 4, NOSAVE | DEBUGVAR, 1, 16);
			CVar(SeparatorLength, "Separator length", 12.f, NOSAVE | DEBUGVAR, 2.f, 16.f);
		SUBNAMESPACE_END(Simulation);

		SUBNAMESPACE_BEGIN(Trajectory)
			CVar(Override, "Simulation override", false, NOSAVE | DEBUGVAR);
			CVar(OffsetX, "Offset X", 16.f, NOSAVE | DEBUGVAR | SLIDER_PRECISION, -25.f, 25.f, 0.5f);
			CVar(OffsetY, "Offset Y", 8.f, NOSAVE | DEBUGVAR | SLIDER_PRECISION, -25.f, 25.f, 0.5f);
			CVar(OffsetZ, "Offset Z", -6.f, NOSAVE | DEBUGVAR | SLIDER_PRECISION, -25.f, 25.f, 0.5f);
			CVar(Pipes, "Pipes", true, NOSAVE | DEBUGVAR);
			CVar(Hull, "Hull", 5.f, NOSAVE | DEBUGVAR | SLIDER_MIN | SLIDER_PRECISION, 0.f, 10.f, 0.5f);
			CVar(Speed, "Speed", 1200.f, NOSAVE | DEBUGVAR | SLIDER_MIN | SLIDER_PRECISION, 0.f, 5000.f, 50.f);
			CVar(Gravity, "Gravity", 1.f, NOSAVE | DEBUGVAR | SLIDER_PRECISION, 0.f, 1.f, 0.1f);
			CVar(LifeTime, "Life time", 2.2f, NOSAVE | DEBUGVAR | SLIDER_MIN | SLIDER_PRECISION, 0.f, 10.f, 0.1f);
			CVar(UpVelocity, "Up velocity", 200.f, NOSAVE | DEBUGVAR | SLIDER_PRECISION, 0.f, 1000.f, 50.f);
			CVar(AngularVelocityX, "Angular velocity X", 600.f, NOSAVE | DEBUGVAR | SLIDER_PRECISION, -1000.f, 1000.f, 50.f);
			CVar(AngularVelocityY, "Angular velocity Y", -1200.f, NOSAVE | DEBUGVAR | SLIDER_PRECISION, -1000.f, 1000.f, 50.f);
			CVar(AngularVelocityZ, "Angular velocity Z", 0.f, NOSAVE | DEBUGVAR | SLIDER_PRECISION, -1000.f, 1000.f, 50.f);
			CVar(Drag, "Drag", 1.f, NOSAVE | DEBUGVAR | SLIDER_PRECISION, 0.f, 2.f, 0.1f);
			CVar(DragX, "Drag X", 0.003902f, NOSAVE | DEBUGVAR | SLIDER_PRECISION, 0.f, 0.1f, 0.01f, "%.15g");
			CVar(DragY, "Drag Y", 0.009962f, NOSAVE | DEBUGVAR | SLIDER_PRECISION, 0.f, 0.1f, 0.01f, "%.15g");
			CVar(DragZ, "Drag Z", 0.009962f, NOSAVE | DEBUGVAR | SLIDER_PRECISION, 0.f, 0.1f, 0.01f, "%.15g");
			CVar(AngularDragX, "Angular drag X", 0.003618f, NOSAVE | DEBUGVAR | SLIDER_PRECISION, 0.f, 0.1f, 0.01f, "%.15g");
			CVar(AngularDragY, "Angular drag Y", 0.001514f, NOSAVE | DEBUGVAR | SLIDER_PRECISION, 0.f, 0.1f, 0.01f, "%.15g");
			CVar(AngularDragZ, "Angular drag Z", 0.001514f, NOSAVE | DEBUGVAR | SLIDER_PRECISION, 0.f, 0.1f, 0.01f, "%.15g");
			CVar(MaxVelocity, "Max velocity", 2000.f, NOSAVE | DEBUGVAR | SLIDER_MIN | SLIDER_PRECISION, 0.f, 4000.f, 50.f);
			CVar(MaxAngularVelocity, "Max angular velocity", 3600.f, NOSAVE | DEBUGVAR | SLIDER_MIN | SLIDER_PRECISION, 0.f, 7200.f, 50.f);
		SUBNAMESPACE_END(ProjectileTrajectory);

		// ESP tab "Model Preview" window: renders the heavy player model (stand_MELEE)
		SUBNAMESPACE_BEGIN(ModelPreview)
			CVar(Enabled, "Model preview", false, VISUAL);
		SUBNAMESPACE_END(ModelPreview);
	NAMESPACE_END(Visuals);

	NAMESPACE_BEGIN(Misc)
		SUBNAMESPACE_BEGIN(Movement)
			CVar(AutoStrafe, "Auto strafe", false); // single checkbox, perfect-strafe
			// manual strafe optimizer. Perfects strafes you do yourself -
			// while airborne, A/D only (no W/S) and turning the mouse the right way, it snaps the view
			CVar(StrafeOptimizer, "Strafe optimizer", false);
			CVar(StrafeOptimizerGain, "Strafe optimizer gain", 100, SLIDER_CLAMP, 0, 100); // % of optimal correction
			CVar(StrafeOptimizerMinSpeed, "Strafe optimizer min speed", 30.f, SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 320.f, 5.f);
			CVar(Bunnyhop, "Bunnyhop", false);
			CVar(EdgeJump, "Edge jump", false);
			CVar(AutoJumpbug, "Jumpbug", false);
			CVar(NoPush, "No push", false);
			CVar(AutoRocketJump, "Auto rocket jump", false);
			CVar(AutoCTap, "Auto ctap", false);
			CVar(FastStop, "Fast stop", false);
			CVar(FastAccelerate, "Fast accelerate", false);
			CVar(DuckSpeed, "Duck speed", false);
			CVar(MovementLock, "Movement lock", false);
			CVar(BreakJump, "Break jump", false);
			CVar(ShieldTurnRate, "Shield turn rate", false);
			CVar(EdgeBug, "Edge bug", false);
			CVar(EdgeBugLockTicks, "Edge bug scan ticks", 32, SLIDER_CLAMP, 1, 128);
			CVar(EdgeBugAdvancedSearch, "Edge bug advanced search", false);
			CVar(EdgeBugAutoStrafe, "Edge bug auto strafe", false);
			CVar(EdgeBugAngleLimit, "Edge bug angle limit", 20.f, SLIDER_CLAMP, 0.f, 45.f);
			CVar(EdgeBugScanRadius, "Edge bug scan radius", 4, SLIDER_CLAMP, 1, 12);
			CVar(EdgeBugVisualize, "Edge bug visualize", false);
			CVar(EdgeBugChatPrint, "Edge bug chat print", false);
			// play a sound when an edge bug is hit. The presets map to
			// stock TF2 ui/tool sounds; Custom plays EdgeBugSoundCustom (a path under tf/sound/).
			CVarEnum(EdgeBugSound, "Edge bug sound", 0, NONE, nullptr,
				VA_LIST("Off", "Panel open", "Menu focus", "Button click", "Button rollover", "Beep", "Blip", "Custom"),
				Off, PanelOpen, MenuFocus, ButtonClick, ButtonRollover, Beep, Blip, Custom);
			CVar(EdgeBugSoundCustom, "Edge bug custom sound", std::string(""), NOBIND);
			CVar(EdgeBugMouseLock, "Edge bug mouse lock", false);
			CVarEnum(EdgeBugMouseLockType, "Edge bug lock type", 0, NONE, nullptr,
				VA_LIST("Classic", "Static", "Dynamic", "Predicted"),
				Classic, Static, Dynamic, Predicted);
			CVar(EdgeBugLockAmount, "Edge bug lock amount", 80.f, SLIDER_CLAMP, 0.f, 100.f);
			CVar(LongJump, "Long jump", false);
			CVar(MiniJump, "Mini jump", false);
			CVar(MiniJumpHoldDuck, "Mini jump hold duck", false);
			// jump -> minijump repair: park a jump press the engine would eat (mid-air, or grounded
			// while still ducked/unducking from the last hop) and replay it the first tick a clean
			CVar(MiniJumpQueue, "Mini jump queue presses", false);
			CVar(AutoAlign, "Auto align", false);
			CVar(PixelSurf, "Pixel surf", false);
			CVar(TextureBug, "Texture bug", false);
			CVar(TextureBugChokeTick, "Texture bug choke tick", false);
			CVar(TextureBugChokeTicks, "Texture bug choke ticks", 1, SLIDER_CLAMP, 1, 14);
			CVar(TextureBugAutoCrouch, "Texture bug auto crouch", false); // duck into the wall when standing misses
			// Edge stop: brake at surf ends + steer onto passing ledges. Grabbier (manufactures grabs).
			CVar(TextureBugEdgeStop, "Texture bug edge stop", true);
			// Ride hold: ticks to replay the last catch move across a brush seam so rides carry over.
			CVar(TextureBugHoldTicks, "Texture bug hold ticks", 6, SLIDER_CLAMP, 0, 16);
			// Catch tolerance: the pin band. Raise to trigger/stick more, lower for fewer false catches.
			CVar(TextureBugCatchEps, "Texture bug catch tolerance", 1.0f, SLIDER_CLAMP | SLIDER_PRECISION, 0.25f, 4.0f, 0.05f);
			// Wall reach (HU past the hull): how far out a wall is grabbed.
			CVar(TextureBugReach, "Texture bug reach", 12.f, SLIDER_CLAMP, 1.f, 48.f, 1.f);
			// Vertical scan step (HU): the perf lever. 1 = densest/most accurate, up to 8 = coarser/cheaper.
			CVar(TextureBugScanStep, "Texture bug scan step", 2.f, SLIDER_CLAMP, 1.f, 8.f, 1.f);
			// HeadSurf (from the tb reference): mirror of a wall catch - pin vz to +half-gravity when a
			// brush bottom is just above your head so the engine thinks you stand ON it. Own toggle now.
			CVar(HeadSurf, "Head surf", false);
			CVar(WallClimb, "Wall climb", false);
			// -- Air stuck -------------------------
			CVar(AirStuck, "Air stuck", false);
			// Catch tolerance: is_target_predict_z_velocity band. RAISE = sticks more/easier (looser catch).
			CVar(AirStuckCatchEps, "Air stuck catch tolerance", 4.5f, SLIDER_CLAMP | SLIDER_PRECISION, 1.0f, 8.0f, 0.25f);
			// Wall detection reach (HU): how far out a wall is grabbed. Higher = engages sooner/farther.
			CVar(AirStuckReach, "Air stuck reach", 72.f, SLIDER_CLAMP, 16.f, 128.f, 1.f);
			// Gap (HU) at which the catch sweep starts instead of cheap drift. Higher = sweep starts sooner.
			CVar(AirStuckFlushTol, "Air stuck flush tolerance", 56.f, SLIDER_CLAMP, 8.f, 96.f, 1.f);
			// Drive-in strength (forwardmove = gap * gain). Higher = closes the gap faster.
			CVar(AirStuckDriftGain, "Air stuck drift gain", 18.f, SLIDER_CLAMP, 4.f, 40.f, 1.f);
			// Hard cap on catch-sweep Simulate()s per tick (perf lever). Higher = wider band, more FPS cost.
			CVar(AirStuckSimBudget, "Air stuck sim budget", 64, SLIDER_CLAMP, 16, 128);

			// Pixel finder / pixel surf assist. PixelFinder, PixelSurfAssist
			CVar(PixelFinder, "Pixel finder", false);
			CVar(PixelFinderScanMargin, "Pixel finder scan margin", 8.f, SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 64.f, 1.f);
			// Pixel surf line: the moment you START riding a pixel
			CVar(PixelSurfLine, "Pixel surf line", false);
			CVar(PixelSurfLineColor, "Pixel surf line color", Color_t(0, 255, 255, 255), VISUAL);
			CVar(PixelSurfAssist, "Pixel surf assist", false);
			CVar(PixelSurfAssistSetPoint, "Pixel surf assist set point", false);
			CVar(PixelSurfAssistRender, "Pixel surf assist render", false);
			// On-screen reach meter: needed height to the targeted point + each jump type's live
			// Steer lock: after the assist fires a jump, hold the commit-tick movement (world-space)
			CVar(PixelSurfAssistSteer, "Pixel surf assist steer lock", false);
			// When the assist jumps FOR you, pop a small box near the crosshair naming the jump it
			// fired and the target's Z height, e.g. "minijump (duck) to 143.453".
			CVar(PixelSurfAssistJumpBox, "Pixel surf assist jump box", false);
			// Vertical screen position of that pop-up box, as a fraction of screen height (0 = top,
			// 1 = bottom). Default sits a little below the crosshair.
			CVar(PixelSurfAssistJumpBoxPosY, "Pixel surf assist jump box Y", 0.62f, SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 1.f, 0.01f);
			CVar(PixelSurfAssistRadius, "Pixel surf assist radius", 300.f, SLIDER_CLAMP | SLIDER_PRECISION, 16.f, 1024.f, 16.f);
			// Per jump-type reach.
			CVar(PixelSurfAssistReachMini, "PSA reach mini", 30.f, SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 160.f, 1.f);
			CVar(PixelSurfAssistReachRegular, "PSA reach regular", 45.f, SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 160.f, 1.f);
			CVar(PixelSurfAssistReachMiniDuck, "PSA reach mini+duck", 55.f, SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 160.f, 1.f);
			CVar(PixelSurfAssistReachCrouch, "PSA reach crouch", 72.f, SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 160.f, 1.f);
			CVar(PixelSurfAssistReachLong, "PSA reach long", 60.f, SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 160.f, 1.f);
			CVar(PixelSurfAssistReachLongDuck, "PSA reach long+duck", 85.f, SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 160.f, 1.f);
			CVar(PixelSurfAssistLongMinDist, "PSA long min distance", 200.f, SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 600.f, 10.f);
			CVar(PixelSurfAssistDeletePoint, "Pixel surf assist delete point", false);
			// Tolerance (units) for snapping a saved target onto a finder spot, and for picking
			// which saved point the delete key removes.
			CVar(PixelSurfAssistSnapDist, "PSA snap/delete distance", 48.f, SLIDER_CLAMP | SLIDER_PRECISION, 4.f, 256.f, 1.f);

			// movement recorder. Records the usercmd you send each tick and replays it
			// verbatim. Each map gets a file (chudhook\Recordings\<map>.rec) holding MULTIPLE named
			CVar(MovementRecorder, "Movement recorder", false);
			CVar(MovementRecorderRecord, "Recorder record", false);
			CVar(MovementRecorderPlay, "Recorder play", false);
			CVar(MovementRecorderStop, "Recorder stop", false);
			CVar(MovementRecorderSave, "Recorder save", false);
			CVar(MovementRecorderLoad, "Recorder load", false);
			CVar(MovementRecorderClear, "Recorder clear", false);
			CVar(MovementRecorderHud, "Recorder HUD", false);
			// On Play, walk to the nearest route's start before replaying.
			CVar(MovementRecorderMoveToStart, "Recorder move to start", false);
			CVar(MovementRecorderLockView, "Recorder lock view", false);   // lock the camera to the replayed view
			CVar(MovementRecorderMoveToStartDist, "Recorder move-to-start max", 600, SLIDER_CLAMP, 0, 4096);
			// Start precision: playback only begins once we're within this many units of the recorded
			CVar(MovementRecorderStartTolerance, "Recorder start tolerance", 0.1f, SLIDER_CLAMP | SLIDER_PRECISION, 0.02f, 8.f, 0.01f);
			CVar(MovementRecorderStartSpeed, "Recorder start speed", 6, SLIDER_CLAMP, 0, 50);
			// Drift correction: open-loop input replay diverges because the server simulates movement
			// authoritatively and tiny start/quantization errors compound. With this on, playback nudges
			CVar(MovementRecorderDriftCorrect, "Recorder drift correct", false);
			CVar(MovementRecorderDriftStrength, "Recorder drift strength", 60.f, SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 250.f, 5.f);
			// Verbatim replay: replays the recorded inputs byte-for-byte with NO drift correction at all
			// (closed loop off). For clean routes recorded from a dead standstill this is the most faithful
			CVar(MovementRecorderVerbatim, "Recorder verbatim replay", false);
			// Draw a marker on the map at each saved route's start, and the path line while busy.
			CVar(MovementRecorderShowRoutes, "Recorder show routes", false);
			CVar(MovementRecorderShowPath, "Recorder show path", false);
			// Route start markers: a ring of dots on the floor (so they don't read like PS points),
			// route name in the middle. Solid = a smooth ground circle instead of separate dots.
			CVar(MovementRecorderRouteSolid, "Recorder route solid ring", false);
			CVar(MovementRecorderRoutePoints, "Recorder route points", 8, SLIDER_CLAMP, 3, 32);
			CVar(MovementRecorderRouteRadius, "Recorder route radius", 28.f, SLIDER_CLAMP | SLIDER_PRECISION, 8.f, 128.f, 1.f);
			CVar(MovementRecorderRouteColor, "Recorder route color", Color_t(255, 255, 255, 255), VISUAL);
			// Shadowplay: while idle, keep a rolling buffer of the last N seconds of movement.
			CVar(MovementRecorderShadowPlay, "Recorder shadowplay", false);
			CVar(MovementRecorderShadowSave, "Recorder shadow save", false);
			CVar(MovementRecorderShadowSeconds, "Recorder shadow seconds", 60, SLIDER_CLAMP, 5, 120);

			// jump stats. On landing, shows the last jump's distance, takeoff/max speed,
			// height and a coarse type (jump / crouch jump / bhop xN). the reference shows takeoff
			CVar(JumpStats, "Jump stats", false);
			CVar(JumpStatsDuration, "Jump stats duration", 3.f, SLIDER_CLAMP | SLIDER_PRECISION, 0.5f, 10.f, 0.5f);

			// Fake POV (Movement > FAKE POV): bake a fake camera angle into recorded POV demos via the
			// Clean POV demo path (CPrediction::GetLocalViewAngles) without touching live aim. The angle
			CVar(FakePOV, "Fake POV", false);
			CVarEnum(FakePOVMode, "Fake POV angle", 0, NONE, nullptr,
				VA_LIST("Right", "Left", "Up", "Bottom", "Backwards", "Spinning"),
				Right, Left, Up, Bottom, Backwards, Spinning);
			CVar(FakePOVSpinSpeed, "Fake POV spin speed", 120.f, SLIDER_CLAMP | SLIDER_PRECISION, 10.f, 540.f, 5.f); // deg/sec
			CVar(FakePOVSmoothSpeed, "Fake POV smoothing", 6.f, SLIDER_CLAMP | SLIDER_PRECISION, 0.5f, 20.f, 0.5f);  // higher = snappier turn
			// Snap view on disable: when turning Fake POV OFF while recording a demo, rotate your real
			CVar(FakePOVSnapView, "Snap view on disable", false);
			CVar(FakePOVArrow, "Fake POV arrow", false);
			CVar(FakePOVArrowSize, "Fake POV arrow size", 16.f, SLIDER_CLAMP | SLIDER_PRECISION, 4.f, 64.f, 1.f);
			CVar(FakePOVArrowDist, "Fake POV arrow distance", 90.f, SLIDER_CLAMP | SLIDER_PRECISION, 0.f, 400.f, 5.f);
			CVar(FakePOVArrowColor, "Fake POV arrow color", Color_t(255, 255, 255, 255), VISUAL);

			// Checkpoints (KZ practice). Matches the reference/the reference EXACTLY: one saved spot,
			// Save (must be on ground) + Teleport (setpos+setang) + Noclip, each a bindable key shown
			CVar(Checkpoints, "Checkpoints", false);
			CVar(CheckpointSaveKey, "Checkpoint save key", 0, NOBIND);     // VK code; 0 = unbound
			CVar(CheckpointTeleportKey, "Checkpoint teleport key", 0, NOBIND);
			CVar(CheckpointNoclipKey, "Checkpoint noclip key", 0, NOBIND);

			CVar(TimingOffset, "Timing offset", 0, NOSAVE | DEBUGVAR, 0, 3);
			CVar(ChokeCount, "Choke count", 1, NOSAVE | DEBUGVAR, 0, 3);
			CVar(ApplyAbove, "Apply timing offset above", 0, NOSAVE | DEBUGVAR, 0, 8);
		SUBNAMESPACE_END(Movement);

		SUBNAMESPACE_BEGIN(Automation)
			CVarEnum(AntiBackstab, "Anti-backstab", 0, NONE, nullptr,
				VA_LIST("Off", "Yaw", "Pitch", "Fake"),
				Off, Yaw, Pitch, Fake);
			CVar(AntiAFK, "Anti-AFK", false);
			CVar(AntiAutobalance, "Anti-autobalance", false);
			CVar(TauntControl, "Taunt control", false);
			CVar(KartControl, "Kart control", false);
			CVar(AutoF2Ignored, "Auto F2 ignored", false);
			CVar(AutoF1Priority, "Auto F1 priority", false);
			CVar(AcceptItemDrops, "Auto accept item drops", false);
		SUBNAMESPACE_END(Automation);

		SUBNAMESPACE_BEGIN(Exploits)
			CVar(PureBypass, "Pure bypass", false);
			CVar(CheatsBypass, "Cheats bypass", false);
			CVar(EquipRegionUnlock, "Equip region unlock", false);
			CVar(BackpackExpander, "Backpack expander", false);
			CVar(NoisemakerSpam, "Noisemaker spam", false);
			CVar(PingReducer, "Ping reducer", false);
			CVar(PingTarget, "cl_cmdrate", 1, SLIDER_CLAMP, 1, 66);
		SUBNAMESPACE_END(Exploits);

		SUBNAMESPACE_BEGIN(Game)
			CVar(AntiCheatCompatibility, "Anti-cheat compatibility", false);
			CVar(F2PChatBypass, "F2P chat bypass", false);
			CVar(NetworkFix, "Network fix", false);
			CVar(SetupBonesOptimization, "Bones optimization", false);
			// Write your real view angle into your local POV demos so silent aim never snaps the first-person camera.
			CVar(CleanPOVDemos, "Clean POV demos", false);
			// Stop the Ambassador crosshair from shrinking/growing while the headshot accuracy recharges.
			CVar(NoAmbyCrosshairResize, "No amby xhair resize", false);

			CVar(AntiCheatCritHack, "Anti-cheat crit hack", false, NOSAVE | DEBUGVAR);
		SUBNAMESPACE_END(Game);

		SUBNAMESPACE_BEGIN(Queueing)
			CVarEnum(ForceRegions, "Force regions", 0b0, DROPDOWN_MULTI, nullptr, // i'm not sure all of these are actually used for tf2 servers
				VA_LIST("Atlanta", "Chicago", "Dallas", "Los Angeles", "Seattle", "Virginia", "##Divider", "Amsterdam", "Falkenstein", "Frankfurt", "Helsinki", "London", "Madrid", "Paris", "Stockholm", "Vienna", "Warsaw", "##Divider", "Buenos Aires", "Lima", "Santiago", "Sao Paulo", "##Divider", "Chennai", "Dubai", "Hong Kong", "Mumbai", "Seoul", "Singapore", "Tokyo", "##Divider", "Sydney", "##Divider", "Johannesburg"),
				// North America
				ATL = 1 << 0, // Atlanta
				ORD = 1 << 1, // Chicago
				DFW = 1 << 2, // Dallas
				LAX = 1 << 3, // Los Angeles
				SEA = 1 << 4, // Seattle (+DC_EAT?)
				IAD = 1 << 5, // Virginia
				// Europe
				AMS = 1 << 6, // Amsterdam
				FSN = 1 << 7, // Falkenstein
				FRA = 1 << 8, // Frankfurt
				HEL = 1 << 9, // Helsinki
				LHR = 1 << 10, // London
				MAD = 1 << 11, // Madrid
				PAR = 1 << 12, // Paris
				STO = 1 << 13, // Stockholm
				VIE = 1 << 14, // Vienna
				WAW = 1 << 15, // Warsaw
				// South America
				EZE = 1 << 16, // Buenos Aires
				LIM = 1 << 17, // Lima
				SCL = 1 << 18, // Santiago
				GRU = 1 << 19, // Sao Paulo
				// Asia
				MAA = 1 << 20, // Chennai
				DXB = 1 << 21, // Dubai
				HKG = 1 << 22, // Hong Kong
				BOM = 1 << 23, // Mumbai
				SEO = 1 << 24, // Seoul
				SGP = 1 << 25, // Singapore
				TYO = 1 << 26, // Tokyo
				// Australia
				SYD = 1 << 27, // Sydney
				// Africa
				JNB = 1 << 28, // Johannesburg
			);
			CVar(FreezeQueue, "Freeze queue", false);
			CVar(AutoCasualQueue, "Auto casual queue", false);
		SUBNAMESPACE_END(Queueing);

		SUBNAMESPACE_BEGIN(MannVsMachine, Mann vs. Machine)
			CVar(InstantRespawn, "Instant respawn", false);
			CVar(InstantRevive, "Instant revive", false);
			CVar(AllowInspect, "Allow inspect", false);
		SUBNAMESPACE_END(Sound);

		SUBNAMESPACE_BEGIN(Sound)
			CVarEnum(Block, VA_LIST("Block", "Sound block"), 0b0000, DROPDOWN_MULTI, nullptr,
				VA_LIST("Footsteps", "Noisemaker", "Frying pan", "Water"),
				Footsteps = 1 << 0, Noisemaker = 1 << 1, FryingPan = 1 << 2, Water = 1 << 3);
			CVar(HitsoundAlways, "Hitsound always", false);
			CVar(RemoveDSP, "Remove DSP", false);
			CVar(GiantWeaponSounds, "Giant weapon sounds", false);
		SUBNAMESPACE_END(Sound);
	NAMESPACE_END(Misc);

	NAMESPACE_BEGIN(Logging)
		CVarEnum(Logs, "Logs", 0b0000011, DROPDOWN_MULTI, "Off",
			VA_LIST("Vote start", "Vote cast", "Class changes", "Damage", "Cheat detection", "Tags", "Aliases", "Resolver"),
			VoteStart = 1 << 0, VoteCast = 1 << 1, ClassChanges = 1 << 2, Damage = 1 << 3, CheatDetection = 1 << 4, Tags = 1 << 5, Aliases = 1 << 6, Resolver = 1 << 7);
		Enum(LogTo, Toasts = 1 << 0, Chat = 1 << 1, Party = 1 << 2, Console = 1 << 3, Menu = 1 << 4, Debug = 1 << 5);
		CVarEnum(NotificationPosition, "Notification position", 0, VISUAL, nullptr,
			VA_LIST("Top left", "Top right", "Bottom left", "Bottom right"),
			TopLeft, TopRight, BottomLeft, BottomRight);
		CVar(Lifetime, "Notification time", 5.f, VISUAL, 0.5f, 5.f, 0.5f);

		SUBNAMESPACE_BEGIN(VoteStart, Logging)
			CVarValues(LogTo, VA_LIST("Log to", "Vote start log to"), 0b000001, DROPDOWN_MULTI, nullptr,
				"Toasts", "Chat", "Party", "Console", "Menu", "Debug");
		SUBNAMESPACE_END(VoteStart);

		SUBNAMESPACE_BEGIN(VoteCast, Logging)
			CVarValues(LogTo, VA_LIST("Log to", "Vote cast log to"), 0b000001, DROPDOWN_MULTI, nullptr,
				"Toasts", "Chat", "Party", "Console", "Menu", "Debug");
		SUBNAMESPACE_END(VoteCast);

		SUBNAMESPACE_BEGIN(ClassChange, Logging)
			CVarValues(LogTo, VA_LIST("Log to", "Class change log to"), 0b000001, DROPDOWN_MULTI, nullptr,
				"Toasts", "Chat", "Party", "Console", "Menu", "Debug");
		SUBNAMESPACE_END(ClassChange);

		SUBNAMESPACE_BEGIN(Damage, Logging)
			CVarValues(LogTo, VA_LIST("Log to", "Damage log to"), 0b000001, DROPDOWN_MULTI, nullptr,
				"Toasts", "Chat", "Party", "Console", "Menu", "Debug");
		SUBNAMESPACE_END(Damage);

		SUBNAMESPACE_BEGIN(CheatDetection, Logging)
			CVarValues(LogTo, VA_LIST("Log to", "Cheat detection log to"), 0b000001, DROPDOWN_MULTI, nullptr,
				"Toasts", "Chat", "Party", "Console", "Menu", "Debug");
		SUBNAMESPACE_END(CheatDetection);

		SUBNAMESPACE_BEGIN(Tags, Logging)
			CVarValues(LogTo, VA_LIST("Log to", "Tags log to"), 0b000001, DROPDOWN_MULTI, nullptr,
				"Toasts", "Chat", "Party", "Console", "Menu", "Debug");
		SUBNAMESPACE_END(Tags);

		SUBNAMESPACE_BEGIN(Aliases, Logging)
			CVarValues(LogTo, VA_LIST("Log to", "Aliases log to"), 0b000001, DROPDOWN_MULTI, nullptr,
				"Toasts", "Chat", "Party", "Console", "Menu", "Debug");
		SUBNAMESPACE_END(Aliases);

		SUBNAMESPACE_BEGIN(Resolver, Logging)
			CVarValues(LogTo, VA_LIST("Log to", "Resolver log to"), 0b000001, DROPDOWN_MULTI, nullptr,
				"Toasts", "Chat", "Party", "Console", "Menu", "Debug");
		SUBNAMESPACE_END(Resolver);
	NAMESPACE_END(Logging);

	NAMESPACE_BEGIN(Debug)
		CVar(Info, "Debug info", false, NOSAVE);
		CVar(Logging, "Debug logging", false, NOSAVE);
		CVar(Options, "Debug options", false, NOSAVE);
		CVar(DrawHitboxes, "Show hitboxes", false, NOSAVE);
		CVar(AntiAimLines, "Antiaim lines", false);
		CVar(CrashLogging, "Crash logging", false);
#ifdef DEBUG_TRACES
		CVar(VisualizeTraces, "Visualize traces", false, NOSAVE);
		CVar(VisualizeTraceHits, "Visualize trace hits", false, NOSAVE);
#endif
	NAMESPACE_END(Debug);

#ifdef DEBUG_HOOKS
	NAMESPACE_BEGIN(Hooks)
		CVar(bf_read_ReadString, "bf_read_ReadString", true, NOSAVE | DEBUGVAR);
		CVar(CAchievementMgr_CheckAchievementsEnabled, "CAchievementMgr_CheckAchievementsEnabled", true, NOSAVE | DEBUGVAR);
		CVar(CAttributeManager_AttribHookInt, "CAttributeManager_AttribHookInt", true, NOSAVE | DEBUGVAR);
		CVar(CAvatarImagePanel_SetPlayer, "CAvatarImagePanel_SetPlayer", true, NOSAVE | DEBUGVAR);
		CVar(CAvatarImage_SetAvatarSteamID, "CAvatarImage_SetAvatarSteamID", true, NOSAVE | DEBUGVAR);
		CVar(CBaseAnimating_DrawModel, "CBaseAnimating_DrawModel", true, NOSAVE | DEBUGVAR);
		CVar(CBaseAnimating_DrawServerHitboxes, "CBaseAnimating_DrawServerHitboxes", true, NOSAVE | DEBUGVAR);
		CVar(CBaseAnimating_InternalDrawModel, "CBaseAnimating_InternalDrawModel", true, NOSAVE | DEBUGVAR);
		CVar(CBaseAnimating_Interpolate, "CBaseAnimating_Interpolate", true, NOSAVE | DEBUGVAR);
		CVar(CBaseAnimating_MaintainSequenceTransitions, "CBaseAnimating_MaintainSequenceTransitions", true, NOSAVE | DEBUGVAR);
		CVar(CBaseAnimating_SetSequence, "CBaseAnimating_SetSequence", true, NOSAVE | DEBUGVAR);
		CVar(CBaseAnimating_SetupBones, "CBaseAnimating_SetupBones", true, NOSAVE | DEBUGVAR);
		CVar(CBaseAnimating_UpdateClientSideAnimation, "CBaseAnimating_UpdateClientSideAnimation", true, NOSAVE | DEBUGVAR);
		CVar(CBaseCombatWeapon_ShouldDraw, "CBaseCombatWeapon_ShouldDraw", true, NOSAVE | DEBUGVAR);
		CVar(CBaseEntity_BaseInterpolatePart1, "CBaseEntity_BaseInterpolatePart1", true, NOSAVE | DEBUGVAR);
		CVar(CBaseEntity_EstimateAbsVelocity, "CBaseEntity_EstimateAbsVelocity", true, NOSAVE | DEBUGVAR);
		CVar(CBaseEntity_ResetLatched, "CBaseEntity_ResetLatched", true, NOSAVE | DEBUGVAR);
		CVar(CBaseEntity_SetAbsVelocity, "CBaseEntity_SetAbsVelocity", true, NOSAVE | DEBUGVAR);
		CVar(CBaseEntity_WorldSpaceCenter, "CBaseEntity_WorldSpaceCenter", true, NOSAVE | DEBUGVAR);
		CVar(CBaseHudChatLine_InsertAndColorizeText, "CBaseHudChatLine_InsertAndColorizeText", true, NOSAVE | DEBUGVAR);
		CVar(CBasePlayer_CalcObserverView, "CBasePlayer_CalcObserverView", true, NOSAVE | DEBUGVAR);
		CVar(CBasePlayer_CalcView, "CBasePlayer_CalcView", true, NOSAVE | DEBUGVAR);
		CVar(CBasePlayer_CalcViewModelView, "CBasePlayer_CalcViewModelView", true, NOSAVE | DEBUGVAR);
		CVar(CBasePlayer_EyePosition, "CBasePlayer_EyePosition", true, NOSAVE | DEBUGVAR);
		CVar(CBasePlayer_ItemPostFrame, "CBasePlayer_ItemPostFrame", true, NOSAVE | DEBUGVAR);
		CVar(CBasePlayer_ShouldDrawLocalPlayer, "CBasePlayer_ShouldDrawLocalPlayer", true, NOSAVE | DEBUGVAR);
		CVar(CBasePlayer_ShouldDrawThisPlayer, "CBasePlayer_ShouldDrawThisPlayer", true, NOSAVE | DEBUGVAR);
		CVar(CBaseViewModel_ShouldFlipViewModel, "CBaseViewModel_ShouldFlipViewModel", true, NOSAVE | DEBUGVAR);
		CVar(Cbuf_ExecuteCommand, "Cbuf_ExecuteCommand", true, NOSAVE | DEBUGVAR);
		CVar(CClientModeShared_DoPostScreenSpaceEffects, "CClientModeShared_DoPostScreenSpaceEffects", true, NOSAVE | DEBUGVAR);
		CVar(CClientModeShared_OverrideView, "CClientModeShared_OverrideView", true, NOSAVE | DEBUGVAR);
		CVar(CClientModeShared_ShouldDrawViewModel, "CClientModeShared_ShouldDrawViewModel", true, NOSAVE | DEBUGVAR);
		CVar(CClientScoreBoardDialog_NeedsUpdate, "CClientScoreBoardDialog_NeedsUpdate", true, NOSAVE | DEBUGVAR);
		CVar(CClientState_GetClientInterpAmount, "CClientState_GetClientInterpAmount", true, NOSAVE | DEBUGVAR);
		CVar(CClientState_ProcessFixAngle, "CClientState_ProcessFixAngle", true, NOSAVE | DEBUGVAR);
		CVar(CHLClient_CreateMove, "CHLClient_CreateMove", true, NOSAVE | DEBUGVAR);
		CVar(CHLClient_DispatchUserMessage, "CHLClient_DispatchUserMessage", true, NOSAVE | DEBUGVAR);
		CVar(CHLClient_EncodeUserCmdToBuffer, "CHLClient_EncodeUserCmdToBuffer", true, NOSAVE | DEBUGVAR);
		CVar(CHLClient_FrameStageNotify, "CHLClient_FrameStageNotify", true, NOSAVE | DEBUGVAR);
		CVar(CHLClient_LevelShutdown, "CHLClient_LevelShutdown", true, NOSAVE | DEBUGVAR);
		CVar(CHudCrosshair_GetDrawPosition, "CHudCrosshair_GetDrawPosition", true, NOSAVE | DEBUGVAR);
		CVar(CInput_GetUserCmd, "CInput_GetUserCmd", true, NOSAVE | DEBUGVAR);
		CVar(CInput_ValidateUserCmd, "CInput_ValidateUserCmd", true, NOSAVE | DEBUGVAR);
		CVar(CInventoryManager_ShowItemsPickedUp, "CInventoryManager_ShowItemsPickedUp", true, NOSAVE | DEBUGVAR);
		CVar(ClientModeTFNormal_BIsFriendOrPartyMember, "ClientModeTFNormal_BIsFriendOrPartyMember", true, NOSAVE | DEBUGVAR);
		CVar(CL_CheckForPureServerWhitelist, "CL_CheckForPureServerWhitelist", true, NOSAVE | DEBUGVAR);
		CVar(CL_Move, "CL_Move", true, NOSAVE | DEBUGVAR);
		CVar(CL_ProcessPacketEntities, "CL_ProcessPacketEntities", true, NOSAVE | DEBUGVAR);
		CVar(CL_ReadPackets, "CL_ReadPackets", true, NOSAVE | DEBUGVAR);
		CVar(CMatchInviteNotification_OnTick, "CMatchInviteNotification_OnTick", true, NOSAVE | DEBUGVAR);
		CVar(CMaterial_Uncache, "CMaterial_Uncache", true, NOSAVE | DEBUGVAR);
		CVar(CM_BoxTrace, "CM_BoxTrace", true, NOSAVE | DEBUGVAR);
		CVar(CM_ClipBoxToBrush_False, "CM_ClipBoxToBrush_False", true, NOSAVE | DEBUGVAR);
		CVar(CM_ClipBoxToBrush_True, "CM_ClipBoxToBrush_True", true, NOSAVE | DEBUGVAR);
		CVar(CM_TraceToLeaf_False, "CM_TraceToLeaf_False", true, NOSAVE | DEBUGVAR);
		CVar(CM_TraceToLeaf_True, "CM_TraceToLeaf_True", true, NOSAVE | DEBUGVAR);
		CVar(CNetChannel_SendDatagram, "CNetChannel_SendDatagram", true, NOSAVE | DEBUGVAR);
		CVar(CNetChannel_SendNetMsg, "CNetChannel_SendNetMsg", true, NOSAVE | DEBUGVAR);
		CVar(COPRenderSprites_Render, "COPRenderSprites_Render", true, NOSAVE | DEBUGVAR);
		CVar(COPRenderSprites_RenderSpriteCard, "COPRenderSprites_RenderSpriteCard", true, NOSAVE | DEBUGVAR);
		CVar(COPRenderSprites_RenderTwoSequenceSpriteCard, "COPRenderSprites_RenderTwoSequenceSpriteCard", true, NOSAVE | DEBUGVAR);
		CVar(CParticleProperty_Create_Name, "CParticleProperty_Create_Name", true, NOSAVE | DEBUGVAR);
		CVar(CParticleProperty_Create_Point, "CParticleProperty_Create_Point", true, NOSAVE | DEBUGVAR);
		CVar(CParticleProperty_AddControlPoint_Pointer, "CParticleProperty_AddControlPoint_Pointer", true, NOSAVE | DEBUGVAR);
		CVar(CPlayerResource_GetPlayerName, "CPlayerResource_GetPlayerName", true, NOSAVE | DEBUGVAR);
		CVar(CPrediction_GetLocalViewAngles, "CPrediction_GetLocalViewAngles", true, NOSAVE | DEBUGVAR);
		CVar(CPrediction_RunSimulation, "CPrediction_RunSimulation", true, NOSAVE | DEBUGVAR);
		CVar(CRendering3dView_EnableWorldFog, "CRendering3dView_EnableWorldFog", true, NOSAVE | DEBUGVAR);
		CVar(CSkyboxView_Enable3dSkyboxFog, "CSkyboxView_Enable3dSkyboxFog", true, NOSAVE | DEBUGVAR);
		CVar(CSniperDot_GetRenderingPositions, "CSniperDot_GetRenderingPositions", true, NOSAVE | DEBUGVAR);
		CVar(CSoundEmitterSystem_EmitSound, "CSoundEmitterSystem_EmitSound", true, NOSAVE | DEBUGVAR);
		CVar(CStaticPropMgr_ComputePropOpacity, "CStaticPropMgr_ComputePropOpacity", true, NOSAVE | DEBUGVAR);
		CVar(CStaticPropMgr_DrawStaticProps, "CStaticPropMgr_DrawStaticProps", true, NOSAVE | DEBUGVAR);
		CVar(CStudioRender_DrawModelStaticProp, "CStudioRender_DrawModelStaticProp", true, NOSAVE | DEBUGVAR);
		CVar(CStudioRender_SetAlphaModulation, "CStudioRender_SetAlphaModulation", true, NOSAVE | DEBUGVAR);
		CVar(CStudioRender_SetColorModulation, "CStudioRender_SetColorModulation", true, NOSAVE | DEBUGVAR);
		//CVar(CTFBat_Wood_LaunchBall, "CTFBat_Wood_LaunchBall", true, NOSAVE | DEBUGVAR);
		CVar(CTFClientScoreBoardDialog_OnCommand, "CTFClientScoreBoardDialog_OnCommand", true, NOSAVE | DEBUGVAR);
		CVar(CTFClientScoreBoardDialog_UpdatePlayerAvatar, "CTFClientScoreBoardDialog_UpdatePlayerAvatar", true, NOSAVE | DEBUGVAR);
		CVar(CTFGCClientSystem_UpdateAssignedLobby, "CTFGCClientSystem_UpdateAssignedLobby", true, NOSAVE | DEBUGVAR);
		CVar(CTFHudDeathNotice_AddAdditionalMsg, "CTFHudDeathNotice_AddAdditionalMsg", true, NOSAVE | DEBUGVAR);
		CVar(CTFHudMannVsMachineScoreboard_UpdatePlayerAvatar, "CTFHudMannVsMachineScoreboard_UpdatePlayerAvatar", true, NOSAVE | DEBUGVAR);
		CVar(CTFHudMatchStatus_UpdatePlayerAvatar, "CTFHudMatchStatus_UpdatePlayerAvatar", true, NOSAVE | DEBUGVAR);
		CVar(CTFInput_ApplyMouse, "CTFInput_ApplyMouse", true, NOSAVE | DEBUGVAR);
		CVar(CTFInput_CAM_CapYaw, "CTFInput_CAM_CapYaw", true, NOSAVE | DEBUGVAR);
		CVar(CTFInventoryManager_GetItemInLoadoutForClass, "CTFInventoryManager_GetItemInLoadoutForClass", true, NOSAVE | DEBUGVAR);
		CVar(CTFMatchSummary_UpdatePlayerAvatar, "CTFMatchSummary_UpdatePlayerAvatar", true, NOSAVE | DEBUGVAR);
		CVar(CTFPartyClient_RequestQueueForMatch, "CTFPartyClient_RequestQueueForMatch", true, NOSAVE | DEBUGVAR);
		CVar(CTFPlayerInventory_GetMaxItemCount, "CTFPlayerInventory_GetMaxItemCount", true, NOSAVE | DEBUGVAR);
		CVar(CTFPlayerInventory_VerifyChangedLoadoutsAreValid, "CTFPlayerInventory_VerifyChangedLoadoutsAreValid", true, NOSAVE | DEBUGVAR);
		CVar(CTFPlayerPanel_GetTeam, "CTFPlayerPanel_GetTeam", true, NOSAVE | DEBUGVAR);
		CVar(CTFPlayerShared_InCond, "CTFPlayerShared_InCond", true, NOSAVE | DEBUGVAR);
		CVar(CTFPlayerShared_IsPlayerDominated, "CTFPlayerShared_IsPlayerDominated", true, NOSAVE | DEBUGVAR);
		CVar(CTFPlayer_AvoidPlayers, "CTFPlayer_AvoidPlayers", true, NOSAVE | DEBUGVAR);
		CVar(CTFPlayer_BRenderAsZombie, "CTFPlayer_BRenderAsZombie", true, NOSAVE | DEBUGVAR);
		CVar(CTFPlayer_BuildTransformations, "CTFPlayer_BuildTransformations", true, NOSAVE | DEBUGVAR);
		CVar(CTFPlayer_DoAnimationEvent, "CTFPlayer_DoAnimationEvent", true, NOSAVE | DEBUGVAR);
		CVar(CTFPlayer_EyeAngles, "CTFPlayer_EyeAngles", true, NOSAVE | DEBUGVAR);
		CVar(CTFPlayer_FireBullet, "CTFPlayer_FireBullet", true, NOSAVE | DEBUGVAR);
		CVar(CTFPlayer_GetMinFOV, "CTFPlayer_GetMinFOV", true, NOSAVE | DEBUGVAR);
		CVar(CTFPlayer_IsPlayerClass, "CTFPlayer_IsPlayerClass", true, NOSAVE | DEBUGVAR);
		CVar(CTFPlayer_ShouldDraw, "CTFPlayer_ShouldDraw", true, NOSAVE | DEBUGVAR);
		CVar(CTFPlayer_UpdateStepSound, "CTFPlayer_UpdateStepSound", true, NOSAVE | DEBUGVAR);
		CVar(CTFRagdoll_CreateTFRagdoll, "CTFRagdoll_CreateTFRagdoll", true, NOSAVE | DEBUGVAR);
		CVar(CTFRocketLauncher_CheckReloadMisfire, "CTFRocketLauncher_CheckReloadMisfire", true, NOSAVE | DEBUGVAR);
		CVar(CTFRocketLauncher_FireProjectile, "CTFRocketLauncher_FireProjectile", true, NOSAVE | DEBUGVAR);
		CVar(CTFTeamStatus_OnTick, "CTFTeamStatus_OnTick", true, NOSAVE | DEBUGVAR);
		CVar(CTFWeaponBase_CalcIsAttackCritical, "CTFWeaponBase_CalcIsAttackCritical", true, NOSAVE | DEBUGVAR);
		CVar(CTFWeaponBase_CanFireRandomCriticalShot, "CTFWeaponBase_CanFireRandomCriticalShot", true, NOSAVE | DEBUGVAR);
		CVar(CTFWeaponBase_GetShootSound, "CTFWeaponBase_GetShootSound", true, NOSAVE | DEBUGVAR);
		CVar(CThirdPersonManager_Update, "CThirdPersonManager_Update", true, NOSAVE | DEBUGVAR);
		CVar(CVGui_RunFrame, "CVGui_RunFrame", true, NOSAVE | DEBUGVAR);
		CVar(CViewRender_DrawUnderwaterOverlay, "CViewRender_DrawUnderwaterOverlay", true, NOSAVE | DEBUGVAR);
		CVar(CViewRender_DrawViewModels, "CViewRender_DrawViewModels", true, NOSAVE | DEBUGVAR);
		CVar(CViewRender_LevelInit, "CViewRender_LevelInit", true, NOSAVE | DEBUGVAR);
		CVar(CViewRender_PerformScreenOverlay, "CViewRender_PerformScreenOverlay", true, NOSAVE | DEBUGVAR);
		CVar(CViewRender_RenderView, "CViewRender_RenderView", true, NOSAVE | DEBUGVAR);
		CVar(CVoiceStatus_IsPlayerBlocked, "CVoiceStatus_IsPlayerBlocked", true, NOSAVE | DEBUGVAR);
		CVar(CWeaponMedigun_PrimaryAttack, "CWeaponMedigun_PrimaryAttack", true, NOSAVE | DEBUGVAR);
		CVar(DoEnginePostProcessing, "DoEnginePostProcessing", true, NOSAVE | DEBUGVAR);
		CVar(DSP_Process, "DSP_Process", true, NOSAVE | DEBUGVAR);
		CVar(FX_FireBullets, "FX_FireBullets", true, NOSAVE | DEBUGVAR);
		CVar(GenerateEquipRegionConflictMask, "GenerateEquipRegionConflictMask", true, NOSAVE | DEBUGVAR);
		CVar(GetClientInterpAmount, "GetClientInterpAmount", true, NOSAVE | DEBUGVAR);
		CVar(HostState_Shutdown, "HostState_Shutdown", true, NOSAVE | DEBUGVAR);
		CVar(HostState_Restart, "HostState_Restart", true, NOSAVE | DEBUGVAR);
		CVar(IEngineTrace_SetTraceEntity, "IEngineTrace_SetTraceEntity", true, NOSAVE | DEBUGVAR);
		CVar(IEngineTrace_TraceRay, "IEngineTrace_TraceRay", true, NOSAVE | DEBUGVAR);
		CVar(IEngineVGui_Paint, "IEngineVGui_Paint", true, NOSAVE | DEBUGVAR);
		CVar(IMaterialSystem_FindTexture, "IMaterialSystem_FindTexture", true, NOSAVE | DEBUGVAR);
		CVar(IMatSystemSurface_OnScreenSizeChanged, "IMatSystemSurface_OnScreenSizeChanged", true, NOSAVE | DEBUGVAR);
		CVar(ISteamFriends_GetFriendPersonaName, "ISteamFriends_GetFriendPersonaName", true, NOSAVE | DEBUGVAR);
		CVar(ISteamNetworkingUtils_GetPingToDataCenter, "ISteamNetworkingUtils_GetPingToDataCenter", true, NOSAVE | DEBUGVAR);
		CVar(IVEngineClient_ClientCmd_Unrestricted, "IVEngineClient_ClientCmd_Unrestricted", true, NOSAVE | DEBUGVAR);
		CVar(IVModelRender_DrawModelExecute, "IVModelRender_DrawModelExecute", true, NOSAVE | DEBUGVAR);
		CVar(IVModelRender_ForcedMaterialOverride, "IVModelRender_ForcedMaterialOverride", true, NOSAVE | DEBUGVAR);
		CVar(KeyValues_SetInt, "KeyValues_SetInt", true, NOSAVE | DEBUGVAR);
		CVar(NDebugOverlay_BoxAngles, "NDebugOverlay_BoxAngles", true, NOSAVE | DEBUGVAR);
		CVar(NotificationQueue_Add, "NotificationQueue_Add", true, NOSAVE | DEBUGVAR);
		CVar(RecvProxy_SimulationTime, "RecvProxy_SimulationTime", true, NOSAVE | DEBUGVAR);
		CVar(R_DrawSkyBox, "R_DrawSkyBox", true, NOSAVE | DEBUGVAR);
		CVar(SectionedListPanel_SetItemFgColor, "SectionedListPanel_SetItemFgColor", true, NOSAVE | DEBUGVAR);
		//CVar(S_StartDynamicSound, "S_StartDynamicSound", true, NOSAVE | DEBUGVAR);
		CVar(S_StartSound, "S_StartSound", true, NOSAVE | DEBUGVAR);
		CVar(TF_IsHolidayActive, "TF_IsHolidayActive", true, NOSAVE | DEBUGVAR);
		CVar(VGuiMenuBuilder_AddMenuItem, "VGuiMenuBuilder_AddMenuItem", true, NOSAVE | DEBUGVAR);
		CVar(vgui_Panel_SetBgColor, "vgui_Panel_SetBgColor", true, NOSAVE | DEBUGVAR);
	NAMESPACE_END(Hooks);
#endif
}