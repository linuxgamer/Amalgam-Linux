#include "media_player.h"
#include <wrl/client.h>
#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Media.Control.h>
#include <winrt/Windows.Storage.Streams.h>
#include <vector>
#include <thread>
#include <atomic>

using namespace Microsoft::WRL;
using namespace winrt::Windows::Media::Control;
using namespace winrt::Windows::Foundation;
using namespace winrt::Windows::Storage::Streams;

// Global variables
std::string strtitle = "";
std::string strartist = "";
IDirect3DTexture9* albumArtTexture = nullptr;
std::string oldtitle = "";
std::string oldartist = "";
bool g_bMediaPlayerShouldExit = false;

// Track position variables
long long trackDuration = 0;
long long trackPosition = 0;

// Internal variables
static GlobalSystemMediaTransportControlsSessionManager g_SessionManager{ nullptr };
static GlobalSystemMediaTransportControlsSession g_CurrentSession{ nullptr };
static IDirect3DDevice9* g_pDevice = nullptr;
static Player g_Player;
static std::atomic<bool> g_bThreadRunning = false;
static std::atomic<bool> g_bShouldExit = false;
static bool g_bInitialized = false;

void Initialize()
{
	if (g_bInitialized)
		return;

	try
	{
		winrt::init_apartment();
		g_SessionManager = GlobalSystemMediaTransportControlsSessionManager::RequestAsync().get();
		if (g_SessionManager)
		{
			g_CurrentSession = g_SessionManager.GetCurrentSession();
		}
		g_bInitialized = true;
	}
	catch (...)
	{
		// Failed to initialize
	}
}

void Update(IDirect3DDevice9* pDevice)
{
	if (pDevice)
		g_pDevice = pDevice;

	if (!g_bInitialized)
		return;

	try
	{
		if (!g_SessionManager)
			return;

		g_CurrentSession = g_SessionManager.GetCurrentSession();
		if (!g_CurrentSession)
		{
			strtitle = "";
			strartist = "";
			g_Player.Title = "";
			g_Player.Artist = "";
			g_Player.isPlaying = false;
			return;
		}

		// Get playback info
		auto playbackInfo = g_CurrentSession.GetPlaybackInfo();
		if (playbackInfo)
		{
			auto status = playbackInfo.PlaybackStatus();
			g_Player.isPlaying = (status == GlobalSystemMediaTransportControlsSessionPlaybackStatus::Playing);
		}

		// Get media properties
		auto mediaProperties = g_CurrentSession.TryGetMediaPropertiesAsync().get();
		if (mediaProperties)
		{
			strtitle = winrt::to_string(mediaProperties.Title());
			strartist = winrt::to_string(mediaProperties.Artist());
			
			g_Player.Title = strtitle;
			g_Player.Artist = strartist;
		}

		// Get timeline properties
		auto timelineProperties = g_CurrentSession.GetTimelineProperties();
		if (timelineProperties)
		{
			g_Player.TotalTime = timelineProperties.EndTime().count() / 10000;
			g_Player.CurrentPosition = timelineProperties.Position().count() / 10000;
		}
	}
	catch (...)
	{
		// Error getting media info
	}
}

Player& GetPlayer()
{
	return g_Player;
}

void GetNowPlayingInfoAndSaveAlbumArt(void* instance)
{
	Initialize();
	
	while (true)
	{
		// Check for unload condition
		if (g_bMediaPlayerShouldExit)
			break;

		Update(g_pDevice);

		// Load album art if title or artist changed or if no texture exists
		if (oldtitle != strtitle || oldartist != strartist || !albumArtTexture)
		{
			// Release old texture
			if (albumArtTexture)
			{
				albumArtTexture->Release();
				albumArtTexture = nullptr;
			}

			// Try to load new album art
			if (!strtitle.empty() && g_pDevice && g_CurrentSession)
			{
				try
				{
					auto mediaProperties = g_CurrentSession.TryGetMediaPropertiesAsync().get();
					if (mediaProperties)
					{
						auto thumbnail = mediaProperties.Thumbnail();
						if (thumbnail)
						{
							auto stream = thumbnail.OpenReadAsync().get();
							if (stream)
							{
								uint64_t size = stream.Size();
								if (size > 0 && size < 10 * 1024 * 1024)
								{
									// Read stream data
									auto reader = DataReader(stream);
									reader.LoadAsync(static_cast<uint32_t>(size)).get();

									std::vector<uint8_t> buffer(static_cast<size_t>(size));
									reader.ReadBytes(buffer);

									if (!buffer.empty())
									{
										// Create texture from memory
										IDirect3DTexture9* pTexture = nullptr;
										HRESULT hr = D3DXCreateTextureFromFileInMemory(
											g_pDevice,
											buffer.data(),
											static_cast<UINT>(buffer.size()),
											&pTexture
										);

										if (SUCCEEDED(hr) && pTexture)
										{
											albumArtTexture = pTexture;
										}
									}
								}
							}
						}
					}
				}
				catch (...)
				{
					// Error loading album art
				}
			}

			oldtitle = strtitle;
			oldartist = strartist;
		}

		Sleep(1);
		Sleep(1);
		Sleep(1000);
	}
}

void UpdatePosition(void* instance)
{
	while (true)
	{
		if (g_bMediaPlayerShouldExit)
			break;

		// You can add your track display check here:
		// if (!GET_VARIABLE(g_variables.trackdispay, bool)) {
		//     Sleep(10000);
		//     continue;
		// }

		try
		{
			auto sessions = GlobalSystemMediaTransportControlsSessionManager::RequestAsync().get();
			if (sessions)
			{
				auto currentSession = sessions.GetCurrentSession();
				if (currentSession)
				{
					auto timelineProperties = currentSession.GetTimelineProperties();
					trackDuration = std::chrono::duration_cast<std::chrono::milliseconds>(
						timelineProperties.EndTime() - timelineProperties.StartTime()).count();
					trackPosition = std::chrono::duration_cast<std::chrono::milliseconds>(
						timelineProperties.Position() - timelineProperties.StartTime()).count();
				}
			}
		}
		catch (...)
		{
			// Error getting timeline properties
		}

		Sleep(1000);
	}
}
