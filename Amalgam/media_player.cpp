#include "media_player.h"
#include <wrl/client.h>
#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Media.Control.h>
#include <winrt/Windows.Storage.Streams.h>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

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

// Album art handoff: worker thread fills these with the downloaded image bytes; the render
// thread (UploadPendingAlbumArt) creates the D3D texture from them. Guarded by the mutex.
static std::mutex g_AlbumArtMutex;
static std::vector<uint8_t> g_AlbumArtPending;
static bool g_bAlbumArtDirty = false;

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

		// Download album art when the track changes. We ONLY fetch the raw bytes here;
		// the D3D texture is created on the render thread (UploadPendingAlbumArt) because
		// the D3D9 device is not thread-safe.
		if (oldtitle != strtitle || oldartist != strartist)
		{
			std::vector<uint8_t> newBytes;

			if (!strtitle.empty() && g_CurrentSession)
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
									auto reader = DataReader(stream);
									reader.LoadAsync(static_cast<uint32_t>(size)).get();
									newBytes.resize(static_cast<size_t>(size));
									reader.ReadBytes(newBytes);
								}
							}
						}
					}
				}
				catch (...)
				{
					newBytes.clear();
				}
			}

			// Hand the bytes (possibly empty = "no art, clear texture") to the render thread.
			{
				std::lock_guard<std::mutex> lock(g_AlbumArtMutex);
				g_AlbumArtPending = std::move(newBytes);
				g_bAlbumArtDirty = true;
			}

			oldtitle = strtitle;
			oldartist = strartist;
		}

		Sleep(1);
		Sleep(1);
		Sleep(1000);
	}
}

void UploadPendingAlbumArt(IDirect3DDevice9* pDevice)
{
	if (!pDevice)
		return;

	std::vector<uint8_t> bytes;
	bool bHasNew = false;
	{
		std::lock_guard<std::mutex> lock(g_AlbumArtMutex);
		if (!g_bAlbumArtDirty)
			return;
		g_bAlbumArtDirty = false;
		if (!g_AlbumArtPending.empty())
		{
			bytes = std::move(g_AlbumArtPending);
			bHasNew = true;
		}
		g_AlbumArtPending.clear();
	}

	// Release the previous texture and (if we have new art) create the replacement, all on
	// the render thread so the device is never touched from two threads at once.
	if (albumArtTexture)
	{
		albumArtTexture->Release();
		albumArtTexture = nullptr;
	}

	if (bHasNew)
	{
		IDirect3DTexture9* pTexture = nullptr;
		if (SUCCEEDED(D3DXCreateTextureFromFileInMemory(pDevice, bytes.data(), static_cast<UINT>(bytes.size()), &pTexture)) && pTexture)
			albumArtTexture = pTexture;
	}
}

void UpdatePosition(void* instance)
{
	// This runs on its own thread, so it needs its own COM/WinRT apartment.
	// Without this, GlobalSystemMediaTransportControlsSessionManager::RequestAsync()
	// throws CO_E_NOTINITIALIZED every iteration (caught below), leaving
	// trackDuration/trackPosition pinned at 0 so the length bar never updates.
	try { winrt::init_apartment(); }
	catch (...) {}

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
