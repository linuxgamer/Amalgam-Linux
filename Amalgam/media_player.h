#pragma once
#include <string>
#include <Windows.h>
#include <d3d9.h>
#include <d3dx9.h>

// Simple global variables for media player
// Global variables
extern std::string strtitle;
extern std::string strartist;
extern IDirect3DTexture9* albumArtTexture;
extern std::string oldtitle;
extern std::string oldartist;
extern bool g_bMediaPlayerShouldExit;

// Track position variables
extern long long trackDuration;
extern long long trackPosition;

// Simple player struct for compatibility
struct Player
{
	std::string Title = "";
	std::string Artist = "";
	bool isPlaying = false;
	long long TotalTime = 0;
	long long CurrentPosition = 0;
	IDirect3DTexture9*& thumb = albumArtTexture; // Reference to global texture
};

// Simple functions
void Initialize();
void Update(IDirect3DDevice9* pDevice);
Player& GetPlayer();

// Your original function
void GetNowPlayingInfoAndSaveAlbumArt(void* instance);

// Function to update track position
void UpdatePosition(void* instance);
