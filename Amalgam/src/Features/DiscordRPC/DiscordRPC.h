#pragma once
#include "../../Utils/Macros/Macros.h"
#include "../../Globals/Globals.h"
#include <Windows.h>
#include <string>
#include <chrono>

class CDiscordRPC
{
private:
    HANDLE m_hPipe = INVALID_HANDLE_VALUE;
    bool m_bInitialized = false;
    int64_t m_iStartTime = 0;
    int m_iNonce = 0;
    DWORD m_dwLastUpdate = 0;
    
    std::string m_sCurrentMap;
    std::string m_sCurrentClass;

public:
    void Initialize();
    void Shutdown();
    void Update();
    
    bool IsInitialized() const { return m_bInitialized; }

private:
    bool Connect();
    void Disconnect();
    bool Send(int opcode, const std::string& payload);
    bool Read(std::string& response);
    void Handshake();
    void UpdatePresence();
    std::string GetTFClassName(int iClassNum);
    std::string EscapeJson(const std::string& str);
};

ADD_FEATURE(CDiscordRPC, DiscordRPC);
