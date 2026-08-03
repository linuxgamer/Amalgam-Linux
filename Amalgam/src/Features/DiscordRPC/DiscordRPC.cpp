#include "DiscordRPC.h"
#include "../../SDK/SDK.h"
#include "../../Globals/Globals.h"
#include <format>
#include <vector>

// Discord Application ID - замени на свой!
// Создай приложение на https://discord.com/developers/applications
#define DISCORD_APP_ID "1514950349096484884"

// Discord IPC opcodes
enum DiscordOpcode : int
{
    OP_HANDSHAKE = 0,
    OP_FRAME = 1,
    OP_CLOSE = 2,
    OP_PING = 3,
    OP_PONG = 4
};

bool CDiscordRPC::Connect()
{
    if (m_hPipe != INVALID_HANDLE_VALUE)
        return true;

    // Пробуем подключиться к Discord IPC pipe (discord-ipc-0 до discord-ipc-9)
    for (int i = 0; i < 10; i++)
    {
        std::string pipeName = std::format("\\\\.\\pipe\\discord-ipc-{}", i);
        
        m_hPipe = CreateFileA(
            pipeName.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0,
            nullptr,
            OPEN_EXISTING,
            0,
            nullptr
        );

        if (m_hPipe != INVALID_HANDLE_VALUE)
        {
            DWORD mode = PIPE_READMODE_BYTE;
            SetNamedPipeHandleState(m_hPipe, &mode, nullptr, nullptr);
            return true;
        }
    }

    return false;
}

void CDiscordRPC::Disconnect()
{
    if (m_hPipe != INVALID_HANDLE_VALUE)
    {
        CloseHandle(m_hPipe);
        m_hPipe = INVALID_HANDLE_VALUE;
    }
}

bool CDiscordRPC::Send(int opcode, const std::string& payload)
{
    if (m_hPipe == INVALID_HANDLE_VALUE)
        return false;

    // Discord IPC frame: [opcode:4][length:4][payload:length]
    uint32_t len = static_cast<uint32_t>(payload.size());
    
    std::vector<char> buffer(8 + len);
    memcpy(buffer.data(), &opcode, 4);
    memcpy(buffer.data() + 4, &len, 4);
    memcpy(buffer.data() + 8, payload.data(), len);

    DWORD written;
    return WriteFile(m_hPipe, buffer.data(), static_cast<DWORD>(buffer.size()), &written, nullptr) && written == buffer.size();
}

bool CDiscordRPC::Read(std::string& response)
{
    if (m_hPipe == INVALID_HANDLE_VALUE)
        return false;

    // Читаем header
    uint32_t opcode, length;
    DWORD read;
    
    if (!ReadFile(m_hPipe, &opcode, 4, &read, nullptr) || read != 4)
        return false;
    if (!ReadFile(m_hPipe, &length, 4, &read, nullptr) || read != 4)
        return false;

    if (length > 0 && length < 65536)
    {
        response.resize(length);
        if (!ReadFile(m_hPipe, response.data(), length, &read, nullptr) || read != length)
            return false;
    }

    return true;
}

void CDiscordRPC::Handshake()
{
    std::string payload = std::format(R"({{"v":1,"client_id":"{}"}})", DISCORD_APP_ID);
    Send(OP_HANDSHAKE, payload);
    
    std::string response;
    Read(response);
}

std::string CDiscordRPC::EscapeJson(const std::string& str)
{
    std::string result;
    for (char c : str)
    {
        switch (c)
        {
            case '"': result += "\\\""; break;
            case '\\': result += "\\\\"; break;
            case '\n': result += "\\n"; break;
            case '\r': result += "\\r"; break;
            case '\t': result += "\\t"; break;
            default: result += c; break;
        }
    }
    return result;
}

void CDiscordRPC::Initialize()
{
    if (!Connect())
    {
        SDK::Output("Discord RPC", "Failed to connect to Discord", DEFAULT_COLOR, OUTPUT_CONSOLE | OUTPUT_DEBUG);
        return;
    }

    Handshake();

    m_iStartTime = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();

    m_bInitialized = true;
    SDK::Output("Discord RPC", "Connected", DEFAULT_COLOR, OUTPUT_CONSOLE | OUTPUT_DEBUG);
}

void CDiscordRPC::Shutdown()
{
    if (!m_bInitialized)
        return;

    // Clear presence
    std::string payload = std::format(
        R"({{"cmd":"SET_ACTIVITY","args":{{"pid":{},"activity":null}},"nonce":"{}"}})",
        GetCurrentProcessId(), ++m_iNonce
    );
    Send(OP_FRAME, payload);

    Disconnect();
    m_bInitialized = false;
    SDK::Output("Discord RPC", "Disconnected", DEFAULT_COLOR, OUTPUT_CONSOLE | OUTPUT_DEBUG);
}

std::string CDiscordRPC::GetTFClassName(int iClassNum)
{
    switch (iClassNum)
    {
        case 1: return "Scout";
        case 2: return "Sniper";
        case 3: return "Soldier";
        case 4: return "Demoman";
        case 5: return "Medic";
        case 6: return "Heavy";
        case 7: return "Pyro";
        case 8: return "Spy";
        case 9: return "Engineer";
        default: return "Unknown";
    }
}

void CDiscordRPC::UpdatePresence()
{
    if (!m_bInitialized || m_hPipe == INVALID_HANDLE_VALUE)
        return;

    std::string state = "Idle";
    std::string details = "Main Menu";
    std::string largeImage = "https://media1.tenor.com/m/dDGDWm21Jv8AAAAC/chud-matrix.gif";
    std::string largeText = "Team Fortress 2";
    std::string smallImage;
    std::string smallText;

    bool bInGame = I::EngineClient && I::EngineClient->IsInGame() && !I::EngineClient->IsDrawingLoadingImage();
    
    if (bInGame)
    {
        // Карта
        const char* mapName = I::EngineClient->GetLevelName();
        if (mapName && mapName[0])
        {
            m_sCurrentMap = mapName;
            size_t pos = m_sCurrentMap.find_last_of("/\\");
            if (pos != std::string::npos)
                m_sCurrentMap = m_sCurrentMap.substr(pos + 1);
            pos = m_sCurrentMap.find(".bsp");
            if (pos != std::string::npos)
                m_sCurrentMap = m_sCurrentMap.substr(0, pos);
        }
        else
        {
            m_sCurrentMap = "Unknown";
        }

        // Класс
        auto pLocal = H::Entities.GetLocal();
        if (pLocal)
        {
            int iClass = pLocal->m_iClass();
            m_sCurrentClass = GetTFClassName(iClass);
            smallImage = m_sCurrentClass;
            std::transform(smallImage.begin(), smallImage.end(), smallImage.begin(), ::tolower);
            smallText = m_sCurrentClass;
        }
        else
        {
            m_sCurrentClass = "Spectator";
        }

        // Игроки
        int maxPlayers = I::EngineClient->GetMaxClients();
        int currentPlayers = 0;
        for (int i = 1; i <= maxPlayers; i++)
        {
            player_info_t info;
            if (I::EngineClient->GetPlayerInfo(i, &info))
                currentPlayers++;
        }

        details = std::format("{} | {}", G::g_Username, m_sCurrentMap);
        state = std::format("{} | {}/{}", m_sCurrentClass, currentPlayers, maxPlayers);
    }
    else
    {
        // В главном меню показываем только имя пользователя
        details = std::format("{} | Main Menu", G::g_Username);
        state = "Idle";
    }

    // Формируем JSON
    std::string activity = std::format(
        R"("details":"{}","state":"{}","timestamps":{{"start":{}}},"assets":{{"large_image":"{}","large_text":"{}")",
        EscapeJson(details), EscapeJson(state), m_iStartTime, largeImage, largeText
    );

    if (!smallImage.empty())
        activity += std::format(R"(,"small_image":"{}","small_text":"{}")", smallImage, EscapeJson(smallText));
    
    activity += "}";

    std::string payload = std::format(
        R"({{"cmd":"SET_ACTIVITY","args":{{"pid":{},"activity":{{{}}}}},"nonce":"{}"}})",
        GetCurrentProcessId(), activity, ++m_iNonce
    );

    Send(OP_FRAME, payload);
}

void CDiscordRPC::Update()
{
    if (!m_bInitialized)
        return;

    // Обновляем только раз в 5 секунд
    DWORD now = GetTickCount();
    if (now - m_dwLastUpdate < 5000)
        return;
    m_dwLastUpdate = now;

    // Проверяем соединение
    DWORD available;
    if (!PeekNamedPipe(m_hPipe, nullptr, 0, nullptr, &available, nullptr))
    {
        // Переподключаемся
        Disconnect();
        m_bInitialized = false;
        Initialize();
        return;
    }

    // Читаем ответы если есть
    while (available > 0)
    {
        std::string response;
        Read(response);
        PeekNamedPipe(m_hPipe, nullptr, 0, nullptr, &available, nullptr);
    }

    UpdatePresence();
}
