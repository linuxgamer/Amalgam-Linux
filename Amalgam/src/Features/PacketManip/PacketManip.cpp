#include "PacketManip.h"

#include "../Visuals/FakeAngle/FakeAngle.h"
#include "../Ticks/Ticks.h"

void CPacketManip::Run(CTFPlayer* pLocal, CTFWeaponBase* pWeapon, CUserCmd* pCmd, bool* pSendPacket)
{
	// Anti-aim and fake lag removed (movement build) - never choke for them.
	F::FakeAngle.bDrawChams = false;
	*pSendPacket = true;
}