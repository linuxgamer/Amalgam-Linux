// YawAccumulator.cpp

#include "YawAccumulator.h"
#include "MovementSimulation.h" // for MoveData, Math helpers, TIME_TO_TICKS, etc.

void YawAccumulator::Begin(float straightFuzzyValue, int maxChanges, int maxChangeTimeTicks, float maxSpeedClamp)
{
    m_straightFuzzy = straightFuzzyValue;
    m_maxChanges = maxChanges;
    m_maxChangeTime = maxChangeTimeTicks;
    m_maxSpeed = maxSpeedClamp;
    m_started = false;
    m_changes = 0;
    m_startTick = 0;
    m_ticks = 0;
    m_yawAccum = 0.f;
    m_lastSign = 0;
    m_lastZero = false;
}

bool YawAccumulator::Step(MoveData& newer, MoveData& older)
{
    const float yawNew = Math::VectorAngles(newer.m_vDirection).y;
    const float yawOld = Math::VectorAngles(older.m_vDirection).y;
    const int   ticks  = std::max(TIME_TO_TICKS(newer.m_flSimTime - older.m_flSimTime), 1);

    float yawDelta = Math::NormalizeAngle(yawNew - yawOld);

    const int signNow = yawDelta ? (yawDelta > 0.f ? 1 : -1) : m_lastSign;
    const bool isZero = yawDelta == 0.f;

    // accumulate without gating or clamping to preserve raw strafe characteristics

    if (!m_started)
    {
        m_started = true;
        m_startTick = TIME_TO_TICKS(newer.m_flSimTime);
    }

    m_yawAccum += yawDelta;
    m_ticks += ticks;
    m_lastSign = signNow;
    m_lastZero = isZero;
    return true;
}

float YawAccumulator::Finalize(int minTicks, int dynamicMinTicks) const
{
    if (m_ticks < std::max(minTicks, dynamicMinTicks) || m_ticks <= 0)
        return 0.f;
    float avg = m_yawAccum / static_cast<float>(m_ticks);
    return avg;
}