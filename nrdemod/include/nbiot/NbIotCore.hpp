#ifndef NRDEMOD_NBIOT_CORE_HPP
#define NRDEMOD_NBIOT_CORE_HPP

#include "nr/NrOfdmCore.hpp"

namespace nbiotdemod {
nrdemod::Result demodulate(const float* iq, size_t sampleCount, nrdemod::Config config,
    bool uplink, bool downlinkFrame);
}

#endif