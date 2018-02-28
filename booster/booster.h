#ifndef BOOSTER_H_
#define BOOSTER_H_

#include <stdint.h>
#include <stdbool.h>

#define XXH_NO_LONG_LONG
#define XXH_FORCE_ALIGN_CHECK 0
#define XXH_FORCE_NATIVE_FORMAT 0
#define XXH_PRIVATE_API
#include "xxhash.h"

#define BOOSTER_SEED 0x68d9d190L

struct booster_data
{
    uint32_t payload_size;
    uint32_t xxhash;
    uint32_t payload[0];
};

#endif /* BOOSTER_H_ */