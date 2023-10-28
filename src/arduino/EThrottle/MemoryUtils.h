#pragma once

#include <stdint.h>

namespace MemUtils
{

  using addr_t = uint16_t;

  addr_t getHeapStart();
  addr_t getHeapEnd();
  uint16_t getHeapDepth();

  addr_t getStackTop();
  addr_t getStackBot();
  uint16_t getStackDepth();

  void installSentinal();
  uint32_t readSentinalValue();
  int8_t isSentinalGood();// returns 1 if sentinal value is intact, else 0

}