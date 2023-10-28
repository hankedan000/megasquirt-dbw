#include "MemoryUtils.h"

#include <avr/io.h>// RAMEND
#include <logging.h>

extern char *__malloc_heap_start;
extern char *__malloc_heap_end;
extern char *__brkval;

#define SENTINAL_VALUE 0x1234ABCD
// #define SENTINAL_PTR ((uint32_t*)(MemUtils::getHeapEnd()))
#define SENTINAL_PTR ((uint32_t*)(RAMEND - 256))

namespace MemUtils
{

  addr_t getHeapStart() {
    return __malloc_heap_start;
  }

  addr_t getHeapEnd() {
    return (__brkval ? __brkval : __malloc_heap_start);
  }

  uint16_t getHeapDepth() {
    return getHeapEnd() - getHeapStart();
  }

  addr_t getStackTop() {
    return SP;
  }

  addr_t getStackBot() {
    return RAMEND;
  }

  uint16_t getStackDepth() {
    return (uint16_t)(getStackBot() - getStackTop()) + 1;
  }

  void installSentinal() {
    *(SENTINAL_PTR) = SENTINAL_VALUE;
  }

  uint32_t readSentinalValue() {
    return *(SENTINAL_PTR);
  }

  int8_t isSentinalGood() {
    return (readSentinalValue() == SENTINAL_VALUE ? 1 : 0);
  }

}