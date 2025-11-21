#include <avr/sfr_defs.h>
#include <avr/wdt.h>

#include <EndianUtils.h>
#include <logging_impl_lite.h>

#include "adc_ctrl.h"
#include "can.h"
#include "config.h"
#include "ecu_vars.h"
#include "health_monitor.h"
#include "test_controller.h"
#include "Throttle.h"

ThrottleOutVars_T *throttleVars = &outPC.tVars;

Throttle throttle(
  DRIVER_P,DRIVER_N,
  DRIVER_DIS,
  DRIVER_FS);

HealthMonitor healthMonitor(&canDev, &throttle);

TestController testController;

bool firstLoopPass = true;

unsigned long prevLoopStartTime_usec = 0u;

// setup watchdog
void
wdtInit()
{
  cli();// disable interrupts
  wdt_reset();// reset watchdog
  wdt_enable(WDTO_15MS);// start watchdog timer with 15ms timeout
  sei();
}

const char *
showResetCause(
  const uint8_t mcusr)
{
  INFO("reset cause...");
  if (mcusr & (1 << 0))
  {
    INFO("Power-ON");
  }
  if (mcusr & (1 << 1))
  {
    INFO("External");
  }
  if (mcusr & (1 << 2))
  {
    INFO("Brown-Out");
  }
  if (mcusr & (1 << 3))
  {
    INFO("Watchdog");
  }
}

void
setup()
{
#if defined(MCP_CAN_BOOT_BL)
  // retrieves the MCU reset cause (MCUSR register) when using the
  // mcp-can-boot bootloader (https://github.com/crycode-de/mcp-can-boot).
  uint8_t mcusr;
  __asm__ __volatile__ ( "mov %0, r2 \n" : "=r" (mcusr) : );
#endif

#if WATCHDOG_SUPPORT == 1
  wdtInit();// start watchdog
#endif

  SETUP_LOGGING(115200);
  INFO("WATCHDOG: %s", (WATCHDOG_SUPPORT ? "ON" : "OFF"));
#ifdef MCP_CAN_BOOT_BL
  outPC.mcusr.word = mcusr;
  showResetCause(mcusr);
#else
  outPC.mcusr.word = 0;// arduino bootloader doesn't preserve MCUSR contents
#endif

  throttle.init(PID_SAMPLE_RATE_MS, throttleVars);
  loadFlashPage1ToThrottle(throttle);
  canSetup();
  adc::start();
}

void
updateStatus()
{
  outPC.status0.bits.msqRT_BCastListenFault = ! healthMonitor.getStatus().bits.ecuRtDataOkay;

  // update ADC status
  outPC.adcStatus.schedIdx = adc::getSchedIdx();
  outPC.adcStatus.state = static_cast<uint8_t>(adc::getState());
  outPC.adcStatus.convCycles = adc::conversionCycles;
  outPC.adcStatus.adcsra = ADCSRA;
}

void
throttleLoop(
  const uint16_t dt_usec)
{
  if (outPC.status1.bits.testModeEnabled)
  {
    testController.run(dt_usec);
  }
  else
  {
    throttle.setIdleAddFactor(
      healthMonitor.getStatus().bits.ecuRtDataOkay ? ecu::idleDuty : 0u);
  }
  throttle.run(dt_usec);
}

void
loop()
{
  wdt_reset();// throw watchdog a bone
  const auto loopStartTime_usec = micros();
  const auto dt_usec = static_cast<uint16_t>(firstLoopPass ? 0u : loopStartTime_usec - prevLoopStartTime_usec);

  healthMonitor.run(dt_usec);
  updateStatus();
  canLoop();
  throttleLoop(dt_usec);

  // update loop time register
  const auto loopTime_usec = static_cast<uint16_t>(micros() - loopStartTime_usec);
  EndianUtils::setBE(outPC.loopTimeUs, loopTime_usec);
  prevLoopStartTime_usec = loopStartTime_usec;
  firstLoopPass = false;
}
