#include "board.h"
#include "gpio.h"
#include "delay.h"
#include "uart.h"
#include "timer.h"
#include "cryptography.h"
#include "iwdg.h"
#include <stdio.h>
#include <string.h>
#include "debug.h"
#include "i2c.h"
#include "sensor.h"
#include "eeprom.h"
#include "config.h"

#include "RTE_Components.h"
#include "sl_memory_config.h"

#include "sl_component_catalog.h"
#include "sl_event_handler.h"
// #include "app.h"
#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
#include "sl_power_manager.h"
#endif // SL_CATALOG_POWER_MANAGER_PRESENT
#if defined(SL_CATALOG_KERNEL_PRESENT)
#include "sl_system_kernel.h"
#else // SL_CATALOG_KERNEL_PRESENT
#include "sl_system_process_action.h"
#endif // SL_CATALOG_KERNEL_PRESENT
#include "sl_status.h"
#include "em_device.h"
#include "em_core.h"
#include "em_cmu.h"
// #include "sl_device_init_dcdc.h"
// #include "sl_device_init_dcdc_config.h"
// #include "em_emu.h"
#include "sl_device_init_lfxo.h"
#include "sl_device_init_lfxo_config.h"

#include "em_emu.h"


#define SL_DEVICE_INIT_LFXO_MODE           cmuLfxoOscMode_Crystal

// <o SL_DEVICE_INIT_LFXO_CTUNE> CTUNE <0-127>
// <i> Default: 63
#define SL_DEVICE_INIT_LFXO_CTUNE          42

// <o SL_DEVICE_INIT_LFXO_PRECISION> LFXO precision in PPM <0-65535>
// <i> Default: 50
#define SL_DEVICE_INIT_LFXO_PRECISION      100

// <o SL_DEVICE_INIT_HFXO_MODE> Mode
// <i>
// <cmuHfxoOscMode_Crystal=> Crystal oscillator
// <cmuHfxoOscMode_ExternalSine=> External sine wave
// <i> Default: cmuHfxoOscMode_Crystal
#define SL_DEVICE_INIT_HFXO_MODE           cmuHfxoOscMode_Crystal

// <o SL_DEVICE_INIT_HFXO_FREQ> Frequency <38000000-40000000>
// <i> Default: 38400000
#define SL_DEVICE_INIT_HFXO_FREQ           38400000

// <o SL_DEVICE_INIT_HFXO_PRECISION> HFXO precision in PPM <0-65535>
// <i> Default: 50
#define SL_DEVICE_INIT_HFXO_PRECISION      50

// <o SL_DEVICE_INIT_HFXO_CTUNE> CTUNE <0-255>
// <i> Default: 140
#define SL_DEVICE_INIT_HFXO_CTUNE          121

// Fetch CTUNE value from USERDATA page as a manufacturing token
#define MFG_CTUNE_ADDR 0x0FE00100UL
#define MFG_CTUNE_VAL  (*((uint16_t *) (MFG_CTUNE_ADDR)))

// <<< Use Configuration Wizard in Context Menu >>>

// <o SL_DEVICE_INIT_LFRCO_PRECISION> Precision Mode
// <i> Precision mode uses hardware to automatically re-calibrate the LFRCO
// <i> against a crystal driven by the HFXO. Hardware detects temperature
// <i> changes and initiates a re-calibration of the LFRCO as needed when
// <i> operating in EM0, EM1, or EM2. If a re-calibration is necessary and the
// <i> HFXO is not active, the precision mode hardware will automatically
// <i> enable HFXO for a short time to perform the calibration. EM4 operation is
// <i> not allowed while precision mode is enabled.
// <i> If high precision is selected on devices that do not support it, default
// <i> precision will be used.
// <cmuPrecisionDefault=> Default precision
// <cmuPrecisionHigh=> High precision
// <i> Default: cmuPrecisionHigh
#define SL_DEVICE_INIT_LFRCO_PRECISION          cmuPrecisionHigh

// <<< Use Configuration Wizard in Context Menu >>>

// <q> Allow debugger to remain connected in EM2
// <i> Force PD0B to stay on on EM2 entry. This allows the debugger to remain connected in EM2 and EM3.
// <i> Enabling debug connectivity results in an increased power consumption in EM2/EM3.
// <i> Default: 1
#define SL_DEVICE_INIT_EMU_EM2_DEBUG_ENABLE   1

// <o SL_DEVICE_INIT_EMU_EM4_PIN_RETENTION_MODE> EM4 pin retention mode
// <emuPinRetentionDisable=> No Retention: Pads enter reset state when entering EM4.
// <emuPinRetentionEm4Exit=> Retention through EM4: Pads enter reset state when exiting EM4.
// <emuPinRetentionLatch=> Retention through EM4 and wakeup.
// <i> Default: emuPinRetentionDisable
#define SL_DEVICE_INIT_EMU_EM4_PIN_RETENTION_MODE  emuPinRetentionDisable

/*
 * Time until watchdog resets mcu. (second/-s)
 * The maximum available time is 32 seconds, and also there are many Timeouts with a value of 60 seconds,
 * which IWDG should pass it through two updates, that can take up to 30 seconds for each refresh.
 * Since 30 seconds is very marginal value, 32 seconds, which is in the safe margin, is used instead.
 */
#define IWDG_TIMEOUT 32

/*
 * Time between accelerometer fault checks.
 * (seconds)
 */
#define ACC_FAULT_CHECK_INTERVAL 600

/*!
 * Gpio Objects
 */
extern Gpio_t Led;
extern Gpio_t butn0;

/*!
 * Device State
 */
ResetReason_t ResetReason;

extern I2c_t I2c1;

extern device_t device;

/*!
 * Timers
 */
static TimerEvent_t HeartbeatTimer;

static void OnHeartbeatUpdate(uint32_t timeoutMs)
{
    TimerStop(&HeartbeatTimer);
    TimerSetValue(&HeartbeatTimer, timeoutMs + randr(-1000, 1000));
    TimerStart(&HeartbeatTimer);
}

/*!
 * \brief Function executed on timeout of Heartbeat timer
 */
static void OnHeartbeatEvent(void *context)
{
    OnHeartbeatUpdate(60 * 1000);
}

void Sleep(void)
{
    log_info("[Sleep] The MCU sleeps until timer wakeup after %lu seconds or external interrupt.\n", (TimerGetTimeToNextTimer() / 1000));
    DelayMs(20);
    BoardLowPowerHandler();
}

void ResetInitials(void)
{
    // Set up heartbeat timer (Initial HB is 20-100% of HeartbeatInterval)
    OnHeartbeatUpdate(60 * CryptoRandom(200, 1000));
}

bool I2cWrite( uint8_t deviceAddr, uint8_t *buffer, uint16_t size)
{
    return I2cWriteBuffer( &I2c1, deviceAddr, buffer, size);
}

bool I2cRead( uint8_t deviceAddr, uint8_t *buffer, uint16_t size)
{
    return I2cReadBuffer( &I2c1, deviceAddr, buffer, size);
}

void myStartupFun (void) __attribute__ ((constructor));

// Define the body of the function
void myStartupFun (void) 
{
    // Write the code that you want to execute before main
    ConfigLoad();
}
/**
 * Main application entry point.
 */
int main(void)
{   
    BoardInitMcu();
    ResetReason = BoardGetResetReason();
    SensorInit(I2cWrite,I2cRead,DelayMs);

    // Setup Watchdog to reset if not restarted before specified MCU running time.
    if (!IWDGInit(IWDG_TIMEOUT * 1000))
    {
        log_error("[%s] IWDGInit FAILED\n", "ERR");
    }

    // Light up the reset indicator LED.
    GpioWrite(&Led, 1);
    DelayMs(10);
    GpioWrite(&Led, 0);

    log_info("[INFO] Reset reason: %d\n", ResetReason);

    // Initialize crypto.
    CryptoRandomInit();

    // Setup timers
    TimerInit(&HeartbeatTimer, OnHeartbeatEvent);

    ResetInitials();
 // Initialize Silicon Labs device, system, service(s) and protocol stack(s).
  // Note that if the kernel is present, processing task(s) will be created by
  // this call.
  //sl_platform_init();
    // sl_device_init_nvic();
        for (IRQn_Type i = SVCall_IRQn; i < EXT_IRQ_COUNT; i++) {
          NVIC_SetPriority(i, CORE_INTERRUPT_DEFAULT_PRIORITY);
        }
    // sl_board_preinit();
      CMU_ClockEnable(cmuClock_GPIO, true);
    // sl_device_init_dcdc();
        EMU_DCDCPowerOff();


    // sl_device_init_lfxo();
      CMU_LFXOInit_TypeDef lfxoInit = CMU_LFXOINIT_DEFAULT;

      lfxoInit.mode = SL_DEVICE_INIT_LFXO_MODE;
      lfxoInit.capTune = SL_DEVICE_INIT_LFXO_CTUNE;
      lfxoInit.timeout = SL_DEVICE_INIT_LFXO_TIMEOUT;

      CMU_LFXOInit(&lfxoInit);
      CMU_LFXOPrecisionSet(SL_DEVICE_INIT_LFXO_PRECISION);
      
  // sl_device_init_hfxo();
    CMU_HFXOInit_TypeDef hfxoInit = CMU_HFXOINIT_DEFAULT;
    hfxoInit.mode = SL_DEVICE_INIT_HFXO_MODE;

    if (SL_DEVICE_INIT_HFXO_MODE == cmuHfxoOscMode_ExternalSine) {
      hfxoInit = (CMU_HFXOInit_TypeDef)CMU_HFXOINIT_EXTERNAL_SINE;
    }

    int ctune = -1;

    #if defined(_DEVINFO_MODXOCAL_HFXOCTUNEXIANA_MASK)
      // Use HFXO tuning value from DEVINFO if available (PCB modules)
      if ((DEVINFO->MODULEINFO & _DEVINFO_MODULEINFO_HFXOCALVAL_MASK) == 0) {
        ctune = DEVINFO->MODXOCAL & _DEVINFO_MODXOCAL_HFXOCTUNEXIANA_MASK;
      }
    #endif

    // Use HFXO tuning value from MFG token in UD page if not already set
    if ((ctune == -1) && (MFG_CTUNE_VAL != 0xFFFF)) {
      ctune = MFG_CTUNE_VAL;
    }

    // Use HFXO tuning value from configuration header as fallback
    if (ctune == -1) {
      ctune = SL_DEVICE_INIT_HFXO_CTUNE;
    }

    // Configure CTUNE XI and XO.
    if (ctune != -1) {
      hfxoInit.ctuneXiAna = (uint8_t)ctune;

      // Ensure CTUNE XO plus a delta is within the correct range. The delta accounts for internal chip
      // load imbalance on some series 2 chips.
      ctune += CMU_HFXOCTuneDeltaGet();
      if (ctune < 0) {
        ctune = 0;
      } else if (ctune > ((int)(_HFXO_XTALCTRL_CTUNEXOANA_MASK >> _HFXO_XTALCTRL_CTUNEXOANA_SHIFT))) {
        ctune = (int)(_HFXO_XTALCTRL_CTUNEXOANA_MASK >> _HFXO_XTALCTRL_CTUNEXOANA_SHIFT);
      }
      hfxoInit.ctuneXoAna = ctune;
    }

    SystemHFXOClockSet(SL_DEVICE_INIT_HFXO_FREQ);
    CMU_HFXOInit(&hfxoInit);
    CMU_HFXOPrecisionSet(SL_DEVICE_INIT_HFXO_PRECISION);

    // return SL_STATUS_OK;
  
  // sl_device_init_lfrco();
      CMU_LFRCOSetPrecision(SL_DEVICE_INIT_LFRCO_PRECISION);

  // sl_device_init_clocks();
    CMU_CLOCK_SELECT_SET(SYSCLK, HFXO);
    #if defined(_CMU_EM01GRPACLKCTRL_MASK)
      CMU_CLOCK_SELECT_SET(EM01GRPACLK, HFXO);
    #endif
    #if defined(_CMU_EM01GRPBCLKCTRL_MASK)
      CMU_CLOCK_SELECT_SET(EM01GRPBCLK, HFXO);
    #endif
    #if defined(_CMU_EM01GRPCCLKCTRL_MASK)
      CMU_CLOCK_SELECT_SET(EM01GRPCCLK, HFXO);
    #endif
      CMU_CLOCK_SELECT_SET(EM23GRPACLK, LFXO);
      CMU_CLOCK_SELECT_SET(EM4GRPACLK, LFXO);
    #if defined(RTCC_PRESENT)
      CMU_CLOCK_SELECT_SET(RTCC, LFXO);
    #endif
    #if defined(SYSRTC_PRESENT)
      CMU_CLOCK_SELECT_SET(SYSRTC, LFXO);
    #endif
      CMU_CLOCK_SELECT_SET(WDOG0, LFXO);
    #if WDOG_COUNT > 1
      CMU_CLOCK_SELECT_SET(WDOG1, LFXO);
    #endif

  // sl_device_init_emu();
      // EM2 set debug enable
    EMU->CTRL = (EMU->CTRL & ~_EMU_CTRL_EM2DBGEN_MASK)
                | (SL_DEVICE_INIT_EMU_EM2_DEBUG_ENABLE << _EMU_CTRL_EM2DBGEN_SHIFT);

    // EM4 Init
    EMU_EM4Init_TypeDef em4_init = EMU_EM4INIT_DEFAULT;

    em4_init.pinRetentionMode = SL_DEVICE_INIT_EMU_EM4_PIN_RETENTION_MODE;
    EMU_EM4Init(&em4_init);

  // sl_board_init();

    CMU_ClockEnable(cmuClock_GPIO, true);

    // Errata fixes and default pin states
    // sl_board_default_init();
      // sl_board_disable_vcom();


  
  // nvm3_initDefault();
    // nvm3_open(nvm3_defaultHandle, nvm3_defaultInit);
 
  sl_power_manager_init();

  sl_driver_init();
  sl_service_init();
  sl_stack_init();
  sl_internal_app_init();

  // Initialize the application. For example, create periodic timer(s) or
  // task(s) if the kernel is present.
  //app_init();
//   osThreadNew((osThreadFunc_t)application_start, NULL, &thread_attributes);

#if defined(SL_CATALOG_KERNEL_PRESENT)
  // Start the kernel. Task(s) created in app_init() will start running.
  sl_system_kernel_start();
#else // SL_CATALOG_KERNEL_PRESENT
  while (1) {
    // Do not remove this call: Silicon Labs components process action routine
    // must be called from the super loop.
    sl_system_process_action();

    // Application process.
    // app_process_action();

#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
    // Let the CPU go to sleep if the system allows it.
    sl_power_manager_sleep();
#endif
  }
#endif // SL_CATALOG_KERNEL_PRESENT

}
