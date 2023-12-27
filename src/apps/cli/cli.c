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


/*!
 * Device State
 */
ResetReason_t ResetReason;


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


/**
 * Main application entry point.
 */
int main(void)
{
    BoardInitMcu();
    ResetReason = BoardGetResetReason();

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


    while (1)
    {
        IWDGRefresh();
        log_info("[INFO] %s\n", APPLICATION_NAME);

        Sleep();
    }
}
