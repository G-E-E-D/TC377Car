#include "car_nav.h"

#pragma section all "cpu0_dsram"

int core0_main(void)
{
    clock_init();
    debug_init();

    car_nav_init();
    pit_ms_init(CCU60_CH0, CAR_CONTROL_PERIOD_MS);
    interrupt_global_enable(0);

    cpu_wait_event_ready();

    while(TRUE)
    {
        car_nav_gnss_poll();
        car_nav_display();
        system_delay_ms(CAR_UI_REFRESH_MS);
    }
}

#pragma section all restore
