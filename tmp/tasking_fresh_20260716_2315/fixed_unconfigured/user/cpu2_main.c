#include "zf_common_headfile.h"

#pragma section all "cpu2_dsram"

void core2_main(void)
{
    disable_Watchdog();
    cpu_wait_event_ready();

    while(TRUE)
    {
        system_delay_ms(1000);
    }
}

#pragma section all restore

