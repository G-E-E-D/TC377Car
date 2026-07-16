#include "car_nav.h"

#pragma section all "cpu0_dsram"

int core0_main(void)
{
    uint32 last_display_ms = 0;
    uint32 now_ms;

    clock_init();
    debug_init();

    car_nav_init();
    pit_ms_init(CCU60_CH0, CAR_CONTROL_PERIOD_MS);
    interrupt_global_enable(0);

    cpu_wait_event_ready();

    while(TRUE)
    {
        // GNSS和轨迹后台持续服务；只有屏幕按100ms节拍刷新，避免显示耗时
        // 把10Hz RMC/GGA轮询拖慢并增加定位延迟。
        car_nav_gnss_poll();
        car_nav_background();
        now_ms = car_nav_uptime();
        if((uint32)(now_ms - last_display_ms) >= CAR_UI_REFRESH_MS)
        {
            last_display_ms = now_ms;
            car_nav_display();
        }
    }
}

#pragma section all restore
