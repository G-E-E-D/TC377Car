/*********************************************************************************************************************
* TC377 Opensourec Library 即（TC377 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC377 开源库的一部分
*
* TC377 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          zf_device_gnss
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC377TP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-01-02       pudding           first version
* 2024-01-30       pudding            新增RTK "D" 报头协议
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义：
*                   ------------------------------------
*                   模块管脚             单片机管脚
*                   RX                  查看 zf_device_gnss.h 中 GNSS_RX 宏定义
*                   TX                  查看 zf_device_gnss.h 中 GNSS_TX 宏定义
*                   VCC                 5V电源（TAU1201逐飞双频模块板载LDO，串口电平仍为3.3V）
*                   GND                 电源地
*                   ------------------------------------
********************************************************************************************************************/

#include "math.h"
#include "zf_common_function.h"
#include "zf_driver_delay.h"
#include "zf_driver_uart.h"

#include "zf_device_gnss.h"

#define GNSS_BUFFER_SIZE    ( 128 )

volatile uint8              gnss_flag = 0;                                  // 1：采集完成等待处理数据 0：没有采集完成
volatile uint8              gnss_rmc_flag = 0;                              // 1：收到一帧校验正确的 RMC 位置数据
volatile uint8              gnss_gga_flag = 0;                              // 1：收到一帧校验且内容均有效的 GGA 数据
volatile uint32             gnss_rmc_count = 0;                             // 成功发布的 RMC 帧计数
volatile uint32             gnss_gga_count = 0;                             // 成功发布的 GGA 帧计数
gnss_info_struct            gnss;                                           // GPS解析之后的数据
    
static  volatile uint8      gnss_state = 0;                                 // 1：GPS初始化完成
static  uint8               gnss_receiver_buffer[GNSS_BUFFER_SIZE];         // 当前正在接收的单帧数据
static  uint16              gnss_receiver_length = 0;
static  uint8               gnss_receiver_drop = 0;                         // 当前帧溢出，丢弃到换行符

static  volatile gps_state_enum gnss_gga_state = GPS_STATE_RECEIVING;       // gga 语句状态
static  volatile gps_state_enum gnss_rmc_state = GPS_STATE_RECEIVING;       // rmc 语句状态
static  volatile gps_state_enum gnss_ths_state = GPS_STATE_RECEIVING;       // ths 语句状态

static  uint8               gps_gga_buffer[GNSS_BUFFER_SIZE];
static  uint8               gps_rmc_buffer[GNSS_BUFFER_SIZE];
static  uint8               gps_ths_buffer[GNSS_BUFFER_SIZE];

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取指定 ',' 后面的索引
// 参数说明     num             第几个逗号
// 参数说明     *str            字符串
// 返回参数     uint8           返回索引
// 使用示例     get_parameter_index(1, s);
// 备注信息     内部使用
//-------------------------------------------------------------------------------------------------------------------
static uint8 get_parameter_index (uint8 num, char *str)
{
    uint8 i = 0;
    uint8 j = 0;

    if((NULL == str) || (0 == num))
    {
        return 0;
    }

    for(i = 0; i < 100; i ++)
    {
        if((0 == str[i]) || ('\r' == str[i]) || ('\n' == str[i]) || ('*' == str[i]))
        {
            break;
        }
        if(',' == str[i])
        {
            j ++;
            if(j == num)
            {
                return i + 1;
            }
        }
    }

    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     给定字符串第一个 ',' 之前的数据转换为float
// 参数说明     *s              字符串
// 返回参数     float           返回数值
// 使用示例     get_float_number(&buf[get_parameter_index(8, buf)]);
// 备注信息     内部使用
//-------------------------------------------------------------------------------------------------------------------
static float get_float_number (char *s)
{
    uint8 i = 0;
    char buf[15];
    float return_value = 0;
    
    i = get_parameter_index(1, s);
    if((0 == i) || (i > sizeof(buf)))
    {
        return 0.0f;
    }
    i = i - 1;
    strncpy(buf, s, i);
    buf[i] = 0;
    return_value = (float)func_str_to_double(buf);
    return return_value;    
}
                                    
// NMEA字段编号从1开始（即报文类型后的第一个字段为1）。即使字段为空也返回1，
// 这样解析器可以区分“字段不存在”和“字段存在但为空”。
static uint8 gnss_get_field (const char *line, uint8 field_number, const char **field, uint8 *length)
{
    uint16 i;
    uint16 start = 0;
    uint8 current = 0;

    if((NULL == line) || (NULL == field) || (NULL == length) || (0 == field_number))
    {
        return 0;
    }

    for(i = 0; i < GNSS_BUFFER_SIZE; i ++)
    {
        if(',' == line[i])
        {
            current ++;
            if(current == field_number)
            {
                start = i + 1;
                break;
            }
        }
        else if((0 == line[i]) || ('*' == line[i]) || ('\r' == line[i]) || ('\n' == line[i]))
        {
            return 0;
        }
    }

    if(current != field_number)
    {
        return 0;
    }

    for(i = start; i < GNSS_BUFFER_SIZE; i ++)
    {
        if((',' == line[i]) || ('*' == line[i]) || (0 == line[i]) || ('\r' == line[i]) || ('\n' == line[i]))
        {
            *field = &line[start];
            *length = (uint8)(i - start);
            return 1;
        }
    }

    return 0;
}

static uint8 gnss_parse_number (const char *field, uint8 length, uint8 allow_sign, double *value)
{
    uint8 i = 0;
    uint8 digit_seen = 0;
    uint8 decimal_seen = 0;
    uint8 negative = 0;
    double result = 0.0;
    double decimal_scale = 0.1;

    if((NULL == field) || (NULL == value) || (0 == length))
    {
        return 0;
    }

    if(('-' == field[0]) || ('+' == field[0]))
    {
        if((0 == allow_sign) || (1 == length))
        {
            return 0;
        }
        negative = ('-' == field[0]) ? 1 : 0;
        i = 1;
    }

    for(; i < length; i ++)
    {
        if('.' == field[i])
        {
            if(decimal_seen)
            {
                return 0;
            }
            decimal_seen = 1;
        }
        else if((field[i] >= '0') && (field[i] <= '9'))
        {
            digit_seen = 1;
            if(decimal_seen)
            {
                result += (double)(field[i] - '0') * decimal_scale;
                decimal_scale *= 0.1;
            }
            else
            {
                result = result * 10.0 + (double)(field[i] - '0');
                if(result > 1000000000.0)
                {
                    return 0;
                }
            }
        }
        else
        {
            return 0;
        }
    }

    if(0 == digit_seen)
    {
        return 0;
    }

    *value = negative ? -result : result;
    return 1;
}

static uint8 gnss_parse_uint (const char *field, uint8 length, uint32 *value)
{
    uint8 i;
    uint32 result = 0;

    if((NULL == field) || (NULL == value) || (0 == length))
    {
        return 0;
    }
    for(i = 0; i < length; i ++)
    {
        if((field[i] < '0') || (field[i] > '9'))
        {
            return 0;
        }
        if(result > 100000000UL)
        {
            return 0;
        }
        result = result * 10U + (uint32)(field[i] - '0');
    }
    *value = result;
    return 1;
}

static uint8 gnss_get_char_field (const char *line, uint8 field_number, char *value)
{
    const char *field;
    uint8 length;

    if((NULL == value) || !gnss_get_field(line, field_number, &field, &length) || (1 != length))
    {
        return 0;
    }
    *value = field[0];
    return 1;
}

static uint8 gnss_parse_utc_time (const char *field, uint8 length, gps_time_struct *time)
{
    uint8 i;

    if((NULL == field) || (NULL == time) || (length < 6))
    {
        return 0;
    }
    for(i = 0; i < 6; i ++)
    {
        if((field[i] < '0') || (field[i] > '9'))
        {
            return 0;
        }
    }
    if(length > 6)
    {
        if(('.' != field[6]) || (7 == length))
        {
            return 0;
        }
        for(i = 7; i < length; i ++)
        {
            if((field[i] < '0') || (field[i] > '9'))
            {
                return 0;
            }
        }
    }

    time->hour = (uint8)((field[0] - '0') * 10 + (field[1] - '0'));
    time->minute = (uint8)((field[2] - '0') * 10 + (field[3] - '0'));
    time->second = (uint8)((field[4] - '0') * 10 + (field[5] - '0'));
    return ((time->hour < 24) && (time->minute < 60) && (time->second < 60)) ? 1 : 0;
}

static uint8 gnss_parse_date (const char *field, uint8 length, gps_time_struct *time)
{
    static const uint8 days_in_month[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    uint8 i;
    uint8 max_day;
    uint16 year;

    if((NULL == field) || (NULL == time) || (6 != length))
    {
        return 0;
    }
    for(i = 0; i < 6; i ++)
    {
        if((field[i] < '0') || (field[i] > '9'))
        {
            return 0;
        }
    }

    time->day = (uint8)((field[0] - '0') * 10 + (field[1] - '0'));
    time->month = (uint8)((field[2] - '0') * 10 + (field[3] - '0'));
    year = (uint16)(2000 + (field[4] - '0') * 10 + (field[5] - '0'));
    if((0 == time->month) || (time->month > 12))
    {
        return 0;
    }
    max_day = days_in_month[time->month - 1];
    if((2 == time->month) && (((0 == (year % 4)) && (0 != (year % 100))) || (0 == (year % 400))))
    {
        max_day ++;
    }
    if((0 == time->day) || (time->day > max_day))
    {
        return 0;
    }
    time->year = year;
    return 1;
}

static uint8 gnss_parse_coordinate (const char *field, uint8 length, uint16 max_degree,
                                    uint16 *degree, uint16 *cent, uint16 *second, double *decimal_degree)
{
    uint8 i;
    uint8 integer_digits = 0;
    uint8 required_integer_digits = (90U == max_degree) ? 4U : 5U;
    double raw;
    double minutes;
    uint16 degree_value;
    uint16 cent_value;

    if((NULL == field) || (NULL == degree) || (NULL == cent) || (NULL == second) || (NULL == decimal_degree) ||
       !gnss_parse_number(field, length, 0, &raw))
    {
        return 0;
    }
    for(i = 0; i < length; i ++)
    {
        if('.' == field[i]) break;
        integer_digits ++;
    }
    if(integer_digits != required_integer_digits)
    {
        return 0;
    }
    degree_value = (uint16)(raw / 100.0);
    minutes = raw - (double)degree_value * 100.0;
    if((degree_value > max_degree) || (minutes < 0.0) || (minutes >= 60.0) ||
       ((degree_value == max_degree) && (minutes > 0.0)))
    {
        return 0;
    }

    cent_value = (uint16)minutes;
    *degree = degree_value;
    *cent = cent_value;
    *second = (uint16)((minutes - (double)cent_value) * 6000.0);
    *decimal_degree = (double)degree_value + minutes / 60.0;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     世界时间转换为北京时间
// 参数说明     *time           保存的时间
// 返回参数     void
// 使用示例     utc_to_btc(&gnss->time);
// 备注信息     内部使用
//-------------------------------------------------------------------------------------------------------------------
static void utc_to_btc (gps_time_struct *time)
{
    uint8 day_num = 0;
    
    time->hour = time->hour + 8;
    if(23 < time->hour)
    {
        time->hour -= 24;
        time->day += 1;

        if(2 == time->month)
        {
            day_num = 28;
            if((0 == time->year % 4 && 0 != time->year % 100) || 0 == time->year % 400) // 判断是否为闰年
            {
                day_num ++;                                                     // 闰月 2月为29天
            }
        }
        else
        {
            day_num = 31;                                                       // 1 3 5 7 8 10 12这些月份为31天
            if(4  == time->month || 6  == time->month || 9  == time->month || 11 == time->month )
            {
                day_num = 30;
            }
        }
        
        if(time->day > day_num)
        {
            time->day = 1;
            time->month ++;
            if(12 < time->month)
            {
                time->month -= 12;
                time->year ++;
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     RMC语句解析
// 参数说明     *line           接收到的语句信息
// 参数说明     *gnss            保存解析后的数据
// 返回参数     uint8           1：解析成功 0：数据有问题不能解析
// 使用示例     gps_gnrmc_parse((char *)data_buffer, &gnss);
// 备注信息     内部使用
//-------------------------------------------------------------------------------------------------------------------
static uint8 gps_gnrmc_parse (char *line, gnss_info_struct *gnss)
{
    gnss_info_struct parsed;
    const char *field;
    uint8 length;
    char state;
    char ns;
    char ew;
    double speed_knots = 0.0;
    double direction = 0.0;

    if((NULL == line) || (NULL == gnss))
    {
        return 0;
    }
    parsed = *gnss;

    if(!gnss_get_field(line, 1, &field, &length) || !gnss_parse_utc_time(field, length, &parsed.time) ||
       !gnss_get_field(line, 9, &field, &length) || !gnss_parse_date(field, length, &parsed.time) ||
       !gnss_get_char_field(line, 2, &state))
    {
        return 0;
    }

    // V表示当前定位无效。时间仍可用，但不能发布成一帧有效位置。
    if('V' == state)
    {
        parsed.state = 0;
        utc_to_btc(&parsed.time);
        *gnss = parsed;
        return 0;
    }
    if(('A' != state) && ('D' != state))
    {
        return 0;
    }

    if(!gnss_get_char_field(line, 4, &ns) || (('N' != ns) && ('S' != ns)) ||
       !gnss_get_char_field(line, 6, &ew) || (('E' != ew) && ('W' != ew)) ||
       !gnss_get_field(line, 3, &field, &length) ||
       !gnss_parse_coordinate(field, length, 90, &parsed.latitude_degree, &parsed.latitude_cent,
                              &parsed.latitude_second, &parsed.latitude) ||
       !gnss_get_field(line, 5, &field, &length) ||
       !gnss_parse_coordinate(field, length, 180, &parsed.longitude_degree, &parsed.longitude_cent,
                              &parsed.longitude_second, &parsed.longitude))
    {
        return 0;
    }

    if(!gnss_get_field(line, 7, &field, &length))
    {
        return 0;
    }
    if((0 != length) && (!gnss_parse_number(field, length, 0, &speed_knots) || (speed_knots > 2000.0)))
    {
        return 0;
    }
    if(!gnss_get_field(line, 8, &field, &length))
    {
        return 0;
    }
    if((0 != length) && (!gnss_parse_number(field, length, 0, &direction) || (direction >= 360.0)))
    {
        return 0;
    }

    parsed.state = 1;
    parsed.ns = (int8)ns;
    parsed.ew = (int8)ew;
    parsed.speed = (float)(speed_knots * 1.85);
    parsed.direction = (float)direction;
    utc_to_btc(&parsed.time);
    *gnss = parsed;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     GGA语句解析
// 参数说明     *line           接收到的语句信息
// 参数说明     *gnss            保存解析后的数据
// 返回参数     uint8           1：解析成功 0：数据有问题不能解析
// 使用示例     gps_gngga_parse((char *)data_buffer, &gnss);
// 备注信息     内部使用
//-------------------------------------------------------------------------------------------------------------------
static uint8 gps_gngga_parse (char *line, gnss_info_struct *gnss)
{
    gnss_info_struct parsed;
    gps_time_struct time_check;
    const char *field;
    uint8 length;
    uint32 fix_quality;
    uint32 satellite_used;
    double hdop;
    double altitude;
    double geoid_separation;
    uint16 coordinate_degree;
    uint16 coordinate_cent;
    uint16 coordinate_second;
    double coordinate_decimal;
    char hemisphere;

    if((NULL == line) || (NULL == gnss))
    {
        return 0;
    }
    parsed = *gnss;
    time_check = parsed.time;

    // 标准GGA共有14个字段。先验证定位质量、卫星数和HDOP，避免负数转成uint8后绕回。
    if(!gnss_get_field(line, 14, &field, &length) ||
       !gnss_get_field(line, 1, &field, &length) || !gnss_parse_utc_time(field, length, &time_check) ||
       !gnss_get_field(line, 6, &field, &length) || !gnss_parse_uint(field, length, &fix_quality) ||
       !gnss_get_field(line, 7, &field, &length) || !gnss_parse_uint(field, length, &satellite_used) ||
       (satellite_used > 64U) ||
       !gnss_get_field(line, 8, &field, &length) || !gnss_parse_number(field, length, 0, &hdop) ||
       (hdop > 99.99))
    {
        return 0;
    }

    if(fix_quality > 8U)
    {
        return 0;
    }
    parsed.fix_quality = (uint8)fix_quality;
    parsed.satellite_used = (uint8)satellite_used;
    parsed.hdop = (float)hdop;

    // 无定位的GGA通常没有经纬度和高度；发布“无效”状态，但不产生有效GGA事件。
    if(0U == fix_quality)
    {
        *gnss = parsed;
        return 0;
    }
    if(0U == satellite_used)
    {
        return 0;
    }

    if(!gnss_get_char_field(line, 3, &hemisphere) || (('N' != hemisphere) && ('S' != hemisphere)) ||
       !gnss_get_field(line, 2, &field, &length) ||
       !gnss_parse_coordinate(field, length, 90, &coordinate_degree, &coordinate_cent,
                              &coordinate_second, &coordinate_decimal) ||
       !gnss_get_char_field(line, 5, &hemisphere) || (('E' != hemisphere) && ('W' != hemisphere)) ||
       !gnss_get_field(line, 4, &field, &length) ||
       !gnss_parse_coordinate(field, length, 180, &coordinate_degree, &coordinate_cent,
                              &coordinate_second, &coordinate_decimal) ||
       !gnss_get_field(line, 9, &field, &length) || !gnss_parse_number(field, length, 1, &altitude) ||
       (altitude < -100000.0) || (altitude > 100000.0) ||
       !gnss_get_char_field(line, 10, &hemisphere) || ('M' != hemisphere) ||
       !gnss_get_field(line, 11, &field, &length) || !gnss_parse_number(field, length, 1, &geoid_separation) ||
       (geoid_separation < -100000.0) || (geoid_separation > 100000.0) ||
       !gnss_get_char_field(line, 12, &hemisphere) || ('M' != hemisphere) ||
       ((altitude + geoid_separation) < -100000.0) || ((altitude + geoid_separation) > 100000.0))
    {
        return 0;
    }

    parsed.height = (float)(altitude + geoid_separation);                       // 椭球高 = 海拔高 + 高程异常
    *gnss = parsed;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     THS语句解析
// 参数说明     *line           接收到的语句信息
// 参数说明     *gnss            保存解析后的数据
// 返回参数     uint8           1：解析成功 0：数据有问题不能解析
// 使用示例     gps_gnths_parse((char *)data_buffer, &gnss);
// 备注信息     内部使用
//-------------------------------------------------------------------------------------------------------------------
static uint8 gps_gnths_parse (char *line, gnss_info_struct *gnss)
{
    uint8 state = 0;
    char *buf = line;
    uint8 return_state = 0;

    state = buf[get_parameter_index(2, buf)];

    if('A' == state)
    {
        gnss->antenna_direction_state = 1;
        gnss->antenna_direction = get_float_number(&buf[get_parameter_index(1, buf)]);
        return_state = 1;
    }
    else
    {
        gnss->antenna_direction_state = 0;
    }
    
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算从第一个点到第二个点的距离
// 参数说明     latitude1       第一个点的纬度
// 参数说明     longitude1      第一个点的经度
// 参数说明     latitude2       第二个点的纬度
// 参数说明     longitude2      第二个点的经度
// 返回参数     double          返回两点距离
// 使用示例     get_two_points_distance(latitude1_1, longitude1, latitude2, longitude2);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
double get_two_points_distance (double latitude1, double longitude1, double latitude2, double longitude2)
{  
    const double EARTH_RADIUS = 6378137;                                        // 地球半径(单位：m)
    double rad_latitude1 = 0;
    double rad_latitude2 = 0;
    double rad_longitude1 = 0;
    double rad_longitude2 = 0;
    double distance = 0;
    double a = 0;
    double b = 0;
    
    rad_latitude1 = ANGLE_TO_RAD(latitude1);                                    // 根据角度计算弧度
    rad_latitude2 = ANGLE_TO_RAD(latitude2);
    rad_longitude1 = ANGLE_TO_RAD(longitude1);
    rad_longitude2 = ANGLE_TO_RAD(longitude2);

    a = rad_latitude1 - rad_latitude2;
    b = rad_longitude1 - rad_longitude2;

    distance = 2 * asin(sqrt(pow(sin(a / 2), 2) + cos(rad_latitude1) * cos(rad_latitude2) * pow(sin(b / 2), 2)));   // google maps 里面实现的算法
    distance = distance * EARTH_RADIUS;  

    return distance;  
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算从第一个点到第二个点的方位角
// 参数说明     latitude1       第一个点的纬度
// 参数说明     longitude1      第一个点的经度
// 参数说明     latitude2       第二个点的纬度
// 参数说明     longitude2      第二个点的经度
// 返回参数     double          返回方位角（0至360）
// 使用示例     get_two_points_azimuth(latitude1_1, longitude1, latitude2, longitude2);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
double get_two_points_azimuth (double latitude1, double longitude1, double latitude2, double longitude2)
{
    latitude1 = ANGLE_TO_RAD(latitude1);
    latitude2 = ANGLE_TO_RAD(latitude2);
    longitude1 = ANGLE_TO_RAD(longitude1);
    longitude2 = ANGLE_TO_RAD(longitude2);

    double x = sin(longitude2 - longitude1) * cos(latitude2);
    double y = cos(latitude1) * sin(latitude2) - sin(latitude1) * cos(latitude2) * cos(longitude2 - longitude1);
    double angle = RAD_TO_ANGLE(atan2(x, y));
    return ((0 < angle) ? angle : (angle + 360));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     解析GPS数据
// 参数说明     void
// 返回参数     uint8           0-解析成功 1-解析失败 可能数据包错误
// 使用示例     gps_data_parse();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
static int8 gnss_hex_value (uint8 value)
{
    if((value >= '0') && (value <= '9')) return (int8)(value - '0');
    if((value >= 'A') && (value <= 'F')) return (int8)(value - 'A' + 10);
    if((value >= 'a') && (value <= 'f')) return (int8)(value - 'a' + 10);
    return -1;
}

static uint8 gnss_checksum_valid (const uint8 *buffer)
{
    uint32 i;
    uint32 star_index = 0;
    uint8 calculation = 0;
    int8 high;
    int8 low;

    if((NULL == buffer) || ('$' != buffer[0]))
    {
        return 0;
    }

    for(i = 1; i < GNSS_BUFFER_SIZE; i++)
    {
        if('*' == buffer[i])
        {
            star_index = i;
            break;
        }
        if((0 == buffer[i]) || ('\r' == buffer[i]) || ('\n' == buffer[i]))
        {
            return 0;
        }
        calculation ^= buffer[i];
    }

    if((0 == star_index) || ((star_index + 2) >= GNSS_BUFFER_SIZE))
    {
        return 0;
    }
    high = gnss_hex_value(buffer[star_index + 1]);
    low = gnss_hex_value(buffer[star_index + 2]);
    if((high < 0) || (low < 0))
    {
        return 0;
    }
    return (calculation == (uint8)(((uint8)high << 4) | (uint8)low)) ? 1 : 0;
}

uint8 gnss_data_parse (void)
{
    uint8 return_state = 0;

    if(GPS_STATE_RECEIVED == gnss_rmc_state)
    {
        gnss_rmc_state = GPS_STATE_PARSING;
        if(gnss_checksum_valid(gps_rmc_buffer) && gps_gnrmc_parse((char *)gps_rmc_buffer, &gnss))
        {
            gnss_rmc_flag = 1;
            gnss_rmc_count ++;
        }
        else
        {
            return_state = 1;
        }
        gnss_rmc_state = GPS_STATE_RECEIVING;
    }

    if(GPS_STATE_RECEIVED == gnss_gga_state)
    {
        gnss_gga_state = GPS_STATE_PARSING;
        if(gnss_checksum_valid(gps_gga_buffer) && gps_gngga_parse((char *)gps_gga_buffer, &gnss))
        {
            gnss_gga_flag = 1;
            gnss_gga_count ++;
        }
        else
        {
            return_state = 1;
        }
        gnss_gga_state = GPS_STATE_RECEIVING;
    }

    if(GPS_STATE_RECEIVED == gnss_ths_state)
    {
        gnss_ths_state = GPS_STATE_PARSING;
        if(!gnss_checksum_valid(gps_ths_buffer) || !gps_gnths_parse((char *)gps_ths_buffer, &gnss))
        {
            return_state = 1;
        }
        gnss_ths_state = GPS_STATE_RECEIVING;
    }
    return return_state;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     GPS串口回调函数
// 参数说明     void
// 返回参数     void
// 使用示例     gps_uart_callback();
// 备注信息     此函数需要在串口接收中断内进行调用
//-------------------------------------------------------------------------------------------------------------------
static void gnss_store_received_frame (const uint8 *frame, uint16 length)
{
    uint16 i;
    uint8 recognized = 0;

    if((NULL == frame) || (length < 6) || (length >= GNSS_BUFFER_SIZE) || ('$' != frame[0]))
    {
        return;
    }

    if(('R' == frame[3]) && ('M' == frame[4]) && ('C' == frame[5]))
    {
        recognized = 1;
        // RECEIVED状态尚未开始解析时可用更新的一帧覆盖；PARSING期间绝不改写缓冲区。
        if(GPS_STATE_PARSING != gnss_rmc_state)
        {
            for(i = 0; i < length; i ++) gps_rmc_buffer[i] = frame[i];
            gps_rmc_buffer[length] = 0;
            gnss_rmc_state = GPS_STATE_RECEIVED;
        }
    }
    else if(('G' == frame[3]) && ('G' == frame[4]) && ('A' == frame[5]))
    {
        recognized = 1;
        if(GPS_STATE_PARSING != gnss_gga_state)
        {
            for(i = 0; i < length; i ++) gps_gga_buffer[i] = frame[i];
            gps_gga_buffer[length] = 0;
            gnss_gga_state = GPS_STATE_RECEIVED;
        }
    }
    else if(('T' == frame[3]) && ('H' == frame[4]) && ('S' == frame[5]))
    {
        recognized = 1;
        if(GPS_STATE_PARSING != gnss_ths_state)
        {
            for(i = 0; i < length; i ++) gps_ths_buffer[i] = frame[i];
            gps_ths_buffer[length] = 0;
            gnss_ths_state = GPS_STATE_RECEIVED;
        }
    }

    if(recognized)
    {
        gnss_flag = 1;
    }
}

void gnss_uart_callback (void)
{
    uint8 dat = 0;

    if(gnss_state)
    {
        while(uart_query_byte(GNSS_UART, &dat))
        {
            if(gnss_receiver_drop)
            {
                if('\n' == dat)
                {
                    gnss_receiver_drop = 0;
                    gnss_receiver_length = 0;
                }
                continue;
            }

            if('\n' == dat)
            {
                if(gnss_receiver_length)
                {
                    gnss_receiver_buffer[gnss_receiver_length] = 0;
                    gnss_store_received_frame(gnss_receiver_buffer, gnss_receiver_length);
                }
                gnss_receiver_length = 0;
                continue;
            }

            if('\r' == dat)
            {
                continue;
            }

            // 只从'$'开始组帧；若上一帧缺少换行，新的'$'可立即重新同步。
            if('$' == dat)
            {
                gnss_receiver_length = 0;
            }
            else if(0 == gnss_receiver_length)
            {
                continue;
            }

            if(gnss_receiver_length >= (GNSS_BUFFER_SIZE - 1))
            {
                gnss_receiver_length = 0;
                gnss_receiver_drop = 1;
            }
            else
            {
                gnss_receiver_buffer[gnss_receiver_length] = dat;
                gnss_receiver_length ++;
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     GPS初始化
// 参数说明     void
// 返回参数     void
// 使用示例     gps_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void gnss_init (gps_device_enum gps_device)
{
    const uint8 set_rate[]      = {0xF1, 0xD9, 0x06, 0x42, 0x14, 0x00, 0x00, 0x0A, 0x05, 0x00, 0x64, 0x00, 0x00, 0x00, 0x60, 0xEA, 0x00, 0x00, 0xD0, 0x07, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x00, 0xB8, 0xED};
    const uint8 open_gga[]      = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x01, 0xFB, 0x10};
    const uint8 open_rmc[]      = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x01, 0x00, 0x1A};
    
    const uint8 close_gll[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11};
    const uint8 close_gsa[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13};
    const uint8 close_grs[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
    const uint8 close_gsv[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17};
    const uint8 close_vtg[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x06, 0x00, 0x00, 0x1B};
    const uint8 close_zda[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x07, 0x00, 0x01, 0x1D};
    const uint8 close_gst[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x08, 0x00, 0x02, 0x1F};
    const uint8 close_txt[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x40, 0x00, 0x3A, 0x8F};
    const uint8 close_txt_ant[] = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x20, 0x00, 0x1A, 0x4F};

    gnss_state = 0;
    gnss_flag = 0;
    gnss_rmc_flag = 0;
    gnss_gga_flag = 0;
    gnss_rmc_count = 0;
    gnss_gga_count = 0;
    gnss.state = 0;
    gnss.satellite_used = 0;
    gnss.fix_quality = 0;
    gnss.hdop = 0.0f;
    gnss_receiver_length = 0;
    gnss_receiver_drop = 0;
    gnss_rmc_state = GPS_STATE_RECEIVING;
    gnss_gga_state = GPS_STATE_RECEIVING;
    gnss_ths_state = GPS_STATE_RECEIVING;
    
    if((TAU1201 == gps_device) || (GN42A == gps_device))
    {
        system_delay_ms(500);                                                   // 等待GPS启动后开始初始化
        uart_init(GNSS_UART, 115200, GNSS_RX, GNSS_TX);

        uart_write_buffer(GNSS_UART, (uint8 *)set_rate, sizeof(set_rate));      // 设置GPS更新速率为10hz 如果不调用此语句则默认为1hz
        system_delay_ms(200);   
            
        uart_write_buffer(GNSS_UART, (uint8 *)open_rmc, sizeof(open_rmc));      // 开启rmc语句
        system_delay_ms(50);    
        uart_write_buffer(GNSS_UART, (uint8 *)open_gga, sizeof(open_gga));      // 开启gga语句
        system_delay_ms(50);
        uart_write_buffer(GNSS_UART, (uint8 *)close_gll, sizeof(close_gll));
        system_delay_ms(50);
        uart_write_buffer(GNSS_UART, (uint8 *)close_gsa, sizeof(close_gsa));
        system_delay_ms(50);
        uart_write_buffer(GNSS_UART, (uint8 *)close_grs, sizeof(close_grs));
        system_delay_ms(50);
        uart_write_buffer(GNSS_UART, (uint8 *)close_gsv, sizeof(close_gsv));
        system_delay_ms(50);
        uart_write_buffer(GNSS_UART, (uint8 *)close_vtg, sizeof(close_vtg));
        system_delay_ms(50);
        uart_write_buffer(GNSS_UART, (uint8 *)close_zda, sizeof(close_zda));
        system_delay_ms(50);
        uart_write_buffer(GNSS_UART, (uint8 *)close_gst, sizeof(close_gst));
        system_delay_ms(50);
        uart_write_buffer(GNSS_UART, (uint8 *)close_txt, sizeof(close_txt));
        system_delay_ms(50);
        uart_write_buffer(GNSS_UART, (uint8 *)close_txt_ant, sizeof(close_txt_ant));
        system_delay_ms(50);

        gnss_state = 1;
        uart_rx_interrupt(GNSS_UART, 1);
    }
    else if(GN43RFA == gps_device)
    {
        // GN43RFA RTK模块不需要进行参数设置，如果需要修改参数应该使用专用的上位机修改参数
        uart_init(GNSS_UART, 115200, GNSS_RX, GNSS_TX);
        gnss_state = 1;
        uart_rx_interrupt(GNSS_UART, 1);
    }
    
}
