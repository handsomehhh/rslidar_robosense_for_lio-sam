#include <stdio.h>
#include <time.h>
#include <locale.h>

#include <stdint.h>
#include <stddef.h>

struct raw_time
{
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint16_t msecond;
  uint16_t usecond;
};

double get_timestamp(const raw_time raw_time_data)
{
  //处理sec
  struct tm cur_time;
  cur_time.tm_year = raw_time_data.year + 100;
  cur_time.tm_mon = raw_time_data.month - 1;
  cur_time.tm_mday = raw_time_data.day;
  cur_time.tm_hour = raw_time_data.hour;
  cur_time.tm_min = raw_time_data.minute;
  cur_time.tm_sec = raw_time_data.second;
  printf("%d %d\n",cur_time.tm_year,cur_time.tm_mon);

  uint32_t sec = mktime(&cur_time);
  printf("%d\n",sec);

  //处理nsec
  uint32_t nsec = raw_time_data.msecond * 1e6 + raw_time_data.usecond;

  printf("%04x %04x  %d\n",raw_time_data.msecond,raw_time_data.usecond,nsec);

  //得到double类型时间戳
  return (static_cast<double>(sec) + 1e-9*static_cast<double>(nsec));
}

int main()
{
    uint8_t arr[10] = {0x11,0x03,0x0A,0x09,0x2D,0x1E,0x64,0x00,0xce,0x01};

    const raw_time* raw_time_data = (const raw_time*) &arr[0];

    double timestamp = get_timestamp(*raw_time_data);

    printf("%.9lf\n",timestamp);
}