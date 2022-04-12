
#include <stdio.h>
#include <time.h>
#include <locale.h>
 
int main()
{
 
        // struct tm my_time = { .tm_year=112, // = year 2012
        //                   .tm_mon=9,    // = 10th month
        //                   .tm_mday=9,   // = 9th day
        //                   .tm_hour=8,   // = 8 hours
        //                   .tm_min=10,   // = 10 minutes
        //                   .tm_sec=20    // = 20 secs
        //                     };

    struct tm my_time;
    my_time.tm_year = 112;
    my_time.tm_mon = 9;
    my_time.tm_mday = 9;
    my_time.tm_hour = 8;
    my_time.tm_min = 10;
    my_time.tm_sec = 21;
    printf("Today is           %s", asctime(&my_time));
    printf("(DST is %s)\n", my_time.tm_isdst ? "in effect" : "not in effect");
    //my_time.tm_mon -= 100;  // tm_mon is now outside its normal range
    time_t timet = mktime(&my_time);       // tm_isdst is not set to -1; today's DST status is used
    // printf("100 months ago was %s", asctime(&my_time));
    // printf("(DST was %s)\n", my_time.tm_isdst ? "in effect" : "not in effect");
    printf("%ld\n", timet);


}