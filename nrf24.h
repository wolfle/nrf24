#ifndef _NRF24_H_
#define _NRF24_H_
#include <linux/sockios.h>
//#define TIMEOUT HZ*2  //2 seconds

//netdev driver ioctl commands
#define SET_CHANNEL SIOCDEVPRIVATE //...+15  
#define GET_CHANNEL SIOCDEVPRIVATE+1
#define GET_RPD SIOCDEVPRIVATE+2
#define SET_PIPES SIOCDEVPRIVATE+3
#define GET_PIPES SIOCDEVPRIVATE+4

#endif
