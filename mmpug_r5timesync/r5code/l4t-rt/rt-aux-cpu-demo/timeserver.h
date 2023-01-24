
#ifndef __TIMESERVER_H
#define __TIMESERVER_H


/* All the time server variables */
struct TimeServerInformation
{
	unsigned int   ticks_persecond;
	 int   phaseoffset_ticks;
	unsigned int synced;
};

extern struct TimeServerInformation timeserverdata;



#endif