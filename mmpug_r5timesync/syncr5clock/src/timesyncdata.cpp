#include <timesyncdata.h>
#include <cstdlib>
#include <sstream>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

using namespace std;
using namespace TimeSync;
using namespace boost::interprocess;

//Initialize struct to 0
Data::Data():
ticks_persecond(0),
offset_us(0),
frequency(0),
valid(false)
{
  offsetraw.tv_sec = 0;
  offsetraw.tv_nsec = 0;
  offsetmono.tv_nsec = 0;
  offsetmono.tv_nsec = 0;
}


bool TimeSync::convertToSystemTime(const unsigned long &tsc,const Data &data, timespec &time)
{
  //Could be made much more efficient by just doing fixed point but too lazy to do the mathc
	if(!data.valid)
		return false;
	double timeinsec = (((double)(tsc ))/data.frequency + ((double)data.offset_us))/1000000.0;
	//printf("%lu %f %f\n",tsc,timeinsec,((double)tsc)/data.frequency);
	time.tv_sec = timeinsec;
	double remainder = 1000000000.0*(timeinsec - time.tv_sec);
	time.tv_nsec = remainder;
}


timespec add(timespec start, timespec end)
{
  timespec temp;
  
  if (start.tv_nsec + end.tv_nsec >= 1E9)
    {
      temp.tv_sec = start.tv_sec + end.tv_sec + 1;
      temp.tv_nsec = start.tv_nsec + end.tv_nsec - 1E9;
    }
  else
    {
      temp.tv_sec = start.tv_sec + end.tv_sec;
      temp.tv_nsec = start.tv_nsec + end.tv_nsec;
    };
  
  return temp;
}


bool TimeSync::convertToSystemTimeRAW(const timespec &rawtime,const Data &data, timespec &time)
{
  if(!data.valid)
    return false;
  double ratio = 1.0/(((double)data.ticks_persecond)/31250000.0);
  timespec adjusted;
  convertRawTimeToFreq(ratio,rawtime,adjusted);
  //printf("offsetraw: %lu %lu\n",data.offsetraw.tv_sec,data.offsetraw.tv_nsec);
  time = add(adjusted, data.offsetraw);
}

bool TimeSync::convertToSystemTimeMono(const timespec &monotime,const Data &data, timespec &time)
{
  if(!data.valid)
    return false;
  time = add(monotime, data.offsetmono);
}


void TimeSync::convertRawTimeToFreq(const double &ratio,const timespec &rawtime, timespec &timefreq)
{
  double timeinsec = (rawtime.tv_sec + ((double)rawtime.tv_nsec)/1000000000.0 ) *ratio;
  timefreq.tv_sec = timeinsec;
  double remainder = 1000000000.0*(timeinsec - timefreq.tv_sec);
  timefreq.tv_nsec = remainder;
  //printf("timeinsec %f ratio: %f\n",timeinsec, ratio);
}




SharedMemoryInterface::SharedMemoryInterface():
 shdmem(NULL),
 region(NULL)
 {
   boost::interprocess::permissions per;
   per.set_unrestricted();
   shdmem = new shared_memory_object(open_or_create, "TimeSyncSharedMemory", read_write,per);
 	shdmem->truncate(1024);
 	region = new mapped_region{*shdmem, read_write};
 	Data *bg = static_cast<Data *>(region->get_address());
 	bg->valid = false;
}

SharedMemoryInterface::~SharedMemoryInterface()
{
	delete region;
 	shared_memory_object::remove("TimeSyncSharedMemory");
 	delete shdmem;
}

void SharedMemoryInterface::getData(Data & dataoutput)
{
	Data *bg = static_cast<Data *>(region->get_address());
  	dataoutput.ticks_persecond = bg->ticks_persecond;
  	dataoutput.offset_us = bg->offset_us;
    dataoutput.offset_ns = bg->offset_ns;
	dataoutput.valid = bg->valid;
	dataoutput.frequency = bg->frequency;
	dataoutput.offsetraw.tv_sec  = bg->offsetraw.tv_sec;
	dataoutput.offsetraw.tv_nsec  = bg->offsetraw.tv_nsec;
	dataoutput.offsetmono.tv_sec  = bg->offsetmono.tv_sec;
	dataoutput.offsetmono.tv_nsec  = bg->offsetmono.tv_nsec;
}

void SharedMemoryInterface::setData(const Data & datainput)
{
	Data *bg = static_cast<Data *>(region->get_address());
  	bg->ticks_persecond = datainput.ticks_persecond;
  	bg->offset_us = datainput.offset_us;
     bg->offset_ns = datainput.offset_ns;
	bg->valid = datainput.valid;
	bg->frequency = datainput.frequency;
	bg->offsetraw.tv_sec  = datainput.offsetraw.tv_sec;
	bg->offsetraw.tv_nsec  = datainput.offsetraw.tv_nsec;
	bg->offsetmono.tv_sec  = datainput.offsetmono.tv_sec;
	bg->offsetmono.tv_nsec  = datainput.offsetmono.tv_nsec;

}
