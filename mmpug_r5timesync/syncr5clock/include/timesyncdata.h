#ifndef __TIMESYNCDATA_H_
#define __TIMESYNCDATA_H_

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <ctime>

namespace TimeSync
{
  
  struct Data
  {
    unsigned int ticks_persecond;
    long int offset_us;
    long int offset_ns;
    bool valid;
    double frequency;
    timespec offsetraw;
    timespec offsetmono;
    Data();
  };


  /** Convert TSC to System time */
  
  bool convertToSystemTime(const unsigned long &tsc,const Data &data, timespec &time);

  /** Convert MONOTONIC_RAW to System time */
  
  bool convertToSystemTimeRAW(const timespec &rawtime,const Data &data, timespec &time);

  /** Convert MONOTONIC to System time */
  
  bool convertToSystemTimeMono(const timespec &monotime,const Data &data, timespec &time);

  /** Adjust clock by frequency (internal function) */
  
  void convertRawTimeToFreq(const double &ratio,const timespec &rawtime, timespec &timefreq);
  
  class SharedMemoryInterface
  {
  public:
    //Initialize Interface
    SharedMemoryInterface();
    //ReleaseInterface
    ~SharedMemoryInterface();
    void getData(Data & data);
    void setData(const Data & data);
  private:
    boost::interprocess::shared_memory_object *shdmem;
    boost::interprocess::mapped_region *region;
  };
  
}


#endif
