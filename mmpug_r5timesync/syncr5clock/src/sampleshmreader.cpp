#include <timesyncdata.h>

// Physical addresses of the timers
#define TSC_LO_ADDR 0x03010000
#define TSC_HI_ADDR 0x03010004

int map_addr(unsigned long *physical_addr, int *mem_fd, void **map, 
        volatile unsigned int **addr, size_t block_size) {
   
   printf("Mapping physical address 0x%lx\n", *physical_addr);
   // check if we have valid physical address
   if (!(*physical_addr)) {
       printf("Unspecified physical address!\n"); 
       exit(-1);
   }  
    
   // open /dev/mem
   *mem_fd = open("/dev/mem", O_RDONLY);
   if (mem_fd < 0) {
       printf("Failed to open /dev/mem :(\n");
       exit(-1);
   }

   // mmap target physical address to memory
   unsigned long MAP_MASK = (sysconf(_SC_PAGE_SIZE) - 1);
   *map = mmap(
            NULL,
            block_size,
            PROT_READ,
            MAP_PRIVATE,
            *mem_fd,
            (off_t)*physical_addr & ~MAP_MASK
            ); 
   if (*map ==  MAP_FAILED) {
        printf("mmap failed!\n");
        exit (-1);
   }

   *addr = (volatile unsigned int *) (*map + (*physical_addr & MAP_MASK));

   return 0;
}

volatile unsigned int *tsc_lo_addr;
volatile unsigned int *tsc_hi_addr;
unsigned long tsc_lo_count;
unsigned long tsc_hi_count;
/* Get times*/


int main() {

  unsigned long tsc_lo_physical = TSC_LO_ADDR;
  unsigned long tsc_hi_physical = TSC_HI_ADDR;
  int tsc_lo_mem_fd;
  int tsc_hi_mem_fd;   
  void *tsc_lo_map;
  void *tsc_hi_map; 
  size_t block_size = 4;     
  printf("Getting registers to oscillator\n");
  map_addr(&tsc_lo_physical, &tsc_lo_mem_fd, &tsc_lo_map, &tsc_lo_addr, block_size);
  map_addr(&tsc_hi_physical, &tsc_hi_mem_fd, &tsc_hi_map, &tsc_hi_addr, block_size);
 
  //Successfully passed to here:
  TimeSync::SharedMemoryInterface shminterface;
  TimeSync::Data shmdata;
  // combined tsc micro sec
  timespec systemtime1,systemtime2,systemtime,raw,monotonic;
  while (1)
    {

      clock_gettime(CLOCK_REALTIME, &systemtime1);
      clock_gettime(CLOCK_MONOTONIC, &monotonic); 
      clock_gettime(CLOCK_MONOTONIC_RAW, &raw); 
      tsc_lo_count = (unsigned long)*tsc_lo_addr;
      tsc_hi_count = (unsigned long)*tsc_hi_addr;
      clock_gettime(CLOCK_REALTIME, &systemtime2); 
      unsigned long tsc_hi_countl = tsc_hi_count<<32;
      unsigned long tsc_lo_countl = tsc_lo_count;
      unsigned long tsc = tsc_hi_countl+tsc_lo_countl;

      systemtime.tv_sec = systemtime1.tv_sec;
      systemtime.tv_nsec = (systemtime1.tv_nsec + systemtime2.tv_nsec)/2;
      shminterface.getData(shmdata);
      printf("----------------------------------------------------------\n");
      printf("monotonic: %ld %ld\n",monotonic.tv_sec,monotonic.tv_nsec);
      printf("monotonic raw: %ld %ld\n",raw.tv_sec,raw.tv_nsec);
      printf("st: %ld %ld\n",systemtime.tv_sec,systemtime.tv_nsec);
      printf("Freq: %u Offset: %lu usec Valid %d\n",shmdata.ticks_persecond, shmdata.offset_us, shmdata.valid);
      timespec time;
      if(convertToSystemTime(tsc ,shmdata, time))
      {
        printf("tc: %ld %ld\n",time.tv_sec,time.tv_nsec);
	printf("tsc diff: %ld ns\n",time.tv_nsec-systemtime.tv_nsec);
      }
      if(convertToSystemTimeRAW(raw,shmdata,time))
	{
	  printf("tc: %ld %ld\n",time.tv_sec,time.tv_nsec);
	  printf("raw monotonic diff: %ld ns\n",time.tv_nsec-systemtime.tv_nsec);
	  
	}

      if(convertToSystemTimeMono(monotonic,shmdata,time))
	{
	  printf("tc: %ld %ld\n",time.tv_sec,time.tv_nsec);
	  printf("monotonic diff: %ld ns\n",time.tv_nsec-systemtime.tv_nsec);  
	}     
      usleep(100000);
    }
  return 0;
}
