////////////////////////////////////////////////////////////////////////
/// @file nmea
/// @brief Handles storing nmea message to send to velodyne
////////////////////////////////////////////////////////////////////////
#pragma once

#include <sys/time.h>
#include <iostream>
#include <ostream>
#include <chrono>

namespace nmea {

// /////////////////////////////////////////////////////////////////////////////
/// @brief TimeBookkeeper
// /////////////////////////////////////////////////////////////////////////////
class TimeBookkeeper {
public:
  /// @brief me ptr
  typedef std::shared_ptr<TimeBookkeeper> Ptr;

  /// @brief me ptr creator
  static Ptr MakeShared() { return Ptr(new TimeBookkeeper()); }

  /// @brief default constructor
  TimeBookkeeper() : dayOffset_(0), timeDiff_(0) { initTime(); };

  /// @brief default destructor
  ~TimeBookkeeper() {};

  /// updates the local time intervals
  void updateTime(int computerSec) {
    day_ = int(computerSec / 86400) - dayOffset_ + 1;
	  hour_ = int(computerSec / 3600) % 24;
	  minute_ = int(computerSec / 60) % 60;
	  second_ = computerSec % 60;
  }

  /// initialize the time
  void initTime() {
    int computerSecInit = int(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
    dayOffset_ = int(computerSecInit / 86400);
  }
  
  /// @brief getters to time variables
  int get_day()         { return day_;      }
  int get_hour()        { return hour_;     }
  int get_minute()      { return minute_;   }
  int get_second()      { return second_;   }
  int get_day_offset()  { return dayOffset_; }

private:
  
  /// @brief bookkeep the time intervals
  int day_;
  int hour_;
  int minute_;
  int second_;
  int dayOffset_;
  double timeDiff_;
};

// /////////////////////////////////////////////////////////////////////////////
/// @brief Timer
// /////////////////////////////////////////////////////////////////////////////
class Timer {
public:

  /// @brief default constructor
  Timer() { reset(); }

  /// @brief reset start time to wall time
  void reset() {
    start_ = this->get_wall_time();
  }

  /// @brief reset start time to wall time
  double now() {
    return this->get_wall_time();
  }

  /// @brief reset start time to wall time
  template<typename T>
  int print_time(std::chrono::time_point<T> time) {
    
    time_t curr_time = T::to_time_t(time);
    char sRep[100];
    strftime(sRep, sizeof(sRep),"%Y-%m-%d %H:%M:%S", localtime(&curr_time));

    // auto mili = delay(time);
    std::cout << '[' << sRep << ":" << decimate(time).count() << "] " << std::endl;
    std::cout << std::flush;
  }

  /// @brief convert wall time to seconds
  double seconds() const {
    return this->get_wall_time() - start_;
  }

  /// @brief convert wall time to milliseconds
  double milliseconds() const {
    return magnitude::millisec * (this->get_wall_time() - start_);
  }

  /// @brief convert wall time to minutes
  double minutes() const {
    return (this->get_wall_time() - start_) / magnitude::minutes;
  }

  /// @brief print
  friend std::ostream& operator<< (std::ostream& os, const Timer& timer)  {
    os << std::endl << " == timer info == " << std::endl;
    os << " milli is: " << timer.milliseconds() << std::endl;
    os << " seconds is: " << timer.seconds() << std::endl;
    os << " minutes is: " << timer.minutes() << std::endl;
  }

  /// @brief get the time amount to delay (miliseconds)
  template<typename T>
  std::chrono::milliseconds decimate(std::chrono::time_point<T> time) {
    // get the current time since epoch, removing all the seconds passed
    typename T::duration since_epoch = time.time_since_epoch();
    auto s = std::chrono::duration_cast<std::chrono::seconds>(since_epoch);
    since_epoch -= s;
    return std::chrono::duration_cast<std::chrono::milliseconds>(since_epoch);
  }

  /// @brief get the time amount to delay (miliseconds)
  template<typename T>
  int delay(std::chrono::time_point<T> time) {
    /// return the miliseconds needed to put a delay for
    return 1000 - decimate(time).count();
  }

private:

  /// @brief orders of magnitude for time. Base unit magnitude is 'second'.
  struct magnitude {
    static constexpr auto millisec = 1000.0;
    static constexpr auto minutes = 60.0;
  };

  /// @brief get real wall-clock time in seconds
  double get_wall_time() const {
    struct timeval time;
    if (gettimeofday(&time, NULL))
      return 0;
    return (double)time.tv_sec + (double)time.tv_usec * .000001;
  }

  /// @brief get cpu time
  double get_cpu_time(){
    return (double)clock() / CLOCKS_PER_SEC;
  }

  /// @brief start time of timer
  double start_;
};


}; // namespace nmea
