extern "C" {
#include "hcl.h"
#include "hcl_gpio.h"
#include "hcl_uart.h"
#include "sensor_epsonCommon.h"
}
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <termios.h>
#include <string>
#include <iostream>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include "nmea/nmea.h"

using namespace std;

int comPort;
string serial_port;
int baudrate;

//=========================================================================
//------------------------ IMU Initialization -----------------------------
//=========================================================================

bool init(const struct EpsonOptions& options){
  ROS_INFO("Initializing HCL layer...");
  if (!seInit()){
    ROS_ERROR("Error: could not initialize the Seiko Epson HCL layer. Exiting...");
    return false;
  }

  ROS_INFO("Initializing GPIO interface...");
  if (!gpioInit()){
    ROS_ERROR("Error: could not initialize the GPIO layer. Exiting...");
    seRelease();
    return false;
  }

  ROS_INFO("Initializing UART interface...");
  comPort = uartInit(serial_port.c_str(), baudrate);
  if(comPort == -1){
    ROS_ERROR("Error: could not initialize UART interface. Exiting...");
    gpioRelease();
    seRelease();
    return false;
  }
  ROS_INFO("Resetting sensor...");
  // This is required for NVIDIA Jetson TK1 but root cause is still TBD.
  // Current assumption is that when the serial port is first opened, there are
  // corrupted characters sent on first serial messages.
  // So sending sensor stop may work around this condition
  // Investigation TBD
  sensorRecoverFromGarbageInit();
  sensorReset();
  sensorStop();

  ROS_INFO("Checking sensor NOT_READY status...");
  if(!sensorPowerOn()){
    ROS_ERROR("Error: failed to power on Sensor. Exiting...");
    uartRelease(comPort);
    gpioRelease();
    seRelease();
    return false;
  }
  printf("...done.");

  ROS_INFO("Initializing Sensor...");
  if(!sensorInitOptions(options)){
    ROS_ERROR("Error: could not initialize Epson Sensor. Exiting...");
    uartRelease(comPort);
    gpioRelease();
    seRelease();
    return false;
  }

  ROS_INFO("...Epson IMU initialized.");
  return true;
}

//=========================================================================
//----------------------- Timestamp Correction ----------------------------
//=========================================================================

class TimeCorrection {
  private:
    int MAX_COUNT;
    int ALMOST_ROLLOVER;
    int ONE_SEC_NSEC;
    int HALF_SEC_NSEC;
    int UP_THRESHOLD_SEC_NSEC;

    int32_t count_corrected;
    int32_t count_old;
    int32_t count_diff;
    int32_t time_current;
    int32_t time_old;
    int32_t time_nsec_current;
    int32_t count_corrected_old;
    bool rollover;
    bool flag_imu_lead;

    bool send_velodyne_nmea;
    int VELODYNE_NMEA_THRESH;

    nmea::NmeaConnection<UDPSend>::Params params;
    nmea::NmeaConnection<UDPSend> velodyne_conn;
    nmea::NmeaMessage nmea;
  public:
    TimeCorrection();
    ros::Time get_stamp(int);
};

TimeCorrection::TimeCorrection()
    : params("192.168.1.201", 10110), velodyne_conn(params) {
#if defined G365PDC0 || defined G365PDF0 || defined G370PDC0 || defined G370PDF0 || defined G325PDF0
  MAX_COUNT       = 1048560000;
  ALMOST_ROLLOVER = 1045000000;
#else // G354/G364PDC0/G364PDCA/V340
  MAX_COUNT       = 1398080000;
  ALMOST_ROLLOVER = 1340000000;
#endif
  ONE_SEC_NSEC = 1000000000;
  HALF_SEC_NSEC = 500000000;
  UP_THRESHOLD_SEC_NSEC = 800000000;

  count_corrected = 0;
  count_old = 0;
  count_diff = 0;
  time_old = 0;
  time_current = 0;
  time_nsec_current = 0;
  count_corrected_old = 0;
  rollover = false;
  flag_imu_lead = false;

  // henryzh47: canary PPS duty cycle high 300ms + 50ms buffer
  //            set delay to 400ms
  send_velodyne_nmea = false;
  VELODYNE_NMEA_THRESH = 400000000;

  ROS_INFO("TimeCorrection initializing NMEA...");
  nmea = nmea::NmeaMessage();
}

ros::Time TimeCorrection::get_stamp(int count){
  // This assumes that the ROS time is sync'ed to 1PPS pulse sent to IMU
  // and the ROS latency from IMU sample to calling ros::Time::now() is
  // less than 0.020 seconds, otherwise the time in seconds can be off by 1
  // The IMU count is already converted to nsecs units (should always because
  // less than ONE_SEC_NSEC (1e9)
  time_current = ros::Time::now().toSec();
  time_nsec_current = ros::Time::now().nsec;
  std::cout.precision(20);

  count_diff = count - count_old;
  if(count > ALMOST_ROLLOVER){
    rollover = true;
  }
  if(count_diff<0){
    if(rollover){
      count_diff = count + (MAX_COUNT - count_old);
      ROS_INFO_STREAM("rollover at: " << ros::Time::now().toSec());
      rollover = false;
    }
    else {
      count_diff = count;
      // trying to only correct when time is close to a full second
      // if(time_nsec_current > UP_THRESHOLD_SEC_NSEC || time_nsec_current < (ONE_SEC_NSEC - UP_THRESHOLD_SEC_NSEC))
      // {
        count_corrected = 0;
      // }
      send_velodyne_nmea = true;
      ROS_INFO_STREAM("reset at: " << ros::Time::now().toSec());
    }
#ifdef DEBUG
    std::cout << "cnt: " << count <<"\t";
    std::cout << "cnt_o: " << count_old <<"\t";
    std::cout << "cnt_d: " << count_diff <<"\t";
    std::cout << "cntc: " << count_corrected <<"\t";
    std::cout << "cntc_o: " << count_corrected_old <<"\t";
    std::cout << "tm_c: " << time_current <<"\t";
    std::cout << "tm_o: " << time_old <<"\t";
    std::cout << "rovr: " << rollover <<"\t";
    std::cout << "imu_ld: " << flag_imu_lead << std::endl;
#endif
    rollover = 0;
  }
  count_corrected = (count_corrected + count_diff) % ONE_SEC_NSEC;
  // henryzh47: send velodyne nmea 400ms after PPS
  if (send_velodyne_nmea && (count_corrected > VELODYNE_NMEA_THRESH)) {
    /*
    std::string msg = nmea.create(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
    ROS_INFO_STREAM("send msg to velodyne " << ros::Time::now().toSec() << ": " << msg);
    velodyne_conn.send(msg);
    send_velodyne_nmea = false;
    */
  }
  if((time_current != time_old )&& (count_corrected > HALF_SEC_NSEC)){
#ifdef DEBUG
    std::cout << "tm_c != tm_o" << "\t";
    std::cout << "cnt: " << count <<"\t";
    std::cout << "cnt_o: " << count_old <<"\t";
    std::cout << "cnt_d: " << count_diff <<"\t";
    std::cout << "cntc: " << count_corrected <<"\t";
    std::cout << "cntc_o: " << count_corrected_old <<"\t";
    std::cout << "tm_c: " << time_current <<"\t";
    std::cout << "tm_o: " << time_old <<"\t";
    std::cout << "rovr: " << rollover <<"\t";
    std::cout << "imu_ld: " << flag_imu_lead << std::endl;
#endif
    time_current = time_current - 1;
    flag_imu_lead = 0;
  }
  else if(((count_corrected-count_corrected_old) < 0) && (time_nsec_current > HALF_SEC_NSEC)){
#ifdef DEBUG
    std::cout << "cntc < cntc_o" << "\t";
    std::cout << "cnt: " << count <<"\t";
    std::cout << "cnt_o: " << count_old <<"\t";
    std::cout << "cnt_d: " << count_diff <<"\t";
    std::cout << "cntc: " << count_corrected <<"\t";
    std::cout << "cntc_o: " << count_corrected_old <<"\t";
    std::cout << "tm_c: " << time_current <<"\t";
    std::cout << "tm_o: " << time_old <<"\t";
    std::cout << "rovr: " << rollover <<"\t";
    std::cout << "imu_ld: " << flag_imu_lead << std::endl;
#endif
    time_current = time_current + 1;
    flag_imu_lead = 1;
  }
  else if(flag_imu_lead && (time_nsec_current > HALF_SEC_NSEC)){
#ifdef DEBUG
    std::cout << "imu_ld = 1" << "\t";
    std::cout << "cnt: " << count <<"\t";
    std::cout << "cnt_o: " << count_old <<"\t";
    std::cout << "cnt_d: " << count_diff <<"\t";
    std::cout << "cntc: " << count_corrected <<"\t";
    std::cout << "cntc_o: " << count_corrected_old <<"\t";
    std::cout << "tm_c: " << time_current <<"\t";
    std::cout << "tm_o: " << time_old <<"\t";
    std::cout << "rovr: " << rollover <<"\t";
    std::cout << "imu_ld: " << flag_imu_lead << std::endl;
#endif
    time_current = time_current + 1;
  }
  else{
    flag_imu_lead = 0;
  }
  ros::Time time;
  time.nsec = count_corrected;
  time.sec = time_current;
  time_old=time_current;
  count_old = count;
  count_corrected_old = count_corrected;

  return time;
}

#include <math.h>

//=========================================================================
//------------------------------ Main -------------------------------------
//=========================================================================

int main(int argc, char** argv){
  ros::init(argc, argv, "epson_imu_driver_node");
  ros::NodeHandle nh;
  ros::NodeHandle np("~");

  np.param<string>("port", serial_port, "/dev/ttyUSB0");

  struct EpsonOptions options;
  int time_correction = 0;

  // Recommended to change these parameters via .launch file instead of
  // modifying source code below directly
  np.param("ext_sel", options.ext_sel, 0);
  np.param("ext_pol", options.ext_pol, 0);
  np.param("drdy_on", options.drdy_on, 0);
  np.param("drdy_pol", options.drdy_pol, 0);

  np.param("dout_rate", options.dout_rate, 3);
  np.param("filter_sel", options.filter_sel, 5);

  np.param("flag_out", options.flag_out, 1);
  np.param("temp_out", options.temp_out, 1);
  np.param("gyro_out", options.gyro_out, 1);
  np.param("accel_out", options.accel_out, 1);
  np.param("gyro_delta_out", options.gyro_delta_out, 0);
  np.param("accel_delta_out", options.accel_delta_out, 0);
  np.param("atti_out", options.atti_out, 0);
  np.param("gpio_out", options.gpio_out, 0);
  np.param("count_out", options.count_out, 1);
  np.param("checksum_out", options.checksum_out, 1);

  np.param("temp_bit", options.temp_bit, 1);
  np.param("gyro_bit", options.gyro_bit, 1);
  np.param("accel_bit", options.accel_bit, 1);
  np.param("gyro_delta_bit", options.gyro_delta_bit, 1);
  np.param("accel_delta_bit", options.accel_delta_bit, 1);
  np.param("atti_bit", options.atti_bit, 1);

  np.param("invert_xgyro", options.invert_xgyro, 0);
  np.param("invert_ygyro", options.invert_ygyro, 0);
  np.param("invert_zgyro", options.invert_zgyro, 0);
  np.param("invert_xaccel", options.invert_xaccel, 0);
  np.param("invert_yaccel", options.invert_yaccel, 0);
  np.param("invert_zaccel", options.invert_zaccel, 0);

  np.param("dlt_ovf_en", options.dlt_ovf_en, 0);
  np.param("dlt_range_ctrl", options.dlt_range_ctrl, 8);

  np.param("atti_mode", options.atti_mode, 1);
  np.param("atti_conv", options.atti_conv, 0);

  np.param("time_correction", time_correction, 0);

  // The baudrate value should be set the the same setting as currently 
  // flashed value in the IMU UART_CTRL BAUD_RATE register
  baudrate = 460800;

  bool successful_init = init(options);
  for(int i = 0; i < 100; ++i) {
    if(successful_init) break;
    successful_init = init(options);
  }
  sensorStart();
  double imu_start_time = ros::Time::now().toSec();

  struct EpsonData epson_data;
  TimeCorrection tc;

  sensor_msgs::Imu imu_msg;
  for(int i = 0; i < 9; i++){
    imu_msg.orientation_covariance[i] = 0;
    imu_msg.angular_velocity_covariance[i] = 0;
    imu_msg.linear_acceleration_covariance[i] = 0;
  }
  imu_msg.orientation_covariance[0] = -1;

  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("epson_imu", 1);
  ros::Publisher rpy_pub = nh.advertise<geometry_msgs::Vector3Stamped>("epson_imu_rpy", 1);
  tf2::Quaternion myQuaternion;

  
  while(ros::ok()){
    if(sensorDataReadBurstNOptions(options, &epson_data)){
      imu_msg.header.frame_id = "epson";
      double imu_on_time = ros::Time::now().toSec() - imu_start_time;
      if(!time_correction){
        // Usually see a 1.5-2ms delay
        imu_msg.header.stamp = imu_msg.header.stamp.fromSec(ros::Time::now().toSec() - 0.002);
      } else if (imu_on_time < 2.0){
        // Let the IMU see at least 2 pulses before starting get_stamp 
        imu_msg.header.stamp = imu_msg.header.stamp.fromSec(ros::Time::now().toSec() - 0.0025);

      } else if(imu_on_time < 4.0){    
        // If <1s from SensorStart() then IMU hasn't seen PPS yet
        // May need 500/41 = 12.2s buffer here to ensure PPS has shifted into place 
        // Keep get_stamp() running to avoid large jumps at init       
        imu_msg.header.stamp = tc.get_stamp(epson_data.count);
        imu_msg.header.stamp = imu_msg.header.stamp.fromSec(ros::Time::now().toSec() - 0.0025);
      }
      else
        imu_msg.header.stamp = tc.get_stamp(epson_data.count);
      
      // change to vel && acc
      
      imu_msg.angular_velocity.x = -epson_data.gyro_x;
      imu_msg.angular_velocity.y = -epson_data.gyro_y;
      imu_msg.angular_velocity.z = epson_data.gyro_z;
      imu_msg.linear_acceleration.x = -epson_data.accel_x;
      imu_msg.linear_acceleration.y = -epson_data.accel_y;
      imu_msg.linear_acceleration.z = epson_data.accel_z;

      // swap roll and pitch because G365 rotation order is yaw -> roll -> pitch
      // reverse polarity roll to maintain Right Hand Rule
      // myQuaternion.setRPY(epson_data.pitch, -epson_data.roll, epson_data.yaw); // Original
      
      auto roll = -epson_data.roll;
      auto pitch = -epson_data.pitch;
      auto yaw = epson_data.yaw;
      // ROS_INFO_STREAM("roll: " << roll << "  pitch: " << pitch << "  yaw: " << yaw); //ivan
      // ROS_INFO_STREAM("atti_out: " << options.atti_out); //ivan
      double t1 = 0.5 * yaw;
      double t2 = 0.5 * roll;
      double t3 = 0.5 * pitch;
      myQuaternion[3] = -sin(t1) * sin(t2) * sin(t3) + cos(t1) * cos(t2)*cos(t3);
      myQuaternion[0] = -sin(t1)*sin(t3)*cos(t2) + sin(t2)*cos(t1)*cos(t3);
      myQuaternion[1] = sin(t1)*sin(t2)*cos(t3) + sin(t3)*cos(t1)*cos(t2);
      myQuaternion[2] = sin(t1)*cos(t2)*cos(t3) + sin(t2)*sin(t3)*cos(t1);
      
      // tf2::Quaternion q_rot;
      // q_rot.setRPY(0.0, 0.0, 3.14);
      tf2::Quaternion q_new = myQuaternion;
      q_new.normalize();
      
      myQuaternion.normalize();
      imu_msg.orientation.x = q_new[0];
      imu_msg.orientation.y = q_new[1];
      imu_msg.orientation.z = q_new[2];
      imu_msg.orientation.w = q_new[3];
      
      imu_pub.publish(imu_msg);

      // publish roll, pitch, yaw angles
      geometry_msgs::Vector3Stamped rpy;
      rpy.header = imu_msg.header;
      rpy.vector.x = roll;
      rpy.vector.y = pitch;
      rpy.vector.z = yaw;
      rpy_pub.publish(rpy);

    }
  }

  sensorStop();
  uartRelease(comPort);
  gpioRelease();
  seRelease();

  return 0;
}
