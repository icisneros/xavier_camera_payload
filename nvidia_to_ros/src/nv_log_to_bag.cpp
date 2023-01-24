#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <fstream>

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "nv_log_to_bag");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string log_filename = pnh.param("log_filename", std::string(""));
  std::string bag_filename = pnh.param("bag_filename", std::string(""));
  std::string output_topic = pnh.param("output_topic", std::string(""));

  rosbag::Bag bag;
  bag.open(bag_filename, rosbag::bagmode::Write);
  sensor_msgs::Image image;
  ros::Time time;
  
  std::ifstream fin;
  fin.open(log_filename, std::ios::binary);
  
  if(fin){
    int width, height, pitch;
    fin.read((char*)&width, sizeof(width));
    fin.read((char*)&height, sizeof(height));
    fin.read((char*)&pitch, sizeof(pitch));
    image.width = width;
    image.height = height;
    image.step = pitch;
    image.encoding = "rgb8";  //ivan
    image.data.resize(height*pitch);
    
    std::cout << "first: " << width << ", " << height << ", " << pitch << std::endl;

    while(fin){
      fin.read((char*)&time.sec, sizeof(time.sec));
      fin.read((char*)&time.nsec, sizeof(time.nsec));
      std::cout << "reading image: " << time << std::endl;

      fin.read((char*)&image.data[0], image.data.size());
      image.header.stamp = time;
      
      if(!(fin.rdstate() & std::ifstream::eofbit)){
	bag.write(output_topic, time, image);
      }
      else
	std::cout << "EOF" << std::endl;
    }
  }
  fin.close();
  bag.close();

  return 0;
}
