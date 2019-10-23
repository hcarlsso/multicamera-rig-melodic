#include "ros/ros.h"

#include <boost/thread.hpp>

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

#include <iostream>
#include <exception>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <curl/curl.h>

std::string frame_id;

ros::Publisher imu_pub;
sensor_msgs::Imu imu_msg;
boost::mutex mutex;


// Data sent in network byte order
//
//
// Fix rate ut, 1kHz



using namespace std;

void handler(const boost::system::error_code& error,
             int signal_number) {
  if (!error) {
    // A signal occurred.
    exit(0);
  }
}

void spin_thread () {
  // ROS_ERROR("Before Spin in spin_thread");
  ros::spin();
}

float bytesToFloat(unsigned char b0, unsigned char b1, unsigned char b2,  unsigned char b3) {
  // unsigned char byte_array[] = { b3, b2, b1, b0 };
  unsigned char byte_array[] = { b0, b1, b2, b3 };
  float result;
  std::copy(reinterpret_cast<const char*>(&byte_array[0]),
            reinterpret_cast<const char*>(&byte_array[4]),
            reinterpret_cast<char*>(&result));
  return result;
}


class udp_server {
public:
    udp_server(boost::asio::io_service& io_service, int port_number)
        : socket_(io_service, boost::asio::ip::udp::udp::endpoint(boost::asio::ip::udp::udp::v4(), port_number)) {
    std::cout << "UDP server listening on " << port_number << std::endl;
    have_gyro_flag = false;
    have_acc_flag = false;
    have_pos_flag = false;
    start_receive();
  }

private:

  double bytesToDouble(unsigned int i) {
  //unsigned char byte_array[] = { b0, b1, b2, b3, b4, b5, b6, b7 };
    unsigned char byte_array[] = { recv_buffer_[i+7],
                                   recv_buffer_[i+6],
                                   recv_buffer_[i+5],
                                   recv_buffer_[i+4],
                                   recv_buffer_[i+3],
                                   recv_buffer_[i+2],
                                   recv_buffer_[i+1],
                                   recv_buffer_[i+0]}; //netw byte order
    double result;
    std::copy(reinterpret_cast<const char*>(&byte_array[0]),
              reinterpret_cast<const char*>(&byte_array[8]),
              reinterpret_cast<char*>(&result));
    return result;
  }


    void start_receive() {
      socket_.async_receive_from(
                                 boost::asio::buffer(recv_buffer_), remote_endpoint_,
                                 boost::bind(&udp_server::handle_receive, this,
                                             boost::asio::placeholders::error,
                                             boost::asio::placeholders::bytes_transferred));
  }

  void handle_receive(const boost::system::error_code& error,
                      std::size_t bytes_transferred) {
    if (!error || error == boost::asio::error::message_size) {
      // MESSAGES SIZE: 91 bytes
      // cerr << "Got message: " << bytes_transferred << endl;

      //      cerr << "Got message: " << bytes_transferred << " - " << recv_buffer_[0] << " - " << recv_buffer_[33] << endl;
      //      float time = bytesToFloat(recv_buffer_[1], recv_buffer_[2], recv_buffer_[3], recv_buffer_[4]);

      double gtime = bytesToDouble(1);
      ros::Time mytime = ros::Time::now();
      printf(" secs %d nsecs %d %f \n",  mytime.sec, mytime.nsec,  gtime) ;
      //float ax, ay, az, gx, gy, gz;
      double ax, ay, az, gx, gy, gz;
      // Should be in rad/s
      gx = bytesToDouble(9);
      gy = bytesToDouble(17);
      gz = bytesToDouble(25);

      double atime = bytesToDouble(34);
      // Accel should be in m/s²
      ax = bytesToDouble(42);
      ay = bytesToDouble(50);
      az = bytesToDouble(58);
      if (recv_buffer_[0] == 'G') {
        have_gyro_flag = true;
        // ROS_INFO("         GYRO: %f - %f %f %f", gtime, gx, gy, gz);
      }
      if (recv_buffer_[33] == 'A') {
        have_acc_flag = true;
        // ROS_INFO("ACCELEROMETER: %f - %f %f %f", atime, ax, ay, az);
      }

      double ptime = 0.0;
      double pan = 0.0;
      double tilt = 0.0;

      if (recv_buffer_[66] == 'P') {
        have_pos_flag = true;
        ptime = bytesToDouble(67);
        pan = bytesToDouble(75);
        tilt = bytesToDouble(83);
        // ROS_INFO("POSITION TIME - PAN TILT: %f - %f %f", pan, tilt);
      }

      if (!have_gyro_flag) {
        start_receive();
        return;
      }

      if (!have_acc_flag) {
        start_receive();
        return;
      }

      {
          // boost::mutex::scoped_lock lock(mutex);

      /* Fill the IMU message */

      // Fill the header
      // imu_msg.header.stamp = enable_Tsync ? ros::Time(pstampSynchronizer->sync(data.timeStamp, ros::Time::now().toSec(), data.frameCount)) : ros::Time::now();

      imu_msg.header.stamp = ros::Time::now();
      imu_msg.header.frame_id = frame_id;

      imu_msg.orientation.x = 0.0;
      imu_msg.orientation.y = 0.0;
      imu_msg.orientation.z = 0.0;
      imu_msg.orientation.w = 1.0;

      imu_msg.orientation_covariance[0] = -1.0;

      imu_msg.angular_velocity.x = gx*M_PI/180.0;
      imu_msg.angular_velocity.y = gy*M_PI/180.0;
      imu_msg.angular_velocity.z = gz*M_PI/180.0;

      imu_msg.angular_velocity_covariance[0] = 0.0;
      imu_msg.angular_velocity_covariance[1] = 0.0;
      imu_msg.angular_velocity_covariance[2] = 0.0;

      imu_msg.angular_velocity_covariance[3] = 0.0;
      imu_msg.angular_velocity_covariance[4] = 0.0;
      imu_msg.angular_velocity_covariance[5] = 0.0;

      imu_msg.angular_velocity_covariance[6] = 0.0;
      imu_msg.angular_velocity_covariance[7] = 0.0;
      imu_msg.angular_velocity_covariance[8] = 0.0;

      imu_msg.linear_acceleration.x = -ax*9.81;
      imu_msg.linear_acceleration.y = -ay*9.81;
      imu_msg.linear_acceleration.z = -az*9.81;

      imu_msg.linear_acceleration_covariance[0] = 0.0;
      imu_msg.linear_acceleration_covariance[1] = 0.0;
      imu_msg.linear_acceleration_covariance[2] = 0.0;
      imu_msg.linear_acceleration_covariance[3] = 0.0;
      imu_msg.linear_acceleration_covariance[4] = 0.0;
      imu_msg.linear_acceleration_covariance[5] = 0.0;
      imu_msg.linear_acceleration_covariance[6] = 0.0;
      imu_msg.linear_acceleration_covariance[7] = 0.0;
      imu_msg.linear_acceleration_covariance[8] = 0.0;

      imu_pub.publish(imu_msg);
      /* Fill the magnetometer message */
      //      mag_msg.header.stamp = imu_msg.header.stamp;
      //      mag_msg.header.frame_id = frame_id;

      // Units are microTesla in the LPMS library, Tesla in ROS.
      //      mag_msg.magnetic_field.x = data.b[0]*1e-6;
      //      mag_msg.magnetic_field.y = data.b[1]*1e-6;
      //      mag_msg.magnetic_field.z = data.b[2]*1e-6;
      }

      //      mag_pub.publish(mag_msg);

      start_receive();
    }
  }

  void handle_send(boost::shared_ptr<std::string> /*message*/,
                   const boost::system::error_code& /*error*/,
                   std::size_t bytes_transferred) {
    //    ROS_ERROR("SENT: %zu", bytes_transferred);
  }

  boost::asio::ip::udp::udp::socket socket_;
  boost::asio::ip::udp::udp::endpoint remote_endpoint_;
  boost::array<unsigned char, 128> recv_buffer_;

  bool have_gyro_flag;
  bool have_acc_flag;
  bool have_pos_flag;
};

void runUDPServer(int port) {
  try {
    boost::asio::io_service io_service;
    udp_server server(io_service, port);
    // Construct a signal set registered for process termination.
    boost::asio::signal_set signals(io_service, SIGINT, SIGTERM);
    signals.async_wait(handler);

    io_service.run();
  }
  catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
  }
}

void timer_callback(const ros::TimerEvent&) {
    sensor_msgs::Imu msg;
    {
        boost::mutex::scoped_lock lock(mutex);
        msg = imu_msg;
    }
    //  ROS_ERROR("publish imu");
    // Publish the messages
    imu_pub.publish(msg);
    usleep(100);
}

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "axis_imu");
  ros::NodeHandle nh, private_nh("~");

  int port;
  int rate;
  int pos_enabled;
  std::string camera_ip;
  std::string driver_ip;
  std::string root_passwd;

  private_nh.param<int>("port", port, 5000);
  private_nh.param<std::string>("frame_id", frame_id, "imu");
  private_nh.param<int>("rate", rate, 200);
  private_nh.param<int>("pos_enabled", pos_enabled, 0);
  private_nh.param<std::string>("camera_ip", camera_ip, "192.168.7.19");
  private_nh.param<std::string>("driver_ip", driver_ip, "192.168.7.160");
  private_nh.param<std::string>("root_passwd", root_passwd, "pass");

  ROS_INFO("  Camera IP: %s", camera_ip.c_str());
  ROS_INFO("  Driver IP: %s", driver_ip.c_str());
  ROS_INFO("       Port: %d", port);
  ROS_INFO("   Frame Id: %s", frame_id.c_str());
  ROS_INFO("       Rate: %d", rate);
  ROS_INFO("Pos Enabled: %d", pos_enabled);
  ROS_INFO("  Root pass: %s", root_passwd.c_str());

  int gyro_div = 2000/rate;
  int accel_div = 1000/rate;

  // currently fixed rate out so set div to 0

  gyro_div = 2;
  accel_div = 1;

  std::ostringstream os;
  std::ostringstream osstop;

  osstop << "http://" << camera_ip << "/local/motionloggerudp/control.cgi?cmd=stop";


  os << "http://" << camera_ip
     << "/local/motionloggerudp/control.cgi?cmd=start&target_ip=" << driver_ip
     << "&target_port=" << port
     << "&accel_enabled=1&accel_rate_div=" << accel_div << "&gyro_enabled=1&gyro_rate_div=" << gyro_div << "&pos_enabled=" << pos_enabled
    ;

  std::string stopurl = osstop.str();
  std::string url = os.str();

  ROS_INFO("STOPCOMMAND URL: %s", stopurl.c_str());
  ROS_INFO("COMMAND URL: %s", url.c_str());

  CURL *curl;
  CURLcode res;

  curl = curl_easy_init();

  if (curl) {
    /* example.com is redirected, so we tell libcurl to follow redirection */
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    //    curl_easy_setopt(curl, CURLOPT_HTTPAUTH, CURLAUTH_DIGEST);
    curl_easy_setopt(curl, CURLOPT_HTTPAUTH, CURLAUTH_ANY);
    curl_easy_setopt(curl, CURLOPT_USERNAME, "root");
    curl_easy_setopt(curl, CURLOPT_PASSWORD, root_passwd.c_str());
    curl_easy_setopt(curl, CURLOPT_URL, stopurl.c_str());

    /* Perform the request, res will get the return code */
    res = curl_easy_perform(curl);
    /* Check for errors */
    if(res != CURLE_OK) {
      fprintf(stderr, "curl_easy_perform() failed: %s\n",
              curl_easy_strerror(res));
    } else {
      ROS_INFO("Curl call succeeded");
    }

    //sleep(3);
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    res = curl_easy_perform(curl);
    /* Check for errors */
    if(res != CURLE_OK) {
      fprintf(stderr, "curl_easy_perform() failed: %s\n",
              curl_easy_strerror(res));
    } else {
      ROS_INFO("Curl call succeeded");
    }

    /* always cleanup */
    curl_easy_cleanup(curl);
  } else {
    ROS_ERROR("Could not create curl object, exiting");
    exit(1);
  }


  // Queue size is 1, why such a small queue?
  imu_pub = nh.advertise<sensor_msgs::Imu>("imu",1);

  // ros::Timer timer = nh.createTimer(ros::Duration(1.0/rate), timer_callback);

  boost::thread sthread (spin_thread);

  ROS_INFO("runUDPServer");

  //runUDPServer(port);

  runUDPServer(port);

  return 0;
}
