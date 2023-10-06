#include <ros/package.h>
#include <ros/ros.h>

#include "sensor_msgs/Joy.h"
#include "yaml-cpp/yaml.h"
#include <arpa/inet.h>
#include <chrono>
#include <errno.h>
#include <fstream>
#include <linux/if.h>
#include <linux/if_ether.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <pthread.h>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

// Define the IP address and port for the UDP sender
#define SEND_IP "10.0.3.207" // IP address of the UDP receiver node (adjust as needed)
#define SEND_PORT 1234

float analog_l[2][2], analog_r[2][2], axes_l2[2], axes_r2[2], toggle_a, toggle_b;
int8_t button_sq[2], button_x[2], button_o[2], button_tr[2], button_r1[2], button_l1[2], button_ps[2], button_sl[2], button_pl[2], button_up[2], button_down[2], button_left[2], button_right[2];
bool is_red_js[2] = { true }; // Default: hitam

void JoyCllbckjs0(const sensor_msgs::Joy::ConstPtr& msg);
void send_cllbck(const ros::TimerEvent&);
char send_buffer[128];
char header[4] = "its";
int8_t indentifier = '7';

int sock;
struct sockaddr_in server_addr;

ros::Timer send_timer;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "udp_sender");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(0);

    // Create a UDP socket for sending data
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        ROS_ERROR("Failed to create socket");
        return -1;
    }

    // Set up the server address for UDP transmission
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SEND_PORT);
    server_addr.sin_addr.s_addr = inet_addr(SEND_IP);

    // Create a subscriber for your joystick topic
    ros::Subscriber subjs0 = nh.subscribe("/js0/joy", 100, JoyCllbckjs0);

    // Create a timer to send UDP data periodically
    send_timer = nh.createTimer(ros::Duration(0.1), send_cllbck);

    // Start the ROS multi-threaded spinner
    spinner.spin();

    // Close the UDP socket when the program exits
    close(sock);

    return 0;
}

void JoyCllbckjs0(const sensor_msgs::Joy::ConstPtr& msg)
{
    // ROS_INFO("[%d] [%f] [%f] [%d] [%f] [%f] [%d] [%f] [%f] [%d]",msg->buttons[0], msg->axes[7], msg->axes[6], msg->buttons[3], msg->axes[0], msg->axes[1], msg->buttons[5], msg->axes[2], msg->axes[5], msg->buttons[4]);
    analog_l[0][0] = -msg->axes[0]; //analog x
    analog_l[0][1] = msg->axes[1]; //analog y
    axes_l2[0] = msg->axes[2];
    analog_r[0][0] = msg->axes[3];
    analog_r[0][1] = msg->axes[4];
    axes_r2[0] = msg->axes[5];
    button_x[0] = msg->buttons[0];
    button_o[0] = msg->buttons[1];
    button_tr[0] = msg->buttons[2];
    button_sq[0] = msg->buttons[3];
    button_l1[0] = msg->buttons[4];
    button_r1[0] = msg->buttons[5];
    button_sl[0] = msg->buttons[8];
    button_pl[0] = msg->buttons[9];
    button_ps[0] = msg->buttons[10];

    //stik hitam
    button_up[0] = msg->buttons[13];
    button_down[0] = msg->buttons[14];
    button_left[0] = msg->buttons[15];
    button_right[0] = msg->buttons[16];

    if (button_up[0] > 1 || button_up[0] < -1 || button_down[0] > 1 || button_down[0] < -1 || button_left[0] > 1 || button_left[0] < -1 || button_right[0] > 1 || button_right[0] < -1) {
        is_red_js[0] = true;
    } else {
        is_red_js[0] = false;
    }

    if (is_red_js[0]) {
        //stik merah
        toggle_a = msg->axes[7];
        toggle_b = msg->axes[6];
        button_up[0] = 0;
        button_down[0] = 0;
        button_left[0] = 0;
        button_right[0] = 0;
        if (toggle_a > 0) {
            button_up[0] = 1;
        } else if (toggle_a < 0) {
            button_down[0] = 1;
        }
        if (toggle_b > 0) {
            button_left[0] = 1;
        } else if (toggle_b < 0) {
            button_right[0] = 1;
        }
    }

    ROS_INFO("%d %d %d %d || %.f %.f || %.f %.f || %.f %.f", button_up[0], button_down[0], button_left[0], button_right[0], toggle_a, toggle_b, analog_l[0][0], analog_l[0][1], axes_l2[0], axes_r2[0]);
}

void send_cllbck(const ros::TimerEvent&)
{
    // Create a buffer to hold your data
    char send_buffer[128];

    // Fill the buffer with your data
    // For example, you can use memcpy to copy your data into send_buffer

    // Send the message over UDP
    ssize_t bytes_sent = sendto(sock, send_buffer, sizeof(send_buffer), 0,
        (struct sockaddr*)&server_addr, sizeof(server_addr));

    if (bytes_sent < 0) {
        ROS_ERROR("Failed to send UDP message");
    } else {
        ROS_INFO("Sent UDP message");
    }
}
