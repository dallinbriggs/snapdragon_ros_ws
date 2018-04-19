/************************************************************
 * 
 * Snappy Comm Node
 * 
 * This node subscribes to certain messages from ROS and sends 
 * them out to some other computer or system over a serial port.
 * 
 * Author: Neil Johnson
 * Date: 1/16/18
 ************************************************************/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "SerialPort/SerialPortLinux.h"

//#define SERIAL_DEVICE "/dev/slamdunk"
#define SERIAL_DEVICE "/dev/ttyUSB0"
#define SERIAL_BAUD B115200
//#define SERIAL_BAUD B921600

#define TEST_SERIAL_TX 0

class SnappyComm {
	
public:
    SnappyComm()
    {
		m_serialPort.Open(SERIAL_DEVICE, SERIAL_BAUD, PARITY_NONE);
	}
	
    ~SnappyComm() 
    {
		disconnect();
	}
	
    void disconnect()
    {
		m_serialPort.Close();
	}
	
	void poseCallback(const geometry_msgs::PoseStamped& msg)
	{
		//ROS_INFO("Got a pose!  I think I'll send that over the serial port!");
		SendPose(msg);
	}

	#define PACKET_LENGTH 74
	
	#define PACKET_TYPE_POSE_STAMPED 101
	
	uint8_t computeChecksum(uint8_t *pkt, int len)
	{
		uint8_t c = 0;
		for (int i=0; i<len; i++) {
			c ^= pkt[i];
		}
		
		return c;
	}
	
    void SendPose(const geometry_msgs::PoseStamped& msg)
    {
		//!!!!!! Start packet header:
		
		//! Create and send a pose packet:
		uint8_t packet[PACKET_LENGTH];
		
		//! Two start bytes:
		packet[0] = 0xAA;
		packet[1] = 0xBB;
		
		//! packet length
		uint16_t packet_len = PACKET_LENGTH;
		memcpy(&packet[2], &packet_len, 2);
		
		//! packet type
		packet[4] = PACKET_TYPE_POSE_STAMPED;
		
		//!!!!!! Done with packet header
		
		//! stuff from msg header: (Ignore frame_id...not sure how to use this right now)
		memcpy(&packet[5], &msg.header.seq, 4); //! uint32_t
		memcpy(&packet[9], &msg.header.stamp.sec, 4);	//! Seconds portion of the timestamp
		memcpy(&packet[13], &msg.header.stamp.nsec, 4); //! nsec portion of the timestamp
		
		//! Pose:
		//! Position:
		memcpy(&packet[17], &msg.pose.position.x, 8);
		memcpy(&packet[25], &msg.pose.position.y, 8);
		memcpy(&packet[33], &msg.pose.position.z, 8);
		
		//! Orientation:
		memcpy(&packet[41], &msg.pose.orientation.x, 8);
		memcpy(&packet[49], &msg.pose.orientation.y, 8);
		memcpy(&packet[57], &msg.pose.orientation.z, 8);
		memcpy(&packet[65], &msg.pose.orientation.w, 8);
		
		//! Checksum:
		packet[73] = computeChecksum(packet, PACKET_LENGTH-1);
		
#if TEST_SERIAL_TX
		printf("Sending serial packet, seq = %d\n", msg.header.seq);
#endif
		
		m_serialPort.sendBytes(packet, PACKET_LENGTH);		
	}

private:
    SerialPortLinux m_serialPort;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "snappy_comm");
	ros::NodeHandle nh;
	
	SnappyComm comm;
	
#if TEST_SERIAL_TX
	ros::Rate loop_rate(2);
	
	int cnt = 0;
	
	while (ros::ok()) 
	{
		geometry_msgs::PoseStamped msg;
		msg.header.seq = cnt++;
		msg.pose.position.x = 10.1;
		msg.pose.position.y = 11.2;
		msg.pose.position.z = 12.3;
		msg.pose.orientation.x = 0.0;
		msg.pose.orientation.y = 0.0;
		msg.pose.orientation.z = 0.0;
		msg.pose.orientation.w = 1.0;
		
		comm.SendPose(msg);
		
		ros::spinOnce();
		loop_rate.sleep();
	}

#else
	ros::Subscriber poseSub = nh.subscribe("vislam/pose", 1, &SnappyComm::poseCallback, &comm);
	
	ros::spin();
#endif
	
	return 0;	
}
