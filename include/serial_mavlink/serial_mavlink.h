#ifndef SERIAL_MAVLINK_H
#define SERIAL_MAVLINK_H
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "time.h"
#include <termios.h> 
#include <errno.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h> 
#include <cstdio>
#include <cstring>

using namespace std;

#define COMNUM 3
#define BAUD 57600
#define MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE_LEN 32
#define MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE_CRC 56
#define MAVLINK_CORE_HEADER_LEN 5 ///< Length of core header (of the comm. layer): message length (1 byte) + message sequence (1 byte) + message system id (1 byte) + message component id (1 byte) + message type id (1 byte)
#define MAVLINK_NUM_HEADER_BYTES (MAVLINK_CORE_HEADER_LEN + 1) ///< Length of all header bytes, including core and checksum
#define MAVLINK_STX 254
#define X25_INIT_CRC 0xffff
#define _MAV_PAYLOAD(msg) ((const char *)(&((msg)->payload64[0])))
#define _MAV_PAYLOAD_NON_CONST(msg) ((char *)(&((msg)->payload64[0])))
#define MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE 104
#define PI 3.1415926
#ifndef __int8_t_defined
# define __int8_t_defined
typedef signed char             int8_t; 
typedef short int               int16_t;
typedef int                     int32_t;
# if __WORDSIZE == 64
typedef long int                int64_t;
# else
//__extension__
typedef long long int           int64_t;
# endif
#endif
typedef unsigned char           uint8_t;
typedef unsigned short int      uint16_t;
#ifndef __uint32_t_defined
typedef unsigned int            uint32_t;
# define __uint32_t_defined
#endif
#if __WORDSIZE == 64
typedef unsigned long int       uint64_t;
#else
//__extension__
typedef unsigned long long int  uint64_t;
#endif

/////////////////////////////////////////////////////////////////////////////////////
// MAVLINK_ATT_POS_MOCAP
/////////////////////////////////////////////////////////////////////////////////////
#define MAVLINK_MSG_ID_ATT_POS_MOCAP 138
#define MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN 36
#define MAVLINK_MSG_ID_ATT_POS_MOCAP_CRC 109
typedef struct __mavlink_att_pos_mocap_t
{
	uint64_t time_usec;
	float q[4];
	float x;
	float y;
	float z;
} mavlink_att_pos_mocap_t;
/////////////////////////////////////////////////////////////////////////////////////
// MAVLINK_VISION_POSITION_ESTIMATE
/////////////////////////////////////////////////////////////////////////////////////
#define MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE 102
#define MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN 32
#define MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_CRC 158
typedef struct __mavlink_vision_position_estimate_t
{
	uint64_t usec;
	float x;
	float y;
	float z;
	float roll;
	float pitch;
	float yaw;
} mavlink_vision_position_estimate_t;

/////////////////////////////////////////////////////////////////////////////////////
// MAVLINK_VICON_POSITION_ESTIMATE
/////////////////////////////////////////////////////////////////////////////////////
typedef struct __mavlink_vicon_position_estimate_t
{
 uint64_t usec; /*< Timestamp (microseconds, synced to UNIX time or since system boot)*/
 float x; /*< Global X position*/
 float y; /*< Global Y position*/
 float z; /*< Global Z position*/
 float roll; /*< Roll angle in rad*/
 float pitch; /*< Pitch angle in rad*/
 float yaw; /*< Yaw angle in rad*/
} mavlink_vicon_position_estimate_t;


typedef struct mavlink_message{
	uint8_t magic;
	uint8_t length;
	uint8_t seq;
	uint8_t sysid;
	uint8_t compid;
	uint8_t msgid;
	uint8_t payload64[255];
	uint8_t checksum[2];
} mavlink_message_t;


class Serial_mavlink
{
public:
	//For ROS
	ros::NodeHandle nh_;
	ros::NodeHandle nh_param;
	ros::Subscriber mavlink_vision_position_estimate_sub;
	ros::Subscriber mavlink_vicon_position_estimate_sub;
	ros::Subscriber mavlink_att_pos_mocap_sub;
	//Variables
	string serial_port;
	int baudrate;
	int serial_fd;
	struct termios oldtio;
	
	
	// Fuctions
	Serial_mavlink(ros::NodeHandle nh);
	~Serial_mavlink();
	void _mavlink_send_uart(const char *buf, uint8_t length);
	void crc_init(uint16_t* crcAccum);
	void crc_accumulate(uint8_t data, uint16_t *crcAccum);
	uint16_t crc_calculate(const uint8_t* pBuffer, uint16_t length);
	void crc_accumulate_buffer(uint16_t *crcAccum, const char *pBuffer, uint16_t length);
	void mavlink_finalize_message(mavlink_message_t* msg, uint8_t sysid, uint8_t compid, uint8_t length, uint8_t crc_extra);
	void mavlink_msg_vicon_position_estimate_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw);
	void mavlink_msg_vision_position_estimate_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw);
	void mavlink_msg_att_pos_mocap_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint64_t time_usec, float x, float y, float z, float qw, float qx, float qy, float qz);
	int set_term(int fd, int nSpeed, int nBits, char nEvent, int nStop);
	void mavlink_vision_position_estimate_callback(const geometry_msgs::TwistStampedConstPtr& msg);
	void mavlink_vicon_position_estimate_callback(const geometry_msgs::TwistStampedConstPtr& msg);
	void mavlink_att_pos_mocap_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
};

#endif
