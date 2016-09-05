#include "serial_mavlink.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

using namespace std;

Serial_mavlink::Serial_mavlink(ros::NodeHandle nh):nh_(nh)
{
	mavlink_vision_position_estimate_sub = nh_.subscribe("mavlink_vision_position_estimate_topic", 10, &Serial_mavlink::mavlink_vision_position_estimate_callback, this);
	mavlink_vicon_position_estimate_sub = nh_.subscribe("mavlink_vicon_position_estimate_topic", 10, &Serial_mavlink::mavlink_vicon_position_estimate_callback, this);
	mavlink_att_pos_mocap_sub = nh_.subscribe("mavlink_att_pos_mocap_topic", 10, &Serial_mavlink::mavlink_att_pos_mocap_callback, this);
	if(!nh_param.getParam("serial_port", serial_port)) serial_port = "/dev/ttyUSB0";
	if(!nh_param.getParam("baudrate", baudrate)) baudrate = 57600;
	serial_fd = open(serial_port.c_str(), O_RDWR);
    if(-1 == serial_fd)
    {
		perror("open serial error!");
		exit(1);
    }
    if((tcgetattr(serial_fd, &oldtio)) != 0)
	{
		perror("SetupSerial");
        //return -1;
	}
    if(set_term(serial_fd, baudrate, 8, 'N', 1) < 0)
	{
		perror("set_term error");
		exit(1);
	}
	ros::spin();
}

Serial_mavlink::~Serial_mavlink()
{
	cout<<"Destroying Serial_mavlink..."<<endl;
	int closefd = close(serial_fd);
}


int Serial_mavlink::set_term(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio;
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;
    switch(nBits)
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    case 9:
        newtio.c_cflag |= CS8;
        break;
    }
    /* Set the Parity Bit */
    switch(nEvent)
    {
    case 'O': // Odd Parity
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP); // I'm not sure, but it doesn't matters
        break;
    case 'E': // Even Parity
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        newtio.c_cflag |= (INPCK | ISTRIP); // I'm not sure, but it doesn't matters
        break;
    case 'N': // None Parity
        newtio.c_cflag &= ~PARENB;
        break;
    default:
        newtio.c_cflag &= ~PARENB;
        break;
    }
    /* Set the Baud Rate */
    switch(nSpeed)
    {
    case 2400:
		cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 19200:
        cfsetispeed(&newtio, B19200);
        cfsetospeed(&newtio, B19200);
        break;
	case 57600:
        cfsetispeed(&newtio, B57600);
        cfsetospeed(&newtio, B57600);
        break;		
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    /* Set the Stop Bit */
    if(nStop == 1)
        newtio.c_cflag &= ~CSTOPB;
    else if(nStop == 2)
        newtio.c_cflag |= CSTOPB;

    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 12;

    tcflush(fd, TCIFLUSH);

    if((tcsetattr(fd, TCSANOW, &newtio)) != 0) {
        perror("com set error");
        return -1;
    }

    printf("set done!\n");
    return 0;
}


void Serial_mavlink::_mavlink_send_uart(const char *buf, uint8_t length)
{
	write(serial_fd, buf, length);
}

void Serial_mavlink::crc_init(uint16_t* crcAccum)
{
	*crcAccum = X25_INIT_CRC;
}

void Serial_mavlink::crc_accumulate(uint8_t data, uint16_t *crcAccum)
{
	uint8_t tmp;
	tmp = data ^ (uint8_t)(*crcAccum &0xff);
	tmp ^= (tmp<<4);
	*crcAccum = (*crcAccum>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);
}

uint16_t Serial_mavlink::crc_calculate(const uint8_t* pBuffer, uint16_t length)
{
	uint16_t crcTmp;
	crc_init(&crcTmp);
	while (length--) {
			crc_accumulate(*pBuffer++, &crcTmp);
	}
	return crcTmp;
}

void Serial_mavlink::crc_accumulate_buffer(uint16_t *crcAccum, const char *pBuffer, uint16_t length)
{
	const uint8_t *p = (const uint8_t *)pBuffer;
	while (length--) 
	{
		crc_accumulate(*p++, crcAccum);
	}
}



void Serial_mavlink::mavlink_finalize_message(mavlink_message_t* msg, uint8_t sysid, uint8_t compid, uint8_t length, uint8_t crc_extra)
{
	uint16_t checksum;
	uint8_t buf[MAVLINK_NUM_HEADER_BYTES];
	uint8_t ck[2];
	buf[0] = MAVLINK_STX;
	buf[1] = length;
	buf[2] = 0;
	buf[3] = sysid;
	buf[4] = compid;
	buf[5] = msg->msgid;
	checksum = crc_calculate((const uint8_t*)&buf[1], MAVLINK_CORE_HEADER_LEN);
	crc_accumulate_buffer(&checksum, _MAV_PAYLOAD_NON_CONST(msg), length);
	crc_accumulate(crc_extra, &checksum);
	ck[0] = (uint8_t)(checksum & 0xFF);
	ck[1] = (uint8_t)(checksum >> 8);
	_mavlink_send_uart((const char *)buf, MAVLINK_NUM_HEADER_BYTES);
	_mavlink_send_uart(_MAV_PAYLOAD_NON_CONST(msg), length);
	_mavlink_send_uart((const char *)ck, 2);
}

void Serial_mavlink::mavlink_msg_vision_position_estimate_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw)
{
	mavlink_vision_position_estimate_t packet;
	packet.usec = usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN);
	msg->msgid = MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE;
	mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_CRC);
}

void Serial_mavlink::mavlink_msg_vicon_position_estimate_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw)
{
	mavlink_vicon_position_estimate_t packet;
	packet.usec = usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE_LEN);
	msg->msgid = MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE;
	mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE_LEN, MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE_CRC);
}

void Serial_mavlink::mavlink_msg_att_pos_mocap_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint64_t time_usec, float x, float y, float z, float qw, float qx, float qy, float qz)
{
	mavlink_att_pos_mocap_t packet;
	packet.time_usec = time_usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.q[0] = qw;
	packet.q[1] = qx;
	packet.q[2] = qy;
	packet.q[3] = qz;
	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN);
	msg->msgid = MAVLINK_MSG_ID_ATT_POS_MOCAP;
	mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN, MAVLINK_MSG_ID_ATT_POS_MOCAP_CRC);
}

void Serial_mavlink::mavlink_vision_position_estimate_callback(const geometry_msgs::TwistStampedConstPtr& msg)
{
	uint64_t usec = 0;
	float x = msg->twist.linear.x;
	float y = msg->twist.linear.y;
	float z = msg->twist.linear.z;
	float roll = msg->twist.angular.x;
	float pitch = msg->twist.angular.y;
	float yaw = msg->twist.angular.z;
	mavlink_message_t mavlink_msg;
	mavlink_msg_vision_position_estimate_pack(1, 255, &mavlink_msg, usec, x, y, z, roll, pitch, yaw);
}

void Serial_mavlink::mavlink_vicon_position_estimate_callback(const geometry_msgs::TwistStampedConstPtr& msg)
{
	uint64_t usec = 0;
	float x = msg->twist.linear.x;
	float y = msg->twist.linear.y;
	float z = msg->twist.linear.z;
	float roll = msg->twist.angular.x;
	float pitch = msg->twist.angular.y;
	float yaw = msg->twist.angular.z;
	mavlink_message_t mavlink_msg;
	mavlink_msg_vicon_position_estimate_pack(1, 255, &mavlink_msg, usec, x, y, z, roll, pitch, yaw);
}

void Serial_mavlink::mavlink_att_pos_mocap_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
	uint64_t usec = 0;
	float x = (float)msg->pose.pose.position.x;
	float y = (float)msg->pose.pose.position.y;
	float z = (float)msg->pose.pose.position.z;
	float w_ = (float)msg->pose.pose.orientation.w;
	float x_ = (float)msg->pose.pose.orientation.x;
	float y_ = (float)msg->pose.pose.orientation.y;
	float z_ = (float)msg->pose.pose.orientation.z;
	mavlink_message_t mavlink_msg;
	mavlink_msg_att_pos_mocap_pack(1, 255, &mavlink_msg, usec, x, y, z, w_, x_, y_, z_);
	
}




















