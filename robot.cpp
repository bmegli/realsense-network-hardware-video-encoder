#include "robot.h"
#include "mlsp.h"


#include <iostream>
#include <thread>
#include <limits.h> //INT_MAX
#include <cstring> //memcpy

using namespace std;

const uint8_t FRONT_MOTOR_ADDRESS=0x80;
const uint8_t MIDDLE_MOTOR_ADDRESS=0x81;
const uint8_t REAR_MOTOR_ADDRESS=0x82;
const int32_t MOTOR_ACCELERATION=6000;

struct drive_packet
{
	int16_t command;
	int16_t left;
	int16_t right;
} __attribute__((packed));

struct dead_reconning_packet
{
	uint64_t timestamp_us;
	int32_t position_left;
	int32_t position_right;
	//quaternion as in Unity coordinate system
	float w;
	float x;
	float y;
	float z;
};

const int DEAD_RECONNING_PACKET_BYTES=32; //8 + 2*4 + 4*4
const int CONTROL_PACKET_BYTES = 18; //8 + 5*2 = 18 bytes
enum Commands {KEEPALIVE=0, SET_SPEED=1, TO_POSITION_WITH_SPEED=2};

bool Robot::initMotors(const char* tty, int baudrate)
{
	int16_t voltage;
	bool ok=true;
	
	m_rc = roboclaw_init(tty, baudrate);
	
	if(!m_rc)
	{
		cerr << "robot: unable to initialize motors" << endl;
		return false;
	}
	
	ok &= roboclaw_main_battery_voltage(m_rc, FRONT_MOTOR_ADDRESS, &voltage) == ROBOCLAW_OK;
	ok &= roboclaw_main_battery_voltage(m_rc, MIDDLE_MOTOR_ADDRESS, &voltage) == ROBOCLAW_OK;
	ok &= roboclaw_main_battery_voltage(m_rc, REAR_MOTOR_ADDRESS, &voltage) == ROBOCLAW_OK;
	
	if(!ok)
	{
		closeMotors();
		cerr << "robot: unable to communicate with motors" << endl;
		return false;
	}
	
	cout << "robot: battery voltage is %d.%d" << voltage/10 << voltage % 10 << endl;
		
	return true;
	
}

void Robot::closeMotors()
{
	roboclaw_close(m_rc);
}

void Robot::stopMotors(roboclaw *rc)
{
	bool ok=true;
	ok &= roboclaw_duty_m1m2(rc, FRONT_MOTOR_ADDRESS, 0, 0) == ROBOCLAW_OK;
	ok &= roboclaw_duty_m1m2(rc, MIDDLE_MOTOR_ADDRESS, 0, 0) == ROBOCLAW_OK;
	ok &= roboclaw_duty_m1m2(rc, REAR_MOTOR_ADDRESS, 0, 0) == ROBOCLAW_OK;

	if(!ok)
		cerr << "robot: unable to stop motors" << endl;
}

bool Robot::initIMU(const char* tty)
{
	if( !(m_vmu=vmu_init(tty)) )
	{
		cerr << "unable to initialize VMU931" << endl;
		return false;
	} 
	
	if( vmu_stream(m_vmu, VMU_STREAM_QUAT) == VMU_ERROR )
	{
		cerr << "vmu failed to stream quaternion data" << endl;
		vmu_close(m_vmu);
		return false;
	}
}

void Robot::closeIMU()
{
	vmu_close(m_vmu);
}

bool Robot::initNetwork(uint16_t port, int timeout_ms)
{
	mlsp_config mlsp_cfg = {"", port, timeout_ms, 1};

	m_streamer = mlsp_init_server(&mlsp_cfg);
	
	if(!m_streamer)
	{
		cerr << "failed to initialize network server" << endl;
		return false;
	}

	return true;
}

void Robot::closeNetwork()
{
	mlsp_close(m_streamer);
}

bool Robot::init(const char* motor_tty, int baudrate,
					  const char* imu_tty,
                 uint16_t port, int timeout_ms)
{
	/*
	if(!initIMU(imu_tty))
		return false;
	
	if(!initMotors(motor_tty, baudrate))
	{
		closeIMU();
		return false;
	}
	*/
	if(!initNetwork(port, timeout_ms))
	{
		//closeIMU();
		//closeMotors();
		return false;
	}
	
	return true;
}

void Robot::close()
{
	closeNetwork();
	closeMotors();
	closeIMU();
}

void Robot::controlLoop()
{
	int error;
	const mlsp_frame *streamer_frame;

	while(m_keepWorking)
	{
		if( ( streamer_frame = mlsp_receive(m_streamer, &error) ) == NULL )
		{
			if(error == MLSP_TIMEOUT)
			{
				stopMotors(m_rc);
				continue;
			}
			break; //error
		}

		processDriveMessage(streamer_frame);
	}
	
	cerr << "robot: finished thread" << endl;
	
	stopMotors(m_rc);
}

void Robot::processDriveMessage(const mlsp_frame *streamer_frame)
{
	static int last_left=INT_MAX, last_right=INT_MAX;
	drive_packet command;

	if(streamer_frame->size != sizeof(drive_packet))
	{
		cerr << "robot: ignoring invalid size message" << endl;
		return;
	}

	memcpy(&command, streamer_frame->data, sizeof(drive_packet));

	if(command.command == KEEPALIVE)
		return;

	if(command.command != SET_SPEED)
	{
		cerr << "robot: unknown command: " << command.command << endl;
		return;
	}

	int16_t l = command.left, r = command.right;
	bool ok = true;

	if(l == last_left && r == last_right)
		return;

	ok &= roboclaw_speed_accel_m1m2(m_rc, FRONT_MOTOR_ADDRESS, r, l, MOTOR_ACCELERATION) == ROBOCLAW_OK;
	ok &= roboclaw_speed_accel_m1m2(m_rc, MIDDLE_MOTOR_ADDRESS, r, l,  MOTOR_ACCELERATION) == ROBOCLAW_OK;
	ok &= roboclaw_speed_accel_m1m2(m_rc, REAR_MOTOR_ADDRESS, r, l, MOTOR_ACCELERATION) == ROBOCLAW_OK;	

	if(ok)
		last_left = l, last_right = r;
	else
		cerr << "robot: failed to set motor speed, no reaction implemented" << endl;
}

void Robot::startThread()
{
	cout << "robot: starting thread" << endl;
	m_keepWorking = true;
	m_thread = thread(&Robot::controlLoop, this);
}

void Robot::stopThread()
{
	cout << "robot: stopping thread" << endl;
	if(!m_keepWorking)
		return;
	m_keepWorking = false;
	m_thread.join();
}

int get_encoders(roboclaw *rc, dead_reconning_packet *packet)
{
	int status;
	status=roboclaw_encoders(rc, MIDDLE_MOTOR_ADDRESS, &packet->position_left, &packet->position_right);

	if(status == ROBOCLAW_OK)
		return 0;

	if(status == ROBOCLAW_ERROR)
		cerr << "ccdrive: unable to read encoders" << endl;
	if(status == ROBOCLAW_RETRIES_EXCEEDED)
		cerr << "ccdrive: retries exceeded while reading encoders" << endl;
	return -1;
}

/*
void Robot::controlLoop()
{
	
	int status;	
	fd_set rfds;
	struct timeval tv;

	struct dead_reconning_packet odometry_packet;
	struct drive_packet drive_packet;
	uint64_t last_drive_packet_timestamp_us=TimestampUs();

	uint64_t TIMEOUT_US=500*1000;
	
	//temp
	int server_udp; //dummy
	int vmu_fd;//dummy
		
	while(m_keepWorking)
	{
		FD_ZERO(&rfds);
		FD_SET(server_udp, &rfds);
		FD_SET(vmu_fd(vmu), &rfds);

		tv.tv_sec=0;
		tv.tv_usec=1000*50; //50 ms hardcoded

		status=select(server_udp+1, &rfds, NULL, NULL, &tv);
		if(status == -1)
		{
			perror("ccdrive: select failed");
			break;
		}
		else if(status) //got something on udp socket
		{
			if( FD_ISSET(vmu_fd(vmu), &rfds) )
			{
				odometry_packet.timestamp_us = TimestampUs();
				if(GetQuaternion(vmu, &odometry_packet) == -1)
					break; //consider if it is possible to not get data
				if(GetEncoders(rc, &odometry_packet) == -1)
					break;
			}
			if( FD_ISSET(server_udp, &rfds) )
			{
				if( (status=RecvDrivePacket(server_udp, &drive_packet)) < 0 )
					break;
				if(status == 0)
				{
					StopMotors(rc);
					fprintf(stderr, "ccdrive: timeout reading from server, this should not happen!...\n");
				}

				ProcessMessage(drive_packet, rc);
				last_drive_packet_timestamp_us=TimestampUs();
			}
		}
		
		//check timeout
		if(TimestampUs() - last_drive_packet_timestamp_us > TIMEOUT_US)
		{
			StopMotors(rc);
			last_drive_packet_timestamp_us=TimestampUs(); //mark timestamp not to flood with messages
			fprintf(stderr, "ccdrive: timeout...\n");		
		}
		
		if( FD_ISSET(vmu_fd(vmu), &rfds) )
			SendDeadReconningPacketUDP(client_udp, destination_udp, odometry_packet);
		//printf("l=%d r=%d x=%f y=%f z=%f\n", odometry_packet.position_left, odometry_packet.position_right, odometry_packet.x, odometry_packet.y, odometry_packet.z);
			
		if(IsStandardInputEOF()) //the parent process has closed it's pipe end
			break;		
	}
	
}
*/
