#include "robot.h"
#include "mlsp.h"

#include <Eigen/Geometry>

#include <iostream>
#include <thread>
#include <cmath> //M_PI
#include <climits> //INT_MAX
#include <cstring> //memcpy
#include <cassert> // assert
#include <sys/select.h> //select

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

struct odometry_packet
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

struct physics
{
	float wheel_diameter_mm;
	float encoder_counts_per_rotation;
};

const int DEAD_RECONNING_PACKET_BYTES=32; //8 + 2*4 + 4*4
const int CONTROL_PACKET_BYTES = 18; //8 + 5*2 = 18 bytes
enum Commands {KEEPALIVE=0, SET_SPEED=1, TO_POSITION_WITH_SPEED=2};

Robot::Robot(): m_position(0.0f,0.0f,0.0f), m_heading(1.0f, 0.0f, 0.0f, 0.0f)
{
}

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

	cout << "robot: battery voltage is" << voltage/10.0 << endl;

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

	return true;
}

void Robot::closeIMU()
{
	vmu_close(m_vmu);
}

int Robot::getQuaternion(float *w, float *x, float *y, float *z) const
{
	static vmu_twxyz quat_data[10];
	int status;

	while( (status = vmu_quat(m_vmu, quat_data, 10)) > 10 )
		; //burn through old readings to get the lastest

	if(status == VMU_ERROR)
	{
		cerr << "robot: failed to read imu data" << endl;
		return -1;
	}

	if(status == 0) //this should not happen
	{
		cerr << "robot: vmu status 0 WTF?" << endl;
		return -1;
	}

	//the last reading is the latest
	--status;

	//quaterion as in Unity coordinate system
	*w = quat_data[status].w;
	*x = quat_data[status].x;
	*y = quat_data[status].y;
	*z = quat_data[status].z;
	//*y = -quat_data[status].z;
	//*z = quat_data[status].y;

	return 0;
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

	if(!initIMU(imu_tty))
		return false;

	if(!initMotors(motor_tty, baudrate))
	{
		closeIMU();
		return false;
	}

	if(!initNetwork(port, timeout_ms))
	{
		closeIMU();
		closeMotors();
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
	int error, status;
	fd_set rfds;
	timeval tv;

	const int network_fd = mlsp_fd(m_streamer);
	const int imu_fd = vmu_fd(m_vmu);

	const mlsp_frame *streamer_frame;

	odometry_packet odometry;
	uint64_t command_timestamp_us = timestampUs();
	uint64_t TIMEOUT_US=500*1000; //hardcoded 500 msee

	while(m_keepWorking)
	{
		FD_ZERO(&rfds);
		FD_SET(network_fd, &rfds);
		FD_SET(imu_fd, &rfds);

		tv.tv_sec=0;
		tv.tv_usec=1000*50; //50 ms hardcoded

		status = select(network_fd + 1, &rfds, NULL, NULL, &tv);

		if(status == -1)
		{
			cerr << "robot: select failed, " << errno << endl;
			break;
		}
		else if(status)
		{
			if( FD_ISSET(imu_fd, &rfds) )
			{
				odometry.timestamp_us = timestampUs();
				if(getQuaternion(&odometry.w, &odometry.x, &odometry.y, &odometry.z) == -1)
					break; //consider if it is possible to not get data
				if(getEncoders(&odometry.position_left, &odometry.position_right) == -1)
					break;

				if(!m_timestamp)
				{
					m_timestamp = odometry.timestamp_us;
					m_left = odometry.position_left;
					m_right = odometry.position_right;
				}
			}

			if( FD_ISSET(network_fd, &rfds) )
			{
				if( ( streamer_frame = mlsp_receive(m_streamer, &error) ) == NULL )
				{
					if(error == MLSP_TIMEOUT)
					{
						cerr << "robot: timeout reading from network, this should not happen!" << endl;
						stopMotors(m_rc);
						continue;
					}
					break; //error
				}

				processDriveMessage(streamer_frame);
				command_timestamp_us = timestampUs();
			}
		}
		//status == 0 indicates timetout

		//check timeout
		if(timestampUs() - command_timestamp_us > TIMEOUT_US)
		{
			stopMotors(m_rc);
			command_timestamp_us=timestampUs(); //mark timestamp not to flood with messages
			cerr <<  "robot: timeout..." << endl;
		}

		if( FD_ISSET(imu_fd, &rfds) )
		{
			cout << "l=" << odometry.position_left << " r=" << odometry.position_right << endl;
			odometryUpdate(odometry.position_left, odometry.position_right,
			               odometry.w, odometry.x, odometry.y, odometry.z,
								odometry.timestamp_us);
		}

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

int Robot::getEncoders(int32_t *left, int32_t *right)
{
	int status = roboclaw_encoders(m_rc, MIDDLE_MOTOR_ADDRESS, left, right);

	if(status == ROBOCLAW_OK)
		return 0;

	if(status == ROBOCLAW_ERROR)
		cerr << "robot: unable to read encoders" << endl;
	if(status == ROBOCLAW_RETRIES_EXCEEDED)
		cerr << "robot: retries exceeded while reading encoders" << endl;

	return -1;
}

uint64_t Robot::timestampUs() const
{
	timespec ts;
	assert(clock_gettime(CLOCK_MONOTONIC, &ts) == 0);
	return (uint64_t)ts.tv_sec*1000000 + (uint64_t)ts.tv_nsec / 1000;
}

void Robot::odometryUpdate(int32_t left, int32_t right,
	                        float w, float x, float y, float z,
									uint64_t timestamp_us)
{
	physics p {120.0f, 1196.8f};
	const float MM_IN_M=1000.0f;

	// Calculate the linear displacement since last packet
	float distance_per_encoder_count_mm = M_PI * p.wheel_diameter_mm / p.encoder_counts_per_rotation;
	float ldiff = left - m_left;
	float rdiff = right - m_right;

	float displacement_m = (ldiff + rdiff) * distance_per_encoder_count_mm / 2.0f / MM_IN_M;

	Eigen::Quaternionf heading_new(w, x, y, z);
	//Eigen::Quaternionf heading_avg = m_heading.slerp(0.5f, heading_new);

	Eigen::Vector3f forward(0.0f, 1.0f, 0.0f);
	Eigen::Vector3f ahead = heading_new * forward;

	cout << "forward = " << forward << endl;
	cout << "ahead = " << ahead << endl;

	Eigen::Vector3f displacement = displacement_m * ahead;

	m_position = m_position + displacement;
	m_heading = heading_new;

	m_left = left;
	m_right = right;
	m_timestamp = timestamp_us;

	//cout << "position = " << m_position << endl;
	//cout << "heading = " << m_heading.w() << " orientation " << m_heading.vec() << endl;
}
