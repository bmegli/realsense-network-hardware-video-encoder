#include "ieo.h"

#include <Eigen/Geometry>

#include <iostream>
#include <ctime> //clock_gettime
#include <cmath> //M_PI
#include <cassert> // assert

using namespace std;

IEO::IEO(float wheel_diameter_mm, float encoder_counts_per_rotation):
	m_wheel_diameter_mm(wheel_diameter_mm),
	m_encoder_counts_per_rotation(encoder_counts_per_rotation),
	m_position(0.0f,0.0f,0.0f),
	m_heading(1.0f, 0.0f, 0.0f, 0.0f)
{
}

uint64_t IEO::timestampUs()
{
	timespec ts;
	assert(clock_gettime(CLOCK_MONOTONIC, &ts) == 0);
	return (uint64_t)ts.tv_sec*1000000 + (uint64_t)ts.tv_nsec / 1000;
}

void IEO::update(int32_t left, int32_t right,
	                        float w, float x, float y, float z,
									uint64_t timestamp_us)
{
	if(!m_timestamp)
	{
		m_timestamp = timestamp_us;
		m_left = left;
		m_right = right;
	}
	
	const float MM_IN_M=1000.0f;

	// Calculate the linear displacement since last packet
	float distance_per_encoder_count_mm = M_PI * m_wheel_diameter_mm / m_encoder_counts_per_rotation;
	float ldiff = left - m_left;
	float rdiff = right - m_right;

	float displacement_m = (ldiff + rdiff) * distance_per_encoder_count_mm / 2.0f / MM_IN_M;

	Eigen::Quaternionf heading_new(w, x, y, z);
	//Eigen::Quaternionf heading_avg = m_heading.slerp(0.5f, heading_new);

	Eigen::Vector3f forward(0.0f, 1.0f, 0.0f);
	Eigen::Vector3f ahead = heading_new * forward;

	Eigen::Vector3f displacement = displacement_m * ahead;

	m_position = m_position + displacement;
	m_heading = heading_new;

	m_left = left;
	m_right = right;
	m_timestamp = timestamp_us;

	//cout << "position = " << m_position << endl;
}

IEOPose IEO::getPose() const
{
	IEOPose pose {m_timestamp,
					 {m_position[0], m_position[1], m_position[2]},
	             {m_heading.x(), m_heading.y(), m_heading.z(), m_heading.w()} };
	return pose;
}
