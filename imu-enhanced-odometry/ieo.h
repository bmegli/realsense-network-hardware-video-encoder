#ifndef IEO_H
#define IEO_H

#include <Eigen/Geometry>

#include <cstdint>

struct IEOPose
{
	uint64_t timestamp_us;
	float position_xyz[3]; //vector
	float heading_xyzw[4]; //quaternion
};

class IEO
{
public:
	IEO(float wheel_diameter_mm, float encoder_counts_per_rotation);

	void update(int32_t left, int32_t right,
	            float w, float x, float y, float z,
				   uint64_t timestamp_us);

	static uint64_t timestampUs();
	
	IEOPose getPose() const;
private:
	const float m_wheel_diameter_mm;
	const float m_encoder_counts_per_rotation;
	
	int32_t m_left = 0;
	int32_t m_right = 0;
	uint64_t m_timestamp = 0;

	Eigen::Vector3f m_position;
	Eigen::Quaternionf m_heading;
};

#endif //IEO_H
