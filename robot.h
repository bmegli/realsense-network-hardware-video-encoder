#include "roboclaw.h"
#include "vmu931.h"
#include "mlsp.h"

#include <stdint.h>
#include <thread>

class Robot
{
public:
	bool init(const char *motor_tty, int baudrate,
	          const char *imu_tty,
	          uint16_t port, int timeout_ms);
	void close();

	void startThread();
	void stopThread();

	void controlLoop();

private:
	roboclaw *m_rc = nullptr;
	vmu *m_vmu = nullptr;
	mlsp *m_streamer = nullptr;

	std::thread m_thread;
	bool m_keepWorking = false;

	bool initMotors(const char *tty, int baudrate);
	void closeMotors();
	void stopMotors(roboclaw *rc);
	int getEncoders(int32_t *left, int32_t *right);

	bool initIMU(const char *tty);
	void closeIMU();
	int getQuaternion(float *w, float *x, float *y, float *z) const;

	bool initNetwork(uint16_t port, int timeout_ms);
	void closeNetwork();

	void processDriveMessage(const mlsp_frame *msg);

	uint64_t timestampUs() const;
};
