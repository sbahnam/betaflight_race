
#include "common/maths.h"
#include <stdbool.h>

#define MAX_BANK_DEGREE 40
#define ATT_MAX_RATE_XY 500
#define ATT_MAX_RATE_Z 150

extern fp_quaternion_t attSpNed;
extern float zAccSpNed;
extern float yawRateSpNed;
extern bool attTrackYaw;
extern float u[4];

void indiController(void);

void getYawRateSpBody(void);
void getAttErrBody(void);
void getAlphaBody(void);
void getSpfSpBodyZ(void);
void getMotor(void);
