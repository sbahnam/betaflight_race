
#include "common/maths.h"
#include <stdbool.h>

#include "platform.h"
#include "flight/pid.h"

#define MAX_BANK_DEGREE 40.f
#define ATT_MAX_RATE_XY 500.f
#define ATT_MAX_RATE_Z 150.f

#define MAXU MAX_SUPPORTED_MOTORS
#define MAX_SUPPORTED_PSEUDOCONTROLS 6
#define MAXV MAX_SUPPORTED_PSEUDOCONTROLS

extern fp_quaternion_t attSpNed;
extern float zAccSpNed;
extern float yawRateSpNed;
extern bool attTrackYaw;
extern float u[MAXU];

void indiInit(const pidProfile_t * pidProfile);
void indiController(void);
float indiThrustLinearization(float in);
float indiThrustCurve(float in);

void getYawRateSpBody(void);
void getAttErrBody(void);
void getAlphaBody(void);
void getSpfSpBodyZ(void);
void getMotor(void);
