
#include "common/maths.h"
#include <stdbool.h>

#define MAX_BANK_DEGREE 40
#define ATT_MAX_RATE_XY 500
#define ATT_MAX_RATE_Z 150

extern fp_quaternion_t attSpNed;
extern float spfSpBodyZ;
extern bool attTrackYaw;

void overrideAttSpNedFromRc(void);
void getSpfBody(void);
void getMotor(void);
