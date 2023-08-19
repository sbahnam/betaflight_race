
#include "common/maths.h"
#include <stdbool.h>

#define MAX_BANK_DEGREE 40
#define MAX_ACC_Z_NEG -15 // around 1.5g
#define MAX_ACC_Z_POS +9.81 // exactly 1g, for obvious reasons we cant do much more

extern t_fp_quaternion attSpNed;
extern bool attTrackYaw;
