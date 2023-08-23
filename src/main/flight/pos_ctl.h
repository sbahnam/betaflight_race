
#include "common/maths.h"

#define MAX_ACC_Z_NEG -15 // around 1.5g
#define MAX_ACC_Z_POS +9.81 // exactly 1g, for obvious reasons we cant do much more

extern t_fp_vector posSpNed;
extern t_fp_vector velSpNed;
extern t_fp_vector accSpNed;

void getAccSpNed(void);
void getAttSpNed(void);
