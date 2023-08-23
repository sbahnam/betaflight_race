
#include "drivers/time.h"
#include "common/maths.h"

typedef enum {
    EXT_POS_NO_SIGNAL,
    EXT_POS_STILL_VALID,
    EXT_POS_NEW_MESSAGE,
} ext_pos_state_t;

// todo: reformulate using t_fp_vector
typedef struct __ext_pos_ned_t {
    t_fp_vector pos;
    t_fp_vector vel;
    float psi;
} ext_pos_ned_t;

extern ext_pos_ned_t extPosNed;
extern ext_pos_state_t extPosState;

#define EXT_POS_FREQ 50
#define EXT_POS_TIMEOUT_US 300000

void checkNewPos(void);
void getExternalPos(timeUs_t current);



