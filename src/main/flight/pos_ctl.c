

#include "io/external_pos.h"
#include "flight/att_ctl.h"
#include "common/maths.h"

#include "pos_ctl.h"

t_fp_vector posSpNed = {.V.X = 0., .V.Y = 0., .V.Z = -1.};
t_fp_vector velSpNed = {.V.X = 0., .V.Y = 0., .V.Z = 0.};
t_fp_vector accSpNed = {.V.X = 0., .V.Y = 0., .V.Z = 0.};

// position controller configuration
t_fp_vector posGains = {.V.X = 1., .V.Y = 1., .V.Z = 0.5};
t_fp_vector velSpLimits = {.V.X = 1., .V.Y = 1., .V.Z = 0.5};
float weathervaneP = 0.;

// precalculations
float accMax = tanf( DEGREES_TO_RADIAN( MAX_BANK_DEGREE ) );

// probably code bloat... convert to static functions?
#define VEC3_SCALAR_MULT_ADD(_orig, _add, _sc) { \
    _orig.V.X += _sc * _add.V.X; \
    _orig.V.Y += _sc * _add.V.Y; \
    _orig.V.Z += _sc * _add.V.Z; \
}

#define VEC3_ELEM_MULT_ADD(_orig, _add, _gains) { \
    _orig.V.X += _gains.V.X * _add.V.X; \
    _orig.V.Y += _gains.V.Y * _add.V.Y; \
    _orig.V.Z += _gains.V.Z * _add.V.Z; \
}

#define VEC3_SCALAR_MULT(_orig, _sc) { \
    _orig.V.X *= _sc; \
    _orig.V.Y *= _sc; \
    _orig.V.Z *= _sc; \
    }

#define VEC3_LENGTH(_orig) { \
    sqrtf( \ 
        _orig.V.X*_orig.V.X \
        + _orig.V.Y*_orig.V.Y \
        + _orig.V.Z*_orig.V.Z \
    ) \
}

#define VEC3_XY_LENGTH(_orig) { \
    sqrtf( //rather do hypotf? \ 
        _orig.V.X*_orig.V.X \
        + _orig.V.Y*_orig.V.Y \
    ) \
}

void getAccSpNed() {
    // pos error = pos setpoint - pos estimate
    t_fp_vector posError = posSpNed;
    VEC3_SCALAR_MULT_ADD(posError, -1.0, extPosNed.pos);

    // vel error = vel setpoint - vel estimate
    t_fp_vector velError = velSpNed;
    VEC3_SCALAR_MULT_ADD(velError, -1.0, extPosNed.vel);

    // ----- implement output limited P controller here, keep it simple
    // acceleration setpoint = posGains * posError + velGains * velError
    VEC3_ELEM_MULT_ADD(accSpNed, posGains, posError);
    VEC3_ELEM_MULT_ADD(accSpNed, velGains, velError);

    // limit such that max acceleration likely results in bank angle below 40 deg
    float accTotal = VEC3_XY_LENGTH(accSpNed);
// todo: fix this shit
#define BIG_FLOAT 1e100
    float limiter = constrainf(accTotal/accMax, 1, +BIG_FLOAT);
    accSpNed.V.X /= limiter; // can never be 0 due to constrainf
    accSpNed.V.Y /= limiter; // can never be 0 due to constrainf
    accSpNed.V.Z = constrainf(accSpNed.V.Z, MAX_ACC_Z_NEG, MAX_ACC_Z_POS);

    // todo: how to handle course/yaw?
    // todo: weathervaning
}

void getAttSp() {
    // quaternion magic here

    // rotate accSp into aero frame (body frame, but with z down somehow, this
    // is where the fuckery comes in), then allocate
    accSpBody = vectorRotate(accSpNed, aero_quat);



}
