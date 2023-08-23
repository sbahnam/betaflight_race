

#include "io/external_pos.h"
#include "flight/att_ctl.h"
#include "common/maths.h"

#include "pos_ctl.h"

// --- control variables
// externs
t_fp_vector posSpNed = {.V.X = 0., .V.Y = 0., .V.Z = -1.};
t_fp_vector velSpNed = {.V.X = 0., .V.Y = 0., .V.Z = 0.};
t_fp_vector accSpNed = {.V.X = 0., .V.Y = 0., .V.Z = 0.};

// locals
float yawSpNedRad = 0.;

// position controller configuration
// todo DONE: split in horizontal/vertical, not XYZ.. thats kinda meaningless with yaw
float posHGainP = 1.0;
float posHGainD = 1.5;
float posVGainP = 2.5;
float posVGainD = 2.5;
float velSpLimitXY = 1.5;
float velSpLimitZ  = 0.5;
float weathervaneP = 0.;

void updatePosCtl(timeUs_t current) {
    UNUSED(current);

    if (extPosState == EXT_POS_NO_SIGNAL) {
        // panic and level craft
        attSpNed.qi = 1.;
        attSpNed.qx = 0.;
        attSpNed.qy = 0.;
        attSpNed.qz = 0.;
        return;
    }

    getAccSpNed();
    getAttSpNed();
}

void getAccSpNed(void) {
    // precalculations
    float accMax = tanf( DEGREES_TO_RADIANS( MAX_BANK_DEGREE ) );
    float posHGainPCasc = posHGainP / posHGainD; // emulate parallel PD with Casc system
    float posVGainPCasc = posVGainP / posVGainD;

    // pos error = pos setpoint - pos estimate
    t_fp_vector posError = posSpNed;
    VEC3_SCALAR_MULT_ADD(posError, -1.0, extPosNed.pos);
    // vel setpoint = posGains * posError
    velSpNed.V.X = posError.V.X * posHGainPCasc;
    velSpNed.V.Y = posError.V.Y * posHGainPCasc;
    velSpNed.V.Z = posError.V.Z * posVGainPCasc;

    // constrain magnitude here
    VEC3_CONSTRAIN_XY_LENGTH(velSpNed, velSpLimitXY);
    velSpNed.V.Z = constrainf(velSpNed.V.Z, -velSpLimitZ, +velSpLimitZ);

    // vel error = vel setpoint - vel estimate
    t_fp_vector velError = velSpNed;
    VEC3_SCALAR_MULT_ADD(velError, -1.0, extPosNed.vel);
    // acceleration setpoint = velGains * velError
    accSpNed.V.X = velError.V.X * posHGainD;
    accSpNed.V.Y = velError.V.Y * posHGainD;
    accSpNed.V.Z = velError.V.Z * posVGainD;

    // limit such that max acceleration likely results in bank angle below 40 deg
    VEC3_CONSTRAIN_XY_LENGTH(accSpNed, accMax);
    accSpNed.V.Z = constrainf(accSpNed.V.Z, MAX_ACC_Z_NEG, MAX_ACC_Z_POS);

    // todo: how to handle course/yaw?
    // todo: weathervaning
    // todo: transform acc measurement from body to global and use INDI on acc
}

void getAttSpNed(void) {
    // quaternion magic here

    // Approach 1: we know that:
    // z-axis has to align with negative thrust axis, which is something like
    //    thrust_axis = (accSpNed.V.X, accSpNed.V.Y, -1 )
    //    thrust_factor = len(thrust_axis)
    //    attSp_z       = -thrust_axis / thrust_factor;
    // 
    // we define aero_x to have angle yawSpNedRad with north, and perp-to aero_z
    //    aero_x = ( cos(yawSpNedRad), sin(yawSpNedRad), 0)
    //    aero_y = (-sin(yawSpNedRad), cos(yawSpNedRad), 0)
    //    aero_z = (0 0 1)
    // 
    // define attSp_x to also be in the plane that includes north + yawSpNedRad
    //    attSp_x       = cross(aero_y, attSp_z) / len(cross(aero_y, attSp_z))

    // other approach, without requiring attSp_x to be pointed towards yawSpNedRad
    //    attSp = z-rotation(yawSpNedRad)  *  (-accSpNed.V.Y, accNed.V.X, 0)-rotation(atan( hypot(accSpNed.V.X, accSpNed.V.Y) ))
    //    attSp =       yaw                *              tilt

    fp_quaternion_t sp_yaw = {
        .qi = cosf(yawSpNedRad/2),
        .qx = 0.,
        .qy = 0.,
        .qz = sinf(yawSpNedRad/2)
    };

    float acc_mag = VEC3_XY_LENGTH(accSpNed);
    float tilt_angle = 0.;
    t_fp_vector tilt_axis = {0};

    if (acc_mag > 1e-6) {
        tilt_angle = atanf(acc_mag);
        // define axis such that:
        //     positive rotation through tilt angle aligns thrust axis correctly
        tilt_axis.V.X = accSpNed.V.Y/acc_mag;
        tilt_axis.V.Y = -accSpNed.V.X/acc_mag;
    }

    fp_quaternion_t sp_tilt = {
        .qi = cosf(tilt_angle/2),
        .qx = tilt_axis.V.X * sinf(tilt_angle/2),
        .qy = tilt_axis.V.Y * sinf(tilt_angle/2),
        .qz = 0.
    };

    attSpNed = quatMult(sp_tilt, sp_yaw);

    spfSpBodyZ = (-9.81 + accSpNed.V.Z) / cosf(tilt_angle);
    // note: this only gives offset free pos control, if spfSpBodyZ is reached 
    //       offset-free, ie through INDI
    // note2: if accSpNed.V.Z is non-zero, then accSpNed is over or undershot.
    //        Can we compromise on both at the same time somehow? or adjust 
    //        tilt angle setpoint based on accSpNed? There will be a system of 
    //        constrained nl equations that can be solved within bounds.

}
