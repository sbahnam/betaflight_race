
#include "att_ctl.h"
#include "common/maths.h"

#include "drivers/motor.h"

#include <stdbool.h>

fp_quaternion_t attSpNed = {.qi=1};
bool attTrackYaw = true; 
float spfSpBodyZ;

fp_quaternion_t attErrBody = {.qi=1};
t_fp_vector rateSpBody = {0};
t_fp_vector alphaSpBody = {0};

t_fp_vector attGains = {.V.X = 80., .V.Y = 80., .V.Z = 35.};
t_fp_vector rateGains = {.V.X = 13., .V.Y = 13., .V.Z = 10.};

// todo: have to exchange this with the global fliht modes
typedef enum {
    ATT_SP_SOURCE_POS,
    ATT_SP_SOURCE_RC,
} att_sp_source_t;

att_sp_source_t attSpSource = ATT_SP_SOURCE_POS;

void getSpfBody(void) {
    // get body rates
    t_fp_vector rateEstBody = {
        .V.X = 0.,
        .V.Y = 0.,
        .V.Z = 0.,
    };

    // emulate parallel PD with cascaded (so we can limit velocity)
    t_fp_vector attGainsCasc = {
        .V.X = attGains.V.X / rateGains.V.X,
        .V.Y = attGains.V.Y / rateGains.V.Y,
        .V.Z = attGains.V.Z / rateGains.V.Z
    };

    // --- get rate setpoint
    // memset?
    rateSpBody.V.X = 0.;
    rateSpBody.V.Y = 0.;
    rateSpBody.V.Z = 0.;

    float angleErr = 2*acos(attErrBody.qi); // should never be above 1 or below -1
    if (angleErr > M_PIf)
        angleErr -= 2*M_PIf; // make sure angleErr is [-pi, pi]

    t_fp_vector attErr = {
        .V.X = attErrBody.qx, .V.Y = attErrBody.qy, .V.Z = attErrBody.qz};

    // final error is angle times axis
    VEC3_SCALAR_MULT(attErr, angleErr);

    // multiply with gains and constrain
    VEC3_ELEM_MULT_ADD(rateSpBody, attGainsCasc, attErr);
    VEC3_CONSTRAIN_XY_LENGTH(rateSpBody, DEGREES_TO_RADIANS(ATT_MAX_RATE_XY));
    rateSpBody.V.Z = constrainf(rateSpBody.V.Z, -DEGREES_TO_RADIANS(ATT_MAX_RATE_Z), -DEGREES_TO_RADIANS(ATT_MAX_RATE_Z));

    // get rotation acc setpoint simply by multiplying with gains
    // rateErr = rateSpBody - rateEstBody
    t_fp_vector rateErr = rateSpBody;
    VEC3_SCALAR_MULT_ADD(rateSpBody, -1.0, rateEstBody);
    VEC3_ELEM_MULT_ADD(alphaSpBody, rateGains, rateErr);

    // INDI!
}

void getMotor(void) {
    // mix! And call writeMotors or whatever

    // BIG ASS TODO: CHECK THAT GETTING STUCK IN WHILE LOOP CANNOT CAUSE FLYAWAYS!
}


void getAttErrBody(void) {
    // get attitude estimate and inverse
    fp_quaternion_t attEstNed = {
        .qi = 1.,
        .qx = 0.,
        .qy = 0.,
        .qz = 0.
    };
    fp_quaternion_t attEstNedInv = attEstNed;
    attEstNedInv.qi = -attEstNed.qi;

    // get error
    switch(attSpSource){
        case ATT_SP_SOURCE_POS:
            attErrBody = quatMult(attEstNedInv, attSpNed);
            break;
        case ATT_SP_SOURCE_RC: ;
            // slow, but good code clarity
            t_fp_vector bodyXNed = quatRotMatCol(attEstNed, 0);
            float bodyXProjLen = VEC3_XY_LENGTH(bodyXNed);
            t_fp_vector bodyYNed = quatRotMatCol(attEstNed, 1);
            float bodyYProjLen = VEC3_XY_LENGTH(bodyYNed);

            t_fp_vector stickXNed = {0};
            t_fp_vector stickYNed = {0};
            if (bodyXProjLen > bodyYProjLen) { // can never be 0 together
                // stickX
                stickXNed.V.X = bodyXNed.V.X / bodyXProjLen;
                stickXNed.V.Y = bodyXNed.V.Y / bodyXProjLen;
                // stickY
                stickYNed.V.X = -stickXNed.V.Y;
                stickYNed.V.Y = +stickXNed.V.X;
            } else {
                // stickY
                stickYNed.V.X = bodyYNed.V.X / bodyYProjLen;
                stickYNed.V.Y = bodyYNed.V.Y / bodyYProjLen;
                // stickX
                stickXNed.V.X = +stickYNed.V.Y;
                stickXNed.V.Y = -stickYNed.V.X;
            }

            // find tilt axis --> this can be done better! 
            // Now combined XY input gives bigger tilt angle before limiting,
            // which results in a sort of radial deadzone past the x*y=1 circle
            float RC[2] = {0., 0.}; //replace with actual RC. Assumed positive in body-xy and between -1 and 1
#define RC_MAX_TILT_DEG 20
            float rc_scaler = tanf(DEGREES_TO_RADIANS(RC_MAX_TILT_DEG));
            t_fp_vector bodyZspNed = {
                .V.X = rc_scaler * (-stickXNed.V.X * RC[0] + -stickYNed.V.X * RC[1]),
                .V.Y = rc_scaler * (-stickXNed.V.Y * RC[0] + -stickYNed.V.Y * RC[1]),
                .V.Z = 1.
            };
            VEC3_CONSTRAIN_XY_LENGTH(bodyZspNed, rc_scaler); // limit total tilt
            VEC3_SCALAR_MULT(bodyZspNed, 1/VEC3_LENGTH(bodyZspNed)); // normalize

            // construct shortest possible rotation quaternion from BodyZ to 
            // BodyZ setpoint
            // https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
            // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/halfAngle.htm

            t_fp_vector bodyZNed = quatRotMatCol(attEstNed, 2);
            float k_cos_theta = VEC3_DOT(bodyZNed, bodyZspNed);
            t_fp_vector axis;
            if (k_cos_theta <= -0.9999) {
                // we are upside down with respect to the target z axis.
                // --> choose some orthogonal axis and rotate 180 degrees.
                attErrBody.qi = 0;

                // in particular choose cross product with the most orthogonal
                // coordinate axis for speed.
                float x = fabsf(bodyZNed.V.X);
                float y = fabsf(bodyZNed.V.Y);
                float z = fabsf(bodyZNed.V.Z);
                int most_ortho = (x < y) ? ((x < z) ? 0 : 1) : ((y < z) ? 1 : 2);
                // axis = normalize( cross(bodyZNed, most_ortho) )
                switch(most_ortho){
                    case 0:
                        axis.V.X = 0.;
                        axis.V.Y = +bodyZNed.V.Z;
                        axis.V.Z = -bodyZNed.V.Y;
                        break;
                    case 1:
                        axis.V.X = -bodyZNed.V.Z;
                        axis.V.Y = 0.;
                        axis.V.Z = +bodyZNed.V.X;
                        break;
                    case 2:
                        axis.V.X = +bodyZNed.V.Y;
                        axis.V.Y = -bodyZNed.V.X;
                        axis.V.Z = 0.;
                        break;
                }
                VEC3_NORMALIZE(axis);
                attErrBody.qx = axis.V.X;
                attErrBody.qy = axis.V.Y;
                attErrBody.qz = axis.V.Z;
            } else {
                // normal case, just follow http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/halfAngle.htm
                // scalar = k_cos_theta + 1
                // axis = cross(bodyZNed, bodyZSpNed)
                // normalize the quat
                attErrBody.qi = k_cos_theta + 1.;
                VEC3_CROSS(axis, bodyZNed, bodyZspNed);
                attErrBody.qx = axis.V.X;
                attErrBody.qy = axis.V.Y;
                attErrBody.qz = axis.V.Z;
                QUAT_NORMALIZE(attErrBody);
            }
            break;
        default:
            break;
    }
}
