
#include "att_ctl.h"
#include "common/maths.h"
#include "common/axis.h"

#include "sensors/gyro.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "flight/imu.h"
#include "drivers/motor.h"

#include <stdbool.h>

#define RC_SCALE_STICKS 0.002
#define RC_SCALE_THROTTLE 0.001
#define RC_OFFSET_THROTTLE 1000
#define RC_MAX_YAWRATE_DEG_S 150
#define RC_MAX_SPF_Z -30

fp_quaternion_t attSpNed = {.qi=1};
bool attTrackYaw = true;
float zAccSpNed;
float yawRateSpNed;
float u[4] = {0,0,0,0};

fp_quaternion_t attErrBody = {.qi=1};
t_fp_vector rateSpBody = {0};
t_fp_vector alphaSpBody = {0};
t_fp_vector yawRateSpBody = {0};
float zSpfSpBody = 0.;

t_fp_vector attGains = {.V.X = 800., .V.Y = 800., .V.Z = 200.};
t_fp_vector rateGains = {.V.X = 13., .V.Y = 13., .V.Z = 10.};

void indiController(void) {
    getAlphaBody();
    getSpfSpBodyZ();

    // allocation and INDI
    getMotor();
}

void getAlphaBody(void) {

    // get body rates in z-down, x-fwd frame
    t_fp_vector rateEstBody = {
        .V.X = DEGREES_TO_RADIANS(gyro.gyroADCf[FD_ROLL]), 
        .V.Y = DEGREES_TO_RADIANS(-gyro.gyroADCf[FD_PITCH]),
        .V.Z = DEGREES_TO_RADIANS(-gyro.gyroADCf[FD_YAW]),
    };

    // emulate parallel PD with cascaded (so we can limit velocity)
    t_fp_vector attGainsCasc = {
        .V.X = attGains.V.X / rateGains.V.X,
        .V.Y = attGains.V.Y / rateGains.V.Y,
        .V.Z = attGains.V.Z / rateGains.V.Z
    };

    // --- get body rates setpoint
    // initialize to yaw-rate (transform) from higher level controllers/sticks
    getYawRateSpBody();
    rateSpBody = yawRateSpBody;

    // add in the error term based on attitude quaternion error
    getAttErrBody(); // sets attErrBody

    float angleErr = 2*acos(attErrBody.qi); // should never be above 1 or below -1
    if (angleErr > M_PIf)
        angleErr -= 2*M_PIf; // make sure angleErr is [-pi, pi]

    t_fp_vector attErrAxis = {
        .V.X = attErrBody.qx, .V.Y = attErrBody.qy, .V.Z = attErrBody.qz};

    // final error is angle times axis
    VEC3_SCALAR_MULT(attErrAxis, angleErr);

    // multiply with gains and constrain
    VEC3_ELEM_MULT_ADD(rateSpBody, attGainsCasc, attErrAxis);
    VEC3_CONSTRAIN_XY_LENGTH(rateSpBody, DEGREES_TO_RADIANS(ATT_MAX_RATE_XY));
    rateSpBody.V.Z = constrainf(rateSpBody.V.Z, -DEGREES_TO_RADIANS(ATT_MAX_RATE_Z), DEGREES_TO_RADIANS(ATT_MAX_RATE_Z));

    // --- get rotation acc setpoint simply by multiplying with gains
    // rateErr = rateSpBody - rateEstBody
    t_fp_vector rateErr = rateSpBody;
    VEC3_SCALAR_MULT_ADD(rateErr, -1.0, rateEstBody);
    alphaSpBody.V.X = 0.;
    alphaSpBody.V.Y = 0.;
    alphaSpBody.V.Z = 0.;
    VEC3_ELEM_MULT_ADD(alphaSpBody, rateGains, rateErr);

}

void getMotor(void) {
    // mix! And call writeMotors or whatever

    // python code:
    /*
import numpy as np
GRAVITY = 9.81

m = 0.37

# calc inertia
# oscillation period
P = 0.64 # sec
R = 0.153/2
IzzRL = (m*GRAVITY*R*P*P) / (4*np.pi*np.pi)
Izz = IzzRL - m*R*R
# Izz = 0.677e-3

Ixx = 0.6 * Izz
Iyy = 0.8 * Izz
I = np.diag([Ixx, Iyy, Izz])

# we know that we can do 4g vertical acc, so load factor 4. Divide by 4 motors
Tmax = m*GRAVITY*4 / 4

# moment coefficient
CM = 0.02

# rotation directions (positive-z down)
direc = [1, -1, 1, -1]

width = 0.14
length = 0.10

X = .5 * np.array([[length, -width, 0.], [length, width, 0.], [-length, width, 0.], [-length, -width, 0.]])
T = np.array([0., 0., -Tmax])
G = np.zeros((4, 4))
GF = np.zeros((1, 4))
GM = np.zeros((3, 4))

for i in range(4):
    GF[0, i] = T[2]
    GM[:, i] = np.cross(X[i, :], T)
    GM[:, i] += -T*CM*direc[i] # negative because reaction force

G[0, :] = GF / m
G[1:, :] = np.linalg.inv(I) @ GM

Ginv = np.linalg.pinv(G)

print(Ginv)
    */

    // TODO: just use activeSetSolve
    // TODO: scale with currentPidProfile->motor_output_limit / 100.0f

    // FL, FR, RR, RL
    float Ginv[4][4] = {
        {-0.0254842,   0.00042246,  0.0007886,   0.00246437},
        {-0.0254842,  -0.00042246,  0.0007886,  -0.00246437},
        {-0.0254842,  -0.00042246, -0.0007886,   0.00246437},
        {-0.0254842,   0.00042246, -0.0007886,  -0.00246437},
    };

    // Fz, Mx, My, Mz
    float v[4] = {zSpfSpBody, alphaSpBody.V.X, alphaSpBody.V.Y, alphaSpBody.V.Z};

    // FL, FR, RR, RL
    //float u[4]; // is not an extern

    // INDI!

    // Ginv * v and then constrain between 0 and 1
    for (int i=0; i < 4; i++) {
        float accumulate = Ginv[i][0] * v[0];
        for (int j=1; j < 4; j++)
            accumulate += Ginv[i][j] * v[j];
        u[i] = constrainf(accumulate, 0., 1.);
    }

    // BIG ASS TODO: CHECK THAT GETTING STUCK IN WHILE LOOP CANNOT CAUSE FLYAWAYS!
}

void getYawRateSpBody(void) {
    /*
    if (attTrackYaw) {
        // just track yaw setpoint from attitude quaternion setpoint
        // no need to deal with anything
        yawRateSpBody.V.X = 0.;
        yawRateSpBody.V.Y = 0.;
        yawRateSpBody.V.Z = 0.;
        return;
    }
    */
    // get yawRateSpNed, unless assumed set by position controller
    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
        // get from RC
        yawRateSpNed = getSetpointRate(YAW) * RC_SCALE_STICKS;
        yawRateSpNed *= DEGREES_TO_RADIANS(RC_MAX_YAWRATE_DEG_S);
    }
    // no else... just assume that yawRateSpNed has been set properly by the posiotion controller

    // convert yawRateSpNed to Body
    // todo: this local is defined twice.. make static somehow
    fp_quaternion_t attEstNedInv = {
        .qi = -attitude_q.w,
        .qx = attitude_q.x,
        .qy = attitude_q.y,
        .qz = attitude_q.z,
    };
    yawRateSpBody = quatRotMatCol(attEstNedInv, 2);
    VEC3_SCALAR_MULT(yawRateSpBody, yawRateSpNed);
}

void getSpfSpBodyZ(void) {
    // similar to getYawRateSpBody, decide which Ned z accel to use and transform

    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
        // get from RC directly
        zSpfSpBody = (rcCommand[THROTTLE] - RC_OFFSET_THROTTLE);
        zSpfSpBody *= RC_SCALE_THROTTLE * RC_MAX_SPF_Z;
    } else {
        // assume that zAccSpNed has been set by position controller
        zSpfSpBody = (-9.81 + zAccSpNed);

        float cos_tilt_angle = getCosTiltAngle();
        if ((cos_tilt_angle < 0.5) && (cos_tilt_angle > 0.)) {
            // high tilt, but not inverted, limit divisor to 0.5
            cos_tilt_angle = 0.5;
        } else if (cos_tilt_angle <= 0.) {
            // inverted: disable throttle correction
            cos_tilt_angle = 1.0;
        }

        zSpfSpBody /= cos_tilt_angle;
        // note: this only gives offset free pos control, if spfSpBodyZ is reached 
        //       offset-free, ie through INDI
        // note2: if accSpNed.V.Z is non-zero, then accSpNed is over or undershot.
        //        Can we compromise on both at the same time somehow? or adjust 
        //        tilt angle setpoint based on accSpNed? There will be a system of 
        //        constrained nl equations that can be solved within bounds.

        // this should probably be in pos_ctl, because of the tradeoff... another allocation step?
    }
}

void getAttErrBody(void) {
    // get attitude estimate and inverse
    // todo: rewrite with the quaternion type defined in imu
    fp_quaternion_t attEstNed = {
        .qi = attitude_q.w,
        .qx = attitude_q.x,
        .qy = -attitude_q.y,
        .qz = -attitude_q.z,
    };
    fp_quaternion_t attEstNedInv = attEstNed;
    attEstNedInv.qi = -attEstNed.qi;

    // get error
    if (FLIGHT_MODE(POS_CTL_MODE)) {
        attErrBody = quatMult(attEstNedInv, attSpNed);
    } else if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {

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
        float roll = getSetpointRate(ROLL) * RC_SCALE_STICKS;
        float pitch = getSetpointRate(PITCH) * RC_SCALE_STICKS;
#define RC_MAX_TILT_DEG 20
        float rc_scaler = tanf(DEGREES_TO_RADIANS(RC_MAX_TILT_DEG));
        t_fp_vector bodyZspNed = {
            .V.X = rc_scaler * (-stickXNed.V.X * pitch + -stickYNed.V.X * roll),
            .V.Y = rc_scaler * (-stickXNed.V.Y * pitch + -stickYNed.V.Y * roll),
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
    } else {
        // not sure what happened, just command upright
        attSpNed.qi = 1.;
        attSpNed.qx = 0.;
        attSpNed.qy = 0.;
        attSpNed.qz = 0.;
        attErrBody = quatMult(attEstNedInv, attSpNed);
    }
}
