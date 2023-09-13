
#include "pi-messages.h"
#include "external_pos.h"
#include "common/maths.h"
#include "drivers/time.h"
#include "common/time.h"

//extern
ext_pos_ned_t extPosNed;
ext_pos_state_t extPosState = EXT_POS_NO_SIGNAL;

//local
timeUs_t latestMsgTime = 0;

void checkNewPos(void) {
    if (piMsgExternalPoseRxState < PI_MSG_RX_STATE_NONE) {
        // data (already) message available
        timeUs_t currentMsgTime = piMsgExternalPoseRx->time_ms*1e3;
        timeDelta_t deltaMsgs = cmpTimeUs(currentMsgTime, latestMsgTime);
        if (deltaMsgs != 0) {
        //if (deltaMsgs > 0) {
            // new message available
            // todo: how to catch stuck values? Like new messages comming in,
            // but the values are frozed, which is also basically signal loss
            extPosState = EXT_POS_NEW_MESSAGE;
            latestMsgTime = currentMsgTime;
        //} else if (deltaMsgs) < 0) {
            // I think this may happen if there is a 35 to 70min delay between
            // messages (or multiples of that). This will cause the difference 
            // calculations to roll-over (expected behaviour)
            // panic for one call, but update latestMsgTime
        //    extPosState = EXT_POS_NO_SIGMAL;
        //    latestMsgTime = currentMsgTime;
        //    return;
        } else {
            // assume still valid for now
            extPosState = EXT_POS_STILL_VALID;
        }

        // regardless of new or old message, we may have timeout
        timeDelta_t delta = cmpTimeUs(microsISR(), latestMsgTime);
        if (delta > EXT_POS_TIMEOUT_US) {
            // signal lost
            extPosState = EXT_POS_NO_SIGNAL;
        }
    } else {
        // data (noy yet) message available
        extPosState = EXT_POS_NO_SIGNAL;
    }
}

void getExternalPos(timeUs_t current) {

    UNUSED(current);

    checkNewPos();
    if (extPosState == EXT_POS_NO_SIGNAL)
        return;

    if (extPosState == EXT_POS_NEW_MESSAGE) {
        // process new message into NED
        extPosNed.pos.V.X = piMsgExternalPoseRx->enu_y;
        extPosNed.pos.V.Y = piMsgExternalPoseRx->enu_x;
        extPosNed.pos.V.Z = -piMsgExternalPoseRx->enu_z;
        extPosNed.vel.V.X = piMsgExternalPoseRx->enu_yd;
        extPosNed.vel.V.Y = piMsgExternalPoseRx->enu_xd;
        extPosNed.vel.V.Z = -piMsgExternalPoseRx->enu_zd;
        fp_angles_t eulers;
        fp_quaternion_t quat;
        quat.qi = piMsgExternalPoseRx->body_qi;
        quat.qx = piMsgExternalPoseRx->body_qx;
        quat.qy = piMsgExternalPoseRx->body_qy;
        quat.qz = piMsgExternalPoseRx->body_qz;
        float_eulers_of_quat(&eulers, &quat);
        extPosNed.psi = -eulers.angles.yaw;
    }

    // extrapolate position with speed info
    timeDelta_t delta = cmpTimeUs(microsISR(), latestMsgTime);
    if (delta > 0) {
        extPosNed.pos.V.X += ((float) delta * 1e-6f) * extPosNed.vel.V.X;
        extPosNed.pos.V.Y += ((float) delta * 1e-6f) * extPosNed.vel.V.Y;
        extPosNed.pos.V.Z += ((float) delta * 1e-6f) * extPosNed.vel.V.Z;
        // todo: extrapolate psi from body rates somehow using the quat
    }
}
