# flake8: noqa
import sys

import numpy as np


def load_hrpsys_log_astate(astate_path, angle_unit="deg"):
    """LOAD_ASTATE Load the actual state values (openhrp log)
    A = LOAD_ASTATE(PATH_TO_ASTATE, ANGLE_UNIT) loads the actual values in
    separate cells in A using ANGLE_UNIT ('rad' or 'deg') for the joint
    angles. If ANGLE_UNIT is not specified 'deg' is the default.
    """

    astate = np.loadtxt(astate_path, comments="%")
    if astate.shape[1] != 176:
        sys.exit([astate_path, " is not a valid " "*-astate.log" " file"])

    m = astate.shape[0]
    dt = "f4"
    a = np.zeros(
        m,
        dtype=[
            ("enc", dt, 40),
            ("torque", dt, 40),
            ("ss", dt, 40),
            ("forceRL", dt, 6),
            ("forceLL", dt, 6),
            ("forceRA", dt, 6),
            ("forceLA", dt, 6),
            ("acc", dt, 6),
            ("gyro", dt, 3),
            ("qrot", dt, 9),
            ("waist_p", dt, 3),
            ("waist_rot", dt, 9),
            ("time", dt, 1),
            ("controltimev2", dt, 1),
            ("type", "a6"),
            ("ang_type", "a3"),
        ],
    )
    a["enc"] = astate[:, 0:40]
    # JA0-JA39
    a["torque"] = astate[:, 40:80]
    # TQ0-TQ39
    a["ss"] = astate[:, 80:120]
    # SS0-SS39
    a["forceRL"] = astate[:, 120:126]
    # FX0-MZ0
    a["forceLL"] = astate[:, 126:132]
    # FX1-MZ1
    a["forceRA"] = astate[:, 132:138]
    # FX2-MZ2
    a["forceLA"] = astate[:, 138:144]
    # FX3-MZ3
    a["acc"] = astate[:, 144:150]
    # AX0,AY0,AZ0,AX1,AY1,AZ1
    a["gyro"] = astate[:, 150:153]
    # WX0,WY0,WZ0
    a["qrot"] = astate[:, 153:162]
    # QX0,QY0,QZ0,QW0 (rot matrix)
    a["waist_p"] = astate[:, 162:165]
    # waistPX-waistPZ
    a["waist_rot"] = astate[:, 165:174]
    # waistQX-waistQW (rot matrix)
    a["time"] = astate[:, 174]
    # clockv2
    a["controltimev2"] = astate[:, 175]
    # ControlTimeV2

    a["type"] = "astate"
    a["ang_type"] = "rad"

    if angle_unit == "deg":
        a["enc"] = 180 / np.pi * a["enc"]
        # Joint angles in degrees
        a["ang_type"] = "deg"
        print("Warning: enc angles (astate) in degrees!")

    return a


def load_hrpsys_log_rstate(rstate_path, angle_unit="deg"):
    """% LOAD_RSTATE Load the reference state values (openhrp log)
    R = LOAD_RSTATE(RSTATE_LOG_PATH, ANGLE_UNIT) loads the reference values in
    separate cells in R using ANGLE_UNIT ('rad' or 'deg') for the joint
    angles. If ANGLE_UNIT is not specified 'deg' is the default.
    """

    rstate = np.loadtxt(rstate_path, comments="%")
    if rstate.shape[1] != 100:
        sys.exit([rstate_path, " is not a valid " "*-rstate.log" " file"])

    m = rstate.shape[0]
    dt = "f4"
    r = np.zeros(
        m,
        dtype=[
            ("enc", dt, 40),
            ("jvelocity", dt, 40),
            ("acc", dt, 3),
            ("zmp", dt, 3),
            ("waist_p", dt, 3),
            ("waist_rot", dt, 9),
            ("time", dt, 1),
            ("controltimev2", dt, 1),
            ("type", "a6"),
            ("ang_type", "a3"),
        ],
    )
    r["enc"] = rstate[:, 0:40]
    # JA0-JA39
    r["jvelocity"] = rstate[:, 40:80]
    #
    r["acc"] = rstate[:, 80:83]
    #
    r["zmp"] = rstate[:, 83:86]
    #
    r["waist_p"] = rstate[:, 86:89]
    # waistPX-waistPZ
    r["waist_rot"] = rstate[:, 89:98]
    # waistQX-waistQW (rot matrix)
    r["time"] = rstate[:, 98]
    # clockv2
    r["controltimev2"] = rstate[:, 99]
    # ControlTimeV2

    r["type"] = "rstate"
    r["ang_type"] = "rad"

    if angle_unit == "deg":
        r["enc"] = 180 / np.pi * r["enc"]
        # Joint angles in degrees
        r["ang_type"] = "deg"
        print("Warning: joint angles (astate) in degrees!")

    return r


if __name__ == "__main__":
    r = load_hrpsys_log_rstate(
        "/home/adelpret/devel/yarp_gazebo/src/motorFrictionIdentification/data/20140807-legTorqueId/legTorqueId_pos1-rstate.log"
    )
    a = load_hrpsys_log_astate(
        "/home/adelpret/devel/yarp_gazebo/src/motorFrictionIdentification/data/20140807-legTorqueId/legTorqueId_pos1-astate.log"
    )
