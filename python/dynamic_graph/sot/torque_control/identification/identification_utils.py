import time
from subprocess import call

import numpy as np

jID = {
    "rhy": 0,
    "rhr": 1,
    "rhp": 2,
    "rk": 3,
    "rap": 4,
    "rar": 5,
    "lhy": 6,
    "lhr": 7,
    "lhp": 8,
    "lk": 9,
    "lap": 10,
    "lar": 11,
    "ty": 12,
    "tp": 13,
    "hy": 14,
    "hp": 15,
    "rsp": 16,
    "rsr": 17,
    "rsy": 18,
    "re": 19,
    "rwy": 20,
    "rwp": 21,
    "rh": 22,
    "lsp": 23,
    "lsr": 24,
    "lsy": 25,
    "le": 26,
    "lwy": 27,
    "lwp": 28,
    "lh": 29,
}


def solve1stOrderLeastSquare(x, y):
    """Solve the least square problem:
    solve   y=ax+b in L2 norm
    """
    Q = np.vstack([np.ones(len(x)), x])
    coef = solveLeastSquare(Q.T, y)
    (a, b) = coef[1, 0], coef[0, 0]
    return (a, b)


def solveLeastSquare(A, b):
    """Solve the least square problem:
    minimize   || A*x-b ||^2
    """
    return np.linalg.pinv(A) * np.matrix(b).T


def gentleStop(traj_gen, joint):
    """Stop the joint when vel is low"""
    while abs(traj_gen.dq.value[jID[joint]]) > 0.0001:
        time.sleep(0.001)
    traj_gen.stop(joint)


def doNCycles(traj_gen, joint, min_pos, max_pos, N, times, mode="constAcc"):
    """Do N cycles in cost vel or acc with speeds given by times (ex times=[5.0,4.0,3.0])"""
    traj_gen.moveJoint(joint, min_pos, 3.0)
    time.sleep(3.5)
    for T in times:
        if mode == "constAcc":
            traj_gen.startConstAcc(joint, max_pos, T)
        elif mode == "constVel":
            traj_gen.startTriangle(joint, max_pos, T, 0.3)
        time.sleep(T * 2 * N - 1.0)
        gentleStop(traj_gen, joint)
        traj_gen.moveJoint(joint, min_pos, 3.0)
        time.sleep(3.5)


# (-0.785398, 0.523599);  #// right hip yaw *****************************
def identify_rhy_static(traj_gen, staticTime=60.0):
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)
    traj_gen.moveJoint("rhp", -1.57, 5.0)
    time.sleep(5.0)
    time.sleep(staticTime)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)


def identify_rhy_dynamic(traj_gen, mode="constAcc", N=3, times=[5.0, 4.0, 3.0]):
    (joint, min_pos, max_pos) = ("rhy", -0.0, 0.5)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)
    traj_gen.moveJoint("lhr", 0.1, 3.0)
    time.sleep(3.0 + 0.5)
    doNCycles(traj_gen, joint, min_pos, max_pos, N, times, mode)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)


# (-0.610865, 0.349066);  #// right hip roll ****************************
def identify_rhr_static(traj_gen, staticTime=60.0):
    # (joint, min_pos, max_pos) = ('rhr', -0.5, 0.25)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)
    traj_gen.moveJoint("lhr", 0.25, 5.0)
    time.sleep(5.0)
    time.sleep(staticTime)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)


def identify_rhr_dynamic(traj_gen, mode="constAcc", N=3, times=[5.0, 4.0, 3.0, 2.5]):
    (joint, min_pos, max_pos) = ("rhr", -0.5, 0.25)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)
    traj_gen.moveJoint("rsp", -1.57, 5.0)
    traj_gen.moveJoint("lsp", -1.57, 5.0)
    traj_gen.moveJoint("re", -1.57, 5.0)
    traj_gen.moveJoint("le", -1.57, 5.0)
    traj_gen.moveJoint("lhr", 0.25, 5.0)
    time.sleep(5.0 + 0.5)
    doNCycles(traj_gen, joint, min_pos, max_pos, N, times, mode)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)


# (-2.18166, 0.733038);   #// right hip pitch ***************************:
def identify_rhp_static(traj_gen, staticTime=60.0):
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)
    time.sleep(5.0)
    time.sleep(staticTime)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)


def identify_rhp_dynamic(traj_gen, mode="constAcc", N=3, times=[5.0, 4.0, 3.0]):
    (joint, min_pos, max_pos) = ("rhp", -1.7, 0.6)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)
    traj_gen.moveJoint("rhr", -0.2, 5.0)
    time.sleep(5.0 + 0.5)
    doNCycles(traj_gen, joint, min_pos, max_pos, N, times, mode)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)


# (-0.0349066, 2.61799);  #// right knee ********************************
def identify_rk_static(traj_gen, staticTime=60.0):
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)
    traj_gen.moveJoint("rhp", -1.57, 5.0)
    traj_gen.moveJoint("rk", 1.57, 5.0)
    time.sleep(5.0 + 0.5)
    time.sleep(staticTime)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)


def identify_rk_dynamic(traj_gen, mode="constAcc", N=3, times=[5.0, 4.0, 3.0]):
    (joint, min_pos, max_pos) = ("rk", 0.0, 2.5)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)
    traj_gen.moveJoint("rhp", -1.57, 5.0)
    time.sleep(5.0 + 0.5)
    doNCycles(traj_gen, joint, min_pos, max_pos, N, times, mode)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)


# (-1.309, 0.733038);     #// right ankle pitch *************************
def identify_rap_static(traj_gen, staticTime=60.0):
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)
    traj_gen.moveJoint("rhp", -1.57, 5.0)
    traj_gen.moveJoint("rk", 1.57, 5.0)
    time.sleep(5.0 + 0.5)
    time.sleep(staticTime)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)


def identify_rap_dynamic(traj_gen, mode="constAcc", N=3, times=[5.0, 4.0, 3.0]):
    (joint, min_pos, max_pos) = ("rap", -1.2, 0.6)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)
    traj_gen.moveJoint("rhp", -1.57, 5.0)
    traj_gen.moveJoint("rk", 1.57, 5.0)
    time.sleep(5.0 + 0.5)
    doNCycles(traj_gen, joint, min_pos, max_pos, N, times, mode)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)


# (-0.349066, 0.610865);  #// right ankle roll **************************
def identify_rar_static(traj_gen, staticTime=60.0):
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)
    traj_gen.moveJoint("rhp", -1.57, 5.0)
    traj_gen.moveJoint("rk", 1.57, 5.0)
    time.sleep(5.0 + 0.5)
    time.sleep(staticTime)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)


def identify_rar_dynamic(traj_gen, mode="constAcc", N=3, times=[5.0, 4.0, 3.0]):
    (joint, min_pos, max_pos) = ("rar", -0.25, 0.5)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)
    traj_gen.moveJoint("rhp", -1.57, 5.0)
    traj_gen.moveJoint("rk", 1.57, 5.0)
    time.sleep(5.0 + 0.5)
    doNCycles(traj_gen, joint, min_pos, max_pos, N, times, mode)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)


# (-0.785398, 0.523599);  #// left hip yaw *********************INVERTED
def identify_lhy_static(traj_gen, staticTime=60.0):
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)
    traj_gen.moveJoint("lhp", -1.57, 5.0)
    time.sleep(5.0)
    time.sleep(staticTime)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)


def identify_lhy_dynamic(traj_gen, mode="constAcc", N=3, times=[5.0, 4.0, 3.0]):
    (joint, min_pos, max_pos) = ("lhy", +0.0, -0.5)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)
    traj_gen.moveJoint("rhr", -0.1, 3.0)
    time.sleep(3.0 + 0.5)
    doNCycles(traj_gen, joint, min_pos, max_pos, N, times, mode)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)


# (-0.610865, 0.349066);  #// left hip roll ********************INVERTED
def identify_lhr_static(traj_gen, staticTime=60.0):
    # (joint, min_pos, max_pos) = ('lhr', +0.5, -0.25)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)
    traj_gen.moveJoint("rhr", -0.25, 5.0)
    time.sleep(5.0)
    time.sleep(staticTime)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)


def identify_lhr_dynamic(traj_gen, mode="constAcc", N=3, times=[5.0, 4.0, 3.0, 2.5]):
    (joint, min_pos, max_pos) = ("lhr", +0.5, -0.25)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)
    traj_gen.moveJoint("lsp", -1.57, 5.0)
    traj_gen.moveJoint("rsp", -1.57, 5.0)
    traj_gen.moveJoint("le", -1.57, 5.0)
    traj_gen.moveJoint("re", -1.57, 5.0)
    traj_gen.moveJoint("rhr", -0.25, 5.0)
    time.sleep(5.0 + 0.5)
    doNCycles(traj_gen, joint, min_pos, max_pos, N, times, mode)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)


# (-2.18166, 0.733038);   #// left hip pitch ***************************:
def identify_lhp_static(traj_gen, staticTime=60.0):
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)
    time.sleep(5.0)
    time.sleep(staticTime)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)


def identify_lhp_dynamic(traj_gen, mode="constAcc", N=3, times=[5.0, 4.0, 3.0]):
    (joint, min_pos, max_pos) = ("lhp", -1.7, 0.6)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)
    traj_gen.moveJoint("lsp", -1.57, 5.0)
    traj_gen.moveJoint("rsp", -1.57, 5.0)
    traj_gen.moveJoint("le", -1.57, 5.0)
    traj_gen.moveJoint("re", -1.57, 5.0)
    traj_gen.moveJoint("lhr", +0.2, 5.0)
    time.sleep(5.0 + 0.5)
    doNCycles(traj_gen, joint, min_pos, max_pos, N, times, mode)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)


# (-0.0349066, 2.61799);  #// left knee ********************************
def identify_lk_static(traj_gen, staticTime=60.0):
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)
    traj_gen.moveJoint("lhp", -1.57, 5.0)
    traj_gen.moveJoint("lk", 1.57, 5.0)
    time.sleep(5.0 + 0.5)
    time.sleep(staticTime)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)


def identify_lk_dynamic(traj_gen, mode="constAcc", N=3, times=[5.0, 4.0, 3.0]):
    (joint, min_pos, max_pos) = ("lk", 0.0, 2.5)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)
    traj_gen.moveJoint("lhp", -1.57, 5.0)
    time.sleep(5.0 + 0.5)
    doNCycles(traj_gen, joint, min_pos, max_pos, N, times, mode)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)


# (-1.309, 0.733038);     #// left ankle pitch *************************
def identify_lap_static(traj_gen, staticTime=60.0):
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)
    traj_gen.moveJoint("lhp", -1.57, 5.0)
    traj_gen.moveJoint("lk", 1.57, 5.0)
    time.sleep(5.0 + 0.5)
    time.sleep(staticTime)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)


def identify_lap_dynamic(traj_gen, mode="constAcc", N=3, times=[5.0, 4.0, 3.0]):
    (joint, min_pos, max_pos) = ("lap", -1.2, 0.6)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)
    traj_gen.moveJoint("lhp", -1.57, 5.0)
    traj_gen.moveJoint("lk", 1.57, 5.0)
    time.sleep(5.0 + 0.5)
    doNCycles(traj_gen, joint, min_pos, max_pos, N, times, mode)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)


# (-0.349066, 0.610865);  #// left ankle roll ******************INVERTED
def identify_lar_static(traj_gen, staticTime=60.0):
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)
    traj_gen.moveJoint("lhp", -1.57, 5.0)
    traj_gen.moveJoint("lk", 1.57, 5.0)
    time.sleep(5.0 + 0.5)
    time.sleep(staticTime)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)


def identify_lar_dynamic(traj_gen, mode="constAcc", N=3, times=[5.0, 4.0, 3.0]):
    (joint, min_pos, max_pos) = ("lar", +0.25, -0.5)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)
    traj_gen.moveJoint("lhp", -1.57, 5.0)
    traj_gen.moveJoint("lk", 1.57, 5.0)
    time.sleep(5.0 + 0.5)
    doNCycles(traj_gen, joint, min_pos, max_pos, N, times, mode)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)


def identify_tp_dynamic(traj_gen, mode="constAcc", N=3, times=[5.0, 4.0, 3.0, 2.5]):
    (joint, min_pos, max_pos) = ("tp", 0.0, 1.0)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)
    doNCycles(traj_gen, joint, min_pos, max_pos, N, times, mode)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)


def identify_ty_dynamic(traj_gen, mode="constAcc", N=3, times=[5.0, 4.0, 3.0, 2.5]):
    (joint, min_pos, max_pos) = ("ty", -0.7, 0.7)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)
    traj_gen.moveJoint("lsp", -1.57, 5.0)
    traj_gen.moveJoint("rsp", -1.57, 5.0)
    time.sleep(5.0 + 0.5)
    doNCycles(traj_gen, joint, min_pos, max_pos, N, times, mode)
    go_to_zero_position(traj_gen, 5.0)
    time.sleep(5.0 + 0.5)


def go_to_zero_position(traj_gen, T=10.0):
    # Python interpreter can't deal with input(..) ??
    #    ret = input('Are you sure you want to put the robot in zero position? All joints will move: [y/N]')
    #    if ret!="y" :
    #        print('Cancel zero position')
    #        return

    # put the robot in position q0
    # RLEG TO 0 **********************
    traj_gen.moveJoint("rhy", 0.0, T)  # 0
    traj_gen.moveJoint("rhr", 0.0, T)  # 1
    traj_gen.moveJoint("rhp", 0.0, T)  # 2
    traj_gen.moveJoint("rk", 0.0, T)  # 3
    traj_gen.moveJoint("rap", 0.0, T)  # 4
    traj_gen.moveJoint("rar", 0.0, T)  # 5

    # LLEG TO 0 **********************
    traj_gen.moveJoint("lhy", 0.0, T)  # 6
    traj_gen.moveJoint("lhr", 0.0, T)  # 7
    traj_gen.moveJoint("lhp", 0.0, T)  # 8
    traj_gen.moveJoint("lk", 0.0, T)  # 9
    traj_gen.moveJoint("lap", 0.0, T)  # 10
    traj_gen.moveJoint("lar", 0.0, T)  # 11

    # TORSO TO 0
    traj_gen.moveJoint("ty", 0.0, T)  # 12
    traj_gen.moveJoint("tp", 0.0, T)  # 13

    # HEAD TO 0
    traj_gen.moveJoint("hy", 0.0, T)  # 14
    traj_gen.moveJoint("hp", 0.0, T)  # 15

    # RARM TO 0 **********************
    traj_gen.moveJoint("rsp", 0.0, T)  # 16
    traj_gen.moveJoint("rsr", 0.0, T)  # 17
    traj_gen.moveJoint("rsy", 0.0, T)  # 18
    traj_gen.moveJoint("re", 0.0, T)  # 19
    traj_gen.moveJoint("rwy", 0.0, T)  # 20
    traj_gen.moveJoint("rwp", 0.0, T)  # 21
    traj_gen.moveJoint("rh", 0.3, T)  # 22

    # LARM TO 0 **********************
    traj_gen.moveJoint("lsp", 0.0, T)  # 23
    traj_gen.moveJoint("lsr", 0.0, T)  # 24
    traj_gen.moveJoint("lsy", 0.0, T)  # 25
    traj_gen.moveJoint("le", 0.0, T)  # 26
    traj_gen.moveJoint("lwy", 0.0, T)  # 27
    traj_gen.moveJoint("lwp", 0.0, T)  # 28
    traj_gen.moveJoint("lh", 0.3, T)  # 29


def deleteDatFilesInTmp():
    call("rm /tmp/*.dat", shell=True)


def stopTracerAndCopyFiles(tracer, directory):
    tracer.stop()
    tracer.dump()
    time.sleep(2.0)
    call("mkdir " + directory, shell=True)
    call("mv /tmp/*.dat " + directory, shell=True)


# deleteDatFilesInTmp()
# tracer = start_tracer(robot, estimator, torque_ctrl, traj_gen, ctrl_manager, inv_dyn, None)
# do your experiment here
# stopTracerAndCopyFiles(tracer,directory='/tmp/JOINT0_ID_static')
