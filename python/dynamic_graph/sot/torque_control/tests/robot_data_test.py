import numpy as np

import example_robot_data


class initRobotData:
    nbJoints = 29
    controlDT = 0.005
    maxCurrent = 5
    robotRef = "control-manager-robot"
    urdftosot = (12, 13, 14, 15, 23, 24, 25, 26, 27, 28, 16, 17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4,
                 5)

    ctrlManagerCurrentToControlGain = 1.0

    mapJointNameToID = {
        'rhy': 0,
        'rhr': 1,
        'rhp': 2,
        'rk': 3,
        'rap': 4,
        'rar': 5,
        'lhy': 6,
        'lhr': 7,
        'lhp': 8,
        'lk': 9,
        'lap': 10,
        'lar': 11,
        'ty': 12,
        'tp': 13,
        'hp': 14,
        'rsp': 15,
        'rsr': 16,
        'rsy': 17,
        're': 18,
        'rwy': 19,
        'rwp': 20,
        'rh': 21,
        'lsp': 22,
        'lsr': 23,
        'lsy': 24,
        'le': 25,
        'lwy': 26,
        'lwp': 27,
        'lh': 28
    }

    mapJointLimits = {
        0: [-0.785398, 0.523599],
        1: [-0.610865, 0.349066],
        2: [-2.18166, 0.733038],
        3: [-0.0349066, 2.61799],
        4: [-1.309, 0.733038],
        5: [-0.349066, 0.610865],
        6: [-0.523599, 0.785398],
        7: [-0.349066, 0.610865],
        8: [-2.18166, 0.733038],
        9: [-0.0349066, 2.61799],
        10: [-1.309, 0.733038],
        11: [-0.610865, 0.349066],
        12: [-0.785398, 0.785398],
        13: [-0.0872665, 1.0472],
        14: [-0.785398, 0.78539],
        15: [-0.523599, 0.785398],
        16: [-3.14159, 1.0472],
        17: [-1.65806, 0.174533],
        18: [-1.6057, 1.6057],
        19: [-2.3911, 0.0349066],
        20: [-1.6057, 1.6057],
        21: [-1.6057, 1.6057],
        22: [-1.0, 1.0],
        23: [-3.14159, 1.0472],
        24: [-0.174533, 1.65806],
        25: [-1.6057, 1.6057],
        26: [-2.3911, 0.0349066],
        27: [-1.6057, 1.6057],
        28: [-1.6057, 1.6057],
    }

    fMax = np.array([100.0, 100.0, 300.0, 80.0, 80.0, 30.0])
    fMin = -fMax
    mapForceIdToForceLimits = {0: [fMin, fMax], 1: [fMin, fMax], 2: [fMin, fMax], 3: [fMin, fMax]}

    mapNameToForceId = {"rf": 0, "lf": 1, "rh": 2, "lh": 3}

    indexOfForceSensors = ()
    FootFrameNames = {"Right": "RLEG_ANKLE_R", "Left": "LLEG_ANKLE_R"}

    RightFootSensorXYZ = (0.0, 0.0, -0.085)

    def __init__(self):
        _, _, urdf, _ = example_robot_data.load_full('simple_humanoid')
        self.testRobotPath = urdf

    def init_and_set_controller_manager(self, cm):
        # Init should be called before addCtrlMode
        # because the size of state vector must be known.
        cm.init(self.controlDT, self.testRobotPath, self.robotRef)

        # Set the map from joint name to joint ID
        for key in self.mapJointNameToID:
            cm.setNameToId(key, self.mapJointNameToID[key])

        # Set the map joint limits for each id
        for key in self.mapJointLimits:
            cm.setJointLimitsFromId(key, self.mapJointLimits[key][0], self.mapJointLimits[key][1])

        # Set the force limits for each id
        for key in self.mapForceIdToForceLimits:
            cm.setForceLimitsFromId(key, np.array(self.mapForceIdToForceLimits[key][0]),
                                    np.array(self.mapForceIdToForceLimits[key][1]))

        # Set the force sensor id for each sensor name
        for key in self.mapNameToForceId:
            cm.setForceNameToForceId(key, self.mapNameToForceId[key])

        # Set the map from the urdf joint list to the sot joint list
        cm.setJointsUrdfToSot(np.array(self.urdftosot))

        # Set the foot frame name
        for key in self.FootFrameNames:
            cm.setFootFrameName(key, self.FootFrameNames[key])

        cm.setRightFootSoleXYZ(np.array(self.RightFootSensorXYZ))
