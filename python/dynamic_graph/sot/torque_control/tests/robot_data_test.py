
import numpy

class initRobotData:
  nbJoints=30
  testRobotPath="/opt/openrobots/share/hrp2_14_description/urdf/hrp2_14_reduced.urdf"
  controlDT=0.005
  maxCurrent=5

  urdftosot=(12,13,14,15,23,24,25,26,27,28,29,16,17,18,19,20,21,22,6,7,8,9,10,11,0,1,2,3,4,5)

  mapJointNameToID={
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
    'hy': 14, 
    'hp': 15, 
    'rsp': 16, 
    'rsr': 17, 
    'rsy': 18, 
    're': 19, 
    'rwy': 20, 
    'rwp': 21,
    'rh': 22,
    'lsp': 23,
    'lsr': 24,
    'lsy': 25,
    'le': 26,
    'lwy': 27,
    'lwp': 28,
    'lh': 29
  }
    
  mapJointLimits={
    0 : [-0.785398, 0.523599],  
    1 : [-0.610865, 0.349066],  
    2 : [-2.18166, 0.733038],   
    3 : [-0.0349066, 2.61799], 
    4 : [-1.309, 0.733038],     
    5 : [-0.349066, 0.610865],  
    6 : [-0.523599, 0.785398],  
    7 : [-0.349066, 0.610865],  
    8 : [-2.18166, 0.733038],   
    9 : [-0.0349066, 2.61799], 
    10 : [-1.309, 0.733038],    
    11 : [-0.610865, 0.349066], 
    12 : [-0.785398, 0.785398],
    13 : [-0.0872665, 1.0472], 
    14 : [-0.785398, 0.78539], 
    15 : [-0.523599, 0.785398],
    16 : [-3.14159, 1.0472],    
    17 : [-1.65806, 0.174533],  
    18 : [-1.6057, 1.6057],     
    19 : [-2.3911, 0.0349066],  
    20 : [-1.6057, 1.6057],     
    21 : [-1.6057, 1.6057],    
    22 : [-1.0, 1.0],      
    23 : [-3.14159, 1.0472],    
    24 : [-0.174533, 1.65806],  
    25 : [-1.6057, 1.6057],     
    26 : [-2.3911, 0.0349066],  
    27 : [-1.6057, 1.6057],     
    28 : [-1.6057, 1.6057],    
    29 : [-1.0, 1.0]      
  }
    
  fMax=numpy.array([100.0,100.0,300.0,80.0,80.0,30.0])
  fMin=-fMax
  mapForceIdToForceLimits={
    0: [fMin,fMax],
    1: [fMin,fMax],
    2: [fMin,fMax],
    3: [fMin,fMax]
  }
    
  mapNameToForceId={
    "rf": 0,
    "lf": 1,
    "rh": 2,
    "lh": 3
  }
    
  indexOfForceSensors= () 
  FootFrameNames= { 
    "Right": "RLEG_JOINT5",
    "Left" : "LLEG_JOINT5"
  }
      
  RightFootSensorXYZ = (0.0,0.0,-0.085)

  def init_and_set_controller_manager(self, cm):
    # Init should be called before addCtrlMode 
    # because the size of state vector must be known.
    cm.init(self.controlDT,self.testRobotPath,self.maxCurrent,"control-manager-robot")

    # Set the map from joint name to joint ID
    for key in self.mapJointNameToID:
      cm.setNameToId(key,self.mapJointNameToID[key])
            
    # Set the map joint limits for each id
    for key in self.mapJointLimits:
      cm.setJointLimitsFromId(key,self.mapJointLimits[key][0], \
                              self.mapJointLimits[key][1])
          
    # Set the force limits for each id
    for key in self.mapForceIdToForceLimits:
      cm.setForceLimitsFromId(key,tuple(self.mapForceIdToForceLimits[key][0]), \
                              tuple(self.mapForceIdToForceLimits[key][1]))

    # Set the force sensor id for each sensor name
    for key in self.mapNameToForceId:
      cm.setForceNameToForceId(key,self.mapNameToForceId[key])

    # Set the map from the urdf joint list to the sot joint list
    cm.setJointsUrdfToSot(self.urdftosot)

    # Set the foot frame name
    for key in self.FootFrameNames:
      cm.setFootFrameName(key,self.FootFrameNames[key])

    cm.setRightFootSoleXYZ(self.RightFootSensorXYZ)

    cm.setDefaultMaxCurrent(-10.0)
    cm.setDefaultMaxCurrent(self.maxCurrent)
    cm.getDefaultMaxCurrent()

