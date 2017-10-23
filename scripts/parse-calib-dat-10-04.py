#Thomas F.
#This script reads and translates encoders calibration file from 10.04 to 14.04 format
import numpy as np
fname = 'calib.dat'
lines = open(fname).readlines()
range_to_limits = [-0.785398163, -0.610865238, 0.733038285,-0.034906585, 0.733038285, -0.34906585, # RLEG
                    0.785398163,  0.610865238, 0.733038285,-0.034906585, 0.733038285, 0.34906585,  # LLEG
                    0.785398163,-0.087266462, # CHEST
                    0.785398163,-0.523598775, # HEAD
                    1.047197551, -1.658062789, -1.605702912, 0.034906585,-1.605702912, -1.605702912, 0.785398163,  # RARM
                    1.047197551,  1.658062789,  1.605702912, 0.034906585, 1.605702912, -1.605702912, 0.785398163]; # LARM
offsets_correction = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
#read calib.dat
for l in lines[:30]:
    cols = l.split()
    jointId = int(cols[0])
    offset  = float(cols[1]) * np.pi / 180.0
    offsets_correction[jointId] = offset - range_to_limits[jointId]
print 'This table should be copy-paste in "hrp2-n14-system/src/direct-access/hrp2-io-boards-init.cpp"'
print 'double range_to_limits_correction[30] =  {%s, // RLEG' % ', '.join(map(str, offsets_correction[00:06]))
print '                                          %s, // LLEG' % ', '.join(map(str, offsets_correction[06:12]))
print '                                          %s, // CHEST'% ', '.join(map(str, offsets_correction[12:14]))
print '                                          %s, // HEAD' % ', '.join(map(str, offsets_correction[14:16]))
print '                                          %s-0.1, // RARM' % ', '.join(map(str, offsets_correction[16:23])) # Mind the hack (-0.1) to prevent the hand to close too much in q0
print '                                          %s-0.1};// LARM' % ', '.join(map(str, offsets_correction[23:30])) # Mind the hack (-0.1) to prevent the hand to close too much in q0
