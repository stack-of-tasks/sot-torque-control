from parametriccurves import spline, spline6
from locomote import ContactSequenceHumanoid
from numpy import polyfit
from numpy.linalg import lstsq, solve
from pinocchio.utils import *
import numpy as np


####################FOOT TRAJECTORY
Z_AMP=0.06
TIME_TO_LIFT=(2,2.7)
TOTAL_TRAJ_TIME=4.7
LF_INIT=np.array([0.0244904,0.105,0.])


####CONFIG######################
CONTACT_SEQUENCE_WHOLEBODY_FILE="../data/traj_com_y_0.08/contact_sequence_trajectory.xml"
CONTACT_SEQUENCE_XML_TAG="ContactSequence"

COM_SPLINE_OUTPUT_FILE="../data/traj_com_y_0.08/com_spline.curve"
RF_FORCE_OUTPUT_FILE ="../data/traj_com_y_0.08/rf_force.curve"
LF_FORCE_OUTPUT_FILE ="../data/traj_com_y_0.08/lf_force.curve"
RH_FORCE_OUTPUT_FILE ="../data/traj_com_y_0.08/rh_force.curve"
LH_FORCE_OUTPUT_FILE ="../data/traj_com_y_0.08/lh_force.curve"
LF_SPLINE_OUTPUT_FILE="../data/traj_com_y_0.08/lf_spline"+ str(int(Z_AMP*100))+".curve"
VERIFY=False
WRITE_OUTPUT =True
PLOT =True
##########################################


def array_polyfit(x, y, deg, rcond=None, full=False, w=None, cov=False, eps=1e-18):
  assert len(x.shape)==1;
  p = np.zeros((y.shape[0], deg+1))
  for i,y_a in enumerate(y):
    assert y_a.size ==y_a.shape[1]
    p_a , residual, _, _, _ = np.polyfit(x, np.asarray(y_a).squeeze(), deg, rcond, full, w, cov);
    p[i, :] = p_a[::-1] 
    assert(residual <=eps)
  return p

def dd_polyfit(x, y, dy, ddy, deg_y, eps=1e-20):
  assert(y.shape[1]==dy.shape[1]==ddy.shape[1])
  p = np.zeros((y.shape[0], deg_y+1))
  for i in xrange(y.shape[0]):
    p_a = dd_polyfit_1d(x, y[i], dy[i], ddy[i], deg_y, eps)
    p[i,:] = p_a.squeeze()
  return p


def dd_polyfit_1d(x, y, dy, ddy, deg_y, eps=1e-20):
  assert len(x.shape)==1;
  assert deg_y>=2;
  A = np.zeros((x.shape[0]*3, deg_y+1))

  for i,t in enumerate(x):
    A[i,:] = np.array([t**n for n in xrange(deg_y+1)])

  for i,t in enumerate(x):
    A[i+len(x),1:] = np.array([n*t**(n-1) for n in xrange(1, deg_y+1)]);
    A[i+len(x),0] = 0.0;

  for i,t in enumerate(x):
    A[i+2*len(x),2:] = np.array([n*(n-1)*t**(n-2) for n in xrange(2,deg_y+1)]);
    A[i+2*len(x),0:2] = 0.0;

  B = np.concatenate((y, dy, ddy), axis=1).transpose()
  x, residual, _, _ = lstsq(A, B)
  assert(residual <=eps)
  return x

cs = ContactSequenceHumanoid(0)
cs.loadFromXML(CONTACT_SEQUENCE_WHOLEBODY_FILE,CONTACT_SEQUENCE_XML_TAG)

traj_times = []
poly_com_list = []; poly_dcom_list = []; poly_ddcom_list = []
poly_L_list = []; poly_dL_list = []
poly_control_list_rf = [];
poly_control_list_lf = [];
poly_control_list_rh = [];
poly_control_list_lh = [];
time_vector = np.zeros(len(cs.ms_interval_data))
control_list = np.array([])
for spl in cs.ms_interval_data:
  x = np.array([]);
  y_com = np.array([]);  y_dcom = np.array([]);  y_ddcom = np.array([]);
  y_L = np.array([]);   y_dL = np.array([]);
  y_control_lf = np.array([]); 
  y_control_rf = np.array([]);
  y_control_rh = np.array([]);
  y_control_lh = np.array([]);
  for t in spl.time_trajectory:
    x = np.append(x,t)
  if x.size==0: break;
  for y in spl.state_trajectory:
    if y_com.size==0:
      y_com = y[0:3]
      y_dcom = y[3:6]
      y_L = y[6:9]
    else:
      y_com = np.append(y_com, y[0:3], axis=1)
      y_dcom = np.append(y_dcom, y[3:6], axis=1)
      y_L = np.append(y_L, y[6:9], axis=1)
  for y in spl.dot_state_trajectory:
    if y_ddcom.size==0:
      y_ddcom = y[3:6]
      y_dL = y[6:9]
    else:
      y_ddcom = np.append(y_ddcom, y[3:6], axis=1)
      y_dL = np.append(y_dL, y[6:9], axis=1)
  for y in spl.control_trajectory:

    if control_list.size ==0:
      control_list = y
    else:
      control_list = np.append(control_list, y, axis=1)

    if y_control_rf.size==0:
      y_control_rf = y[0:6]
      y_control_lf = y[6:12]
      y_control_rh = y[12:18]
      y_control_lh = y[18:24]
    else:
      y_control_rf = np.append(y_control_rf, y[0:6], axis=1)
      y_control_lf = np.append(y_control_lf, y[6:12], axis=1)
      y_control_rh = np.append(y_control_rh, y[12:18], axis=1)
      y_control_lh = np.append(y_control_lh, y[18:24], axis=1)

  traj_times.append(x)
  poly_com_list.append(dd_polyfit(x, y_com, y_dcom, y_ddcom, deg_y=5, eps=1e-20))
  #poly_L_list.append(array_polyfit(x, y_L, deg=4, full=True))
  #poly_dL_list.append(array_polyfit(x, y_dL, deg=3, full=True))
  #Fitting for control vector is the same?
  poly_control_list_rf.append(array_polyfit(x, y_control_rf, deg=3, full=True, eps=1e-18))
  poly_control_list_lf.append(array_polyfit(x, y_control_lf, deg=3, full=True, eps=1e-18))
  poly_control_list_rh.append(array_polyfit(x, y_control_rh, deg=3, full=True, eps=1e-18))
  poly_control_list_lh.append(array_polyfit(x, y_control_lh, deg=3, full=True, eps=1e-18))

time_vector = np.zeros(len(traj_times)+1)
for ts in xrange(len(traj_times)):
  time_vector[ts] = traj_times[ts][0]
time_vector[-1] = traj_times[-1][-1]

com_spline = spline(poly_com_list, time_vector)
rf_force = spline6(poly_control_list_rf, time_vector)
lf_force = spline6(poly_control_list_lf, time_vector)
rh_force = spline6(poly_control_list_rh, time_vector)
lh_force = spline6(poly_control_list_lh, time_vector)

def foot_lift0(initPos, z_amp=Z_AMP, timeToLift=TIME_TO_LIFT,
               totalTrajTime=TOTAL_TRAJ_TIME):

  s = timeToLift[1]-timeToLift[0]
  s0=timeToLift[0]
  s1 = timeToLift[1]

  A = np.zeros((7, 7))
  A[0,:] = np.array([s0**n for n in xrange(7)])
  A[1,:] = np.array([(0.5*(s0+s1))**n for n in xrange(7)])
  A[2,:] = np.array([s1**n for n in xrange(7)])
  A[3,1:] = np.array([n*(s0**(n-1)) for n in xrange(1,7)])
  A[4,1:] = np.array([n*(s1**(n-1)) for n in xrange(1,7)])
  A[5,2:] = np.array([n*(n-1)*(s0**(n-2)) for n in xrange(2,7)])
  A[6,2:] = np.array([n*(n-1)*(s1**(n-2)) for n in xrange(2,7)])

  B = np.matrix([[0.0,z_amp,0.0,0.,0.,0.,0.]]).transpose()

  poly_coeff_begin=np.zeros((3, 7))
  poly_coeff_s =np.zeros((3, 7))
  poly_coeff_end= np.zeros((3, 7))

  for arr in [poly_coeff_begin, poly_coeff_s, poly_coeff_end]:
    arr[0][0] = initPos[0]
    arr[1][0] = initPos[1]

  time_vector = np.array([0., s0, s1, totalTrajTime])
  poly_coeff_s[2,:] = np.array(solve(A,B)).squeeze()

  return ([poly_coeff_begin, poly_coeff_s, poly_coeff_end], time_vector)

lf_traj = foot_lift0(initPos=LF_INIT, z_amp=Z_AMP, timeToLift=TIME_TO_LIFT,
                    totalTrajTime=TOTAL_TRAJ_TIME)

lf_spline = spline(lf_traj[0], lf_traj[1])

print "Trajectories created"
if WRITE_OUTPUT:
  com_spline.save_to_file(COM_SPLINE_OUTPUT_FILE)
  rf_force.save_to_file(RF_FORCE_OUTPUT_FILE)
  lf_force.save_to_file(LF_FORCE_OUTPUT_FILE)
  rh_force.save_to_file(RH_FORCE_OUTPUT_FILE)
  lh_force.save_to_file(LH_FORCE_OUTPUT_FILE)
  lf_spline.save_to_file(LF_SPLINE_OUTPUT_FILE)
###############CONFIRM SPLINE OUTPUT##########################

if VERIFY:
  for spl in cs.ms_interval_data:
    for j,t in enumerate(spl.time_trajectory):
      assert(np.isclose(spl.state_trajectory[j][0:3],com_spline(t)).all())
      if not np.isclose(spl.control_trajectory[j][0:6],rf_force(t)).all():
        print t, spl.control_trajectory[j][0:6], rf_force(t)
        raw_input("Error in xml vs spline rf force")
      if not np.isclose(spl.control_trajectory[j][6:12],lf_force(t)).all():
        print t, spl.control_trajectory[j][6:12], lf_force(t)
        raw_input("Error in xml vs spline lf force")
      if not np.isclose(spl.state_trajectory[j][3:6],com_spline.derivate(t,1)).all():
        raw_input("Oops")
      if not np.isclose(spl.dot_state_trajectory[j][3:6],com_spline.derivate(t,2)).all():
        raw_input("Oops again")

if PLOT:
  import matplotlib.pyplot as plt
  N = 1000
  lf_spline_data = np.array([lf_spline((t/(1.0*N))*lf_spline.max())[2] \
                             for t in xrange(N)]).squeeze()
  time_data = np.array([(t/(1.0*N))*lf_spline.max() for t in xrange(N)]).squeeze()
  plt.plot(time_data, lf_spline_data)
  plt.show()
