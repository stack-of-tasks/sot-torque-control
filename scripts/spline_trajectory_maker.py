####CONFIG######################
CONTACT_SEQUENCE_WHOLEBODY_FILE="../data/seq1/contact_sequence_trajectory.xml"
CONTACT_SEQUENCE_XML_TAG="ContactSequence"

COM_SPLINE_OUTPUT_FILE="../data/seq1/com_spline.curve"
RF_FORCE_OUTPUT_FILE ="../data/seq1/rf_force.curve"
LF_FORCE_OUTPUT_FILE ="../data/seq1/lf_force.curve"
RH_FORCE_OUTPUT_FILE ="../data/seq1/rh_force.curve"
LH_FORCE_OUTPUT_FILE ="../data/seq1/lh_force.curve"
VERIFY=True
WRITE_OUTPUT =False
####CONFIG######################



from parametriccurves import spline, forcecurve
from locomote import ContactSequenceHumanoid
from numpy import polyfit
from numpy.linalg import lstsq
from pinocchio.utils import *
import numpy as np

def array_polyfit(x, y, deg, rcond=None, full=False, w=None, cov=False, eps=1e-18):
  assert len(x.shape)==1;
  p = np.zeros((y.shape[0], deg+1))
  for i,y_a in enumerate(y):
    p_a , residual, _, _, _ = np.polyfit(x, np.asarray(y_a).squeeze(), deg, rcond, full, w, cov);
    p[i, :] = p_a
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
poly_control_list_rf_lin = []; poly_control_list_rf_ang = []
poly_control_list_lf_lin = []; poly_control_list_lf_ang = []
poly_control_list_rh_lin = []; poly_control_list_rh_ang = []
poly_control_list_lh_lin = []; poly_control_list_lh_ang = []
time_vector = np.zeros(len(cs.ms_interval_data))

for spl in cs.ms_interval_data:
  x = np.array([]);
  y_com = np.array([]);  y_dcom = np.array([]);  y_ddcom = np.array([]);
  y_L = np.array([]);   y_dL = np.array([]);
  y_control_lf_lin = np.array([]);  y_control_lf_ang = np.array([]); 
  y_control_rf_lin = np.array([]);  y_control_rf_ang = np.array([]);
  y_control_rh_lin = np.array([]);  y_control_rh_ang = np.array([]);
  y_control_lh_lin = np.array([]);  y_control_lh_ang = np.array([]);
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
    if y_control_rf_lin.size==0:
      y_control_rf_lin = y[0:3]
      y_control_rf_ang = y[3:6]
      y_control_lf_lin = y[6:9]
      y_control_lf_ang = y[9:12]
      y_control_rh_lin = y[12:15]
      y_control_rh_ang = y[15:18]
      y_control_lh_lin = y[18:21]
      y_control_lh_ang = y[21:24]
    else:
      y_control_rf_lin = np.append(y_control_rf_lin, y[0:3], axis=1)
      y_control_rf_ang = np.append(y_control_rf_ang, y[3:6], axis=1)
      y_control_lf_lin = np.append(y_control_lf_lin, y[6:9], axis=1)
      y_control_lf_ang = np.append(y_control_lf_ang, y[9:12], axis=1)
      y_control_rh_lin = np.append(y_control_rh_lin, y[12:15], axis=1)
      y_control_rh_ang = np.append(y_control_rh_ang, y[15:18], axis=1)
      y_control_lh_lin = np.append(y_control_lh_lin, y[18:21], axis=1)
      y_control_lh_ang = np.append(y_control_lh_ang, y[21:24], axis=1)
  print x.shape, y_com.shape, y_control_rf_lin.shape

  traj_times.append(x)
  poly_com_list.append(dd_polyfit(x, y_com, y_dcom, y_ddcom, deg_y=5, eps=1e-20))
  #poly_L_list.append(array_polyfit(x, y_L, deg=4, full=True))
  #poly_dL_list.append(array_polyfit(x, y_dL, deg=3, full=True))
  #Fitting for control vector is the same?
  poly_control_list_rf_lin.append(array_polyfit(x, y_control_rf_lin, deg=3, full=True, eps=1e-18))
  poly_control_list_rf_ang.append(array_polyfit(x, y_control_rf_ang, deg=3, full=True, eps=1e-18))
  poly_control_list_lf_lin.append(array_polyfit(x, y_control_lf_lin, deg=3, full=True, eps=1e-18))
  poly_control_list_lf_ang.append(array_polyfit(x, y_control_lf_ang, deg=3, full=True, eps=1e-18))
  poly_control_list_rh_lin.append(array_polyfit(x, y_control_rh_lin, deg=3, full=True, eps=1e-18))
  poly_control_list_rh_ang.append(array_polyfit(x, y_control_rh_ang, deg=3, full=True, eps=1e-18))
  poly_control_list_lh_lin.append(array_polyfit(x, y_control_lh_lin, deg=3, full=True, eps=1e-18))
  poly_control_list_lh_ang.append(array_polyfit(x, y_control_lh_ang, deg=3, full=True, eps=1e-18))

time_vector = np.zeros(len(traj_times)+1)
for ts in xrange(len(traj_times)):
  time_vector[ts] = traj_times[ts][0]
time_vector[-1] = traj_times[-1][-1]
  
com_spline = spline(poly_com_list, time_vector)
rf_force = forcecurve(spline(poly_control_list_rf_lin, time_vector),
                      spline(poly_control_list_rf_ang, time_vector))
lf_force = forcecurve(spline(poly_control_list_lf_lin, time_vector),
                      spline(poly_control_list_lf_ang, time_vector))
rh_force = forcecurve(spline(poly_control_list_rh_lin, time_vector),
                      spline(poly_control_list_rh_ang, time_vector))
lh_force = forcecurve(spline(poly_control_list_lh_lin, time_vector),
                      spline(poly_control_list_lh_ang, time_vector))
if WRITE_OUTPUT:
  com_spline.save_to_file(COM_SPLINE_OUTPUT_FILE)
  rf_force.save_to_file(RF_FORCE_OUTPUT_FILE)
  lf_force.save_to_file(LF_FORCE_OUTPUT_FILE)
  rh_force.save_to_file(RH_FORCE_OUTPUT_FILE)
  lh_force.save_to_file(LH_FORCE_OUTPUT_FILE)

###############CONFIRM SPLINE OUTPUT##########################

if VERIFY:
  for spl in cs.ms_interval_data:
    for j,t in enumerate(spl.time_trajectory):
      assert(np.isclose(spl.state_trajectory[j][0:3],com_spline(t)).all())
      print j, spl.state_trajectory[j][3:6],com_spline.derivate(t,1)
      if not np.isclose(spl.state_trajectory[j][3:6],com_spline.derivate(t,1)).all():
        raw_input("Oops")
      if not np.isclose(spl.dot_state_trajectory[j][3:6],com_spline.derivate(t,2)).all():
        raw_input("Oops again")
