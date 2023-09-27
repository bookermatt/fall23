
import sys
sys.path.append("C:/Users/mattb/Documents/Aero Fall 23/Flight Mech/Hw3")
import numpy as np
from math import cos, sin, tan
import scipy.linalg as linalg
import parameters.simulation_parameters as SIM
import parameters.aerosonde_parameters as P
from dynamics.mavDynamics import MavDynamics
from viewers.uav_animation import uav_animation
from tools.signalGenerator import signalGenerator
import matplotlib.pyplot as plt

state=P.states0
mav=MavDynamics()
plane_anim=uav_animation(state, scale = 1)

# angles = plane_anim.fig.add_subplot(3,2,5)
# translations = plane_anim.fig.add_subplot(3,2,2)
# angledx = plane_anim.fig.add_subplot(3,2,6)
# translatedx = plane_anim.fig.add_subplot(3,2,5)

# generate focres and moments
forces = signalGenerator(amplitude = 2000, frequency = 1)
moments = signalGenerator(amplitude = 15, frequency = 2)

# initialize the simulation time
sim_time = SIM.start_time

ts = []
fxs = []
fys = []
fzs = []
ls = []
ms = []
ns = []
pns = []
pes = []
pds = []
us = []
vs = []
ws = []
phis = []
thetas = []
psis = []
ps = []
qs = []
rs = []

pn = state[0,0]
pe = state[1,0]
pd = state[2,0]
u = state[3,0]
v = state[4,0]
w = state[5,0]
phi = state[6,0]
theta = state[7,0]
psi = state[8,0]
p = state[9,0]
q = state[10,0]
r = state[11,0]

while sim_time < SIM.end_time:
    # set forces per one period
    if sim_time <= 0.5:
        fx = forces.sin(sim_time)
        fy = 0
        fz = 0
        l = 0
        m = 0
        n = 0
    elif sim_time <= 1:
        fx = 0
        fy = forces.sin(sim_time)
    elif sim_time <= 1.5:
        fy = 0
        fz = -forces.sin(sim_time)
    elif sim_time <= 2:
        fz = 0
        l = moments.sin(sim_time)
    elif sim_time <= 2.5:
        l = 0
        m = moments.sin(sim_time)
    elif sim_time <= 3:
        m = 0
        n = moments.sin(sim_time)
    else:
        n = 0
    
    # update everything
    y = mav.update(fx, fy, fz, l, m, n)
    plane_anim.update(y[0][0], y[1][0], y[2][0], y[6][0], y[7][0], y[8][0])
   
    # store data in lists from earlier
    ts.append(sim_time)
    fxs.append(fx)
    fys.append(fy)
    fzs.append(fz)
    ls.append(l)
    ms.append(m)
    ns.append(n)

    pns.append(y[0][0])
    pes.append(y[1][0])
    pds.append(y[2][0])
    us.append(y[3][0])
    vs.append(y[4][0])
    ws.append(y[5][0])
    phis.append(y[6][0])
    thetas.append(y[7][0])
    psis.append(y[8][0])
    ps.append(y[9][0])
    qs.append(y[10][0])
    rs.append(y[11][0])

    # plot data on subplots
    # angles.clear()
    # angledx.clear()
    # translations.clear()
    # translatedx.clear()
    # angledx.plot(ts, np.rad2deg(ps), "b-", label="p")
    # angledx.plot(ts, np.rad2deg(qs), "r-", label="q")
    # angledx.plot(ts, np.rad2deg(rs), "g-", label="r")
    # translatedx.plot(ts, us, "b-", label="u")
    # translatedx.plot(ts, vs, "r-", label="v")
    # translatedx.plot(ts, ws, "g-", label="w")
    # angles.plot(ts, np.rad2deg(phis), "r-", label="$\phi$")
    # angles.plot(ts, np.rad2deg(thetas), "g-", label="$\ttheta$")
    # angles.plot(ts, np.rad2deg(psis), "b-", label="$\psi$")
    # translations.plot(ts, pns, "m-", label="North")
    # translations.plot(ts, pes, "y-", label="East")
    # translations.plot(ts, pds, "k-", label="Down (z)")
    # angles.grid('major')
    # angles.set_xlabel('Times (s)')
    # angles.set_ylabel('Angle (degrees)')
    # angles.legend(loc='upper right')
    # translations.grid('major')
    # translations.set_xlabel('Times (s)')
    # translations.set_ylabel('Distance (m)')
    # translations.legend(loc='upper right')
    # angledx.grid('major')
    # angledx.set_xlabel('Times (s)')
    # angledx.set_ylabel('Angular Velocity (degrees/s)')
    # angledx.legend(loc='upper right')
    # translatedx.grid('major')
    # translatedx.set_xlabel('Times (s)')
    # translatedx.set_ylabel('Velocity (m/s)')
    # translatedx.legend(loc='upper right')

    

    # -------increment time-------------
    sim_time += SIM.ts_simulation