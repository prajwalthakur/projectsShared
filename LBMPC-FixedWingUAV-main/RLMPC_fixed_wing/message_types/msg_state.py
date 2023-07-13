"""
msgState 
    - messages type for state, that will be passed between blocks in the architecture
    
part of mavPySim 
    - Beard & McLain, PUP, 2012
    - Update history:  
        1/9/2019 - RWB
"""


class msg_state:
    def __init__(self):
        self.pn = 0.  # inertial north position in meters
        self.pe = 0.  # inertial east position in meters
        self.h = 0.  # inertial altitude in meters
        self.phi = 0.  # roll angle in radians
        self.theta = 0.  # pitch angle in radians
        self.psi = 0.  # yaw angle in radians
        self.Va = 0.  # airspeed in meters/sec
        self.alpha = 0.  # angle of attack in radians
        self.beta = 0.  # sideslip angle in radians
        self.p = 0.  # roll rate in radians/sec
        self.q = 0.  # pitch rate in radians/sec
        self.r = 0.  # yaw rate in radians/sec
        self.Vg = 0.  # groundspeed in meters/sec
        self.gamma = 0.  # flight path angle in radians
        self.chi = 0.  # course angle in radians
        self.wn = 0.  # inertial windspeed in north direction in meters/sec
        self.we = 0.  # inertial windspeed in east direction in meters/sec
        self.bx = 0.  # gyro bias along roll axis in radians/sec
        self.by = 0.  # gyro bias along pitch axis in radians/sec
        self.bz = 0.  # gyro bias along yaw axis in radians/sec
        self.ug = 0.
        self.vg = 0.
        self.wg = 0.
        self.ur = 0.
        self.vr = 0.
        self.wr = 0.
        # ur = self._state.item(3) - wind2.item(0)
        # vr = self._state.item(4) - wind2.item(1)
        # wr = self._state.item(5) - wind2.item(2)
