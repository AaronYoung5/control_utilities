from random import randint
from control_utilities.chrono_utilities import calcPose
from control_utilities.chrono_vehicle import ChronoVehicle

import math
import pychrono as chrono

class Opponent:
    def __init__(self, vehicle, controller):
        self.vehicle = vehicle
        self.controller = controller

    def Update(self, time, step):
        # Synchronize vehicle
        self.vehicle.Synchronize(time)

        # Update controller
        self.controller.Advance(step, self.vehicle)

        # Advance vehicle
        self.vehicle.Advance(step)

    class State:
        """ Opponent state class """

        def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, pos_dt=chrono.ChVectorD(0,0,0)):
            self.x = x
            self.y = y
            self.yaw = yaw
            self.v = v
            self.pos_dt = pos_dt

            self.pos = chrono.ChVectorD(self.x, self.y, 0)

        def __str__(self):
            return str('({}, {}, {}, {})'.format(self.x, self.y, self.yaw, self.v))

    def GetState(self):
        """ Returns State: [x position, y position, heading, speed, position derivative] """
        return self.State(
            x=self.vehicle.vehicle.GetVehiclePos().x,
            y=self.vehicle.vehicle.GetVehiclePos().y,
            yaw=self.vehicle.vehicle.GetVehicleRot().Q_to_Euler123().z,
            v=self.vehicle.vehicle.GetVehicleSpeed(),
            pos_dt=self.vehicle.vehicle.GetChassisBody().GetFrame_REF_to_abs().GetPos_dt()
            )

def opponentInSight(current_state, opponent_state, perception_distance=20, field_of_view=chrono.CH_C_PI/4):
    v1 = opponent_state.pos - current_state.pos
    v2 = opponent_state.pos_dt / opponent_state.pos_dt.Length()
    ang = math.atan2((v1 % v2).Length(), v1 ^ v2)
    if chrono.ChVectorD(0, 0, 1) ^ (v1 % v2) > 0.0:
        ang *= -1

    # Return true if within certain distance and infront of vehicle
    return (current_state.pos - opponent_state.pos).Length() <= perception_distance and abs(ang) < field_of_view
