from random import randint
from control_utilities.chrono_utilities import calcPose
from control_utilities.driver import Driver

import math
import pychrono as chrono

class Opponent:
    def __init__(self, pos, rot, path, controller=None, lat_controller=None, long_controller=None):
        self.pos = pos
        self.rot = rot
        self.pos_dt = None
        self.path = path

        if controller == None and (lat_controller == None or long_controller == None):
            raise Exception("MovingObstacle Class :: Must pass in either a single controller or a steering and throttle controllers.")
        elif controller != None and (lat_controller != None or long_controller != None):
            print("You passed in a controller and a steering and/or throttle controllers. Setting controller. Please only choose one type.")

        self.controller = controller
        self.lat_controller = lat_controller
        self.long_controller = long_controller
        self.single_controller = (controller != None)

    def Update(self, step, vehicle, driver):
        # Update controllers
        if self.single_controller:
            steering, throttle, braking = self.controller.Advance(step, vehicle, driver)
        else:
            steering = self.lat_controller.Advance(step, vehicle)
            throttle, braking = self.long_controller.Advance(step, vehicle, driver)

        driver.SetTargetSteering(steering)
        driver.SetTargetThrottle(throttle)
        driver.SetTargetBraking(braking)

        self.pos = vehicle.GetVehiclePos()
        self.pos_dt = vehicle.GetChassisBody().GetFrame_REF_to_abs().GetPos_dt()


def generateRandomOpponent(path, s_min=None, s_max=None, controller=None, lat_controller=None, long_controller=None):
    if s_min == None:
        print("generateRandomOpponents :: Setting s_min")
        s_min = path.s[0]
    if s_max == None:
        print("generateRandomOpponents :: Setting s_max")
        s_max = path.s[-1]

    s_rand = randint(int(s_min), int(s_max))
    pos, _ = path.calcPosition(s_rand)
    print(s_rand, pos)
    p, rot = calcPose(path.points[path.calcIndex(pos)+1], pos)
    print(pos, path.points[path.calcIndex(pos)+1], p)
    return Opponent(pos=pos, rot=rot, path=path, controller=controller, lat_controller=lat_controller, long_controller=long_controller)

def opponentInSight(current_pos, opponent_pos, pos_dt, perception_distance):
    if pos_dt == None:
        return
    v1 = opponent_pos - current_pos
    v2 = pos_dt / pos_dt.Length()
    ang = math.atan2((v1 % v2).Length(), v1 ^ v2)
    if chrono.ChVectorD(0, 0, 1) ^ (v1 % v2) > 0.0:
        ang *= -1

    # Return true if within certain distance and infront of vehicle
    return (current_pos - opponent_pos).Length() <= perception_distance and abs(ang) < chrono.CH_C_PI / 2
