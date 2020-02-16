import pychrono.vehicle as veh

class Driver(veh.ChDriver):
    def __init__(self,vehicle):
        veh.ChDriver.__init__(self, vehicle)

        self.steering_target = 0
        self.throttle_target = 0
        self.braking_target = 0

        self.steering_delta = 1.0 / 50
        self.throttle_delta = 1.0 / 50
        self.braking_delta = 1.0 / 50

        self.steering_gain = 4.0
        self.throttle_gain = 4.0
        self.braking_gain = 4.0

    def SetGains(self, steering_gain, throttle_gain, braking_gain):
        self.steering_gain = steering_gain
        self.throttle_gain = throttle_gain
        self.braking_gain = braking_gain

    def Advance(self, step):
        # Integrate dynamics, taking as many steps as required to reach the value 'step'
        t = 0.0
        while t < step:
            h = step - t

            throttle_deriv = self.throttle_gain * (self.throttle_target - self.GetThrottle())
            steering_deriv = self.steering_gain * (self.steering_target - self.GetSteering())
            braking_deriv = self.braking_gain * (self.braking_target - self.GetBraking())

            self.SetThrottle(self.GetThrottle() + h * throttle_deriv)
            self.SetSteering(self.GetSteering() + h * steering_deriv)
            self.SetBraking(self.GetBraking() + h * braking_deriv)

            t = t + h

    def GetTargetThrottle(self):
        return self.throttle_target

    def SetTargetThrottle(self, throttle):
        self.throttle_target = throttle

    def SetTargetSteering(self, steering):
        self.steering_target = steering

    def SetTargetBraking(self, braking):
        self.braking_target = braking

    def SetThrottleDelta(self, throttle):
        self.throttle_delta = throttle

    def SetSteeringDelta(self, steering):
        self.steering_delta = steering

    def SetBrakingDelta(self, braking):
        self.braking_delta = braking
