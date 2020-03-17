import pychrono as chrono
import pychrono.vehicle as veh
import os
from control_utilities.driver import Driver
from control_utilities.chrono_utilities import checkFile

class ChronoVehicle:
    def __init__(self, step_size, sys, controller, irrlicht=False, vehicle_type='json', initLoc=chrono.ChVectorD(0,0,0), initRot=chrono.ChQuaternionD(1,0,0,0), vis_balls=False, render_step_size=1.0/60):
        # Chrono parameters
        self.step_size = step_size
        self.irrlicht = irrlicht
        self.step_number = 0

        # Vehicle controller
        self.controller = controller

        # Initial vehicle position
        self.initLoc = initLoc

        # Initial vehicle orientation
        self.initRot = initRot

        # Point on chassis tracked by the camera (Irrlicht only)
        self.trackPoint = chrono.ChVectorD(0.0, 0.0, 1.75)

        if vehicle_type == 'json':

            # JSON file for vehicle model
            self.vehicle_file = veh.GetDataPath() + os.path.join('hmmwv', 'vehicle', 'HMMWV_Vehicle.json')
            checkFile(self.vehicle_file)

            # JSON file for powertrain (simple)
            self.simplepowertrain_file = veh.GetDataPath() + os.path.join('generic', 'powertrain', 'SimplePowertrain.json')
            checkFile(self.simplepowertrain_file)

            # JSON files tire models (rigid)
            self.rigidtire_file = veh.GetDataPath() + os.path.join('hmmwv', 'tire', 'HMMWV_RigidTire.json')
            checkFile(self.rigidtire_file)

            # --------------------------
            # Create the various modules
            # --------------------------
            if sys == None:
                self.wheeled_vehicle = veh.WheeledVehicle(self.vehicle_file)
            else:
                self.wheeled_vehicle = veh.WheeledVehicle(sys, self.vehicle_file)
            self.wheeled_vehicle.Initialize(chrono.ChCoordsysD(self.initLoc, self.initRot))
            self.wheeled_vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
            self.wheeled_vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
            self.wheeled_vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
            self.wheeled_vehicle.SetWheelVisualizationType(veh.VisualizationType_NONE)

            # Create and initialize the powertrain system
            self.powertrain = veh.SimplePowertrain(self.simplepowertrain_file)
            self.wheeled_vehicle.InitializePowertrain(self.powertrain)

            # Create and initialize the tires
            for axle in self.wheeled_vehicle.GetAxles():
                tireL = veh.RigidTire(self.rigidtire_file)
                self.wheeled_vehicle.InitializeTire(tireL, axle.m_wheels[0], veh.VisualizationType_MESH)
                tireR = veh.RigidTire(self.rigidtire_file)
                self.wheeled_vehicle.InitializeTire(tireR, axle.m_wheels[1], veh.VisualizationType_MESH)

            self.vehicle = self.wheeled_vehicle
            self.sys = self.wheeled_vehicle.GetSystem()

        elif vehicle_type == 'rccar':
            if sys == None:
                self.rc_vehicle = veh.RCCar()
                self.rc_vehicle.SetContactMethod(chrono.ChMaterialSurface.SMC)
                self.rc_vehicle.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
            else:
                self.rc_vehicle = veh.RCCar(sys)
            self.rc_vehicle.SetChassisFixed(False)
            self.rc_vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
            self.rc_vehicle.SetTireType(veh.TireModelType_RIGID)
            self.rc_vehicle.SetTireStepSize(step_size)
            self.rc_vehicle.Initialize()

            self.rc_vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
            self.rc_vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
            self.rc_vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
            self.rc_vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
            self.rc_vehicle.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

            self.vehicle = self.rc_vehicle.GetVehicle()
            self.sys = self.vehicle.GetSystem()

            self.trackPoint = chrono.ChVectorD(4, 0.0, .15)

        elif vehicle_type == 'sedan':
            if sys == None:
                self.sedan = veh.Sedan()
                self.sedan.SetContactMethod(chrono.ChMaterialSurface.NSC)
                self.sedan.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
            else:
                self.sedan = veh.Sedan(sys)
            self.sedan.SetChassisFixed(False)
            self.sedan.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
            self.sedan.SetTireType(veh.TireModelType_RIGID)
            self.sedan.SetTireStepSize(step_size)
            self.sedan.Initialize()

            self.sedan.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
            self.sedan.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
            self.sedan.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
            self.sedan.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
            self.sedan.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

            self.vehicle = self.sedan.GetVehicle()
            self.sys = self.vehicle.GetVehicle().GetSystem()

        # -------------
        # Create driver
        # -------------
        self.driver = Driver(self.vehicle)
        self.driver.SetStepSize(step_size)

        # Set the time response for steering and throttle inputs.
        # NOTE: this is not exact, since we do not render quite at the specified FPS.
        steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
        throttle_time = 1.0  # time to go from 0 to +1
        braking_time = 0.3   # time to go from 0 to +1
        self.driver.SetSteeringDelta(render_step_size / steering_time)
        self.driver.SetThrottleDelta(render_step_size / throttle_time)
        self.driver.SetBrakingDelta(render_step_size / braking_time)

        self.vis_balls = vis_balls
        if self.vis_balls:
            self.sentinel_sphere = chrono.ChBodyEasySphere(.25, 1000, False, True)
            self.sentinel_sphere.SetBodyFixed(True)
            self.sentinel_sphere.AddAsset(chrono.ChColorAsset(1,0,0))
            self.sys.Add(self.sentinel_sphere)

            self.sentinel_target = chrono.ChBodyEasySphere(.25, 1000, False, True)
            self.sentinel_target.SetBodyFixed(True)
            self.sentinel_target.AddAsset(chrono.ChColorAsset(0,1,0));
            self.sys.Add(self.sentinel_target)

        # Vehicle parameters for matplotlib
        self.length = self.vehicle.GetWheelbase() + 2.0 # [m]
        self.width = self.vehicle.GetWheeltrack(0) # [m]
        self.backtowheel = 1.0 # [m]
        self.wheel_len = self.vehicle.GetWheel(0, 1).GetWidth() * 2 # [m]
        self.wheel_width = self.vehicle.GetWheel(0, 1).GetWidth() # [m]
        self.tread = self.vehicle.GetWheeltrack(0) / 2 # [m]
        self.wb = self.vehicle.GetWheelbase() # [m]
        self.offset = [-4.0,0] # [m]

    def SetTerrain(self, terrain):
        """ Sets the terrain for this class """

        self.terrain = terrain.terrain

    def Synchronize(self, time):
        """ Synchronize driver and vehicle and update them """
        # Collect driver inputs
        driver_inputs = self.driver.GetInputs()

        # Synchronize driver and vehicle
        self.driver.Synchronize(time)
        self.vehicle.Synchronize(time, driver_inputs, self.terrain)

    def Advance(self, step):
        """ Advance vehicle for one timestep """

        self.driver.Advance(step)
        self.vehicle.Advance(step)

        # Update position of sentinel and target visualizations
        if self.vis_balls:
            target, sentinel = self.controller.GetTargetAndSentinel()
            self.sentinel_target.SetPos(target)
            self.sentinel_sphere.SetPos(sentinel)

    def GetSystem(self):
        """ Get chrono system object"""
        return self.sys

    class State:
        """ Vehicle state class """

        def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
            self.x = x
            self.y = y
            self.yaw = yaw
            self.v = v

            self.pos = chrono.ChVectorD(self.x, self.y, 0)

        def __str__(self):
            return str('({}, {}, {}, {})'.format(self.x, self.y, self.yaw, self.v))

    def GetState(self):
        """ Returns State: [x position, y position, heading, speed] """
        return self.State(
            x=self.vehicle.GetVehiclePos().x,
            y=self.vehicle.GetVehiclePos().y,
            yaw=self.vehicle.GetVehicleRot().Q_to_Euler123().z,
            v=self.vehicle.GetVehicleSpeed())
