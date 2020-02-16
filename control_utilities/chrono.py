import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import os
import math
import sys
from control_utilities.driver import Driver

# ----------------------------------------------------------------------------------------------------
# Set data directory
#
# This is useful so data directory paths don't need to be changed everytime
# you pull from or push to github. To make this useful, make sure you perform
# step 2, as defined for your operating system.
#
# For Linux or Mac users:
#   Replace bashrc with the shell your using. Could be .zshrc.
#   1. echo 'export CHRONO_DATA_DIR=<chrono's data directory>' >> ~/.bashrc
#       Ex. echo 'export CHRONO_DATA_DIR=/home/user/chrono/data/' >> ~/.zshrc
#   2. source ~/.zshrc
#
# For Windows users:
#   Link as reference: https://helpdeskgeek.com/how-to/create-custom-environment-variables-in-windows/
#   1. Open the System Properties dialog, click on Advanced and then Environment Variables
#   2. Under User variables, click New... and create a variable as described below
#       Variable name: CHRONO_DATA_DIR
#       Variable value: <chrono's data directory>
#           Ex. Variable value: C:\Users\user\chrono\data\
# ----------------------------------------------------------------------------------------------------

try:
    chrono.SetChronoDataPath(os.environ['CHRONO_DATA_DIR'])
    veh.SetDataPath(os.path.join(os.environ['CHRONO_DATA_DIR'], 'vehicle', ''))
except:
    raise Exception('Cannot find CHRONO_DATA_DIR environmental variable. Explanation located in chrono_sim.py file')

def GetInitPose(p1, p2, z=0.5, reversed=0):
    p1 = chrono.ChVectorD(p1[0], p1[1], z)
    p2 = chrono.ChVectorD(p2[0], p2[1], z)

    initLoc = p1

    vec = p2 - p1
    theta = math.atan2((vec%chrono.ChVectorD(1,0,0)).Length(),vec^chrono.ChVectorD(1,0,0))
    if reversed:
        theta *= -1
    initRot = chrono.ChQuaternionD()
    initRot.Q_from_AngZ(theta)

    return initLoc, initRot

def checkFile(file):
    if not os.path.exists(file):
        raise Exception('Cannot find {}. Explanation located in chrono_sim.py file'.format(file))

class ChronoSim:
    def __init__(self, step_size, track, irrlicht=False, initLoc=chrono.ChVectorD(0,0,0), initRot=chrono.ChQuaternionD(1,0,0,0)):
        # Vehicle parameters for matplotlib
        f = 2
        self.length = 4.5  * f# [m]
        self.width = 2.0  * f# [m]
        self.backtowheel = 1.0  * f# [m]
        self.wheel_len = 0.3  * f# [m]
        self.wheel_width = 0.2  * f# [m]
        self.tread = 0.7  * f# [m]
        self.wb = 2.5  * f# [m]

        # Chrono parameters
        self.step_size = step_size
        self.irrlicht = irrlicht
        self.step_number = 0

        # Time interval between two render frames
        self.render_step_size = 1.0 / 60  # FPS = 60
        self.render_steps = int(math.ceil(self.render_step_size / self.step_size))

        # JSON file for vehicle model
        self.vehicle_file = veh.GetDataPath() + os.path.join('hmmwv', 'vehicle', 'HMMWV_Vehicle.json')
        checkFile(self.vehicle_file)

        # JSON files for terrain
        self.rigidterrain_file = veh.GetDataPath() + os.path.join('terrain', 'RigidPlane.json')
        checkFile(self.rigidterrain_file)

        # JSON file for powertrain (simple)
        self.simplepowertrain_file = veh.GetDataPath() + os.path.join('generic', 'powertrain', 'SimplePowertrain.json')
        checkFile(self.simplepowertrain_file)

        # JSON files tire models (rigid)
        self.rigidtire_file = veh.GetDataPath() + os.path.join('hmmwv', 'tire', 'HMMWV_RigidTire.json')
        checkFile(self.rigidtire_file)

        # Initial vehicle position
        self.initLoc = initLoc

        # Initial vehicle orientation
        self.initRot = initRot

        # Rigid terrain dimensions
        self.terrainHeight = 0
        self.terrainLength = 300.0  # size in X direction
        self.terrainWidth = 300.0  # size in Y direction

        # Point on chassis tracked by the camera (Irrlicht only)
        self.trackPoint = chrono.ChVectorD(0.0, 0.0, 1.75)

        self.track = track

        # --------------------------
        # Create the various modules
        # --------------------------

        self.vehicle = veh.WheeledVehicle(
            self.vehicle_file, chrono.ChMaterialSurface.NSC)
        self.vehicle.Initialize(chrono.ChCoordsysD(self.initLoc, self.initRot))
        self.vehicle.SetChassisVisualizationType(
            veh.VisualizationType_PRIMITIVES)
        self.vehicle.SetSuspensionVisualizationType(
            veh.VisualizationType_PRIMITIVES)
        self.vehicle.SetSteeringVisualizationType(
            veh.VisualizationType_PRIMITIVES)
        self.vehicle.SetWheelVisualizationType(veh.VisualizationType_NONE)

        # Create and initialize the powertrain system
        self.powertrain = veh.SimplePowertrain(self.simplepowertrain_file)
        self.vehicle.InitializePowertrain(self.powertrain)

        # Create and initialize the tires
        for axle in self.vehicle.GetAxles():
            tireL = veh.RigidTire(self.rigidtire_file)
            self.vehicle.InitializeTire(
                tireL, axle.m_wheels[0], veh.VisualizationType_MESH)
            tireR = veh.RigidTire(self.rigidtire_file)
            self.vehicle.InitializeTire(
                tireR, axle.m_wheels[1], veh.VisualizationType_MESH)

        # Create the ground
        self.terrain = veh.RigidTerrain(
            self.vehicle.GetSystem(), self.rigidterrain_file)
        # -------------
        # Create driver
        # -------------
        self.driver = Driver(self.vehicle)

        # Set the time response for steering and throttle inputs.
        # NOTE: this is not exact, since we do not render quite at the specified FPS.
        self.steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
        self.throttle_time = 1.0  # time to go from 0 to +1
        self.braking_time = 0.3   # time to go from 0 to +1
        self.driver.SetSteeringDelta(
            self.render_step_size / self.steering_time)
        self.driver.SetThrottleDelta(
            self.render_step_size / self.throttle_time)
        self.driver.SetBrakingDelta(self.render_step_size / self.braking_time)

        if self.irrlicht:
            self.DrawTrack(track)

        if self.irrlicht:
            self.app = veh.ChVehicleIrrApp(self.vehicle)
            self.app.SetHUDLocation(500, 20)
            self.app.SetSkyBox()
            self.app.AddTypicalLogo()
            self.app.AddTypicalLights(chronoirr.vector3df(-150., -150., 200.), chronoirr.vector3df(-150., 150., 200.), 100,
                                      100)
            self.app.AddTypicalLights(chronoirr.vector3df(150., -150., 200.), chronoirr.vector3df(150., 150., 200.), 100,
                                      100)
            self.app.EnableGrid(False)
            self.app.SetChaseCamera(self.trackPoint, 6.0, 0.5)

            self.app.SetTimestep(self.step_size)
            # ---------------------------------------------------------------------
            #
            #  Create an Irrlicht application to visualize the system
            #
            # ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
            # in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
            # If you need a finer control on which item really needs a visualization proxy
            # Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

            self.app.AssetBindAll()

            # ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
            # that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

            self.app.AssetUpdateAll()

    def DrawTrack(self, track, z=0.5):
        if self.irrlicht:
            road = self.vehicle.GetSystem().NewBody()
            road.SetBodyFixed(True)
            self.vehicle.GetSystem().AddBody(road)

            def toChPath(path):
                ch_path = chrono.vector_ChVectorD()
                for x,y in zip(path.x, path.y):
                    point = chrono.ChVectorD(x, y, z)
                    ch_path.push_back(point)
                return chrono.ChBezierCurve(ch_path)

            num_points = len(track.center.x)
            path_asset = chrono.ChLineShape()
            path_asset.SetLineGeometry(
                chrono.ChLineBezier(toChPath(track.center)))
            path_asset.SetColor(chrono.ChColor(0.0, 0.8, 0.0))
            path_asset.SetNumRenderPoints(max(2 * num_points, 400))
            road.AddAsset(path_asset)

    def Advance(self, step):
        if self.irrlicht:
            if not self.app.GetDevice().run():
                return -1
            if self.step_number % self.render_steps == 0:
                self.app.BeginScene(
                    True, True, chronoirr.SColor(255, 140, 161, 192))
                self.app.DrawAll()
                self.app.EndScene()
        else:
            self.vehicle.GetSystem().DoStepDynamics(step)

        # Collect output data from modules (for inter-module communication)
        driver_inputs = self.driver.GetInputs()

        # Update modules (process inputs from other modules)
        time = self.vehicle.GetSystem().GetChTime()

        self.driver.Synchronize(time)
        self.vehicle.Synchronize(time, driver_inputs, self.terrain)
        self.terrain.Synchronize(time)
        if self.irrlicht:
            self.app.Synchronize("", driver_inputs)

        # Advance simulation for one timestep for all modules
        self.driver.Advance(step)
        self.vehicle.Advance(step)
        self.terrain.Advance(step)
        if self.irrlicht:
            self.app.Advance(step)
            self.step_number += 1

    def Close(self):
        if self.app.GetDevice().run():
            self.app.GetDevice().closeDevice()
        self.irrlicht = False

    class State:
        """
        vehicle state class
        """

        def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
            self.x = x
            self.y = y
            self.yaw = yaw
            self.v = v

        def __str__(self):
            return str('({}, {}, {}, {})'.format(self.x, self.y, self.yaw, self.v))

    def GetState(self):
        """
        Returns State:
            [x,y,v,heading]
        """
        return self.State(
            x=self.vehicle.GetVehiclePos().x,
            y=self.vehicle.GetVehiclePos().y,
            yaw=self.vehicle.GetVehicleRot().Q_to_Euler123().z,
            v=self.vehicle.GetVehicleSpeed())
