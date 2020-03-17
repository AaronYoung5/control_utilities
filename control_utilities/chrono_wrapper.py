import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

from control_utilities.chrono_utilities import checkFile
from control_utilities.obstacle import RandomObstacleGenerator

import math

class ChronoWrapper:
    def __init__(self, step_size, system, track, vehicle, terrain, irrlicht=False, obstacles=None, opponents=None, draw_barriers=False, draw_cones=False):
        # Chrono parameters
        self.step_size = step_size
        self.irrlicht = irrlicht
        self.step_number = 0

        # Time interval between two render frames
        self.render_step_size = 1.0 / 60  # FPS = 60
        self.render_steps = int(math.ceil(self.render_step_size / self.step_size))

        # Track that vehicle is going through
        self.track = track

        # Static obstacles in the environment
        self.obstacles = obstacles

        # Dynamic opponents in the environment
        self.opponents = opponents

        self.system = system
        self.vehicle = vehicle
        self.terrain = terrain

        if self.irrlicht:
            self.DrawTrack(track, .5)

            if obstacles != None:
                self.DrawObstacles(obstacles)

            if opponents != None:
                temp = dict()
                for opponent in opponents:
                    temp[opponent] = (opponent.vehicle, opponent.vehicle.driver)
                self.opponents = temp

            if draw_barriers:
                self.DrawBarriers(self.track.left.points)
                self.DrawBarriers(self.track.right.points)
            if draw_cones:
                self.DrawCones(self.track.left.points, 'red')
                self.DrawCones(self.track.right.points, 'green')

        if self.irrlicht:
            self.app = veh.ChVehicleIrrApp(self.vehicle.vehicle)
            self.app.SetHUDLocation(500, 20)
            self.app.SetSkyBox()
            self.app.AddTypicalLogo()
            self.app.AddTypicalLights(chronoirr.vector3df(-150., -150., 200.), chronoirr.vector3df(-150., 150., 200.), 100, 100)
            self.app.AddTypicalLights(chronoirr.vector3df(150., -150., 200.), chronoirr.vector3df(150., 150., 200.), 100, 100)
            self.app.EnableGrid(False)
            self.app.SetChaseCamera(self.vehicle.trackPoint, 6.0, 0.5)

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

    def DrawTrack(self, track, z=0.0):
        road = self.system.NewBody()
        road.SetBodyFixed(True)
        self.system.AddBody(road)

        def toChPath(path):
            ch_path = chrono.vector_ChVectorD()
            for x,y in zip(path.x, path.y):
                point = chrono.ChVectorD(x, y, z)
                ch_path.push_back(point)
            return chrono.ChBezierCurve(ch_path)

        num_points = len(track.center.x)
        path_asset = chrono.ChLineShape()
        path_asset.SetLineGeometry(chrono.ChLineBezier(toChPath(track.center)))
        path_asset.SetColor(chrono.ChColor(0.0, 0.8, 0.0))
        path_asset.SetNumRenderPoints(max(2 * num_points, 400))
        road.AddAsset(path_asset)

    def DrawBarriers(self, points, n=10, height=1, width=1):
        points = points[::n]
        if points[-1] != points[0]:
            points.append(points[-1])
        for i in range(len(points) - 1):
            p1 = points[i]
            p2 = points[i + 1]
            box = chrono.ChBodyEasyBox((p2 - p1).Length(), height, width, 1000, True, True)
            box.SetPos(p1)

            q = chrono.ChQuaternionD()
            v1 = p2 - p1
            v2 = chrono.ChVectorD(1, 0, 0)
            ang = math.atan2((v1 % v2).Length(), v1 ^ v2)
            if chrono.ChVectorD(0, 0, 1) ^ (v1 % v2) > 0.0:
                ang *= -1
            q.Q_from_AngZ(ang)
            box.SetRot(q)
            box.SetBodyFixed(True)

            color = chrono.ChColorAsset()
            if i % 2 == 0:
                color.SetColor(chrono.ChColor(1, 0, 0))
            else:
                color.SetColor(chrono.ChColor(1, 1, 1))
            box.AddAsset(color)
            self.system.Add(box)

    def DrawCones(self, points, color, z=.3, n=10):
        for p in points[::n]:
            p.z += z
            cmesh = chrono.ChTriangleMeshConnected()
            if color=='red':
                cmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("sensor/cones/red_cone.obj"), False, True)
            elif color=='green':
                cmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("sensor/cones/green_cone.obj"), False, True)

            cshape = chrono.ChTriangleMeshShape()
            cshape.SetMesh(cmesh)
            cshape.SetName("Cone")
            cshape.SetStatic(True)

            cbody = chrono.ChBody()
            cbody.SetPos(p)
            cbody.AddAsset(cshape)
            cbody.SetBodyFixed(True)
            cbody.SetCollide(False)
            if color=='red':
                cbody.AddAsset(chrono.ChColorAsset(1,0,0))
            elif color=='green':
                cbody.AddAsset(chrono.ChColorAsset(0,1,0))

            self.system.Add(cbody)

    def DrawObstacles(self, obstacles, z=0.0):
        self.boxes = []
        for i, o in obstacles.items():
            p1 = o.p1
            p2 = o.p2
            box = chrono.ChBodyEasyBox(o.length, o.width, 1, 1000, True, True)
            box.SetPos(p1)

            q = chrono.ChQuaternionD()
            v1 = p2 - p1
            v2 = chrono.ChVectorD(1, 0, 0)
            ang = math.atan2((v1 % v2).Length(), v1 ^ v2)
            if chrono.ChVectorD(0, 0, 1) ^ (v1 % v2) > 0.0:
                ang *= -1
            q.Q_from_AngZ(ang)
            box.SetRot(q)
            box.SetBodyFixed(True)
            box_asset = box.GetAssets()[0]
            visual_asset = chrono.CastToChVisualization(box_asset)

            self.system.Add(box)
            self.boxes.append(box)

    def Advance(self, step):
        if self.irrlicht:
            if not self.app.GetDevice().run():
                return -1
            if self.step_number % self.render_steps == 0:
                self.app.BeginScene(
                    True, True, chronoirr.SColor(255, 140, 161, 192))
                self.app.DrawAll()
                self.app.EndScene()

        # Update modules (process inputs from other modules)
        time = self.system.GetChTime()
        self.vehicle.Synchronize(time)
        self.terrain.Synchronize(time)
        if self.irrlicht:
            self.app.Synchronize("", self.vehicle.driver.GetInputs())

        if self.obstacles != None:
            for n in range(len(self.obstacles)):
                i = list(self.obstacles)[n]
                obstacle = self.obstacles[i]
                if obstacle.Update(step):
                    self.obstacles = RandomObstacleGenerator.moveObstacle(self.track.center, self.obstacles, obstacle, i)
                    self.boxes[n].SetPos(obstacle.p1)
                    q = chrono.ChQuaternionD()
                    v1 = obstacle.p2 - obstacle.p1
                    v2 = chrono.ChVectorD(1, 0, 0)
                    ang = math.atan2((v1 % v2).Length(), v1 ^ v2)
                    if chrono.ChVectorD(0, 0, 1) ^ (v1 % v2) > 0.0:
                        ang *= -1
                    q.Q_from_AngZ(ang)
                    self.boxes[n].SetRot(q)
        if self.opponents != None:
            for opponent in self.opponents:
                opponent.Update(time, step)


        # Advance simulation for one timestep for all modules
        self.vehicle.Advance(step)
        self.terrain.Advance(step)
        if self.irrlicht:
            self.app.Advance(step)
            self.step_number += 1

        self.system.DoStepDynamics(step)

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
