import pychrono as chrono
import pychrono.vehicle as veh

from control_utilities.chrono_utilities import checkFile

class ChronoTerrain:
    def __init__(self, step_size, sys, irrlicht=False, terrain_type='json', height=-0.5, width=300, length=300):
        # Chrono parameters
        self.step_size = step_size
        self.irrlicht = irrlicht
        self.step_number = 0

        # Rigid terrain dimensions
        self.height = height
        self.length = length  # size in X direction
        self.width = width  # size in Y direction

        self.sys = sys

        if terrain_type == 'json':
            import os
            # JSON files for terrain
            self.rigidterrain_file = veh.GetDataPath() + os.path.join('terrain', 'RigidPlane.json')
            checkFile(self.rigidterrain_file)

            # Create the ground
            self.terrain = veh.RigidTerrain(self.sys, self.rigidterrain_file)

        elif terrain_type == 'concrete':
            # Create the terrain
            self.terrain = veh.RigidTerrain(self.sys)
            patch = self.terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, self.height - 5), chrono.QUNIT),
                                     chrono.ChVectorD(self.width, self.length, 10))

            patch.SetContactFrictionCoefficient(0.9)
            patch.SetContactRestitutionCoefficient(0.01)
            patch.SetContactMaterialProperties(2e7, 0.3)
            patch.SetTexture(chrono.GetChronoDataFile("concrete.jpg"), self.length, self.width)
            patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
            self.terrain.Initialize()

            try:
                ground_body = patch.GetGroundBody()
                ground_asset = ground_body.GetAssets()[0]
                visual_asset = chrono.CastToChVisualization(ground_asset)
                vis_mat = chrono.ChVisualMaterial()
                vis_mat.SetKdTexture(chrono.GetChronoDataFile("concrete.jpg"))
                vis_mat.SetFresnelMax(0);
                visual_asset.material_list.append(vis_mat)
            except:
                print("Not Visual Material")

        elif terrain_type == 'hallway':
            y_max = 5.65
            x_max = 23
            offset = chrono.ChVectorD(-x_max/2, -y_max/2, .21)
            offsetF = chrono.ChVectorF(offset.x, offset.y, offset.z)

            self.terrain = veh.RigidTerrain(self.sys)
            coord_sys = chrono.ChCoordsysD(offset, chrono.ChQuaternionD(1,0,0,0))
            patch = self.terrain.AddPatch(coord_sys, chrono.GetChronoDataFile("sensor/textures/hallway_contact.obj"), "mesh", 0.01, False)

            vis_mesh = chrono.ChTriangleMeshConnected()
            vis_mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("sensor/textures/hallway.obj"), True, True)

            trimesh_shape = chrono.ChTriangleMeshShape()
            trimesh_shape.SetMesh(vis_mesh)
            trimesh_shape.SetName("mesh_name")
            trimesh_shape.SetStatic(True)

            patch.GetGroundBody().AddAsset(trimesh_shape)

            patch.SetContactFrictionCoefficient(0.9)
            patch.SetContactRestitutionCoefficient(0.01)
            patch.SetContactMaterialProperties(2e7, 0.3)

            self.terrain.Initialize()

        elif terrain_type == 'off-road':
            self.terrain = veh.RigidTerrain(self.sys)
            patch = self.terrain.AddPatch(chrono.CSYSNORM, veh.GetDataFile("terrain/height_maps/test.bmp"), "test", 128, 128, -4, 0);
            patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 16, 16);

            patch.SetContactFrictionCoefficient(0.9)
            patch.SetContactRestitutionCoefficient(0.01)
            patch.SetContactMaterialProperties(2e7, 0.3)
            patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
            self.terrain.Initialize()

    def Synchronize(self, time):
        self.terrain.Synchronize(time)

    def Advance(self, step):
        self.terrain.Advance(step)
