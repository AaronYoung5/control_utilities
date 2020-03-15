import pychrono as chrono
import math

def calcPose(p1, p2, z=0.5):
    if isinstance(p1, list):
        p1 = chrono.ChVectorD(p1[0], p1[1], z)
        p2 = chrono.ChVectorD(p2[0], p2[1], z)
        print('testset')

    initLoc = p1

    initRot = chrono.ChQuaternionD()
    v1 = p2 - p1
    v2 = chrono.ChVectorD(1, 0, 0)
    ang = math.atan2((v1 % v2).Length(), v1 ^ v2)
    if chrono.ChVectorD(0, 0, 1) ^ (v1 % v2) > 0.0:
        ang *= -1
    initRot.Q_from_AngZ(ang)

    return initLoc, initRot
