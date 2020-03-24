import pychrono as chrono
import pychrono.vehicle as veh
import math
import os
from random import randint

def calcPose(p1, p2, z=0.0):
    """ Calculates pose (position and orientation) from two points """

    if isinstance(p1, list):
        p1 = chrono.ChVectorD(p1[0], p1[1], z)
        p2 = chrono.ChVectorD(p2[0], p2[1], z)

    loc = p1

    rot = chrono.ChQuaternionD()
    v1 = p2 - p1
    v2 = chrono.ChVectorD(1, 0, 0)
    ang = math.atan2((v1 % v2).Length(), v1 ^ v2)
    if chrono.ChVectorD(0, 0, 1) ^ (v1 % v2) > 0.0:
        ang *= -1
    rot.Q_from_AngZ(ang)

    loc.z = z
    return loc, rot

def calcAngle(p1, p2, z=0.0):
    """ Calculates angle from two points """

    if isinstance(p1, list):
        p1 = chrono.ChVectorD(p1[0], p1[1], z)
        p2 = chrono.ChVectorD(p2[0], p2[1], z)

    v1 = p2 - p1
    v2 = chrono.ChVectorD(1, 0, 0)
    ang = math.atan2((v1 % v2).Length(), v1 ^ v2)
    if chrono.ChVectorD(0, 0, 1) ^ (v1 % v2) > 0.0:
        ang *= -1

    return ang

def calcRandomPose(path, s_min=None, s_max=None):
    """ Calculate random pose (position and orientation) from progress along a path """

    if s_min == None:
        print("generateRandomOpponents :: Setting s_min")
        s_min = path.s[0]
    if s_max == None:
        print("generateRandomOpponents :: Setting s_max")
        s_max = path.s[-1]

    print(s_min)
    s_rand = randint(int(s_min), int(s_max))
    pos, _ = path.calcPosition(s_rand)
    _, rot = calcPose(path.points[path.calcIndex(pos)+1], pos)
    return pos, rot

def createChronoSystem():
    """ Default method for creating a lone chrono system"""

    sys = chrono.ChSystemNSC()
    sys.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
    sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    sys.SetSolverMaxIterations(150)
    sys.SetMaxPenetrationRecoverySpeed(4.0)
    return sys

def checkFile(file):
    if not os.path.exists(file):
        raise Exception('Cannot find {}. Please verify that CHRONO_DATA_DIR is set correctly.'.format(file))

def setDataDirectory():
    """
    Set data directory

    This is useful so data directory paths don't need to be changed everytime
    you pull from or push to github. To make this useful, make sure you perform
    step 2, as defined for your operating system.

    For Linux or Mac users:
      Replace bashrc with the shell your using. Could be .zshrc.
      1. echo 'export CHRONO_DATA_DIR=<chrono's data directory>' >> ~/.bashrc
          Ex. echo 'export CHRONO_DATA_DIR=/home/user/chrono/data/' >> ~/.zshrc
      2. source ~/.zshrc

    For Windows users:
      Link as reference: https://helpdeskgeek.com/how-to/create-custom-environment-variables-in-windows/
      1. Open the System Properties dialog, click on Advanced and then Environment Variables
      2. Under User variables, click New... and create a variable as described below
          Variable name: CHRONO_DATA_DIR
          Variable value: <chrono's data directory>
              Ex. Variable value: C:\ Users\ user\ chrono\ data\
    """
    from pathlib import Path

    CONDA_PREFIX = os.environ.get('CONDA_PREFIX')
    CHRONO_DATA_DIR = os.environ.get('CHRONO_DATA_DIR')
    if CONDA_PREFIX and not CHRONO_DATA_DIR:
        CHRONO_DATA_DIR = os.path.join(CONDA_PREFIX, "share", "chrono", "data")
    if not CHRONO_DATA_DIR:
        CHRONO_DATA_DIR = os.path.join(Path(os.path.dirname(os.path.realpath(__file__))).parents[1], "chrono", "data", "")
    elif not CHRONO_DATA_DIR:
        raise Exception('Cannot find the chrono data directory. Please verify that CHRONO_DATA_DIR is set correctly.')

    chrono.SetChronoDataPath(CHRONO_DATA_DIR)
    veh.SetDataPath(os.path.join(CHRONO_DATA_DIR, 'vehicle', ''))
