import numpy as np
import math
import matplotlib.pyplot as plt
import time
from control_utilities.chrono_utilities import calcAngle

class MatplotlibWrapper:
    def __init__(self, step_size, vehicle, opponents=None, obstacles=None, render_step_size=1.0/60):
        self.step_size = step_size
        self.time = 0

        # Time interval between two render frames
        self.render_step_size = render_step_size  # FPS = 60
        self.render_steps = int(math.ceil(self.render_step_size / self.step_size))
        self.step_number = 0
        self.vehicle = vehicle
        self.vehicles = dict()
        self.vehicles[self.vehicle] = (0)
        self.opponents = opponents
        if self.opponents != None:
            for opponent in self.opponents:
                self.vehicles[opponent.vehicle] = (0)

        self.obstacles = obstacles
        if self.obstacles != None:
            self.obstacle_outlines = dict()
            for _, obstacle in self.obstacles.items():
                self.obstacle_outlines[obstacle.num] = (0)

        self.fig = plt.figure(figsize=(8, 5))

    def close(self):
        plt.close()

    def plotTrack(self, track):
        # track.center.plot(color='-r', show=False)
        track.left.plot(color='-k', show=False)
        track.right.plot(color='-k', show=False)

    def plotOpponents(self):
        for opponent in self.opponents:
            self.plotVehicle(opponent.vehicle, cabcolor="-b", wheelcolor="-k")

    def Advance(self, step, save=False):
        if self.step_number % self.render_steps == 0:
            self.plotVehicle(self.vehicle)
            self.plotText()
            if self.opponents != None:
                self.plotOpponents()
            if self.obstacles != None:
                self.plotObstacles()
            plt.pause(1e-9)
            if save:
                file_name = "fig{}.png".format(int(self.step_number/5))
                print("Saving to {}".format(file_name))
                plt.savefig(file_name, dpi=300, quality=80, optimize=True, progressive=True, format="jpg")
            if len(plt.get_fignums()) == 0:
                return False
        self.step_number += 1
        self.time += step
        return True

    def plotText(self):
        str = 'Time :: {0:0.1f}\nThrottle :: {1:0.2f}\nSteering :: {2:0.2f}\nBraking :: {3:0.2f}\nSpeed :: {4:0.2f}'.format(
            self.time, self.vehicle.driver.GetThrottle(), self.vehicle.driver.GetSteering(), self.vehicle.driver.GetBraking(), self.vehicle.vehicle.GetVehicleSpeed())
        if not hasattr(self, 'annotation'):
            bbox_props = dict(boxstyle="round", fc="w", ec="0.5", alpha=0.9)
            self.annotation = plt.annotate(str, xy=(.97, .7), xytext=(0, 10), xycoords=('axes fraction', 'figure fraction'), textcoords='offset points', size=10, ha='right', va='bottom',bbox=bbox_props)
        else:
            self.annotation.set_text(str)
            # Now let's add your additional information

    def plotVehicle(self, vehicle, cabcolor="-k", wheelcolor="-k"):  # pragma: no cover
        state = vehicle.GetState()

        outline = np.array([[-vehicle.backtowheel, (vehicle.length - vehicle.backtowheel), (vehicle.length - vehicle.backtowheel), -vehicle.backtowheel, -vehicle.backtowheel],
                            [vehicle.width / 2, vehicle.width / 2, - vehicle.width / 2, -vehicle.width / 2, vehicle.width / 2]])

        fr_wheel = np.array([[vehicle.wheel_len, -vehicle.wheel_len, -vehicle.wheel_len, vehicle.wheel_len, vehicle.wheel_len],
                             [-vehicle.wheel_width - vehicle.tread, -vehicle.wheel_width - vehicle.tread, vehicle.wheel_width - vehicle.tread, vehicle.wheel_width - vehicle.tread, -vehicle.wheel_width - vehicle.tread]])

        rr_wheel = np.copy(fr_wheel)

        fl_wheel = np.copy(fr_wheel)
        fl_wheel[1, :] *= -1
        rl_wheel = np.copy(rr_wheel)
        rl_wheel[1, :] *= -1

        Rot1 = np.array([[math.cos(state.yaw), math.sin(state.yaw)],
                         [-math.sin(state.yaw), math.cos(state.yaw)]])
        Rot2 = np.array([[math.cos(vehicle.driver.GetSteering()), math.sin(vehicle.driver.GetSteering())],
                         [-math.sin(vehicle.driver.GetSteering()), math.cos(vehicle.driver.GetSteering())]])

        fr_wheel = (fr_wheel.T.dot(Rot2)).T
        fl_wheel = (fl_wheel.T.dot(Rot2)).T
        fr_wheel[0, :] += vehicle.wb
        fl_wheel[0, :] += vehicle.wb

        fr_wheel = (fr_wheel.T.dot(Rot1)).T
        fl_wheel = (fl_wheel.T.dot(Rot1)).T

        outline = (outline.T.dot(Rot1)).T
        rr_wheel = (rr_wheel.T.dot(Rot1)).T
        rl_wheel = (rl_wheel.T.dot(Rot1)).T

        offset = np.array(vehicle.offset)
        offset = (offset.T.dot(Rot1)).T

        outline[0, :] += offset[0] + state.x
        outline[1, :] += offset[1] + state.y
        fr_wheel[0, :] += offset[0] + state.x
        fr_wheel[1, :] += offset[1] + state.y
        rr_wheel[0, :] += offset[0] + state.x
        rr_wheel[1, :] += offset[1] + state.y
        fl_wheel[0, :] += offset[0] + state.x
        fl_wheel[1, :] += offset[1] + state.y
        rl_wheel[0, :] += offset[0] + state.x
        rl_wheel[1, :] += offset[1] + state.y


        if self.vehicles[vehicle] == 0:
            cab, = plt.plot(np.array(outline[0, :]).flatten(),np.array(outline[1, :]).flatten(), cabcolor)
            fr, = plt.plot(np.array(fr_wheel[0, :]).flatten(), np.array(fr_wheel[1, :]).flatten(), wheelcolor)
            rr, = plt.plot(np.array(rr_wheel[0, :]).flatten(), np.array(rr_wheel[1, :]).flatten(), wheelcolor)
            fl, = plt.plot(np.array(fl_wheel[0, :]).flatten(), np.array(fl_wheel[1, :]).flatten(), wheelcolor)
            rl, = plt.plot(np.array(rl_wheel[0, :]).flatten(), np.array(rl_wheel[1, :]).flatten(), wheelcolor)
            self.vehicles[vehicle] = (cab,fr,rr,fl,rl)
        else:
            (cab, fr, rr, fl, rl) = self.vehicles[vehicle]
            cab.set_ydata(np.array(outline[1, :]).flatten())
            cab.set_xdata(np.array(outline[0, :]).flatten())
            fr.set_ydata(np.array(fr_wheel[1, :]).flatten())
            fr.set_xdata(np.array(fr_wheel[0, :]).flatten())
            rr.set_ydata(np.array(rr_wheel[1, :]).flatten())
            rr.set_xdata(np.array(rr_wheel[0, :]).flatten())
            fl.set_ydata(np.array(fl_wheel[1, :]).flatten())
            fl.set_xdata(np.array(fl_wheel[0, :]).flatten())
            rl.set_ydata(np.array(rl_wheel[1, :]).flatten())
            rl.set_xdata(np.array(rl_wheel[0, :]).flatten())



    def plotObstacles(self, color="-k"):
        num = 0
        for i in self.obstacles.keys():
            o = self.obstacles[i]
            outline = np.array([[-o.length, o.length, o.length, -o.length, -o.length], [o.width / 2, o.width / 2, - o.width / 2, -o.width / 2, o.width / 2]])

            ang = calcAngle(o.p1, o.p2)

            Rot1 = np.array([[math.cos(ang), math.sin(ang)], [-math.sin(ang), math.cos(ang)]])
            outline = (outline.T.dot(Rot1)).T
            outline[0, :] += o.p1.x
            outline[1, :] += o.p1.y

            if self.obstacle_outlines[o.num] == 0:
                outline, = plt.plot(np.array(outline[0, :]).flatten(), np.array(outline[1, :]).flatten(), color)
                self.obstacle_outlines[o.num] = (outline)
            else:
                border = self.obstacle_outlines[o.num]
                border.set_ydata(np.array(outline[1, :]).flatten())
                border.set_xdata(np.array(outline[0, :]).flatten())
