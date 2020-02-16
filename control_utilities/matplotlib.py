import numpy as np
import math
import matplotlib.pyplot as plt

class MatSim:
    def __init__(self, step_size, render_step_size=1./60):
        self.step_size = step_size

        # Time interval between two render frames
        self.render_step_size = render_step_size  # FPS = 60
        self.render_steps = int(math.ceil(self.render_step_size / self.step_size))
        self.step_number = 0
        self.track_plotted = False

    def close(self):
        plt.close()

    def plot(self, track, veh_model):
        if self.step_number % self.render_steps == 0:
            if not self.track_plotted:
                self.plot_track(track)
                self.track_plotted = True
            self.plot_car(veh_model, veh_model.GetState())
            self.plot_text(veh_model)
            plt.pause(.0000001)
        if len(plt.get_fignums()) == 0:
            return -1

        self.step_number += 1

    def plot_text(self, veh_model):
        str = 'Throttle :: {0:0.2f}\nSteering :: {1:0.2f}\nBraking :: {2:0.2f}\nSpeed :: {3:0.2f}'.format(
            veh_model.driver.GetThrottle(), veh_model.driver.GetSteering(), veh_model.driver.GetBraking(), veh_model.vehicle.GetVehicleSpeed())
        if not hasattr(self, 'annotation'):
            self.annotation = plt.annotate(str, xy=(.97, .70), xytext=(0, 10), xycoords=(
                'axes fraction', 'figure fraction'), textcoords='offset points', size=10, ha='right', va='bottom')
        else:
            self.annotation.set_text(str)
    def plot_track(self, track, trackcolor="-b"):
        track.center.plot(color='-r', show=False)
        track.left.plot(color='-k', show=False)
        track.right.plot(color='-k', show=False)

    def plot_car(self, veh, state, cabcolor="-r", truckcolor="-k"):  # pragma: no cover

        outline = np.array([[-veh.backtowheel, (veh.length - veh.backtowheel), (veh.length - veh.backtowheel), -veh.backtowheel, -veh.backtowheel],
                            [veh.width / 2, veh.width / 2, - veh.width / 2, -veh.width / 2, veh.width / 2]])

        fr_wheel = np.array([[veh.wheel_len, -veh.wheel_len, -veh.wheel_len, veh.wheel_len, veh.wheel_len],
                             [-veh.wheel_width - veh.tread, -veh.wheel_width - veh.tread, veh.wheel_width - veh.tread, veh.wheel_width - veh.tread, -veh.wheel_width - veh.tread]])

        rr_wheel = np.copy(fr_wheel)

        fl_wheel = np.copy(fr_wheel)
        fl_wheel[1, :] *= -1
        rl_wheel = np.copy(rr_wheel)
        rl_wheel[1, :] *= -1

        Rot1 = np.array([[math.cos(state.yaw), math.sin(state.yaw)],
                         [-math.sin(state.yaw), math.cos(state.yaw)]])
        Rot2 = np.array([[math.cos(veh.driver.GetSteering()), math.sin(veh.driver.GetSteering())],
                         [-math.sin(veh.driver.GetSteering()), math.cos(veh.driver.GetSteering())]])

        fr_wheel = (fr_wheel.T.dot(Rot2)).T
        fl_wheel = (fl_wheel.T.dot(Rot2)).T
        fr_wheel[0, :] += veh.wb
        fl_wheel[0, :] += veh.wb

        fr_wheel = (fr_wheel.T.dot(Rot1)).T
        fl_wheel = (fl_wheel.T.dot(Rot1)).T

        outline = (outline.T.dot(Rot1)).T
        rr_wheel = (rr_wheel.T.dot(Rot1)).T
        rl_wheel = (rl_wheel.T.dot(Rot1)).T

        outline[0, :] += state.x
        outline[1, :] += state.y
        fr_wheel[0, :] += state.x
        fr_wheel[1, :] += state.y
        rr_wheel[0, :] += state.x
        rr_wheel[1, :] += state.y
        fl_wheel[0, :] += state.x
        fl_wheel[1, :] += state.y
        rl_wheel[0, :] += state.x
        rl_wheel[1, :] += state.y


        if not hasattr(self, 'outline'):
            self.outline, = plt.plot(np.array(outline[0, :]).flatten(),
                     np.array(outline[1, :]).flatten(), truckcolor)
            self.fr, = plt.plot(np.array(fr_wheel[0, :]).flatten(),
                     np.array(fr_wheel[1, :]).flatten(), truckcolor)
            self.rr, = plt.plot(np.array(rr_wheel[0, :]).flatten(),
                     np.array(rr_wheel[1, :]).flatten(), truckcolor)
            self.fl, = plt.plot(np.array(fl_wheel[0, :]).flatten(),
                     np.array(fl_wheel[1, :]).flatten(), truckcolor)
            self.rl, = plt.plot(np.array(rl_wheel[0, :]).flatten(),
                     np.array(rl_wheel[1, :]).flatten(), truckcolor)
        else:
            self.outline.set_ydata(np.array(outline[1, :]).flatten())
            self.outline.set_xdata(np.array(outline[0, :]).flatten())
            self.fr.set_ydata(np.array(fr_wheel[1, :]).flatten())
            self.fr.set_xdata(np.array(fr_wheel[0, :]).flatten())
            self.rr.set_ydata(np.array(rr_wheel[1, :]).flatten())
            self.rr.set_xdata(np.array(rr_wheel[0, :]).flatten())
            self.fl.set_ydata(np.array(fl_wheel[1, :]).flatten())
            self.fl.set_xdata(np.array(fl_wheel[0, :]).flatten())
            self.rl.set_ydata(np.array(rl_wheel[1, :]).flatten())
            self.rl.set_xdata(np.array(rl_wheel[0, :]).flatten())
        # plt.plot(state.x, state.y, "*")
