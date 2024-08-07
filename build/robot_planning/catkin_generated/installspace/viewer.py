import time

import mujoco as mj
import numpy as np
import math


from mujoco.glfw import glfw
import os
import sys


class Mujoco_Viewer:
    def __init__(self, model, data, simend, controller, RbtMj):
        self.model = model
        self.data = data
        self.simend = simend
        self.RbtMj = RbtMj
        self.model.opt.timestep = 0.02

        # compute mouse displacement, save
        self.lastx = 0
        self.lasty = 0
        self.button_left = False
        self.button_middle = False
        self.button_right = False


        glfw.init()
        self.window = glfw.create_window(1200, 900, "Demo", None, None)
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)

        self.cam = mj.MjvCamera()  # Abstract camera
        self.opt = mj.MjvOption()  # visualization options



        # initialize visualization data structures
        mj.mjv_defaultCamera(self.cam)
        mj.mjv_defaultOption(self.opt)
        self.scene = mj.MjvScene(self.model, maxgeom=10000)
        self.context = mj.MjrContext(self.model, mj.mjtFontScale.mjFONTSCALE_150.value)
        self.controller = controller

        self.cam.azimuth = 0
        self.cam.elevation = -40
        self.cam.distance = 4
        self.cam.lookat = np.array([0.3, 0.8, 0])


        self.print_camera_config = 0  # set to 1 to print camera config

        # install GLFW mouse and keyboard callbacks
        glfw.set_key_callback(self.window, self.keyboard)
        glfw.set_cursor_pos_callback(self.window, self.mouse_move)
        glfw.set_mouse_button_callback(self.window, self.mouse_button)
        glfw.set_scroll_callback(self.window, self.scroll)

        # initialize the controller
        # self.init_controller(self.model, self.data)
        self.init_controller()

        # set the controller
        mj.set_mjcb_control(self.controller)

    # def init_controller(self, model, data):
    #     # initialize the controller here. This function is called once, in the beginning
    #     pass

    # def controller(self, model, data):
    #     # put the controller here. This function is called inside the simulation.
    #     pass

    def keyboard(self, window, key, scancode, act, mods):
        if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
            mj.mj_resetData(self.model, self.data)
            mj.mj_forward(self.model, self.data)

    def mouse_button(self, window, button, act, mods):
        # update button state


        self.button_left = (glfw.get_mouse_button(
            window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
        self.button_middle = (glfw.get_mouse_button(
            window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
        self.button_right = (glfw.get_mouse_button(
            window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

        # update mouse position
        glfw.get_cursor_pos(window)

    def mouse_move(self, window, xpos, ypos):


        dx = xpos - self.lastx
        dy = ypos - self.lasty
        self.lastx = xpos
        self.lasty = ypos

        # no buttons down: nothing to do
        if (not self.button_left) and (not self.button_middle) and (not self.button_right):
            return

        # get current window size
        width, height = glfw.get_window_size(window)

        # get shift key state
        PRESS_LEFT_SHIFT = glfw.get_key(
            window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
        PRESS_RIGHT_SHIFT = glfw.get_key(
            window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
        mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

        # determine action based on mouse button
        if self.button_right:
            if mod_shift:
                action = mj.mjtMouse.mjMOUSE_MOVE_H
            else:
                action = mj.mjtMouse.mjMOUSE_MOVE_V
        elif self.button_left:
            if mod_shift:
                action = mj.mjtMouse.mjMOUSE_ROTATE_H
            else:
                action = mj.mjtMouse.mjMOUSE_ROTATE_V
        else:
            action = mj.mjtMouse.mjMOUSE_ZOOM

        mj.mjv_moveCamera(self.model, action, dx / height,
                          dy / height, self.scene, self.cam)

    def scroll(self, window, xoffset, yoffset):
        action = mj.mjtMouse.mjMOUSE_ZOOM
        mj.mjv_moveCamera(self.model, action, 0.0, -0.05 *
                          yoffset, self.scene, self.cam)

    def init_controller(self):
        initial_qpos = {
            'l_j1': 0.506,
            'l_j2': 1.2314,
            'l_j3': 2.1612,
            'l_j4': 2.8390,
            'l_j5': 1.58,
            'l_j6': 2.3884,
            'r_j1': -0.7297,
            'r_j2': 0.8122,
            'r_j3': 2.0985,
            'r_j4': 1.803,
            'r_j5': -1.58,
            'r_j6': 1.7431,
        }
        # 2.0985
        for name, value in initial_qpos.items():
            self.data.qpos[mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, name)] = value

        id_left_base = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, 'l_j1')
        left_index = np.arange(id_left_base, id_left_base + 6)
        id_right_base = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, 'r_j1')
        right_index = np.arange(id_right_base, id_right_base + 6)
        # self.data.ctrl[left_index], self.data.ctrl[right_index] = self.RbtMj.coriolis_gravity()
        mj.mj_forward(self.model, self.data)


    def step_viewer(self):

        time_prev = self.data.time

        # while (self.data.time - time_prev < 1.0 / 60.0):
        mj.mj_step(self.model, self.data)
            # mj.mj_forward(self.model, self.data)
            # print(self.data.time)

        # if (self.data.time >= self.simend):
        #     break

        # get framebuffer viewport
        viewport_width, viewport_height = glfw.get_framebuffer_size(
            self.window)
        viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)


        # Update scene and render
        mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
                           mj.mjtCatBit.mjCAT_ALL.value, self.scene)
        mj.mjr_render(viewport, self.scene, self.context)
        # swap OpenGL buffers (blocking call due to v-sync)
        glfw.swap_buffers(self.window)
        # process pending GUI events, call GLFW callbacks
        glfw.poll_events()

        # glfw.terminate()


    def viewer(self):
        # print_camera_config = 0  # set to 1 to print camera config
        #
        # # install GLFW mouse and keyboard callbacks
        # glfw.set_key_callback(self.window, self.keyboard)
        # glfw.set_cursor_pos_callback(self.window, self.mouse_move)
        # glfw.set_mouse_button_callback(self.window, self.mouse_button)
        # glfw.set_scroll_callback(self.window, self.scroll)
        #
        # # initialize the controller
        # # self.init_controller(self.model, self.data)
        # self.init_controller()
        # # set the controller
        # mj.set_mjcb_control(self.controller)

        while not glfw.window_should_close(self.window):
            time_prev = self.data.time

            # while (self.data.time - time_prev < 1.0 / 10.0):
            #
            #     # mj.mj_forward(self.model, self.data)
            #     # print(self.data.time)
            #     pass
            time.sleep(0.05)
            mj.mj_step(self.model, self.data)

            if (self.data.time >= self.simend):
                break

            # get framebuffer viewport
            viewport_width, viewport_height = glfw.get_framebuffer_size(
                self.window)
            viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

            # print camera configuration (help to initialize the view)
            if (self.print_camera_config == 1):
                print('cam.azimuth =', self.cam.azimuth, ';', 'cam.elevation =', self.cam.elevation, ';', 'cam.distance = ',
                      self.cam.distance)
                print('cam.lookat =np.array([', self.cam.lookat[0], ',', self.cam.lookat[1], ',', self.cam.lookat[2], '])')

            # Update scene and render
            mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
                               mj.mjtCatBit.mjCAT_ALL.value, self.scene)
            mj.mjr_render(viewport, self.scene, self.context)

            # swap OpenGL buffers (blocking call due to v-sync)
            glfw.swap_buffers(self.window)

            # process pending GUI events, call GLFW callbacks
            glfw.poll_events()

        glfw.terminate()




# if __name__ == '__main__':
#
#     xml_path = '../urdf/mjmodel.xml'  # xml file (assumes this is in the same folder as this file)
#     simend = 5 # simulation time
#     print_camera_config = 0  # set to 1 to print camera config
#     # this is useful for initializing view of the model)
#
#     # For callback functions
#     button_left = False
#     button_middle = False
#     button_right = False
#     lastx = 0
#     lasty = 0
#
#     # get the full path
#     dirname = os.path.dirname(__file__)
#     abspath = os.path.join(dirname + "/" + xml_path)
#     xml_path = abspath
#     # MuJoCo data structures
#     model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
#     data = mj.MjData(model)  # MuJoCo data
#     cam = mj.MjvCamera()  # Abstract camera
#     opt = mj.MjvOption()  # visualization options
#
#     # Init GLFW, create window, make OpenGL context current, request v-sync
#     glfw.init()
#     window = glfw.create_window(1200, 900, "Demo", None, None)
#     glfw.make_context_current(window)
#     glfw.swap_interval(1)
#
#     # initialize visualization data structures
#     mj.mjv_defaultCamera(cam)
#     mj.mjv_defaultOption(opt)
#     scene = mj.MjvScene(model, maxgeom=10000)
#     context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)
#
#     # install GLFW mouse and keyboard callbacks
#     glfw.set_key_callback(window, keyboard)
#     glfw.set_cursor_pos_callback(window, mouse_move)
#     glfw.set_mouse_button_callback(window, mouse_button)
#     glfw.set_scroll_callback(window, scroll)
#
#     # Example on how to set camera configuration
#     # cam.azimuth = 90
#     # cam.elevation = -45
#     # cam.distance = 2
#     # cam.lookat = np.array([0.0, 0.0, 0])
#
#     # 初始化数据存储
#     x_data = []
#     y_data = []
#     z_data = []
#
#     # initialize the controller
#     init_controller(model, data)
#
#     # set the controller
#     mj.set_mjcb_control(controller)
#
#     while not glfw.window_should_close(window):
#         time_prev = data.time
#
#         while (data.time - time_prev < 1.0 / 60.0):
#             mj.mj_step(model, data)
#
#         if (data.time >= simend):
#             break
#
#         # get framebuffer viewport
#         viewport_width, viewport_height = glfw.get_framebuffer_size(
#             window)
#         viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
#
#         # print camera configuration (help to initialize the view)
#         if (print_camera_config == 1):
#             print('cam.azimuth =', cam.azimuth, ';', 'cam.elevation =', cam.elevation, ';', 'cam.distance = ',
#                   cam.distance)
#             print('cam.lookat =np.array([', cam.lookat[0], ',', cam.lookat[1], ',', cam.lookat[2], '])')
#
#         # Update scene and render
#         mj.mjv_updateScene(model, data, opt, None, cam,
#                            mj.mjtCatBit.mjCAT_ALL.value, scene)
#         mj.mjr_render(viewport, scene, context)
#
#         # swap OpenGL buffers (blocking call due to v-sync)
#         glfw.swap_buffers(window)
#
#         # process pending GUI events, call GLFW callbacks
#         glfw.poll_events()
#
#     glfw.terminate()