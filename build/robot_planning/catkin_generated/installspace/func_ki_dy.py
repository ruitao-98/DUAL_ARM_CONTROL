# import PyKDL as kdl
import utils
import mujoco as mj
import numpy as np


class Robot:
    def __init__(self,data, model, left_name, right_name):
        self.data = data
        self.model = model
        self.left_name = left_name
        self.right_name = right_name

    def fk_left(self):
        pos = self.data.site_xpos[mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_SITE, self.left_name)]
        ori = np.array(
            self.data.site_xmat[mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_SITE, self.left_name)].reshape([3, 3]))
        return pos, ori

    def fk_right(self):
        pos = self.data.site_xpos[mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_SITE, self.right_name)]
        ori = np.array(
            self.data.site_xmat[mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_SITE, self.right_name)].reshape([3, 3]))
        return pos, ori

    # get the jacobian matrix of the site x_dot = J*q_dot
    def jacobian_left(self):
        pos = self.data.site_xpos[mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_SITE, self.left_name)]
        J_p = np.zeros((3, 6))
        J_r = np.zeros((3, 6))
        mj.mj_jac(self.model, self.data, J_p, J_r, pos, mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, 'gripper_body'))
        J_full = np.array(np.vstack([J_p, J_r]))
        return J_full

    def jacobian_right(self):
        pos = self.data.site_xpos[mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_SITE, self.right_name)]
        J_p = np.zeros((3, 6))
        J_r = np.zeros((3, 6))
        mj.mj_jac(self.model, self.data, J_p, J_r, pos, mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, 'tool_body'))
        J_full = np.array(np.vstack([J_p, J_r]))
        return J_full

    # mass_matrix M
    # def mass_matrix(self):
    #     mass_matrix = np.zeros((len(self.data.qvel), len(self.data.qvel)))
    #     mj.mj_fullM(self.model, mass_matrix, self.data.qM)
    #
    #     return mass_matrix

    # coriolis and gravity G*q_dot + G
    def coriolis_gravity(self):
        id_left_base = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, 'l_j1')
        left_index = np.arange(id_left_base, id_left_base + 6)
        id_right_base = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, 'r_j1')
        right_index = np.arange(id_right_base, id_right_base + 6)

        left_c_g = self.data.qfrc_bias[left_index]
        right_c_g = self.data.qfrc_bias[right_index]
        return left_c_g, right_c_g

