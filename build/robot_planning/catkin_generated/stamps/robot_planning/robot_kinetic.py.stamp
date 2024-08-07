import PyKDL as kdl
import utils


class RobotKdl():

    # the parameter of the chain is suitable for both left and right arms
    # pykdl install： conda install -c conda-forge python-orocos-kdl

    def __init__(self, model, data):
        self.data = data
        self.model = model
        self.chain = kdl.Chain()
        # the follow paramter is the D-H parameter from jaza zu5
        self.chain.addSegment(
            kdl.Segment("link1", kdl.Joint(kdl.Vector( 0, -0.00022535, 0.12015), kdl.Vector(0, 0, 1), kdl.Joint.JointType(0)),
                        kdl.Frame(kdl.Rotation(1, 0, 0, 0, 1, 0, 0, 0, 1), kdl.Vector(0, -0.00022535,  0.12015)),
                        kdl.RigidBodyInertia(15.135, kdl.Vector(-2.5186e-07 ,0.0033226, -0.001509),
                                             kdl.RotationalInertia(0.0445045, 0.0431245, 0.0306891, -1.54516e-08, -4.11722e-07, 9.02559e-05)))) #check

        self.chain.addSegment(
            kdl.Segment("link2", kdl.Joint(kdl.Vector( 0, 0, 0), kdl.Vector(0, -1,-3.67321e-06), kdl.Joint.JointType(0)),
                        kdl.Frame(kdl.Rotation(1, 0, 0,  0,-3.67321e-06, -1, 0, 1,-3.67321e-06),
                                  kdl.Vector(0,           0,           0)),
                        kdl.RigidBodyInertia(45.847, kdl.Vector(0.215, 6.4317e-09, -0.14315),
                                             kdl.RotationalInertia(1.04324, 3.29889,  2.33185, -1.22108e-07, 1.41104, 6.37592e-08))))  #check

        self.chain.addSegment(
            kdl.Segment("link3", kdl.Joint(kdl.Vector( 0.43, 0, 0), kdl.Vector(0, 0, 1), kdl.Joint.JointType(0)),
                        kdl.Frame(kdl.Rotation(1, 0, 0,  0, 1, 0,  0, 0, 1), kdl.Vector( 0.43, 0, 0)),
                        kdl.RigidBodyInertia(18.053, kdl.Vector( 0.19074, -1.5269e-05, -0.010637),
                                             kdl.RotationalInertia(0.0218336, 0.827472, 0.82207, 5.32263e-05, 0.0203478, -2.59446e-06))))  #check

        self.chain.addSegment(
            kdl.Segment("link4", kdl.Joint(kdl.Vector(0.3685,  -1.185e-05, -0.114), kdl.Vector(0, 0, 1), kdl.Joint.JointType(0)),
                        kdl.Frame(kdl.Rotation(1, 0, 0,  0, 1, 0, 0, 0, 1),
                                  kdl.Vector(0.3685,  -1.185e-05,      -0.114)),
                        kdl.RigidBodyInertia(5.5378, kdl.Vector(-1.9873e-06, -0.0059512, 0.0026789),
                                             kdl.RotationalInertia(0.00732007, 0.00520094, 0.00703933, 1.17715e-07, 5.86502e-09, -0.000309453))))   #check

        self.chain.addSegment(
            kdl.Segment("link5", kdl.Joint(kdl.Vector(0, -0.1135, 0), kdl.Vector(0, -1,-3.67321e-06), kdl.Joint.JointType(0)),
                        kdl.Frame(kdl.Rotation(1, 0, 0, 0,-3.67321e-06, -1,  0, 1,-3.67321e-06), kdl.Vector(0,  -0.1135,  0)),
                        kdl.RigidBodyInertia(6.3339, kdl.Vector(-1.9094e-06, -0.0028891, -0.0023424),
                                             kdl.RotationalInertia(0.0130076, 0.00588005, 0.0126809, -1.43601e-07, -2.24383e-08, 9.57888e-08))))   #check

        self.chain.addSegment(
            kdl.Segment("link6", kdl.Joint(kdl.Vector(0, 0.107, 0), kdl.Vector(0,  1,-3.67321e-06), kdl.Joint.JointType(0)),
                        kdl.Frame(kdl.Rotation(  1,  0, 0, 0,-3.67321e-06,  1, 0, -1,-3.67321e-06), kdl.Vector( 0,  0.107, 0)),
                        kdl.RigidBodyInertia(2.0327, kdl.Vector(0.018424, -0.018363, -0.006442),
                                             kdl.RotationalInertia(0.00239898, 0.00241124, 0.00368151, 0.000283313, 0.000232969, -0.00023473))))  #check

        self.jac_solver = kdl.ChainJntToJacSolver(self.chain)
        self.jacdot_solver = kdl.ChainJntToJacDotSolver(self.chain)
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.ik_solver = kdl.ChainIkSolverPos_LMA(self.chain, maxiter=1500)
        self.gravity = kdl.Vector(0, 0, 0)
        self.dyn_params = kdl.ChainDynParam(self.chain, self.gravity)

    def mass_matrix(self, input_q):
        input_q = utils.array2jnt_array(input_q)
        output = kdl.JntSpaceInertiaMatrix(6)
        self.dyn_params.JntToMass(input_q, output)
        return utils.matrix2array(output)

    def jacobian(self, input_q):
        input_q = utils.array2jnt_array(input_q)
        output = kdl.Jacobian(6)
        self.jac_solver.JntToJac(input_q, output)
        return utils.matrix2array(output)

    def jacobian_dot(self, input_q, input_qd):
        input_q = utils.array2jnt_array(input_q)
        input_qd = utils.array2jnt_array(input_qd)
        input_qav = kdl.JntArrayVel(input_q, input_qd)
        output = kdl.Jacobian(6)
        self.jacdot_solver.JntToJacDot(input_qav, output)
        return utils.matrix2array(output)

    def coriolis(self, input_q, input_qd):
        input_q = utils.array2jnt_array(input_q)
        input_qd = utils.array2jnt_array(input_qd)
        output = kdl.JntArray(6)
        self.dyn_params.JntToCoriolis(input_q, input_qd, output)
        return utils.jnt_array2array(output)

    def gravity_torque(self, input_q):
        input_q = utils.array2jnt_array(input_q)
        output = kdl.JntArray(6)
        self.dyn_params.JntToGravity(input_q, output)
        return utils.jnt_array2array(output)

    def fk(self, q):
        q = utils.array2jnt_array(q)
        # print('q', q)
        frame = kdl.Frame()
        self.fk_solver.JntToCart(q, frame)
        r, p = [0] * 3, [[0] * 3 for _ in range(3)]
        for i in range(3):
            r[i] = frame.__getitem__((i, 3))
        for i in range(3):
            for j in range(3):
                p[i][j] = frame.__getitem__((i, j))
        return r, p

    def ik(self, init, pos, ori):
        pos, ori, ok = utils.check_pos_ori_valid(pos, ori)
        if not ok:
            return []
        q_init = utils.array2jnt_array(init)
        p = kdl.Frame(kdl.Rotation(ori[0][0], ori[0][1], ori[0][2],
                                   ori[1][0], ori[1][1], ori[1][2],
                                   ori[2][0], ori[2][1], ori[2][2]),
                      kdl.Vector(pos[0], pos[1], pos[2]))
        q = kdl.JntArray(6)
        self.ik_solver.CartToJnt(q_init, p, q)
        return utils.jnt_array2array(q)