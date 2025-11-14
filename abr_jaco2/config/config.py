import abr_control
import numpy as np
import sympy as sp
from abr_control.arms.base_config import BaseConfig
from abr_control.utils.paths import cache_dir


class Config(BaseConfig):
    """Robot config file for the Kinova Jaco^2 V2 7DoF with force sensors

    Attributes
    ----------
    START_ANGLES : numpy.array
        The joint angles for a safe home or rest position
    _M_LINKS : sympy.diag
        inertia matrix of the links
    _M_JOINTS : sympy.diag
        inertia matrix of the joints
    L : numpy.array
        segment lengths of arm [meters]
    L_HANDCOM : numpy.array
        offset to the center of mass of the hand [meters]

    Transform Naming Convention: Tpoint1point2
    ex: Tj1l1 transforms from joint 1 to link 1

    Transforms are broken up into two matrices for simplification
    ex: Tj0l1a and Tj0l1b where the former transform accounts for
    joint rotations and the latter accounts for static rotations
    and translations
    """

    def __init__(self, **kwargs):
        
        N_LINKS = 8
        N_JOINTS = 7

        self._T = {}  # dictionary for storing calculated transforms

        super().__init__(
            N_JOINTS=N_JOINTS, N_LINKS=N_LINKS, ROBOT_NAME="j2s7s300", **kwargs
        )

        # set up saved functions folder to be in the abr_jaco repo
        self.config_folder = cache_dir + "/abr_jaco2/saved_functions_"
        self.config_folder += "_" + self.config_hash
        # make folder if it doesn't exist
        abr_control.utils.os_utils.makedirs(self.config_folder)

        # position in radians to move to before switching to torque mode
        self.START_ANGLES = np.array(
            [1.95, 3.28, 2.73, 0.66, 0, 1.63, 3.22], dtype="float32"
        )
        # https://drive.google.com/file/d/1xcImziD5G7B7iAh0CUKmyMCHcOJiheDu/view
        # create inertia matrices for each link of the Kinova Jaco^2 7DoF
        self._M_LINKS = [
            sp.Matrix(
                [  # link0 (base)
                    [0.640, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.640, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.640, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.1, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.1, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.1],
                ]
            ),
            sp.Matrix(
                [  # link1 (shoulder)
                    [0.182, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.182, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.182, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.15, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.15, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.15],
                ]
            ),
            sp.Matrix(
                [  # link2 (arm-firsthalf)
                    [0.224, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.224, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.224, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.075, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.075, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.075],
                ]
            ),
            sp.Matrix(
                [  # link3 (arm-secondhalf)
                    [0.200, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.200, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.200, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.075, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.075, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.075],
                ]
            ),

            sp.Matrix(
                [  # link4
                    [0.211, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.211, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.2113, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.125, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.125, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.125],
                ]
            ),
            sp.Matrix(
                [  # link5
                    [0.069, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.069, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.069, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.025, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.025, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.025],
                ]
            ),
            sp.Matrix(
                [  # link6
                    [0.069, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.069, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.069, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.025, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.025, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.025],
                ]
            ),
            sp.Matrix(
                [  # hand
                    [0.727, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.727, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.727, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.025, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.025, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.025],
                ]
            ),
        ]

        # create inertia matrices for each joint (motor) of the Kinova Jaco^2
        self._M_JOINTS = [  # mass of rings added
            sp.Matrix(
                [  # motor 1
                    [0.586, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.586, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.586, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.15, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.15, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.15],
                ]
            ),
            sp.Matrix(
                [  # motor 2
                    [0.572, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.572, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.572, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.15, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.15, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.15],
                ]
            ),
            sp.Matrix(
                [  # motor3
                    [0.586, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.586, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.586, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.15, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.15, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.15],
                ]
            ),
            sp.Matrix(
                [  # motor4
                    [0.586, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.586, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.586, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.15, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.15, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.15],
                ]
            ),
            sp.Matrix(
                [  # motor5
                    [0.348, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.348, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.348, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.15, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.15, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.15],
                ]
            ),
            sp.Matrix(
                [  # motor6
                    [0.348, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.348, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.348, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.15, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.15, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.15],
                ]
            ),
            sp.Matrix(
                [  # motor7
                    [0.348, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.348, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.348, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.15, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.15, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.15],
                ]
            ),
        ]

        # segment lengths associated with each transform
        # ignoring lengths < 1e-6
        # All through this controller the motors are called 0-6 for joints 1-7 
        # the joint frame origins are all aligned when the robot is in candlestick position, but the CoM frames are not
        self.L = [
            # these four should add up to .275m (shoulder height)
            [0.0, 0.0, 7.8369e-02],  # CoM of link 0 offset from base frame (z is up)
            [-3.2712e-05, -1.7324e-05, 7.8381e-02],  # joint 1 offset from CoM of link 0 in frame of link 0 (z up)
            [2.1217e-05, 4.8455e-05, -7.9515e-02],  # link CoM 1 offset from joint 1 in frame of joint 1 (z down to match TF)
            [-2.2042e-05, 1.3245e-04, -3.8863e-02],  # joint 2 offset from CoM of link 1 in frame of link 1

            # these next two should add up to .205m (upper arm length) in -y direction
            [-1.9519e-03, -1.04502e-01, 0],  # link CoM 2 offset from joint 2
            [+1.9519e-03, -1.945498e-01, 0],  # joint 3 offset from CoM of link 2

            # these next two should add up to .205m (second half of upper arm length) in -z direction
            [0, 0, -0.0881],  # link CoM 3 offset from joint 3
            [0, 0, -.1169],  # joint 4 offset from link 3

            # these next two should add up to .2073m (forearm length) +y direction
            [0, .1168, -.005],  # link CoM 4 offset from joint 4
            [0, .0905, .005],  # joint 5 offset from link 4 CoM

            # these next two should add up to .1038m in -z direction
            [0, .002, -0.047],  # link CoM 5 offset from joint 5
            [0, -.002, -.056],  # joint 6 offset from link 5

            # these next two should add up to .1038m (length to joint 7 wrist) in +y direction
            [0, 0.066, 0],  # link CoM 6 offset from joint 6
            [0, 0.0378, 0],  # joint 7 offset from link 6

            # gripper offsets (this is for the robotiq 2F-85 gripper
            [0.005, 0.0, 0.088],  # distance to center of gripper (silver divot) from joint 7 (in model without Bota sensor)
            [0.0, 0.0, 0.0389], # distance to "grip point" between two fingers (ee frame) from silver divot
        ] 

        self.L = np.array(self.L)

        # ---- Transform Matrices ----

        # Transform matrix : origin -> CoM of link 0
        # no change of axes, account for offsets
        self.Torgl0 = sp.Matrix(
            [
                [1, 0, 0, self.L[0, 0]],
                [0, 1, 0, self.L[0, 1]],
                [0, 0, 1, self.L[0, 2]],
                [0, 0, 0, 1],
            ]
        )

        # Transform matrix : CoM of link0 -> joint 1 which abr calls joint 0
        # account for change of axes (z is now down) and offsets
        self.Tl0j0 = sp.Matrix(
            [
                [-1, 0, 0, self.L[1, 0]],
                [0, 1, 0, self.L[1, 1]],
                [0, 0, -1, self.L[1, 2]],
                [0, 0, 0, 1],
            ]
        )

        # Transform matrix : joint 1 (which abr calls joint 0) -> link 1 (base rotation/pan)
        # account for rotations due to q (the joint angles list)
        self.Tj0l1a = sp.Matrix(
            [
                [sp.cos(self.q[0]), -sp.sin(self.q[0]), 0, 0],
                [sp.sin(self.q[0]), sp.cos(self.q[0]), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        # account for change of axes and offsets (no rotation, just translation)
        self.Tj0l1b = sp.Matrix(
            [
                [1, 0, 0, self.L[2, 0]],
                [0, 1, 0, self.L[2, 1]],
                [0, 0, 1, self.L[2, 2]],
                [0, 0, 0, 1],
            ]
        )
        self.Tj0l1 = self.Tj0l1a * self.Tj0l1b

        # Transform matrix : link 1 CoM-> joint 2
        # account for axes rotation and offset (rotation 90deg around x)
        self.Tl1j1 = sp.Matrix(
            [
                [-1, 0, 0, self.L[3, 0]],
                [0, 0, 1, self.L[3, 1]],
                [0, 1, 0, self.L[3, 2]],
                [0, 0, 0, 1],
            ]
        )

        # Transform matrix : joint 2 (abr j1)-> link 2 (shoulder) 
        # account for rotations due to q (around z axis)
        self.Tj1l2a = sp.Matrix(
            [
                [sp.cos(self.q[1]), -sp.sin(self.q[1]), 0, 0],
                [sp.sin(self.q[1]), sp.cos(self.q[1]), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        # account for axes rotation and offsets  (no axes rotations for links)
        self.Tj1l2b = sp.Matrix(
            [
                [1, 0, 0, self.L[4, 0]],
                [0, 1, 0, self.L[4, 1]],
                [0, 0, 1, self.L[4, 2]],
                [0, 0, 0, 1],
            ]
        )
        self.Tj1l2 = self.Tj1l2a * self.Tj1l2b

        # Transform matrix : link 2 -> joint 3 (mid-arm, abr joint 2)
        # account for axes rotation and offsets (z axis down )
        self.Tl2j2 = sp.Matrix(
            [
                [1, 0, 0, self.L[5, 0]],
                [0, 0, 1, self.L[5, 1]],
                [0, -1, 0, self.L[5, 2]],
                [0, 0, 0, 1],
            ]
        )

        # Transform matrix: joint 3 -> link 3 (mid-arm wrist to second half of upper arm)
        # account for rotations due to q (around the z axis)
        self.Tj2l3a = sp.Matrix(
            [
                [sp.cos(self.q[2]), -sp.sin(self.q[2]), 0, 0],
                [sp.sin(self.q[2]), sp.cos(self.q[2]), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        # account for axes and rotation and offsets (no axes rotation for links)
        self.Tj2l3b = sp.Matrix(
            [
                [1, 0, 0, self.L[8, 0]],
                [0, 1, 0, self.L[8, 1]],
                [0, 0, 1, self.L[8, 2]],
                [0, 0, 0, 1],
            ]
        )
        self.Tj2l3 = self.Tj2l3a * self.Tj2l3b

    ######
            
        # Transform matrix: link 3 -> joint 4 (abr j3)
        # axes change (z down) and account for offsets
        self.Tl3j3 = sp.Matrix(
            [
                [-1, 0, 0, self.L[9, 0]],
                [0, 0, -1, self.L[9, 1]],
                [0, -1, 0, self.L[9, 2]],
                [0, 0, 0, 1],
            ]
        )

            
        # Transform matrix : joint 4 -> link 4 (elbow, abr link 3)
        # account for rotations due to q (around the z axis)
        self.Tj3l4a = sp.Matrix(
            [
                [sp.cos(self.q[3]), -sp.sin(self.q[3]), 0, 0],
                [sp.sin(self.q[3]), sp.cos(self.q[3]), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        # account for axes rotation and offsets
        self.Tj3l4b = sp.Matrix(
            [
                [1, 0, 0, self.L[6, 0]],
                [0, 1, 0, self.L[6, 1]],
                [0, 0, 1, self.L[6, 2]],
                [0, 0, 0, 1],
            ]
        )
        self.Tj3l4 = self.Tj3l4a * self.Tj3l4b

        # Transform matrix : link 4 -> joint 5 (abr j4)
        # account for axes change and offsets
        self.Tl4j4 = sp.Matrix(
            [
                [-1, 0, 0, self.L[7, 0]],
                [0, 0, 1, self.L[7, 1]],
                [0, 1, 0, self.L[7, 2]],
                [0, 0, 0, 1],
            ]
        )

        # Transform matrix: joint 5 (wrist) -> link 5
        # account for rotations due to q (around the z axis)
        self.Tj4l5a = sp.Matrix(
            [
                [sp.cos(self.q[4]), -sp.sin(self.q[4]), 0, 0],
                [sp.sin(self.q[4]), sp.cos(self.q[4]), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        # account for axes and rotation and offsets
        self.Tj4l5b = sp.Matrix(
            [
                [1, 0, 0, self.L[8, 0]],
                [0, 1, 0, self.L[8, 1]],
                [0, 0, 1, self.L[8, 2]],
                [0, 0, 0, 1],
            ]
        )
        self.Tj4l5 = self.Tj4l5a * self.Tj4l5b

        # Transform matrix: link 5 -> joint 6 (abr j5)
        # account for offsets and rotation around x axis
        self.Tl5j5 = sp.Matrix(
            [
                [-1, 0, 0, self.L[9, 0]],
                [0, 0, -1, self.L[9, 1]],
                [0, -1, 0, self.L[9, 2]],
                [0, 0, 0, 1],
            ]
        )

        # Transform matrix: joint 6 (abr j5) -> link 6
        # account for rotations due to q
        self.Tj5l6a = sp.Matrix(
            [
                [sp.cos(self.q[5]), -sp.sin(self.q[5]), 0, 0],
                [sp.sin(self.q[5]), sp.cos(self.q[5]), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        # account for axes and rotation and offsets
        # no axes change, account for offsets
        self.Tj5l6b = sp.Matrix(
            [
                [1, 0, 0, self.L[10, 0]],
                [0, 1, 0, self.L[10, 1]],
                [0, 0, 1, self.L[10, 2]],
                [0, 0, 0, 1],
            ]
        )
        self.Tj5l6 = self.Tj5l6a * self.Tj5l6b

        # Transform matrix : link 6 -> joint 7
        # account for offsets and rotation around x axis
        self.Tl6j6 = sp.Matrix(
            [
                [-1, 0, 0, self.L[11, 0]],
                [0, 0, 1, self.L[11, 1]],
                [0, 1, 0, self.L[11, 2]],
                [0, 0, 0, 1],
            ]
        )

        # Transform matrix: joint 7 -> gripper COM
        # account for rotations due to q
        self.Tj6GrB = sp.Matrix(
            [
                [sp.cos(self.q[6]), -sp.sin(self.q[6]), 0, 0],
                [sp.sin(self.q[6]), sp.cos(self.q[6]), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        # account for axes changes and offsets (rotation around y axis so x is aligned with fingers)
        self.Tj6GrA = sp.Matrix(
            [
                [0, -1, 0, self.L[12, 0]],
                [-1, 0, 0, self.L[12, 1]],
                [0, 0, -1, self.L[12, 2]],
                [0, 0, 0, 1],
            ]
        )
        self.Tj6Gr = self.Tj6GrA * self.Tj6GrB

        # offsets to the end-effector (between fingers)
        self.TGrcomfingers = sp.Matrix(
            [
                [1, 0, 0, self.L[13, 0]],
                [0, 1, 0, self.L[13, 1]],
                [0, 0, 1, self.L[13, 2]],
                [0, 0, 0, 1],
            ]
        )


        # orientation part of the Jacobian (compensating for angular velocity)
        KZ = sp.Matrix([0, 0, 1])
        self.J_orientation = [
            self._calc_T("joint0")[:3, :3] * KZ,  # joint 1 orientation
            self._calc_T("joint1")[:3, :3] * KZ,  # joint 2 orientation
            self._calc_T("joint2")[:3, :3] * KZ,  # joint 3 orientation
            self._calc_T("joint3")[:3, :3] * KZ,  # joint 4 orientation
            self._calc_T("joint4")[:3, :3] * KZ,  # joint 5 orientation
            self._calc_T("joint5")[:3, :3] * KZ,  # joint 6 orientation
            self._calc_T("joint6")[:3, :3] * KZ,  # joint 7 orientation
        ]  

    def _calc_T(self, name):  # noqa C907
        """Uses Sympy to generate the transform for a joint or link

        name : string
            name of the joint, link, or end-effector
        """

        if self._T.get(name, None) is None: # this needs to use joints starting at 0 for abr_control 
            if name == "link0":
                self._T[name] = self.Torgl0
            elif name == "joint0":
                self._T[name] = self._calc_T("link0") * self.Tl0j0
            elif name == "link1":
                self._T[name] = self._calc_T("joint0") * self.Tj0l1
            elif name == "joint1":
                self._T[name] = self._calc_T("link1") * self.Tl1j1
            elif name == "link2":
                self._T[name] = self._calc_T("joint1") * self.Tj1l2
            elif name == "joint2":
                self._T[name] = self._calc_T("link2") * self.Tl2j2
            elif name == "link3":
                self._T[name] = self._calc_T("joint2") * self.Tj2l3
            elif name == "joint3":
                self._T[name] = self._calc_T("link3") * self.Tl3j3
            elif name == "link4":
                self._T[name] = self._calc_T("joint3") * self.Tj3l4
            elif name == "joint4":
                self._T[name] = self._calc_T("link4") * self.Tl4j4
            elif name == "link5":
                self._T[name] = self._calc_T("joint4") * self.Tj4l5
            elif name == "joint5":
                self._T[name] = self._calc_T("link5") * self.Tl5j5
            elif name == "link6":
                self._T[name] = self._calc_T("joint5") * self.Tj5l6
            elif name == "joint6":
                self._T[name] = self._calc_T("link6") * self.Tl6j6
            elif name=="link7":
                self._T[name] = self._calc_T("joint6") * self.Tj6Gr
            elif name == "EE":
                self._T[name] = self._calc_T("link7") * self.TGrcomfingers
            else:
                raise Exception(f"Invalid transformation name: {name}")

        return self._T[name]
