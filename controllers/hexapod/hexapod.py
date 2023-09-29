"""hexapod controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import copy

import numpy as np
from scipy.spatial.transform import Rotation as R
import ikpy.chain as ikc

def get_transformation_homo(rot, trans, degrees=True):
    M = np.identity(4)
    M[:3, :3] = R.from_euler("XYZ", rot, degrees=degrees).as_matrix()
    # translation
    M[:3, 3] = trans
    return M

class Hexapod(Robot):
    def __init__(self):
        super(Hexapod, self).__init__()
        self.time_step = int(self.getBasicTimeStep())
        self.loop_counter = 0
        
        # save initial body contact points
        self.body_contact_points = {
            'MR':np.array([0.2, 0, 0, 1]),
            'FR':np.array([0.1, 0.1732, 0, 1]),
            'FL':np.array([-0.1, 0.1732, 0, 1]),
            'ML':np.array([-0.2, 0, 0, 1]),
            'HL':np.array([-0.1, -0.1732, 0, 1]),
            'HR':np.array([0.1, -0.1732, 0, 1]),
        }
        # save initial rotation
        self.leg_init_rotation = {
            'MR':0,
            'FR':np.pi/3,
            'FL':2*np.pi/3,
            'ML':np.pi,
            'HL':-2*np.pi/3,
            'HR':-np.pi/3,
        }
        self.motors = {}
        self.p_sensors = {}
        for leg in ['MR', 'FR', 'FL', 'ML', 'HL', 'HR']:
            for seg in ['COXA','FEMUR', 'TIBIA']:
                self.motors[leg + '_' + seg] = self.getDevice("M_" + leg + '_' + seg)
                self.p_sensors[leg + '_' + seg] = self.getDevice("S_" + leg + '_' + seg)
                self.p_sensors[leg + '_' + seg].enable(self.time_step)
        # self.set_pose({'MR':{'COXA':np.pi/2}})
        self.initial_pose = self.init_pose()
        self.current_pose = copy.deepcopy(self.initial_pose)
        
        # read leg urdf for ikpy computation
        self.leg_chain = ikc.Chain.from_urdf_file("leg.urdf",
                                                  active_links_mask=[False, True, True, True, False])
        
    def init_pose(self, a=0, b=-0.2, c=np.pi/2):
        pose = {}
        for leg in ['MR', 'FR', 'FL', 'ML', 'HL', 'HR']:
            self.motors[leg + '_COXA'].setPosition(a)
            self.motors[leg + '_FEMUR'].setPosition(b)
            self.motors[leg + '_TIBIA'].setPosition(c)
            pose[leg] = {'COXA':a, 'FEMUR':b, 'TIBIA':c}
        return pose
            
    def set_pose(self, pose:dict):
        '''poses is a dict of dicts: {"leg_name":{"leg_seg":angle}}
        '''
        for leg, v in pose.items():
            for seg, angle in v.items():
                self.motors[leg + '_' + seg].setPosition(angle)
                self.current_pose[leg][seg] = angle
    
    def move_leg(self, leg, hip_swing=0.6, lift_swing=0.4, step_num=20):
        alpha = self.current_pose[leg]['COXA']
        beta = self.current_pose[leg]['FEMUR']
        gamma = self.current_pose[leg]['TIBIA']
        
        alpha_s = np.linspace(-hip_swing, hip_swing, int(step_num*2)) + alpha
        
        beta_s = np.linspace(0, -lift_swing, int(step_num)) + beta
        beta_s_r = beta_s[::-1]
        beta_s = np.hstack([beta_s, beta_s_r])
        
        gamma_s = np.linspace(0, lift_swing, int(step_num)) + gamma
        gamma_s_r = gamma_s[::-1]
        gamma_s = np.hstack([gamma_s, gamma_s_r])
        
        for t in range(int(step_num*2)):
            self.set_pose({
                leg:{'COXA':alpha_s[t], 'FEMUR':beta_s[t], 'TIBIA':gamma_s[t]}
            })
            self.my_step()
    def generate_walking_sequence_fk(self, paras:dict):
        # read parameters of gait
        gait = paras['Gait']
        hip_swing = paras['HipSwing']
        lift_swing = paras['LiftSwing']
        step_num = paras['StepNum']
        
        self.walking_sequence_fk = {}
        
        if gait == 'Tripod':
            for n in self.motors.keys():
                # get leg name
                leg = n.split('_')[0]
                alpha = self.current_pose[leg]['COXA']
                beta = self.current_pose[leg]['FEMUR']
                gamma = self.current_pose[leg]['TIBIA']
                
                # leg in swing state
                # alpha_s = np.linspace(-hip_swing, hip_swing, int(step_num*2)) + alpha
        
                beta_s = np.linspace(0, -lift_swing, int(step_num)) + beta
                beta_s_r = beta_s[::-1]
                beta_s = np.hstack([beta_s, beta_s_r])
                
                gamma_s = np.linspace(0, lift_swing, int(step_num)) + gamma
                gamma_s_r = gamma_s[::-1]
                gamma_s = np.hstack([gamma_s, gamma_s_r])
                
                # leg in stance state
                beta_s_0 = beta_s[0]*np.ones(int(step_num*2))
                gamma_s_0 = gamma_s[0]*np.ones(int(step_num*2))
                
                if leg in ('FR', 'MR', 'HR'):
                    alpha_s = np.linspace(-hip_swing, hip_swing, int(step_num*2)) + alpha
                else:
                    alpha_s = np.linspace(hip_swing, -hip_swing, int(step_num*2)) + alpha
                alpha_s_r = alpha_s[::-1]
                alpha_s_a = np.hstack([alpha_s, alpha_s_r])
                alpha_s_b = np.hstack([alpha_s_r, alpha_s])
                
                if leg in ('MR', 'FL', 'HL'):
                    self.walking_sequence_fk[leg] = {
                        'COXA':alpha_s_a,
                        'FEMUR':np.hstack([beta_s, beta_s_0]),
                        'TIBIA':np.hstack([gamma_s, gamma_s_0])
                    }
                else:
                    self.walking_sequence_fk[leg] = {
                        'COXA':alpha_s_b,
                        'FEMUR':np.hstack([beta_s_0, beta_s]),
                        'TIBIA':np.hstack([gamma_s_0, gamma_s])
                    }
    def solve_body_ik(self, rot, trans):
        pose = {}
        # get homo transformation matrix
        tm = get_transformation_homo(rot, trans)
        for k, v in self.body_contact_points.items():
            p = tm.dot(v)
            offset = v - p
            
            initial_angle = [
                0,
                self.initial_pose[k]['COXA'],
                self.initial_pose[k]['FEMUR'],
                self.initial_pose[k]['TIBIA'],
                0,
            ]
            
            p_old = self.leg_chain.forward_kinematics(initial_angle)[:, 3]
            
            a_ = self.leg_init_rotation[k]
            
            target_position = [
                offset[0]*np.cos(a_) + offset[1]*np.sin(a_) + p_old[0],
                -offset[0]*np.sin(a_) + offset[1]*np.cos(a_) + p_old[1],
                offset[2] + p_old[2]
            ]
            
            ik = self.leg_chain.inverse_kinematics(target_position=target_position,
                                                   initial_position=initial_angle)
            
            pose[k] = {'COXA':ik[1], 'FEMUR':ik[2], 'TIBIA':ik[3]}
        
        return pose
    
    def generate_walking_sequence_ik(self, paras:dict):
        # read parameters of gait
        gait = paras['Gait']
        length = paras['Length']
        height = paras['Height']
        step_num = paras['StepNum']
        stance_pose = []
        swing_pose = []
        self.walking_sequence_ik = {}
        if gait == 'Tripod':
            stance_y = np.linspace(length, -length, int(step_num), endpoint=True)
            swing_y = np.linspace(-length, length, int(step_num), endpoint=True)
            swing_z = np.linspace(0, height, int(step_num), endpoint=True)
            
            for sty, swy, swz in zip(stance_y, swing_y, swing_z):
                stance_pose.append(self.solve_body_ik([0,0,0],[0, -sty, 0]))
                swing_pose.append(self.solve_body_ik([0,0,0],[0, -swy, -swz]))
            
            for n in self.motors.keys():
                leg = n.split('_')[0]
                if leg in ('MR', 'FL', 'HL'):
                    self.walking_sequence_ik[leg] = {
                        'COXA':[p[leg]['COXA'] for p in stance_pose] + [p[leg]['COXA'] for p in swing_pose],
                        'FEMUR':[p[leg]['FEMUR'] for p in stance_pose] + [p[leg]['FEMUR'] for p in swing_pose],
                        'TIBIA':[p[leg]['TIBIA'] for p in stance_pose] + [p[leg]['TIBIA'] for p in swing_pose],
                    }
                else:
                    self.walking_sequence_ik[leg] = {
                        'COXA':[p[leg]['COXA'] for p in swing_pose] + [p[leg]['COXA'] for p in stance_pose],
                        'FEMUR':[p[leg]['FEMUR'] for p in swing_pose] + [p[leg]['FEMUR'] for p in stance_pose],
                        'TIBIA':[p[leg]['TIBIA'] for p in swing_pose] + [p[leg]['TIBIA'] for p in stance_pose],
                    }
            
    def set_pose_from_walking_sequence(self, seq, step):
        pose = {}
        # get specific pose at time 'step' from 'seq'
        for k, v in seq.items():
            P = {}
            for seg in ['COXA', 'FEMUR', 'TIBIA']:
                P[seg] = v[seg][step]
            pose[k] = P
        self.set_pose(pose)
        return pose
    
    def walk(self, seq):
        seq_len = len(seq['MR']['COXA'])
        while not self.my_step():
            self.set_pose_from_walking_sequence(seq, self.loop_counter%seq_len)
            
    def my_step(self):
        if self.step(self.time_step) == -1:
            return 1
        else:
            self.loop_counter += 1
            return 0
                

if __name__ == "__main__":
    robot = Hexapod()
    robot.my_step()
    # robot.generate_walking_sequence_fk({
    #     'Gait':'Tripod',
    #     'HipSwing': 0.2,
    #     'LiftSwing': 0.6,
    #     'StepNum':80
    # })
    # robot.walk(robot.walking_sequence_fk)
    
    # robot.move_leg("MR")
    # robot.move_leg("FL")
    
    # test IK - body-wave
    # t = np.linspace(0, np.pi*2, 100)
    # rot_x = 10*np.sin(t)
    # rot_y = -10*np.cos(t)
    # for t_ in range(int(10*len(t))):
    #     pose = robot.solve_body_ik([rot_x[t_%len(t)], rot_y[t_%len(t)], 0],
    #                                [0,0,0])
    #     robot.set_pose(pose)
    #     robot.my_step()
    
    # test IK - WALKING
    robot.generate_walking_sequence_ik({
        'Gait':'Tripod',
        'Length': 0.04,
        'Height': 0.06,
        'StepNum':20
    })
    
    robot.walk(robot.walking_sequence_ik)