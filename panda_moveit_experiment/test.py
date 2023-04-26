import pickle
import os
import numpy as np

# import utils.moveit_utils
def dict():
    # with open("env/trainEnvironments.pkl", "rb") as env_f:
    #     envDict = pickle.load(env_f)

    with open("env/trainEnvironments_testGoals.pkl", "rb") as goal_f:
        goalDict = pickle.load(goal_f)

    # with open('env/train_environments.pkl', 'rb') as env_f_2:
    #     env_dict = pickle.load(env_f_2)

    # print(env_dict['pose_dict'])
    # print("path : ", os.getcwd())

    # print((goalDict['trainEnv_4']['Pose'][0]))
    # print('=========')
    # a = goalDict['trainEnv_4']['Joints'][0]
    # print(a)
    # print(a.values())
    # b = [1,2,3,4, a.values()]
    # print(b)
    # print(goalDict.keys())

    # with open('env/goal_example_train_env_0.pkl',"rb") as goal_ff:
    #     goal_example_dict = pickle.load(goal_ff)
    # print(goal_example_dict['configuration_space'][:2])
    with open("env/goal_example_train_env_1.pkl",'rb') as goal_ex_f:
        goal_ex_dict = pickle.load(goal_ex_f)
    print(goal_ex_dict.keys())
    print(goal_ex_dict['configuration_space'][99])
    print(goal_ex_dict['cartesian_space'][99])



def sample():
    s1 = np.random.uniform(low=0.0, high=1.0, size=1)[0]
    s2 = np.random.uniform(low=0.0, high=1.0, size=1)[0]
    s = [s1,s2]
    print(s)



if __name__ == '__main__':
    dict()
    # sample()