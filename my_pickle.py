import pickle
import numpy as np

with open("trainEnvironments.pkl","rb") as env:
    envDict = pickle.load(env)

# print(envDict.keys())
# print("================================================================================")
# print(envDict['poses'].keys())


with open("trainEnvironments_testGoals.pkl", "rb") as goal:
    goalDict = pickle.load(goal)


test_envs = envDict['poses']['trainEnv_4']
# print(test_envs)
print(test_envs)
print(envDict['poses']['trainEnv_1'])

obs = envDict['obsData']

# print(obs)
# print(envDict.keys())
# print()

collision_free_goals = goalDict['trainEnv_4']['Joints']


goal = collision_free_goals[np.random.randint(0,len(collision_free_goals))]
# print(goal.keys())
# print(goal.values())

# print(len(goalDict['trainEnv_4']['Joints']))
# print(goalDict['trainEnv_4']['Pose'][19])

# for i in range(0,10):
#     print(goalDict['trainEnv_4']['Pose'][i])

# for i_env, env_name in enumerate(test_envs):
#     # print("env iteration number : " + str(i_env))
#     # print("env_name: " + str(env_name))

#     new_pose = envDict['poses'][env_name]

#     print(new_pose)


"""
output > 

{'obsData': {'mpnet_box1': {'dim': [0.14, 0.09, 0.12], 'orientation': None, 'mesh_file': None, 'is_mesh': 0, 'z_offset': 0.02, 'name': 'mpnet_box1'}, 'mpnet_book': {'dim': [0.0017, 0.0017, 0.0017], 'orientation': None, 'mesh_file': './env/gazebo_models/mpnet_book/meshes/Dictionary.STL', 'is_mesh': 1, 'z_offset': -0.04, 'name': 'mpnet_book'}, 'mpnet_pepsi': {'dim': [0.001, 0.001, 0.001], 'orientation': [0.70710678, 0, 0, 0.70710678], 'mesh_file': './env/gazebo_models/mpnet_pepsi/meshes/pepsi.STL', 'is_mesh': 1, 'z_offset': -0.041, 'name': 'mpnet_pepsi'}, 'mpnet_coffee': {'dim': [0.035, 0.035, 0.035], 'orientation': [0.70710678, 0, 0, 0.70710678], 'mesh_file': './env/gazebo_models/mpnet_coffee/meshes/coffee.stl', 'is_mesh': 1, 'z_offset': -0.035, 'name': 'mpnet_coffee'}, 'mpnet_bottle': {'dim': [0.001, 0.001, 0.001], 'orientation': None, 'mesh_file': './env/gazebo_models/mpnet_bottle/meshes/bottle.stl', 'is_mesh': 1, 'z_offset': -0.02, 'name': 'mpnet_bottle'}}, 'poses': {'trainEnv_4': {'mpnet_box1': [0.7784902170120498, -0.7914352892021506, 0.24], 'mpnet_book': [0.8795174836553489, -0.6349345120757817, 0.24], 'mpnet_coffee': [-0.03876699968792682, -0.8447366688484587, 0.24], 'mpnet_bottle': [0.863432167029997, -0.06069701838375874, 0.24], 'mpnet_pepsi': [-0.013186364218207708, -1.0431719449499073, 0.24]}, 'trainEnv_5': {'mpnet_box1': [0.9133034961915647, -0.30261459030581506, 0.24], 'mpnet_book': [0.9771572465222467, -0.7961990545441712, 0.24], 'mpnet_coffee': [0.27030076281399146, -0.8660455977070651, 0.24], 'mpnet_bottle': [0.8674371261324864, -0.13805618413365905, 0.24], 'mpnet_pepsi': [0.9423469948667507, -0.4632579735761542, 0.24]}, 'trainEnv_6': {'mpnet_box1': [0.8308938990989286, -0.21472338497781296, 0.24], 'mpnet_book': [0.8806215506840039, -0.5173317617582148, 0.24], 'mpnet_coffee': [0.8727802956477603, -0.08566647955395335, 0.24], 'mpnet_bottle': [0.09195062289155043, -0.9354778154243377, 0.24], 'mpnet_pepsi': [0.9335330311600821, -0.3039699154632445, 0.24]}, 'trainEnv_7': {'mpnet_box1': [0.4824152150442251, -1.0118180545400801, 0.24], 'mpnet_book': [0.8659455801511943, -0.5144807453070006, 0.24], 'mpnet_coffee': [0.9120659251838943, -0.9981364911062948, 0.24], 'mpnet_bottle': [0.08686605257928054, -0.79151464148949, 0.24], 'mpnet_pepsi': [0.6187679467348852, -0.8166497548792512, 0.24]}, 'trainEnv_0': {'mpnet_box1': [0.9267559197042876, -0.6694801964553225, 0.24], 'mpnet_book': [0.47730896004019746, -0.8441402426539891, 0.24], 'mpnet_coffee': [0.8557441845493343, -0.41264344248076196, 0.24], 'mpnet_bottle': [0.1685688068667585, -0.82243300871251, 0.24], 'mpnet_pepsi': [0.868432374663792, -0.23601568146281748, 0.24]}, 'trainEnv_1': {'mpnet_box1': [0.8424127457253687, -0.3994998082888706, 0.24], 'mpnet_book': [-0.14857240731096172, -0.8290754236228585, 0.24], 'mpnet_coffee': [0.07968511426936897, -1.0039641179153733, 0.24], 'mpnet_bottle': [0.9421715381139223, -0.6279010398931718, 0.24], 'mpnet_pepsi': [0.948551979319958, -0.02474576769322212, 0.24]}, 'trainEnv_2': {'mpnet_box1': [0.8899552155706381, -0.43571564840696225, 0.24], 'mpnet_book': [0.7308004483714838, -0.8677717416281757, 0.24], 'mpnet_coffee': [0.9172368280816048, -0.61731724876537, 0.24], 'mpnet_bottle': [0.8673893545497058, -0.03499484330897207, 0.24], 'mpnet_pepsi': [0.024593257898893217, -0.863796247455991, 0.24]}, 'trainEnv_3': {'mpnet_box1': [0.9463772367940841, -0.8745314936099278, 0.24], 'mpnet_book': [0.8039601389335418, -0.4170498663180217, 0.24], 'mpnet_coffee': [0.6128299307828482, -1.0363133813149734, 0.24], 'mpnet_bottle': [0.8559951502378584, -0.16269554164929279, 0.24], 'mpnet_pepsi': [0.9043530798915556, -0.6529478702865975, 0.24]}, 'trainEnv_8': {'mpnet_box1': [0.9303105175250062, -0.2962635677756264, 0.24], 'mpnet_book': [-0.01877561724238369, -1.048699209494604, 0.24], 'mpnet_coffee': [0.8051012299816986, -0.22147167639844678, 0.24], 'mpnet_bottle': [0.9214402023066379, -0.11811917700903873, 0.24], 'mpnet_pepsi': [0.681345921365913, -0.9838569605908294, 0.24]}, 'trainEnv_9': {'mpnet_box1': [0.8044553685021398, -0.2147024188617095, 0.24], 'mpnet_book': [0.8351047144707341, -0.5252302013627487, 0.24], 'mpnet_coffee': [0.9036015453463023, -0.6400933784094065, 0.24], 'mpnet_bottle': [-0.18447993842481034, -0.9636206858137312, 0.24], 'mpnet_pepsi': [0.8504517843690438, -0.15307375737495355, 0.24]}}}
================================================================================


dict_keys(['trainEnv_4', 'trainEnv_5', 'trainEnv_6', 'trainEnv_7', 'trainEnv_0', 'trainEnv_1', 'trainEnv_2', 'trainEnv_3', 'trainEnv_8', 'trainEnv_9'])

dict_keys(['obsData', 'poses'])
================================================================================
dict_keys(['trainEnv_4', 'trainEnv_5', 'trainEnv_6', 'trainEnv_7', 'trainEnv_0', 'trainEnv_1', 'trainEnv_2', 'trainEnv_3', 'trainEnv_8', 'trainEnv_9'])
"""


