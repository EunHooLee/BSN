def c(color):
    print(color)
    color = [x/255.0 for x in color]
    print(color)

color = [255, 1, ]
c(color)


a= (1,2)
print(a)

_dict = {}

_dict['root_1'] = {}
_dict['root_2'] = {}

_dict['root_1']['id'] = [1,2,34]
_dict['root_1']['password'] = [1,2,3,4,5]

_dict['root_2']['id'] = {}
_dict['root_2']['id']['1'] = [114]
_dict['root_2']['id']['2'] = [226]

_dict['root_2']['password'] = [2,3,4]
# _dict['root_1']['id'] = [11, 4]
# _dict['root_1']['passworld'] = [1,1,4]

# _dict['root_2']['id'] = [488]
# _dict['root_2']['passward'] = [4,8,8]

print(_dict)