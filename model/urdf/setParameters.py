import pandas as pd
import yaml
import os
from ament_index_python.packages import get_package_share_directory


# def readYAMLfile():
# 	with open("model/urdf/model_links_params.yaml", 'r') as file:

# 		data = yaml.load(file, Loader=yaml.FullLoader)

# 	params=[]
# 	boxX = data['box_x']
# 	boxY = data['box_y']
# 	boxZ = data['box_z']
# 	cylinder_radious = data['cylinder_radious']
# 	translator_z = data['translator_z']
# 	pi = data['pi']
# 	tool_x = data['tool_x']
# 	tool_y = data['tool_y']
# 	tool_z = data['tool_z']
# 	link_offset = data['link_offset']

# 	params.extend((boxX, boxY,boxZ, cylinder_radious, translator_z,pi,tool_x, tool_y, tool_z,link_offset))

# 	return params

def writeYamlFile():
    with open('model_link_params.yaml', 'r') as file:
        data = yaml.load(file, Loader=yaml.FullLoader)

    params = dict(
    boxX = float(data['box_x']),
	boxY = float(data['box_y']),
	boxZ = float(data['box_z']),
	cylinder_radius = float(data['cylinder_radius']),
	translator_z = float(data['translator_z']),
	pi = float(data['pi']),
	tool_x = float(data['tool_x']),
	tool_y = float(data['tool_y']),
	tool_z = float(data['tool_z']),
	link_offset = float(data['link_offset']),
	d = float(data['d']),
	a = float(data['a'])
)
    params_to_write = dict(
		d = float(data['d']),

    	boxX = float(data['box_x']),
		boxY = float(data['box_y']),
		boxZ = float(data['box_z']),
		half_box_height = float(data['box_z'])/2,
		first_link_length = float(data['d']/2+float(data['box_z'])),
		first_link_z = float(float(data['d']/2+float(data['box_z']))/2+float(data['box_z'])),
		cylinder_radius = float(data['cylinder_radius']),
		pi = float(data['pi']),
		tool_x = float(data['tool_x']),
		tool_y = float(data['tool_y']),
		tool_z = float(data['tool_z']),
		offset0_2 = float(0.2),
		half_pi = float(float(data['pi'])/2),
		a = float(data['a'])
	)

    with open('params_to_write.yaml', 'w') as file:
        documents = yaml.dump(params_to_write, file)

if __name__ == "__main__":
    writeYamlFile()