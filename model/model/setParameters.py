import pandas as pd
import yaml
import os


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
    with open('urdf/model_link_params.yaml', 'r') as file:
        data = yaml.load(file, Loader=yaml.FullLoader)

    params = dict(
    boxX = float(data['box_x']),
	boxY = float(data['box_y']),
	boxZ = float(data['box_z']),
	cylinder_radius = float(data['cylinder_radius']),
	pi = float(data['pi']),
	tool_x = float(data['tool_x']),
	tool_y = float(data['tool_y']),
	tool_z = float(data['tool_z']),
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
		first_to_translator_z = float(float(data['cylinder_radius'])+float(data['box_z'])),
		half_pi = float(float(data['pi'])/2),
		a = float(data['a']),
		a_half = float(float(data['a']/2)),
		tool_origin_x = float(float(data['tool_x'])/2),
		joint_upper_limit = float(float(data['d']/2+float(data['box_z']))-2*float(data['box_z']))
	)

    with open('urdf/model.yaml', 'w') as file:
        documents = yaml.dump(params_to_write, file)

if __name__ == "__main__":
    writeYamlFile()