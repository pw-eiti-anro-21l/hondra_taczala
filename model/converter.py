d = 1
th1 = 0
th2 = 0
a = 1
dh = [[0,d,0,0],
      [0,0,0,th1],
      [a,0,0,th2]]

first_link_length = d
first_link_z = first_link_length/2 + box_height
joint_upper_limit = first_link_length - 2*cylinder_radius
translator_to_second_roll = th1
second_to_tool_roll = th2
second_link_length = a
second_link_z = second_link_length/2+cylinder_radius
second_to_tool_z = second_link_length + 0.1