import math
import pandas as pd



def createUrdf(params):
    data = pd.read_csv('../resource/dh.csv')

    a_table = []
    d_table = []
    alpha_table = []
    theta_table = []


    #zapisanie wartości tabeli DH
    for argument in data.a:
        a_table.append(params[argument])
    for argument in data.d:
        d_table.append(params[argument])
    for argument in data.alpha:
        alpha_table.append(params[argument])
    for argument in data.theta:
        theta_table.append(params[argument])





    with open('../urdf/robot.urdf.xml','w') as file:
        file.write('<robot name="robot_lab2">\n')   
        #napisanie podstawy pod robota
        file.write("""
<link name="part1">
<visual>
<origin xyz="0 0 0.2"/>
<geometry>
<box size="2 2 0.4"/>
</geometry>
<material name="green">
<color rgba="0 1 0 1"/>
</material>
</visual>
</link>
        """)
        for i, cos in enumerate(a_table):

            a = a_table[i]
            d = d_table[i]
            alpha = alpha_table[i]
            theta = theta_table[i]

            #ustawienie parametrów
            if a!=0:
                joint_type = "revolute"
                geometry_type_ = "cylinder"
            elif d != 0:
                    joint_type = "prismatic"
                    geometry_type_ = "cylinder"
            else:
                joint_type = "revolute"
                geometry_type_ = "cylinder"
            
            len_ = math.sqrt(a**2+d**2)
            geometry_parameters = f'radius="0.05" length="{len_}"'
            if len_ !=0:
                ang = math.acos(d/len_)
            else:
                ang = 0
            #napisanie jointa
            file.write(f"""
<joint name="joint_{str(i+1)+str(i+2)}" type="{joint_type}">
<origin xyz="{a} 0 {d}" rpy="{alpha} 0 {theta}"/>
<parent link="part{i+1}"/>
<child link="part{i+2}"/>
<axis xyz="0 0 1" />
<limit upper="3.14" lower="-3.14" effort="10" velocity="10" />
</joint>
    """)
            #napisanie części głównych
            file.write(f"""
<link name="part{i+2}">
    <visual>
        <origin xyz="{a/2} 0 {-d/2}" rpy="0 {ang} 0"/>
        <geometry>
        <{geometry_type_} {geometry_parameters}/>
        </geometry>
        <material name="green">
        <color rgba="0 1 0 1" />
        </material>
    </visual>
</link>
    """
            )
        file.write(f"""
<joint name="joint_45" type="fixed">
<origin xyz="0 0 0" rpy="0 0 0"/>
<parent link="part4"/>
<child link="part5"/>
<axis xyz="0 0 1" />
<limit upper="3.14" lower="-3.14" effort="10" velocity="10" />
</joint>   
<link name="part5">
    <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
        <box size="0.2 0.2 0.2"/>
        </geometry>
        <material name="green">
        <color rgba="0 1 0 1" />
        </material>
    </visual>
</link>
""")
        file.write('</robot>\n')

