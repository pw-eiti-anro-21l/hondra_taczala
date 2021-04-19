import pandas as pd


class LinksHandler:
    def __init__(self):
        self.__link_params = pd.read_csv('urdf_model/resource/links.csv')

    @property
    def link_params(self):
        return self.__link_params

    def get_geometry_type(self, type_: str, params: str) -> str:
        if type_ == "box":
            return f'box size="{params}"'
        elif type_ == "sphere":
            return f'sphere radius="{params}"'
        elif type_ == "cylinder":
            rad, len_ = params.split()
            return f'cylinder radius="{rad}" length="{len_}"'

    def generate_link_lines(self) -> list:
        link_lines = []
        for _, row in self.link_params.iterrows():
            a = f'''<link name="{row['name']}">
    <visual>
        <origin xyz="{row['origin_xyz']}" rpy="{row['origin_rpy']}"/>
        <geometry>
        <{self.get_geometry_type(row['geometry_type'],
                row['geometry_params'])}/>
        </geometry>
        <material name="green">
        <color rgba="0 1 0 1" />
        </material>
    </visual>
</link>'''
            link_lines.append(a)
        return link_lines


class JointsHandler:
    def __init__(self):
        self.__dh_params = pd.read_csv('urdf_model/resource/dh.csv')
        self.__joint_params = pd.read_csv('urdf_model/resource/joints.csv')

    @property
    def dh_params(self):
        return self.__dh_params

    @property
    def join_params(self):
        return self.__joint_params

    def generate_joints_lines(self) -> list:
        joint_lines = []
        a = f"""<joint name="joint_12" type="prismatic">
  <origin xyz="{self.dh_params.at[2, 'a']} 0 {self.dh_params.at[0, 'd']}" rpy="0 0 0"/>
  <parent link="part1"/>
  <child link="part2"/>
  <axis xyz="0 1 0" />
  <limit upper="0" lower="0.0" effort="10" velocity="10" />
</joint>"""
        joint_lines.append(a)
        for _, row in self.join_params.iterrows():
            name = row['name']
            b = f'''<joint name="{name}" type="{row['type']}">
  <origin xyz="{row['origin_xyz']}" />
  <parent link="{self.get_parent(name)}"/>
  <child link="{self.get_child(name)}"/>
  <axis xyz="{row['axis_xyz']}" />
  <limit upper="0" lower="0.0" effort="10" velocity="10" />
</joint>'''
            joint_lines.append(b)
        return joint_lines

    def get_parent(self, name: str) -> str:
        return 'link' + name[5:6]  #  'joint23'[5:6] = 2

    def get_child(self, name: str) -> str:
        return 'link' + name[6:7]  #  'joint23'[6:7] = 3


class UrdfWriter:
    def __init__(self):
        self.writer = open('urdf_model/urdf/robot.urdf.xml', 'w')
        self.maker = UrdfMaker()

    def __del__(self):
        self.writer.close()

    def write(self):
        self.writer.writelines([line + "\n" for line in self.maker.lines])


class UrdfMaker:
    def __init__(self):
        self.__link_lines = LinksHandler().generate_link_lines()
        self.__joint_lines = JointsHandler().generate_joints_lines()
        self.__lines = []
        self.writeLines()

    @property
    def lines(self):
        return self.__lines

    @property
    def link_lines(self):
        return self.__link_lines

    @property
    def joint_lines(self):
        return self.__joint_lines

    def writeLines(self):
        self.lines.append('<robot name="RobotModel">')
        self.write_links()
        self.write_joints()
        self.lines.append('</robot>')

    def write_links(self):
        for link in self.link_lines:
            self.lines.append(link)

    def write_joints(self):
        for joint in self.joint_lines:
            self.lines.append(joint)


if __name__ == '__main__':
    a = UrdfWriter()
    a.write()
