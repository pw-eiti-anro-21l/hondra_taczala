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
        <origin xyz="{row['xyz']}" />
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
        pass

    def generate_joints(self):
        f"""<joint name="joint_12" type="revolute">
  <origin xyz="a 0 d" rpy="alfa 0 0"/>
  <parent link="part1"/>
  <child link="part2"/>
  <axis xyz="0 1 0" />
  <limit upper="0" lower="0.0" effort="10" velocity="10" />
</joint>"""


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
        self.__dh_params = pd.read_csv('urdf_model/resource/dh.csv')
        self.__link_lines = LinksHandler().generate_link_lines()
        self.__lines = []
        self.writeLines()

    @property
    def dh_params(self):
        return self.__dh_params

    @property
    def lines(self):
        return self.__lines

    @property
    def link_lines(self):
        return self.__link_lines

    def writeLines(self):
        self.lines.append('<robot name="RobotModel">')
        self.write_links()
        self.lines.append('</robot>')

    def write_links(self):
        for link in self.link_lines:
            self.lines.append(link)

    def joint(self):
        pass


if __name__ == '__main__':
    a = UrdfWriter()
    a.write()
