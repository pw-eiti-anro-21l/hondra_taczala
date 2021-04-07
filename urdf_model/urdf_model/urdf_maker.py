import pandas as pd

df = pd.read_csv('urdf_model/resource/dh.csv')
print(df)


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
        self.__lines = []
        self.makeLines()

    @property
    def lines(self):
        return self.__lines

    def makeLines(self):
        self.lines.append('<robot name="RobotModel">')
        self.link()
        self.lines.append('</robot>')

    def link(self):
        a = '''<link name="part1">
        <visual>
            <origin xyz="0 0 0.2" />
            <geometry>
            <box size="2 2 0.4" />
            </geometry>
            <material name="green">
            <color rgba="0 1 0 1" />
            </material>
        </visual>
        </link>'''
        self.lines.append(a)

    def joint(self):
        pass

if __name__ == '__main__':
    a = UrdfWriter()
    a.write()
