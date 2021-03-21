import curses
from geometry_msgs.msg import Twist


class TurleSpeed:

    def __init__(self):
        self.stdscr = curses.initscr()

        self.trtl_speed = Twist()
        self.kq = 'q'
        self.kf = 'w'
        self.kb = 's'
        self.kr = 'd'
        self.kl = 'a'
        self.vel_l = 1
        self.vel_a = 0.8

    def update(self, params: dict):
        for param in params.values():
            if not isinstance(param, str) and len(param) != 1:
                raise TypeError(f'{param} is not a char')

        self.kf = params['up']
        self.kb = params['down']
        self.kr = params['right']
        self.kl = params['left']

    def reset_trtl_speed(self):
        self.trtl_speed.linear.x = float(0)
        self.trtl_speed.linear.y = float(0)
        self.trtl_speed.angular.z = float(0)

    def set_trtl_speed(self, screen, publisher):
        while True:
            try:
                #self.reset_trtl_speed()

                clicked_key = screen.getch()
                if clicked_key == ord(self.kq):
                    self.reset_trtl_speed()
                    return True
                if clicked_key == ord(self.kf):
                    self.reset_trtl_speed()
                    self.trtl_speed.linear.x = float(self.vel_l)
                    publisher.publish(self.get_trtl_speed())
                if clicked_key == ord(self.kb):
                    self.reset_trtl_speed()
                    self.trtl_speed.linear.x = float(-1*self.vel_l)
                    publisher.publish(self.get_trtl_speed())
                   
                if clicked_key == ord(self.kr):
                    self.reset_trtl_speed()
                    self.trtl_speed.angular.z = float(-1*self.vel_a)
                    publisher.publish(self.get_trtl_speed())                    
                if clicked_key == ord(self.kl):
                    self.reset_trtl_speed()
                    self.trtl_speed.angular.z = float(self.vel_a)
                    publisher.publish(self.get_trtl_speed())
            except Exception as e:
                print(f"zly przycisk - {e}")
        return False

    def get_trtl_speed(self):
        return self.trtl_speed
    
