import tkinter as tk

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop') 

        self.publisher = self.create_publisher(Twist, '/marines_rov/cmd_vel', 10)

        self.root = tk.Tk()
        self.root.title('ROV keyboard teleop')
        self.root.geometry('320x200') 
        self.label = tk.Label(self.root, text='Control the ROV')
        self.label.pack(expand=True)   


        self.root.protocol('WM_DELETE_WINDOW', self.on_close) 

        self.pressed_keys = set()

        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)

        #window itself calls send_command every 50ms
        self.root.after(50, self.send_command) 

    def send_command(self):
        twist = Twist() 

        twist.linear.x = float('w' in self.pressed_keys) - float('s' in self.pressed_keys)   # forward/ backward
        twist.linear.y = float('a' in self.pressed_keys) - float('d' in self.pressed_keys)   # left/ right
        twist.linear.z = float('r' in self.pressed_keys) - float('f' in self.pressed_keys)   # up/ down
        twist.angular.z = float('q' in self.pressed_keys) - float('e' in self.pressed_keys)  # turn left/ right

        self.publisher.publish(twist)
        self.root.after(50, self.send_command)  # Schedule the next call  


    def on_key_press(self, event):
        key = event.keysym.lower() 

        if key == 'space':
            self.pressed_keys.clear()  # Clear all pressed keys
        else:
            self.pressed_keys.add(event.keysym)
        

    def on_key_release(self, event):
        key = event.keysym.lower() 

        self.pressed_keys.discard(key)

    def on_close(self):
        self.publisher.publish(Twist())          
        self.root.destroy()                     # close the window, ends mainloop

    def run(self):
        self.root.mainloop()
        


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()