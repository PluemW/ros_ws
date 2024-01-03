import rclpy
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import String, Int8MultiArray, Int8, Bool
from rclpy import qos, Parameter


class motor(Node):
    def __init__(self):
        super().__init__("motor_node")
        self.sent_bucket_pos = self.create_publisher(
            Twist, "gripper/bucket/pos", qos_profile=qos.qos_profile_system_default
        )
        super().__init__("motor_node")
        self.sent_bucket_sv = self.create_publisher(
            Vector3, "gripper/bucket/servo", qos_profile=qos.qos_profile_system_default
        )
        ##############################################################################
        super().__init__("motor_node")
        self.sent_state_mainros = self.create_publisher(
            Int8, "state/main_ros", qos_profile=qos.qos_profile_system_default
        )
        ##############################################################################
        super().__init__("motor_node")
        self.sent_bucket_storage = self.create_publisher(
            Int8MultiArray, "bucket/storage", qos_profile=qos.qos_profile_system_default
        )
        
        self.sub_state = self.create_subscription(
            Twist,
            "step/motor",
            self.sub_step_motor_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_state
        
        self.sub_bucket_detect = self.create_subscription(
            Int8MultiArray,
            "bucket/detect",
            self.sub_bucket_detect_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_bucket_detect
        
        self.sub_bucket_detect = self.create_subscription(
            Bool,
            "bucket/detect_center",
            self.sub_center_detect_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_bucket_detect

        self.sent_timer = self.create_timer(0.05, self.timer_callback)
        
        self.declare_parameters(
            "",
            [
                ("position.x", Parameter.Type.DOUBLE),
                ("position.y", Parameter.Type.DOUBLE),
                ("position.zl", Parameter.Type.DOUBLE),
                ("position.zr", Parameter.Type.DOUBLE),
                ("speed.x", Parameter.Type.DOUBLE),
                ("speed.y", Parameter.Type.DOUBLE),
                ("speed.z", Parameter.Type.DOUBLE),
            ],
        )
        
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_zl = 0.0
        self.position_zr = 0.0
        self.current_position_x = 0.0
        self.current_position_y = 0.0
        self.current_position_zr = 0.0
        self.current_position_zl = 0.0
        self.speed_x = 0.0
        self.speed_y = 0.0
        self.speed_z = 0.0
        self.mainros_state = -1
        self.detect_array = [0,0,0,0,0,0,0,0,0]
        self.center_detect = False
        self.collect_state = 0
        self.storage = [0,0,0]
        self.aim = -1
        self.pos_z = -1
        
    def timer_callback(self):
        msg_pos = Twist()
        msg_sv = Vector3()
        msg_statemain = Int8()
        msg_storage = Int8MultiArray()
        self.position_x = self.get_parameter("position.x").get_parameter_value().double_value
        self.position_y = self.get_parameter("position.y").get_parameter_value().double_value
        self.position_zl = self.get_parameter("position.zl").get_parameter_value().double_value
        self.position_zr = self.get_parameter("position.zr").get_parameter_value().double_value
        self.speed_x = self.get_parameter("speed.x").get_parameter_value().double_value
        self.speed_y = self.get_parameter("speed.y").get_parameter_value().double_value
        # self.speed_z = self.get_parameter("speed.z").get_parameter_value().double_value
        # self.state_bucket = self.get_parameter("main").get_parameter_value().string_value
        msg_pos.linear.x = self.position_x
        msg_pos.linear.y = self.position_y
        msg_pos.linear.z = self.position_zl
        msg_pos.angular.x = self.position_zr
        msg_sv.x = self.speed_x #svcw 180->0
        msg_sv.y = self.speed_y #svccw 0->180
        msg_storage.data = self.storage
        msg_statemain.data = self.mainros_state
        
        #--------------------checking-----------------------------#
        ########## x,y position ############
        if self.mainros_state == 1 or self.mainros_state == 3 or self.mainros_state == 5:
            for i in range(9):
                if self.detect_array[i] != 0:
                    if self.detect_array[i] not in self.storage:
                        self.position_zr = 3000.0
                        self.position_zl = 3000.0
                        if self.current_position_zr == 3000.0 and self.current_position_zl == 3000.0:
                            self.get_bucket(i)
                            break
                    else : continue
        ########## collect #################
        if self.mainros_state == 2 or self.mainros_state == 4 or self.mainros_state ==6:
            if self.position_x == self.current_position_x and self.position_y == self.current_position_y:
                if self.center_detect and self.collect_state==0:
                    self.center_detect = False
                    self.z_go(4200.0)
                    self.collect_state = 1
            if self.collect_state==1 and (self.current_position_zr == 4200.0 or self.current_position_zl == 4200.0):
                self.speed_x = 0.0
                self.speed_y = 180.0
                self.z_go(1000.0)
                self.collect_state = 2
            if self.collect_state==2 and (self.current_position_zr == 1000.0 or self.current_position_zl == 1000.0):
                match self.aim:
                    case 1: #B storage
                        self.position_x = -215000.0
                        self.position_y = 23000.0
                        self.collect_state = 3
                    case 2: #G storage
                        self.position_x = -110000.0
                        self.position_y = 23000.0
                        self.collect_state = 3
                    case 3: #R storage
                        self.position_x = -1000.0
                        self.position_y = 23000.0                    
                        self.collect_state = 3
            if self.position_x == self.current_position_x and self.position_y == self.current_position_y and self.collect_state == 3:
                self.z_go(-4800.0)
                if self.current_position_zr == -4800.0 or self.current_position_zl == -4800.0:
                    match self.aim:
                        case 1: #B data
                            self.storage[2] = self.aim
                            self.aim = -1
                            self.mainros_state += 1
                        case 2: #G data
                            self.storage[1] = self.aim
                            self.aim = -1
                            self.mainros_state += 1
                        case 3: #R data
                            self.storage[0] = self.aim
                            self.aim = -1
                            self.mainros_state += 1
                    for i in range(3):
                        if self.storage[i] == 0:
                            self.collect_state = 0
                            break
                        else: 
                            self.collect_state+=1
        #---------------------------------------------------------#
        
        self.sent_bucket_pos.publish(msg_pos)
        self.sent_bucket_sv.publish(msg_sv)
        self.sent_bucket_storage.publish(msg_storage)
        self.sent_state_mainros.publish(msg_statemain)

    def get_bucket(self, i):
        match i:
            case 0: #[0,0]
                self.position_x = 1000.0 
                self.position_y = 180000.0 
                self.pos_z = 0
            case 1: #[0,1]
                self.position_x = 1000.0 
                self.position_y = 95000.0 
                self.pos_z = 0
            case 2: #[0,2]
                self.position_x = 1000.0 
                self.position_y = 23000.0 
                self.pos_z = 0
            case 3: #[1.0]
                self.position_x = 400000.0 
                self.position_y = 180000.0 
                self.pos_z = 1
            case 4: #[1,1]
                self.position_x = 400000.0 
                self.position_y = 95000.0 
                self.pos_z = 1
            case 5: #[1,2]
                self.position_x = 400000.0 
                self.position_y = 23000.0 
                self.pos_z = 1
            case 6: #[2,0]
                self.position_x = 780000.0 
                self.position_y = 180000.0 
                self.pos_z = 2
            case 7: #[2,1]
                self.position_x = 780000.0 
                self.position_y = 95000.0 
                self.pos_z = 2
            case 8: #[2,2]
                self.position_x = 780000.0 
                self.position_y = 23000.0 
                self.pos_z = 2
        self.aim = self.detect_array[i]
        self.mainros_state +=1
    
    def z_go(self, round):
        match self.pos_z:
            case 0:
                self.position_zl = round
            case 1:
                self.position_zl = round
                self.position_zr = round
            case 2:
                self.position_zr = round    

    def sub_step_motor_callback(self, msg_in):
        self.current_position_x = msg_in.linear.x   # x axis stepper
        self.current_position_y = msg_in.linear.y   # y axis stepper
        self.current_position_zr = msg_in.angular.x  # z axis stepper
        self.current_position_zl = msg_in.angular.y  # z axis stepper
        
        if  msg_in.linear.x == 400000.0 and msg_in.linear.y == 23000.0 and msg_in.angular.x == 3000.0 and msg_in.angular.y == 3000.0 and msg_in.angular.z == 1 and self.mainros_state == -1:
            self.mainros_state = 0
        
    def sub_bucket_detect_callback(self, msg_in):
        if self.detect_array != msg_in.data and self.mainros_state == 0:
            self.detect_array = msg_in.data
            self.mainros_state = 1

    def sub_center_detect_callback(self, msg_in):
        self.center_detect = msg_in.data

def main():
    rclpy.init()
    sub = motor()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()