# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# class LidarProcessor(Node):

#     def __init__(self):
#         super().__init__('lidar_processor')

#         qos_prof = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=10
#         )

#         self.subscription = self.create_subscription(
#             LaserScan,
#             '/lidar_topic',  # Reemplaza '/lidar_topic' con el nombre correcto de tu tópico Lidar
#             self.lidar_callback,
#             # 10,  # Ajusta el valor según la cantidad de mensajes que desees acumular antes de procesarlos
#             qos_profile=qos_prof
#         )

#         self.msg_buffer = []
#         self.publisher = self.create_publisher(
#             LaserScan,
#             '/scan',  # Reemplaza '/merged_lidar_topic' con el tópico donde quieres publicar los mensajes mezclados
#             # 10,
#             qos_profile=qos_prof
#         )
#         self.msg_count = 0

#     def lidar_callback(self, msg):
#         self.msg_count += 1
#         if self.msg_count <= 6:
#             self.msg_buffer.append(msg)
#             if self.msg_count == 6:
#                 merged_msg = self.merge_lidar_msgs()
#                 self.publish_merged_msg(merged_msg)
#         else:
#             self.publisher.publish(msg)

#     def merge_lidar_msgs(self):
#         merged_ranges = [0.0] * len(self.msg_buffer[0].ranges)

#         for i in range(len(self.msg_buffer)):
#             for j in range(len(self.msg_buffer[i].ranges)):
#                 if self.msg_buffer[i].ranges[j] != float('inf'):
#                     merged_ranges[j] = self.msg_buffer[i].ranges[j]

#         merged_msg = LaserScan()
#         merged_msg.header = self.msg_buffer[0].header
#         merged_msg.ranges = merged_ranges

#         return merged_msg


# def main(args=None):
#     rclpy.init(args=args)
#     lidar_processor = LidarProcessor()
#     rclpy.spin(lidar_processor)
#     lidar_processor.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


##############################################################################################

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# import logging

# class LidarProcessor(Node):

#     def __init__(self):
#         super().__init__('lidar_processor')

#         qos_prof_1 = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=10
#         )

#         self.subscription = self.create_subscription(
#             LaserScan,
#             '/lidar_topic',  # Reemplaza '/lidar_topic' con el nombre correcto de tu tópico Lidar
#             self.lidar_callback,
#             qos_profile=qos_prof_1
#         )

#         qos_prof_2 = QoSProfile(
#             reliability=ReliabilityPolicy.RELIABLE,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=10
#         )

#         self.msg_buffer = []
#         self.publisher = self.create_publisher(
#             LaserScan,
#             '/scan',  # Reemplaza '/merged_lidar_topic' con el tópico donde quieres publicar los mensajes mezclados
#             qos_profile=qos_prof_2
#         )
#         self.msg_count = 0
#         self.logger = self.get_logger('LidarProcessor')
#         self.logger.setLevel(logging.DEBUG)

#     def lidar_callback(self, msg):
#         self.msg_count += 1
#         if self.msg_count <=6:
#             self.msg_buffer.append(msg)
#             if self.msg_count > 6:
#                 self.msg_count = 0
#                 merged_msg = self.merge_lidar_msgs()
#                 self.msg_buffer = []
#                 self.publisher.publish(merged_msg)  # Publicar el mensaje combinado directamente

#         else:
#             self.publisher.publish(msg)

#     def merge_lidar_msgs(self):
#         merged_ranges = [0.0] * len(self.msg_buffer[0].ranges)
#         cont_inf = 0
#         cont_tot = 0
#         for i in range(len(self.msg_buffer)):
#             cont_tot += 1
#             for j in range(len(self.msg_buffer[i].ranges)):
#                 if self.msg_buffer[i].ranges[j] != float('inf'):
#                     merged_ranges[j] = self.msg_buffer[i].ranges[j]
#                 else:
#                     cont_inf += 1

#         self.logger.info("Number of inifinite values from total: " + cont_inf + "/" + cont_tot)


#         merged_msg = LaserScan()
#         merged_msg.header = self.msg_buffer[0].header
#         merged_msg.ranges = merged_ranges

#         return merged_msg


# def main(args=None):
#     rclpy.init(args=args)
#     lidar_processor = LidarProcessor()
#     rclpy.spin(lidar_processor)
#     lidar_processor.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import logging


class LidarProcessor(Node):
    def __init__(self):
        super().__init__("lidar_processor")

        qos_prof_1 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
        )

        self.subscription = self.create_subscription(
            LaserScan, "/lidar_topic", self.lidar_callback, qos_profile=qos_prof_1
        )

        qos_prof_2 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
        )

        self.msg_buffer = []
        self.publisher = self.create_publisher(LaserScan, "/scan", qos_profile=qos_prof_2)
        self.msg_count = 0

        # Configuración del logger
        self.logger = self.get_logger()
        self.logger.set_level(logging.INFO)

    def lidar_callback(self, msg):
        self.msg_count += 1
        if self.msg_count <= 7:
            self.msg_buffer.append(msg)
            if self.msg_count > 7:
                self.msg_count = 0
                merged_msg = self.merge_lidar_msgs()
                self.msg_buffer = []
                self.publisher.publish(merged_msg)  # Publicar el mensaje combinado directamente

        else:
            self.publisher.publish(msg)

    def merge_lidar_msgs(self):
        merged_ranges = [0.0] * len(self.msg_buffer[0].ranges)
        cont_inf = 0
        cont_tot = 0
        for i in range(len(self.msg_buffer)):
            cont_tot += 1
            for j in range(len(self.msg_buffer[i].ranges)):
                if self.msg_buffer[i].ranges[j] != float("inf"):
                    merged_ranges[j] = self.msg_buffer[i].ranges[j]
                else:
                    cont_inf += 1

        self.logger.info(f"Number of infinite values from total: {cont_inf}/{cont_tot}")

        merged_msg = LaserScan()
        merged_msg.header = self.msg_buffer[0].header
        merged_msg.ranges = merged_ranges

        return merged_msg


def main(args=None):
    rclpy.init(args=args)
    lidar_processor = LidarProcessor()
    rclpy.spin(lidar_processor)
    lidar_processor.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
