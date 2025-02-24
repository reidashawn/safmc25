from std_msgs.msg import Int32
class Button:
    def __init__(self, node, topic, press_callback=None, short_callback=None, long_callback=None):
        self.topic = topic
        self.node = node
        self.sub = self.node.create_subscription(Int32, topic, self.data_callback, 10)
        self.press_callback = press_callback
        self.short_callback = short_callback
        self.long_callback = long_callback
    
    def data_callback(self, ):

