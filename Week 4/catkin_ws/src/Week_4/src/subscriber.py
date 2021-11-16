import rospy
from std_msgs.msg import Float32

error_in_estimate = float(0) # variable to hold current error in estimate
error_in_measurement = float(0) # variable to hold approx error in measurements
estimate = float(0) # variable to hold current estimates

def onData(data):
    global error_in_estimate, error_in_measurement, estimate # select global variables
    measurement = data.data # get measurement from ros message

    # filtering measurements
    kalman_gain = error_in_estimate / (error_in_estimate + error_in_measurement)
    estimate = estimate + kalman_gain * (measurement - estimate)
    error_in_estimate = (1 - kalman_gain) * error_in_estimate



def listener():
    rospy.init_node('subscriber')

    rospy.Subscriber("yaw", Float32, onData)

    rospy.spin()



if __name__ == '__main__':
    listener()