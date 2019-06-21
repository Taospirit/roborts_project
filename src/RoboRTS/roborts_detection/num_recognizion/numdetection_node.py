import tensorflow as tf 
import numpy as np 
import rospy
import roslib
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from roborts_msgs.srv import NumDetection


class Network:
    def __init__(self):
        self.x = tf.placeholder(tf.float32, shape=[None, 784])
        y_ = tf.placeholder(tf.float32, shape=[None, 2])
        x_image = tf.reshape(self.x, [-1, 28, 28, 1])

        W_conv1 = self.weight_variable([5, 5, 1, 32])
        b_conv1 = self.bias_variable([32])
        h_conv1 = tf.nn.relu(self.conv2d(x_image, W_conv1) + b_conv1)
        h_pool1 = self.max_pool_2x2(h_conv1)

        W_conv2 = self.weight_variable([5, 5, 32, 64])
        b_conv2 = self.bias_variable([64])
        h_conv2 = tf.nn.relu(self.conv2d(h_pool1, W_conv2) + b_conv2)
        h_pool2 = self.max_pool_2x2(h_conv2)

        W_fc1 = self.weight_variable([7 * 7 * 64, 1024])
        b_fc1 = self.bias_variable([1024])
        h_pool2_flat = tf.reshape(h_pool2, [-1, 7 * 7 * 64])
        h_fc1 = tf.nn.relu(tf.matmul(h_pool2_flat, W_fc1) + b_fc1)

        self.keep_prob = tf.placeholder("float")
        h_fc1_drop = tf.nn.dropout(h_fc1, self.keep_prob)
        W_fc2 = self.weight_variable([1024, 2])
        b_fc2 = self.bias_variable([2])

        self.y_conv = tf.nn.softmax(tf.matmul(h_fc1_drop, W_fc2) + b_fc2)

    def weight_variable(self, shape):
        initial = tf.truncated_normal(shape, stddev=0.1)
        return tf.Variable(initial)

    def bias_variable(self, shape):
        initial = tf.constant(0.1, shape=shape)
        return tf.Variable(initial)

    def conv2d(self, x, W):
        return tf.nn.conv2d(x, W, strides=[1, 1, 1, 1], padding='SAME')

    def max_pool_2x2(self, x):
        return tf.nn.max_pool(x, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')


class Predictor:
    def __init__(self, ckpt_path):
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        self.output_thresh = 0.5
        self.net = Network()
        self.sess = tf.Session()
        self.sess.run(tf.global_variables_initializer())
        self.bridge = CvBridge()
        saver = tf.train.Saver()
        ckpt = tf.train.get_checkpoint_state(ckpt_path)
        if ckpt and ckpt.model_checkpoint_path:
 	    print("ckpt_ot load sucessfully!\n")
            saver.restore(self.sess, ckpt.model_checkpoint_path)
        else:
            print("ckpt file load failed\n") 

    def preprocess(self, pic):
        roi = cv2.cvtColor(pic, cv2.COLOR_BGR2GRAY)
        roi = cv2.resize(roi, (28, 28))
        equal = cv2.equalizeHist(roi)
        ret, img_b = cv2.threshold(equal, 0, 255, cv2.THRESH_OTSU)
        binary = cv2.erode(img_b, self.kernel)
        cv2.imshow('roi', binary)
        flatten_img = binary.flatten()
        input_array = np.array(flatten_img.astype(float) / 255)
        return input_array

    def predict(self, img):
        img_rgb = self.bridge.imgmsg_to_cv2(img, "bgr8")
        input = self.preprocess(img_rgb)
        output = self.net.y_conv.eval(feed_dict={self.net.x: input, self.net.keep_prob: 1.0}, session=self.sess)
        # y = self.sess.run(self.net.y,feed_dict={self.net.x:x})
        if abs(output[0][0] - output[0][1]) > self.output_thresh:
            return NumDetectionResponse(np.argmax(output[0])+1)
        else:
            return 0


if __name__ == '__main__':
    prdt = Predictor('ckpt_ot/')
    rospy.init_node('num_detection_node')
    s = rospy.Service('num_detection', NumDetection, prdt.predict)
    rospy.loginfo('start the num detection service')
    rospy.spin()

