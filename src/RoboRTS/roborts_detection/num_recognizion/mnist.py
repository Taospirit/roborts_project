import tensorflow as tf
from tensorflow.examples.tutorials.mnist import input_data
import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
import random

train = False
sess = tf.InteractiveSession()
mnist = input_data.read_data_sets('./data/mnist', one_hot=True)


ot_image = []
ot_label = []

for i in range(mnist.train.images.shape[0]):
    if mnist.train.labels[i, 1] == 1.0:
        ot_image.append(mnist.train.images[i, :])
        ot_label.append([1.0, 0.0])
    elif mnist.train.labels[i, 2] == 1.0:
        ot_image.append(mnist.train.images[i, :])
        ot_label.append([0.0, 1.0])

labels = np.array(ot_label)
image = np.array(ot_image)


def next_batch(data, label, batch_size):
    x = []
    y = []
    for i in range(batch_size):
        a = random.randint(0, label.shape[0] - 1)
        x.append(data[a, :])
        y.append(label[a, :])

    x_batch = np.array(x)
    y_batch = np.array(y)
    return x_batch, y_batch


def weight_variable(shape):
    initial = tf.truncated_normal(shape, stddev=0.1)
    return tf.Variable(initial)


def bias_variable(shape):
    initial = tf.constant(0.1, shape=shape)
    return tf.Variable(initial)


def conv2d(x, W):
    return tf.nn.conv2d(x, W, strides=[1, 1, 1, 1], padding='SAME')


def max_pool_2x2(x):
    return tf.nn.max_pool(x, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')


x = tf.placeholder(tf.float32, shape=[None, 784])
y_ = tf.placeholder(tf.float32, shape=[None, 2])
x_image = tf.reshape(x, [-1, 28, 28, 1])

W_conv1 = weight_variable([5, 5, 1, 32])
b_conv1 = bias_variable([32])
h_conv1 = tf.nn.relu(conv2d(x_image, W_conv1) + b_conv1)
h_pool1 = max_pool_2x2(h_conv1)

W_conv2 = weight_variable([5, 5, 32, 64])
b_conv2 = bias_variable([64])
h_conv2 = tf.nn.relu(conv2d(h_pool1, W_conv2) + b_conv2)
h_pool2 = max_pool_2x2(h_conv2)

W_fc1 = weight_variable([7 * 7 * 64, 1024])
b_fc1 = bias_variable([1024])
h_pool2_flat = tf.reshape(h_pool2, [-1, 7*7*64])
h_fc1 = tf.nn.relu(tf.matmul(h_pool2_flat, W_fc1) + b_fc1)

keep_prob = tf.placeholder("float")
h_fc1_drop = tf.nn.dropout(h_fc1, keep_prob)
W_fc2 = weight_variable([1024, 2])
b_fc2 = bias_variable([2])

y_conv = tf.nn.softmax(tf.matmul(h_fc1_drop, W_fc2) + b_fc2)

cross_entropy = -tf.reduce_sum(y_*tf.log(y_conv))
train_step = tf.train.AdamOptimizer(1e-4).minimize(cross_entropy)
correct_prediction = tf.equal(tf.argmax(y_conv, 1), tf.argmax(y_, 1))
accuracy = tf.reduce_mean(tf.cast(correct_prediction, "float"))
sess.run(tf.initialize_all_variables())

saver = tf.train.Saver(max_to_keep=1)

if train:
    writer = tf.summary.FileWriter('logs/', sess.graph)
    for i in range(10000):
        #batch = mnist.train.next_batch(50)
        #img_b = image.batch(50)
        x_batch, y_batch = next_batch(image, labels, 50)
        #data, label = sess.run([x_batch, y_batch])

        if i % 50 == 0:
            train_accuracy = accuracy.eval(feed_dict={x: x_batch, y_: y_batch, keep_prob: 1.0})
            print("step %d, training accuracy %g"%(i, train_accuracy))
        train_step.run(feed_dict={x: x_batch, y_: y_batch, keep_prob: 0.5})
    saver.save(sess, 'ckpt_ot/mnist.ckpt', global_step=10000)

else:
    model_file = tf.train.latest_checkpoint('ckpt_ot/')
    saver.restore(sess, model_file)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    pic = cv2.imread("3.jpg")
    pic = cv2.resize(pic, (640, 480))
    cv2.imshow('pic', pic)
    #cv2.waitKey(0)
    #roi = pic[220:290, 249:319]
    #roi = pic[235:305, 300:370]
    roi1 = pic[185:215, 255:285]

    # print(roi.shape)
    roi1 = cv2.cvtColor(roi1, cv2.COLOR_BGR2GRAY)
    roi1 = cv2.resize(roi1, (28, 28))
    ret, img_b1 = cv2.threshold(roi1, 0, 255, cv2.THRESH_OTSU)
    binary1 = cv2.erode(img_b1, kernel)
    cv2.imshow("roi1", roi1)
    cv2.imshow('b1', binary1)
    flatten_img1 = binary1.flatten()
    xx1 = np.array(flatten_img1.astype(float) / 255)

    roi2 = pic[200:230, 355:380]
    roi2 = cv2.cvtColor(roi2, cv2.COLOR_BGR2GRAY)
    roi2 = cv2.resize(roi2, (28, 28))
    ret, img_b2 = cv2.threshold(roi2, 0, 255, cv2.THRESH_OTSU)
    binary2 = cv2.erode(img_b2, kernel)
    cv2.imshow("roi2", roi2)
    cv2.imshow('b2', binary2)
    flatten_img2 = binary2.flatten()
    xx2 = np.array(flatten_img2.astype(float) / 255)
    '''
    y = sess.run(y_, feed_dict={x: xx})
    print('predict num:', np.argmax(y[0]))
    #val_loss, val_acc = sess.run([loss, acc], feed_dict={x: mnist.test.images, y_: mnist.test.labels})
    #print('val_loss:%f, val_acc:%f' % (val_loss, val_acc))
    print("test accuracy %g" % accuracy.eval(feed_dict={x: mnist.test.images, y_: mnist.test.labels, keep_prob: 1.0}))
    
    pic = mnist.test.images[1, :]
    p = pic.reshape(28, 28)
    #print(pic)
    print(mnist.test.labels[1, :])
    '''
    #prediction = tf.argmax(y_conv, 1)
    #prediction = y_conv
    predint1 = y_conv.eval(feed_dict={x: [xx1], keep_prob: 1.0}, session=sess)
    print(predint1[0])
    predint2 = y_conv.eval(feed_dict={x: [xx2], keep_prob: 1.0}, session=sess)
    print(predint2[0])
    sess.close()
    '''
    plt.figure()
    plt.imshow(p)
    plt.show()
    '''
    cv2.waitKey(0)
    #p = pic.reshape((1, 784))
    # y = sess.run(y_, feed_dict={x: pic})
    # print('predict num:', np.argmax(y[0]))
