import os
from flask import Flask, render_template, request, redirect, url_for, Response
import rospy, rosnode
import sys
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import UInt8, Bool, String
from cv_bridge import CvBridge, CvBridgeError


node_name = 'web_page'
# if '/'+node_name not in rosnode.get_node_names():
print('initing a node')
rospy.init_node(node_name, anonymous=True)


bridge=CvBridge()
IMAGE_PATH = "im.jpg"
def update_im(msg):
    try:
        global bridge
        im = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        # cv2.imshow('f',cv2.resize(im,  (0, 0), fx=1, fy=1))
        # cv2.waitKey(1)
        cv2.imwrite(IMAGE_PATH,im)

    except CvBridgeError as e:
        print(e)

def gen(im_path):
    while not rospy.is_shutdown():
        with open(im_path, 'rb') as f:
            frame = f.read()
        if frame is not None:
            yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            print('no image')
        rospy.sleep(.1)

def create_app(test_config=None):

    # Creates & configures the app
    app = Flask(__name__, instance_relative_config=True)
    app.config.from_mapping( # add DATABASE here when applicable
        SECRET_KEY='dev',
    )

    if test_config is None:
        # load the instance config, if it exists, when not testing
        app.config.from_pyfile('config.py', silent=True)
    else:
        # load the test config if passed in
        app.config.from_mapping(test_config)

    # ensure the instance folder exists
    try:
        os.makedirs(app.instance_path)
    except OSError:
        pass

    IMAGE_PATH = "im.jpg"
    bridge=CvBridge()


    @app.route('/stream-test')
    def stream():
        return render_template('stream.html')


    @app.route('/stream')
    def video_feed():
        return Response(gen(IMAGE_PATH),
                        mimetype='multipart/x-mixed-replace; boundary=frame')

    # Current page/route
    @app.route('/')
    @app.route('/current')
    def current():
        return render_template('current.html')

    # Teleop page/route
    @app.route('/teleop')
    def teleop():
        return render_template('teleop.html')

    # Control page/route
    @app.route('/control')
    def control():
        return render_template('control.html')

    # Admin page/route
    @app.route('/admin')
    def admin():
        return render_template('admin.html')

    # needed to import packages
    # os.sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__) ) ) )

    # app.register_blueprint(teleop.bp)
    # Teleop forward
    @app.route('/teleop/forward', methods=['POST'])
    def teleop_forward():
        teleop_pub.publish(1)
        print('Driving forward')
        return render_template('teleop.html')

    # Teleop backward
    @app.route('/teleop/backward', methods=['POST'])
    def teleop_backward():
        teleop_pub.publish(2)
        print('Driving backward')
        return render_template('teleop.html')

    # Teleop left
    @app.route('/teleop/left', methods=['POST'])
    def teleop_left():
        teleop_pub.publish(3)
        print('Driving left')
        return render_template('teleop.html')

    # Teleop right
    @app.route('/teleop/right', methods=['POST'])
    def teleop_right():
        teleop_pub.publish(4)
        print('Driving right')
        return render_template('teleop.html')

    # Teleop stop
    @app.route('/teleop/stop', methods=['POST'])
    def teleop_stop():
        teleop_pub.publish(0)
        print('Stopping')
        return render_template('teleop.html')

    return app

def update_state():
    pass


im_sub = rospy.Subscriber('/web/camera', CompressedImage, update_im)
state_string = rospy.Subscriber('/web/state', String, update_state)

teleop_pub = rospy.Publisher('/web/teleop', UInt8, queue_size=1)
goal_pub =  rospy.Publisher('/web/destination', String, queue_size=1)

if __name__ == '__main__':
    app = create_app()
    app.run(host='0.0.0.0',port=int(os.getenv('PORT', 5000)))
