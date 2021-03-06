import os
from flask import Flask, render_template, request, redirect, url_for, Response
import rospy, rosnode
import sys
import cv2
import json
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import UInt8, Bool, String
from cv_bridge import CvBridge, CvBridgeError


node_name = 'web_page'
rospy.init_node(node_name)


bridge=CvBridge()
IMAGE_PATH = "im.jpg"
def update_im(msg):
    try:
        global bridge
        im = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow('f',cv2.resize(im,  (0, 0), fx=1, fy=1))
        cv2.waitKey(1)
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


# Loads from JSON a dictionary with waypoint names as keys
def getWaypointDict():
    with open('static/waypoints.json') as waypointsFile:
        waypointDict = {}
        waypoints = json.load(waypointsFile)
        for waypoint in waypoints:
            waypointDict[waypoint['name']] = waypoint
        return waypointDict

def navigateToDistination(destination_name, waypoints):
    print("Starting navigation to:")
    print(destination_name)


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

    # Load a mapping of waypoint names to full waypoint data
    waypoints = getWaypointDict()


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

    @app.route('/teleop/command', methods=['POST'])
    def sendCommand():
        command_mappings = {"stop":0, "forward":1, "backward":2, "left":3, "right":4}
        command_string = request.form["command"]
        teleop_pub.publish(command_mappings[command_string])
        print("Issued " + command_string + " command")
        return "Issued " + command_string + " command"


    # Control page/route
    @app.route('/control')
    def control():
        return render_template('control.html', waypoints=waypoints, navigateToDistination=navigateToDistination)

    @app.route('/control/go', methods=['POST'])
    def goToDestination():
        waypoint_name = request.form["waypointName"]
        goal_pub.publish(waypoint_name)
        print(waypoints[waypoint_name])
        return "Navigating to " + waypoint_name


    # Admin page/route
    @app.route('/admin')
    def admin():
        return render_template('admin.html')

    # needed to import packages
    # os.sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__) ) ) )

    return app

def update_state():
    pass


im_sub = rospy.Subscriber('/web/camera', CompressedImage, update_im)
state_string = rospy.Subscriber('/web/state', String, update_state)

teleop_pub = rospy.Publisher('/web/teleop', UInt8, queue_size=1)
goal_pub =  rospy.Publisher('/web/destination', String, queue_size=1)

if __name__ == '__main__':
    app = create_app()
    app.run(host='0.0.0.0',port=int(os.getenv('PORT', 5000)),use_reloader=False)
