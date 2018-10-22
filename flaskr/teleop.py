import functools
from flask import Blueprint, flash, g, redirect, render_template, request, session, url_for

# Define blueprint
bp = Blueprint('teleop', __name__, url_prefix='/teleop')

# Teleop forward
@bp.route('/forward', methods=['POST'])
def teleop_forward():
    print('Driving forward')
    return render_template('teleop.html')

# Teleop backward
@bp.route('/backward', methods=['POST'])
def teleop_backward():
    print('Driving backward')
    return render_template('teleop.html')

# Teleop left
@bp.route('/left', methods=['POST'])
def teleop_left():
    print('Driving left')
    return render_template('teleop.html')

# Teleop right
@bp.route('/right', methods=['POST'])
def teleop_right():
    print('Driving right')
    return render_template('teleop.html')

# Teleop stop
@bp.route('/stop', methods=['POST'])
def teleop_stop():
    print('Stopping')
    return render_template('teleop.html')