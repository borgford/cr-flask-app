import os
from flask import Flask, render_template, request, redirect, url_for

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

    from . import teleop
    app.register_blueprint(teleop.bp)

    return app