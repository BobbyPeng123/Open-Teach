import hydra
from flask import Flask, render_template, Response
from application_manager import MonitoringApplication


# Initializing the monitor class
hydra.initialize(config_path = '../configs', version_base = '1.2')
configs = hydra.compose('server')
monitor_info = MonitoringApplication(configs)

# Initializing the flask application
app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html', num_cams = monitor_info.num_cams, reskin_num_mags = monitor_info.reskin_num_mags)

@app.route('/cam_<int:id>_feed')
def video_feed(id):
    return Response(
        monitor_info.get_cam_streamer(id).yield_frames(), 
        mimetype = 'multipart/x-mixed-replace; boundary=frame'
    )

@app.route('/reskin_feed')
def reskin_feed():
    return Response(
        monitor_info.get_reskin_streamer().yield_frames(), 
        mimetype = 'multipart/x-mixed-replace; boundary=frame'
    )

@app.route('/reset_reskin_baseline')
def update_baseline():
    monitor_info.get_reskin_streamer().update_baseline()
    return 'Baseline updated'

if __name__ == '__main__':
    app.run(threaded = True)