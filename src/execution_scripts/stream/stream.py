import time
from pathlib import Path
from flask import Flask, render_template, Response
import cv2 as cv


app = Flask(__name__, template_folder=Path(__file__).parent.absolute())
analyser = None

delay = 1


@app.route('/')
def index():
    return render_template('index.html')


def gen():
    yieldTime = time.time()
    while True:
        elapsed = time.time() - yieldTime

        if elapsed < delay:
            time.sleep(delay - elapsed)

        if analyser is None:
            continue

        frame = analyser.iterationData.cameraImage

        if frame is None:
            continue

        ret, jpeg = cv.imencode('.jpg', frame)

        if jpeg is None:
            continue

        frame = jpeg.tobytes()

        yieldTime = time.time()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')


@app.route('/video')
def video_feed():
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')


def start():
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
