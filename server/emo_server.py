from flask import Flask,render_template,request
import numpy as np
from deepface import DeepFace
from deepface.commons import functions
import cv2
import base64
app = Flask('__name__')
count = 0
happy_level = 0

emo_model = DeepFace.build_model('Emotion')
emotion_labels = ['angry', 'disgust', 'fear', 'happy', 'sad', 'surprise', 'neutral']

def emo(img_path , detector_backend = 'opencv'):

        resp_obj = {}
        try:
            img, region = functions.preprocess_face(img = img_path, target_size = (48, 48), grayscale = True, enforce_detection = True, detector_backend = detector_backend, return_region = True)
        except:
            return {}
        emotion_predictions = emo_model.predict(img)[0,:]
        sum_of_predictions = emotion_predictions.sum()

        for i in range(0, len(emotion_labels)):
            emotion_label = emotion_labels[i]
            emotion_prediction = 100 * emotion_predictions[i] / sum_of_predictions
            resp_obj[emotion_label] = emotion_prediction

        resp_obj["dominant_emotion"] = emotion_labels[np.argmax(emotion_predictions)]
        return resp_obj


@app.route('/reset')
def reset():
    global count, happy_level
    count = 0
    happy_level = 0
    return f"end"

@app.route('/image', methods=['POST'])
def image():
    global count,happy_level
    data = request.json
    img = base64_bytes = data['image'].encode('utf-8')
    img = base64.b64decode(img)
    img = np.fromstring(img,dtype=np.uint8)

    img = cv2.imdecode(img, cv2.IMREAD_COLOR)
    emotion = emo(img)
    print(emotion)
    if emotion:
        happy_level += float(emotion['happy'])
        count += 1
    if count == 0:
        return 0
    return f'{happy_level/count}'

if __name__ == "__main__":
    app.run(host="0.0.0.0",port=5000)
   