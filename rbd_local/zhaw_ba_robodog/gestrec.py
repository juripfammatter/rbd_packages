import joblib
from collections import deque
import cv2
import mediapipe as mp
import numpy as np
from multiprocessing import Process, Value
import ctypes
import sys
from time import sleep
from commandexec import COMMAND_WALK, COMMAND_WIGGLE, COMMAND_DOWN, COMMAND_SPIN, execute_command
import roslibpy
import time

from auxillary_funcs.ml import get_nearest_hand, preprocess_landmarks, write_csv, read_labels, bounding_rect
from auxillary_funcs.interprocess_comms import get_conn_listener, get_conn_client, GESTREC_PORT
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands


_GESTREC_ON = 'on'
_GESTREC_OFF = 'off'
_GSTREC_STOP = 'stop'

_COMMANDS_DICT = {
    1: COMMAND_WIGGLE,
    2: COMMAND_WALK,
    3: COMMAND_DOWN,
    4: COMMAND_SPIN
}


class Gestrec():

    def __init__(self) -> None:
        # mediapipe
        self.mediapipe_model_complexity = 0
        self.mediapipe_min_detection_confidence = 0.8
        self.mediapipe_min_tracking_confidence = 0.8

        # model
        self.model_min_confidence = 0.9
        self.model_path = '/home/juri/git/rbd_packages/rbd_local/zhaw_ba_robodog/model/logmod.pkl'
        self.model_labels_path = '/home/juri/git/rbd_packages/rbd_local/zhaw_ba_robodog/data/labels.txt'
        self._model_active = Value(ctypes.c_bool, False)

        # development mode
        self.dev_data_path = 'data/data.csv'
        self.dev_mode = False

        # openCV
        self.cv_cap_flip = True
        # self.cv_cap_source = 0  # 0 == default device
        self.cv_cap_source = 'http://192.168.123.12:8080/?action=stream'  # 0 == default device

        # processes
        self._cap_proc = None

    def __start_listener__(self):
        self._cap_proc = Process(target=self.__run_gestrec__)
        self._cap_proc.start()
        listener = get_conn_listener(GESTREC_PORT)
        while True:
            con = listener.accept()
            msg = con.recv()
            if msg == _GSTREC_STOP:
                self._cap_proc.terminate()
                sys.exit()
            elif msg == _GESTREC_OFF:
                self._model_active.value = False
            elif msg == _GESTREC_ON:
                self._model_active.value = True

    def start(self):
        proc = Process(target=self.__start_listener__)
        proc.start()
        sleep(1)
        return proc

    def __run_gestrec__(self):
        #client= roslibpy.Ros(host='localhost', port=9090)
        client= roslibpy.Ros(host='0.0.0.0', port=9090)
        client.run()
        talker= roslibpy.Topic(client, '/chatter', 'std_msgs/String')
        self._model_active.value = True

        # video capture ################################################################################################

        cap = cv2.VideoCapture(self.cv_cap_source)

        # mediapipe ####################################################################################################

        hands = mp_hands.Hands(
            model_complexity=self.mediapipe_model_complexity,
            min_detection_confidence=self.mediapipe_min_detection_confidence,
            min_tracking_confidence=self.mediapipe_min_tracking_confidence,
            max_num_hands=100,
        )

        # landmarks queue ##############################################################################################

        land_q = deque(maxlen=32)

        # ml model #####################################################################################################
        model = joblib.load(self.model_path) if not self.dev_mode else None
        labels = read_labels(self.model_labels_path) if not self.dev_mode else None

        # app ##########################################################################################################

        label = -1
        label_count = 1

        while cap.isOpened() and client.is_connected:
            # capture image
            success, image = cap.read()
            if not success:
                print("Ignoring empty camera frame.")
                continue
            if self.cv_cap_flip:
                image = cv2.flip(image, 1)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image.flags.writeable = False

            # process with mediapipe
            results = hands.process(image)

            # revert image color back
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            # capture key imput
            key = cv2.waitKey(10) & 0xFF
            if key == 27:  # esc
                cv2.destroyAllWindows()
                cap.release()
                break
            if 48 <= key <= 57:  # 0 - 9
                label = key - 48

            if results.multi_hand_landmarks:

                # process hand landmarks
                hand_landmarks = get_nearest_hand(results.multi_hand_landmarks)
                landmarks = preprocess_landmarks(hand_landmarks)
                land_q.append(landmarks)

                # process landmarks and save to CSV when in dev_mode
                if self.dev_mode and label != -1:
                    if len(land_q) == land_q.maxlen:
                        write_csv(label, list(land_q), self.dev_data_path)
                        land_q.clear()
                        print(f'done_saving {label_count}st label with id {label}')
                        label_count += 1
                        label = -1

                # predict gesture using model
                if not self.dev_mode and self._model_active.value and len(land_q) == land_q.maxlen:
                    predict_result = np.squeeze(model.predict_proba(np.array(land_q).reshape(1, -1)))
                    idx = np.argmax(predict_result)
                    gesture, confidence = labels[idx], predict_result[idx]
                    if idx != 0:
                        if confidence >= self.model_min_confidence:
                            # execute command
                            #execute_command(_COMMANDS_DICT[idx])
                            land_q.clear()
                            talker.publish(roslibpy.Message({'data': _COMMANDS_DICT[idx]}))
                            print('Sending gesture '+_COMMANDS_DICT[idx])
                            time.sleep(1)

                        # print text to image
                        cv2.putText(
                            image, f'Gesture: {gesture}', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0),
                            4, cv2.LINE_AA)
                        cv2.putText(
                            image, f'Gesture: {gesture}', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255),
                            2, cv2.LINE_AA)
                        cv2.putText(image, f'Confidence: {confidence:.3f}', (10, 65),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 4, cv2.LINE_AA)
                        cv2.putText(image, f'Confidence: {confidence:.3f}', (10, 65),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)

                # draw boundingbox
                x, y, w, h = bounding_rect(image, hand_landmarks)
                cv2.rectangle(image, (x, y), (w, h), (0, 0, 255), 1)
                cv2.rectangle(image, (x, y), (w, y-22), (0, 0, 255), -1)
                cv2.putText(image, 'Tracking', (x + 5, y - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

                # draw hands
                for hand_landmarks in results.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(
                        image,
                        hand_landmarks,
                        mp_hands.HAND_CONNECTIONS,
                        mp_drawing_styles.get_default_hand_landmarks_style(),
                        mp_drawing_styles.get_default_hand_connections_style())

            # show capture
            cv2.imshow('MediaPipe Hands', image)

        talker.unadvertise()
        client.terminate()



def __send_command__(command):
    con = get_conn_client(GESTREC_PORT)
    con.send(command)
    con.close()


def gestrec_stop():
    __send_command__(_GSTREC_STOP)


def gestrec_on():
    __send_command__(_GESTREC_ON)


def gestrec_off():
    __send_command__(_GESTREC_OFF)


# main when gestrec.py is used without app.py (GUI) ################################################################################################

if __name__ == '__main__':
    Gestrec().start()

