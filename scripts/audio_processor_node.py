import rospy
import redis
import librosa
import numpy as np
import pickle
import time
from std_msgs.msg import Float32MultiArray
from collections import deque

MEAN = -35.80143737792969
STD = 16.67781639099121

def audio_processor():
    # Initialize Redis connection and buffer
    r = redis.Redis(host='localhost', port=6379, db=0)
    buffer = deque(maxlen=5)

    def callback(msg):
        # Append the incoming audio chunk to the buffer
        buffer.append(np.array(msg.data, dtype=np.int16))

        # Publish raw audio chunk to Redis with a unique timestamp
        raw_audio_timestamp = time.time()
        r.set(f'raw_audio:{raw_audio_timestamp}', pickle.dumps(msg.data))
        
        # Once we have 5 seconds of audio data, compute the Mel spectrogram
        if len(buffer) == 5:
            audio_data = np.concatenate(buffer)
            audio_data = audio_data.astype(np.float32)
            mel_spec = librosa.feature.melspectrogram(y=np.array(audio_data),
                                                      sr=16000,
                                                      n_mels=64)
            mel_spec_db = librosa.power_to_db(mel_spec, ref=np.max)
            mel_spec_normalized = (mel_spec_db - MEAN) / STD
            
            # Publish Mel spectrogram to Redis
            mel_spec_timestamp = time.time()
            r.set(f'mel_spec:{mel_spec_timestamp}', pickle.dumps(mel_spec_normalized))

    rospy.init_node('audio_processor')
    rospy.Subscriber('/audio_stream', Float32MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    audio_processor()
