import rospy
import pyaudio
from collections import deque
from std_msgs.msg import Float32MultiArray
import struct

def audio_streamer():
    rospy.init_node('audio_streamer')
    pub = rospy.Publisher('/audio_stream', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    chunk_duration = 1  # seconds
    sample_rate = 16000
    chunk_size = int(sample_rate * chunk_duration)
    buffer = deque(maxlen=5)

    audio = pyaudio.PyAudio()
    stream = audio.open(format=pyaudio.paInt16,  # Use 16-bit integer format for higher amplitude values
                        channels=1,
                        rate=sample_rate,
                        input=True,
                        input_device_index=None,  # Set to specific device index if necessary
                        frames_per_buffer=chunk_size)

    while not rospy.is_shutdown():
        data = stream.read(chunk_size)
        
        # Unpack audio data as 16-bit integers and normalize if necessary
        audio_chunk = list(struct.unpack(str(chunk_size) + 'h', data))  # 'h' for 16-bit integers
        # Optional: Scale audio values for visualization purposes (only for display, not for audio processing)
        #audio_chunk = [sample / 32768.0 for sample in audio_chunk]  # Normalize to range [-1, 1]

        buffer.append(audio_chunk)
        msg = Float32MultiArray(data=audio_chunk)
        pub.publish(msg)
        rate.sleep()

    stream.stop_stream()
    stream.close()
    audio.terminate()

if __name__ == '__main__':
    try:
        audio_streamer()
    except rospy.ROSInterruptException:
        pass
