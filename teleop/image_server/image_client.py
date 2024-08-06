import cv2
import zmq
import pickle
import zlib

def start_client():
    # set ZeroMQ context and socket
    context = zmq.Context()
    socket = context.socket(zmq.PULL)
    socket.connect("tcp://192.168.123.162:5555")
    print("The client is connected and waiting to receive data...")

    while True:
        try:
            compressed_data = b''
            while True:
                chunk = socket.recv()
                compressed_data += chunk
                if len(chunk) < 60000:
                    break

            data = zlib.decompress(compressed_data)
            frame_data = pickle.loads(data)

            # decode and display the image
            frame = cv2.imdecode(frame_data, cv2.IMREAD_COLOR)

            # convert BGR to RGB
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            cv2.imshow('Video Stream', frame_rgb)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        except Exception as e:
            print(f"An error occurred while receiving data: {e}")
            break
    context.term()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    start_client()
