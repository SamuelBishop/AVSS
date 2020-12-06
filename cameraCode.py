
import nanocamera as nano 
import cv2


def create_camera():
# Create the Camera instance
    camera = nano.Camera(camera_type=0, device_id=1, width=640, height=480, fps=30)
    print('USB Camera ready? - ', camera.isReady())
    while camera.isReady():
        try:
            # read the camera image
            frame = camera.read()
            # display the frame
            cv2.imshow("Video Frame", frame)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break
        except KeyboardInterrupt:
            break

    # close the camera instance
    camera.release()










