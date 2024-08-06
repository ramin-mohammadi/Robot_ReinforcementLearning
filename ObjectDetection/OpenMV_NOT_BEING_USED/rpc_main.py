# Image Transfer - As The Controller Device
#
# This script is meant to talk to the "image_transfer_jpg_as_the_remote_device_for_your_computer.py" on the OpenMV Cam.
#
# This script shows off how to transfer the frame buffer to your computer as a jpeg image.

import io, rpc, serial, serial.tools.list_ports, struct, sys, cv2
import PIL.Image as Image, numpy as np

# The RPC library above is installed on your OpenMV Cam and provides multiple classes for
# allowing your OpenMV Cam to control over USB or WIFI.

# NOTE: When program runs, enter correct port (assuming camera is plugged in).

print("\nAvailable Ports:\n")
for port, desc, hwid in serial.tools.list_ports.comports():
    print("{} : {} [{}]".format(port, desc, hwid))
sys.stdout.write("\nPlease enter a port name: ")
sys.stdout.flush()
interface = rpc.rpc_usb_vcp_master(port=input())
print("")
sys.stdout.flush()

# https://gist.github.com/panzi/1ceac1cb30bb6b3450aa5227c02eedd3
# Converts the image data from PIL to cv2 format, allowing for usage of cv2 library
def pil2cv(image: Image) -> np.ndarray:
    mode = image.mode
    new_image: np.ndarray
    if mode == '1':
        new_image = np.array(image, dtype=np.uint8)
        new_image *= 255
    elif mode == 'L':
        new_image = np.array(image, dtype=np.uint8)
    elif mode == 'LA' or mode == 'La':
        new_image = np.array(image.convert('RGBA'), dtype=np.uint8)
        new_image = cv2.cvtColor(new_image, cv2.COLOR_RGBA2BGRA)
    elif mode == 'RGB':
        new_image = np.array(image, dtype=np.uint8)
        new_image = cv2.cvtColor(new_image, cv2.COLOR_RGB2BGR)
        #print('RGB')
    elif mode == 'RGBA':
        new_image = np.array(image, dtype=np.uint8)
        new_image = cv2.cvtColor(new_image, cv2.COLOR_RGBA2BGRA)
    elif mode == 'LAB':
        new_image = np.array(image, dtype=np.uint8)
        new_image = cv2.cvtColor(new_image, cv2.COLOR_LAB2BGR)
    elif mode == 'HSV':
        new_image = np.array(image, dtype=np.uint8)
        new_image = cv2.cvtColor(new_image, cv2.COLOR_HSV2BGR)
        print('HSV')
    elif mode == 'YCbCr':
        # XXX: not sure if YCbCr == YCrCb
        new_image = np.array(image, dtype=np.uint8)
        new_image = cv2.cvtColor(new_image, cv2.COLOR_YCrCb2BGR)
    elif mode == 'P' or mode == 'CMYK':
        new_image = np.array(image.convert('RGB'), dtype=np.uint8)
        new_image = cv2.cvtColor(new_image, cv2.COLOR_RGB2BGR)
    elif mode == 'PA' or mode == 'Pa':
        new_image = np.array(image.convert('RGBA'), dtype=np.uint8)
        new_image = cv2.cvtColor(new_image, cv2.COLOR_RGBA2BGRA)
    else:
        raise ValueError(f'unhandled image color mode: {mode}')

    return new_image

def get_frame_buffer_call_back(pixformat_str: str, framesize_str: str, 
                               cutthrough: bool, silent: bool) -> bytearray: 
    if not silent: print("Getting Remote Frame...")

    result = interface.call("jpeg_image_snapshot", "%s,%s" % (pixformat_str, 
                                                              framesize_str))
    if result is not None:

        size = struct.unpack("<I", result)[0]
        img = bytearray(size)

        if cutthrough:
            # Fast cutthrough data transfer with no error checking.

            # Before starting the cut through data transfer we need to sync both the master and the
            # slave device. On return both devices are in sync.
            result = interface.call("jpeg_image_read")
            if result is not None:

                # Read all the image data in one very large transfer.
                interface.get_bytes(img, 5000) # timeout

        else:
            # Slower data transfer with error checking.

            # Transfer 32 KB chunks.
            chunk_size = (1 << 15)

            if not silent: print("Reading %d bytes..." % size)
            for i in range(0, size, chunk_size):
                ok = False
                for j in range(3): # Try up to 3 times.
                    result = interface.call("jpeg_image_read", 
                                            struct.pack("<II", i, chunk_size))
                    if result is not None:
                        img[i:i+chunk_size] = result # Write the image data.
                        if not silent: print("%.2f%%" % ((i * 100) / size))
                        ok = True
                        break
                    if not silent: print("Retrying... %d/2" % (j + 1))
                if not ok:
                    if not silent: print("Error!")
                    return None

        return img

    else:
        if not silent: print("Failed to get Remote Frame!")

    return None

# Continuous loop to get data from camera
while(True):
    sys.stdout.flush()

    img = get_frame_buffer_call_back("sensor.RGB565", "sensor.VGA", 
                                     cutthrough = True, silent = True)
    # Add "Q"'s before "VGA" to lower quality (tradeoff with FPS)
    if img is not None:
        opencv_image = pil2cv(Image.open(io.BytesIO(img)))

    cv2.imshow("Open MV Cam H7 Plus", opencv_image) 
    if cv2.waitKey(1) & 0xFF == ord('q'): 
        cv2.destroyAllWindows() 
        break

quit()