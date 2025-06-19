"""camera_pid controller."""

from controller import Display, Keyboard, Robot, Camera
from vehicle import Car, Driver
import numpy as np
import cv2
from datetime import datetime
import os
import csv
import keras
from tensorflow.keras.models import load_model

#Getting image from camera
def get_image(camera):
    raw_image = camera.getImage()  
    image = np.frombuffer(raw_image, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)
    )
    return image

#Image processing
def greyscale_cv2(image):
    gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return gray_img

#Display image 
def display_image(display, image):
    # Image to display
    #image_rgb = np.dstack((image, image,image,))
    # Display image
    image_ref = display.imageNew(
        image.tobytes(),
        Display.RGB,
        width=image.shape[1],
        height=image.shape[0],
    )
    display.imagePaste(image_ref, 0, 0, False)

#initial angle and speed 
manual_steering = 0
steering_angle = 0
angle = 0.0
speed = 15

# set target speed
def set_speed(kmh):
    global speed            #robot.step(50)
#update steering angle
def set_steering_angle(wheel_angle):
    global angle, steering_angle
    # Check limits of steering
    if (wheel_angle - steering_angle) > 0.1:
        wheel_angle = steering_angle + 0.1
    if (wheel_angle - steering_angle) < -0.1:
        wheel_angle = steering_angle - 0.1
    steering_angle = wheel_angle
  
    # limit range of the steering angle
    if wheel_angle > 0.5:
        wheel_angle = 0.5
    elif wheel_angle < -0.5:
        wheel_angle = -0.5
    # update steering angle
    angle = wheel_angle

#validate increment of steering angle
def change_steer_angle(inc):
    global manual_steering
    # Apply increment
    new_manual_steering = manual_steering + inc
    # Validate interval 
    if new_manual_steering <= 25.0 and new_manual_steering >= -25.0: 
        manual_steering = new_manual_steering
        set_steering_angle(manual_steering * 0.02)
    # Debugging
    if manual_steering == 0:
        print("going straight")
    else:
        turn = "left" if steering_angle < 0 else "right"
        print("turning {} rad {}".format(str(steering_angle),turn))

# main
def main():
    # Create the Robot instance.
    robot = Car()
    driver = Driver()

    # Get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())

    # Create camera instance
    camera = robot.getDevice("camera")
    camera.enable(timestep)  # timestep

    # processing display
    display_img = Display("display")

    #create keyboard instance
    keyboard=Keyboard()
    keyboard.enable(timestep)

    keras.config.enable_unsafe_deserialization()
    model = load_model("C:/Users/Josue/OneDrive/MNA/Navegacion Autonoma/FinalJosue/final.keras")
 
    prediction = 0.0    
    while robot.step() != -1:
        # Get image from camera
        image = get_image(camera)
        image = image[95:135,:,:]
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2RGB)
        #image = cv2.GaussianBlur(image, (3, 3), 0)
        image = cv2.resize(image, (200, 66))
        display_image(display_img, image)
        image = image / 255
        input_tensor = np.expand_dims(image, axis=0)
        prediction = model.predict(input_tensor, verbose=0)[0]
        prediction = float(prediction)
        print(f"prediction -> {prediction}")

        # Read keyboard
        key=keyboard.getKey()
        if key == keyboard.UP: #up
            set_speed(speed + 5.0)
            print("up")
        elif key == keyboard.DOWN: #down
            set_speed(speed - 5.0)
            print("down")
        elif key == keyboard.RIGHT: #right
            change_steer_angle(+1)
            print("right")
        elif key == keyboard.LEFT: #left
            change_steer_angle(-1)
            print("left")
        elif key == ord('A'):
            #filename with timestamp and saved in current directory
            current_datetime = str(datetime.now().strftime("%Y-%m-%d %H-%M-%S"))
            file_name = current_datetime + ".png"
            print("Image taken")
            #mask_image.saveImage("C:/Users/Josue/OneDrive/MNA/Navegacion Autonoma/Final/data/" + file_name, 1)
            mask_image = cv2.cvtColor(mask_image, cv2.COLOR_RGB2BGR)
            cv2.imwrite("C:/Users/Josue/OneDrive/MNA/Navegacion Autonoma/Final/data/" + file_name, mask_image)
        #update angle and speed
        driver.setSteeringAngle(prediction)
        driver.setCruisingSpeed(speed)


if __name__ == "__main__":
    main()