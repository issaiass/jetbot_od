# Jetbot_OD or Jetbot Object Detection

<details open>
<summary> <b>Brief Review<b></summary>

This project is an implementation of YOLOv3 object detection using OpenCV, the ROS cv_bridge and Nodelets for perform simple inferences.

The YOLO implementation is based on code of the github repo of [Satya Mallick](https://github.com/spmallick/learnopencv/tree/master/ObjectDetection-YOLO).  

Below an example image of the outcome.

<p align="center">
<img src = "imgs/gazebo.PNG?raw=true" width="75%"/>
</p>

The project tree:

<p align="center">
<img src = "imgs/tree.PNG?raw=true" width="45%"/>
</p>

This applications function as follows after the *jetbot_diff_drive* is loaded and this package is loaded later.
- After you load the nodelet you have to wait a few seconds
- An image of the coke_can will pop up
- Next you have to rotate the robot to see the soccer ball using the controls
- Wait a few seconds to see the inference. 
- You can also use dynamic reconfigure to play with the inferences

</details>

<details open>
<summary> <b>Using Jetbot Object Detection Package<b></summary>

- You need to have the [jetbot_diff_drive package](https://github.com/issaiass/jetbot_diff_drive)
- Also, there are other prerequisites like:
  - cv_bridge
  - image_transport
  - nodelet
  - roscpp
  - std_msgs
  - dynamic_reconfigure
- For YOLO to inference you have to download the weights file in *jetbot_od/object_detection/yolo* folder
~~~
    wget https://pjreddie.com/media/files/yolov3.weights
~~~
- We supplied the *yolo.cfg* and *coco.names* files pre-configured in that folder, you do not need to download it.

Next:
- Create a ROS ros workspace and compile an empty package:
~~~
    cd ~
    mkdir -p catkin_ws/src
    cd catkin_ws
    catkin_make
~~~
- Open the `.bashrc` with nano:
~~~
    nano ~/.bashrc
~~~    
- Insert this line at the end of the `~/.bashrc` file for sourcing your workspace:
~~~
    source ~/catkin_ws/devel/setup.bash
~~~
- Clone this repo in the `~/catkin_ws/src` folder by typing:
~~~ 
    cd ~/catkin_ws/src
    git clone https://github.com/issaiass/jetbot_diff_drive
    rm -rf README.md
    rm -rf imgs
~~~
- Go to the root folder `~/catkin_ws` and make the folder running `catkin_make` to ensure the application compiles.
- Finally launch the application by:
~~~
    roslaunch jetbot_diff_drive jetbot_rviz.launch
    roslaunch jetbot_od yolo.launch
~~~
- You must see that `roscore` and all configurations loading succesfully.
- When everything ends, you must see gazebo and rviz loaded and the jetbot displaying a coke can in rviz and also a soccer ball model that is supplied in the jetbot_diff_drive package.

<p align="center">
<img src = "imgs/gazebo_world.PNG?raw=true" width="55%"/>
</p>

- Next, with the controller or the gazebo toolbar, rotate the jetbot until it has the soccer ball in front.

<p align="center">
<img src = "imgs/gazebo_jetbot_rotated.PNG?raw=true" width="55%"/>
</p>

- Wait until the neural network inference the ball.  This could take some time (between 2-4 seconds), depending on the speed of your computer and also the inference depends on the simulator ilumniation.

<details open>
<summary> <b>Results<b></summary>

You could see the results on this youtube video.  

<p align="center">

[<img src= "https://img.youtube.com/vi/Awma3b0hk6M/0.jpg" />](https://youtu.be/Awma3b0hk6M)
</p>

The video only shows the application running, not the explanation of the code.

</details>

<details open>
<summary> <b>Video Explanation<b></summary>

I will try my best for making an explanatory video of the application as in this youtube video.

<p align="center">

[<img src= "https://img.youtube.com/vi/Da0NUU4JmKk/0.jpg" />](https://youtu.be/Da0NUU4JmKk)

</p>

</details>

<details open>
<summary> <b>Issues<b></summary>

- None, but just to let know that the implementation is running only on the CPU, you will have to adapt the code to add GPU support.

</details>

<details open>
<summary> <b>Future Work<b></summary>

Planning to add to this project:
- OpenVINO for inferencing and accelerate the speed.

</details>

<details open>
<summary> <b>Contributiong<b></summary>

Your contributions are always welcome! Please feel free to fork and modify the content but remember to finally do a pull request.

</details>

<details open>
<summary> :iphone: <b>Having Problems?<b></summary>

<p align = "center">

[<img src="https://img.shields.io/badge/linkedin-%230077B5.svg?&style=for-the-badge&logo=linkedin&logoColor=white" />](https://www.linkedin.com/in/riawa)
[<img src="https://img.shields.io/badge/telegram-2CA5E0?style=for-the-badge&logo=telegram&logoColor=white"/>](https://t.me/issaiass)
[<img src="https://img.shields.io/badge/instagram-%23E4405F.svg?&style=for-the-badge&logo=instagram&logoColor=white">](https://www.instagram.com/daqsyspty/)
[<img src="https://img.shields.io/badge/twitter-%231DA1F2.svg?&style=for-the-badge&logo=twitter&logoColor=white" />](https://twitter.com/daqsyspty) 
[<img src ="https://img.shields.io/badge/facebook-%233b5998.svg?&style=for-the-badge&logo=facebook&logoColor=white%22">](https://www.facebook.com/daqsyspty)
[<img src="https://img.shields.io/badge/linkedin-%230077B5.svg?&style=for-the-badge&logo=linkedin&logoColor=white" />](https://www.linkedin.com/in/riawe)
[<img src="https://img.shields.io/badge/tiktok-%23000000.svg?&style=for-the-badge&logo=tiktok&logoColor=white" />](https://www.linkedin.com/in/riawe)
[<img src="https://img.shields.io/badge/whatsapp-%23075e54.svg?&style=for-the-badge&logo=whatsapp&logoColor=white" />](https://wa.me/50766168542?text=Hello%20Rangel)
[<img src="https://img.shields.io/badge/hotmail-%23ffbb00.svg?&style=for-the-badge&logo=hotmail&logoColor=white" />](mailto:issaiass@hotmail.com)
[<img src="https://img.shields.io/badge/gmail-%23D14836.svg?&style=for-the-badge&logo=gmail&logoColor=white" />](mailto:riawalles@gmail.com)

</p

</details>

<details open>
<summary> <b>License<b></summary>
<p align = "center">
<img src= "https://mirrors.creativecommons.org/presskit/buttons/88x31/svg/by-sa.svg" />
</p>
</details>