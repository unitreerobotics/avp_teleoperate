
# 📺 Video Demo
<img src="./img/1.webp" autoplay loop="loop" style="width: 49%" controls></img><img src="./img/2.webp" autoplay loop="loop" style="width: 49%" controls></img>

# 0. 📖 Introduction
This repository implements teleoperation of the **Unitree humanoid robot** using **Apple Vision Pro**.

Here are the robots that will be supported,

<table>
  <tr>
    <th style="text-align: center;"> &#129302; Robot </th>
    <th style="text-align: center;"> &#9898; Status </th>
  </tr>
  <tr>
    <td style="text-align: center;"> G1(29DoF) + Dex3-1 </td>
    <td style="text-align: center;"> &#9989; Completed </td>
  </tr>
  <tr>
    <td style="text-align: center;"> G1(23DoF) </td>
    <td style="text-align: center;"> &#9201; In Progress </td>
  </tr>
  <tr>
    <td style="text-align: center;"> H1(Arm 4DoF) </td>
    <td style="text-align: center;"> &#9201; In Progress </td>
  </tr>
  <tr>
    <td style="text-align: center;"> H1_2(Arm 7DoF) + Inspire </td>
    <td style="text-align: center;"> &#9201; In Progress </td>
  </tr>
</table>



# 1. 📦 Prerequisites

We tested our code on Ubuntu 20.04 and Ubuntu 22.04, other operating systems may be configured differently.  

For more information, you can refer to [Official Documentation ](https://support.unitree.com/home/zh/Teleoperation) and [OpenTeleVision](https://github.com/OpenTeleVision/TeleVision).



## 1.1 🦾  inverse kinematics 

```bash
conda create -n tv python=3.8
conda activate tv
# If you use `pip install`, Make sure pinocchio version is 3.1.0
conda install pinocchio -c conda-forge
pip install meshcat
pip install casadi
```

## 1.2 🕹️ unitree_dds_wrapper

```bash
# Install the Python version of the unitree_dds_wrapper.
git clone https://github.com/unitreerobotics/unitree_dds_wrapper.git
cd unitree_dds_wrapper/python
pip install -e .
```

>  p.s. This is a temporary version, and it will be replaced with [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) in the future.



# 2. 🛠️ TeleVision and Apple Vision Pro configuration

## 2.1 📥 basic

```bash
cd ~
git clone https://github.com/unitreerobotics/avp_teleoperate.git 
cd ~/avp_teleoperate
pip install -r requirements.txt
cd act/detr && pip install -e .
```

## 2.2 🔌 Local streaming

**Apple** does not allow WebXR on non-https connections. To test the application locally, we need to create a self-signed certificate and install it on the client. You need a ubuntu machine and a router. Connect the VisionPro and the ubuntu machine to the same router.

1. install mkcert: https://github.com/FiloSottile/mkcert
2. check local ip address:

```bash
ifconfig | grep inet
```

Suppose the local ip address of the **Host machine** is `192.168.123.2`

> p.s. you can use `ifconfig` command to check your **Host machine** ip address.

3. create certificate:

```bash
mkcert -install && mkcert -cert-file cert.pem -key-file key.pem 192.168.123.2 localhost 127.0.0.1
```

ps. place the generated cert.pem and key.pem files in `teleop`.

```bash
cp cert.pem key.pem ~/avp_teleoperate/teleop/
```

4. open firewall on server:

```bash
sudo ufw allow 8012
```

5. install ca-certificates on VisionPro:

```
mkcert -CAROOT
```

Copy the `rootCA.pem` via AirDrop to VisionPro and install it.

Settings > General > About > Certificate Trust Settings. Under "Enable full trust for root certificates", turn on trust for the certificate.

Settings > Apps > Safari > Advanced > Feature Flags > Enable WebXR Related Features.

## 2.3 🔎 Test environment

This step is to verify that the environment is installed correctly.

1. Download Isaac Gym: https://developer.nvidia.com/isaac-gym/download

    Extracting to the current directory, go to the `IsaacGym_Preview_4_Package/isaacgym/python` directory and execute the command:

    ```bash
    pip install -e .
    ```

2. After setup up streaming with local following the above instructions, you can try teleoperating two robot hands in Issac Gym:

    ```bash
    cd teleop
    python teleop_test_gym.py
    ```

3. Wear your Apple Vision Pro device.

4. Open Safari on Apple Vision Pro and visit: https://192.168.123.2:8012?ws=wss://192.168.123.2:8012

    > p.s. This IP address should match the IP address of your **Host machine**.

5. Click `Enter VR` and `Allow` to start the VR session.

6. See your hands in 3D!

<div style="center">
  <img src="https://oss-global-cdn.unitree.com/static/d079e884c3074e8495f81e78c9586f7f_2556x1179.jpg" width="50%">
</div>


# 3. 🚀 Usage

Please read the  [Official Documentation ](https://support.unitree.com/home/zh/Teleoperation) at least once before starting this program.


## 3.1 🖼️ Image Server

Copy `image_server.py` in the `avp_teleoperate/teleop/image_server` directory to the **PC2** of Unitree Robot (G1/H1/H1_2/etc.), and execute the following command **in the PC2**:

```bash
# Now located in Unitree Robot PC2
python image_server.py
# You can see the terminal output as follows:
# Image server has started, waiting for client connections...
# Image Resolution: width is x, height is x
```

After image service is started, you can use `image_client.py` **in the Host** terminal to test whether the communication is successful:

```bash
python image_client.py
```

## 3.2 ✋ Inspire hands Server (optional)

You can refer to [Dexterous Hand Development](https://support.unitree.com/home/zh/H1_developer/Dexterous_hand) to configure related environments and compile control programs. First, use [this URL](https://oss-global-cdn.unitree.com/static/0a8335f7498548d28412c31ea047d4be.zip) to download the dexterous hand control interface program. Copy it to PC of  Unitree H1_2. 

On Unitree H1_2's PC, execute command:

```bash
sudo apt install libboost-all-dev libspdlog-dev
# Build project
cd h1_inspire_service & mkdir build & cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
# Terminal 1. Run h1 inspire hand service
sudo ./inspire_hand -s /dev/ttyUSB0
# Terminal 2. Run example
./h1_hand_example
```

If two hands open and close continuously, it indicates success. Once successful, close the `./h1_hand_example` program in Terminal 2.

## 3.3 🚀 Start

> ![Warning](https://img.shields.io/badge/Warning-Important-red) 
>
> 1. Everyone must keep a safe distance from the robot to prevent any potential danger!
>
> 2. Please make sure to read the [Official Documentation](https://support.unitree.com/home/zh/Teleoperation) at least once before running this program.

It's best to have two operators to run this program, referred to as **Operator A** and **Operator B**.

Now, **Operator B** execute the following command on **Host machine** :

```bash
python teleop_hand_and_arm.py
```

And then, **Operator A**

1. Wear your Apple Vision Pro device.

2. Open Safari on Apple Vision Pro and visit : https://192.168.123.2:8012?ws=wss://192.168.123.2:8012

   > p.s. This IP address should match the IP address of your **Host machine**.

3. Click `Enter VR` and `Allow` to start the VR session.

When host terminal outputs "Please enter the start signal (enter 'r' to start the subsequent program):", **Operator B** can start teleoperation program by pressing the **r** key in the terminal.

At this time, **Operator A** can remotely control the robot's arms and dexterous hands.

Next, **Operator B** can press **s** key to begin recording data in the 'record image' window that opens, and press **s** again to stop. This can be repeated as necessary.

## 3.4 🔚 Exit

To exit the program, **Operator B** can press the **q** key in the 'record image'  window.  

>  ![Warning](https://img.shields.io/badge/Warning-Important-red) 
>
> To avoid damaging the robot, **Operator A** need to make sure the robot's arms are in a natural down position before exiting.



# 4. 🗺️ Codebase Tutorial

```
avp_teleoperate/
│
├── assets                    [Storage of robot URDF-related files]
│
├── teleop
│   ├── image_server
│   │     ├── image_client.py [Used to receive image data from the robot image server]
│   │     ├── image_server.py [Capture images from cameras and send via network (Running on robot's on-board computer)]
│   │
│   ├── open_television
│   │      ├── television.py    [Using Vuer to capture wrist and hand data from apple vision pro]  
│   │      ├── tv_wrapper.py    [Post-processing of captured data]
│   │
│   ├── robot_control
│   │      ├── robot_arm_ik.py        [Inverse kinematics of the arm]  
│   │      ├── robot_arm.py           [Control dual arm joints and lock the others]
│   │      ├── robot_hand_inspire.py  [Control inspire hand joints]
│   │      ├── robot_hand_unitree.py  [Control unitree hand joints]
│   │
│   ├── utils
│   │      ├── episode_writer.py          [Used to record data for imitation learning]  
│   │      ├── mat_tool.py                [Some small math tools]
│   │      ├── weighted_moving_filter.py  [For filtering joint data]
│   │
│   │──teleop_hand_and_arm.py   [Startup execution code for teleoperation]
|   |——teleop_test_gym.py       [Can be used to verify that the environment is installed correctly]
```



# 5. 🙏 Acknowledgement

This code builds upon following open-source code-bases. Please visit the URLs to see the respective LICENSES:

1) https://github.com/OpenTeleVision/TeleVision
2) https://github.com/dexsuite/dex-retargeting
3) https://github.com/vuer-ai/vuer
4) https://github.com/stack-of-tasks/pinocchio
5) https://github.com/casadi/casadi
6) https://github.com/meshcat-dev/meshcat-python
7) https://github.com/zeromq/pyzmq
8) https://github.com/unitreerobotics/unitree_dds_wrapper
9) https://github.com/tonyzhaozh/act
10) https://github.com/facebookresearch/detr
11) https://github.com/Dingry/BunnyVisionPro