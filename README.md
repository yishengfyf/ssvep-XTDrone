# SSVEP-XTDrone 闭环脑控无人机仿真系统

## 1\. 项目介绍

本项目构建了一个基于稳态视觉诱发电位（SSVEP）的闭环脑控无人机仿真系统。该系统利用**离线SSVEP脑电数据集**来模拟一个真实的在线BCI指令生成器，实现了对**XTDrone仿真平台**中无人机的实时控制。

系统的核心特色是采用了“**数字孪生**”的概念，通过UDP通讯将仿真与控制分离，实现了两个独立的运行环境：

  * **主机 (Host / NUC2)**: 模拟真实无人机，它在一个独立的仿真环境XRDrone中根据接收到的外部脑控指令进行物理飞行，并实时通过UDP广播自身的位姿状态。
  * **用户端 (Puppet / PC2)**: 负责生成SSVEP视觉刺激界面; 并通过回放真实的脑电数据，并进行实时解码；若解码错误在终端进行显示，若解码成功在刺激界面显示解码出来的动作指令，并将生成控制指令通过UDP发送给主机。同时，它会监听接收主机发回的位姿数据，基于实时反馈的位姿数据来驱动一个无物理引擎的“木偶”无人机模型，为用户提供一个纯净、实时的可视化控制结果的反馈。

整个系统通过自定义的**BUDP(Bci UDP)协议**进行控制指令的传输，并通过独立的UDP通道进行位姿数据的反馈，构成了一个完整的“**刺激-采集-解码-控制-反馈**”闭环。

该系统运行有两种部署方式：一种是同一台电脑，利用docker部署两个container，分别对应Host/NUC2和Puppet/PC2；另一种是在两台主机进行部署。该系统提供了一种基于BCI脑电指令的无人机控制系统的科研平台，可以高效验证基于真实SSVEP脑电信号控制的无人机的整个pipeline。

未来，可进一步接入实时采集的SSVEP脑电信号和解码算法，实现真实系统的搭建。

## 2\. 代码结构

```
XTDrone/
└── bci/                              # BCI脑机接口的相关项目
	└── ssvep-XTDrone-k/              # 脑控无人机仿真pipeline项目：git clone到bci下的默认文件夹名称：与仓库名称相同
    	├── data/                     # 存放SSVEP数据集文件
    	├── launchers/
    	│   ├── host_launcher.py      # 主机端(NUC2)的一键启动器
        │   ├── bci_launcher.py       # 用户端(PC2)的一键启动器
        │   └── test/                 # 存放各类测试脚本
        └── scripts/
            ├── budp_controller.py    # 【主机核心】接收BUDP指令并控制无人机
            ├── state_sender.py       # 【主机核心】订阅无人机位姿并通过UDP发送
            ├── bci_main.py           # 【用户端核心】SSVEP界面、脑电回放、解码与指令发送
            ├── pose_updater.py       # 【用户端核心】接收UDP位姿并更新木偶无人机
            ├── budp_sender.py        # BUDP指令发送器的类定义
            ├── ssvep_sender.py       # SSVEP刺激信号生成器的类定义
            └── test/                 # 存放各类测试脚本
```

## 3\. 使用指南

本指南提供两种测试模式：**A) 单机双容器测试** 和 **B) 双物理机部署**。

- 请在NUC2和PC2上，分别将仓库克隆到**新建的/XTDrone/bci目录**下；注意bci/下会默认生成一个与仓库同名的文件夹，用于代码的git管理，建议保留。
- 并将项目内的**outdoor3_visualizer.launch**文件剪贴至/PX4_Firmware/launch目录下；
- 同时确保/PX4_Firmware与/XTDrone处于**同一层级**。

-----

### **A. 机双容器测试指南**

此模式适用于在单台电脑上完整地模拟整个系统。

#### **A.1 环境准备**

1.  **Docker**: 确保宿主机已安装Docker。
2.  **Docker镜像**: 确保您已成功创建了一个名为`my_xtdrone_image:v1`的、包含了所有环境和依赖的Docker镜像。
3.  **Docker网络**: 创建一个专用的Docker网络：
    ```bash
    docker network create xtdrone_net
    ```
4.  **X11授权**: 每次重启电脑后，请在**宿主机**终端运行：
    ```bash
    xhost +
    ```
5. **中文环境配置**: 为了让SSVEP界面和终端能正常显示中文，请在每一个启动的容器内，都先执行一次中文环境配置脚本：
    ```bash
    # 在容器终端内运行
    apt-get update
    apt-get install -y language-pack-zh-hans fonts-wqy-zenhei
    locale-gen zh_CN.UTF-8
    export LANG=zh_CN.UTF-8
    export LC_ALL=zh_CN.UTF-8
    fc-cache -f -v
    ```
   该脚本会设置环境变量，确保中文字符能正确显示。

#### **A.2 启动双容器**

  * **启动容器A (主机 - nuc2\_host)**:
    ```bash
    docker run -it --name nuc2_host --hostname nuc2 --network xtdrone_net -e "DISPLAY=$DISPLAY" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" my_xtdrone_image:v1
    ```
  * **启动容器B (用户端 - pc2\_puppet)**:
    ```bash
    docker run -it --name pc2_puppet --hostname pc2 --network xtdrone_net -e "DISPLAY=$DISPLAY" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" my_xtdrone_image:v1
    ```

#### **A.3 运行仿真**

由于`gnome-terminal`在双容器下的兼容性问题，我们采用**手动启动**的方式，这最为稳健。如果您希望使用脚本自动化启动，请参考`launchers/host_launcher.py`和`launchers/bci_launcher.py`中的代码。**正常情况下可在一个容器（如nuc2_host）中运行自动化脚本，在另一容器中手动启动。**

**➡️ 在容器A (`nuc2_host`) 中启动主机 (需要打开4个终端)**

您需要通过 `docker exec -it nuc2_host bash` 命令，打开4个终端并分别执行以下命令：

1.  **终端1**: 启动主仿真环境
    ```bash
    roslaunch px4 outdoor3.launch
    ```
2.  **终端2**: 启动通信桥梁
    ```bash
    cd ~/XTDrone/communication/
    python3 multirotor_communication.py iris 0
    ```
3.  **终端3**: 启动BUDP指令接收器
    ```bash
    cd ~/XTDrone/bci/ssvep-XTDrone-k/scripts/
    python3 budp_controller.py iris vel
    ```
4.  **终端4**: 启动位姿状态发送器
    ```bash
    cd ~/XTDrone/bci/ssvep-XTDrone-k/scripts/
    python3 state_sender.py
    ```

**➡️ 在容器B (`pc2_puppet`) 中启动用户端 (需要打开3个终端)**

您需要通过 `docker exec -it pc2_puppet bash` 命令，打开3个终端并分别执行以下命令：

1.  **终端5**: 启动木偶可视化环境
    ```bash
    roslaunch px4 outdoor3_visualizer.launch
    ```
2.  **终端6**: 启动姿态更新器
    ```bash
    cd ~/XTDrone/bci/ssvep-XTDrone-k/scripts/
    python3 pose_updater.py
    ```
3.  **终端7**: 启动BCI主控程序
    ```bash
    cd ~/XTDrone/bci/ssvep-XTDrone-k/scripts/
    python3 bci_main.py
    ```

-----

### **B). 双物理机部署指南**

此模式是项目的理想运行状态，完全模拟真实场景。目前已在两台电脑上进行联合运行测试：

  * **主机 (Host / NUC2)**: NUC主机，Ubuntu 20.04 , ROS Noetic
  * **用户端 (Puppet / PC2)**:  服务器主机，Ubuntu 20.04, ROS Noetic

#### **B.1 环境准备**

1.  **两台电脑**:
    
      * **主机 (NUC2)**: 用于运行无人机物理仿真。
      * **用户端 (PC2)**: 用于运行SSVEP界面和可视化。
    
2. **网络连接**: 确保两台电脑连接在**同一个局域网**下（例如，连接到同一个路由器或WiFi）。

3. **环境安装**: 在**两台电脑**上，都需要按照XTDrone平台提供的安装教程，配置好ROS 1 Noetic, Gazebo, PX4以及所有依赖。

   参考安装PX4 1.13版对应的XTDrone：https://www.yuque.com/xtdrone/manual_cn/basic_config_13

4.  **获取IP地址**:
    
      * 在主机(NUC2)的终端里运行`ifconfig`或`ip addr`，记下其IP地址（例如`192.168.1.100`）。
      * 在用户端(PC2)的终端里运行`ifconfig`或`ip addr`，记下其IP地址（例如`192.168.1.101`）。

​	5 **获防火墙设置**:

​		-  Ubuntu默认防火墙（ufw）可能阻塞UDP：运行：

```
sudo ufw allow 20002/udp
sudo ufw reload
```

​		- 确认防火墙状态：

```
sudo ufw status
```



## B.2 关键代码配置（IP地址）

#### **B.2 关键代码配置（IP地址）**

1.  **配置`state_sender.py`**

      * 在**主机 (NUC2)** 上，编辑`~/XTDrone/bci/ssvep-XTDrone-k/scripts/state_sender.py`。
      * 将`UDP_IP`变量修改为**用户端 (PC2)** 的真实IP地址：
        ```python
        UDP_IP = "192.168.1.101" # <- 这里替换成PC2的IP
        ```

2.  **配置`bci_main.py`**

      * 在**用户端 (PC2)** 上，编辑`~/XTDrone/bci/ssvep-XTDrone-k/scripts/bci_main.py`。
      * 将`NUC2_IP`变量修改为**主机 (NUC2)** 的真实IP地址：
        ```python
        NUC2_IP = "192.168.1.100" # <- 这里替换成NUC2的IP
        ```

#### **B.3 运行仿真（一键启动）**

**➡️ 在主机 (NUC2) 上启动**

  * 打开一个终端，运行主机的一键启动器：
    ```bash
    cd ~/XTDrone/bci/ssvep-XTDrone-k/launchers/
    python3 host_launcher.py
    ```

**➡️ 在用户端 (PC2) 上启动**

  * 打开一个终端，运行用户端的一键启动器：
    ```bash
    cd ~/XTDrone/bci/ssvep-XTDrone-k/launchers/
    python3 bci_launcher.py
    ```


#### **B.4 预期结果**

与单机测试完全相同。用户在PC2上看到SSVEP界面，程序自动解码并发送指令到NUC2。NUC2上的无人机根据指令飞行，并将状态实时传回PC2，驱动PC2上的“木偶”无人机进行同步飞行。

## 4\. 致谢
  * **XTDrone 仿真平台**: 本项目的所有无人机仿真、控制及测试均基于强大的开源XTDrone平台。
      * 项目链接: [https://github.com/robin-shaun/XTDrone](https://github.com/robin-shaun/XTDrone)
  * **SSVEP 公开数据集**: 脑电解码模拟功能使用了由Nakanishi等研究者提供的12分类SSVEP数据集，为项目的逼真模拟提供了宝贵的数据支持。
      * 数据集链接: [https://github.com/mnakanishi/12JFPM\_SSVEP](https://github.com/mnakanishi/12JFPM_SSVEP)
      * 参考文献: Masaki Nakanishi, Yijun Wang, Yu-Te Wang and Tzyy-Ping Jung, "A Comparison Study of Canonical Correlation Analysis Based Methods for Detecting Steady-State Visual Evoked Potentials," PLoS One, vol.10, no.10, e140703, 2015.