FROM docker.io/osrf/ros:humble-desktop-full

# 2. 새 사용자 생성 (호스트 유저와 UID/GID 맞춤)
ARG USERNAME
ARG USER_UID
ARG USER_GID

# 기본 쉘 설정
SHELL ["/bin/bash", "-c"]

# 2. 필수 패키지 및 Nav2 의존성 설치
RUN apt update && apt upgrade -y && apt install -y \
    python3-pip \
    ros-humble-xacro \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-vision-opencv \
    ros-humble-joint-state-publisher-gui \
    ros-humble-joint-state-publisher \
    # --- [Nav2 관련 패키지 추가] ---
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3-simulations \
    # -----------------------------
    libopencv-dev \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*0

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# 1. Gazebo 공식 모델 및 TurtleBot3 모델 미리 다운로드 (바닥 추락 방지)
RUN mkdir -p /home/$USERNAME/.gazebo/models && \
    git clone https://github.com/osrf/gazebo_models.git /home/$USERNAME/.gazebo/models/gazebo_models_repo && \
    mv /home/$USERNAME/.gazebo/models/gazebo_models_repo/* /home/$USERNAME/.gazebo/models/ && \
    rm -rf /home/$USERNAME/.gazebo/models/gazebo_models_repo

# 3. 작업 디렉토리 설정 및 소유권 변경
WORKDIR /home/$USERNAME/colcon_ws
RUN chown -R $USERNAME:$USERNAME /home/$USERNAME/colcon_ws
RUN chown -R $USERNAME:$USERNAME /home/$USERNAME/.gazebo

# 4. 일반 사용자로 전환
USER $USERNAME

# 환경 변수 자동 로드
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "if [ -f ~/colcon_ws/install/setup.bash ]; then source ~/colcon_ws/install/setup.bash; fi" >> ~/.bashrc \
    && echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc \
    && echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/$USERNAME/.gazebo/models:/opt/ros/humble/share/turtlebot3_gazebo/models" >> ~/.bashrc \
    && echo "export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:/usr/share/gazebo_models" >> ~/.bashrc \
    && echo "export GAZEBO_MODEL_DATABASE_URI=''" >> ~/.bashrc

ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

CMD ["bash"]
