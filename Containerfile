FROM docker.io/osrf/ros:humble-desktop-full

# 2. 새 사용자 생성 (호스트 유저와 UID/GID 맞춤)
ARG USER_NAME

# 2. 필수 패키지 및 Nav2 의존성 설치
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    locales clangd python3-pip jq \
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

RUN locale-gen en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN useradd -m -s /bin/bash -G sudo $USER_NAME \
    && echo "$USER_NAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USER_NAME \
    && chmod 0440 /etc/sudoers.d/$USER_NAME

RUN usermod --shell /bin/bash $USER_NAME

# 1. Gazebo 공식 모델 및 TurtleBot3 모델 미리 다운로드 (바닥 추락 방지)
RUN mkdir -p /home/$USER_NAME/.gazebo/models && \
    git clone https://github.com/osrf/gazebo_models.git /home/$USER_NAME/.gazebo/models/gazebo_models_repo && \
    mv /home/$USER_NAME/.gazebo/models/gazebo_models_repo/* /home/$USER_NAME/.gazebo/models/ && \
    rm -rf /home/$USER_NAME/.gazebo/models/gazebo_models_repo

# 3. 작업 디렉토리 설정 및 소유권 변경
RUN chown -R $USER_NAME:$USER_NAME /home/$USER_NAME/.gazebo

RUN mkdir -p /run/user/1000 && chown -R $USER_NAME:$USER_NAME /run/user/1000

# 환경 변수 자동 로드
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/profile.d/ros.sh \
    && echo "if [ -f /home/$USER_NAME/workspace/install/setup.bash ]; then source /home/$USER_NAME/workspace/install/setup.bash; fi" >> /etc/profile.d/ros.sh \
    && echo "export TURTLEBOT3_MODEL=burger" >> /etc/profile.d/ros.sh \
    && echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/$USER_NAME/.gazebo/models:/opt/ros/humble/share/turtlebot3_gazebo/models" >> /etc/profile.d/ros.sh \
    && echo "export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:/usr/share/gazebo_models" >> /etc/profile.d/ros.sh \
    && echo "export GAZEBO_MODEL_DATABASE_URI=''" >> /etc/profile.d/ros.sh

CMD ["/bin/bash"]
