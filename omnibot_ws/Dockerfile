# 1. Define a imagem base com ROS 2 Humble
FROM osrf/ros:humble-desktop

# 2. Garante que o ambiente ROS 2 seja carregado no terminal
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# 3. Define o diretório de trabalho com o nome do seu projeto
WORKDIR /omnibot_ws

# 4. Copia o código-fonte (será substituído pelo volume no desenvolvimento)
COPY ./src /omnibot_ws/src

# 5. Inicializa rosdep (de forma segura) e instala todas as dependências do workspace
RUN apt-get update && \
    apt-get install libserial-dev && \
    rosdep init || true && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro humble -y --ignore-src && \
    rm -rf /var/lib/apt/lists/*

# 6. Compila o workspace
RUN . /opt/ros/humble/setup.bash && colcon build