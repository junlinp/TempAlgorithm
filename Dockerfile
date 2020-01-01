from ubuntu:18.04
RUN apt-get update & apt-get install -y git
RUN git clone https://github.com/opencv/opencv.git & git clone https://github.com/opencv/opencv_contrib.git
RUN cd opencv & mkdir build & cd build & cmake .. & make
RUN
