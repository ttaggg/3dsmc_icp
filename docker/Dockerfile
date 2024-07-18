FROM linuxserver/code-server:latest

RUN apt-get -y update && apt-get install -y

# Install required dependencies
RUN apt-get -y install cmake libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev build-essential g++ gdb python3-dev autotools-dev libicu-dev libbz2-dev libboost-all-dev libflann-dev libfreeimage3 libfreeimage-dev lz4

# Configure GCC 9
RUN apt-get -y install gcc-9 g++-9
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 60 --slave /usr/bin/g++ g++ /usr/bin/g++-9

WORKDIR /
RUN mkdir build_image
WORKDIR /build_image

# Install Glog from package manager
RUN apt-get -y install libgoogle-glog-dev

# Install Ceres Solver
RUN apt-get -y install libceres-dev

WORKDIR /config/workspace/
# Build with: docker build . -t 3dsmc-icp
