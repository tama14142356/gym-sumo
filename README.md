# gym environment for sumo

## prerequirement
OS : Linux, Windows  
(confirmed operation with Ubuntu18.04LTS)  
software : SUMO, python3  
recommend python environment : anaconda python=3.7  

### install SUMO
Linux(Ubuntu):  
you can get infomation about the way to install SUMO by accessing the following URL:  
[Installing/Linux Build](https://sumo.dlr.de/docs/Installing/Linux_Build.html)

Or you can build sumo by following:
1. Install all of the required tools and libraries: 
    ```
    $ sudo apt-get install cmake python g++ libxerces-c-dev libfox-1.6-dev libgdal-dev libproj-dev libgl2ps-dev swig
    ```
1. Get the source code:
    ```
    $ git clone --recursive https://github.com/eclipse/sumo
    $ export SUMO_HOME="$PWD/sumo"
    ```
1. Build the SUMO binaries

    ```
    $ mkdir sumo/build/cmake-build && cd sumo/build/ cmake-build
    $ cmake ../..
    $ make -j$(nproc)
    ```

other OS:  
check following URL:  
[Installing](https://sumo.dlr.de/docs/Installing.html)

## install gym environment
1. install SUMO by following the previous section
1. get source code  
    if ssh is registered:
    ```
    $ cd ~
    $ git clone git@git.esslab.jp:tomo/gym-sumo.git
    ```
    if ssh is not registered:
    ```
    $ cd ~
    $ git clone https://git.esslab.jp/tomo/gym-sumo.git
    ```
1. install requirement packages
    ```
    $ cd ~/gym-sumo/gym-sumo
    $ bash install.sh
    ```

## Usage gym environment
you can run the test code by following:
```
$ cd ~/gym-sumo
$ python sampletraci.py
```
