[日本語での説明](#SUMO用のgym環境)  
[explanation in English](#gym-environment-for-sumo)
# SUMO用のgym環境

## 必須システム要件
OS : Linux, Windows7以上, macOS  
(ただし、動作確認したのはUbuntu18.04LTSのみ)  
ソフトウェア : SUMO, python3  
おすすめのpython環境 : anaconda を使ったpython3.7以上

## SUMO インストール方法
Linux (Ubuntu):  
以下のURLからインストール方法を確認できます。  
[Installing/Linux Build](https://sumo.dlr.de/docs/Installing/Linux_Build.html)

または、以下の手順に沿ってインストールできます。
1. SUMOをビルドするために必要なすべてのtools,ライブラリをインストール  
   以下のコマンドを実行

    ```
    $ sudo apt-get install cmake python g++ libxerces-c-dev libfox-1.6-dev libgdal-dev libproj-dev libgl2ps-dev swig
    ```

1. SUMOのソースコードを取得  
   以下のコマンドを実行

    ```
    $ git clone --recursive https://github.com/eclipse/sumo
    $ export SUMO_HOME="$PWD/sumo"
    ```

1. SUMOをビルド  
   以下のコマンドを実行

    ```
    $ mkdir sumo/build/cmake-build && cd sumo/build/ cmake-build
    $ cmake ../..
    $ make -j$(nproc)
    ```

1. SUMO用PATHの設定  
   以下の内容を~/.bashrcまたは~/.bash_profileの最後の行に追記する。  
   ただし、SUMO_HOMEの部分はsumoをインストールしたパスを書く。ここでは~/においてインストールしたとする。

    ```sh
    export SUMO_HOME="~/sumo"
    export PATH="$SUMO_HOME/bin:$PATH"
    export PYTHONPATH="$SUMO_HOME/tools:$PYTHONPATH"
    ```

その他のOS:  
確認できていないので、以下のURLを見て、インストールしてみてください。:  
[Installing](https://sumo.dlr.de/docs/Installing.html)

## install gym environment
1. 前のセクションのインストール方法を見て、SUMOをインストールする。
1. gym環境のソースコードを取得する  
    もし、gitにssh登録しているなら以下のコマンドを実行

    ```
    $ cd ~
    $ git clone git@git.esslab.jp:tomo/gym-sumo.git
    ```

    登録していないなら以下のコマンドを実行

    ```
    $ cd ~
    $ git clone https://git.esslab.jp/tomo/gym-sumo.git
    ```

1. 実行するために必要なpythonパッケージをインストール  
   CUDAが使えるGPUが搭載されているなら、以下のコマンドを実行

    ```
    $ cd ~/gym-sumo/gym-sumo
    $ bash install_gpu.sh
    ```

   cpuしか使えないなら、以下のコマンドを実行

    ```
    $ cd ~/gym-sumo/gym-sumo
    $ bash install.sh
    ```

## gym環境の使い方の例
テストコード実行  
以下のコマンドを実行することでテストコードが実行されます。

```
$ cd ~/gym-sumo
$ python test_gym.py
```

# gym environment for sumo

## prerequirement
OS : Linux, Windows7+  
(confirmed operation with Ubuntu18.04LTS)  
software : SUMO, python3  
recommend python environment : anaconda python 3.7+  

### install SUMO
Linux(Ubuntu):  
you can get infomation about the way to install SUMO by accessing the following URL:  
[Installing/Linux Build](https://sumo.dlr.de/docs/Installing/Linux_Build.html)

Or you can build and install sumo by following:
1. Install all of the required tools and libraries: 
   run the following command

    ```
    $ sudo apt-get install cmake python g++ libxerces-c-dev libfox-1.6-dev libgdal-dev libproj-dev libgl2ps-dev swig
    ```

1. Get the source code:  
   run the following command

    ```
    $ git clone --recursive https://github.com/eclipse/sumo
    $ export SUMO_HOME="$PWD/sumo"
    ```

1. Build the SUMO binaries:  
   run the following command

    ```
    $ mkdir sumo/build/cmake-build && cd sumo/build/ cmake-build
    $ cmake ../..
    $ make -j$(nproc)
    ```

1. set the path for sumo:  
   append following contents to the end of the file named by ~/.bashrc or ~/.bash_profile
   and you should write the path for sumo against SUMO_HOME.   
   In this time, you install sumo on ~/.

    ```sh
    export SUMO_HOME="~/sumo"
    export PATH="$SUMO_HOME/bin:$PATH"
    export PYTHONPATH="$SUMO_HOME/tools:$PYTHONPATH"
    ```

other OS:  
check following URL:  
[Installing](https://sumo.dlr.de/docs/Installing.html)

## install gym environment
1. install SUMO by following the previous section
1. get source code  
    if ssh is registered, run the following command:
    
    ```
    $ cd ~
    $ git clone git@git.esslab.jp:tomo/gym-sumo.git
    ```
    
    if ssh is not registered, run the following command:
    
    ```
    $ cd ~
    $ git clone https://git.esslab.jp/tomo/gym-sumo.git
    ```

1. install requirement packages:
   if your system has gpu using by cuda driver, run the following command

    ```
    $ cd ~/gym-sumo/gym-sumo
    $ bash install_gpu.sh
    ```

   if your system has only cpu, run the following command

    ```
    $ cd ~/gym-sumo/gym-sumo
    $ bash install_gpu.sh
    ```

## Usage gym environment
you can run the test code by following:

```
$ cd ~/gym-sumo
$ python test_gym.py
```
