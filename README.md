[日本語での説明](#SUMO用のgym環境)  
[explanation in English](#gym-environment-for-sumo)
# SUMO用のgym環境

## 必須システム要件
OS : Linux, Windows7以上, macOS  
(ただし、動作確認したのはUbuntu18.04LTSのみ)  
ソフトウェア : SUMO, python3  
おすすめのpython環境 : anaconda を使ったpython3.7

## SUMO インストール方法
Linux (Ubuntu):  
以下のURLからインストール方法を確認できます。  
[Installing/Linux Build](https://sumo.dlr.de/docs/Installing/Linux_Build.html)

または、以下の手順に沿ってインストールできます。
1. SUMOをビルドするために必要なすべてのtools,ライブラリをインストールします。: 
    ```
    $ sudo apt-get install cmake python g++ libxerces-c-dev libfox-1.6-dev libgdal-dev libproj-dev libgl2ps-dev swig
    ```
1. SUMOのソースコードを取得:
    ```
    $ git clone --recursive https://github.com/eclipse/sumo
    $ export SUMO_HOME="$PWD/sumo"
    ```
1. SUMOをビルド:

    ```
    $ mkdir sumo/build/cmake-build && cd sumo/build/ cmake-build
    $ cmake ../..
    $ make -j$(nproc)
    ```

その他のOS:  
確認できていないので、以下のURLを見て、インストールしてみてください。:  
[Installing](https://sumo.dlr.de/docs/Installing.html)

## install gym environment
1. 前のセクションのインストール方法を見て、SUMOをインストールする。
1. gym環境のソースコードを取得する  
    もし、gitにssh登録しているなら:
    ```
    $ cd ~
    $ git clone git@git.esslab.jp:tomo/gym-sumo.git
    ```
    登録していないなら:
    ```
    $ cd ~
    $ git clone https://git.esslab.jp/tomo/gym-sumo.git
    ```
1. 実行するために必要なpythonパッケージをインストール
    ```
    $ cd ~/gym-sumo/gym-sumo
    $ bash install.sh
    ```

## gym環境の使い方の例
以下のようなコマンドを打つことでテストコードを実行できます:
```
$ cd ~/gym-sumo
$ python sampletraci.py
```

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
