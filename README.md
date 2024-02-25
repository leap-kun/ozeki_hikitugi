# ozeki_hikitugi


# name

image or gif

## Overview

ros_openpoeを実装する際には、scrapboxに上がっているメモ書きを参照してください。 基本的には、home/catkin_ws/src/ros_openpose/の直下にある.CmakeListsやpackage.xmlを参照にすれば大丈夫かと思います。

このリポジトリは、他の研究生が自前のPCを使い為に一応残しておくためのものです。
かなり粗があるかと思いますがご容赦くださいm(_ _)m

## Requirement

**今現段階の環境**

- Ubuntu 20.04
- CUDA 11.7
- Cudmm 8.5
- Nvidia Driver:535

## attention!!

研究室のPCのNvidia Driverのversionを今のやつより幾つか古くして欲しいです。
恐らく,Nvidia Driverの*verison*が新しいせいで、起動時に暗く**点滅**したり**ノイズが入ること**が多々あります。
そのため、研究室のPCを扱う上で、**Nvidia Driverのインストール等**を覚えてもらう為に一度、手を動かして感覚を補いましょう！！
## Usage

研究室のPCを扱う上で以下のコマンドを入力すれば動きます。

```
cd catkin_ws
source devel/setup.bash
roslaunch ros_openpose run.launch camera:=azurekinect
```


## Features



## Author

梶田研究室卒業生
ER20027:尾関健太郎

## なんちゃってhelp
このリポジトリは、2023年度の卒研生:尾関健太郎の引き継ぎです。 分からない事があったら、覚えている範囲&分かる範囲でSlackでお答えします。


## Reference

[openpose](https://github.com/CMU-Perceptual-Computing-Lab/openpose)  
[ros_openpose](https://github.com/ravijo/ros_openpose)  
[ros_openposeの構築の上で参照になったサイト](https://yoshistl.hatenablog.com/entry/2022/07/31/014254)  
[azure_kinect_ROS_Driver](https://github.com/microsoft/Azure_Kinect_ROS_Driver)  
[Nvidia_Driverおすすめ構築サイト](https://qiita.com/tf63/items/0c6da72fe749319423b4)
