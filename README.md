# ozeki_hikitugi

実際にうごかすとこんな感じになります。

![gif](https://github.com/leap-kun/ozeki_hikitugi/blob/main/image/OpenPose_KHR_test.gif)

本研究はravi Joshi氏が作成した、ros_openposeとMeridian計画をかけ合わせたものである。

## Overview

このリポジトリは、他の研究生が自前のPCを使う為に一応残しておくためのものです。
かなり粗があるかと思いますがご容赦くださいm(_ _)m

## Requirement

**今現段階の環境**

- Ubuntu 20.04
- CUDA 11.7
- Cudnn 8.5
- Nvidia Driver:535

## attention!!

研究室のPCのNvidia Driverのversionを今のやつより幾つか古くして欲しいです。(研究室にあるPCのGPUのドライバーが古い為)  
恐らく,Nvidia Driverの*verison*が新しいせいで、起動時に暗く**点滅**したり**ノイズが入ること**が多々あります。  
そのため、研究室のPCを扱う上で、**Nvidia Driverのインストール等**を覚えてもらう為に一度、手を動かして感覚を補いましょう！！  
参考**Reference**に参考になるサイトも掲載していますので、がんばりましょう！！


## Nvidia Driverインストールの注意点

**recomended**って書いてあるやつは基本入れない。  
基本的に,non-free等の何も書いてないやつをインストールする。


## Usage

研究室のPCを扱う上で以下のコマンドを入力すれば動きます。

```
cd catkin_ws
source devel/setup.bash
roslaunch ros_openpose run.launch camera:=azurekinect
```
この際に、AzurekinectDKを接続してください。

## Azure kinect ROSのDriverについて

自前のPCで実装したい場合に、Usageに記載しているコマンドを入力しても実行出来ない為、  
Referenceにある[azure_kinect_ROS_Driver](https://github.com/microsoft/Azure_Kinect_ROS_Driver)をインストールと、[azure kinect DK ROS Noetic use](https://qiita.com/sunrise_lover/items/1a70ddafee44419eda2a)
を参照しながら実装してください。


## 手法1と手法2の切り替え

launchファイルの中に入っている,core.launchの下記の部分を変更する。

```
 <!-- print ros message to terminal -->
  <group if="$(arg print)" >
  <!-- output: ターミナルに値を表示するか否か, requireded:ノードが必須かどうか , type:実行するプログラム-->
    <node name="echo" pkg="ros_openpose" type="echo_3D.py" output = "screen" required="true" >
      <param name="pub_topic" value="$(arg pub_topic)" />
    </node>
       <!--Meridian consoleを起動するための部分 -->

    <node name = "Subscribe_Console" pkg="ros_openpose" type="Meridian_OpenPose_console_3D.py" output = "screen"/>
    
  </group>
</launch>
```

手法1の場合

```
type = "echo_3D_NoIK.py" , type = "Meridian_OpenPose_console_3D_NoIK.py"
```
手法2の場合
```
type = "echo_3D.py" , type = "Meridian_OpenPose_console_3D.py"
```

手法1と手法2の２つのプログラムに共通することだが、一度launchファイルで実行すると、**角度と座標データをrosbagファイル**で自動的に保存する。
これらは卒論では話していない。(冗長になるため)


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
[azure kinect DK ROS Noetic use](https://qiita.com/sunrise_lover/items/1a70ddafee44419eda2a)
[Nvidia_Driverおすすめ構築サイト](https://qiita.com/tf63/items/0c6da72fe749319423b4)  
[Meridian_Twin](https://github.com/Ninagawa123/Meridian_TWIN%EF%BF%BC)  
[Meridian_LITE](https://github.com/Ninagawa123/Meridian_LITE)
