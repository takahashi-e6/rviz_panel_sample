# rviz_panel_sample

# 資料
https://www.slideshare.net/Takahashi1/rvizplugin

# 使い方
## インストールからビルド
```
$ git clone https://github.com/takahashi-e6/rviz_panel_sample
$ cd rviz_panel_sample
$ rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
$ catkin_make
```
## 実行
```
$ source devel/setup.bash 
$ roslaunch sample_panel sample_panel.launch
```
Rvizの画面が出たら上部メニューの
「Panels」→「AddNewPanel」→「sample_panel」→「SamplePanel」
を選択して、OKボタンをクリック。

後は、追加されたパネル上でスライダーを動かせば動作します。
