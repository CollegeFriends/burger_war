# GitHubからのリポジトリのフォーク

## Gitの設定

説明不要だと思うけどねんのため。

```bash:
$ sudo apt-get install git
$ git config --global user.name "Yoda M"
$ git config --global user.email "xxx@hotmail.com"
```

## GitHubの個人アカウントにフォーク
説明不要だと思うけど、ねんのため。
[チームのリポジトリ](https://github.com/CollegeFriends/burger_war)にアクセスして右上のForkをクリック。


## 通信をSSHに切り替える
これしてからフォークをしないと、PushするたびにIDとPassword聞かれて面倒なので設定。
まず、[このサイト](https://qiita.com/shizuma/items/2b2f873a0034839e47ce)の手順で接続を確かめるまで実施

## SSHでリポジトリをフォーク
GitHubの個人のリポジトリ(CollegeFriendsからフォークしたやつ)にアクセス。  
Clone or DownloadのClone with SSHのコマンドをコピーする  
(おそらくgit@github.com:hogehoge/burger_warみたいになっているはず。)  
*** Clone with HTTPSとなっていたら、Use SSHを押して切り替えること！！ ***  
下記コマンドで~/catkin_ws/src/内にフォークできる  

```bash:
$ cd ~/catkin_ws/src/
$ git clone **さっきコピーしたコマンド**
```

## 上流リポジトリの追加
本家(OneNightROBOCON)やチーム(CollegeFriends)のリポジトリの更新分を反映するために必要

```bash:
# 更新されてるかどうか把握するため、リモートリポジトリの設定に追加
$ cd ~/catkin_ws/src/burger_war
$ git remote add master_stream git@github.com:OneNightROBOCON/burger_war.git
$ git remote add upstream git@github.com:CollegeFriends/burger_war.git
```
```bash:
# 更新されているかどうか把握
$ git fetch master_stream
$ git fetch upstream
```
```bash:
# 更新を反映する
$ git merge master_stream/master
$ git merge upstream/master
```
```bash:
# ちゃんと設定できているかどうかの確認
$ git branch -a

#  dev
#* master
#  remotes/master_stream/dev
#  remotes/master_stream/master
#  remotes/master_stream/move_base_issue
#  remotes/origin/HEAD -> origin/master
#  remotes/origin/dev
#  remotes/origin/master
#  remotes/origin/move_base_issue
#  remotes/upstream/dev
#  remotes/upstream/master
#  remotes/upstream/move_base_issue
```

---

# burger_warのための変更

## ~/.bashrcの変更

~/.bashrcに下記を追記

```bash:~/.bashrc
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export GAZEBO_MODEL_PATH=$HOME/catkin_ws/src/burger_war/burger_war/models/
export TURTLEBOT3_MODEL=burger
```

## 追加ライブラリのインストール

### キーボード操作やburger_navigationの方の動作のため

下記がないとうまく動作しないスクリプトがあった。
```bash:
# teleop-twist-keyboard (キーボード操作のライブラリ)
$ sudo apt-get install ros-kinetic-teleop-twist-keyboard
# burger_navigation関連のエラーが出ていたので解決するするために導入
$ sudo apt-get install ros-kinetic-navigation
$ sudo apt-get install ros-kinetic-gmapping ros-kinetic-amcl ros-kinetic-map-server 
```

# メモ(For Matsumura)
今後の障害物検知の際にobstacle_detectorを使用するかも。
その際の手順。
``` bash:
# obstacle_detectorに必要な数学ライブラリ
$ sudo apt-get install -y libarmadillo-dev libarmadillo6 
# obstacle_detectorのROSファイルのダウンロード
$ cd ~/catkin_ws/src/
$ git clone https://github.com/tysik/obstacle_detector.git
```

# 試合の初期化
start2.shを止めたと後に、下記スクリプトを実行
```bash:
# 試合情報リセットスクリプト
$ bash scripts/reset.sh
```


(参考までに)
```bash:
# ロボットの位置の初期化
$ rosservice call /gazebo/reset_simulation "{}"
# 審判サーバーのリセット
$ bash judge/test_scripts/reset_server.sh localhost:5000
```