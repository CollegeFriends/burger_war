# Anaconda導入

1. Anacondaをインストール

    ```bash
    $ cd Downloads
    $ wget https://repo.anaconda.com/archive/Anaconda3-2020.07-Linux-x86_64.sh
    $ bash Anaconda3-2020.07-Linux-x86_64.sh
    ```

1. 仮想環境作成

    ```bash
    $ conda create -n CollegeFriends python=3.8.5
    $ conda activate CollegeFriends
    $ conda install tensorflow==2.2.0
    $ conda install keras==2.4.3
    $ conda install -c conda-forge ros-rospy==1.14.3
    ```

1. `.bashrc` に追加されたanaconda関連のスクリプトをコメントアウト

    `~/anaconda3/envs/CollegeFriends/bin/python` でPythonを実行できる

    shebangに記入するときは
    ```bash
    #!/[user]/anaconda3/envs/CollegeFriends/bin/python
    ```
    と絶対パスにする



# 仮想環境への入り方

```bash
$ . ~/anaconda3/etc/profile.d/conda.sh
$ conda activate CollegeFriends
```

`. ~/anaconda3/etc/profile.d/conda.sh` でcondaコマンドが使える