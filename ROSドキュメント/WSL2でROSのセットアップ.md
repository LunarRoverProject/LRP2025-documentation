ストレージ要件：30 ～ 70 　できれば 150GB
RAM 要件：16GB 以上あると望ましい　最低 8

参考：https://qiita.com/GAI-313/items/fd393ca505da814692d6

## Windows の機能を有効可

1. Windows の機能の有効化または無効化　を開く
2. 以下のようなウィンドウが出る
   ![[Pasted image 20250723112204.png]]
3. 以下を有効化し、OK
   - Hyper-V
   - Linux 用 Windows サブシステム
   - Windows ハイパーバイザープラットフォーム
   - 仮想マシンプラットフォーム（あれば）
4. 再起動

## WSL2 のインストール

1. パワーシェルを開く
2. 以下のコマンドを実行

```
wsl --install -d Ubuntu-22.04
```

3.  Ubuntu の初期設定
    - Enter new UNIX username:　と出たら、ユーザーネームを記入
    - New password: と出たらパスワードを記入
    - パスワード再入力
    - 完了
      ![[Pasted image 20250723112711.png]]

## Hyper-V のセットアップ

以下のサイトを参照
https://qiita.com/GAI-313/items/0f734c0d06b55d58b08b

## WSL2 に ROS2 をインストール

1. アップデート

```
sudo apt update
```

2. ROS2 のインストールに必要なパッケージをインストール

```
sudo apt install curl gnupg lsb-release net-tools
```

3. 以下のコマンドを順に実行

```
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

3. ROS Humble (base)のインストール

```
sudo apt install ros-humble-ros-base
```

## ROS2 の初期設定

1. bashrc ファイル（ターミナル起動時に読み込まれるファイル）に ROS の起動コマンドを記述
   - nano はテキストエディタアプリ

```
nano ~/.bashrc
```

    最後の行に以下を追加

```
source /opt/ros/humble/setup.bash
```

![[Pasted image 20250723114323.png]] - Ctrl+s で保存　 Ctrl+x で閉じる 2. ros 関連ツールのインストール

```shell
sudo apt update && sudo apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools
```

```shell
sudo apt install -y \
   python3-flake8-blind-except \
   python3-flake8-builtins \
   python3-flake8-class-newline \
   python3-flake8-comprehensions \
   python3-flake8-deprecated \
   python3-flake8-import-order \
   python3-flake8-quotes \
   python3-pytest-repeat \
   python3-pytest-rerunfailures
```

```shell
sudo apt upgrade
```

```shell
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
```

## Turtlesim でのテスト

1. turtlesim を入れる

```shell
sudo apt install ros-humble-turtlesim
```

2.  turtlesim を起動

```shell
ros2 run turtlesim turtlesim_node
```

![[Pasted image 20250723121035.png]]

3. 新しい Ubuntu コマンドウィンドウを開いて topic を確認

```shell
ros2 topic list
```

以下のように出ていれば成功
![[Pasted image 20250723120510.png]] 4. 新しい Ubuntu コマンドウィンドウで、キーボード操作パッケージをインストール

```shell
sudo apt install ros-humble-teleop-twist-keyboard
```

5. teleop twist keyboard を起動

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

以下のように出ていれば OK
　![[Pasted image 20250723120934.png]]
　 i キーで前進、u キーで右旋回　のように操作できる

## ROS2 のコマンド一覧

#### 1. 環境設定とワークスペース管理

- **ROS 2 環境のソース:**（重要！）

  ```bash
  source /opt/ros/humble/setup.bash  # ROS 2本体のセットアップ（bashrcに書いたので必要なし）
  source ~/your_ros2_ws/install/setup.bash # ワークスペースのセットアップ（colcon build後　これもbashrcに書いておくと楽）
  ```

  _ヒント: `~/.bashrc` に追加すると便利です。_

- **ワークスペースの作成:**（重要！）

  ```bash
  mkdir -p ~/your_ros2_ws/src
  cd ~/your_ros2_ws
  ```

- **パッケージのクローン/作成:**（重要！）
  ```bash
  cd ~/your_ros2_ws/src
  git clone <repository_url> # 既存パッケージのクローン
  ros2 pkg create --build-type ament_python your_package_name # Pythonパッケージの作成
  ros2 pkg create --build-type ament_cmake your_package_name # C++パッケージの作成
  ```

* **オープンソースパッケージのインストール**

```shell
	sudo apt install ros-humble-<package_name> #例　ros-humble-turtlesim
```

- **ワークスペースのビルド:**（重要！）

  ```bash
  cd ~/your_ros2_ws
  colcon build                   # ワークスペース内の全パッケージをビルド
  colcon build --packages-select your_package_name # 特定のパッケージのみビルド
  colcon build --symlink-install # 開発中にソースコードを変更した際に再ビルド不要になる (Python)
  ```

- **ビルドキャッシュのクリーンアップ:**
  ```bash
  rm -rf install log build
  ```

#### 2. ノードの実行と管理

- **ノードの実行:**（重要！）

  ```bash
  ros2 run <package_name> <executable_name> # 例: ros2 run turtlesim turtlesim_node
  ```

- **Launch ファイルの実行:**

  ```bash
  ros2 launch <package_name> <launch_file_name.launch.py> # 例: ros2 launch turtlesim turtle_rviz.launch.py
  ```

- **実行中の ROS ノード一覧表示:**（重要！）

  ```bash
  ros2 node list
  ```

- **特定のノード情報の表示:**
  ```bash
  ros2 node info /<node_name> # 例: ros2 node info /turtlesim
  ```

#### 3. トピックの操作

- **アクティブなトピックの一覧表示:**（重要！）

  ```bash
  ros2 topic list
  ros2 topic list -t # トピックの型も表示
  ```

- **トピック情報の表示 (パブリッシャー/サブスクライバー、メッセージ型):**

  ```bash
  ros2 topic info /<topic_name> # 例: ros2 topic info /turtle1/cmd_vel
  ```

- **トピックにパブリッシュされるメッセージの表示:**（重要！）

  ```bash
  ros2 topic echo /<topic_name> # 例: ros2 topic echo /turtle1/pose
  ros2 topic echo /<topic_name> --once # 一度だけ表示
  ros2 topic echo /<topic_name> -c # 継続的に表示し、Ctrl+Cで停止
  ```

- **手動でトピックにメッセージをパブリッシュ:**（重要！）

  ```bash
  ros2 topic pub /<topic_name> <message_type> '<data>' # 例:
  ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
  ```

  _ヒント: `ros2 interface show <message_type>` でメッセージの構造を確認できます。_

- **トピックの帯域幅とレートの表示:**
  ```bash
  ros2 topic bw /<topic_name>
  ros2 topic hz /<topic_name>
  ```

#### 4. サービスとアクションの操作

- **アクティブなサービスの一覧表示:**

  ```bash
  ros2 service list
  ros2 service list -t # サービス型も表示
  ```

- **特定のサービス情報の表示:**

  ```bash
  ros2 service info /<service_name> # 例: ros2 service info /spawn
  ```

- **サービス要求の呼び出し:**

  ```bash
  ros2 service call /<service_name> <service_type> '<request_data>' # 例:
  ros2 service call /spawn turtlesim/srv/Spawn "{x: 1.0, y: 2.0, theta: 0.0, name: 'my_turtle'}"
  ```

  _ヒント: `ros2 interface show <service_type>` でサービス要求/応答の構造を確認できます。_

- **アクティブなアクションの一覧表示:**

  ```bash
  ros2 action list
  ros2 action list -t # アクション型も表示
  ```

- **特定のアクション情報の表示:**

  ```bash
  ros2 action info /<action_name>
  ```

- **アクションゴールの送信:**
  ```bash
  ros2 action send_goal /<action_name> <action_type> '<goal_data>' # 例:
  ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
  ```

#### 5. パラメータの操作

- **ノードのパラメータ一覧表示:**

  ```bash
  ros2 param list /<node_name> # 例: ros2 param list /turtlesim
  ```

- **ノードの特定のパラメータ値の取得:**

  ```bash
  ros2 param get /<node_name> <parameter_name> # 例: ros2 param get /turtlesim background_r
  ```

- **ノードのパラメータ値の設定:**

  ```bash
  ros2 param set /<node_name> <parameter_name> <value> # 例: ros2 param set /turtlesim background_r 255
  ```

- **パラメータのロード/ダンプ (YAML ファイル):**
  ```bash
  ros2 param dump /<node_name> > <file_name.yaml> # ノードの全パラメータをファイルに保存
  ros2 param load /<node_name> <file_name.yaml> # ファイルからパラメータをロード
  ```

#### 6. ROS 2 デバッグツール

- **ログレベルの設定:**

  ```bash
  ros2 param set /<node_name> logger_level.<logger_name> DEBUG # 特定のロガーのレベルを変更
  ```

- **RQt の起動 (ROS 2 GUI ツールスイート):**

  ```bash
  rqt
  ```

  _よく使う RQt プラグイン:_

  - `rqt_graph`: ROS グラフの可視化
  - `rqt_console`: ROS ログメッセージの表示
  - `rqt_topic`: トピックのパブリッシュ/サブスクライブ、メッセージの表示
  - `rqt_plot`: トピックデータのプロット

- **RViz2 の起動 (3D 可視化ツール):**

  ```bash
  rviz2
  ```

- **rosbag の記録と再生:**

  - **記録:**
    ```bash
    ros2 bag record -a           # 全トピックを記録
    ros2 bag record -o my_bag /topic1 /topic2 # 特定のトピックを記録し、出力フォルダ名を指定
    ```
  - **再生:**
    ```bash
    ros2 bag play <bag_file_directory> # 例: ros2 bag play my_bag_0
    ros2 bag play <bag_file_directory> -r 2.0 # 2倍速で再生
    ```

- **ROS 2 イベントの監視:**
  ```bash
  ros2 event list
  ros2 event echo <event_name>
  ```

---

これらのコマンドは ROS 2 開発の基盤となるものです。それぞれのコマンドのオプションや詳細については、`ros2 <command> --help` を実行することで確認できます。例えば、`ros2 topic --help` や `ros2 topic pub --help` などです。
