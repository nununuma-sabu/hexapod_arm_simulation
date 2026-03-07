#!/bin/bash

# エラー発生時に以降の処理を停止する
set -e

echo "=========================================="
echo " 六脚（ジンバル関節アーム）ロボット 初期設定開始"
echo "=========================================="

# ワークスペースのディレクトリに移動
WORKSPACE_DIR=~/robot
echo "[1/4] ワークスペース ($WORKSPACE_DIR) に移動します..."
cd $WORKSPACE_DIR

echo "[2/4] ROS 2 Humble の環境を読み込みます..."
source /opt/ros/humble/setup.bash

echo "[3/4] 依存関係のインストール (rosdep) を実行します..."
# rosdep が初期化されていない場合は初期化を試みる
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "rosdep が初期化されていないため、sudo rosdep init を実行します。パスワードを求められる場合があります。"
    sudo rosdep init || echo "rosdep init は既に実行されているか処理をスキップしました。"
fi
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo "[4/4] ワークスペースをビルドします (colcon build)..."
colcon build --symlink-install

echo "=========================================="
echo " 初期設定およびビルドが完了しました！"
echo "=========================================="
echo ""
echo "※ 実際のシミュレーションやRViz2を実行する前に、"
echo "   現在のターミナルで以下のコマンドを実行して"
echo "   ワークスペースの環境を読み込んでください："
echo ""
echo "source ~/robot/install/setup.bash"
echo ""
echo "※ もしくは ~/.bashrc に以下を追記すると便利です："
echo 'echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc'
echo 'echo "source ~/robot/install/setup.bash" >> ~/.bashrc'
