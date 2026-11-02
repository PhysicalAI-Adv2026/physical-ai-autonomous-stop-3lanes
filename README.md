# AIによる接近判断と自動停止を行うロボットシミュレーション

Physical AI 応用編1・最終課題。ROS 2 Humble / turtlesimで3レーンの亀型ロボットをA→B→Cの順に走らせ、決定木AIの推論結果を速度指令に反映して自動停止する小規模な実装です。

## 実装のしくみ

- `turtlesim` が出す `/turtleN/pose` から各亀のX座標を取得します。
- 仮想距離 `max(0, 8.5 - x)` を1入力として `DecisionTreeClassifier(max_depth=1)` で `MOVE` / `STOP` を推論します。
- 教師データは距離0.00〜5.00の101例で、1.0以下をSTOPとした合成データです。学習した境界値は1.025です。
- `MOVE` ではROS 2トピック `/turtleN/cmd_vel` へ速度0.17、`STOP` では速度0.0を送信します。
- 色付きの線は前もって描かれる**表示用ガイド**であり、AIが追跡・画像認識しているわけではありません。短いオレンジ色の線はX=8.5の仮想バリア位置です。
- 実世界の障害物、距離センサー、物理的な衝突検知は用いていません。

## 動作環境（検証時）

- Ubuntu 22.04.5 LTS（ARM64版Dockerコンテナ）
- ROS 2 Humble / `turtlesim` / `rclpy`
- Python 3.10.12
- scikit-learn 0.23.2 / NumPy 1.21.5 / SciPy 1.8.0

検証環境では `tiryoh/ros2-desktop-vnc:humble-20261018T0849` のイメージを使用しました。Ubuntu配布のNumPy 1.21.5を優先するため、スクリプトは `/usr/lib/python3/dist-packages` を先頭に追加します。NumPy 2.xと古いSciPyの混在環境には注意してください。

## 実行手順

1. `turtlesim` がインストールされたROS 2環境を用意します。
2. 1つ目のターミナルで実行し、turtlesimウィンドウを開きます。

   ```bash
   source /opt/ros/humble/setup.bash
   ros2 run turtlesim turtlesim_node
   ```

3. 2つ目のターミナルで、Pythonファイルのあるディレクトリから実行します。

   ```bash
   source /opt/ros/humble/setup.bash
   python3 pai_ai_stop_3lanes_guides.py --self-test
   python3 pai_ai_stop_3lanes_guides.py --speed 0.17
   ```

4. 画面左側に亀3匹がA→B→Cの順に移動し、右側のターミナルに `SUCCESS` が3行表示されることを確認します。

**注意：** 実行時にturtlesim内の描画が消去・再作成され、以前の実験で作成した所定名の仮想マーカー亀が削除される場合があります。共有のROS環境ではなく専用のturtlesimで実行してください。ログはコンテナの `/workspace/logs` に出力されます。`/workspace` をMac側にマウントしておけば保存できます。

## 検証データ

`results/` にCSV原本3件と集計表が入っています。停止距離は最初にAIがSTOPを出した瞬間の測定値です。

| 試行 | 開始X | レーンY | 動作時間 | 停止距離 | 結果 |
|---|---:|---:|---:|---:|---|
| A | 2.0 | 8.40 | 32.218秒 | 1.0246 | SUCCESS |
| B | 3.5 | 5.55 | 23.473秒 | 1.0234 | SUCCESS |
| C | 5.0 | 2.70 | 14.563秒 | 1.0248 | SUCCESS |

3件の動作時間合計は70.254秒、平均停止距離は1.0243シミュレータ単位です。3件の成功は特定の固定条件でのみ確認したもので、未知の障害物や実機に一般化できるという意味ではありません。

## 収録動画・PDF

提出用MP4（1分42秒）と最終課題レポート（2ページ）はomnicampus提出ZIPに含める構成です。GitHubでの動画公開は必須ではありません。

## その他

講義配布資料や第三者のスライドを転載せず、オリジナルの制御コード・録画・実験データを提出しています。実装と文書の整理にはAIによるコーディング支援を利用し、ROS 2実機環境での動作と実測データは利用者が検証しました。

このリポジトリにLICENSEファイルは現時点で付けていません。公開に際してライセンスを選ぶ場合は、コードの取り扱いを確認してから付与してください。
