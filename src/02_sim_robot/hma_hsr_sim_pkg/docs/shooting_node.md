# 撮影用プログラム
シミュレータを録画するためのプログラム．

# launchファイル
以下を追加．
```
<!-- Shooting -->
<node pkg="hma_hsr_sim_pkg" type="shooting_node.py" name="shooting_node" output="screen" required="true"/>
```


# ステートマシン変更点
変更しているチームは以下を参考に開始前に`/sim_time_supervisor/run_enable`にTrueをPublishするようにする（Startのexecuteを書き換える）．
```
    def execute(
        self,
        userdata
    ):
        # タイマースタート
        raw_input("Start >>> ")
        run_enable = Bool(True)
        self._pub_time_supervisor.publish(run_enable)
        rospy.sleep(1.0)


        return "next"
```

# 採点
ToDo

# 動画の作成の前準備
以下のファイルがあることを確認&作成する
```
- hma_hsr_sim_pkg/io/images/0/*jpg : 画像
- hma_hsr_sim_pkg/io/images/1/*jpg : 画像
- hma_hsr_sim_pkg/io/images/data.csv : 採点データ
- hma_hsr_sim_pkg/io/images/fonts/corpround-ver2-font/Corporate-Logo-Rounded.ttf : コーポレート・ロゴver2のフォントデータ
```

# 動画作成
```
$ cd hma_wrs_sim_ws/src/02_sim_robot/hma_hsr_sim_pkg/io/images
$ python3 ../../script/src/shooting/make_video_py.py
```
# 注意事項
- launchファイルのTimeSupervisorのtimeは10.0にしておくこと．
- Startステートでエンターキー入力待ちをしている．必ずすべての物体が出現してからスタートすること．
    - `Spawn finished`が表示されるまで待ってからエンターを押せばOK．
- `hma_hsr_sim_pkg/io/images/`の`0`および`1`ディレクトリに画像が保存されているのを確認すること（スタート後）．
    - 何らかのバグで画像が保存されていなかったらやり直しになる．
- 撮り直しする場合，`0`および`1`ディレクトリ内の画像は消す（またはバックアップを取っておく）こと．
- 最終的に提出するのは`0`および`1`ディレクトリである．間違って上書きしてしまったらやり直しになるので注意．