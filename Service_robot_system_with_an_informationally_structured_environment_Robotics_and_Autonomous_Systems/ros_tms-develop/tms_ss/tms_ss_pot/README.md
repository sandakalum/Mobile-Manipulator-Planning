# 4台のLRFによるトラッキング

## Run Tracker
```bash
roslaunch tms_ss_pot minimal.launch
```

このとき，tms_ss_pot配下にある`lrf*.yaml`に注意 \
このファイルは背景差分なので，設置位置などが変わった場合はこのファイルを消してから立ち上げる．

## Ex1. Set Parameter
LRFの設置位置や移動体を追跡する範囲を設定する際は,
`config.yaml`内の値を変更.

## Ex2. Laser Calibration
LRFの設置位置を調整するときは,
```bash
roslaunch tms_ss_pot laser_calib.launch
```
で,LaserのMarkerArrayを表示して調整.
