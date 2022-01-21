座標設定

- visualize.pyとdebug_anchor_visualize.pyはconfig_anchors.yamlを参照している
- localize.inoはソースに直接書き込む
(単位はどちらもメートル)

ninebot navigationの動かし方
```
ssh odroid@192.168.12.22
sudo bash
date --set "2018/01/12 08:40:00"
roslaunch tms_rc_bot ninebot_navigation.launch
```