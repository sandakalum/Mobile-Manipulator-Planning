# 実行手順
以下のコマンドを実行

`roscore`    
`roslaunch tms_db_manager tms_db_manager.launch`   (`double_control.cpp` において、viconから位置を直接取得すれば不要)

加えて、以下のどちらかを実行(`double_controll.cpp`を使うセンサに合わせて要編集)

`rosrun tms_ss_vicon vicon_stream`
`rosrun tms_rc_double tms_ss_double_pozyx`



### 注意
- /tms_rc_double/room○○○/cmd_vel　を発行することでDoubleが動く  
- 事前にDouble上のアプリケーションのROSMASTER_URIを，roscoreが起動しているPCのIPアドレスにしておく
- Double上のアプリはビルド後、１週間程度経過すると起動不可となるためその場合は再度ビルドしておく(その際に各部屋の設定に合わせる)