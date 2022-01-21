# tms_rc_rtkbot

## 動かし方
NUCで`navigation.launch`を，
モニタ用PCで`monitor.launch`を立ち上げてください．

また，ネットワーク型RTKを使用する場合は，NUCで<b>BNC</b>を立ち上げてください．(下記参照)

更に，P<sup>2</sup>-Senを利用する場合は，tms_ss_pot内の`minimal.launch`を立ち上げてください．

## 追加で必要なパッケージ
### aptで入手
- ros-kinetic-nmea-msgs
- ros-kinetic-pointcloud-to-laserscan
### GitHubから入手
- ros-kinetic-nmea-navsat-driver

(2020/02/09現在，driver.pyに以下の2つのバグがあるので，使うときは修正してください)

* PublisherのQueue Sizeが1だと動かない可能性が高いので，適当なサイズに変更
* RTK FixとRTK Floatに同じステータス番号(NavSatStatus.STATUS_GBAS_FIX = 2)が割り当てられているので，
区別できるように変更(tms_rc_rtkbotでは，Float=2，Fix=3として扱っています)

### pipで入手
- pyproj

### サイトから入手
- BKG Ntrip Client (BNC)  <- (ネットワーク型RTKを使用する場合)


## シリアルの設定

- シリアルを一般ユーザで使う設定(初めてのときだけで良い)
```
sudo gpasswd -a ユーザ名 dialout
```
後は再ログインすればOK

- /etc/udev/rules.d/99-serial.rulesを作成し，以下の内容を記述
~~~
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", SYMLINK+="nucleo", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="04bb", ATTRS{idProduct}=="0a0e", SYMLINK+="usb_rsaq5", MODE="0666"
~~~

これでマイコンボードは`/dev/nucleo`として，USB-Serialコネクタは`/dev/usb_rsaq5`として扱える．  
(/dev/ttyACM*で数字が変わる問題に悩まされない)
(詳しくは[Qiita](https://qiita.com/caad1229/items/309be550441515e185c0)などを参考に)


## BKG Ntrip Client (BNC)の使い方 (ネットワークRTK使用時)

実行ファイルの名前を`bnc`とすると，基本的には以下で立ち上がります．
~~~
./bnc
~~~
起動後，適宜設定を入力して使ってください．

また，設定はファイルに保存できます．
設定ファイルの名前を`rtk.bnc`とすると，
~~~
./bnc --conf rtk.bnc
~~~
というように，`conf`というオプションを付けることで，設定を読み込んだ状態で立ち上げられます．

また，`nw`というオプションを付けることで，no windowモードで立ち上げられます．
SSH利用時など，コマンドラインで処理を完結したい場合はこれを使って，以下のように立ち上げてください．
~~~
./bnc --conf rtk.bnc --nw
~~~

詳しくは，公式のドキュメントを参照してください．
* [Network RTK](https://software.rtcm-ntrip.org/export/HEAD/ntrip/trunk/BNC/src/bnchelp.html#serial)
* [Command Line](https://software.rtcm-ntrip.org/export/8238/ntrip/trunk/BNC/src/bnchelp.html#confList)