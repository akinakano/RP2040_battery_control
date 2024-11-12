# PRIMO4 MP



## はじめに

promo4用のMP（FCX-2基板）のCortex-M7用の開発環境です。

下記のbuild 環境で適当な作業ディレクトリを用意します。

ここのgitをcloneします。

```bash
git clone https://www.airbg.sony.co.jp/gitlab/XP024584/primo4_mp
```

build toolsをinstallしてPATHをとおしてセットアップします。



## build環境

多分Windowsの場合、WSL2上でやるのがいい気がする

Linux, OS-Xの場合はそのままで大丈夫。

Dockerに移行した方がいいかも。。。そのうち。。。。



## build tools

- GNU Make 3.81　多分あんまりVerは関係ない
- xPack GNU Arm Embedded GCC　Ver.13.2.1
  - https://xpack.github.io/tags/arm-none-eabi-gcc/
  - 古いVerだとLinkerでエラーが出る



## buildの仕方

``cd ./primo4_mp`` でprimo4_mpのtop directoryに入って

``make``　でCM7/Release/primo4_CM7.elfが生成できます。

FCX-2基板にST-Linkを接続して電源を入れ

``make install``で書き込みできます。







