# PRIMO4 MP



## はじめに

promo4用のMP（FCX-2基板）のCortex-M7用の開発環境です。

下記のbuild 環境で適当な作業ディレクトリを用意します。

ここのgitをcloneします。

```bash
git clone https://www.airbg.sony.co.jp/gitlab/XP024584/primo4_mp
```

build toolsをinstallしてPATHをとおしてセットアップします。



## Docker 環境

docker上にprimo4_buildのイメージがある場合

```bash
make clean
make
```

で``primo4.elf, primo4.bin``が生成されます。



## local build環境

多分Windowsの場合、WSL2上でやるのがいい気がする

Linux, OS-Xの場合はそのままでも大丈夫。



### build tools

- Docker環境にはsetup済み
- localでbuildしたい場合のtools
- GNU Make 3.81　多分あんまりVerは関係ない
- xPack GNU Arm Embedded GCC　Ver.13.2.1
  - https://xpack.github.io/tags/arm-none-eabi-gcc/
  - 古いVerだとLinkerでエラーが出る
- ST-LINK Ver.1.8.0
  - https://github.com/stlink-org/stlink/releases/tag/v1.8.0
  - linux, OS-Xの場合はapt, brewでinstallするのが楽




### buildの仕方

``cd ./primo4_mp`` でprimo4_mpのtop directoryに入って

``make``　でbuild/primo4.elfが生成できます。

FCX-2基板にST-Linkを接続して電源を入れ

``make install``で書き込みできます。







