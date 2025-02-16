＊STM32 USB-MIDI

bluepill等のSTM32F103C8T6搭載ボード用の3入力3出力対応USB-MIDIインターフェースファームウェアです。
STM32F103C8T6や中国製コピーのCKS32F103C8T6等と8MHzの水晶を搭載したボードであればどれでも動作すると思います。
とても簡単なものなのでソースを修正すればそれ以外のボードでも動作するはずです。

＊ビルド方法

make -C libopencm3
make

＊ファームウェア書き込み方法

usbmidi.binをお気に入りの方法でボードに書き込んでください。

書き込みツールの例
stm32flash
https://sourceforge.net/p/stm32flash/wiki/Home/
stlink
https://github.com/stlink-org/stlink/

＊MIDI接続回路例

STM32      << MIDI  OUT >>       MIDI
3V3-------------[33R]------------DIN4
OUT-------------[10R]------------DIN5
GND------------------------------DIN2

MIDI       <<< MIDI IN >>>      STM32
               +-----+
DIN4--[220R]--1|     |6-----------3V3
               |     |5-----------IN
DIN5----------3|     |4-----------GND
               +-----+
               TLP2361

上記のOUT/INは以下のピンに置換えて下さい
MIDI1 OUT=PB6 IN=PB7
MIDI2 OUT=PA2 IN=PA3
MIDI3 OUT=PB10 IN=PB11

注: TLP2361の6ピンと4ピンの間にバイパス用のコンデンサ0.1uFを付けて下さい。
注: 基本的に不要ですが必要に応じてTLP2361の1ピンと3ピンの間に逆接続保護用ダイオード(1N4148等 1側カソード 3側アノード)付けて下さい。

TLP2361入手先
https://akizukidenshi.com/catalog/g/g111004/
