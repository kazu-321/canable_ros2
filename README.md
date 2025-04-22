# CANable ROS2

## Setup
[公式サイト](https://canable.io/updater/)でファームウェアを更新できます

CANableデバイスのバージョンを選択し、candlelightを書き込んでください

CANalbe 2.0の場合はボタンを押しながらUSBを再接続することで書き込めるようになります

ファームの書き込みが終わったら、

```bash
sudo dmesg
```
でVendorやProduct、Serialを取得
```bash
[43777.014514] usb 9-1: new full-speed USB device number 9 using xhci_hcd
[43777.168286] usb 9-1: New USB device found, idVendor= {Vendor} , idProduct= {Product} , bcdDevice= 0.00
[43777.168296] usb 9-1: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[43777.168300] usb 9-1: Product: canable2 gs_usb
[43777.168304] usb 9-1: Manufacturer: canable.io
[43777.168308] usb 9-1: SerialNumber: {Serial}
```

sudo権限で `/etc/udev/rules.d/99-canable.rules` を作成し以下を記入
```bash
SUBSYSTEM=="usb", ATTR{idVendor}=="Vendor", ATTR{idProduct}=="Product", ATTR{serial}=="Serial", SYMLINK+="can", \
RUN+="/sbin/ip link set can0 type can bitrate 1000000", \
RUN+="/sbin/ip link set can0 up"
```

上記udev rulesで1Mbpsでcan0として自動設定します

適用するには以下コマンドを実行後CANableを再接続します

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```