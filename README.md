# Smart-Bracelet
基于stm32f407的蓝牙运动手环系统。

引脚连接：</br>
OLED_GNDPDout(15)D1---GND---PD15</br>
OLED_VCCPDout(1)D3---VCC---PD1</br>
OLED_SCKPEout(8)D5---SCK---PE8</br>
OLED_SDA_OUTPEout(10)D7---SDA---PE10</br>
OLED_SDA_INPEin(10)</br>
Usart1</br>
蓝牙RXPA9(TX)</br>
蓝牙TXPA10(RX)</br>
MAX30102_GNDPDout(0)D2---GND---PD0</br>
MAX30102_VCCPEout(11)D4---SCK---PE7</br>
MAX30102_SCKPEout(7)D6---SDA---PE9</br>
MAX30102_SDA_OUTPEout(9)D8---VCC---PE11</br>
MAX30102_SDA_INPEin(9)</br>
启动后等待界面加载完成即可使用</br>
