# Radar
QEA 雷达建图项目。
因为使用Visual GDB编写，推荐使用Visual GDB，下载代码后点击Radar/Radar.sln即可运行，较为方便。

关于如何使用Visual Studio + Visual GDB + STM32Cube MX 编写stm32程序，
请观看以下B站链接https://www.bilibili.com/video/BV1NV411U7Bq/?spm_id_from=333.880.my_history.page.click&vd_source=cd7bff178b79967cd653b46af7fbb681

新增keil版本，现在可下载代码后，直接点击Mount_Doom\MDK-ARM\Mount_Doom.uvproj文件，编译烧录。

使用串口助手接收数据格式如下 [30.0,200,40.0,300...]
其中第一个和第三个参数为雷达探测的角度，第二个和第四个为距离，可根据该数据编写matlab的建图程序。
