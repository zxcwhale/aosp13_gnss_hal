# 卫星定位接收机HAL驱动

适用于Android13或以上版本.

## 源代码修改

1. 把工程拷贝到/vendor目录
2. 修改gps_zkw.c文件中的`GPS_CHANNEL_NAME`为接收机的TTY号.
3. 修改gps_zkw.c文件中的`TTY_BAUD`为接收机实际的波特率.

## 其它参数
1. GNSS_STR 
- 0，使用GpsSvStatus结构体和sv_status_cb回调上报卫星状态。
- 1，使用GnssSvStatus结构体和gnss_sv_status_cb回调上报卫星状态。
2. TTY_BOOST
- 0, 不使用该功能
- 1, 波特率等于9600时，自动调整波特率到115200。(可能会出错，慎用)
* 建议关闭该功能 *
3. REDUCE_SV_FREQ
- 0, 关闭该功能
- 1, 在gps_state_start()时，减小卫星语句的输出频率. 
- 2, 在tty负载高时，减小卫星语句的输出频率. 

## 生成HAL层库

运行以下命令(请根据实际情况, 修改路径)

```bash
mmm vendor/gnsshal
```

## 将HAL层库Push到设备

运行以下命令, 并根据平台实际情况, 修改push的目录

```bash
adb root
adb shell mount -o remount,rw /vendor
adb push $ANDROID_PRODUCT_OUT/vendor/lib64/hw/gps.hikey960.so /vendor/lib64/hw/
adb reboot
```

## 可能的问题

1. 如果编译出现找不到`ALOGD`, `ALOGE`的报错, 可以尝试将`ALOGD`改为`LOGD`, `ALOGE`改为`LOGE`.

2. 使用Gnss1.0接口
修改device.mk,添加如下内容
```
# Gnss HAL
PRODUCT_PACKAGES += \
	android.hardware.gnss@1.0 \
	android.hardware.gnss@1.0-impl \
	android.hardware.gnss@1.0-service
```

3. 使用Gnss2.0接口
修改device.mk,添加如下内容
```
# Gnss HAL
PRODUCT_PACKAGES += \
	android.hardware.gnss@2.0 \
	android.hardware.gnss@2.0-impl \
	android.hardware.gnss@2.0-service
```
