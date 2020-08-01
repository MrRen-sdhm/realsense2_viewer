## Realsense D4xx深度校准

#### 1、配置参数调整：

参考官网配置参数调整教程：https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/BKMs_Tuning_RealSense_D4xx_Cam.pdf

1.调整Auto Exposture

​	自动曝光表现不佳时可关闭，改为手动调整曝光参数。

2.调整Laster Power

​	通常调小Power可以减少点云孔洞。

3.调整Disparity Shift

​	调大Shift值可使得相机能够获得更近的深度信息，即减小Min-Z，从而看到更近的物体。

#### 2、双目标定：

参考官网d400系列动态校准教程：https://www.intel.cn/content/www/cn/zh/support/articles/000026723/emerging-technologies/intel-realsense-technology.html

##### DynamicCalibrator安装教程：

- Register the server's public key:

  ```
  sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
  ```

- Add Intel server to the list of repositories:

  ```
  sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
  ```

- Remove the old records:

  ```
  sudo rm -f /etc/apt/sources.list.d/realsense-public.list
  ```

- Refresh the list of repositories and packages available:

  ```
  sudo apt-get update
  ```

- Install the librscalibrationtool package which includes Intel® RealSenseTM Dynamic
  Calibrator:

  ```
  sudo apt-get install librscalibrationtool
  ```

##### DynamicCalibrator使用教程：

安装成功后可执行文件位于/usr/bin/，终端中使用以下命令打开DynamicCalibrator

```
Intel.Realsense.DynamicCalibrator
```

标定完成后，参数会自动写入设备，标定文件会保存在当前文件夹下。

##### CustomRW使用教程：

- 重置出厂参数设置：

```
Intel.Realsense.CustomRW -g
```

- 查看当前设备参数：

```
Intel.Realsense.CustomRW -r
```

- 查看当前设备参数并保存为xml文件：

```
Intel.Realsense.CustomRW -r -f raw.xml
```

- 将xml文件参数写入设备：

```
Intel.Realsense.CustomRW -w -f custom.xml
```

#### 3、标定参数调整：

相机的深度值不准确，使用标定工具效果不佳，尝试手动修改参数。

这里，利用[ArUco_ros](https://github.com/pal-robotics/aruco_ros)测量平面距离，使用[Depth Quality Tool](https://github.com/IntelRealSense/librealsense/tree/master/tools/depth-quality)评估相机深度值质量。即在平面上贴上ArUco标记，进而得到相机到平面的准确距离，使用尺子测量距离可知，ArUco测得的距离可作为相机与平面的准确距离。再使用Realsense的Depth_Quality_Tool查看此平面与相机的距离，未经标定的相机会有深度(距离)误差。

经测试，PrincipalPointLeft， PrincipalPointRight，RotationLeftRight三个参数对深度值的影响最大。

- 例1：

ArUco测得平面距离：400mm，Depth_Quality_Tool显示距离：370mm。

原始参数：

```
<param name = "PrincipalPointLeft">
        <value>635.267</value>
        <value>402.337</value>
</param>

<param name = "PrincipalPointRight">
        <value>630.523</value>
        <value>395.853</value>
</param>

<param name = "RotationLeftRight">
        <value>0.999976</value>
        <value>0.00223872</value>
        <value>-0.00656334</value>
        <value>-0.00224392</value>
        <value>0.999997</value>
        <value>-0.000783978</value>
        <value>0.00656157</value>
        <value>0.000798686</value>
        <value>0.999978</value>
</param>
```

通过减小FocalLengthRight[0]使得深度值向真实值靠近，微调RotationLeftRight[2] 与 RotationLeftRight[6]

```
<param name = "PrincipalPointLeft">
        <value>635.267</value>
        <value>402.337</value>
</param>
    
<param name = "PrincipalPointRight">
        <value>630.560</value> <!-- - -->
        <value>395.853</value>
</param>

<param name = "RotationLeftRight">
        <value>0.999982</value>
        <value>0.00223913</value>
        <value>-0.00656338</value> <!-- - -->
        <value>-0.00224353</value>
        <value>0.999997</value>
        <value>-0.000785102</value>
        <value>0.00656161</value> <!-- + -->
        <value>0.000797569</value>
        <value>0.999984</value>
</param>
```
- 例2：

ArUco测得平面距离：400mm，Depth_Quality_Tool显示距离：372mm。

原始参数：

```
<param name = "PrincipalPointLeft">
        <value>632.971</value>
        <value>412.754</value>
</param>

<param name = "PrincipalPointRight">
        <value>640.326</value>
        <value>410.301</value>
</param>

<param name = "RotationLeftRight">
        <value>0.999986</value>
        <value>-0.00013191</value>
        <value>-0.0053077</value>
        <value>0.000138159</value>
        <value>0.999999</value>
        <value>0.00117693</value>
        <value>0.00530754</value>
        <value>-0.00117764</value>
        <value>0.999985</value>
</param>
```

观察发现PrincipalPointLeft[1]与PrincipalPointRight[1]相差较大，将PrincipalPointLeft[1]调整为与PrincipalPointRight[1]相同后，发现深度质量有所改善，Depth_Quality_Tool显示距离：391mm。

经测试RotationLeftRight[2] 与 RotationLeftRight[6] 大小接近，同时增大可使深度值向400mm靠近，但增大过多会导致点云畸变，即平面发生弯曲，中心区域点云突出，因而不能通过调整此值来改善深度质量。

最终，修改PrincipalPointLeft[1]与PrincipalPointRight[1]相同，再微调RotationLeftRight[2] 与 RotationLeftRight[6]，从而保证深度质量。修改后，参数如下：

```
<param name = "PrincipalPointLeft">
        <value>640.326</value> <!-- + -->
        <value>412.754</value>
</param>

<param name = "PrincipalPointRight">
        <value>640.326</value>
        <value>410.301</value>
</param>

<param name = "RotationLeftRight">
        <value>0.999986</value>
        <value>-0.00017191</value>
        <value>-0.0061000</value> <!-- - -->
        <value>0.000138159</value>
        <value>0.999999</value>
        <value>0.00117693</value>
        <value>0.00610000</value> <!-- + -->
        <value>-0.00117764</value>
        <value>0.999985</value>
</param>
```
