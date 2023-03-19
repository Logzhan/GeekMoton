## Peak电路原理分析

<img src="C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images\image-20230319230228087.png" alt="image-20230319230228087" style="zoom:50%;" />

### 一、开机电路分析

​		用户在按下EC_KEY之后，EC_KEY会被拉低电平，此时Q1的栅极会被拉低电平 。此时Q1导通，单片机的系统进入有电状态，ESP32启动。此时有两种情况。1）用户立即放下按钮：此时PWR_EN还没及时拉高，Q2这个NMOS管始终处于不导通状态。Q1会处于不导通状态，系统关机。 2）用户持续按下EC_KEY此时：单片机持续运行，PWR_EN被持续拉高，Q2始终会处于导通状态，Q1的栅极会被拉低，系统始终保持有电状态。