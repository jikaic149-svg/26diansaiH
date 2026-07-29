# 感为（Ganwei）DATA/CLK Device

感为八路数字量灰度循迹的同步串行 Device。当前 G3519 BSP 使用 PB8 DAT、
PB9 CLK；每次采样产生八个时钟，第一个采样位映射为通道 0。GPIO、引脚、时钟波形
和采样延时仍由 BSP 持有。

`Device_Ganwei` 只接受恰好八路数字通道的 `BSP_LineSensor` 后端，并将采样提交给
`Device_LineTracking` 的数字阈值/位图路径。它不调用芯流 T8 使用的
`Device_LineTracking_UpdateAnalogRaw()`，因此不执行模拟量共同基线消除，也不使用
模拟量置信度门；后续控制应接 `Mode_Car_DigitalTracking`。

具体感为型号、供电与 DATA 电平、黑线有效极性仍待实物资料确认。产品 Task
负责选择该 Device、调整极性/阈值/位置权重并按产品周期调用更新；通用模板不提供
产品 Task。
