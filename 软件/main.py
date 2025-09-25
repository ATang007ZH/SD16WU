# encoding:utf-8
"""
@ CNRobot V2.0
"""

# https://github.com/wangqi1994/robot/blob/master/yuanhongwai.py
import serial
import time
import numpy as np
import cv2

"""--------------------
串口类定义
--------------------"""
#\Users\zh187    PS C:\Users\zh187> pip install pyserial

class comSerial:
    # 构造串口类的属性
    def __init__(self, portx, bps, waitTime):
        self.serialPort = portx  # 端口号
        self.baudRate = bps  # 波特率
        self.timeOut = waitTime  # 超时等待时间
        self.serial = None

    # 启动串口
    def Start(self):
        global receivdata
        try:
            self.serial = serial.Serial(self.serialPort, self.baudRate, timeout=self.timeOut)
            # print(portx, "is start success...\n")
        except Exception as e:
            print("--ERROR---\n")

    # 关闭串口
    def End(self):
        self.serial.close()

    # 写数据
    def WriteData(self, str):
        # 将指令字符串转化成16进制编码
        data = bytes.fromhex(str)
        # 将指令写入串口
        self.serial.write(data)

    # 读数据
    def ReadData(self):
        global imgBuffer, imgData, receivdata
        data = bytes()

        if self.serial.inWaiting() >= 80000:  # 串口数据量大于两帧图像
            print(self.serial.inWaiting())
            startTime = time.clock()
            receivdata.data = self.serial.read(80000)  # 规定一次读取两帧图像
            receivdata.length = len(receivdata.data)

            endTime = time.clock()
            print("串口读取数据花费%fs" % (endTime - startTime))
            startTime = time.clock()
            PacketAnalysis()
            # AutoGain()
            imgData = np.array(imgBuffer, dtype=np.uint8)
            endTime = time.clock()

            print("解析图像数据花费%fs" % (endTime - startTime))
        # timer = threading.Timer(0.01, ser.ReadData())           # 重新启动定时器
        # timer.start()


"""--------------------
SD16B数据类定义
--------------------"""


class SD16B_Data:
    def __init__(self):
        self.frameStart = 0x0000  # 帧开始
        self.status = 0x00  # 传输状态
        self.command = 0x00  # 指令
        self.dataLength = 0  # 数据长度
        self.crc1 = 0x0000  # 校验
        self.data = [0] * 65535  # 数据
        self.crc2 = 0x0000  # 校验
        self.frameEnd = 0x0000  # 帧结束


"""--------------------
串口接收数据类定义
--------------------"""


class seriaPortReceive:
    def __init__(self):
        self.data = bytes()
        self.length = 0


"""--------------------
传感器分辨率类定义
--------------------"""


class Sensor:
    def __init__(self):
        self.width = 160
        self.high = 120


"""--------------------
图像自动拉伸参数类定义
--------------------"""


class AUTOGAIN:
    def __init__(self):
        self.ThresholdLimits = 0
        self.difThreshold = 0
        self.lowThreshold = 0
        self.gain = 0.0
        self.aveGraygradation = 0.0


"""--------------------
坐标点类定义
--------------------"""


class Point:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.data = 0


"""--------------------
全局变量定义
--------------------"""
# SD16B 指令定义
SD16B_START_SIGNAL = 'AA 55 00 00 00 01 4F 7E 00 00 00 0D 0A'  #SD16B 启动信号
SD16B_END_SIGNAL = 'AA 55 00 00 00 01 4F 7E FF 1E F0 0D 0A'    #SD16B 结束信号
SD16B_GRAY_SIGNAL = 'AA 55 00 09 00 01 D1 EF FF 1E F0 0D 0A'   #SD16B 灰度信号
SD16B_TEMP_SIGNAL = 'AA 55 00 09 00 01 D1 EF 00 00 00 0D 0A'   #SD16B 温度信号

STRGLO = ''  # 读取的数据
BOOL = True  # 读取标志位

# 数据状态
STARTAA = 0
START55 = 1
STATUS = 2
COMMOND = 3
DATALENGTH = 4
CRC1 = 5
DATA = 6
CRC2 = 7
END0D = 8
END0A = 9

SD16B_data = SD16B_Data()  # SD16B数据
receivdata = seriaPortReceive()  # 从串口接收到的数据
sensor = Sensor()  # 传感器尺寸
imgBuffer = [[0 for i in range(160)] for j in range(120)]  # 用于转移图像数据
tempData = np.zeros((120, 160), dtype=np.uint16)  # 用于存储像素点温度，去除负温
imgData = np.zeros((120, 160), dtype=np.uint8)  # 图像数据
imgBinary = np.copy(imgData)  # 二值化图像数据
usart = serial.Serial()  # 串口
tempMax = Point()  # 最高温度
tempMin = Point()  # 最低温度
tempCtr = Point()  # 最低温度

"""------全局变量定义结束-----"""


#
# 往串口写数据
#
def usartWriteData(str):
    global usart

    # 将指令字符串转化成16进制编码
    data = bytes.fromhex(str)
    usart.write(data)


#
# 从串口读数据
#
def usartReadData():
    global usart
    data = bytes()
    if usart.inWaiting() >= 45000:  # 缓冲区数据大于一帧图像数据
        receivdata.data = usart.read(usart.inWaiting())  # 读出串口数据
        receivdata.length = len(receivdata.data)  # 数据长度
        return 1
    else:
        return 0


#
# 数据包处理
#
def PacketAnalysis():
    global SD16B_data, receivdata, tempData
    tranStatus = 0  # 传输状态
    length = 0
    index = 0

    # 处理过程
    for index in range(receivdata.length):
        # 起始字节1
        if tranStatus == STARTAA:
            if receivdata.data[index] == 0xAA:
                SD16B_data.frameStart = 0xAA00
                length = 0
                tranStatus = START55

        # 起始字节2
        elif tranStatus == START55:
            if receivdata.data[index] == 0x55 and length == 1:
                SD16B_data.frameStart |= 0x55
                tranStatus = STATUS
            else:
                tranStatus = STARTAA

        # 状态字节
        elif tranStatus == STATUS:
            if receivdata.data[index] == 0x00 and length == 2:
                SD16B_data.status = 0x00
                tranStatus = COMMOND
            else:
                tranStatus = STARTAA

        # 指令字节
        elif tranStatus == COMMOND:
            if length == 3:
                SD16B_data.command = receivdata.data[index]
                tranStatus = DATALENGTH
            else:
                tranStatus = STARTAA

        # 数据长度字节
        elif tranStatus == DATALENGTH:
            if length == 4:
                SD16B_data.dataLength = receivdata.data[index] << 8
            elif length == 5:
                SD16B_data.dataLength |= receivdata.data[index]
                tranStatus = CRC1
            else:
                tranStatus = STARTAA

        # CRC1字节
        elif tranStatus == CRC1:
            if length == 6:
                crc1 = 0xFFFF
                SD16B_data.crc1 = receivdata.data[index] << 8
            elif length == 7:
                SD16B_data.crc1 |= receivdata.data[index]
                tranStatus = DATA
            else:
                tranStatus = STARTAA

        # 数据字节
        elif tranStatus == DATA:
            if length < SD16B_data.dataLength + 8:
                SD16B_data.data[length - 8] = receivdata.data[index]  # 容易越界！！！
            if length == SD16B_data.dataLength + 7:  # 越界是这里的问题？？？
                tranStatus = CRC2
                # print(7)

        # CRC2字节
        elif tranStatus == CRC2:
            if length == SD16B_data.dataLength + 8:
                crc2 = 0xFFFF
                SD16B_data.crc2 = receivdata.data[index] << 8
            elif length == SD16B_data.dataLength + 9:
                SD16B_data.crc2 |= receivdata.data[index]
                tranStatus = END0D
                # print(8)
            else:
                tranStatus = STARTAA

        # 结束字节
        elif tranStatus == END0D:
            if length == SD16B_data.dataLength + 10 and receivdata.data[index] == 0x0D:
                SD16B_data.frameEnd = 0x0D00
                tranStatus = END0A
                # print(9)
            else:
                tranStatus = STARTAA

        # 结束字节
        elif tranStatus == END0A:
            if length == SD16B_data.dataLength + 11 and receivdata.data[index] == 0x0A:
                SD16B_data.frameEnd |= 0x0A
                tranStatus = STARTAA
                length = 0
                # print("OK!!!")
                DataAnalysis()  # 数据分析
            else:
                length = 0
                tranStatus = STARTAA

        # else
        else:
            tranStatus = STARTAA
            length = 0

        length += 1


#
# 数据分析
#
def DataAnalysis():
    global tempMax, tempMin, imgBuffer, SD16B_data, sensor, tempData, lineFlag

    # 起始信号
    if SD16B_data.command == 0x00:
        pass

    # 分辨率信号
    elif SD16B_data.command == 0x01:
        pass

    # 灰度图像信号
    elif SD16B_data.command == 0x24:
        # print("123")
        lineIndex = SD16B_data.data[0]  # 解析行号
        # print("lineIndex:",lineIndex)
        cnt = 0
        if 0 <= lineIndex <= 119:  # 行号正常
            if SD16B_data.dataLength == 321:  # 这一行数据长度正常
                for i in range(1, SD16B_data.dataLength):
                    if i & 0x01:  # 奇数，高位字节
                        imgBuffer[lineIndex][cnt] = SD16B_data.data[i] << 8
                    else:  # 偶数，高位字节
                        imgBuffer[lineIndex][cnt] = imgBuffer[lineIndex][cnt] + SD16B_data.data[i]
                        # tempData[lineIndex][cnt] = int(imgBuffer[lineIndex][cnt] / 10 - 273)  # 记录像素点灰度
                        tempData[lineIndex][cnt] = (imgBuffer[lineIndex][cnt] / 10 - 273)
                        cnt += 1

    # 温度图像信号
    elif SD16B_data.command == 0x25:
        lineIndex = SD16B_data.data[0]  # 解析行号
        # print("lineIndex:",lineIndex)
        cnt = 0
        if 0 <= lineIndex <= 119:  # 行号正常
            if SD16B_data.dataLength == 321:  # 这一行数据长度正常
                for i in range(1, SD16B_data.dataLength):
                    if i & 0x01:  # 奇数，高位字节
                        imgBuffer[lineIndex][cnt] = SD16B_data.data[i] << 8
                    else:  # 偶数，高位字节
                        imgBuffer[lineIndex][cnt] += SD16B_data.data[i]
                        temp = (imgBuffer[lineIndex][cnt] / 10 - 273)  # 计算温度
                        if temp < 0:  # 滤除负温
                            temp = 0
                        tempData[lineIndex][cnt] = temp
                        cnt += 1

    # 特定点温度信息：温度最高点坐标及温度值，温度最低点坐标及温度值，中心点坐标及温度值，任意点温度值及坐标
    elif SD16B_data.command == 0x0E:
        tempMax.x = SD16B_data.data[0]  # 0：最高温度横坐标
        tempMax.y = SD16B_data.data[1]  # 1：最高温度纵坐标
        tempMax.data = SD16B_data.data[2] << 8  # 2~3：最高温度数值
        tempMax.data |= SD16B_data.data[3]
        tempMax.data = int(tempMax.data / 10 - 100)  # 转化成摄氏度

        tempMin.x = SD16B_data.data[8]  # 8：最高温度横坐标
        tempMin.y = SD16B_data.data[9]  # 9：最高温度纵坐标
        tempMin.data = SD16B_data.data[10] << 8  # 10~11：最高温度数值
        tempMin.data |= SD16B_data.data[11]
        tempMin.data = int(tempMin.data / 10 - 100)  # 转化成摄氏度

        tempCtr.x = SD16B_data.data[4]  # 8：最高温度横坐标
        tempCtr.y = SD16B_data.data[5]  # 9：最高温度纵坐标
        tempCtr.data = SD16B_data.data[6] << 8  # 10~11：最高温度数值
        tempCtr.data |= SD16B_data.data[7]
        tempCtr.data = int(tempCtr.data / 10 - 100)  # 转化成摄氏度


#
# 自动增益
#
def AutoGain():
    global sensor, imgBuffer, imgData
    autoGrainParam = AUTOGAIN()
    gray = 0
    maxValue = 16383
    Vmax = maxValue
    Vmin = 0
    imgHist = [0] * (maxValue + 1)  # 图像直方图

    for row in range(sensor.high):
        for col in range(sensor.width):
            gray = imgBuffer[row][col]
            if gray > maxValue:  # 限幅
                gray = maxValue
            if gray < 0:
                gray = 0
            imgHist[int(gray)] += 1
            autoGrainParam.aveGraygradation += gray

    autoGrainParam.aveGraygradation = autoGrainParam.aveGraygradation / (sensor.width * sensor.high)  # 灰度平均值

    lowerLimit = 0
    heightLimit = 0

    for i in range(maxValue):
        if lowerLimit < 200:
            lowerLimit += imgHist[i]
            Vmin = i
        if heightLimit < sensor.high * sensor.width - 100:
            heightLimit += imgHist[i]
            Vmax = i

    autoGrainParam.gain = 11  # 测温
    autoGrainParam.ThresholdLimits = autoGrainParam.gain * 14

    if Vmax > maxValue:
        Vmax = maxValue
    if Vmax < maxValue - (autoGrainParam.ThresholdLimits / 12):
        Vmax += (autoGrainParam.ThresholdLimits / 12)

    # 以直方图峰值为基准划分边界
    autoGrainParam.difThreshold = Vmax - Vmin + autoGrainParam.ThresholdLimits / 6
    autoGrainParam.lowThreshold = autoGrainParam.aveGraygradation - (autoGrainParam.difThreshold / 2)
    autoGrainParam.difThreshold = Vmax - autoGrainParam.lowThreshold

    if autoGrainParam.difThreshold < autoGrainParam.ThresholdLimits:  # 限制最小拉伸范围，不能无限拉伸
        autoGrainParam.difThreshold = autoGrainParam.ThresholdLimits
        if autoGrainParam.lowThreshold < 0:
            autoGrainParam.lowThreshold = 0
        elif autoGrainParam.lowThreshold > (maxValue - autoGrainParam.ThresholdLimits):
            autoGrainParam.lowThreshold = (maxValue - autoGrainParam.ThresholdLimits)
        autoGrainParam.lowThreshold = autoGrainParam.aveGraygradation - (autoGrainParam.difThreshold / 2)
        if Vmax > (autoGrainParam.lowThreshold + autoGrainParam.ThresholdLimits):
            autoGrainParam.difThreshold = Vmax - autoGrainParam.lowThreshold

    for j in range(sensor.high):
        for i in range(sensor.width):
            if imgBuffer[j][i] < autoGrainParam.lowThreshold:
                imgBuffer[j][i] = autoGrainParam.lowThreshold

            imgBuffer[j][i] = (imgBuffer[j][
                                   i] - autoGrainParam.lowThreshold) * maxValue / autoGrainParam.difThreshold  # 拉伸

            if imgBuffer[j][i] > maxValue:
                imgBuffer[j][i] = maxValue


#
# 转移图像
#
def imgTransfer():
    global sensor
    for i in range(sensor.width):
        for j in range(sensor.high):
            tempValue = imgBuffer[j][i] * 1.0 / 16384 * 255  # 将14位数据转化为无符号8位数据

            if tempValue > 255:  # 限幅
                tempValue = 255
            if tempValue < 0:
                tempValue = 0

            imgData[j][i] = tempValue

    # cv2.medianBlur(imgData, 3, imgData)                # 中值滤波，去除噪声
    cv2.GaussianBlur(imgData, (3, 3), 5, imgData, 5)  # 高斯平滑图像信息
    # cv2.GaussianBlur(tempData, (3, 3), 5, tempData, 5)  # 高斯平滑温度信息
    # cv2.medianBlur(tempData, 3, tempData)  # 中值滤波，去除噪声


#
# 显示图像
#
def ImgDisplay():
    global imgData

    cv2.namedWindow('img', 0)
    cv2.resizeWindow('img', 320, 240)
    cv2.namedWindow('img2', 0)
    cv2.resizeWindow('img2', 320, 240)
    # 红蓝色卡
    # 最高温度
    heatmap = cv2.applyColorMap(imgData, cv2.COLORMAP_JET)  # 将灰度图转化为热力图
    heatmap = cv2.line(heatmap, (tempMax.x - 4, tempMax.y), (tempMax.x - 8, tempMax.y), (255, 255, 255), 1)
    heatmap = cv2.line(heatmap, (tempMax.x + 4, tempMax.y), (tempMax.x + 8, tempMax.y), (255, 255, 255), 1)
    heatmap = cv2.line(heatmap, (tempMax.x, tempMax.y - 4), (tempMax.x, tempMax.y - 8), (255, 255, 255), 1)
    heatmap = cv2.line(heatmap, (tempMax.x, tempMax.y + 4), (tempMax.x, tempMax.y + 8), (255, 255, 255), 1)
    cv2.putText(heatmap, str(tempMax.data), (tempMax.x + 5, tempMax.y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                (255, 255, 255), 1)

    # 最低温度
    heatmap = cv2.line(heatmap, (tempMin.x - 4, tempMin.y), (tempMin.x - 8, tempMin.y), (255, 255, 255), 1)
    heatmap = cv2.line(heatmap, (tempMin.x + 4, tempMin.y), (tempMin.x + 8, tempMin.y), (255, 255, 255), 1)
    heatmap = cv2.line(heatmap, (tempMin.x, tempMin.y - 4), (tempMin.x, tempMin.y - 8), (255, 255, 255), 1)
    heatmap = cv2.line(heatmap, (tempMin.x, tempMin.y + 4), (tempMin.x, tempMin.y + 8), (255, 255, 255), 1)
    cv2.putText(heatmap, str(tempMin.data), (tempMin.x + 5, tempMin.y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                (255, 255, 255), 1)

    # 中心温度
    heatmap = cv2.line(heatmap, (tempCtr.x - 4, tempCtr.y), (tempCtr.x - 8, tempCtr.y), (255, 255, 255), 1)
    heatmap = cv2.line(heatmap, (tempCtr.x + 4, tempCtr.y), (tempCtr.x + 8, tempCtr.y), (255, 255, 255), 1)
    heatmap = cv2.line(heatmap, (tempCtr.x, tempCtr.y - 4), (tempCtr.x, tempCtr.y - 8), (255, 255, 255), 1)
    heatmap = cv2.line(heatmap, (tempCtr.x, tempCtr.y + 4), (tempCtr.x, tempCtr.y + 8), (255, 255, 255), 1)
    cv2.putText(heatmap, str(tempCtr.data), (tempCtr.x + 5, tempCtr.y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                (255, 255, 255), 1)

    # 灰度色卡
    # 显示最高温度
    imgData = cv2.line(imgData, (tempMax.x - 4, tempMax.y), (tempMax.x - 8, tempMax.y), (255, 255, 255), 1)
    imgData = cv2.line(imgData, (tempMax.x + 4, tempMax.y), (tempMax.x + 8, tempMax.y), (255, 255, 255), 1)
    imgData = cv2.line(imgData, (tempMax.x, tempMax.y - 4), (tempMax.x, tempMax.y - 8), (255, 255, 255), 1)
    imgData = cv2.line(imgData, (tempMax.x, tempMax.y + 4), (tempMax.x, tempMax.y + 8), (255, 255, 255), 1)
    cv2.putText(imgData, str(tempMax.data), (tempMax.x + 5, tempMax.y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                (255, 255, 255), 1)

    # 最低温度
    imgData = cv2.line(imgData, (tempMin.x - 4, tempMin.y), (tempMin.x - 8, tempMin.y), (255, 255, 255), 1)
    imgData = cv2.line(imgData, (tempMin.x + 4, tempMin.y), (tempMin.x + 8, tempMin.y), (255, 255, 255), 1)
    imgData = cv2.line(imgData, (tempMin.x, tempMin.y - 4), (tempMin.x, tempMin.y - 8), (255, 255, 255), 1)
    imgData = cv2.line(imgData, (tempMin.x, tempMin.y + 4), (tempMin.x, tempMin.y + 8), (255, 255, 255), 1)
    cv2.putText(imgData, str(tempMin.data), (tempMin.x + 5, tempMin.y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                (255, 255, 255), 1)

    # 中心温度
    imgData = cv2.line(imgData, (tempCtr.x - 4, tempCtr.y), (tempCtr.x - 8, tempCtr.y), (255, 255, 255), 1)
    imgData = cv2.line(imgData, (tempCtr.x + 4, tempCtr.y), (tempCtr.x + 8, tempCtr.y), (255, 255, 255), 1)
    imgData = cv2.line(imgData, (tempCtr.x, tempCtr.y - 4), (tempCtr.x, tempCtr.y - 8), (255, 255, 255), 1)
    imgData = cv2.line(imgData, (tempCtr.x, tempCtr.y + 4), (tempCtr.x, tempCtr.y + 8), (255, 255, 255), 1)
    cv2.putText(imgData, str(tempCtr.data), (tempCtr.x + 5, tempCtr.y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                (255, 255, 255), 1)

    cv2.imshow('img', imgData)
    cv2.imshow('img2', heatmap)

    cv2.waitKey(1)
 

def main():
    usart.port = "COM10"  # 设置端口号
    usart.baudrate = 8000000  # 波特率
    usart.timeout = 1  # 超时等待时间
    usart.open()  # 开启串口
    usart.set_buffer_size(rx_size=150000)  # 设置串口接收缓冲区
    usart.flushInput()  # 清空串口接收缓冲区
    usartWriteData(SD16B_START_SIGNAL)  # 通知模组开始传输数据
    time.sleep(1)  # 延时1s
    usart.flushInput()  # 清空串口接收缓冲区
    usartWriteData(SD16B_GRAY_SIGNAL)  # 通知模组回传温度数据

    while 1:
        if usartReadData() == 1:  # 成功读取串口数据
            PacketAnalysis()  # 对接收到的数据包进行处理
            AutoGain()  # 对接收到的图像进行增益
            imgTransfer()  # 转移图像
        ImgDisplay()  # 显示图像
        """
        if flag == 0:
            startTime = time.clock()
            flag = 1
        elif flag == 1:
            endTime = time.clock()
            flag = 0
            print("帧率：", int(1 / (endTime - startTime)))
        """


if __name__ == '__main__':
    main()
