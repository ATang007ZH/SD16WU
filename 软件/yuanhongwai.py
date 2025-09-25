#encoding:utf-8
"""
@ CNRobot V2.0
"""
# https://github.com/wangqi1994/robot/blob/master/yuanhongwai.py

import serial           # 串口库
import time             # 时间库
from datetime import datetime
import binascii         # 编码库
import threading        # 线程库
import numpy as np      # 数值库
import cv2              # CV库
from PIL import Image   # 图片库
import math             # 数学库
import matplotlib.pyplot as plt

import setting
"""--------------------
串口类定义
--------------------"""
class comSerial:
    # 构造串口类的属性
    def __init__(self, portx, bps, waitTime):
        self.serialPort = portx     # 端口号
        self.baudRate = bps         # 波特率
        self.timeOut = waitTime     # 超时等待时间
        self.serial = None

    # 启动串口
    def Start(self):
        global receivdata
        try:
            self.serial = serial.Serial(self.serialPort, self.baudRate, timeout = self.timeOut)
            print(portx, "is start success...\n")
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
        global  imgBuffer, imgData, receivdata
        data = bytes()

        if self.serial.inWaiting() >= 80000:            # 串口数据量大于两帧图像
            print(self.serial.inWaiting())
            startTime = time.clock()
            receivdata.data = self.serial.read(80000)   # 规定一次读取两帧图像
            receivdata.length = len(receivdata.data)

            endTime = time.clock()
            print("串口读取数据花费%fs"%(endTime - startTime))
            startTime = time.clock()
            PacketAnalysis()
            #AutoGain()
            imgData = np.array(imgBuffer, dtype=np.uint8)
            endTime = time.clock()

            print("解析图像数据花费%fs" % (endTime - startTime))
        #timer = threading.Timer(0.01, ser.ReadData())           # 重新启动定时器
        #timer.start()

"""--------------------
SD16B数据类定义
--------------------"""
class SD16B_Data:
    def __init__(self):
        self.frameStart = 0x0000          # 帧开始
        self.status = 0x00                # 传输状态
        self.command = 0x00               # 指令
        self.dataLength = 0               # 数据长度
        self.crc1 = 0x0000                # 校验
        self.data = [0] * 65535           # 数据
        self.crc2 = 0x0000                # 校验
        self.frameEnd = 0x0000            # 帧结束

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
火焰类定义
--------------------"""
class Fire:
    def __init__(self):
        self.src_x = 0                  # 火焰源点x坐标
        self.src_y = 0                  # 火焰源点y坐标
        self.upLimit = 0                # 火焰区域上界
        self.downLimit = 0              # 火焰区域下界
        self.leftLimit = 0              # 火焰区域左界
        self.rightLimit = 0             # 火焰区域右界
        self.aveTemp = 0                # 区域平均温度
        self.tempRecord = [0] * 10      # 记录10帧平均温度
        self.tempVar = 0                # 温度方差
        self.tempStd = 0                # 温度标准差
        self.tempFlag = 0               # 温度特征
        self.area = 0                   # 区域面积
        self.areaRecord = [0] * 10      # 记录十帧面积信息
        self.areaVar = 0                # 面积方差
        self.areaStd = 0                # 面积标准差
        self.generalFlag = 0            # 是否为火源的最终标志
        self.levelAngle =  0            # 火源源点水平角度，左负右正
        self.pitchAngle = 0             # 火源源点俯仰角度，上正下负

    def set(self):                      # 标记火源
        self.generalFlag = 1

    def clear(self):                    # 清除火源
        """"
        self.src_x = 0
        self.src_y = 0
        self.upLimit = 0
        self.downLimit = 0
        self.leftLimit = 0
        self.rightLimit = 0
        self.aveTemp = 0
        self.tempRecord = [0]
        self.tempFlag = 0
        self.aveArea = 0
        self.areaRecord = [0]
        """
        self.generalFlag = 0

    def isFire(self):
        if self.generalFlag == 1:
            return 1
        else:
            return 0


"""--------------------
全局变量定义
--------------------"""
# SD16B 指令定义
SD16B_START_SIGNAL = 'AA 55 00 00 00 01 4F 7E 00 00 00 0D 0A'
SD16B_END_SIGNAL   = 'AA 55 00 00 00 01 4F 7E FF 1E F0 0D 0A'
SD16B_GRAY_SIGNAL  = 'AA 55 00 09 00 01 D1 EF FF 1E F0 0D 0A'
SD16B_TEMP_SIGNAL  = 'AA 55 00 09 00 01 D1 EF 00 00 00 0D 0A'

STRGLO = ''     #读取的数据
BOOL = True     #读取标志位

#数据状态
STARTAA    = 0
START55    = 1
STATUS     = 2
COMMOND    = 3
DATALENGTH = 4
CRC1       = 5
DATA       = 6
CRC2       = 7
END0D      = 8
END0A      = 9


SD16B_data = SD16B_Data()                                       # SD16B数据
receivdata = seriaPortReceive()                                 # 从串口接收到的数据
sensor = Sensor()                                               # 传感器尺寸
imgBuffer = [[0 for i in range(160)] for j in range(120)]       # 用于转移图像数据
tempData = np.zeros((120, 160), dtype = np.uint8)               # 用于存储像素点温度，去除负温
imgData = np.zeros((120, 160), dtype = np.uint8)                # 图像数据
imgBinary = np.copy(imgData)                                    # 二值化图像数据
usart = serial.Serial()                                         # 串口
tempMax = Point()                                               # 最高温度
tempMin = Point()                                               # 最低温度
fire = Fire()                                                   # 火焰
fireThreashold = 40                                            # 判断为火焰点的阈值

"""------全局变量定义结束-----"""


#
# 往串口写数据
#
def usartWriteData(str):
    global usart

    # 将指令字符串转化成16进制编码
    data = bytes.fromhex(str)
    usart.write(data)
"""------END-----"""

#
# 从串口读数据
#
def usartReadData():
    global usart
    data = bytes()
    if usart.inWaiting() >= 45000:                                     # 缓冲区数据大于一帧图像数据
        receivdata.data = usart.read(usart.inWaiting())                # 读出串口数据
        receivdata.length = len(receivdata.data)                       # 数据长度
        return 1
    else:
        return 0
"""------END-----"""

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
        if tranStatus == STARTAA :
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
                SD16B_data.data[length - 8] = receivdata.data[index]   # 容易越界！！！
            if length == SD16B_data.dataLength + 7:                    # 越界是这里的问题？？？
                tranStatus = CRC2
                #print(7)

        # CRC2字节
        elif tranStatus == CRC2:
            if length == SD16B_data.dataLength + 8:
                crc2 = 0xFFFF
                SD16B_data.crc2 = receivdata.data[index] << 8
            elif length == SD16B_data.dataLength + 9:
                SD16B_data.crc2 |= receivdata.data[index]
                tranStatus = END0D
                #print(8)
            else:
                tranStatus = STARTAA

        # 结束字节
        elif tranStatus == END0D:
            if length == SD16B_data.dataLength + 10 and receivdata.data[index] == 0x0D:
                SD16B_data.frameEnd = 0x0D00
                tranStatus = END0A
                #print(9)
            else:
                tranStatus = STARTAA

        # 结束字节
        elif tranStatus == END0A:
            if length == SD16B_data.dataLength + 11 and receivdata.data[index] == 0x0A:
                SD16B_data.frameEnd |= 0x0A
                tranStatus = STARTAA
                length = 0
                #print("OK!!!")
                DataAnalysis()                  # 数据分析
            else:
                length = 0
                tranStatus = STARTAA

        # else
        else:
            tranStatus = STARTAA
            length = 0

        length += 1
"""------END-----"""

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
        lineIndex = SD16B_data.data[0]                                                                  # 解析行号
        # print("lineIndex:",lineIndex)
        cnt = 0
        if lineIndex >= 0 and lineIndex <= 119:                                                         # 行号正常
            if SD16B_data.dataLength == 321:                                                            # 这一行数据长度正常
                for i in range(1, SD16B_data.dataLength):
                    if i & 0x01:                                                                        # 奇数，高位字节
                        imgBuffer[lineIndex][cnt] = SD16B_data.data[i] << 8
                    else:                                                                               # 偶数，高位字节
                        imgBuffer[lineIndex][cnt] += SD16B_data.data[i]
                        tempData[lineIndex][cnt] = int(imgBuffer[lineIndex][cnt] / 10 - 273)            # 记录像素点灰度
                        cnt += 1

    # 温度图像信号
    elif SD16B_data.command == 0x25:
        lineIndex = SD16B_data.data[0]                                                                  # 解析行号
        #print("lineIndex:",lineIndex)
        cnt = 0
        if lineIndex >= 0 and lineIndex <= 119:                                                         # 行号正常
            if SD16B_data.dataLength == 321:                                                            # 这一行数据长度正常
                for i in range(1, SD16B_data.dataLength):
                    if i & 0x01:                                                                        # 奇数，高位字节
                        imgBuffer[lineIndex][cnt] = SD16B_data.data[i] << 8
                    else:                                                                               # 偶数，高位字节
                        imgBuffer[lineIndex][cnt] += SD16B_data.data[i]
                        temp = (imgBuffer[lineIndex][cnt] / 10 - 273)                                   # 计算温度
                        if temp < 0:                                                                    # 滤除负温
                            temp = 0
                        tempData[lineIndex][cnt] = temp
                        cnt += 1

    # 特定点温度信息：温度最高点坐标及温度值，温度最低点坐标及温度值，中心点坐标及温度值，任意点温度值及坐标
    elif SD16B_data.command == 0x0E:
        tempMax.x = SD16B_data.data[0]                      # 0：最高温度横坐标
        tempMax.y = SD16B_data.data[1]                      # 1：最高温度纵坐标
        tempMax.data = SD16B_data.data[2] << 8              # 2~3：最高温度数值
        tempMax.data |= SD16B_data.data[3]
        tempMax.data = int(tempMax.data / 10 - 100)         # 转化成摄氏度


        tempMin.x = SD16B_data.data[8]                      # 8：最高温度横坐标
        tempMin.y = SD16B_data.data[9]                      # 9：最高温度纵坐标
        tempMin.data = SD16B_data.data[10] << 8             # 10~11：最高温度数值
        tempMin.data |= SD16B_data.data[11]
        tempMin.data = int(tempMin.data / 10 - 100)         # 转化成摄氏度
"""------END-----"""

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
    imgHist = [0] * (maxValue + 1)  #图像直方图

    for row in range(sensor.high):
        for col in range(sensor.width):
            gray = imgBuffer[row][col]
            if gray > maxValue:     # 限幅
                gray = maxValue
            if gray < 0:
                gray = 0
            imgHist[int(gray)] += 1
            autoGrainParam.aveGraygradation += gray

    autoGrainParam.aveGraygradation = autoGrainParam.aveGraygradation / (sensor.width * sensor.high)    # 灰度平均值

    lowerLimit = 0
    hightLimit = 0

    for i in range(maxValue):
        if lowerLimit < 200:
            lowerLimit += imgHist[i]
            Vmin = i
        if hightLimit < sensor.high * sensor.width - 100:
            hightLimit += imgHist[i]
            Vmax = i

    autoGrainParam.gain = 11    # 测温
    autoGrainParam.ThresholdLimits = autoGrainParam.gain * 14

    if Vmax > maxValue:
        Vmax = maxValue
    if Vmax < maxValue - (autoGrainParam.ThresholdLimits / 12):
        Vmax += (autoGrainParam.ThresholdLimits / 12)

    # 以直方图峰值为基准划分边界
    autoGrainParam.difThreshold = Vmax - Vmin + autoGrainParam.ThresholdLimits / 6
    autoGrainParam.lowThreshold = autoGrainParam.aveGraygradation - (autoGrainParam.difThreshold / 2)
    autoGrainParam.difThreshold = Vmax - autoGrainParam.lowThreshold

    if autoGrainParam.difThreshold < autoGrainParam.ThresholdLimits:    # 限制最小拉伸范围，不能无限拉伸
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

            imgBuffer[j][i] = (imgBuffer[j][i] - autoGrainParam.lowThreshold) * maxValue / autoGrainParam.difThreshold # 拉伸

            if imgBuffer[j][i] > maxValue:
                imgBuffer[j][i] = maxValue
"""------END-----"""

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

    #cv2.medianBlur(imgData, 3, imgData)                # 中值滤波，去除噪声
    cv2.GaussianBlur(imgData, (3, 3), 5, imgData, 5)    # 高斯平滑图像信息
    #cv2.GaussianBlur(tempData, (3, 3), 5, tempData, 5)  # 高斯平滑温度信息
    cv2.medianBlur(tempData, 3, tempData)  # 中值滤波，去除噪声
"""------END-----"""

#
# 显示图像
#
def ImgDisplay():
    global imgData

    cv2.namedWindow('img', 0)
 #   cv2.namedWindow("imgBinary", 0)

  #  imgBinary = np.copy(imgData)
  #  if fire.isFire() == 1:
  #      cv2.threshold(imgBinary, 0, 255, cv2.THRESH_OTSU, imgBinary)
  #  else:
  #      cv2.threshold(imgBinary, 255, 255, cv2.THRESH_BINARY, imgBinary)

  #  cv2.imshow("imgBinary", imgBinary)

    if fire.isFire() == 1:
        imgData = cv2.rectangle(imgData, (fire.leftLimit - 3, fire.upLimit - 3), (fire.rightLimit + 3, fire.downLimit + 3), (255, 255, 255), 1)     # 矩形框框住火源
        imgData = cv2.circle(imgData, (int(fire.src_x), int(fire.src_y - 1)), 3, (0, 0, 0), 1, lineType=cv2.LINE_AA)                            # 圈出火焰底部源点
        cv2.putText(imgData, str(int(fire.aveTemp)), (fire.rightLimit + 5, fire.upLimit - 5), cv2.FONT_HERSHEY_DUPLEX, 0.3, (255,255,255), 1)       # 显示温度
        #cv2.putText(imgData, str(int(fire.srcAngle)), (fire.rightLimit + 5, fire.downLimit + 8), cv2.FONT_HERSHEY_DUPLEX, 0.3, (255, 255, 255), 1)  # 显示角度

    cv2.imshow('img', imgData)
    cv2.waitKey(1)
"""------END-----"""

#
# 显示各种数据
#
def DataDisplay():
    #print("Max:", tempMax.x, tempMax.y, int(tempMax.data), int(GetTargetAngle(tempMax.x, tempMax.y)))   # 显示最高点温度信息
    #print("Min:", tempMin.x, tempMin.y, int(tempMin.data), int(GetTargetAngle(tempMin.x, tempMin.y)))   # 显示最低点温度信息
    #print('area:', fire.aveArea)
    """

    print("温度方差：", fire.tempVar)
    print("温度标准差：", fire.tempStd)
    print('\n')
    print("面积方差：", fire.areaVar)
    print("面积标准差：", fire.areaStd)
    print('\n')
    """

    if fire.isFire() == 1:
        print("水平：",fire.levelAngle)
        print("俯仰：", fire.pitchAngle)


"""------END-----"""

#
# 求火焰源点相对于摄像头中心的角度
# 分为水平角度，俯仰角度
#
def GetFireAngle(fire_x, fire_y):
    imgMid_x = 80  # 图像最中点的x坐标
    imgMid_y = 60  # 图像最中点的y坐标
    pixel = 0.017  # 像元间距，单位mm
    f = 6            # 焦距
    PI = 3.141592

    levelAngle = math.atan(pixel * (fire_x - imgMid_x) / f) * 180 / PI        # 水平角度
    pitchAngle = math.atan(pixel * (imgMid_y - fire_y) / f) * 180 / PI        # 俯仰角度

    return levelAngle, pitchAngle


#
# 判断是否是火焰点
#
def isFirePoint(target_x, target_y):

    if target_x >= 1 and target_x <= sensor.width - 2 and target_y >= 1 and target_y <= sensor.high - 2:    # 防止越界
        a = int(tempData[target_y][target_x])
        b = int(tempData[target_y - 1][target_x])
        c = int(tempData[target_y + 1][target_x])
        d = int(tempData[target_y][target_x - 1])
        e = int(tempData[target_y][target_x + 1])
        if (a + b + c + d + e) > fireThreashold * 5:       # 上下左右中五个点的温度之和大于5倍火焰阈值说明目标点火焰点
            return 1
        else:
            return 0
    else:
        return  0
"""------END-----"""

#
# 搜索可能的火焰点，并进一步判断是否为火焰
#
def SearchFire(target_x, target_y):
    global fire
    last_CorrCenter_x = target_x    # 上一次的腐蚀中心
    last_CorrCenter_y = target_y
    curr_CorrCenter_x = target_x    # 当前的腐蚀中心
    curr_CorrCenter_y = target_y
    tempSum = 0
    cnt = 0

    fire.clear()    # 清楚火焰标记

    if isFirePoint(target_x, target_y) == 1:        # 目标点为火焰再进行下一步处理

        # 探寻腐蚀中心
        while 1:
            start_x = int(curr_CorrCenter_x)     # 搜索起始点x坐标
            start_y = int(curr_CorrCenter_y)     # 搜索起始点y坐标
            tempSum = 0                          # 清空累积温度

            while isFirePoint(start_x, start_y) == 1:  # 如果是火焰点，则一直向上走
                tempSum += tempData[start_y][start_x]  # 累积温度值
                start_y = start_y - 1
                if start_y == 0:
                   break

            fire.upLimit = start_y       # 找到上界

            start_y = curr_CorrCenter_y  # 回到起始点

            while isFirePoint(start_x, start_y) == 1:  # 如果是火焰点，则一直向下走
                tempSum += tempData[start_y][start_x]  # 累积温度值
                start_y = start_y + 1
                if start_y == sensor.high - 1:
                    break

            fire.downLimit = start_y  # 找到下界

            start_y = curr_CorrCenter_y

            while isFirePoint(start_x, start_y) == 1:  # 如果是火焰点，则一直向左走
                tempSum += tempData[start_y][start_x]  # 累积温度值
                start_x = start_x - 1
                if start_x == 0:
                    break

            fire.leftLimit = start_x  # 找到左界

            start_x = curr_CorrCenter_x

            while isFirePoint(start_x, start_y) == 1:  # 如果是火焰点，则一直向右走
                tempSum += tempData[start_y][start_x]  # 累积温度值
                start_x = start_x + 1
                if start_x == sensor.width - 1:
                    break

            fire.rightLimit = start_x  # 找到右界

            fire.aveTemp = tempSum / (fire.rightLimit - fire.leftLimit + fire.downLimit - fire.upLimit - 1)  # 计算平均温度

            last_CorrCenter_x = curr_CorrCenter_x                             # 记录旧的腐蚀中心
            last_CorrCenter_y = curr_CorrCenter_y
            curr_CorrCenter_x = int(fire.leftLimit + (fire.rightLimit - fire.leftLimit) / 2)   # 计算新的腐蚀中心
            curr_CorrCenter_y = int(fire.upLimit + (fire.downLimit - fire.upLimit) / 2)
                                                               # 腐蚀中心几乎不变
            if (last_CorrCenter_x - curr_CorrCenter_x) >= -2 and (last_CorrCenter_x - curr_CorrCenter_x) <= 2 and (last_CorrCenter_y - curr_CorrCenter_y) >= -2 and (last_CorrCenter_y - curr_CorrCenter_y) <= 2:
                break

            #if last_CorrCenter_x == curr_CorrCenter_x and last_CorrCenter_y == curr_CorrCenter_y:
             #   break

            cnt += 1
            if cnt >= 100:      # 迭代100次尚未确定腐蚀中心，强制退出
                break

        for i in range(9):  # 平移记录温度
            fire.tempRecord[i] = fire.tempRecord[i + 1]

        fire.tempRecord[9] = fire.aveTemp   # 记录新的温度

        fire.tempStd = np.std(fire.tempRecord, ddof=1)  # 计算温度标准差
        fire.tempVar = np.var(fire.tempRecord, ddof=1)  # 计算温度方差

        k = 0

        for i in range(9):
            k += (fire.tempRecord[i + 1] - fire.tempRecord[i])      # 累加斜率

        k = k / 5

        if k > 1 and fire.aveTemp > fireThreashold:                 # 温度呈现上升状态且当前温度较高，升温过程
            fire.tempflag = 1        # 火焰温度特征标记
        elif k < -1 and fire.aveTemp > fireThreashold:              # 温度呈现下降状态且当前温度较高，降温状态
            fire.tempflag = 1        # 火焰温度标记
        elif fire.aveTemp > 2 * fireThreashold:                      # 温度几乎不变，但是温度极高，大于两倍的火焰阈值，需进行下一步检测
            fire.tempflag = 1

        fire.area = (fire.rightLimit - fire.leftLimit) * (fire.downLimit - fire.upLimit) # 计算区域面积

        for i in range(9):          # 平移记录面积
            fire.areaRecord[i] = fire.areaRecord[i + 1]

        fire.areaRecord[9] = fire.area

        fire.areaStd = np.std(fire.areaRecord, ddof=1)  # 计算面积标准差
        fire.areaVar = np.var(fire.areaRecord, ddof=1)  # 计算面积标准差

        fire.src_x = fire.leftLimit + ((fire.rightLimit - fire.leftLimit) / 2)           # 火焰下方源点x坐标
        fire.src_y = fire.downLimit                                                      # 火焰下方源点y坐标

        fire.levelAngle, fire.pitchAngle = GetFireAngle(fire.src_x, fire.src_y)
        if fire.aveTemp > fireThreashold:
            fire.set()       # 获取火源角度
            # print([fire.levelAngle, fire.pitchAngle])
            if setting.yuanhongwai_need_find_angle:
                setting.yuanhongwai_levelAngle = fire.levelAngle
                setting.yuanhongwai_pitchAngle = fire.pitchAngle
                setting.yuanhongwai_now_time = datetime.now()

"""------END-----"""

#
# 主函数
#
def main():
    usart.port = setting.yuanhongwai_port                           # 设置端口号
    usart.baudrate = 8000000                                        # 波特率
    usart.timeout = 1                                               # 超时等待时间
    usart.open()                                                    # 开启串口
    usart.set_buffer_size(rx_size=150000)                           # 设置串口接收缓冲区
    usart.flushInput()                                              # 清空串口接收缓冲区
    usartWriteData(SD16B_START_SIGNAL)                              # 通知模组开始传输数据
    time.sleep(1)                                                   # 延时1s
    usart.flushInput()                                              # 清空串口接收缓冲区
    usartWriteData(SD16B_TEMP_SIGNAL)                               # 通知模组回传温度数据

    flag = 0
    startTime = 0
    endTime = 0

    while 1:
        if usartReadData() == 1:                                    # 成功读取串口数据
            PacketAnalysis()                                        # 对接收到的数据包进行处理
            AutoGain()                                              # 对接收到的图像进行增益
            imgTransfer()                                           # 转移图像
            SearchFire(tempMax.x, tempMax.y)                        # 寻找，判断火源
        # DataDisplay()                                              # 显示各类数据信息
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

        # imgTransfer()                                               # 转移图像

        #
        #
        # key = cv2.waitKey(1)
        #
        # if key == ord('q') or key == ord('Q'):
        #     break
if __name__ == '__main__':
    main()
