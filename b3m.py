import serial
import time

#WRITEコマンド
def B3M_setPos_CMD(servo_id, pos, MoveTime):

    #送信コマンドを作成
    txCmd = [0x09, #SIZE
                   0x06, #CMD
                   0x00, #OPTION
                   servo_id, #ID
                   pos & 0xff, #POS_L
                   pos >> 8 & 0xff, #POS_H
                   MoveTime & 0xff, #TIME_L
                   MoveTime >> 8 & 0xff] #TIME_H

    #チェックサムを作成
    checksum = 0
    for i in txCmd:
      checksum += i

    #リストの最後にチェックサムを挿入する
    txCmd.append(checksum & 0xff)

    #コマンドを送信
    b3m.write(txCmd)

    #サーボからの返事を受け取る
    rxCmd = b3m.read(7)

    #もしリスト何になにも入っていなかったら正常に受信できていないと判断
    if len(rxCmd) == 0:
      return False

    #問題なければ返事を返す
    return Truie

def B3M_Write_CMD(servo_id, txData, Data_size, Address):

    #txDataがマイナスの場合はsigned=Trueを記述する必要がある
    if txData < 0:
        txByte = txData.to_bytes(Data_size, 'little', signed=True)

    else:
        txByte = txData.to_bytes(Data_size, 'little')

    #送信コマンドを作成
    txCmd = [0x07+len(txByte), #SIZE
            0x04, #CMD
            0x00, #OPTION
            servo_id] #ID

    for i in txByte: #Data
        txCmd.append(i)

    txCmd.append(Address) #ADDRESS
    txCmd.append(0x01) #COUNT

    #チェックサムを作成
    checksum = 0
    for i in txCmd:
        checksum += i

    #リストの最後にチェックサムを挿入する
    txCmd.append(checksum & 0xff)

    #WRITEコマンドを送信
    b3m.write(txCmd)

    #サーボからの返事を受け取る
    rxCmd = b3m.read(5)

    #もしリスト何になにも入っていなかったら正常に受信できていないと判断
    if len(rxCmd) == 0:
        return False

    #問題なければ返事を返す
    return True

def B3M_Save_CMD(servo_id):

    #送信コマンドを作成
    txCmd = [0x05, #SIZE
             0x02, #CMD
             0x00, #OPTION
             servo_id] #ID

    #チェックサムを作成
    checksum = 0
    for i in txCmd:
      checksum += i

    #リストの最後にチェックサムを挿入する
    txCmd.append(checksum & 0xff)

    #WRITEコマンドを送信
    b3m.write(txCmd)

    #サーボからの返事を受け取る
    rxCmd = b3m.read(5)

    #もしリスト何になにも入っていなかったら正常に受信できていないと判断
    if len(rxCmd) == 0:
      return False

    #問題なければ返事を返す
    return True

def B3M_Read_CMD(servo_id, Data_size, Address):

    #送信コマンドを作成
    txCmd = [0x07, #SIZE
             0x03, #CMD
             0x00, #OPTION
             servo_id, #ID
             Address, #ADDRRESS
             Data_size] #LENGTH

    #チェックサムを用意
    checksum = 0
    for i in txCmd:
        checksum += i

    #リストの最後にチェックサムを挿入する
    txCmd.append(checksum & 0xff)

    #WRITEコマンドを送信
    b3m.write(txCmd)

    #サーボからの返事を受け取る
    rxCmd = b3m.read(5+Data_size)

    #もしリスト何になにも入っていなかったら正常に受信できていないと判断
    if len(rxCmd) == 0:
        return False, 0

    #問題なければデータを返す
    #1Byteの場合
    if Data_size == 1:
        return True, rxCmd[4]

    #2Byteの場合
    elif Data_size == 2:
        Value = [rxCmd[4], rxCmd[5]]

        #データの範囲にマイナスが含まれる場合
        if (Address == 0x05 #最小位置制御
            or Address == 0x07 #最大位置制御
            or Address == 0x09 #中央値オフセット
            or Address == 0x0B #MCU温度リミット
            or Address == 0x0E #モーター温度リミット
            or Address == 0x2A #目標位置
            or Address == 0x2C #現在位置
            or Address == 0x2E #前回のサンプリングの位置
            or Address == 0x30 #目標速度
            or Address == 0x32 #現在速度
            or Address == 0x34 #前回のサンプリングの速度
            or Address == 0x3C #目標トルク
            or Address == 0x44 #現在のMCU温度
            or Address == 0x46):#現在のモーター温度

            Value = int.from_bytes(Value, 'little', signed=True)

        #データの範囲がプラスのみの場合
        else:
            Value = int.from_bytes(Value, 'little')

            return True, Value


    #4Byteの場合
    elif Data_size == 4:
        Value = [rxCmd[4], rxCmd[5], rxCmd[6], rxCmd[7]]
        Value = int.from_bytes(Value, 'little')
        return True, Value

    else:
        return False, 0

MOVE_NORMAL = 0x00
MOVE_FREE = 0x02
MOVE_HOLD = 0x03

CONTROL_POSITION = 0x00
CONTROL_SPEED = 0x04
CONTROL_CURRENT = 0x08
CONTROL_FEED_FORWARD = 0x0C


#COMポートを開く
b3m = serial.Serial('/dev/ttyUSB0', baudrate=1500000, parity=serial.PARITY_NONE, timeout=0.5)

#動作モード：Free (動作モードと特性を設定するため、設定書き換え中の誤動作を防止するため脱力にしておく)
ModeSelect = CONTROL_POSITION or MOVE_FREE
reData = B3M_Write_CMD(0, ModeSelect, 1, 0x28)
reData = B3M_Write_CMD(1, ModeSelect, 1, 0x28)
print(reData)
print(reData)

#速度制御モードに設定 (目標速度を指定して軸が回転するモード)
ModeSelect = CONTROL_SPEED or MOVE_FREE
reData = B3M_Write_CMD(0, ModeSelect, 1, 0x28)
reData = B3M_Write_CMD(1, ModeSelect, 1, 0x28)
print(reData)
print(reData)

#ゲインプリセット：No.1 (PIDのプリセットゲインを速度制御モード用に設定)
reData = B3M_Write_CMD(0, 0x01, 1, 0x5C)
reData = B3M_Write_CMD(1, 0x01, 1, 0x5C)
print(reData)
print(reData)

#動作モード：Normal （Freeの状態からトルクオン）
ModeSelect = CONTROL_SPEED or MOVE_NORMAL
reData = B3M_Write_CMD(0, ModeSelect, 1, 0x28)
reData = B3M_Write_CMD(1, ModeSelect, 1, 0x28)
print(reData)
print(reData)

#目標速度を指定して回転(100deg/s)(アドレス：0x30（目標速度))(正回転)
reData = B3M_Write_CMD(0, -10000, 2, 0x30)
reData = B3M_Write_CMD(1, 10000, 2, 0x30)
print(reData)
print(reData)

#0.1秒ごとに現在値を100回取得する
for i in range(100):

    #現在位置を読んでコンソールに表示
    reData = B3M_Read_CMD(0, 2, 0x2C)
    reData = B3M_Read_CMD(1, 2, 0x2C)
    print('現在位置= ', reData)
    print('現在位置= ', reData)

    time.sleep(0.1) #指定した秒数の間処理を停止


#目標速度を指定して回転(0deg/s)(アドレス：0x30（目標速度))目標速度を0に指定し、回転を止める
reData = B3M_Write_CMD(0, 0, 2, 0x30)
reData = B3M_Write_CMD(1, 0, 2, 0x30)
print(reData)
print(reData)


#目標速度を指定して回転(-100deg/s)(アドレス：0x30（目標速度))（逆回転）
reData = B3M_Write_CMD(0, 10000, 2, 0x30)
reData = B3M_Write_CMD(1, -10000, 2, 0x30)
print(reData)
print(reData)

#0.1秒ごとに現在値を100回取得する
for i in range(100):

    #現在位置を読んでコンソールに表示
    reData = B3M_Read_CMD(0, 2, 0x2C)
    reData = B3M_Read_CMD(1, 2, 0x2C)
    print('現在位置= ', reData)
    print('現在位置= ', reData)

    time.sleep(0.1) #指定した秒数の間処理を停止


#目標速度を指定して回転(0deg/s)(アドレス：0x30（目標速度))目標速度を0に指定し、回転を止める
reData = B3M_Write_CMD(0, 0, 2, 0x30)
reData = B3M_Write_CMD(1, 0, 2, 0x30)
print(reData)
print(reData)

#動作モード：Free (動作モードと特性を設定するため、設定書き換え中の誤動作を防止するため脱力にしておく)
ModeSelect = CONTROL_POSITION or MOVE_FREE
reData = B3M_Write_CMD(0, ModeSelect, 1, 0x28)
reData = B3M_Write_CMD(1, ModeSelect, 1, 0x28)
print(reData)
print(reData)

#ポートを閉じる
b3m.close()
