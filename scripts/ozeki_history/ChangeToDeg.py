

#普段の座標系からKRSモーターの座標系に変換(-135°~135の変化(0°原点))
#第一引数:innerから生成された角度
#if文の表記は,Motorの安全確保を図っている。


"""
#############################右##########################################
0:腰ヨー,1:右肩ピッチ,2:右肩ロール,3;右肘ロール,4:右肘ピッチ
5:右股ロール,6:右股ロール,7:右股ピッチ,8:右膝ピッチ,9:右足首ピッチ,10:右足首ロール
"""


def ChangeDegR0(image_deg):
    
    Motor_deg = (image_deg - 90)
    
    if(Motor_deg > 135):
        Motor_deg = 135
    elif(Motor_deg < -135):
        Motor_deg = -135
    
    return Motor_deg


def ChangeDegR1(image_deg):
    
    Motor_deg = (image_deg - 90)
    
    if(Motor_deg > 135):
        Motor_deg = 135
    elif(Motor_deg < -135):
        Motor_deg = -135
    
    return Motor_deg


def ChangeDegR2(image_deg):
    
    Motor_deg = (image_deg - 90)
    
    if(Motor_deg > 135):
        Motor_deg = 135
    elif(Motor_deg < 0):
        Motor_deg = 0
    
    return Motor_deg


def ChangeDegR3(image_deg):
    
    Motor_deg = (image_deg - 90)
    
    if(Motor_deg > 135):
        Motor_deg = 135
    elif(Motor_deg < -135):
        Motor_deg = -135
    
    return Motor_deg


def ChangeDegR4(image_deg):
    
    Motor_deg = (image_deg)
    
    if(Motor_deg > 25):
        Motor_deg = 25
    elif(Motor_deg < -123):
        Motor_deg = -123
    
    return Motor_deg


def ChangeDegR5(image_deg):
    
    Motor_deg = (image_deg - 90)
    
    if(Motor_deg > 135):
        Motor_deg = 135
    elif(Motor_deg < -135):
        Motor_deg = -135
    
    return Motor_deg


def ChangeDegR6(image_deg):
    
    Motor_deg = (image_deg - 90)
    
    if(Motor_deg > 135):
        Motor_deg = 135
    elif(Motor_deg < -135):
        Motor_deg = -135
    
    return Motor_deg


def ChangeDegR7(image_deg):
    
    Motor_deg = (image_deg - 90)
    
    if(Motor_deg > 135):
        Motor_deg = 135
    elif(Motor_deg < -135):
        Motor_deg = -135
    
    return Motor_deg


def ChangeDegR8(image_deg):
    
    Motor_deg = (image_deg - 90)
    
    if(Motor_deg > 135):
        Motor_deg = 135
    elif(Motor_deg < -135):
        Motor_deg = -135
    
    return Motor_deg


def ChangeDegR9(image_deg):
    
    Motor_deg = (image_deg - 90)
    
    if(Motor_deg > 135):
        Motor_deg = 135
    elif(Motor_deg < -135):
        Motor_deg = -135
    
    return Motor_deg


def ChangeDegR10(image_deg):
    
    Motor_deg = (image_deg - 90)
    
    if(Motor_deg > 135):
        Motor_deg = 135
    elif(Motor_deg < -135):
        Motor_deg = -135
    
    return Motor_deg



"""
#############################左##########################################
0:頭ヨー,1:左肩ピッチ,2:左肩ロール,3;左肘ロール,4:左肘ピッチ
5:左股ロール,6:左股ロール,7:左股ピッチ,8:左膝ピッチ,9:左足首ピッチ,10:左足首ロール
"""


def ChangeDegL0(image_deg):
    
    Motor_deg = (image_deg - 90)
    
    if(Motor_deg > 135):
        Motor_deg = 135
    elif(Motor_deg < -135):
        Motor_deg = -135
    
    return Motor_deg

def ChangeDegL1(image_deg):
    
    Motor_deg = (image_deg - 90)
    
    if(Motor_deg > 135):
        Motor_deg = 135
    elif(Motor_deg < -135):
        Motor_deg = -135
    
    return Motor_deg


def ChangeDegL2(image_deg):
    
    Motor_deg = (image_deg - 90)
    
    if(Motor_deg > 135):
        Motor_deg = 135
    elif(Motor_deg < 0):
        Motor_deg = 0
        
    return Motor_deg


def ChangeDegL3(image_deg):
    
    Motor_deg = (image_deg - 90)
    
    if(Motor_deg > 135):
        Motor_deg = 135
    elif(Motor_deg < -135):
        Motor_deg = -135
    
    return Motor_deg


def ChangeDegL4(image_deg):
    
    Motor_deg = (image_deg - 90)
    
    if(Motor_deg > 135):
        Motor_deg = 135
    elif(Motor_deg < -135):
        Motor_deg = -135
    
    return Motor_deg


def ChangeDegL5(image_deg):
    
    Motor_deg = (image_deg - 90)
    
    if(Motor_deg > 135):
        Motor_deg = 135
    elif(Motor_deg < -135):
        Motor_deg = -135
    
    return Motor_deg


def ChangeDegL6(image_deg):
    
    Motor_deg = (image_deg - 90)
    
    if(Motor_deg > 135):
        Motor_deg = 135
    elif(Motor_deg < -135):
        Motor_deg = -135
    
    return Motor_deg


def ChangeDegL7(image_deg):
    
    Motor_deg = (image_deg - 90)
    
    if(Motor_deg > 135):
        Motor_deg = 135
    elif(Motor_deg < -135):
        Motor_deg = -135
    
    return Motor_deg


def ChangeDegL8(image_deg):
    
    Motor_deg = (image_deg - 90)
    
    if(Motor_deg > 135):
        Motor_deg = 135
    elif(Motor_deg < -135):
        Motor_deg = -135
    
    return Motor_deg


def ChangeDegL9(image_deg):
    
    Motor_deg = (image_deg - 90)
    
    if(Motor_deg > 135):
        Motor_deg = 135
    elif(Motor_deg < -135):
        Motor_deg = -135
    
    return Motor_deg


def ChangeDegL10(image_deg):
    
    Motor_deg = (image_deg - 90)
    
    if(Motor_deg > 135):
        Motor_deg = 135
    elif(Motor_deg < -135):
        Motor_deg = -135
    
    return Motor_deg






