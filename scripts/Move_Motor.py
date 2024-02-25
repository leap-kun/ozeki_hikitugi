

#角度からHTH値に変換
#第一引数:degree値,第二引数;回転方向
#change_degにはChangeDegの関数で回転方向を修正したものを入れる
#4000(一象限当たりのmotorの変化分) / 135(一象限当たりの角度の最大変化) = 29.629..............
#if文はモーターの安全確保を図っている。

"""
#############################右##########################################
0:腰ヨー,1:右肩ピッチ,2:右肩ロール,3;右肘ロール,4:右肘ピッチ
5:右股ロール,6:右股ロール,7:右股ピッチ,8:右膝ピッチ,9:右足首ピッチ,10:右足首ロール
"""



"""
#############################左##########################################
0:頭ヨー,1:左肩ピッチ,2:左肩ロール,3;左肘ロール,4:左肘ピッチ
5:左股ロール,6:左股ロール,7:左股ピッチ,8:左膝ピッチ,9:左足首ピッチ,10:左足首ロール
"""



def Motor_R0(change_deg):

    krs = change_deg - 90
    
    if(krs > 11500):
        krs = 11500
    elif(krs < 0):
        krs = 0
           
    return krs



def Motor_R1(change_deg):

    krs = change_deg - 90
    
    if(krs > 11500):
        krs = 11500
    elif(krs < 0):
        krs = 0
           
    return krs


def Motor_R2(change_deg):

    krs = change_deg - 90
    
    if(krs > 14200):
        krs = 14200
    elif(krs < 0):
        krs = 0
    
    return krs



def Motor_R3(change_deg):

    krs = change_deg - 90
    
    if(krs > 11500):
        krs = 11500
    elif(krs < 0):
        krs = 0
           
    return krs



def Motor_R4(change_deg):

    krs = change_deg - 90
    
    if(krs > 11500):
        krs = 11500
    elif(krs < 0):
        krs = 0
           
    return krs



def Motor_R5(change_deg):

    krs = change_deg - 90
    
    if(krs > 11500):
        krs = 11500
    elif(krs < 0):
        krs = 0
           
    return krs


def Motor_R6(change_deg):

    krs = change_deg - 90
    
    if(krs > 11500):
        krs = 11500
    elif(krs < 0):
        krs = 0
           
    return krs

def Motor_R7(change_deg):

    krs = change_deg - 90
    
    if(krs > 11500):
        krs = 11500
    elif(krs < 0):
        krs = 0
           
    return krs


def Motor_R8(change_deg):

    krs = change_deg - 90
    
    if(krs > 11500):
        krs = 11500
    elif(krs < 0):
        krs = 0
           
    return krs


def Motor_R9(change_deg):

    krs = change_deg - 90
    
    if(krs > 11500):
        krs = 11500
    elif(krs < 0):
        krs = 0
           
    return krs


def Motor_R10(change_deg):

    krs = change_deg - 90
    
    if(krs > 11500):
        krs = 11500
    elif(krs < 0):
        krs = 0
           
    return krs


def Motor_L0(change_deg):

    krs = change_deg - 90
    
    if(krs > 11500):
        krs = 11500
    elif(krs < 0):
        krs = 0
           
    return krs



def Motor_L1(change_deg):

    krs = change_deg - 90
    
    if(krs > 11500):
        krs = 11500
    elif(krs < 0):
        krs = 0
           
    return krs


def Motor_L2(change_deg):

    krs = change_deg - 90
    
    if(krs > 11500):
        krs = 11500
    elif(krs < 0):
        krs = 0
           
    return krs



def Motor_L3(change_deg):

    krs = change_deg - 90
    
    if(krs > 11500):
        krs = 11500
    elif(krs < 0):
        krs = 0
           
    return krs



def Motor_L4(change_deg):

    krs = change_deg - 90
    
    if(krs > 11500):
        krs = 11500
    elif(krs < 0):
        krs = 0
           
    return krs



def Motor_L5(change_deg):

    krs = change_deg - 90
    
    if(krs > 11500):
        krs = 11500
    elif(krs < 0):
        krs = 0
           
    return krs


def Motor_L6(change_deg):

    krs = change_deg - 90
    
    if(krs > 11500):
        krs = 11500
    elif(krs < 0):
        krs = 0
           
    return krs

def Motor_L7(change_deg):

    krs = change_deg - 90
    
    if(krs > 11500):
        krs = 11500
    elif(krs < 0):
        krs = 0
           
    return krs


def Motor_L8(change_deg):

    krs = change_deg - 90
    
    if(krs > 11500):
        krs = 11500
    elif(krs < 0):
        krs = 0
           
    return krs


def Motor_L9(change_deg):

    krs = change_deg - 90
    
    if(krs > 11500):
        krs = 11500
    elif(krs < 0):
        krs = 0
           
    return krs


def Motor_L10(change_deg):

    krs = change_deg - 90
    
    if(krs > 11500):
        krs = 11500
    elif(krs < 0):
        krs = 0
           
    return krs

