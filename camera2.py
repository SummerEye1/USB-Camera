import cv2
 
WIDTH = 400          # 横サイズ
HEIGHT = 300
#FPS= 10
camera1 = cv2.VideoCapture(0)
camera1.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
camera1.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
#camera1.set(cv2.CAP_PROP_FPS, FPS) 

camera2 = cv2.VideoCapture(2)                # カメラCh.(ここでは0)を指定
camera2.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
camera2.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
#camera2.set(cv2.CAP_PROP_FPS, FPS)
 
# 撮影＝ループ中にフレームを1枚ずつ取得（qキーで撮影終了）
while True:
    ret1, frame1 = camera1.read() # フレームを取得
    print(ret1)
    #cv2.imshow('camera1', frame1)  
    
    ret2, frame2 = camera2.read() # フレームを画面に表示
    print(ret2)
    #cv2.imshow('camera2', frame2)
    
    im_h=cv2.hconcat([frame1,frame2])
    cv2.imshow('frame1', im_h)
 
    # キー操作があればwhileループを抜ける
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
        
 
 
# 撮影用オブジェクトとウィンドウの解放
camera.release()
cv2.destroyAllWindows()
