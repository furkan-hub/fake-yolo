from dronekit import *
from camera_sim import *
import cv2
from functions import *
from time import *

LocationConverter_ = LocationConverter()


uav1 = connect("tcp:127.0.0.1:5762",wait_ready=False)
uav2 = connect("tcp:127.0.0.1:5772",wait_ready=False)

uavs = [uav1,uav2]

print("connected")



while True:
    
    sleep(0.1)
    
    telems = get_telem(uavs)

    #print(telems)

       #print(telems)
    loc1 = [float(telems[0]["iha_enlem"]),float(telems[0]["iha_boylam"]),float(telems[0]["iha_irtifa"])]# uav 1 için konum dizisi
    angl1 = [float(telems[0]["iha_dikilme"]),float(telems[0]["iha_yonelme"]),float(telems[0]["iha_yatis"])]
    loc2 = [float(telems[1]["iha_enlem"]),float(telems[1]["iha_boylam"]),float(telems[1]["iha_irtifa"])]# uav 2 için konum dizisi
    angl2 = [float(telems[1]["iha_dikilme"]),float(telems[1]["iha_yonelme"]),float(telems[1]["iha_yatis"])]
    cartesian_cords = LocationConverter_.relativeLoc(loc1,loc2)# iki nokta arasın da ki küresel kordinati kartezyen kordinata çevirme
   
    uav_1= {
        "roll":angl1[2],
        "pitch":angl1[0],
        "yaw":angl1[1],
        "x":0,
        "y":0,
        "z":0
    }
    
    uav_2= {
        "roll":angl2[2],
        "pitch":angl2[0],
        "yaw":angl2[1],
        "x":cartesian_cords[0],
        "y":cartesian_cords[1],
        "z":cartesian_cords[2]
    }
   
    #print(uav_1,uav_2)
    
    image_points = bbox_camera_sim(640,480,120,angl1,angl2,cartesian_cords)
    print(image_points)
    
    image = np.zeros((480, 640, 3), dtype=np.uint8)

    # Noktaları görüntü üzerine çiz
    for point in image_points:
        cv2.circle(image, tuple(point.astype(int)), 1, (255, 255, 255), -1)

    # Sonucu göster
    cv2.imshow('Rotated Camera and Cube Projection', image)
    cv2.waitKey(1)
