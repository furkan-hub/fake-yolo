import math
import numpy as np
from dronekit import *
from utils import *

LocationConverter_ = LocationConverter()


if __name__ == "__main__":

    #connection object
    uav1 = connect("tcp:127.0.0.1:5763",wait_ready=False)
    uav2 = connect("tcp:127.0.0.1:5773",wait_ready=False)

    uavs = [uav1,uav2]

    print("connection is ok")

    while True:
        time.sleep(0.5)

        telems = get_telem(uavs)# telemetri paketlerini oluştur
        
        #print(telems)

        loc1 = [float(telems[0]["iha_enlem"]),float(telems[0]["iha_boylam"]),float(telems[0]["iha_enlem"])]# uav 1 için konum dizisi
        angl1 = [float(telems[0]["iha_dikilme"]),float(telems[0]["iha_yonelme"]),float(telems[0]["iha_yatis"])]

        loc2 = [float(telems[1]["iha_enlem"]),float(telems[1]["iha_boylam"]),float(telems[1]["iha_boylam"])]# uav 2 için konum dizisi
        angl2 = [float(telems[1]["iha_dikilme"]),float(telems[1]["iha_yonelme"]),float(telems[1]["iha_yatis"])]


        cartesian_cords = LocationConverter_.relativeLoc(loc1,loc2)# iki nokta arasın da ki küresel kordinati kartezyen kordinata çevirme

        #print(cartesian_cords)
        
        uav1_loc_angle = {
                            "uav_num":0,
                            "x":loc1[0],
                            "y":loc1[1],
                            "z":loc1[2],
                            "pitch":angl1[0],
                            "yaw":angl1[1],
                            "roll":angl1[2]
                        }
        
        uav2_loc_angle = {
                            "uav_num":1,
                            "x":loc2[0],
                            "y":loc2[1],
                            "z":loc2[2],
                            "pitch":angl2[0],
                            "yaw":angl2[1],
                            "roll":angl2[2]
                        }

        bbox = CalcBBox(uav1_loc_angle,uav2_loc_angle,(5*math.pi)/6)

        print(bbox)

        telems.clear()
        
        