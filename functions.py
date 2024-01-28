import math
from dronekit import *
import numpy as np
from pyproj import Transformer

def subList(list1,list2):
    result=[]
    for i in range(len(list1)):
        result.append(list1[i]-list2[i])
    return result

def len3d(vec3):
    return np.sqrt(vec3[0]**2+vec3[1]**2+vec3[2]**2)

class LocationConverter:
    def __init__(self) -> None:
        self.trans_GPS_to_XYZ = Transformer.from_crs(4979, 4978)
    def relativeLoc(self,loc1,loc2):
    # lon,lat,alt -> x,y,z
        first=self.trans_GPS_to_XYZ.transform(*loc1)
        rad1=len3d(first)
        second=self.trans_GPS_to_XYZ.transform(*loc2)
        rad2=len3d(second)
        z=rad2-rad1
        locx=copy.copy(loc1)
        locy=copy.copy(loc1)
        locx[1]=loc2[1]
        locy[0]=loc2[0]
        distx=subList(self.trans_GPS_to_XYZ.transform(*locx),first)
        disty=subList(self.trans_GPS_to_XYZ.transform(*locy),first)
        distx=len3d(distx)*np.sign(loc2[1]-loc1[1])
        disty=len3d(disty)*np.sign(loc2[0]-loc1[0])
        return [distx,disty,z]

def get_telem(uav):
    # Zaman bilgisini bir sözlükte saklayın

    telems = []

    for i in range(len(uav)):

        gps_time = {
                            "saat": 0,
                            "dakika": 0,
                            "saniye": 0,
                            "milisaniye": 0 // 1000  # Mikrosaniyeyi milisaniyeye dönüştürün
                        }

                        #print(gps_time)
                        
        telem = {#telemetri paketlerini oluştur
                        "takim_numarasi": i,
                        "iha_enlem": float(uav[i].location.global_frame.lat),
                        "iha_boylam": float(uav[i].location.global_frame.lon),
                        "iha_irtifa": float(uav[i].location.global_frame.alt),
                        "iha_dikilme": float((uav[i].attitude.pitch)*(180 / math.pi)),
                        "iha_yonelme": float((uav[i].attitude.yaw)*(180 / math.pi)),
                        "iha_yatis": float((uav[i].attitude.roll)*(180 / math.pi)),
                        "iha_hiz": float(uav[i].groundspeed),
                        "iha_batarya": 100,
                        "iha_otonom": 1,
                        "iha_kilitlenme": 0,
                        "hedef_merkez_X": 0,
                        "hedef_merkez_Y": 0,
                        "hedef_genislik": 0,
                        "hedef_yukseklik": 0,
                        "gps_saati":gps_time,
                    }
        
        telems.append(telem)

    return telems
