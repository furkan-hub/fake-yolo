from pyproj import Transformer
import numpy as np
import copy
from scipy.spatial.transform import Rotation as R#bounding box için kullanılıyor
from dronekit import *



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
                        "iha_batarya": int(uav[i].battery.level),
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

def len3d(vec3):
    return np.sqrt(vec3[0]**2+vec3[1]**2+vec3[2]**2)

def subList(list1,list2):
    result=[]
    for i in range(len(list1)):
        result.append(list1[i]-list2[i])
    return result

def cartes2Spher(x,y,z):#kamera simulasyonu için gerekli fonksiyon(inaktif)
    r=np.sqrt(x*x+y*y+z*z)
    theta=np.arccos(z/np.sqrt(x*x+y*y))
    phi=np.arccos(x/np.sqrt(x*x+y*y))*np.sign(y)

    return r,theta,phi

def CalcBBox(uav1,uav2,fov):
    fov/=2
    prismDims=[1.1,1.718,0.25]
    halfDims=[x/2 for x in prismDims]

    cubeCorners=[]
    for i in range(2):
        for j in range(2):
            for k in range(2):
                point=[i*prismDims[0]-halfDims[0],j*prismDims[1]-halfDims[1],k*prismDims[2]-halfDims[2]]
                cubeCorners.append(point)
    cubeCorners=np.array(cubeCorners)
    angles=[uav2["yaw"],uav2["pitch"],uav2["roll"]]

    r =R.from_euler('zyx',angles,degrees=False)

    rotatedCorners=[r.apply(x) for x in cubeCorners]
    #r theta phi
    ######################
    # pitch ve yaw ursinada ters veya 90 derece eksik/fazlaydı ona göre pitch ve yawı değiştirmen gerekebilir
    camDir=[1,uav1["pitch"],uav1["yaw"]]
    #####################
    points=[cartes2Spher(coords[0]-(uav2["x"]-uav1["x"]),coords[1]-(uav2["y"]-uav1["y"]),coords[2]-(uav2["z"]-uav1["z"])) for coords in rotatedCorners]
    points=np.array(points)
    points[:]-=camDir

# points=np.rad2deg(points)
    screenPoints=[]
    for point in points:
        screenPoints.append([point[1]/fov,point[2]/fov])
    screenPoints=np.array(screenPoints)

    xs=[min(screenPoints[:,0]),max(screenPoints[:,0])]
    ys=[min(screenPoints[:,1]),max(screenPoints[:,1])]

    withinScreen=lambda a: -1<a<1

    if withinScreen(xs[0]) or withinScreen(xs[1]) or withinScreen(ys[0]) or withinScreen(ys[1]):
        xs[0]=np.clip(xs[0],-1,1)
        ys[0]=np.clip(ys[0],-1,1)
        xs[1]=np.clip(xs[1],-1,1)
        ys[1]=np.clip(ys[1],-1,1)

        bbox=[xs[0],ys[1],(xs[1]-xs[0]),(ys[1]-ys[0])]
        return bbox
    return [0,0,0,0]

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
    