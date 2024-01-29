import cv2
import numpy as np
import math

def rotate_camera(image_width, image_height, focal_length, angles):
    # Kamera iç matrisi oluştur
    focal_length_x = focal_length
    focal_length_y = focal_length
    optical_center_x = image_width / 2
    optical_center_y = image_height / 2

    K = np.array([
        [focal_length_x, 0, optical_center_x],
        [0, focal_length_y, optical_center_y],
        [0, 0, 1]
    ])

    # Kamera dönüş matrisi (3 eksende döndürme)
    angle_x, angle_y, angle_z = map(math.radians, angles)
    Rx = np.array([
        [1, 0, 0],
        [0, math.cos(angle_x), -math.sin(angle_x)],
        [0, math.sin(angle_x), math.cos(angle_x)]
    ])

    Ry = np.array([
        [math.cos(angle_y), 0, math.sin(angle_y)],
        [0, 1, 0],
        [-math.sin(angle_y), 0, math.cos(angle_y)]
    ])

    Rz = np.array([
        [math.cos(angle_z), -math.sin(angle_z), 0],
        [math.sin(angle_z), math.cos(angle_z), 0],
        [0, 0, 1]
    ])

    R = np.dot(Rz, np.dot(Ry, Rx))  # Z * Y * X sırasında çarpma

    # Kamera translasyon vektörü
    t = np.array([0, 0, 0])  # kamera konumu (2 birim yükseklikte)

    # Kamera projeksiyon matrisi
    P = np.dot(K, np.hstack((R, t.reshape(-1, 1))))

    return P

def rotate_cube(cube_points, angles, translation):
    # Küp dönüş matrisi (6 eksende döndürme)
    Rx = np.array([
        [1, 0, 0],
        [0, math.cos(angles[0]), -math.sin(angles[0])],
        [0, math.sin(angles[0]), math.cos(angles[0])]
    ])

    Ry = np.array([
        [math.cos(angles[1]), 0, math.sin(angles[1])],
        [0, 1, 0],
        [-math.sin(angles[1]), 0, math.cos(angles[1])]
    ])

    Rz = np.array([
        [math.cos(angles[2]), -math.sin(angles[2]), 0],
        [math.sin(angles[2]), math.cos(angles[2]), 0],
        [0, 0, 1]
    ])

    R = np.dot(Rz, np.dot(Ry, Rx))  # Z * Y * X sırasında çarpma

    # Küp translasyon vektörü
    t = np.array(translation)

    # Küpün dünya koordinatlarından kamera koordinatlarına dönüştür
    cube_points_camera = np.dot(R, cube_points.T) + t.reshape(-1, 1)

    return cube_points_camera.T

def project_points(P, points):
    # Nokta bulutunu homojen koordinatlara dönüştür
    points_homo = np.hstack((points, np.ones((points.shape[0], 1))))

    # Kamera koordinatlarındaki noktaları görüntü düzlemine dönüştür
    image_points = np.dot(P, points_homo.T).T

    # Görüntü düzlemine düşen noktaları normalize et
    image_points[:, 0] /= image_points[:, 2]
    image_points[:, 1] /= image_points[:, 2]

    # 2D görüntü düzlemine yansıtma
    image_points_2d = image_points[:, :2]

    return image_points_2d

def bbox_camera_sim(width, height, focus_length, camera_angles, cube_angles, cube_loc, cube_points):
    # Kamera özellikleri
    image_width = width   # görüntü genişliği
    image_height = height  # görüntü yüksekliği
    focal_length = focus_length  # odak uzaklığı

    # Kamera projeksiyon matrisi
    P = rotate_camera(image_width, image_height, focal_length, camera_angles)

    # Dünya koordinatlarındaki küpün noktaları
    cube_points = cube_points

    # Döndür ve ötele
    cube_points_camera = rotate_cube(cube_points, cube_angles, cube_loc)
    image_points = project_points(P, cube_points_camera)

    return image_points

def draw_cube(image, cube_points):
    # Çizilen küpün kenarlarını belirle
    edges = [
        (0, 1), (1, 3), (3, 2), (2, 0),
        (4, 5), (5, 7), (7, 6), (6, 4),
        (0, 4), (1, 5), (2, 6), (3, 7)
    ]

    for edge in edges:
        cv2.line(image, tuple(cube_points[edge[0]].astype(int)), tuple(cube_points[edge[1]].astype(int)), (255, 255, 255), 2)

def main():
    # Kamera özellikleri
    image_width = 640   # görüntü genişliği
    image_height = 480  # görüntü yüksekliği
    focal_length = 100  # odak uzaklığı

    # Dönüş açıları
    camera_angles = [0, 0, 0] # x, y, z
    cube_angles = [0, 0, 30] # x, y, z
    cube_loc = [0, 0, 0]  # x, y, z

    # Dünya koordinatlarındaki küpün noktaları
    cube_points = np.array([
        [-0.5, -0.5, -0.5],  # sol alt ön köşe
        [-0.5, -0.5, 0.5],   # sol alt arka köşe
        [-0.5, 0.5, -0.5],   # sol üst ön köşe
        [-0.5, 0.5, 0.5],    # sol üst arka köşe
        [0.5, -0.5, -0.5],   # sağ alt ön köşe
        [0.5, -0.5, 0.5],    # sağ alt arka köşe
        [0.5, 0.5, -0.5],    # sağ üst ön köşe
        [0.5, 0.5, 0.5]      # sağ üst arka köşe
    ])

    image = np.zeros((image_height, image_width, 3), dtype=np.uint8)

    # Kamera ve Küpü simüle et
    image_points = bbox_camera_sim(image_width, image_height, focal_length, camera_angles, cube_angles, cube_loc, cube_points)

    # Küpü çiz
    draw_cube(image, image_points)

    # Sonucu göster
    cv2.imshow('Rotated Camera and Cube Projection', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
