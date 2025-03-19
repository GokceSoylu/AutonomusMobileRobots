# -*- coding: utf-8 -*-
"""
@author: acseckin
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math
import time

# Bağlantıyı oluştur ve simülasyon modülünü al
client = RemoteAPIClient()
sim = client.require('sim')

# Stepping modunu etkinleştir ve simülasyonu başlat
sim.setStepping(True)
sim.startSimulation()

# -------------------------------------------------------
# 1. Ebeveyn (Parent) Nesnesini Oluşturma ve Ayarlama
# -------------------------------------------------------
# Küçük bir küp (cuboid) kullanarak ebeveyn nesnesi oluşturuyoruz.
parent_handle = sim.createPureShape(0, 0, [0.1, 0.1, 0.1], 1.0, None)
print("Parent Handle:", parent_handle)

# Ebeveyn nesnesinin global konumunu [0,0,0] olarak ayarla
sim.setObjectPosition(parent_handle, -1, [0, 0, 0])
# Ebeveyn nesnesini global frame'de Z etrafında 45° (π/4 radyan) döndür
sim.setObjectOrientation(parent_handle, -1, [0, 0, math.pi/4])
sim.step()  # Ayarların simülasyona yansıması için bir adım ilerlet

# -------------------------------------------------------
# 2. Çocuk (Child) Nesnesini Oluşturma ve Ebeveyne Bağlama
# -------------------------------------------------------
# Daha büyük bir küp oluşturarak çocuk nesnesi elde ediyoruz.
child_handle = sim.createPureShape(0, 0, [0.2, 0.2, 0.2], 1.0, None)
print("Child Handle:", child_handle)

# Çocuğun global konumunu [1, 0, 0] olarak ayarla
sim.setObjectPosition(child_handle, -1, [1, 0, 0])
sim.step()

# Çocuk nesneyi ebeveyn nesneye bağla. 
# "True" parametresi, mevcut global konumun korunmasını sağlar.
sim.setObjectParent(child_handle, parent_handle, True)
sim.step()

# -------------------------------------------------------
# 3. Hareket İşlemlerinde Referans Frame Farklılıkları
# -------------------------------------------------------

# (A) Global Hareket
# Global referans (-1) kullanıldığında, hareket dünya koordinatlarına göre uygulanır.

# Mevcut global konumu oku
global_pos_before = sim.getObjectPosition(child_handle, -1)
print("Global referansta - Hareket öncesi konum:", global_pos_before)

# X ekseni üzerinde global olarak 0.5 birim sağa kaydır
new_global_pos = [global_pos_before[0] + 0.5, global_pos_before[1], global_pos_before[2]]
sim.setObjectPosition(child_handle, -1, new_global_pos)
sim.step()

print("Global referansta - Hareket sonrası konum:", sim.getObjectPosition(child_handle, -1))
time.sleep(1)

# (B) Yerel (Parent Bazlı) Hareket
# Yerel referans olarak ebeveynin frame'i kullanıldığında, hareket ebeveynin koordinat sistemi baz alınarak yapılır.
# Ebeveyn döndürülmüş olduğundan, aynı yerel X yönündeki hareket globalde farklı bir yöne denk gelecektir.

# Çocuğun ebeveyn referansındaki konumunu oku
local_pos_before = sim.getObjectPosition(child_handle, parent_handle)
print("Yerel (parent) referansta - Hareket öncesi konum:", local_pos_before)

# Yerel X ekseninde 0.5 birim sağa kaydır (ebeveynin koordinat sisteminde)
new_local_pos = [local_pos_before[0] + 0.5, local_pos_before[1], local_pos_before[2]]
sim.setObjectPosition(child_handle, parent_handle, new_local_pos)
sim.step()

print("Yerel (parent) referansta - Global konum sonrası:", sim.getObjectPosition(child_handle, -1))
time.sleep(1)

# -------------------------------------------------------
# 4. Döndürme İşlemlerinde Referans Frame Farklılıkları
# -------------------------------------------------------

# (A) Global Döndürme
# Global referans (-1) kullanılarak çocuğun oryantasyonu, dünya koordinatlarına göre değiştirilir.

global_ori_before = sim.getObjectOrientation(child_handle, -1)
print("Global referansta - Döndürme öncesi oryantasyon:", global_ori_before)

# Global Z ekseni etrafında 45° (π/4) döndürme
new_global_ori = [global_ori_before[0],
                  global_ori_before[1],
                  global_ori_before[2] + math.pi/4]
sim.setObjectOrientation(child_handle, -1, new_global_ori)
sim.step()

print("Global referansta - Döndürme sonrası oryantasyon:", sim.getObjectOrientation(child_handle, -1))
time.sleep(1)

# (B) Yerel (Parent Bazlı) Döndürme
# Ebeveyn referansı kullanılarak, çocuğun oryantasyonu ebeveynin koordinat sistemine göre değiştirilir.

local_ori_before = sim.getObjectOrientation(child_handle, parent_handle)
print("Yerel (parent) referansta - Döndürme öncesi oryantasyon:", local_ori_before)

# Yerel Z ekseni etrafında 45° (π/4) döndürme
new_local_ori = [local_ori_before[0],
                 local_ori_before[1],
                 local_ori_before[2] + math.pi/4]
sim.setObjectOrientation(child_handle, parent_handle, new_local_ori)
sim.step()

print("Yerel (parent) referansta - Global oryantasyon sonrası:", sim.getObjectOrientation(child_handle, -1))

# -------------------------------------------------------
# 5. Simülasyonu Sonlandırma
# -------------------------------------------------------
sim.stopSimulation()
