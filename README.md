# BLE_Mesh
Hướng dẫn setup build và nạp code ESP-IDF cho ESP32

1.	Tải và giải nén file Test_Gateway_Send_To_Server.zip (có thể mở xem tham khảo code trong /main/ hello_world_main.c)

2.	Tải ESP-IDF framework để build và flash
Link tải: https://dl.espressif.com/dl/esp-idf/?idf=4.4
B1: Tải bản ESP-IDF v4.3.6 và chạy setup
 ![image](https://github.com/hhoang308/BLE_Mesh/assets/57220076/e76774af-0594-47fb-b0fd-835abf6a78d4)


B2: Sau khi cài xong framework, mở app ESP-IDF CMD. Cửa sổ này dùng để build và nạp code.
 ![image](https://github.com/hhoang308/BLE_Mesh/assets/57220076/588cddf4-c016-4a24-a4b1-cdac0de18d76)


B3: Trong cửa sổ ESP-IDF cmd, đi đến đường dẫn folder project đã tải (Test_Gateway_Send_To_Server). Dùng lệnh cd, ví dụ như sau:
 ![image](https://github.com/hhoang308/BLE_Mesh/assets/57220076/71d1ed1a-9422-4921-a3aa-fae531fbaa29)


B4: Build code
Nếu lần đầu build code, cần chạy lệnh này trước khi build: ipf.py fullclean
 ![image](https://github.com/hhoang308/BLE_Mesh/assets/57220076/332351a7-e54b-4e34-a2fe-a8fb6b3a0954)

Để build code: chạy lệnh idf.py build
 ![image](https://github.com/hhoang308/BLE_Mesh/assets/57220076/03189fc2-64fe-42cb-82bc-43af0cb09d4b)

Nếu build thành công sẽ hiện như sau:
 ![image](https://github.com/hhoang308/BLE_Mesh/assets/57220076/36b30430-7b9d-40b8-adab-7dc4cfe26d76)


B5: Nạp code vào ESP32 và chạy
Dùng lệnh: idf.py -p (PORT) flash monitor
