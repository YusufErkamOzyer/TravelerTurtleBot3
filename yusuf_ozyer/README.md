Yusuf Erkam Özyer    200202094	

Node çalıştırmak için workspace in içerisinde
source /usr/share/gazebo/setup.sh //kendi cihazımda bu komut çalıştırılmadan gazebo ortamı açılmadı
source install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py //gazebo ortamının açılması
 
yeni terminal içerisinde
source install/setup.bash
ros2 run yusuf_ozyer hareket_kontrol

turtlebot3 robotta mesafeye göre hız oransal olarak düzenlenmektedir. eğer en yakın mesafe 0.4 m ise robot odomdan mevcut yaw değerinden 90 derece sağ tarafa yeni bir yaw belirler ve bu yaw değerine ulaşana kadar robotu sonsuz döngü içerisinde döndürmektedir.Döngüden bir türlü çıkılamadığı için robot kendi etrafında dönmeyi durduramamaktadır.Bundan dolayı proje tam değildir






