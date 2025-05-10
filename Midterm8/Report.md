Çözülen Sorunlar ve Yapılan İyileştirmeler:

Engel Geçişi Sonrası Çizgi Kaybı:

AVOIDANCE Durumu İyileştirmesi: Engelden sağa doğru döndükten sonra, robot artık belirli bir süre düz gitmeye çalışacak ve ardından sola doğru dönerek çizgiyi arayacaktır. Eğer bu sürede çizgiyi bulamazsa, daha geniş bir açıyla sağa doğru arama yapmaya devam edecektir.
Daha Uzun Arama Süreleri: Hem "AVOIDANCE" sonrası hem de "SEARCH" durumlarındaki arama süreleri artırıldı.
"SEARCH" Durumunda İyileştirme: "SEARCH" durumu, engelden kaçındıktan sonra çizgi bulunamazsa son çare olarak tekrar devreye girecek şekilde düzenlendi.
"ALIGNMENT" Durumu İyileştirmesi: Hizalama sırasında orta sensörün çizgiyi görmemesine odaklanıldı ve buna göre dönüşler ayarlandı. Hizalama başarısız olursa takibe geri dönme mekanizması eklendi.
Robot Hızı İyileştirilmesi:

PID Parametreleri Ayarlandı: Kp, Kd ve Ki değerleri daha yüksek bir hız ve daha stabil bir çizgi takibi sağlamak amacıyla dikkatlice ayarlandı. Bu değerler deneme yanılma yoluyla farklı senaryolarda test edilerek optimize edilebilir.
Temel Hız Artırıldı: v_base değeri artırılarak robotun genel hareket hızı yükseltildi.
Tur Sürelerinin Ölçülmesi ve Yazdırılması:

Basit bir tur tamamlama kontrolü için robotun X koordinatı kullanıldı. Robot belirli bir X değerini geçtiğinde tur tamamlanmış sayılır.
lap_count, lap_start_time ve lap_times değişkenleri tanımlandı.
İlk ve ikinci turların tamamlanma süreleri hesaplanıp konsola yazdırılır.
Robot Yörüngesinin Çizilmesi ve Kaydedilmesi:

Robotun X ve Y koordinatları her adımda px ve py listelerine kaydedilir.
Simülasyon sonunda matplotlib.pyplot kullanılarak robotun yörüngesi çizilir (plt.scatter).
Yörünge grafiği robot_trajectory.png adıyla kaydedilir (plt.savefig).
Grafiğin daha iyi görünmesi için eksenler etiketlendi ve başlık eklendi. ax1.set_aspect('equal', adjustable='box') ile eksenlerin eşit ölçeklenmesi sağlandı.
Nasıl Çalışır:

Robot, çizgiyi takip ederken bir engelle karşılaştığında "AVOIDANCE" durumuna geçer. Bu durumda önce sağa döner, bir süre düz gider ve sonra sola dönerek çizgiyi tekrar bulmaya çalışır. Eğer çizgiyi bulamazsa daha geniş bir açıyla sağa doğru aramaya devam eder. Çizgi bulunduğunda "FOLLOWING" durumuna geri döner. PID kontrolcüsü, ayarlanan parametrelerle robotun çizgiyi daha hızlı ve stabil bir şekilde takip etmesini sağlar. Tur süreleri kaydedilir ve simülasyon sonunda robotun hareket ettiği yol bir grafik olarak çizilip kaydedilir.

Önemli Notlar:

PID parametreleri ve temel hız, simülasyon ortamınıza ve robotun dinamiklerine göre farklılık gösterebilir. Bu değerleri kendi ortamınızda deneme yanılma yoluyla tekrar ayarlamanız gerekebilir.