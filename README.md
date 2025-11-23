# Junior Software Training Template

Bu proje, robot yazılımı eğitiminde kullanılmak üzere hazırlanmış bir şablondur.  
Depo: https://github.com/zeynepafra07/junior-software-training-template.git

---

## 🎮 Kontrolör Tuş Atamaları

Kontrolör şeması ve hangi tuşa hangi komutun atandığı görsel olarak burada:  
[Controller Scheme](https://www.padcrafter.com/?templates=Controller+Scheme&plat=0&leftBumper=l1+shoot&leftTrigger=l2+shoot&rightBumper=l3+shoot&rightTrigger=l4+shoot&col=%23242424%2C%23606A6E%2C%23FFFFFF&yButton=score+algae+to+net&xButton=intake+algae+L2&aButton=score+algae+to+barge&rightStick=swerve+drive&backButton=slow+mode&startButton=fast+mode&dpadUp=intake&dpadDown=outtake&dpadRight=open+climb&dpadLeft=close+climb&leftStick=move+elevator+manual&leftStickClick=reset+encoder&rightStickClick=&bButton=intake+algae+L3)

### Tuş Atamaları
- Left Bumper (L1) → shoot  
- Left Trigger (L2) → shoot  
- Right Bumper (L3) → shoot  
- Right Trigger (L4) → shoot  
- Y Button → score algae to net  
- X Button → intake algae L2  
- A Button → score algae to barge  
- Right Stick → swerve drive  
- Back Button → slow mode  
- Start Button → fast mode  
- D-Pad Up → intake  
- D-Pad Down → outtake  
- D-Pad Right → open climb  
- D-Pad Left → close climb  
- Left Stick → move elevator manual  
- Left Stick Click → reset encoder  
- B Button → intake algae L3

---

## 🧰 Proje Yapısı & Kullanım

### Başlangıç
1. WPILib’in en güncel geliştirme ortamını kurun ve VS Code ile FRC eklentisinin aktif olduğundan emin olun.  
2. Projeyi klonlayın:
```bash
git clone https://github.com/zeynepafra07/junior-software-training-template.git
```
3. VS Code içerisinde projeyi açın ve Gradle yapılandırmasını çalıştırın.

### Klasör Yapısı
- `src/main/java/frc/robot/` — Robot kodunun ana dizini  
- `subsystems/` — Robot mekanizmaları için (Drive, Elevator, Intake vb.)  
- `commands/` — Komut-temelli mantık kısmı  
- `utils/` — Yardımcı sınıflar, loglama, matematiksel araçlar

### Geliştirme İlkeleri
- WPILib’in komut-temelli (command-based) mimarisine uyun.  
- Ayarlanabilir parametreler için `constants` sınıflarını kullanın; kod içerisinde sabit değerleri doğrudan yazmaktan kaçının.  
- Sık ve anlamlı commit’ler yapın. Kod incelemesi (pull requests) ile ilerleyin.  
- Her alt sistemi bağımsız olarak test edin, ardından bütünleştirin.

### Simülasyon & Test
1. WPILib simülasyon araçlarını kullanarak robot davranışlarını sanal ortamda deneyin.  
2. Motor yönleri, PID kontrolü ve tetik mappings (trigger mappings) gibi detayları doğrulayın.  
3. Fiziksel robota geçmeden önce sanal testlerden geçin.  
4. Elde edilen sonuçları ekip içinde dokümante edin.

---

## ⚙️ Kod Standartları
- Değişken ve sınıf isimleri: CamelCase kullanın.  
- Method isimleri: lowerCamelCase kullanın.  
- Alt sistem ve komut isimleri: açık ve anlamlı olsun (ör. `DriveSubsystem`, `ScoreAlgaeCommand`).  
- Magic number’lar kullanmayın: Tüm sabitleri `Constants.java` içinde tutun.  
- Yorum satırları: Kodun ne yaptığını ve neden yaptığını açıklayın, gereksiz yorum yazmayın.

---

## 📝 Sürüm Notları
- v1.0.0 – Temel robot alt sistemleri ve komut şablonları eklenmiştir.  
- v1.1.0 – Kontrolör tuş atamaları güncellendi, simülasyon destekleri eklendi.  
- v1.2.0 – Kod standartları ve örnek komutlar eklenmiştir.

---

## 📄 Lisans
Bu proje **Apache License 2.0** altında yayımlanmıştır.

---

## ℹ️ İletişim & Katkı
Her türlü fikir, öneri veya hata bildirimi için GitHub üzerindeki “Issues” sekmesini kullanabilirsiniz.  
Katkılar memnuniyetle karşılanır; katkıda bulunmadan önce bir issue açmanız önerilir.