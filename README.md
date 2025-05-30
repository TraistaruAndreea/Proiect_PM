# Sistem de Detectare Mișcare cu Senzor PIR, LED-uri si Control Bluetooth

Acest proiect Arduino permite detectarea miscarii prin intermediul unui senzor PIR, afisarea starii pe un ecran LCD 16x2, controlul LED-urilor printr-un buton sau prin Bluetooth (de pe telefon sau alt dispozitiv), si feedback in timp real.

## Functionalitati

- **Detectare miscare**: Utilizeaza un senzor PIR pentru a detecta prezenta in zona.
- **Afisare LCD**: Starea sistemului si a LED-urilor este afisata pe un ecran LCD 16x2.
- **Control LED-uri**:
  - **LED rosu** si **LED verde** - indica starea sistemului si pot fi comutate:
    - Local, printr-un buton fizic
    - Remote, prin Bluetooth (comenzi „1” si „0”)
- **Comunicare Bluetooth**: Permite schimbarea LED-ului activ si trimite mesaje de status catre utilizator.

## Schema de conexiuni

- **Senzor PIR**: OUT → A2, VCC → 5V, GND → GND
- **LED rosu**: A3 (prin rezistor la GND)
- **LED verde**: A4 (prin rezistor la GND)
- **Buton**: A5 (cu pull-up intern)
- **LCD 16x2**:
  - RS = 8, E = 9, D4 = 10, D5 = 11, D6 = 12, D7 = 13
  - V0 (contrast) → rezistor 2.2kΩ → GND
- **Bluetooth HC-05**:
  - TX → A0 (Arduino RX)
  - RX → A1 (Arduino TX)
  - VCC → 5V, GND → GND

## Cum functioneaza

- La detectarea miscarii, LCD-ul va afisa „Miscare”, iar LED-ul selectat este aprins.
- Fara miscare, LED-urile sunt stinse si LCD-ul afiseaza „Fara miscare”.
- Butonul apasa comuta culoarea LED-ului (rosu/verde).
- Prin Bluetooth, trimitand „1” aprinde LED-ul rosu, „0” aprinde LED-ul verde (daca exista miscare).
- Mesajele de stare sunt transmise inapoi prin Bluetooth.

## Resurse hardware necesare

- Arduino UNO (ATmega328P)
- Senzor PIR
- LED rosu + rezistor
- LED verde + rezistor
- Buton push
- LCD 16x2 cu controller HD44780
- Rezistor 2.2kΩ (pentru contrast LCD)
- Modul Bluetooth HC-05
- Breadboard + fire de conexiune

**Autor:** Traistaru Andreea-Cosmina 332CD
