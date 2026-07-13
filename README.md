# Robot-ARM-Firmware

## Pemetaan Pin Teensy 4.1

Sumber pemetaan pin: `src/main_67.2.cpp`

Catatan:
- Pin 34 dan 35 sekarang khusus dipakai untuk HX711 loadcell.
- Pin 42 sampai 54 tidak ditemukan dipakai di program.
- Tabel memakai format monospaced supaya garisnya lurus saat dibaca.

## Peta Pin Utama

### Pin 0 - 13

```text
+-----+---------+--------+----------+
| Pin | Fungsi  | Mode   | Kelompok |
+-----+---------+--------+----------+
|   0 | J1 Step | OUTPUT | Motor    |
|   1 | J1 Dir  | OUTPUT | Motor    |
|   2 | J2 Step | OUTPUT | Motor    |
|   3 | J2 Dir  | OUTPUT | Motor    |
|   4 | J3 Step | OUTPUT | Motor    |
|   5 | J3 Dir  | OUTPUT | Motor    |
|   6 | J4 Step | OUTPUT | Motor    |
|   7 | J4 Dir  | OUTPUT | Motor    |
|   8 | J5 Step | OUTPUT | Motor    |
|   9 | J5 Dir  | OUTPUT | Motor    |
|  10 | J6 Step | OUTPUT | Motor    |
|  11 | J6 Dir  | OUTPUT | Motor    |
|  12 | J7 Step | OUTPUT | Motor    |
|  13 | J7 Dir  | OUTPUT | Motor    |
+-----+---------+--------+----------+
```

### Pin 14 - 25

```text
+-----+--------------+---------------+----------+
| Pin | Fungsi       | Mode          | Kelompok |
+-----+--------------+---------------+----------+
|  14 | J1 Encoder A | Encoder input | Encoder  |
|  15 | J1 Encoder B | Encoder input | Encoder  |
|  16 | J2 Encoder B | Encoder input | Encoder  |
|  17 | J2 Encoder A | Encoder input | Encoder  |
|  18 | J3 Encoder B | Encoder input | Encoder  |
|  19 | J3 Encoder A | Encoder input | Encoder  |
|  20 | J4 Encoder A | Encoder input | Encoder  |
|  21 | J4 Encoder B | Encoder input | Encoder  |
|  22 | J5 Encoder B | Encoder input | Encoder  |
|  23 | J5 Encoder A | Encoder input | Encoder  |
|  24 | J9 Step      | OUTPUT        | Motor    |
|  25 | J9 Dir       | OUTPUT        | Motor    |
+-----+--------------+---------------+----------+
```

### Pin 26 - 41

```text
+-----+-------------------------+--------------------+---------------+
| Pin | Fungsi                  | Mode/Penggunaan    | Kelompok      |
+-----+-------------------------+--------------------+---------------+
|  26 | J1 Limit/Cal            | INPUT_PULLUP       | Limit         |
|  27 | J2 Limit/Cal            | INPUT_PULLUP       | Limit         |
|  28 | J3 Limit/Cal            | INPUT_PULLUP       | Limit         |
|  29 | J4 Limit/Cal            | INPUT_PULLUP       | Limit         |
|  30 | J5 Limit/Cal            | INPUT_PULLUP       | Limit         |
|  31 | J6 Limit/Cal            | INPUT_PULLUP       | Limit         |
|  32 | J8 Step                 | OUTPUT             | Motor         |
|  33 | J8 Dir                  | OUTPUT             | Motor         |
|  34 | HX711 DOUT              | HX711 data         | Force Sensor  |
|  35 | HX711 SCK               | HX711 clock        | Force Sensor  |
|  36 | J7 Limit/Cal            | INPUT_PULLUP       | Limit         |
|  37 | J8 Limit/Cal            | INPUT_PULLUP       | Limit         |
|  38 | J9 Limit/Cal            | INPUT_PULLUP       | Limit         |
|  39 | E-Stop                  | INPUT_PULLUP + IRQ | Safety        |
|  40 | J6 Encoder A            | Encoder input      | Encoder       |
|  41 | J6 Encoder B            | Encoder input      | Encoder       |
+-----+-------------------------+--------------------+---------------+
```

### Pin 42 - 54

```text
+-----+--------+--------+
| Pin | Fungsi | Status |
+-----+--------+--------+
|  42 | -      | Kosong |
|  43 | -      | Kosong |
|  44 | -      | Kosong |
|  45 | -      | Kosong |
|  46 | -      | Kosong |
|  47 | -      | Kosong |
|  48 | -      | Kosong |
|  49 | -      | Kosong |
|  50 | -      | Kosong |
|  51 | -      | Kosong |
|  52 | -      | Kosong |
|  53 | -      | Kosong |
|  54 | -      | Kosong |
+-----+--------+--------+
```

## Tabel Motor Stepper

```text
+------+----------+---------+---------------+---------------+--------------+
| Axis | Step Pin | Dir Pin | Limit/Cal Pin | Step/Dir Mode | Limit Mode   |
+------+----------+---------+---------------+---------------+--------------+
| J1   |        0 |       1 |            26 | OUTPUT        | INPUT_PULLUP |
| J2   |        2 |       3 |            27 | OUTPUT        | INPUT_PULLUP |
| J3   |        4 |       5 |            28 | OUTPUT        | INPUT_PULLUP |
| J4   |        6 |       7 |            29 | OUTPUT        | INPUT_PULLUP |
| J5   |        8 |       9 |            30 | OUTPUT        | INPUT_PULLUP |
| J6   |       10 |      11 |            31 | OUTPUT        | INPUT_PULLUP |
| J7   |       12 |      13 |            36 | OUTPUT        | INPUT_PULLUP |
| J8   |       32 |      33 |            37 | OUTPUT        | INPUT_PULLUP |
| J9   |       24 |      25 |            38 | OUTPUT        | INPUT_PULLUP |
+------+----------+---------+---------------+---------------+--------------+
```

## Tabel Encoder

```text
+------+-----------+-----------+-----------------------------+
| Axis | Encoder A | Encoder B | Objek Program               |
+------+-----------+-----------+-----------------------------+
| J1   |        14 |        15 | Encoder J1encPos(14, 15)   |
| J2   |        17 |        16 | Encoder J2encPos(17, 16)   |
| J3   |        19 |        18 | Encoder J3encPos(19, 18)   |
| J4   |        20 |        21 | Encoder J4encPos(20, 21)   |
| J5   |        23 |        22 | Encoder J5encPos(23, 22)   |
| J6   |        40 |        41 | Encoder J6encPos(40, 41)   |
+------+-----------+-----------+-----------------------------+
```

## Sensor, Safety, Komunikasi

```text
+------------+----------------+--------------------------+-------------------------+
| Fitur      | Pin/Interface  | Penggunaan               | Catatan                 |
+------------+----------------+--------------------------+-------------------------+
| E-Stop     | 39             | INPUT_PULLUP + interrupt | Aktif saat HIGH         |
| HX711 DOUT | 34             | Data HX711               | Logic 3.3 V             |
| HX711 SCK  | 35             | Clock HX711              | Logic 3.3 V             |
| USB Serial | USB Teensy     | Serial.begin(9600)       | Command dan monitor HMI |
| SD Card    | BUILTIN_SDCARD | SD.begin(BUILTIN_SDCARD) | Slot microSD Teensy 4.1 |
+------------+----------------+--------------------------+-------------------------+
```

## Ringkasan Pin Kosong

```text
+----------+----------------------+-----------------------------+
| Area Pin | Pin Kosong           | Keterangan                  |
+----------+----------------------+-----------------------------+
| 0 - 41   | Tidak ada            | Semua sudah dipakai program |
| 42 - 46  | 42, 43, 44, 45, 46   | Tidak dipakai di program    |
| 47 - 51  | 47, 48, 49, 50, 51   | Tidak dipakai di program    |
| 52 - 54  | 52, 53, 54           | Tidak dipakai di program    |
+----------+----------------------+-----------------------------+
```

## Catatan Pin

```text
+-----+-----------------+--------------------------+-------------------------------+
| Pin | Pemakaian Aktif | Pemakaian Lain/Komentar  | Catatan                       |
+-----+-----------------+--------------------------+-------------------------------+
|  24 | J9 Step         | Komentar lama J6 encoder | Kode aktif memakai pin 24     |
|  25 | J9 Dir          | Komentar lama J6 encoder | Kode aktif memakai pin 25     |
|  34 | HX711 DOUT      | -                        | Khusus loadcell HX711         |
|  35 | HX711 SCK       | -                        | Khusus loadcell HX711         |
|  40 | J6 Encoder A    | Komentar lama J9 Step    | Kode aktif memakai encoder J6 |
|  41 | J6 Encoder B    | Komentar lama J9 Dir     | Kode aktif memakai encoder J6 |
+-----+-----------------+--------------------------+-------------------------------+
```
