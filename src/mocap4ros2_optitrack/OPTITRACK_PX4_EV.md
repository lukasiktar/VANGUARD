# OptiTrack → PX4 EKF2 (external vision): postavke i dijagnostika

Sažetak dijagnostike iz studenog 2026. (Pixhawk 6X, PX4 v1.17, let u zatvorenom bez GPS-a i magnetometra).
Cijeli izvještaj, skripte i snimke nalaze se na Jetsonu u `~/ev_diag_2026-11-11/` (`EV_DIAGNOSTIKA.md`).

## Lanac

```
Motive 3.5.1.1 (192.168.0.149, WiFi)
  └─ NatNet 120 Hz ─► mocap4r2_optitrack_driver (Jetson 192.168.0.10, kontejner vanguard_ros2)
                        └─ /rigid_bodies ─► vanguard/mocap_to_px4_node
                                              └─ /fmu/in/vehicle_visual_odometry (~37 Hz)
                                                   └─ MicroXRCEAgent udp4 :8888 ─► PX4 10.41.10.2 (Ethernet)
```

Pokretanje drivera i lifecycle aktivacija su opisani u glavnom [README.md](../../README.md).

## Početni problem

S uključenim OptiTrackom i OpticalFlow dron je bio iznimno nestabilan. Upravljanje je bilo gotovo nemoguće, te je dolazilo do padova. Npr. prilikom Landanja dron bi nebi spuštao vertikalno prema dolje nego bi također imao i neku horizontalnu kretnju.

## Pronađeni uzroci i rješenja

| # | Uzrok | Rješenje | Gdje |
|---|---|---|---|
| 1 | Motive je slao **Y-up**, a bridge pretpostavlja **Z-up** (`NED = (x, −y, −z)`, `q = [w, x, −y, −z]`). Visina je ulazila kao horizontalna os. | Motive → Edit → Settings → Streaming → **Up Axis = Z Up** | Motive |
| 2 | U Motive je potrebno još provjeriti Local Interface: i Transmission Type: te VRPN koji mora biti Enable | Za naš slučaj Motive → Edit → Settings → **Local Interface: 192.168.0.149** (najmeštena statička adresa računala na kojem se vrti Motive) →  **Transmission Type: Unicast**| Motive |
| 3 | **Streming ID** je također stvar na koju treba obratiti pažnju| Za naš slučaj Motive → Assets → Odabrati željeni objekt → **Streaming ID = 28** | Motive |
| 4 | Rigid body "28" bio je zakrenut za **88,5° po yawu** u odnosu na tijelo drona (FRD). | Motive → **Builder → Modify → Orientation → Reset** dron poravnati sa pozitivnim smjerom osi x ground plane-a koji se definira kod kalibracije Optitrack-a.  | Motive |
| 5 | `EKF2_EV_DELAY` je bio 30 ms, a izmjereno kašnjenje je ~8–10 ms. | Smanjen prema izmjerenom (~10 ms) | PX4 parametar |
| 6 | Visina: EKF je u zraku ~14 cm viši od EV-a jer se uz EV fuziraju i barometar, daljinomjer i optical flow (iako je `EKF2_HGT_REF = Vision`). | Za let samo s OptiTrackom: `EKF2_BARO_CTRL = 0`, `EKF2_RNG_CTRL = 0`, `EKF2_OF_CTRL = 0`, zatim restart PX4. | PX4 parametri (**u tijeku**) |

**Upozorenje za točku 6:** s isključenim ostalim senzorima(nppr. OpticalFlow) OptiTrack je jedini izvor položaja i visine.
Ako se izgubi stream, EKF gubi procjenu i PX4 ulazi u failsafe. Na daljinskom uvijek imaj prekidač za **Stabilized**.

## Što je provjereno i nije problem

- **Timesync** PX4 ↔ agent: RTT 0,3–1,9 ms. `/fmu/out` timestampovi su već u satu Jetsona.
- **Kašnjenje EV-a prema IMU-u** (kroskorelacija kutne brzine iz mocapa i žiroskopa): 4–10 ms.
- **Poravnanje osi nakon reseta** (Kabsch, mocap kutne brzine prema žiroskopu): yaw 0,0°, pitch 1,7°, roll 0,5°, korelacija po osima ≥ 0,997.



## Rezultat (let 3, nakon točaka 1–3)

| | Let 2 (prije) | Let 3 (poslije) |
|---|---|---|
| EKF kasni za EV (x / y) | 470 / 670 ms | **30 / 30 ms** |
| std greške položaja (x / y) | ~0,3 m | **2,1 / 1,8 cm** |
| heading − EV yaw | 0,8° ± 0,9° | 0,0° ± 0,2° |
| resetovi u zraku | da | 0 |
| greška visine (EKF − EV) | – | −15 cm (vidi točku 4) |

Kriterij (kašnjenje < 50 ms, std < 5 cm) je horizontalno **zadovoljen**.
Ostaje potvrdni let bez kabela nakon promjene parametara iz točke 4.

## Predloženo, nije primijenjeno

Diff za `src/vanguard/vanguard/mocap_to_px4_node.py` nalazi se u `~/ev_diag_2026-11-11/mocap_to_px4_node.diff`:
- `timestamp_sample` iz `header.stamp` (sada je 0, pa PX4 uzima vrijeme primitka),
- decimacija po broju frameova (120 → 60 Hz, umjesto vremenskog limitera koji daje ~37 Hz),
- parametar `motive_up_axis` (podržava i Y-up).

Uz te izmjene `EKF2_EV_DELAY` bi bio ~5 ms. Rezultati leta 3 pokazuju da diff nije nužan, nego je samo moguće poboljšanje.

## Ponavljanje provjere

Snimanje leta (u kontejneru; mcap plugin nije instaliran pa se koristi sqlite3):

```bash
ros2 bag record -s sqlite3 -o bags/ev_flightN /rigid_bodies /fmu/in/vehicle_visual_odometry \
  /fmu/out/vehicle_odometry /fmu/out/vehicle_local_position_v1 /fmu/out/vehicle_attitude \
  /fmu/out/sensor_combined /fmu/out/timesync_status /fmu/out/estimator_status_flags \
  /fmu/out/vehicle_status_v1 /fmu/out/vehicle_control_mode /fmu/out/vehicle_land_detected \
  /fmu/out/manual_control_setpoint
```

Ne snimaj `/fmu/out/failsafe_flags`, jer uzrokuje RTPS greške zbog veličine poruke.

Analiza (skripte su u `~/ev_diag_2026-11-11/`):

```bash
# u kontejneru
python3 analyze_ev.py extract bags/ev_flightN ev_flightN.npz
# na hostu (scipy, matplotlib)
python3 analyze_flight.py ev_flightN.npz flightN      # → flightN_results.json, flightN.png
```

Prije leta provjeri da se EKF položaj i heading (`/fmu/out/vehicle_local_position_v1`) poklapaju s
`/fmu/in/vehicle_visual_odometry` i da su u `estimator_status_flags` postavljeni `cs_ev_pos`, `cs_ev_yaw` i `cs_ev_hgt`.
