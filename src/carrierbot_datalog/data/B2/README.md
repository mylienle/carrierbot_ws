# B2 logs (19/10/2026 và 22/10/2026)

Mỗi run = 1 file `guidance_*` + 1 file `amcl_odom_log_*` (logger amcl bật trước guidance ~10 s).
Waypoint: `RECEPTION_ADMIN_GUI/.../Map/B2_config_wp.json` (wp13 [743,444], wp14 [416,457]).
Chỉ giữ các lượt về đích (`finished = 1`).

## Football_court/ (10 run, 20 lượt)
Mỗi run: Home -> wp16 -> Football court, rồi Football court -> wp15 -> Home.

054516, 054934, 055245, 061726, 062128, 062618, 062905, 063329, 063629, 064440_fc

## Home_2/ (4 run, 5 lượt)
Home -> wp15 -> wp1 -> wp14 -> wp13 -> wp12 -> wp11 -> Home 2 (và chiều ngược lại).

| Run | Lượt |
|---|---|
| 051451 | Home -> Home 2 (đã cắt lượt về bị lỗi) |
| 053357 | Home -> Home 2, Home 2 -> Home |
| 064440_home_2 | Home -> Home 2 |
| 075123_home_2 (22/10) | Home -> Home 2. Safety stop chỉ xảy ra sau khi đã đỗ ở Home 2 |

## Lab_CEPP/ (2 run, 2 lượt đi + 2 lượt về)
- 064440_lab_cepp (19/10): Home -> Meeting Room -> Lab CEPP (2 goal liên tiếp, tính là 1 lượt đi),
  rồi Lab CEPP -> Home.
- 075123_lab_cepp (22/10, v_max 0.6): Home -> Lab CEPP (1 goal, không dừng ở Meeting Room),
  rồi Lab CEPP -> Home. Không có safety stop.

## Ghi chú
- 064440 gốc được tách thành 3 file `_fc`, `_lab_cepp`, `_home_2` theo từng goal (cắt tại lúc bắt đầu route mới).
- 075123 gốc (commit fa0e7dd) được tách thành `_lab_cepp` và `_home_2`, cắt tại t = 315.3 s khi bắt đầu route Home 2.
  File amcl 075113 cắt theo cùng thời điểm (amcl trễ guidance 33 s, tính từ lúc robot bắt đầu chạy).
- Đã bỏ: 044618 (wp gốc, safety stop gần wp13), 045802 (wp đang sửa),
  055545, 055838, 060302, 060535, 060944 (bánh không quay, encoder = 0).
