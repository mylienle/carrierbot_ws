# B2 logs (19/10/2026)

Mỗi run = 1 file `guidance_*` + 1 file `amcl_odom_log_*` (logger amcl bật trước guidance ~10 s).
Waypoint: `RECEPTION_ADMIN_GUI/.../Map/B2_config_wp.json` (wp13 [743,444], wp14 [416,457]).
Chỉ giữ các lượt về đích (`finished = 1`).

## Football_court/ (10 run, 20 lượt)
Mỗi run: Home -> wp16 -> Football court, rồi Football court -> wp15 -> Home.

054516, 054934, 055245, 061726, 062128, 062618, 062905, 063329, 063629, 064440_fc

## Home_2/ (3 run, 4 lượt)
Home -> wp15 -> wp1 -> wp14 -> wp13 -> wp12 -> wp11 -> Home 2 (và chiều ngược lại).

| Run | Lượt |
|---|---|
| 051451 | Home -> Home 2 (đã cắt lượt về bị lỗi) |
| 053357 | Home -> Home 2, Home 2 -> Home |
| 064440_home_2 | Home -> Home 2 |

## Lab_CEPP/ (1 run, 1 lượt đi + 1 lượt về)
064440_lab_cepp: Home -> Meeting Room -> Lab CEPP (2 goal liên tiếp, tính là 1 lượt đi),
rồi Lab CEPP -> Home.

## Ghi chú
- 064440 gốc được tách thành 3 file `_fc`, `_lab_cepp`, `_home_2` theo từng goal (cắt tại lúc bắt đầu route mới).
- Đã bỏ: 044618 (wp gốc, safety stop gần wp13), 045802 (wp đang sửa),
  055545, 055838, 060302, 060535, 060944 (bánh không quay, encoder = 0).
