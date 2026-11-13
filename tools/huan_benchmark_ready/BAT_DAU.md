# Thử đối chứng người cắt ngang

Bộ thử `huan-fablab-benchmark-9-standing-close-pass` dùng V8 Reliable làm nền và giữ tối ưu xử lý dữ liệu LiDAR. Xe tiếp tục chờ khi dự đoán của Huân báo xung đột. Sau khi hết xung đột, bộ điều phối bắt đầu xác nhận lối trống lại từ đầu.

Đóng terminal khởi động Gazebo cũ. Trong Terminal 1 của Ubuntu, mở phiên mới.

```bash
bash /mnt/carrierbot-share/huan_benchmark_ready/start_huan_fusion.sh \
  --yield-clearance 0.6 --early-rejoin --final-approach-ramp
```

Sau dòng `SAN SANG THU LIDAR + STEREO`, giữ Terminal 1 mở và chạy trong Terminal 2.

```bash
bash /home/tranduyhoan/carrierbot-huan-benchmark/run_huan_fusion_trial.sh \
  --mode huan --velocity-profile cpu-sparse --vision-threads 1 --reliable-rgbd
```

Bộ thử gửi đích 7 giây mô phỏng sau khi quan sát thấy người ở vị trí xuất phát. Nếu lỡ mốc quá 0,15 giây mô phỏng, lượt thử dừng ở bước chuẩn bị. Người dự kiến qua giữa lối 21,8 giây mô phỏng sau lúc gửi đích. Sai lệch tối đa được chấp nhận ở mốc qua giữa lối là 0,5 giây mô phỏng. Chương trình ghi lại mốc quan sát thực tế để đối chiếu.

Gửi `summary.json` và `trace.jsonl` khi thấy `KET QUA`. Đọc `crossing_timing.valid` và `navigation_checks_passed` riêng. Lưu cả lượt bị ngắt hoặc lệch mốc. Sau khi xác nhận ca này, dùng một phiên mới với cùng lệnh và đổi `--mode huan` thành `--mode lidar-only` để chạy đối chứng.

Bản demo V8 Reliable nằm ở `/mnt/carrierbot-share/huan_fusion_ready`. Bản đối chứng cài ở `/home/tranduyhoan/carrierbot-huan-benchmark`.

## Người đi ngược chiều trong phòng rộng

Dùng cùng xe, bản đồ và bộ điều khiển. Người xuất phát tại `(4.8, 0)`, đi về phía xe tới `(2.8, 0)` với tốc độ `0.25 m/s`, rồi sang bên tới `(2.8, 2.2)`. Đích của xe vẫn là `(4.5, 0)`.

Khởi động phiên mới trong Terminal 1.

```bash
bash /mnt/carrierbot-share/huan_benchmark_ready/start_huan_fusion.sh \
  --scenario head-on-wide --yield-clearance 0.6 --early-rejoin --final-approach-ramp
```

Khi thấy `SAN SANG` và `CA THU: head-on-wide`, giữ Terminal 1 mở. Trong Terminal 2 chạy lệnh sau.

```bash
bash /home/tranduyhoan/carrierbot-huan-benchmark/run_huan_fusion_trial.sh \
  --scenario head-on-wide --mode huan --velocity-profile cpu-sparse --vision-threads 1 --reliable-rgbd
```

Gửi `summary.json` và `performance_review.json` của lượt thử, kể cả khi chưa đạt. Báo cáo ghi mốc người rời lối, nguyên nhân chờ sau mốc đó, lúc xuất hiện đường vòng và độ lệch ngang. Giữ `trace.jsonl` để kiểm tra từng lệnh khi cần.

Để đo bản thử phục hồi sau mất nhịp, khởi động lại cùng ca rồi thêm `--resume-profile fresh-scans` vào lệnh Terminal 2. Xe vẫn chờ khi thiếu dữ liệu. Sau mất nhịp ngắn, xe có thể đi tiếp khi tuyến vẫn cùng phiên bản và có hai scan LiDAR mới, độc lập, khớp TF theo thời điểm đo và không chặn lối. Xung đột cảm biến, đổi tuyến hoặc mất dữ liệu kéo dài vẫn cần xác nhận lối trống 0,5 giây mô phỏng. Đối chiếu `fast_sensor_gap_releases`, thời gian `CONFIRMING_CLEAR_CORRIDOR`, khoảng hở nhỏ nhất và kết quả tới đích với lượt chuẩn.

Để đo riêng ảnh hưởng của việc giảm tốc sớm, dùng phiên mới và thêm `--nearby-slow` vào lệnh Terminal 2 với chế độ phục hồi mặc định. Tùy chọn này giới hạn khoảng hở của vùng `SLOW` ở 2,0 m theo mô hình tròn. Các điều kiện dừng do LiDAR, dự đoán xung đột và `safety_guard` vẫn được áp dụng. So sánh `head_on.first_slow`, `head_on.slow_decision_duration_sim_s`, khoảng hở nhỏ nhất và thời gian tới đích với lượt chuẩn.

Để đo tốc độ trên đoạn thẳng thông thoáng, thêm `--clear-cruise` vào lệnh Terminal 1. Lần đầu bộ thử sẽ biên dịch lại Fablab trong một workspace riêng. Giữ `--nearby-slow --resume-profile standard` ở Terminal 2 để đối chiếu với lượt chỉ thu hẹp vùng giảm tốc. Fablab giữ tốc độ danh định tối đa 0,12 m/s khi xe bám thẳng, có scan mới, không thấy vật cản và bộ điều phối cho phép đi. Xe giữ cách giảm tốc hiện có trước waypoint tiếp theo hoặc đích. Đọc `clear_cruise_active_sim_s` và `head_on.clear_cruise_after_actor_clearance_sim_s` để xác nhận lúc tùy chọn thực sự hoạt động.

## Người đi ngược chiều trong hành lang hẹp

Sau lượt lặp trong phòng rộng, mở phiên mới và đổi `--scenario head-on-wide` thành `--scenario head-on-narrow` ở cả hai terminal. Hành lang rộng lọt lòng 1,8 m, có cửa bên hông rộng 1,2 m tại x = 2,8 m. Xe dùng cùng đích và người rời lối theo cùng lịch. Trong `performance_review.json`, mục `head_on` ghi thời gian lệnh bánh xe bằng 0 do vật cản trước khi người rời lối và thời gian đến lệnh tiến đầu tiên sau đó. Đọc các mốc này riêng với tiêu chí tới đích.

Ca hành lang hẹp đặt khoảng hở tối thiểu tới tường khi Fablab lập đường vòng là 0,32 m từ bề mặt xe. Hành lang LiDAR dùng biên ngang 0,30 m. Khoảng hở lập đường vòng giúp loại những waypoint có thể đưa vách vào hành lang LiDAR. Để cập nhật, khởi động phiên mới từ thư mục chia sẻ và kiểm tra dòng `KHOANG HO TUONG TOI THIEU KHI LAP DUONG VONG: 0.32 m`.

## Người bước ra muộn từ sau vách

Chọn `--scenario occluded-crossing` ở cả hai terminal. Giữ `--yield-clearance 0.6 --early-rejoin --final-approach-ramp --clear-cruise` ở Terminal 1 và `--nearby-slow --resume-profile standard` ở Terminal 2. Người đứng sau vách trước khi xe nhận đích, sau đó cắt ngang tại x = 3,45 m. Hai gói camera mới chưa có track và trạng thái đồng bộ của mô hình người là điều kiện khởi động ca thử.

Gửi `summary.json` và `performance_review.json`. Mục `occlusion` ghi mẫu stereo có vị trí hợp lệ đầu tiên, lệnh giảm tốc hoặc dừng đầu tiên có phản ứng ở đầu ra bánh, và lệnh tiến đầu tiên sau khi người rời lối. Mốc mép vách lấy từ tọa độ mô phỏng của người. Mốc stereo lấy từ dữ liệu quan sát của bộ thử.

Sau lượt thử bị kẹt ở đường vòng, thêm `--yield-before-detour` vào lệnh Terminal 1. Khi lệnh `SLOW` đang có hiệu lực, Fablab hoãn chèn waypoint né mới. Xe bám tuyến với giới hạn giảm tốc và kiểm tra phanh hiện có. Khi trở về `BASELINE`, Fablab có thể cân nhắc đường vòng sau khi xác nhận vật cản lại. Trong báo cáo, đối chiếu `detour_deferred_by_yield_sim_s`, `occlusion.first_detour_sim_s`, khoảng hở nhỏ nhất và kết quả tới đích.

## Người bước vào rồi đứng chắn

Mở phiên mới với `--scenario standing-blocker` ở cả hai terminal. Giữ `--yield-before-detour` ở Terminal 1. Thêm `--standing-detour` vào Terminal 2 để thử lập đường vòng khi xe đang chờ. Người bước từ bên hông vào giữa tuyến tại x = 3,2 m rồi đứng đó trong phòng rộng. Sau 3 giây mô phỏng với các vị trí stereo–LiDAR khớp và ổn định, xe có thể yêu cầu Fablab kiểm tra đường vòng trong lúc vẫn phát lệnh dừng. Nếu có đường vòng, bản tích hợp kiểm tra dữ liệu mới trên tuyến mới trước khi cho xe chạy chậm, tối đa 0,06 m/s trên đoạn vòng. Lượt đối chứng giữ nguyên lệnh Terminal 2 của ca này và bỏ tùy chọn `--standing-detour`.

Ca thử đạt khi xe thực sự đi vòng, vượt qua người lúc họ vẫn ở giữa lối và tới đích với khoảng hở đạt tiêu chí. Trong `performance_review.json`, xem `first_standing_detour_request_sim_s`, `first_standing_detour_seen_sim_s` và `bypassed_standing_person_sim_s` để phân biệt nhận diện người đứng chắn, tìm được đường vòng và đi qua thành công.

Tuyến vòng dành chỗ cho vùng dự báo quanh người và dùng giới hạn 0,06 m/s khi dự báo chuyển động của xe. Đích của ca này ở x = 5,4 m để xe có khoảng nhập lại tuyến sau người đứng chắn. So sánh thời gian giữa những lượt dùng cùng đích.

Với `--standing-detour`, khi stereo và LiDAR cùng thấy một người gần đoạn đường phía trước, xe giảm tốc. Nếu người đang tiến vào hoặc đã chắn đoạn đó, xe giữ chỗ cho cú rẽ trước khi xác nhận đứng yên. Dùng `first_standing_approach_hold_sim_s` và `first_standing_detour_request_center_distance_m` để kiểm tra xe có giữ được khoảng trống khi yêu cầu đường vòng hay không.

Fablab bám các waypoint đã kiểm tra của đường vòng quanh người đứng chắn. Cơ chế nhập tuyến sớm và phục hồi đường vòng thông thường dùng khoảng hở khác, nên hai cơ chế đó không thay tuyến đã chọn cho ca này. Xe vẫn dừng khi cảm biến báo đường vòng bị chặn.

Xe giữ giới hạn 0,06 m/s khi đang vòng qua người. Trên đoạn nhập tuyến đã lập, xe có thể trở về tốc độ bình thường nếu đã vượt qua người, đoạn còn lại đủ khoảng hở và bộ quyết định đang cho chạy bình thường. Mốc `first_standing_rejoin_cruise_sim_s` ghi thời điểm lệnh qua guard vượt 0,06 m/s trên đoạn này.

Để so với lượt đứng chắn đã đạt, giữ `--resume-profile standard`. So sánh thời gian tới đích theo đồng hồ mô phỏng, khoảng hở nhỏ nhất, thời gian `CONFIRMING_CLEAR_CORRIDOR` và độ lệch ngang. `max_abs_lateral_map_m` là độ lệch của xe so với trục y = 0. `planned_max_abs_lateral_map_m` là độ lệch của các waypoint trên đường vòng.

Để thử đường vòng gọn hơn trong phòng rộng, thêm `--compact-standing-detour` ở Terminal 1, giữ `--standing-detour` ở Terminal 2. Bộ lập đường dùng khoảng hở tối thiểu 1,00 m từ mép xe tới mép vật cản theo mô hình của nó. Khoảng hở đo trên đường xe thực sự bám có thể khác. So sánh `minimum_clearance_m` và `standing_blocker.max_abs_lateral_map_m` với lượt trước. Khởi động lại từ thư mục chia sẻ để VMware cập nhật và biên dịch lại Fablab.

Để thử khoảng cách từ tâm xe tới tâm người, dùng `--standing-center-gap 1.65` ở Terminal 1. Mức 1,65 m thêm 0,15 m khi lập đường so với lượt 1,50 m, trong đó xe dừng trước khi vượt qua người vì dự đoán xung đột. Tùy chọn này dành cho `--scenario standing-blocker` trong phòng rộng và dùng riêng với tùy chọn tính theo mép. Terminal 2 giữ `--standing-detour`. Dùng `--standing-center-gap 1.50` để lặp lại cấu hình đối chứng. Đọc `planned_standing_center_gap_m` và `minimum_center_distance_m` trong `summary.json` để đối chiếu khoảng cách lập đường với khoảng cách thực đo theo tọa độ Gazebo. Xem thêm `arrival`, thời gian chờ xung đột và độ lệch ngang.

Khi Fablab đang dừng chờ trên đoạn vòng có giới hạn 0,06 m/s, bộ điều phối gửi cùng mức 0,06 m/s nếu đã đủ điều kiện cho chạy. Đọc `standing_blocker.zero_command_release_confirmation_sim_s` trong `performance_review.json` để đo thời gian xe còn giữ lệnh bằng 0 ở bước xác nhận chạy lại. Lặp cùng mốc 1,65 m để so với lượt trước.

Thêm `--standing-post-pass-cruise` ở Terminal 1 để thử 0,08 m/s trên đoạn nhập lại tuyến với mốc 1,65 m. Xe phải vượt người, đủ khoảng hở trên đoạn còn lại, có LiDAR và vị trí mới, và nhận lệnh cho đi với tốc độ tương ứng. Đọc `post_pass_cruise_max_guard_command_mps` trong `summary.json` và `standing_blocker.bypass_to_arrival_sim_s` trong `performance_review.json`. Giữ `--sim-rate 1.0` để so với lượt 1,65 m đã đạt ở cùng cấu hình đồng hồ.

Thử đường vòng sát hơn với `--standing-center-gap 1.60 --sim-rate 1.0` ở Terminal 1 và `--standing-detour` ở Terminal 2. Dùng lượt 1,65 m chạy theo giới hạn 0,06 m/s trên đoạn né làm đối chứng. Mục `standing_blocker.outline_clearance` trong `performance_review.json` ước lượng khoảng hở từ mô hình bánh xe và thân xe URDF đến mô hình người, có xét góc xe lúc quay. Dữ liệu gồm vị trí Gazebo và hướng xe từ TF. Mốc 0,30 m dùng để đối chiếu kết quả hình học. `minimum_interpolated_outline_lower_bound_m` là ước lượng với giả thiết nội suy giữa hai mẫu liên tiếp có góc hợp lệ.

### Đường vòng 1,00 m trong phòng rộng

Mở phiên mới bằng hai lệnh sau. Giữ Terminal 1 mở và đợi hiện `SAN SANG` trước khi chạy Terminal 2.

```bash
bash /mnt/carrierbot-share/huan_benchmark_ready/start_huan_fusion.sh \
  --scenario standing-blocker --yield-clearance 0.6 \
  --early-rejoin --final-approach-ramp --clear-cruise \
  --yield-before-detour --standing-center-gap 1.00 \
  --sim-rate 0.35 --sim-lidar-rate 20
```

```bash
bash ~/carrierbot-huan-benchmark/run_huan_fusion_trial.sh \
  --scenario standing-blocker --mode huan --velocity-profile cpu-sparse \
  --vision-threads 1 --reliable-rgbd --nearby-slow \
  --resume-profile standard --standing-detour --reduced-standing-laser-reserve
```

Mốc 1,00 m là khoảng cách giữa hai tâm khi lập đường. Ca đạt khi xe vượt người đang đứng và tới đích, đồng thời khoảng hở nhỏ nhất tại các mẫu hợp lệ từ mép xe đến mô hình người đạt ít nhất 0,30 m. Đọc `standing_close_pass` trong `summary.json` và `standing_blocker.outline_clearance` trong `performance_review.json`.

Trên đường vòng 1,00 m, xe yêu cầu dừng khi thiếu dữ liệu LiDAR mới gắn với cùng người hoặc khi một scan báo dấu hiệu chuyển động. Ba scan mới liên tiếp xác nhận chuyển động. Khi vị trí đo thay đổi hoặc đường vòng thiếu khoảng hở, xe kiểm tra lại sau ít nhất 0,40 giây mô phỏng có scan mới của cùng người ở vị trí ổn định. Đường còn đủ khoảng hở có thể được xác nhận lại. Nếu thiếu khoảng hở, Fablab dùng scan mới để kiểm tra vị trí người và lập lại đoạn vòng trong lúc xe chờ. `standing_close_pass` ghi số lần xác nhận lại, yêu cầu lập lại và số lần tuyến thực sự đổi. Kịch bản Gazebo dùng người đứng sau khi bước vào lối. Việc người bước tiếp khi xe đi ngang cần một lượt thử riêng.

Sau khi mất dấu ngắn trên đoạn vòng đã được phép chạy ở mức 0,06 m/s, xe vẫn gửi `WAIT` trong lúc thiếu dữ liệu. Nếu lấy lại được hai scan mới có TF đúng thời điểm, cùng người và cùng phiên bản đường vòng, xe có thể gửi lại lệnh chạy chậm khi cả hai lần kiểm tra đều đạt khoảng hở và dự báo thông. Scan đầu tiên phải xuất hiện trong 0,18 giây mô phỏng kể từ lúc mất dấu. Đọc `standing_close_pass.resumes_after_short_track_gap` để biết số lệnh chạy chậm gửi theo cách này. Mục `lidar_block_samples` trong `performance_review.json` ghi scan chặn đầu và cuối, điểm gần nhất, tối đa 24 điểm chặn và dải kiểm tra trong hệ tọa độ `odom`.

Khi Fablab đang bám đường vòng 1,00 m ở mức 0,06 m/s, bộ điều phối có thể kiểm tra người đứng yên theo cùng khoảng cách hình học mà Fablab dùng để lập vòng. Phép kiểm tra dùng bao hình chiếu xe 0,36 m, bán kính người ít nhất 0,30 m, khoảng hở mép 0,30 m và dự phòng 0,04 m, cộng phần bù chuyển động theo tuổi scan. Nó xét đoạn còn lại và các góc mà bộ bám đường có thể cắt. Mỗi lượt xét cần người đã đứng ổn định ít nhất 0,40 giây, cùng dấu theo dõi, scan và TF đúng thời điểm, cùng đoạn đường và odometry mới. Khi LiDAR báo chắn, tất cả tia chặn phải thuộc đúng cụm đã xác nhận và có cùng dấu thời gian scan. Mọi lệnh chạy được giới hạn ở 0,06 m/s và đi qua `safety_guard`.

Chuỗi xác nhận người đứng ổn định dùng cụm LiDAR đã ghép với cùng ID khi Huân báo nguồn camera hoặc LiDAR. Khi thiếu dữ liệu ngắn, xe gửi `WAIT`. Chuỗi scan có thể được giữ trong tối đa 0,15 giây mô phỏng trên cùng phiên bản tuyến, rồi chỉ dùng lại sau khi nhận scan mới xác nhận cụm. Khi đã đủ bằng chứng đứng yên 0,40 giây, hai scan mới liên tiếp phải cùng đạt phép kiểm tra hình học và các điều kiện dừng trước khi phát lệnh `SLOW`. Xem `standing_history_preserved_samples` và `standing_route_resumes_on_two_scans` trong `standing_close_pass`.

Trong `summary.json`, xem `standing_close_pass.checked_standing_route_samples`, `matched_cluster_cleared_raw_stop_samples` và `stationary_geometry_cleared_fusion_conflict_samples`. Tùy chọn `--reduced-standing-laser-reserve` đặt mức dự phòng 0,50 m cộng phần bù tuổi scan cho các dự báo LiDAR ngoài phép kiểm tra hình học này. Với bán kính mô hình xe 0,52 m và người 0,30 m, mức đó tương đương ít nhất 1,32 m giữa hai tâm.

Terminal 2 kiểm tra vận tốc bánh xe ngay trước khi khởi động Vision và báo `DA NHAN VAN TOC BANH XE TU /odom.` khi nhận mẫu hợp lệ đầu tiên. Nếu thiếu dữ liệu sau 6 giây, lượt thử dừng ở `wheel_odometry_preflight` trước khi gửi đích. `standing_close_pass.fresh_wheel_odometry_samples` ghi số lần có vận tốc bánh xe mới trong giai đoạn vòng. Nếu phép kiểm tra khoảng hở ít được áp dụng, xem `standing_close_pass.standing_route_check_status_counts` để phân biệt thiếu odometry, thiếu cụm cùng scan và đường vòng chưa đủ khoảng hở.

Để thử dự báo lại ở tốc độ thấp, thêm `--crawl-on-predicted-conflict` ở Terminal 2. Khi xe đang vòng, LiDAR có dữ liệu mới về vật cản gần đứng yên và odometry xác nhận xe đã chậm, bộ Huân có thể kiểm tra lại cùng đoạn đường với tốc độ dự báo 0,07 m/s. Nếu dự báo đạt khoảng cách yêu cầu và quá trình xác nhận lối trống hoàn tất, lệnh chạy bị giới hạn ở 0,06 m/s. Xem `crawl_forecast` trong `summary.json` để biết số mẫu kiểm tra và số mẫu thực sự phát lệnh chạy chậm.

Khi thử nhịp chậm `--sim-rate 0.35`, có thể thêm `--sim-lidar-rate 20` ở Terminal 1. Tùy chọn này đặt tần số LiDAR trong bản URDF của phiên Gazebo để giảm khoảng cách giữa các scan có thể ghép với TF đúng thời điểm. `sim_lidar_rate_hz` ghi cấu hình trong báo cáo. Đối chiếu thời gian `CLOSE_PASS_TRACK_UNAVAILABLE`, trạng thái bám đường và lỗi trôi trong lúc chờ với lượt 10 Hz.
