# Tích hợp dữ liệu Huân với Fablab và safety_guard

Bản hiện tại là `huan-fablab-integration-8-final-approach`. Tùy chọn `--early-rejoin` của lệnh khởi động bật một lần nhập tuyến trực tiếp trong ca mô phỏng. Fablab xét khi đang đi vòng và được phép đi tiếp. Đoạn nối phải qua kiểm tra scan, TF và bản đồ trong ít nhất 0,5 giây mô phỏng và tiết kiệm ít nhất 0,5 m. Sau khi thay waypoint, behavior gate xác nhận theo phiên bản tuyến mới. Xem lệnh trong `BAT_DAU.md`.

V8 có tùy chọn `--final-approach-ramp` cho đoạn cuối sau khi đã nhập tuyến. Khi xe bám đúng hướng và gate cho đi tiếp, tốc độ đề nghị giảm theo cửa sổ 0,60–0,15 m thay cho việc kéo mức sàn thấp theo tỷ lệ cả đoạn. Cấu hình thử giảm từ 0,08 về gần 0,03 m/s trước khi dừng. Báo cáo đo `last_half_metre_elapsed_sim_s` từ lần đầu xe vào phạm vi 0,5 m đến khi ghi nhận tới đích. Trace V6 ghi khoảng 18,63 giây cho phần này.

Phép thử hình học dùng chính `planRejoin()` trên 369 mẫu của trace V5. Đoạn nối bị từ chối khi người ở giữa tuyến. Sau khi người ra khỏi hành lang tham chiếu, bản đồ kèm gói và vị trí người ghi trong trace cho đoạn nối 2,86 m so với 5,86 m còn lại của đường vòng. Cần lượt Gazebo V6 để đo hiệu quả với scan trực tiếp. Vị trí thật của người trong trace chỉ dùng cho phép thử offline này.

Lượt V6 `huan_fusion_20261107_045442_y_hif6to` đã nhập tuyến ở 26,036 giây mô phỏng, tới đích sau 108,06 giây và có khoảng hở nhỏ nhất 0,769 m theo mô hình. V7 tách `score_objective=early_rejoin` khỏi `lidar_yield_stop`. Khi chấm nhập tuyến, ba mục yêu cầu có lần dừng do LiDAR được ghi trong `not_applicable_checks`. Kết quả chín mục ban đầu vẫn có trong `original_9_checks`, và điểm chấm lại lượt V6 được lưu riêng.

Lượt v4 `huan_fusion_20261107_034447_8jeois22` dừng lúc khởi động Vision vì Torch dùng 3 luồng trong khi lượt thử yêu cầu 1. Bài thử đã nhận 3 packet và chưa gửi đích. Các lượt v2/v3 dừng ở bước chuẩn bị vì thiếu vận tốc hợp lệ.

V5 áp dụng số luồng được chọn vào cấu hình CPU của Ultralytics trước lần suy luận đầu tiên. Khi detector và pose khởi tạo, `select_device()` dùng cấu hình này để đặt số luồng Torch. Lượt thử tiếp tục kiểm số luồng thực đo được từ Vision sau khi nhận ảnh. [Mã Ultralytics 8.3.40](https://github.com/ultralytics/ultralytics/blob/v8.3.40/ultralytics/utils/torch_utils.py#L208-L215)

## V4: sửa lựa chọn CPU và kiểm tra đầu vào trước

Lượt v3 `huan_fusion_20261106_085142_oaobipf6`: 27 packet, 17 vị trí ghép, 0 vận tốc hợp lệ, 14 TIME_GAP. Chẩn đoán người dùng xác nhận khoảng cách ảnh trung vị 0,5 giây mô phỏng, tối đa 1,612 giây; không có thời điểm đủ bốn vị trí liên tục trong cửa sổ 1,4 giây. Một gzserver và một gzclient đang chạy. Giữ kết quả này là chưa đạt.

Trong chương trình gốc có dòng đặt Torch dùng 2 luồng sau khi nạp model. V3 đặt lựa chọn CPU ở đầu bản sao nên dòng gốc có thể ghi đè lựa chọn 1 luồng. Do đó nhãn CPU THREADS trước đây là lựa chọn được yêu cầu, chưa phải số luồng đo thực tế. V4 áp dụng lựa chọn ngay tại dòng đặt muộn của bản sao được tạo cho mỗi lượt. File Vision gốc vẫn giữ nguyên. Nếu cấu trúc dòng đặt luồng khác bản đã đọc, chương trình dừng thay vì sửa tùy ý.

V4 ghi số luồng thực vào từng packet (`runtime_cpu`) và summary (`runtime_cpu_measured`), kiểm tra số luồng thực khớp lựa chọn trước khi thêm người. Chưa có bằng chứng 1 luồng sẽ nhanh hơn trên VMware; lần thử này đo tác dụng thực tế. Giữ cửa sổ thử 1,4 giây, tối thiểu 4 mẫu, max_gap 0,6 giây, tuổi dữ liệu, toàn bộ Fablab và safety_guard.

Lượt tiếp theo chỉ kiểm tra khởi tạo đầu vào. Nhấn Ctrl+C ở terminal khởi động cũ, rồi:

```bash
# Terminal 1: cập nhật V5 và mở phiên mới
bash /mnt/carrierbot-share/huan_fusion_ready/start_huan_fusion.sh --yield-clearance 0.6

# Terminal 2: sau SAN SANG; giữ xe WAIT, không gửi đích
bash /home/tranduyhoan/carrierbot-huan-tools/run_huan_fusion_trial.sh \
  --velocity-profile cpu-sparse --vision-threads 1 --input-only
```

Phải thấy `huan-fablab-integration-5-cpu-backend`, `CPU THUC TE: 1 luong Torch` và `CHE DO KIEM TRA DAU VAO`. Kết quả đạt của chế độ này chỉ là khởi tạo được đầu vào: `stage=huan_input_readiness`, `navigation_evaluated=false`, `goal_sent_count=0`. Chưa chấm chín tiêu chí điều hướng, độ chính xác vận tốc hoặc độ ổn định dài hạn. Gửi summary và input_diagnosis để quyết định bước tiếp theo.

Nếu đầu vào đạt, dùng một phiên mới cho thử chuyển động với cùng cấu hình và bỏ `--input-only`. Không chạy lại vào fixture đã đi dở. Nếu vẫn TIME_GAP, xem số luồng đo thực, thời gian xử lý, tài nguyên và nguồn ảnh; không tiếp tục kéo dài cửa sổ hoặc nới ngưỡng dữ liệu chỉ để lấy kết quả đạt.

Tại thời điểm đọc cấu hình, VM có 4 vCPU và 8 GB RAM. Snapshot Windows còn khoảng 1,2 GB trống trong 13,7 GB RAM khả dụng; đây là khả năng góp phần làm độ trễ không đều, chưa phải nguyên nhân đã được xác nhận. Có thể đóng bớt ứng dụng/tabs không dùng trước lượt đo; không thay đổi cấu hình VM đang chạy.

---

## Lịch sử chuẩn bị v3

Bản `huan-fablab-integration-3-cpu-window`, chuẩn bị ngày 06/11/2026. Mặc định dừng nhường LiDAR ở khoảng hở 0,6 m từ mô hình thân xe tới điểm đo vật cản. Đây là bản riêng cho Gazebo trên Ubuntu 20.04, ROS 2 Foxy và VMware x86_64. Bản v2 đã chạy tới kiểm tra đầu vào nhưng chưa gửi đích; bản v3 chưa được xác nhận chạy ROS/Gazebo. Các kết quả 9/9 trước đó thuộc bản `fusion-yield-crossing-3-scan-tf`.

## Sửa điều kiện đủ mẫu cho CPU: bản v3

Lượt `huan_fusion_20261106_083101_duda9gj4` của v2 có 30 packet, 23 vị trí ghép camera–LiDAR và không có vận tốc hợp lệ; `goal_sent_count=0`. Kết quả này vẫn giữ nguyên là chưa đạt. Báo cáo chỉ đọc của người dùng xác nhận 18 track `WARMUP`, 6 `TIME_GAP`; khoảng cách ảnh trung vị 0,401 giây mô phỏng. Cửa sổ 0,8 giây không chứa được bốn vị trí liên tục ở thời điểm nào; cửa sổ 1,4 giây có bảy thời điểm đủ số lượng mẫu. Đây mới là kiểm tra số lượng, chưa phải kiểm tra vận tốc đúng.

V3 thêm profile thử `--velocity-profile cpu-sparse`: gọi nguyên bộ tính vận tốc của Huân với cửa sổ lịch sử 1,4 giây. Giữ bốn mẫu tối thiểu, khoảng gián đoạn tối đa 0,6 giây, kiểm tra timestamp/frame/XYZ/outlier, EMA và ngưỡng tuổi dữ liệu của bộ hợp nhất. Không sửa file lõi Vision hay biến vận tốc chưa biết thành 0. Mặc định vẫn là `original`; profile mới phải được chọn rõ.

Đổi cửa sổ không tăng tốc nhận ảnh. Bộ lọc gốc có độ trễ: trong kiểm tra chuyển động đều 0,25 m/s với bốn ảnh cách 0,4 giây, giá trị tính được là 0,1625 m/s. Vì vậy profile này dành cho thử đầu vào trên Gazebo; chưa chứng minh độ chính xác vận tốc hoặc phản ứng khi người đổi hướng. Các đoạn `TIME_GAP` vẫn bị loại. Chưa coi việc qua bước chuẩn bị là kết quả né đạt.

Trước khi khởi động lại, có thể kiểm lại dữ liệu cũ, không điều khiển xe và không sửa điểm đánh giá:

```bash
python3 /mnt/carrierbot-share/huan_fusion_ready/replay_huan_velocity.py \
  /home/tranduyhoan/carrierbot_test_runs/huan_fusion_20261106_083101_duda9gj4
```

Chỉ tiến tới lượt thử mới nếu `original_status_flag_agreement.all_match=true` và profile thử có ít nhất hai track vận tốc hợp lệ. Nếu không trùng kết quả gốc, cần kiểm tra dữ liệu trước. Replay không đánh giá tuổi ảnh lúc điều khiển, Handoff, độ chính xác vận tốc hoặc mức độ né.

Đóng terminal khởi động cũ bằng Ctrl+C, rồi cập nhật và tạo phiên mới:

```bash
# Terminal 1: chép cả bộ v3 và khởi động phiên sạch
bash /mnt/carrierbot-share/huan_fusion_ready/start_huan_fusion.sh --yield-clearance 0.6

# Terminal 2: sau SAN SANG, chọn profile CPU thử nghiệm
bash /home/tranduyhoan/carrierbot-huan-tools/run_huan_fusion_trial.sh --velocity-profile cpu-sparse
```

Phải thấy `huan-fablab-integration-3-cpu-window` và `VELOCITY FILTER: cpu-sparse`. Mỗi lượt v3 tự lưu thêm `input_diagnosis.json` và in tổng số trạng thái vận tốc, giảm việc phải chạy lệnh chẩn đoán riêng khi thiếu mẫu. Fablab, safety_guard và giới hạn tốc độ giữ nguyên; không cần build C++ mới nếu binary hiện có vẫn khớp manifest nguồn.

## Đã ghép những gì

Luồng điều khiển của bản mới:

```text
Vision CPU của Huân + CameraInfo + scan/TF đúng thời điểm
                         ↓
         Bộ ghép camera–LiDAR và mô hình nguy cơ của Huân
                         ↓
              Quyết định WAIT / SLOW / BASELINE
                         ↓
             Fablab của chị Liên + behavior gate
                         ↓
                 safety_guard hiện tại của bạn
                         ↓
                Bộ điều khiển bánh xe mô phỏng
```

- Lấy 9 module từ gói `gazebo-six-20261106-154924.tar.gz`. Chỉ đổi đường import giữa các module để đặt vào package riêng; giữ nguyên thuật toán và ngưỡng gốc. Có đối chiếu hash sau khi đảo lại các đường import.
- Dùng `Handoff.build()` để giữ riêng nguồn đo camera, LiDAR liên kết và hình học vô danh. Không đổi vị trí chưa hợp lệ thành hợp lệ, không biến vận tốc chưa biết thành 0 và không làm mới timestamp ảnh cũ.
- Trong profile cùng máy ảo này, kiểm thêm tuổi publish monotonic từ chính Vision với ngưỡng dữ liệu 0,5 giây của Handoff. Gói đến muộn không được làm mới tuổi bằng thời điểm nhận. Không áp dụng trực tiếp phép so monotonic này cho hai máy có đồng hồ độc lập.
- Dùng `dynamic_obstacles.assess()` của Huân để thêm yêu cầu giảm tốc/chờ. Mô hình này không được phép bỏ qua điểm LiDAR chắn đường, intrinsic stop của Fablab hoặc safety_guard. Bán kính robot khi đánh giá vẫn là 0,52 m của bộ thử hiện tại.
- Bổ sung CameraInfo và kích thước ảnh vào bản sao chương trình Vision CPU được tạo riêng cho từng lượt. Mã lõi và trọng số của Huân trong thư mục Vision không bị ghi đè. Kiểm tra hash các tệp lõi trước khi chạy.
- Bộ nhận clock của Huân dùng mẫu `/clock` mới nhất. Executor ROS chạy riêng để tiếp tục nhận TF, scan, tuyến và clock khi vòng quyết định đang tính toán. Đã tách lý do dừng do camera, scan/TF, tuyến và fixture thay cho việc gọi chung là thiếu LiDAR. Hiệu quả trên VMware cần được đo lại.
- Không lấy `consumer.py` phát thẳng lệnh ra bánh, thư viện ARM, TensorRT engine, Nav2 client hoặc bản vá DiffDrive của Jetson vào luồng này.
- Mặc định giữ bộ lọc vận tốc CPU gốc. Profile Huân `responsive_06` có thể chọn riêng; cửa sổ 0,6 giây của nó không tự bảo đảm đủ bốn mẫu khi CPU chạy chậm.
- Giới hạn Torch/OpenCV/BLAS ở một luồng mặc định trong bộ thử mới để tránh nhiều thư viện cùng tranh CPU với ROS. Có thể thử `--vision-threads 2`; giữ cùng lựa chọn khi so sánh. Mức cải thiện FPS cần đo trên VMware, chưa được xác nhận trên Windows.

## Cách chạy hai terminal

Đóng phiên thử cũ bằng Ctrl+C ở terminal chạy bài thử và terminal khởi động. Dùng phiên Gazebo mới cho mỗi lượt vì fixture và đích chỉ được thêm một lần. Bộ mới kiểm tra server/nguồn lệnh trùng; không tự giết phiên khác.

Terminal 1 trong Ubuntu:

```bash
bash /mnt/carrierbot-share/huan_fusion_ready/start_huan_fusion.sh
```

Lần đầu, script chép đủ bộ vào `/home/tranduyhoan/carrierbot-huan-tools`, kiểm tra file và dependency. Nếu bộ Fablab gate đã build có manifest nguồn trùng khớp thì dùng lại binary đó; nếu cần build thì tạo workspace riêng. Pointer mới là `behavior_huan_workspace.txt`; không ghi đè `behavior_workspace.txt` của bản cũ.

Chờ `SAN SANG THU LIDAR + STEREO`. Giữ terminal 1 mở.

Terminal 2:

```bash
bash /home/tranduyhoan/carrierbot-huan-tools/run_huan_fusion_trial.sh
```

Phải thấy `TOOLSET: huan-fablab-integration-3-cpu-window`, `MODE: huan` và `VELOCITY FILTER: original` khi dùng mặc định; chọn `--velocity-profile cpu-sparse` như mục v3 khi thử cửa sổ CPU mới.

Trước khi gửi đích, chương trình cần ít nhất ba vị trí ghép đúng và hai ảnh có đầu vào vận tốc hợp lệ cho bộ hợp nhất của Huân. Không đủ điều kiện thì dừng chuẩn bị, `goal_sent_count=0`, và ghi `fusion_preflight` trong summary. Nếu người đã bắt đầu cắt ngang trong lúc chờ đầu vào, lượt này cũng dừng; không gửi lại đích vào một fixture đã chạy dở.

Những lần sau terminal 1 dùng bản local:

```bash
bash /home/tranduyhoan/carrierbot-huan-tools/start_huan_fusion.sh
```

Nếu cần mở thêm góc nhìn Gazebo:

```bash
bash /home/tranduyhoan/carrierbot-huan-tools/open_robot_view.sh --gazebo-scene
```

Nếu share chưa gắn, dùng lại mount hiện có; thư mục `huan_fusion_ready` là thư mục mới nằm cùng cấp với `lien_test_ready` trong HuanVision, không cần tạo thêm shared folder VMware.

## Vùng nhường 0,6 m

Bản v2 mặc định dùng khoảng hở 0,6 m **từ bề mặt mô hình thân xe đến điểm LiDAR**, chỉ xét các điểm thuộc hành lang phía trước. Đây không phải chiều rộng toàn bộ vùng quét; robot vẫn có bán kính mô hình 0,52 m. Việc điều chỉnh không thu hẹp phạm vi quét của cảm biến.

Sau khi cập nhật từ thư mục chia sẻ, terminal 1 phải in `VUNG NHUONG LIDAR: 0.60 m tu be mat than xe toi diem LiDAR`. Terminal 2 nhận cùng cấu hình đó, không cần gõ lại tham số.

```bash
# Đặt rõ khoảng hở 0,6 m khi khởi động
bash /home/tranduyhoan/carrierbot-huan-tools/start_huan_fusion.sh --yield-clearance 0.6

# Phiên riêng để đối chiếu với vùng chờ cũ
bash /home/tranduyhoan/carrierbot-huan-tools/start_huan_fusion.sh --legacy-yield-zone
```

Camera có thể yêu cầu giảm tốc hoặc chờ sớm hơn khi dự đoán xung đột; Fablab và safety_guard vẫn giữ các điều kiện dừng riêng. Do đó 0,6 m là ngưỡng hình học của lớp nhường LiDAR, không phải cam kết xe chỉ dừng ở đúng khoảng cách đó.

Chín tiêu chí gốc vẫn được lưu. Vùng nhỏ hơn có thể khiến người đi qua mà không cần dừng; khi đó các tiêu chí yêu cầu WAIT được báo chưa đạt/chưa được thực hiện. Không đổi chúng thành đạt và không coi đó là bằng chứng phản ứng dừng ở 0,6 m đã được kiểm chứng.

## Đọc kết quả

Mỗi lượt lưu ở `~/carrierbot_test_runs/huan_fusion_.../`:

- `summary.json`: giữ chín tiêu chí gốc; thêm điều kiện bản Huân thực sự có dữ liệu nguy cơ hợp lệ trong nhiệm vụ. Có `original_9_checks_passed` và `integration_checks` để phân biệt.
- `fusion_preflight`: điều kiện đầu vào trước đích, nằm trong summary.
- `trace.jsonl`: quyết định, nguồn camera/LiDAR, timestamp, dữ liệu Handoff và lệnh Fablab/guard. Ground truth nằm riêng ở `scoring_only`, không truyền vào policy.
- `performance_review.json`: tổng thời gian WAIT theo nguyên nhân, các lần WAIT sau x=3 m và lệnh BASELINE ghi nhận ở đoạn cuối. Đây là phân tích lệnh, không phải phép đo tốc độ quay bánh thật hay độ trễ phanh.
- `startup_session.json`: đường dẫn và hash safety_guard, hash executable Fablab, tốc độ và nhịp mô phỏng đã dùng.
- `integration_manifest.json`, `huan_source_manifest.json`: nguồn mã của lượt thử.

Nếu `fusion_status=LIDAR_FALLBACK_FUSION_UNAVAILABLE`, bộ thử đang dùng hành vi LiDAR hiện có vì dữ liệu hợp nhất chưa đủ. Không được tính đoạn đó là thành công của camera hoặc Handoff. `laser_source_updates=0` nghĩa là chưa có bằng chứng nhánh đo tiếp bằng LiDAR được dùng trong lượt đó.

## So sánh cùng điều kiện

Mỗi cấu hình phải có phiên khởi động mới và cùng tốc độ, world, goal, người cắt ngang và nhịp mô phỏng.

```bash
# Bộ quyết định cũ, cùng đầu vào và cách nhận ROS của bản tích hợp
bash /home/tranduyhoan/carrierbot-huan-tools/run_huan_fusion_trial.sh --mode legacy

# Điều phối chỉ dùng LiDAR; Vision vẫn chạy để giữ tải CPU và kiểm fixture giống nhau
bash /home/tranduyhoan/carrierbot-huan-tools/run_huan_fusion_trial.sh --mode lidar-only

# Thêm bộ hợp nhất và đánh giá nguy cơ của Huân
bash /home/tranduyhoan/carrierbot-huan-tools/run_huan_fusion_trial.sh --mode huan
```

Trong chế độ lidar-only, tiêu chí tác dụng giảm tốc của camera được ghi rõ là không áp dụng; toàn bộ kết quả chín tiêu chí gốc vẫn được giữ, không đổi một lượt cũ chưa đạt thành đạt. Đây chưa phải lượt đo Fablab nguyên bản không có lớp điều phối; bản nguyên gốc và các lượt cũ vẫn ở `/home/tranduyhoan/carrierbot-test-tools`.

## Tốc độ và các giới hạn còn lại

Giới hạn lệnh vẫn là 0,12 m/s, SLOW 0,06 m/s và nhịp mô phỏng mục tiêu 0,35. Bộ dừng nhường LiDAR mới chỉ yêu cầu WAIT khi có ít nhất ba điểm đo thuộc hành lang phía trước nằm cách mô hình thân xe không quá 0,6 m. Bán kính mô hình thân xe vẫn 0,52 m, nên ngưỡng tương đương 1,12 m từ tâm xe tới bề mặt vật cản được LiDAR đo. Giữ nguyên bề rộng kiểm tra tuyến, toàn bộ dữ liệu scan và safety_guard. Không ép camera dự đoán va chạm hoặc safety_guard chờ tới đúng 0,6 m mới dừng. Luật giảm tốc theo khoảng cách còn lại của Fablab chưa đổi; chưa xác nhận thời gian hoàn thành hay độ mượt của bản mới trên VMware.

Ưu tiên sau lượt đầu: xem phần thời gian WAIT do input, độ liên tục của Handoff và nguồn vận tốc; kiểm khoảng hở khi vùng 0,6 m kích hoạt. Sau đó mới đánh giá profile tốc độ của Fablab, giữ safety_guard, kiểm trên Gazebo và so sánh cùng điều kiện. Không tăng tốc hay nới tuổi dữ liệu chỉ để lấy kết quả đạt.

Phạm vi vẫn là một người mô phỏng đi cùng hình trụ va chạm. Chưa có kiểm chứng ZED thật, nhiều người, mọi hình dạng vật cản hay xe thật của bản tích hợp này.
