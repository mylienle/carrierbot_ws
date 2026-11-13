# Obstacle Avoidance — bản Hoàn–Huân

Bản này dùng ROS 2 Foxy và Gazebo Classic để thử điều phối né vật cản động. Camera stereo của Huân cung cấp track người. LiDAR xác nhận cụm vật cản. Bộ điều phối gửi trạng thái chờ hoặc chạy chậm cho Fablab. Fablab gửi lệnh vận tốc qua safety_guard hiện có trên máy thử.

- [Chạy ca người đứng chắn 1,65 m](BAT_DAU.md#chạy-lại-ca-165-m-đã-đạt) có hai lệnh và cấu hình của lượt Gazebo đã đạt.
- Mã quyết định nằm trong `huan_fusion_policy.py` và `fusion_yield_policy.py`.
- Bản Fablab dùng cho mô phỏng nằm trong `behavior_gate_source`. Script build kiểm tra nguồn Fablab tại commit `d466598` của nhánh `fablab-robot-v2-obstacle-avoidance` trước khi tạo workspace thử riêng.
- Cấu hình bàn giao dùng khoảng cách lập đường **1,65 m giữa hai tâm**. Trong lượt Gazebo tham chiếu, xe vượt qua người khi họ vẫn đứng chắn và tới đích. Khoảng cách tâm–tâm nhỏ nhất đo được là **1,63 m**, sai số tới đích **0,15 m** và thời gian chạy **189,8 giây mô phỏng**.
- Mã băm của 74 tệp chạy khớp với bản `huan-fablab-benchmark-7-post-pass-cruise` dùng trong lượt đạt. Kết quả áp dụng cho ca người đứng chắn trong phòng rộng trên Gazebo.

Bộ thử hiện dùng môi trường đã cài trên máy Hoàn. Để chạy ở máy khác, cần các gói Foxy/Gazebo, workspace mô phỏng của Huân kèm model Vision và tệp `safety_guard.py`. Đường dẫn phụ thuộc nằm trong `start_lien_test.py`, `fusion_yield_trial.py` và `build_behavior_gate.sh`. Các lượt thử lưu `summary.json` và `performance_review.json` để đối chiếu.
