# Obstacle Avoidance — bản Hoàn–Huân

Bản này dùng ROS 2 Foxy và Gazebo Classic để thử điều phối né vật cản động. Camera stereo của Huân cung cấp track người. LiDAR xác nhận cụm vật cản. Bộ điều phối gửi trạng thái chờ hoặc chạy chậm cho Fablab. Fablab gửi lệnh vận tốc qua safety_guard hiện có trên máy thử.

- [Bắt đầu](BAT_DAU.md) có hai lệnh chạy ca người đứng chắn và các ca đối chứng.
- Mã quyết định nằm trong `huan_fusion_policy.py` và `fusion_yield_policy.py`.
- Bản Fablab dùng cho mô phỏng nằm trong `behavior_gate_source`. Script build kiểm tra nguồn Fablab tại commit `d466598` của nhánh `fablab-robot-v2-obstacle-avoidance` trước khi tạo workspace thử riêng.
- Kịch bản 1,00 m là mục tiêu khoảng cách giữa hai tâm khi lập đường. Ca chỉ đạt khi xe vượt người, tới đích và giữ khoảng hở mép ít nhất 0,30 m. Bản này đã qua kiểm tra cục bộ. Kết quả chạy Gazebo đầy đủ của bản mới đang chờ xác nhận.

Bộ thử hiện dùng môi trường đã cài trên máy Hoàn. Để chạy ở máy khác, cần các gói Foxy/Gazebo, workspace mô phỏng của Huân kèm model Vision và tệp `safety_guard.py`. Đường dẫn phụ thuộc nằm trong `start_lien_test.py`, `fusion_yield_trial.py` và `build_behavior_gate.sh`. Các lượt thử lưu `summary.json` và `performance_review.json` để đối chiếu.
