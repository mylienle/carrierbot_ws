# Xử lý file RTAB-Map `.db` thành map 2D + 3D

Thư mục này nằm ngoài `src/` nên colcon không build, robot không dùng.
Script chỉ đọc file `.db`, không sửa file gốc.

Cần: `python3`, `numpy`, `pillow`, `matplotlib`, `scipy`.

```bash
python3 -c "import numpy, PIL, matplotlib, scipy; print('ok')"
pip3 install --user pillow matplotlib scipy   # nếu thiếu
```

Nên chạy trên laptop, không chạy trên Jetson lúc robot đang quét.
Chạy theo thứ tự (bước 1 tạo thư mục `out/`):

```bash
cd ~/Documents/carrierbot_ws/scripts/rtabmap_db

# 1. Map 2D cho Nav2 (từ LiDAR): out/b1.pgm, out/b1.yaml, out/b1_preview.png
python3 rtabmap_db_to_grid.py ~/carrierbot_ws/maps/b1_rtabmap.db out b1

# 2. Đám mây điểm 3D có màu (từ camera): out/b1_cloud.ply
#    Tuỳ chọn thêm: max_depth pixel_step voxel (mặc định 3 4 0.03)
python3 rtabmap_db_to_cloud.py ~/carrierbot_ws/maps/b1_rtabmap.db out/b1_cloud.ply

# 3. Lọc nhiễu + ảnh xem nhanh: out/b1_clean_top.png, out/b1_clean_3d.png
python3 clean_ply_cloud.py out/b1_cloud.ply out b1_clean
```

Ghi chú:
- File `.db` bị tắt ngang (không có pose đã tối ưu) thì script tự tối ưu lại đồ thị,
  in dòng `... nodes, ... links, cost ... -> ..., max shift ... m`. Bước này cần `scipy`.
- Mở file `.ply` bằng CloudCompare hoặc MeshLab.
