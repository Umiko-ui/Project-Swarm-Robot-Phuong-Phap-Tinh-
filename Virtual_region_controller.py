import math
import numpy as np
import formation_cpp
from sklearn.cluster import KMeans



class VirtualRegionType:
    CIRCLE = "CIRCLE"
    ELLIPSE = "ELLIPSE"

class VirtualRegionController:
    def __init__(self, initial_radius, safety_margin=0.2):
        """
        Khởi tạo bộ điều khiển hình học bầy đàn
        :param initial_radius: Bán kính R ban đầu của hình tròn bao bọc bầy robot
        :param safety_margin: Khoảng cách an toàn bù trừ để không cạ vào tường
        """
        self.base_radius = initial_radius
        self.safety_margin = safety_margin
        
        self.current_shape = VirtualRegionType.CIRCLE
        self.params = {
            'R': self.base_radius, 'a': self.base_radius, 
            'b': self.base_radius, 'angle': 0.0
        }

    def _calculate_free_space(self, current_center, goal, open_sights):
        """
        K means clustering for finding out the free-space available between the two nearest obstacles
        """
        # 1. Hướng mặc định (khi không có vật cản) là hướng thẳng về đích
        dx_goal = goal[0] - current_center[0]
        dy_goal = goal[1] - current_center[1]
        passage_angle = math.atan2(dy_goal, dx_goal)
        free_width = self.base_radius * 4.0 # Mặc định đường rất rộng
        
        # 2. Nếu có chướng ngại vật (cảm biến trả về open_sights)
        if open_sights is not None and len(open_sights) > 0:
            # Lấy tọa độ các điểm thuộc vật cản (ptA và ptB) từ open_sights
            obs_pts = []
            for sight in open_sights:
                if len(sight) >= 2:
                    obs_pts.append(sight[0])
                    obs_pts.append(sight[1])
            
            obs_pts = np.array(obs_pts)
            
            # Nếu có đủ điểm, dùng K-Means chia làm 2 cụm (2 bên vách của khe hẹp)
            if len(obs_pts) >= 2:
                kmeans = KMeans(n_clusters=2, random_state=0, n_init=10).fit(obs_pts)
                r_o1, r_o2 = kmeans.cluster_centers_
                
                # Bề rộng khe hẹp chính là khoảng cách giữa 2 cụm vật cản
                free_width = np.linalg.norm(r_o1 - r_o2)
                
                # Tính góc xoay đi qua khe hẹp (Vuông góc với đoạn thẳng nối 2 tâm vật cản)
                dx_obs = r_o2[0] - r_o1[0]
                dy_obs = r_o2[1] - r_o1[1]
                
                # 2 vector vuông góc khả thi
                angle_gap1 = math.atan2(dx_obs, -dy_obs)
                angle_gap2 = math.atan2(-dx_obs, dy_obs)
                
                # Chọn hướng vuông góc đồng thuận với hướng về đích nhất
                diff1 = abs(math.atan2(math.sin(angle_gap1 - passage_angle), math.cos(angle_gap1 - passage_angle)))
                diff2 = abs(math.atan2(math.sin(angle_gap2 - passage_angle), math.cos(angle_gap2 - passage_angle)))
                
                passage_angle = angle_gap1 if diff1 < diff2 else angle_gap2
                
        return free_width, passage_angle

    def update_structural_parameters(self, current_center, goal, open_sights):
        # CIRCLE AND ELLIPSE REASONING 
        free_width, passage_angle = self._calculate_free_space(current_center, goal, open_sights)
        
        effective_width = free_width - self.safety_margin
        diameter = 2 * self.base_radius
        
        if effective_width >= diameter:
            # ---> KHE ĐỦ RỘNG: DÙNG HÌNH TRÒN
            self.current_shape = VirtualRegionType.CIRCLE
            self.params = {
                'R': self.base_radius, 'a': self.base_radius, 
                'b': self.base_radius, 'angle': 0.0
            }
        else:
            # ---> KHE HẸP: ÉP THÀNH HÌNH ELIP
            self.current_shape = VirtualRegionType.ELLIPSE
            b = max(effective_width / 2.0, 0.1) # Chống lỗi chia cho 0
            
            # Tính trục lớn (a) bảo toàn diện tích nhưng thiết lập GIỚI HẠN (Tránh Scale nổ)
            raw_a = (self.base_radius ** 2) / b
            MAX_A = self.base_radius * 4.0 # Chiều dài tối đa không vượt quá 4 lần bán kính
            a = min(raw_a, MAX_A)
            
            self.params = {
                'R': None, 'a': a, 'b': b, 'angle': passage_angle
            }
            
        return self.current_shape, self.params
    # C++ CORE 
    def get_robot_target_positions(self, current_center, num_robots):
        
        is_ellipse = (self.current_shape == VirtualRegionType.ELLIPSE)

        # Extract parameters safely
        R = self.params.get('R', 0.0) or 0.0
        a = self.params.get('a', 0.0)
        b = self.params.get('b', 0.0)
        angle = self.params.get('angle', 0.0)

        # Call C++ backend
        targets = formation_cpp.compute_targets(
            list(current_center),   # ensure std::vector<double>
            int(num_robots),
            bool(is_ellipse),
            float(R),
            float(a),
            float(b),
            float(angle)
        )

        return targets
        
        
        

