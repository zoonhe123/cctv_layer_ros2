import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist

import matplotlib.pyplot as plt
plt.rcParams.update({'font.size': 16})
import matplotlib.cm as cm
import matplotlib.colors as mcolors
import numpy as np
import threading
import time
import collections
import queue
import math # math 모듈 추가

# --- 설정 변수 ---
PLOT_DURATION_SECONDS = 10
TARGET_MODELS = ['burger', 'walking_human_1', 'walking_human_2', 'walking_human_3', 'walking_human_4']
DATA_COLLECTION_INTERVAL = 0.05

RADIUS_BURGER = 0.2
RADIUS_WALKING_HUMAN = 0.25

# --- 모델 그룹별 컬러맵 및 마커 설정 (더 강한 대비로 변경) ---
# BURGER 그룹
BURGER_COLORMAP_NAME = 'jet' # 또는 'YlGnBu', 'viridis' 등 푸른 계열
BURGER_MARKER = 'o' # 궤적 점 마커

# WALKING HUMAN 그룹
HUMAN_COLORMAP_NAME = 'jet' # 또는 'Oranges_r', 'hot' 등 붉은 계열
HUMAN_MARKER = 's' # 궤적 점 마커 (사각형)
# ----------------------------------------------------

class PositionPlotter(Node):
    def __init__(self):
        super().__init__('position_plotter')
        self.get_logger().info('PositionPlotter node has been started.')

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/model_states',
            self.model_states_callback,
            10
        )

        self.is_plotting_active = False
        self.data_buffers = {name: collections.deque() for name in TARGET_MODELS}
        self.start_time = None
        self.last_recorded_time = {name: 0.0 for name in TARGET_MODELS}

        self.plot_thread = None
        self.stop_plot_event = threading.Event()
    
        self.plot_data_queue = queue.Queue()
        self.plot_timer = self.create_timer(1.0, self.check_and_plot)

    def cmd_vel_callback(self, msg: Twist):
        if not self.is_plotting_active:
            self.trigger_plotting_start()

    def trigger_plotting_start(self):
        if not self.is_plotting_active:
            self.get_logger().info('First /cmd_vel received. Starting position tracking for plotting.')
            self.is_plotting_active = True
            self.start_time = self.get_clock().now().nanoseconds / 1_000_000_000.0
            
            for name in TARGET_MODELS:
                self.data_buffers[name].clear()
                self.last_recorded_time[name] = self.start_time

            if self.plot_thread and self.plot_thread.is_alive():
                self.stop_plot_event.set()
                self.plot_thread.join()
                self.stop_plot_event.clear()

            self.plot_thread = threading.Thread(target=self.run_data_collection_logic)
            self.plot_thread.start()

    def model_states_callback(self, msg: ModelStates):
        if not self.is_plotting_active:
            return

        current_ros_time = self.get_clock().now().nanoseconds / 1_000_000_000.0

        if self.start_time and (current_ros_time - self.start_time > PLOT_DURATION_SECONDS):
            return

        for i, model_name in enumerate(msg.name):
            if model_name in TARGET_MODELS:
                if (current_ros_time - self.last_recorded_time[model_name]) >= DATA_COLLECTION_INTERVAL:
                    position = msg.pose[i].position
                    self.data_buffers[model_name].append((current_ros_time - self.start_time, position.x, position.y))
                    self.last_recorded_time[model_name] = current_ros_time

    def run_data_collection_logic(self):
        self.get_logger().info('Data collection thread started. Collecting data...')
        
        while not self.stop_plot_event.is_set():
            current_ros_time = self.get_clock().now().nanoseconds / 1_000_000_000.0
            if self.start_time and (current_ros_time - self.start_time >= PLOT_DURATION_SECONDS):
                self.get_logger().info(f'Data collection for {PLOT_DURATION_SECONDS} seconds completed.')
                break
            time.sleep(DATA_COLLECTION_INTERVAL / 4)
            
        self.get_logger().info('Data collection complete. Passing data to main thread for plotting...')
        
        data_to_plot = {name: list(buffer) for name, buffer in self.data_buffers.items()}
        self.plot_data_queue.put(data_to_plot)

    def check_and_plot(self):
        try:
            data_for_plot = self.plot_data_queue.get_nowait()
            self.get_logger().info('Received plot data from queue. Generating plot in main thread.')
            self.plot_trajectories(data_for_plot)
            self.plot_data_queue.task_done()

            self.get_logger().info('Plot window closed. Resetting for next movement.')
            self.is_plotting_active = False
            for name in TARGET_MODELS:
                self.data_buffers[name].clear()
            self.start_time = None

        except queue.Empty:
            pass

    def plot_trajectories(self, data_for_plot):
        if all(len(buffer) == 0 for buffer in data_for_plot.values()):
            self.get_logger().warn('No data collected for plotting for any target model.')
            return

        # Figure와 서브플롯 생성
        # 1행 3열 그리드 (메인 플롯, Burger 컬러바, Human 컬러바)
        # 이제 새로운 거리 플롯을 위해 2행으로 변경
        fig = plt.figure(figsize=(10, 10)) # 전체 figure 크기 증가
        gs = fig.add_gridspec(2, 3, width_ratios=[1, 0.05, 0.05], height_ratios=[0.7, 0.3]) # 2행으로 조정
        
        ax_main = fig.add_subplot(gs[0, 0]) # 메인 궤적 플롯 (첫째 행 첫째 열)
        ax_cbar_burger = fig.add_subplot(gs[0, 1]) # Burger 컬러바 (첫째 행 둘째 열)
        # ax_cbar_human = fig.add_subplot(gs[0, 2])  # Human 컬러바 (첫째 행 셋째 열)
        ax_dist = fig.add_subplot(gs[1, :])        # 거리 플롯 (둘째 행 전체)


        # 모든 모델의 시간 데이터를 모아서 전체 시간 범위 계산
        all_times_collected = []
        for model_name in TARGET_MODELS:
            data = data_for_plot[model_name]
            if data:
                all_times_collected.extend([d[0] for d in data])
        
        if not all_times_collected:
            self.get_logger().warn('No time data available for plot color bar (all buffers empty).')
            plt.close(fig)
            return

        all_times_np = np.array(all_times_collected)
        min_t, max_t = all_times_np.min(), all_times_np.max()
        
        # --- 각 모델 그룹별 컬러맵 및 ScalarMappable 생성 ---
        norm_burger = mcolors.Normalize(vmin=min_t, vmax=max_t)
        sm_burger = cm.ScalarMappable(cmap=BURGER_COLORMAP_NAME, norm=norm_burger)
        sm_burger.set_array([]) 
        
        norm_human = mcolors.Normalize(vmin=min_t, vmax=max_t)
        # sm_human = cm.ScalarMappable(cmap=HUMAN_COLORMAP_NAME, norm=norm_human)
        # sm_human.set_array([])

        # --- 다중 컬러바 그리기 ---
        cbar_burger = fig.colorbar(sm_burger, cax=ax_cbar_burger, label=f'time(s)') # <--- shrink=0.7 추가
        if max_t != min_t:
            cbar_burger.set_ticks(np.linspace(min_t, max_t, 5))
            cbar_burger.set_ticklabels([f'{t:.1f}' for t in np.linspace(min_t, max_t, 5)])
        else:
            cbar_burger.set_ticks([min_t])
            cbar_burger.set_ticklabels([f'{min_t:.1f}'])

        # cbar_human = fig.colorbar(sm_human, cax=ax_cbar_human, label=f'time(s)')
        # if max_t != min_t:
        #     cbar_human.set_ticks(np.linspace(min_t, max_t, 5))
        #     cbar_human.set_ticklabels([f'{t:.1f}' for t in np.linspace(min_t, max_t, 5)])
        # else:
        #     cbar_human.set_ticks([min_t])
        #     cbar_human.set_ticklabels([f'{min_t:.1f}'])


        # 각 모델별 궤적 플로팅
        for model_name in TARGET_MODELS:
            data = data_for_plot[model_name]
            if not data:
                self.get_logger().info(f'No data for {model_name}. Skipping.')
                continue
            
            times = np.array([d[0] for d in data])
            x_coords = np.array([d[1] for d in data])
            y_coords = np.array([d[2] for d in data])

            # --- 모델별 컬러맵 및 마커 선택 ---
            if model_name == 'burger':
                current_cmap_name = BURGER_COLORMAP_NAME
                current_marker = BURGER_MARKER
                radius = RADIUS_BURGER
                label_suffix = " (Robot)" 
                current_norm = norm_burger
            elif model_name.startswith('walking_human'):
                current_cmap_name = HUMAN_COLORMAP_NAME
                current_marker = HUMAN_MARKER
                radius = RADIUS_WALKING_HUMAN
                label_suffix = " (Human)" 
                current_norm = norm_human
            else: 
                current_cmap_name = 'gray'
                current_marker = 'x'
                radius = 0.1
                label_suffix = ""
                current_norm = mcolors.Normalize(vmin=min_t, vmax=max_t)


            if len(times) > 1:
                normalized_times_for_scatter = current_norm(times)
                
                ax_main.scatter( # ax -> ax_main으로 변경
                    x_coords, y_coords,
                    c=normalized_times_for_scatter, 
                    cmap=current_cmap_name,         
                    s=20, 
                    alpha=0.7,
                    label=f'{model_name} Trajectory{label_suffix}', 
                    marker=current_marker           
                )

                # --- 원형 표시 ---
                for time_value, x_val, y_val in zip(times, x_coords, y_coords):
                    normalized_time_for_color = current_norm(time_value) 
                    circle_color = plt.get_cmap(current_cmap_name)(normalized_time_for_color) 
                    
                    circle = plt.Circle((x_val, y_val), radius, 
                                        color=circle_color, 
                                        fill=True,
                                        linewidth=0.5,
                                        alpha=0.2)
                    ax_main.add_patch(circle) # ax -> ax_main으로 변경

            else: 
                if times.size > 0:
                    single_time_value = times[0]
                    normalized_time_for_color = current_norm(single_time_value)
                    circle_color = plt.get_cmap(current_cmap_name)(normalized_time_for_color)
                    
                    ax_main.plot( # ax -> ax_main으로 변경
                        x_coords, y_coords, 
                        marker=current_marker, 
                        markersize=8, 
                        color=circle_color, 
                        label=f'{model_name} Start Position{label_suffix}'
                    ) 
                    
                    circle = plt.Circle((x_coords[0], y_coords[0]), radius, 
                                        color=circle_color, 
                                        fill=True,
                                        linewidth=0.5,
                                        alpha=0.2)
                    ax_main.add_patch(circle) # ax -> ax_main으로 변경

        ax_main.set_title(f'trajectory')
        ax_main.set_xlabel('x (m)')
        ax_main.set_ylabel('y (m)')
        ax_main.set_aspect('equal', adjustable='box')
        ax_main.grid(True)
        # ax_main.legend() # 범례가 많아지면 충돌할 수 있으므로 주석 처리하거나 위치 조정 필요


        # --- 새로 추가된 부분: 거리 플롯 ---
        if 'burger' in data_for_plot:
            burger_data = data_for_plot['burger']
            burger_times = np.array([d[0] for d in burger_data])
            burger_x = np.array([d[1] for d in burger_data])
            burger_y = np.array([d[2] for d in burger_data])

            for human_model_name in TARGET_MODELS:
                if human_model_name.startswith('walking_human'):
                    human_data = data_for_plot[human_model_name]
                    if not human_data:
                        self.get_logger().info(f'No data for {human_model_name}. Skipping distance plot.')
                        continue
                    
                    human_times = np.array([d[0] for d in human_data])
                    human_x = np.array([d[1] for d in human_data])
                    human_y = np.array([d[2] for d in human_data])

                    # 데이터를 시간으로 정렬하여 정확한 거리 계산을 위해 보간 또는 가장 가까운 시간 선택
                    # 여기서는 간단하게 두 모델의 시간이 동일하다고 가정하고 zip으로 묶음
                    # 실제 시나리오에서는 두 시퀀스의 길이를 맞추거나 보간해야 할 수 있음
                    
                    # 가장 짧은 시간 시퀀스 길이에 맞춰 데이터를 잘라냅니다.
                    min_len = min(len(burger_times), len(human_times))
                    if min_len == 0: continue

                    distances_over_time = []
                    # 현재는 두 모델의 타임스탬프가 거의 일치한다고 가정하고 직접 계산
                    for i in range(min_len):
                        dist = math.sqrt(
                            (burger_x[i] - human_x[i])**2 +
                            (burger_y[i] - human_y[i])**2
                        )
                        distances_over_time.append(dist)
                    
                    # 'walking_human_1' 이런 식으로 넘버링이 되어 있으므로, 레이블에 사용
                    human_id = human_model_name.split('_')[-1] 
                    ax_dist.plot(burger_times[:min_len], distances_over_time, 
                                 label=f'distance(m)', linewidth=2.0)

            # 충돌 임계값 선
            collision_threshold = RADIUS_BURGER + RADIUS_WALKING_HUMAN
            ax_dist.axhline(collision_threshold, color='red', linestyle='--', label='collision threshold')

            # # 충돌 구간 음영 처리 (가장 짧은 거리에 대해)
            # if 'burger' in data_for_plot and any(name.startswith('walking_human') for name in TARGET_MODELS):
            #     # 모든 인간 모델에 대한 거리를 구한 후, 가장 가까웠던 거리 라인을 기준으로 충돌 구간 음영 표시
            #     # 또는 각 휴먼별로 음영 표시 가능 (현재는 모든 인간에 대해 라인만 그림)
            #     # 여기서는 편의상 첫 번째 인간에 대해서만 음영 처리 예시를 들겠습니다.
            #     # 실제로는 모든 인간에 대한 최소 거리를 계산해야 합니다.
                
            #     # 예시로 첫 번째 인간 모델 (walking_human_1)을 기준으로 음영 처리
            #     # TODO: 여러 인간에 대한 충돌을 더 정교하게 표시하려면,
            #     # 각 인간-로봇 쌍별로 fill_between을 호출하거나,
            #     # 모든 인간과의 최소 거리를 추적하는 새로운 리스트를 만들어야 함
                
            #     first_human_name = None
            #     for name in TARGET_MODELS:
            #         if name.startswith('walking_human'):
            #             first_human_name = name
            #             break
                
            #     if first_human_name and data_for_plot[first_human_name]:
            #         first_human_data = data_for_plot[first_human_name]
            #         min_len = min(len(burger_times), len(first_human_data))
            #         if min_len > 0:
            #             first_human_x = np.array([d[1] for d in first_human_data])
            #             first_human_y = np.array([d[2] for d in first_human_data])
                        
            #             distances_to_first_human = []
            #             for i in range(min_len):
            #                 dist = math.sqrt(
            #                     (burger_x[i] - first_human_x[i])**2 +
            #                     (burger_y[i] - first_human_y[i])**2
            #                 )
            #                 distances_to_first_human.append(dist)
                        
            #             distances_to_first_human_np = np.array(distances_to_first_human)
                        
            #             ax_dist.fill_between(burger_times[:min_len], distances_to_first_human_np, collision_threshold, 
            #                                  where=(distances_to_first_human_np < collision_threshold), 
            #                                  color='red', alpha=0.2, label='Collision Zone (Burger to Human_1)')


            # ax_dist.set_title('Distance between Burger and Humans Over Time')
            ax_dist.set_xlabel('time(s)')
            ax_dist.set_ylabel('disetance(m)')
            ax_dist.set_ylim(bottom=0)
            ax_dist.grid(True)
            ax_dist.legend()


        plt.tight_layout() 
        plt.show() 

def main(args=None):
    rclpy.init(args=args)
    position_plotter = PositionPlotter()
    try:
        rclpy.spin(position_plotter)
    except KeyboardInterrupt:
        position_plotter.get_logger().info('PositionPlotter node stopped by user.')
    finally:
        if position_plotter.plot_thread and position_plotter.plot_thread.is_alive():
            position_plotter.stop_plot_event.set()
            position_plotter.plot_thread.join()
        position_plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()