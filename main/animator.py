import datetime
from typing import List, Optional

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import pandas as pd
from tqdm import tqdm


class FormationAnimator:
    """軌跡CSVを元にUAVフォーメーションの推移をアニメーション化するクラス"""

    def __init__(
        self,
        trajectory_filename: str,
        total_uav_num: int,
        tail_length: int = 50,
        interval_ms: int = 50,
        figsize=(10, 8),
    ):
        self.trajectory_filename = trajectory_filename
        self.total_uav_num = total_uav_num
        self.tail_length = tail_length
        self.interval_ms = interval_ms
        self.figsize = figsize

        self.data = self._load_trajectory()
        self.times = self.data["time"].to_numpy()
        self.x_data, self.y_data = self._extract_positions()

        self.fig, self.ax = plt.subplots(figsize=self.figsize)
        self.traces: List[plt.Line2D] = []
        self.markers: List[plt.Line2D] = []
        self._init_plot()

    def _select_writer(self, requested_ext: str) -> tuple[str, str]:
        """Return (writer_name, ext) based on availability."""
        has_ffmpeg = animation.writers.is_available("ffmpeg")
        has_imagemagick = animation.writers.is_available("imagemagick")

        if requested_ext == "mp4" and has_ffmpeg:
            return "ffmpeg", "mp4"

        if requested_ext == "mp4" and not has_ffmpeg:
            print("ffmpeg が見つからないため gif で保存します。")

        if has_imagemagick:
            return "imagemagick", "gif"

        return "pillow", "gif"

    def _load_trajectory(self) -> pd.DataFrame:
        path = f"../data/csv/trajectories/{self.trajectory_filename}"
        return pd.read_csv(path)

    def _extract_positions(self) -> tuple[List[np.ndarray], List[np.ndarray]]:
        xs: List[np.ndarray] = []
        ys: List[np.ndarray] = []
        for i in range(1, self.total_uav_num + 1):
            xs.append(self.data[f"uav{i}_true_pos_x"].to_numpy())
            ys.append(self.data[f"uav{i}_true_pos_y"].to_numpy())
        return xs, ys

    def _init_plot(self):
        self.fig.clf()
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("UAV Formation Animation")
        self.ax.set_xlabel("X position (m)")
        self.ax.set_ylabel("Y position (m)")
        self.ax.grid(True)
        self.ax.set_aspect("equal", adjustable="box")

        colors = plt.rcParams["axes.prop_cycle"].by_key().get("color", [])
        all_x = np.concatenate(self.x_data)
        all_y = np.concatenate(self.y_data)
        margin = 5.0
        xmin, xmax = all_x.min() - margin, all_x.max() + margin
        ymin, ymax = all_y.min() - margin, all_y.max() + margin
        self.ax.set_xlim(xmin, xmax)
        self.ax.set_ylim(ymin, ymax)

        self.traces = []
        self.markers = []
        for idx in range(self.total_uav_num):
            color = colors[idx % len(colors)] if colors else None
            trace_line, = self.ax.plot([], [], color=color, lw=2, label=f"UAV {idx+1}")
            marker, = self.ax.plot([], [], marker="o", color=color, markersize=8)
            self.traces.append(trace_line)
            self.markers.append(marker)

        self.ax.legend()

    def _frame_slice(self, frame: int) -> slice:
        if self.tail_length is None or self.tail_length <= 0:
            return slice(0, frame + 1)
        return slice(max(0, frame - self.tail_length), frame + 1)

    def _update(self, frame: int):
        # フレーム範囲を安全にクリップ
        frame = min(frame, len(self.times) - 1)
        frame_slice = self._frame_slice(frame)
        for idx in range(self.total_uav_num):
            x_vals = self.x_data[idx][frame_slice]
            y_vals = self.y_data[idx][frame_slice]
            self.traces[idx].set_data(x_vals, y_vals)
            # set_data はシーケンスを要求するので単一点も配列化
            self.markers[idx].set_data([self.x_data[idx][frame]], [self.y_data[idx][frame]])
        return [*self.traces, *self.markers]

    def animate(
        self,
        save: bool = False,
        save_filename: Optional[str] = None,
        show: bool = True,
        frame_step: int = 1,
        dpi: int = 120,
        speed_multiplier: float = 1.0,
    ):
        total_frames = len(self.times)
        if total_frames == 0:
            print("time 列が空のためアニメーションを生成できません。CSV を確認してください。")
            return None

        frame_step = max(1, int(frame_step))
        frame_indices = list(range(0, total_frames, frame_step))
        if frame_indices[-1] != total_frames - 1:
            frame_indices.append(total_frames - 1)

        # 進捗バーを作成
        progress_bar = tqdm(total=len(frame_indices), desc="Generating frames", unit="frame", leave=False)

        def update_with_progress(frame):
            result = self._update(frame)
            progress_bar.update(1)
            return result

        anim = animation.FuncAnimation(
            self.fig,
            update_with_progress,
            frames=frame_indices,
            interval=self.interval_ms,
            blit=True,
            repeat=False,
        )

        output_path = None
        if save:
            timestamp_str = datetime.datetime.now().strftime(r"%Y-%m-%d-%H-%M-%S")
            if save_filename is None:
                default_ext = "mp4" if animation.writers.is_available("ffmpeg") else "gif"
                save_filename = f"uav_formation_animation_{timestamp_str}.{default_ext}"
            else:
                # ユーザー指定のファイル名にもタイムスタンプを追加
                name_parts = save_filename.rsplit(".", 1)
                if len(name_parts) == 2:
                    save_filename = f"{name_parts[0]}_{timestamp_str}.{name_parts[1]}"
                else:
                    save_filename = f"{save_filename}_{timestamp_str}"

            requested_ext = save_filename.lower().split(".")[-1]
            writer_name, resolved_ext = self._select_writer(requested_ext)
            if resolved_ext != requested_ext:
                save_filename = save_filename.rsplit(".", 1)[0] + f".{resolved_ext}"

            # speed_multiplier を適用して FPS を調整（倍速再生）
            base_fps = max(1, int(1000 / (self.interval_ms * frame_step)))
            fps = max(1, int(base_fps * speed_multiplier))
            if writer_name == "pillow":
                writer = animation.PillowWriter(fps=fps, metadata={"software": "matplotlib"})
            elif writer_name == "ffmpeg":
                writer = animation.FFMpegWriter(
                    fps=fps,
                    metadata={"software": "matplotlib"},
                    extra_args=['-vcodec', 'libx264', '-preset', 'ultrafast', '-crf', '25']
                )
            elif writer_name == "imagemagick":
                writer = animation.ImageMagickWriter(fps=fps, metadata={"software": "matplotlib"})
            else:
                writer = writer_name

            output_path = f"../data/movie/trajectories/{save_filename}"

            try:
                print(f"Saving animation to {output_path} using {writer_name}...")
                progress_bar.set_description("Saving video")
                # reset counter for save step
                progress_bar.n = 0

                def _save_progress(i, n):
                    # Ensure total matches the number of frames the writer reports
                    if progress_bar.total != n:
                        progress_bar.total = n
                    # Set absolute progress to current frame index + 1
                    progress_bar.n = i + 1
                    progress_bar.refresh()

                anim.save(output_path, writer=writer, dpi=dpi, progress_callback=_save_progress)
                progress_bar.close()
                print(f"Animation saved to {output_path}")
            except Exception as exc:
                progress_bar.close()
                print(f"アニメーションの保存に失敗しました ({writer_name} 使用): {exc}")
        else:
            # show のみの場合はプログレスバーを閉じる
            progress_bar.close()

        if show:
            plt.show()
        else:
            plt.close(self.fig)

        return output_path
