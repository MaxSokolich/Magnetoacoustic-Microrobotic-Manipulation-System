# record_thread.py
from PyQt5.QtCore import QThread
import cv2
import time
import queue

class RecordThread(QThread):
    def __init__(self, frame_queue, output_path, fps):
        super().__init__()
        self.frame_queue = frame_queue
        self.output_path = output_path
        self.fps = fps
        self.running = False
        self.writer = None

    def run(self):
        self.running = True
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        first_frame = self.frame_queue.get()  # Get the first frame to determine size
        h, w, _ = first_frame.shape
        self.writer = cv2.VideoWriter(self.output_path, fourcc, self.fps, (w, h))
        self.writer.write(first_frame)
        start_time = time.time()
        frame_num = 1

        while self.running:
            try:
                frame = self.frame_queue.get(timeout=1)
                elapsed = time.time() - start_time
                timestamp = self.format_time(elapsed)
                
                
                font = cv2.FONT_HERSHEY_SIMPLEX
                scale = 1.5
                thickness = 5
                color = (0, 0, 0)

                (text_width, text_height), _ = cv2.getTextSize(timestamp, font, scale, thickness)
                x = frame.shape[1] - text_width - 20  # 20 px from right
                y = frame.shape[0] - 20               # 20 px from bottom

                cv2.putText(frame, timestamp, (x, y), font, scale, color, thickness, cv2.LINE_AA)

              
                x = frame.shape[1] - text_width - 200  # 20 px from right
                y = frame.shape[0] - 20               # 20 px from bottom

                cv2.putText(frame, str(frame_num), (x, y), font, scale, color, thickness, cv2.LINE_AA)
                
                self.writer.write(frame)
            except queue.Empty:
                continue

            frame_num +=1

        self.writer.release()
    
    def format_time(self, seconds):
        mins = int(seconds // 60)
        secs = int(seconds % 60)
        ms = int((seconds - int(seconds)) * 100)
        return f"{mins:02d}:{secs:02d}.{ms:02d}"

    def stop(self):
        self.running = False
        self.wait()
