import cv2, queue, threading, time
#DON'T USE THIS
# bufferless VideoCapture https://stackoverflow.com/a/54755738
class VideoCapture:

  def __init__(self, name):
    self.cap = cv2.VideoCapture(name, cv2.CAP_FFMPEG)
    self.cap.set(cv2.CAP_PROP_BUFFERSIZE, -1)
    self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('H','2','6','4'))
    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    self.cap.set(cv2.CAP_PROP_FPS, 30)
    self.q = queue.Queue()
    t = threading.Thread(target=self._reader)
    t.daemon = True
    t.start()

  # read frames as soon as they are available, keeping only most recent one
  def _reader(self):
    while True:
      ret, frame = self.cap.read()
      if not ret:
        break
      if not self.q.empty():
        try:
          self.q.get_nowait()   # discard previous (unprocessed) frame
        except queue.Empty:
          pass
      self.q.put(frame)

  def read(self):
    return (True,self.q.get())