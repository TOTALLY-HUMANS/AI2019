import av
import queue
import threading
#Install pyav using conda install av -c conda-forge

class AvVideoCapture:
    '''Reads and decodes a udp multicast stream'''

    def __init__(self, url):
        self.context = av.Codec('h264', 'r').create()
        self.q = queue.Queue()
        self.cap = av.open(url)
        self.target_stream = self.cap.streams.video[0]
        t = threading.Thread(target=self._reader)
        t.daemon = True
        t.start()

    def _reader(self):
        '''Thread that reads encoded packets as they arrive and keeps the latest packet in queue.'''
        for packet in self.cap.demux(self.target_stream):
            if packet is not None:
                if not self.q.empty():
                    try:
                        self.q.get_nowait()
                    except queue.Empty:
                        pass
                self.q.put(packet)
    


    def read(self):
        '''Gets the latest packet from the queue and returns it decoded.'''
        packet = self.q.get()
        latest = ''
        for frame in packet.decode():
            latest = frame              
        return (True, latest.to_ndarray(format='bgr24'))
