"""Latest-JPEG transport over a local tmpfs file, with atomic publishing.

The ROS worker writes one compressed frame atomically. The HTTP process reads
only newer frames; slow browsers never create a queue of stale images.
"""
import os
import time
from pathlib import Path

class CameraFeed:
    def __init__(self,filename,max_bytes=4000000,max_age=2.0):
        self.path=Path(filename)
        self.max_bytes=max_bytes
        self.max_age=max_age
        self._cache=None
        self._mtime=0
    def read(self):
        try:
            st=self.path.stat()
            if st.st_size<4 or st.st_size>self.max_bytes or time.time()-st.st_mtime>self.max_age:
                return None
            if st.st_mtime_ns!=self._mtime:
                data=self.path.read_bytes()
                if not (data.startswith(b'\xff\xd8') and data.endswith(b'\xff\xd9')):return None
                self._cache=data
                self._mtime=st.st_mtime_ns
            return self._cache
        except (FileNotFoundError,PermissionError,OSError):
            return None

def write_frame(target,data):
    # One writer per adapter subprocess; file never exposed partially.
    path=Path(target)
    tmp=path.with_suffix('.writing')
    with open(tmp,'wb') as f:f.write(data)
    os.replace(str(tmp),str(path))
