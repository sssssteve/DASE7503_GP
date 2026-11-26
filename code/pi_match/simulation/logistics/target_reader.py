"""Select a requested identity from actual image detections, preserving coordinates."""
import cv2
from qr_reader import QrReader,read_labels,Detection

class TargetReader(QrReader):
    def __init__(self):
        super().__init__();self.expected='START'
        self.post_decoder=cv2.QRCodeDetector()

    def read(self,image):
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        fallback=[]
        for stage in ('gray','sharpen','contrast','enlarge'):
            if stage=='gray':
                candidate=gray
            elif stage=='sharpen':
                blurred=cv2.GaussianBlur(gray,(0,0),2.5)
                candidate=cv2.addWeighted(gray,3,blurred,-2,0)
            elif stage=='contrast':
                candidate=self.contrast.apply(gray)
            else:
                candidate=cv2.resize(gray,None,fx=2,fy=2,interpolation=cv2.INTER_LINEAR)
            labels,_=read_labels(candidate)
            if stage=='enlarge':labels=[Detection(label.text,tuple((x/2,y/2) for x,y in label.corners)) for label in labels]
            matching=[label for label in labels if label.text==self.expected]
            if len(matching)==1:return matching[0],'已识别当前目标'
            if len(matching)>1:return None,'画面内当前目标编号不唯一'
            if labels:fallback=labels
        # START and END each have one physical post. Keep the exact text check.
        if self.expected in ('START','END'):
            decoded,corners,_=self.post_decoder.detectAndDecode(gray)
            if decoded==self.expected and corners is not None:
                points=tuple(map(tuple,corners.reshape(4,2).tolist()))
                return Detection(decoded,points),'已识别当前立柱'
        if fallback:return None,'暂未看到指定二维码，继续等待'
        return None,'没有可用二维码'
